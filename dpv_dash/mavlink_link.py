"""MAVLink connection to the Pixhawk (ArduPilot Rover/Boat).

Runs in a background thread. The HUD reads `snapshot()`; the controller calls
`set_mode()`, `change_speed()`, `send_throttle()`, which are queued and sent
from the link thread so all serial I/O happens in one place.
"""
from __future__ import annotations

import copy
import glob
import logging
import os
import queue
import threading
import time

os.environ.setdefault("MAVLINK20", "1")  # must be set before importing mavutil
from pymavlink import mavutil  # noqa: E402

from .state import VehicleState  # noqa: E402

log = logging.getLogger(__name__)
mav = mavutil.mavlink

# Messages we ask the autopilot to stream, and how often (Hz).
STREAM_RATES_HZ = {
    "VFR_HUD": 10,
    "ATTITUDE": 10,
    "GLOBAL_POSITION_INT": 5,
    "NAV_CONTROLLER_OUTPUT": 4,
    "SYS_STATUS": 2,
    "GPS_RAW_INT": 2,
    "MISSION_CURRENT": 1,
    "HOME_POSITION": 0.2,
}
RE_REQUEST_EVERY_S = 10.0  # re-send stream requests (cheap; survives a Pixhawk reboot)
MISSION_REFRESH_S = 15.0   # re-download the mission (picks up edits from the land station)
MISSION_ITEM_TIMEOUT_S = 1.0


class MavlinkLink:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self._state = VehicleState()
        self._lock = threading.Lock()
        self._cmds: queue.Queue = queue.Queue()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="mavlink", daemon=True)
        self._conn = None
        self._target_sys: int | None = None
        self._target_comp = 1
        self._mav_type = mav.MAV_TYPE_GROUND_ROVER
        self._last_hb = 0.0
        self._params: dict[str, float] = {}
        self._dl: dict | None = None      # mission download in progress
        self._last_mission_req = 0.0

    # ---------------------------------------------------------------- public API
    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2)

    def snapshot(self) -> VehicleState:
        with self._lock:
            s = copy.copy(self._state)
        s.link_ok = (time.monotonic() - self._last_hb) < self.cfg.get("link_timeout_s", 2.0)
        return s

    def set_mode(self, name: str) -> None:
        self._cmds.put(("mode", name))

    def change_speed(self, mps: float) -> None:
        self._cmds.put(("speed", mps))

    def send_throttle(self, pwm: int, channel: int) -> None:
        self._cmds.put(("rc", (pwm, channel)))

    def arm(self, arm: bool) -> None:
        self._cmds.put(("arm", arm))

    # ---------------------------------------------------------------- thread
    def _run(self) -> None:
        last_err = None
        while not self._stop.is_set():
            try:
                self._connect()
                last_err = None
                self._loop()
            except Exception as e:  # serial unplugged, port missing, etc.
                if str(e) != last_err:
                    log.warning("MAVLink link error: %s (retrying)", e)
                    last_err = str(e)
            finally:
                if self._conn is not None:
                    try:
                        self._conn.close()
                    except Exception:
                        pass
                    self._conn = None
            self._stop.wait(2.0)

    @staticmethod
    def _resolve(conn) -> str:
        """Pick the first existing device from a path, glob, or list of them."""
        candidates = [conn] if isinstance(conn, str) else list(conn)
        for c in candidates:
            if ":" in c and not c.startswith("/"):
                return c  # udp:/tcp: URL
            matches = sorted(glob.glob(c))
            if matches:
                return matches[0]
        raise FileNotFoundError(f"no Pixhawk found at {', '.join(candidates)}")

    def _connect(self) -> None:
        c = self.cfg
        self._conn = mavutil.mavlink_connection(
            self._resolve(c["connection"]),
            baud=c.get("baud", 115200),
            source_system=c.get("source_system", 255),
            source_component=c.get("source_component", 191),
            autoreconnect=False,
        )
        log.info("Opened %s", self._conn.address)

    def _loop(self) -> None:
        last_req = 0.0
        last_own_hb = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now - last_own_hb >= 1.0:
                self._conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_own_hb = now
            if self._target_sys is not None:
                if now - last_req >= RE_REQUEST_EVERY_S:
                    self._request_streams()
                    last_req = now
                self._mission_tick(now)
            self._drain_commands()
            msg = self._conn.recv_match(blocking=True, timeout=0.05)
            if msg is not None:
                self._handle_mission(msg)
                if self._handle(msg):  # first heartbeat from a new autopilot
                    last_req = 0.0
                    self._last_mission_req = 0.0

    # ---------------------------------------------------------------- mission download
    def _mission_tick(self, now: float) -> None:
        dl = self._dl
        if dl is not None:
            if now - dl["t"] > MISSION_ITEM_TIMEOUT_S:
                dl["retries"] += 1
                if dl["retries"] > 5:
                    log.warning("Mission download stalled at item %d; will retry later", dl["next"])
                    self._dl = None
                else:
                    self._request_item(dl["next"])
            return
        if now - self._last_mission_req >= MISSION_REFRESH_S:
            self._last_mission_req = now
            self._conn.mav.mission_request_list_send(self._target_sys, self._target_comp,
                                                     mav.MAV_MISSION_TYPE_MISSION)

    def _request_item(self, seq: int) -> None:
        self._dl["t"] = time.monotonic()
        self._conn.mav.mission_request_int_send(self._target_sys, self._target_comp, seq,
                                                mav.MAV_MISSION_TYPE_MISSION)

    def _finish_mission(self, items: dict) -> None:
        self._conn.mav.mission_ack_send(self._target_sys, self._target_comp,
                                        mav.MAV_MISSION_ACCEPTED, mav.MAV_MISSION_TYPE_MISSION)
        with self._lock:
            if items != self._state.mission:
                log.info("Mission updated: %d waypoints", len(items))
            self._state.mission = items
        self._dl = None

    def _handle_mission(self, msg) -> None:
        t = msg.get_type()
        if t == "MISSION_COUNT" and getattr(msg, "mission_type", 0) == mav.MAV_MISSION_TYPE_MISSION:
            if msg.count == 0:
                self._dl = {"count": 0}
                self._finish_mission({})
            else:
                self._dl = {"count": msg.count, "items": {}, "next": 0, "retries": 0, "t": 0.0}
                self._request_item(0)
        elif t == "MISSION_ITEM_INT" and self._dl is not None and msg.seq == self._dl.get("next"):
            dl = self._dl
            # seq 0 is home in ArduPilot; NAV commands are < 100; keep ones with a location
            if msg.seq > 0 and msg.command < 100 and (msg.x or msg.y):
                dl["items"][msg.seq] = (msg.x / 1e7, msg.y / 1e7)
            dl["next"] += 1
            dl["retries"] = 0
            if dl["next"] >= dl["count"]:
                self._finish_mission(dl["items"])
            else:
                self._request_item(dl["next"])

    # ---------------------------------------------------------------- receive
    def _handle(self, msg) -> bool:
        t = msg.get_type()
        if t == "BAD_DATA":
            return False
        new_target = False
        with self._lock:
            s = self._state
            if t == "HEARTBEAT":
                if msg.autopilot == mav.MAV_AUTOPILOT_INVALID or msg.type == mav.MAV_TYPE_GCS:
                    return False  # another GCS / companion, not the autopilot
                if self._target_sys != msg.get_srcSystem():
                    self._target_sys = msg.get_srcSystem()
                    self._target_comp = msg.get_srcComponent()
                    log.info("Autopilot found: sysid %d comp %d type %d",
                             self._target_sys, self._target_comp, msg.type)
                    new_target = True
                self._mav_type = msg.type
                self._last_hb = time.monotonic()
                s.mode = mavutil.mode_string_v10(msg)
                s.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == "VFR_HUD":
                s.groundspeed = msg.groundspeed
                s.heading = float(msg.heading)
                s.throttle_pct = msg.throttle
            elif t == "ATTITUDE":
                s.roll, s.pitch = msg.roll, msg.pitch
            elif t == "SYS_STATUS":
                s.batt_voltage = None if msg.voltage_battery in (0, 65535) else msg.voltage_battery / 1000.0
                s.batt_current = None if msg.current_battery < 0 else msg.current_battery / 100.0
                s.batt_remaining = None if msg.battery_remaining < 0 else msg.battery_remaining
            elif t == "GPS_RAW_INT":
                s.gps_fix = msg.fix_type
                s.gps_sats = 0 if msg.satellites_visible == 255 else msg.satellites_visible
            elif t == "GLOBAL_POSITION_INT":
                if msg.lat or msg.lon:
                    s.lat, s.lon = msg.lat / 1e7, msg.lon / 1e7
            elif t == "HOME_POSITION":
                if msg.latitude or msg.longitude:
                    s.home_lat, s.home_lon = msg.latitude / 1e7, msg.longitude / 1e7
            elif t == "NAV_CONTROLLER_OUTPUT":
                s.target_bearing = float(msg.target_bearing) % 360.0
                s.wp_dist = float(msg.wp_dist)
                s.nav_time = time.monotonic()
            elif t == "MISSION_CURRENT":
                s.wp_seq = msg.seq
            elif t == "STATUSTEXT":
                s.statustext = msg.text.rstrip("\x00")
                s.statustext_severity = msg.severity
                s.statustext_time = time.monotonic()
                log.info("FC: %s", s.statustext)
            elif t == "PARAM_VALUE":
                pid = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id
                self._params[pid] = msg.param_value
                wp, cruise = self._params.get("WP_SPEED"), self._params.get("CRUISE_SPEED")
                s.cruise_speed = wp if wp else cruise
            elif t == "COMMAND_ACK":
                if msg.result != mav.MAV_RESULT_ACCEPTED:
                    name = mav.enums["MAV_CMD"].get(msg.command)
                    s.statustext = f"REJECTED: {name.name if name else msg.command}"
                    s.statustext_severity = 4
                    s.statustext_time = time.monotonic()
                    log.warning("Command %s rejected (result %d)", msg.command, msg.result)
        return new_target

    # ---------------------------------------------------------------- send
    def _cmd_long(self, command: int, *params: float) -> None:
        p = list(params) + [0.0] * (7 - len(params))
        self._conn.mav.command_long_send(self._target_sys, self._target_comp, command, 0, *p)

    def _request_streams(self) -> None:
        for name, hz in STREAM_RATES_HZ.items():
            msg_id = getattr(mav, f"MAVLINK_MSG_ID_{name}")
            self._cmd_long(mav.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))
        self._cmd_long(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_HOME_POSITION)
        for p in ("WP_SPEED", "CRUISE_SPEED"):
            self._conn.mav.param_request_read_send(self._target_sys, self._target_comp, p.encode(), -1)

    def _drain_commands(self) -> None:
        while True:
            try:
                kind, arg = self._cmds.get_nowait()
            except queue.Empty:
                return
            if self._target_sys is None:
                continue  # nothing to talk to yet; drop
            if kind == "mode":
                mapping = mavutil.mode_mapping_byname(self._mav_type) or {}
                if arg not in mapping:
                    log.error("Mode %s unknown for vehicle type %d", arg, self._mav_type)
                    continue
                log.info("Set mode %s", arg)
                self._cmd_long(mav.MAV_CMD_DO_SET_MODE,
                               mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[arg])
            elif kind == "speed":
                log.info("Change speed %.2f m/s", arg)
                self._cmd_long(mav.MAV_CMD_DO_CHANGE_SPEED, 1, arg, -1)
            elif kind == "arm":
                self._cmd_long(mav.MAV_CMD_COMPONENT_ARM_DISARM, 1 if arg else 0)
            elif kind == "rc":
                pwm, ch = arg
                # MAVLink: ch1-8 UINT16_MAX = ignore, ch9-18 0 = ignore.
                vals = [65535] * 8 + [0] * 10
                vals[ch - 1] = pwm
                self._conn.mav.rc_channels_override_send(self._target_sys, self._target_comp, *vals)
