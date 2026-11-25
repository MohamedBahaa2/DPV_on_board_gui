"""Independent link check:  python -m dpv_dash --check [--seconds N]

Talks to the Pixhawk with plain pymavlink, *without* going through
mavlink_link.py's parsing, and prints what the autopilot really sends:
message rates, the raw values behind every HUD field, sensor health bits
and the ArduPilot parameters this project depends on. Use it to confirm the
HUD shows real data, or to diagnose a link problem.

The dashboard must be stopped first: only one program can use the serial port.
"""
from __future__ import annotations

import os
import sys
import time
from collections import defaultdict
from pathlib import Path

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from .mavlink_link import STREAM_RATES_HZ, MavlinkLink  # noqa: E402

mav = mavutil.mavlink

# name -> (expected value or None for "just show it", why it matters)
PARAMS = {
    "SERIAL0_PROTOCOL": (2, "MAVLink2 on USB"),
    "SYSID_MYGCS": (255, "RC overrides accepted only from this sysid (Pi = 255)"),
    "MAV_GCS_SYSID": (255, "newer name of SYSID_MYGCS"),
    "RC_OVERRIDE_TIME": (0.5, "override expires 0.5 s after the Pi stops"),
    "FS_THR_ENABLE": (1, "radio/throttle failsafe on"),
    "FS_ACTION": (2, "failsafe action = Hold"),
    "RCMAP_THROTTLE": (3, "throttle is RC channel 3"),
    "WP_SPEED": (None, "100 % speed level in AUTO/RTL (0 = use CRUISE_SPEED)"),
    "CRUISE_SPEED": (None, "fallback cruise speed"),
    "BATT_MONITOR": (None, "0 = no battery monitor, 4 = analog V+I power module"),
    "BATT_CAPACITY": (None, "mAh; battery % is counted down from this since FC boot"),
    "GPS1_TYPE": (None, "0 = no GPS configured"),
    "GPS_TYPE": (None, "older name of GPS1_TYPE"),
}

SENSOR_BITS = [
    ("3D gyro", mav.MAV_SYS_STATUS_SENSOR_3D_GYRO),
    ("3D accel", mav.MAV_SYS_STATUS_SENSOR_3D_ACCEL),
    ("3D mag (compass)", mav.MAV_SYS_STATUS_SENSOR_3D_MAG),
    ("GPS", mav.MAV_SYS_STATUS_SENSOR_GPS),
    ("RC receiver", mav.MAV_SYS_STATUS_SENSOR_RC_RECEIVER),
    ("AHRS", mav.MAV_SYS_STATUS_AHRS),
    ("Battery", mav.MAV_SYS_STATUS_SENSOR_BATTERY),
]

GPS_FIX = {0: "no GPS connected", 1: "GPS connected, no fix", 2: "2D fix", 3: "3D fix",
           4: "DGPS", 5: "RTK float", 6: "RTK fixed"}


def _dashboard_running() -> list[int]:
    pids = []
    for p in Path("/proc").iterdir():
        if not p.name.isdigit() or int(p.name) == os.getpid():
            continue
        try:
            cmd = (p / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if "-m dpv_dash" in cmd and "--check" not in cmd and "--test-buttons" not in cmd:
            pids.append(int(p.name))
    return pids


def run_check(mcfg: dict, seconds: float = 10.0) -> int:
    pids = _dashboard_running()
    if pids:
        print(f"The dashboard is running (pid {', '.join(map(str, pids))}) and owns the serial port.\n"
              "Stop it first:  pkill -f scripts/run.sh; pkill -f 'python -m dpv_dash'")
        return 2

    port = MavlinkLink._resolve(mcfg["connection"])
    print(f"Port: {port}")
    conn = mavutil.mavlink_connection(port, baud=mcfg.get("baud", 115200),
                                      source_system=mcfg.get("source_system", 255),
                                      source_component=mcfg.get("source_component", 191),
                                      autoreconnect=False)
    print("Waiting for autopilot heartbeat ...")
    t_end = time.monotonic() + 5
    hb = None
    while time.monotonic() < t_end:
        conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        m = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if m and m.autopilot != mav.MAV_AUTOPILOT_INVALID and m.type != mav.MAV_TYPE_GCS:
            hb = m
            break
    if hb is None:
        print("FAIL: no heartbeat from an autopilot within 5 s.")
        return 1
    sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()

    def cmd(c, *p):
        p = list(p) + [0.0] * (7 - len(p))
        conn.mav.command_long_send(sysid, compid, c, 0, *p)

    for name, hz in STREAM_RATES_HZ.items():
        cmd(mav.MAV_CMD_SET_MESSAGE_INTERVAL, getattr(mav, f"MAVLINK_MSG_ID_{name}"), int(1e6 / hz))
    cmd(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_AUTOPILOT_VERSION)
    cmd(mav.MAV_CMD_REQUEST_MESSAGE, mav.MAVLINK_MSG_ID_HOME_POSITION)

    def ask_params():
        for p in PARAMS:
            conn.mav.param_request_read_send(sysid, compid, p.encode(), -1)

    ask_params()
    print(f"Listening for {seconds:.0f} s ...\n")
    # let the new rates take effect before counting
    t0 = time.monotonic() + 1.0
    t_end = t0 + seconds
    counts: dict[str, int] = defaultdict(int)
    last: dict[str, object] = {}
    params: dict[str, float] = {}
    texts: list[str] = []
    asked_again = False
    last_hb_sent = 0.0
    while (now := time.monotonic()) < t_end:
        if now - last_hb_sent > 1:
            conn.mav.heartbeat_send(mav.MAV_TYPE_GCS, mav.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            last_hb_sent = now
        if not asked_again and now > t0 + seconds / 2:
            ask_params()
            asked_again = True
        m = conn.recv_match(blocking=True, timeout=0.2)
        if m is None or m.get_srcSystem() != sysid:
            continue
        t = m.get_type()
        if t == "HEARTBEAT" and m.get_srcComponent() != compid:
            continue
        last[t] = m
        if now >= t0:
            counts[t] += 1
        if t == "PARAM_VALUE":
            params[m.param_id.rstrip("\x00")] = m.param_value
        elif t == "STATUSTEXT":
            texts.append(m.text.rstrip("\x00"))
    conn.close()

    # ------------------------------------------------------------ report
    hb = last.get("HEARTBEAT", hb)
    print("== Autopilot")
    print(f"  sysid {sysid} comp {compid}, vehicle type {hb.type}, "
          f"mode {mavutil.mode_string_v10(hb)}, "
          f"{'ARMED' if hb.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED else 'disarmed'}")
    av = last.get("AUTOPILOT_VERSION")
    if av:
        v = av.flight_sw_version
        print(f"  firmware {v >> 24}.{(v >> 16) & 0xff}.{(v >> 8) & 0xff}")

    print("\n== Message rates (measured / requested)")
    for t in sorted(counts):
        req = STREAM_RATES_HZ.get(t)
        hz = counts[t] / seconds
        flag = ""
        if req and req >= 1 and hz < 0.7 * req:
            flag = "  <-- LOW"
        print(f"  {t:24s} {hz:6.1f} Hz" + (f"  / {req:g}" if req else "") + flag)
    for t, req in STREAM_RATES_HZ.items():
        if req >= 0.5 and t not in counts:
            note = ("  (normal: only sent in AUTO/RTL/GUIDED)" if t == "NAV_CONTROLLER_OUTPUT"
                    else "  <-- MISSING")
            print(f"  {t:24s}   none  / {req:g}{note}")

    print("\n== Raw values behind the HUD")
    m = last.get("VFR_HUD")
    if m:
        print(f"  VFR_HUD       groundspeed {m.groundspeed:.2f} m/s, heading {m.heading} deg, throttle {m.throttle} %")
    m = last.get("ATTITUDE")
    if m:
        import math
        print(f"  ATTITUDE      roll {math.degrees(m.roll):.1f} deg, pitch {math.degrees(m.pitch):.1f} deg, "
              f"yaw {math.degrees(m.yaw) % 360:.1f} deg")
    ss = last.get("SYS_STATUS")
    if ss:
        print(f"  SYS_STATUS    battery {ss.voltage_battery / 1000:.2f} V (raw {ss.voltage_battery}), "
              f"{ss.current_battery / 100:.2f} A (raw {ss.current_battery}), "
              f"remaining {ss.battery_remaining} % (-1 = unknown)")
    g = last.get("GPS_RAW_INT")
    if g:
        print(f"  GPS_RAW_INT   fix_type {g.fix_type} ({GPS_FIX.get(g.fix_type, '?')}), "
              f"satellites {g.satellites_visible}")
    m = last.get("GLOBAL_POSITION_INT")
    if m:
        print(f"  GLOBAL_POS    lat {m.lat / 1e7:.7f}, lon {m.lon / 1e7:.7f}  (0,0 = no position)")
    else:
        print("  GLOBAL_POS    not received (normal with no GPS)")
    m = last.get("HOME_POSITION")
    print(f"  HOME_POSITION {'lat %.7f lon %.7f' % (m.latitude / 1e7, m.longitude / 1e7) if m else 'not set'}")
    m = last.get("RC_CHANNELS")
    if m:
        print(f"  RC_CHANNELS   ch1..4 = {m.chan1_raw} {m.chan2_raw} {m.chan3_raw} {m.chan4_raw} us "
              f"(0 = no RC input; the dashboard's override shows up on ch3)")
    m = last.get("SERVO_OUTPUT_RAW")
    if m:
        print(f"  SERVO_OUTPUT  servo1..4 = {m.servo1_raw} {m.servo2_raw} {m.servo3_raw} {m.servo4_raw} us")

    if ss:
        print("\n== Sensors (SYS_STATUS bits)")
        for name, bit in SENSOR_BITS:
            present = bool(ss.onboard_control_sensors_present & bit)
            enabled = bool(ss.onboard_control_sensors_enabled & bit)
            healthy = bool(ss.onboard_control_sensors_health & bit)
            state = ("not present" if not present else
                     f"present, {'enabled' if enabled else 'disabled'}, {'HEALTHY' if healthy else 'UNHEALTHY'}")
            print(f"  {name:18s} {state}")

    print("\n== Parameters")
    for name, (exp, why) in PARAMS.items():
        if name not in params:
            continue
        val = params[name]
        mark = "" if exp is None else ("  OK " if abs(val - exp) < 1e-3 else "  !! expected %g" % exp)
        print(f"  {name:18s} {val:g}{mark}   ({why})")

    if texts:
        print("\n== Autopilot messages during the check")
        for t in texts[-10:]:
            print(f"  {t}")

    # ------------------------------------------------------------ verdicts
    print("\n== Verdict")
    bm = params.get("BATT_MONITOR")
    if ss:
        v = ss.voltage_battery / 1000
        if bm == 0:
            print("  Battery: BATT_MONITOR=0, no battery monitor configured. HUD shows NO MON.")
        elif v < 6.0:
            print(f"  Battery: monitor configured (BATT_MONITOR={bm:g}) but only {v:.2f} V measured. "
                  "No battery is on the power module, so V/A/% are noise. HUD shows NO BATT.")
        else:
            print(f"  Battery: {v:.2f} V measured, looks like a real pack. "
                  "Note: % is counted down from BATT_CAPACITY since FC boot, not measured.")
    if g:
        print(f"  GPS: {GPS_FIX.get(g.fix_type, '?')}"
              + ("; speed and position are real." if g.fix_type >= 3 else
                 "; speed, position, HOME and WP guidance can't be trusted, so the HUD blanks them."))
    if "VFR_HUD" in counts:
        print("  Heading/attitude: streaming"
              + (" (compass healthy)." if ss and ss.onboard_control_sensors_health & mav.MAV_SYS_STATUS_SENSOR_3D_MAG
                 else "."))
    att = last.get("ATTITUDE")
    if att:
        import math
        if abs(math.degrees(att.pitch)) > 70 or abs(math.degrees(att.roll)) > 70:
            print(f"  !! Attitude roll {math.degrees(att.roll):.0f} deg / pitch {math.degrees(att.pitch):.0f} deg: "
                  "the board is on its side, or AHRS_ORIENTATION doesn't match how it is mounted. "
                  "Heading is unreliable near +/-90 deg pitch.")
    if ss and ss.onboard_control_sensors_present & mav.MAV_SYS_STATUS_AHRS and \
            not ss.onboard_control_sensors_health & mav.MAV_SYS_STATUS_AHRS:
        print("  AHRS reports unhealthy (expected with no GPS: the EKF has no position). Heading still comes from the compass.")
    rot = params.get("RC_OVERRIDE_TIME")
    if rot is not None and rot > 1.0:
        print(f"  !! RC_OVERRIDE_TIME={rot:g} s: if the Pi/app dies the motor keeps its last throttle for "
              f"{rot:g} s. Set it to 0.5.")
    gcs = params.get("MAV_GCS_SYSID", params.get("SYSID_MYGCS"))
    if gcs is not None and int(gcs) != mcfg.get("source_system", 255):
        print(f"  !! SYSID_MYGCS={gcs:g} but the Pi uses sysid {mcfg.get('source_system', 255)}: "
              "throttle overrides will be IGNORED.")
    return 0


if __name__ == "__main__":
    sys.exit("run as: python -m dpv_dash --check")
