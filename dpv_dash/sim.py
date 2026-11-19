"""Fake vehicle with the same interface as MavlinkLink, for bench testing the HUD."""
from __future__ import annotations

import copy
import math
import random
import threading
import time

from .state import VehicleState, bearing_distance

HOME = (30.0444, 31.2357)
WAYPOINTS = [(30.0460, 31.2380), (30.0475, 31.2350), (30.0450, 31.2330)]
SIM_CRUISE = 1.5  # m/s


class SimLink:
    def __init__(self, cfg: dict | None = None, rate_hz: float = 20.0):
        self._s = VehicleState(link_ok=True, mode="MANUAL", armed=True,
                               batt_voltage=16.6, batt_current=0.0, batt_remaining=100,
                               gps_fix=3, gps_sats=14, lat=HOME[0], lon=HOME[1],
                               home_lat=HOME[0], home_lon=HOME[1], wp_seq=1,
                               cruise_speed=SIM_CRUISE, heading=40.0,
                               mission={i + 1: wp for i, wp in enumerate(WAYPOINTS)})
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._dt = 1.0 / rate_hz
        self._throttle = 0.0           # 0..1 from RC override
        self._throttle_time = 0.0
        self._speed_cmd = SIM_CRUISE
        self._wp_i = 0
        self._thread = threading.Thread(target=self._run, name="sim", daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def snapshot(self) -> VehicleState:
        with self._lock:
            return copy.copy(self._s)

    def set_mode(self, name: str):
        def later():
            time.sleep(0.3)
            with self._lock:
                self._s.mode = name
                self._text(f"Mode {name}")
        threading.Thread(target=later, daemon=True).start()

    def change_speed(self, mps: float):
        self._speed_cmd = mps

    def send_throttle(self, pwm: int, channel: int):
        self._throttle = max(0.0, min(1.0, (pwm - 1500) / 500.0))
        self._throttle_time = time.monotonic()

    def arm(self, arm: bool):
        with self._lock:
            self._s.armed = arm

    def _text(self, t: str, sev: int = 6):
        self._s.statustext, self._s.statustext_severity = t, sev
        self._s.statustext_time = time.monotonic()

    def _run(self):
        t0 = time.monotonic()
        while not self._stop.wait(self._dt):
            t = time.monotonic() - t0
            with self._lock:
                self._step(t)

    def _step(self, t: float):
        s, dt = self._s, self._dt
        if time.monotonic() - self._throttle_time > 0.5:  # mimic RC_OVERRIDE_TIME
            self._throttle = 0.0
        if s.mode == "MANUAL":
            target_v = 2.2 * self._throttle
            s.heading = (s.heading + 6 * math.sin(t * 0.3) * dt) % 360
            s.target_bearing, s.wp_dist = None, None
        else:
            dest = HOME if s.mode == "RTL" else WAYPOINTS[self._wp_i]
            brg, dist = bearing_distance(s.lat, s.lon, *dest)
            err = (brg - s.heading + 180) % 360 - 180
            s.heading = (s.heading + max(-20 * dt, min(20 * dt, err))) % 360
            target_v = self._speed_cmd if dist > 3 else 0.0
            s.target_bearing, s.wp_dist, s.nav_time = brg, dist, time.monotonic()
            s.wp_seq = self._wp_i + 1
            if s.mode == "AUTO" and dist < 5:
                self._wp_i = (self._wp_i + 1) % len(WAYPOINTS)
                self._text(f"Reached waypoint #{s.wp_seq}")
        s.groundspeed += (target_v - s.groundspeed) * min(1.0, dt * 1.5)
        s.throttle_pct = int(100 * min(1.0, s.groundspeed / 2.2))
        # move
        d = s.groundspeed * dt
        s.lat += d * math.cos(math.radians(s.heading)) / 111320.0
        s.lon += d * math.sin(math.radians(s.heading)) / (111320.0 * math.cos(math.radians(s.lat)))
        s.roll = math.radians(3 * math.sin(t * 0.9))
        s.pitch = math.radians(2 * math.sin(t * 0.6))
        # battery
        s.batt_current = 1.0 + 25.0 * s.throttle_pct / 100.0 + random.uniform(-0.3, 0.3)
        s.batt_remaining = max(0, 100 - int(t / 6))
        s.batt_voltage = 13.2 + 3.4 * s.batt_remaining / 100.0 - 0.01 * s.batt_current
        if int(t) % 50 == 49 and time.monotonic() - s.statustext_time > 5:
            self._text("EKF3 IMU0 is using GPS")
