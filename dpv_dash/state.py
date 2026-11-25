"""Shared data structures: what the vehicle reports, and what the driver has selected."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field


# How long a value may go without being refreshed before the HUD stops trusting
# it (seconds). Roughly 3-10x the stream period requested in mavlink_link.py.
STALE_S = {
    "VFR_HUD": 1.0,
    "ATTITUDE": 1.0,
    "GLOBAL_POSITION_INT": 2.0,
    "SYS_STATUS": 3.0,
    "GPS_RAW_INT": 3.0,
}
TILT_WARN_DEG = 70.0  # beyond this roll/pitch the compass heading is unreliable


@dataclass
class VehicleState:
    """Latest telemetry from the Pixhawk. Written by the link thread, read by the HUD.

    `None` always means "unknown / not trustworthy right now": the HUD draws it
    as dashes. Values are never invented. `apply_trust()` turns the raw state
    into what may be displayed.
    """
    link_ok: bool = False
    mode: str = "----"
    armed: bool | None = None

    groundspeed: float | None = None  # m/s (only with a 3D GPS fix)
    heading: float | None = None      # deg, compass
    heading_warn: str = ""            # "" ok, "AHRS" = AHRS unhealthy, "TILT" = near-vertical
    throttle_pct: int | None = None
    roll: float | None = None         # rad
    pitch: float | None = None        # rad
    ahrs_ok: bool | None = None       # SYS_STATUS AHRS health bit

    batt_status: str = "--"           # "OK", "NO MON" (no monitor), "NO BATT" (implausible V), "--" (no data)
    batt_monitor: int | None = None   # BATT_MONITOR param
    batt_present: bool | None = None  # SYS_STATUS battery sensor bit
    batt_voltage: float | None = None  # V
    batt_current: float | None = None  # A
    batt_remaining: int | None = None  # % (ArduPilot counts it down from BATT_CAPACITY since FC boot)

    gps_fix: int | None = None        # 0 no GPS, 1 no fix, 2 = 2D, 3 = 3D, 4+ = DGPS/RTK
    gps_sats: int | None = None
    lat: float | None = None
    lon: float | None = None
    home_lat: float | None = None
    home_lon: float | None = None

    wp_seq: int | None = None         # active mission item (MISSION_CURRENT)
    wp_dist: float | None = None      # m, from NAV_CONTROLLER_OUTPUT (autopilot modes only)
    target_bearing: float | None = None  # deg, ditto
    nav_time: float = 0.0             # monotonic time of last NAV_CONTROLLER_OUTPUT
    # Mission downloaded from the Pixhawk: seq -> (lat, lon) for items with a location.
    # Replaced wholesale on update (never mutated), so shallow snapshot copies are safe.
    mission: dict = field(default_factory=dict)

    cruise_speed: float | None = None  # m/s, WP_SPEED (or CRUISE_SPEED if WP_SPEED=0)

    statustext: str = ""
    statustext_severity: int = 6
    statustext_time: float = 0.0

    # message type -> monotonic time it was last received
    rx: dict = field(default_factory=dict)

    def home_bearing_distance(self) -> tuple[float, float] | None:
        if None in (self.lat, self.lon, self.home_lat, self.home_lon):
            return None
        return bearing_distance(self.lat, self.lon, self.home_lat, self.home_lon)

    def target_bearing_distance(self) -> tuple[int | None, float, float] | None:
        """(waypoint seq, bearing, distance) to the active mission waypoint.

        Computed locally from the downloaded mission so it works in MANUAL too;
        falls back to the autopilot's NAV_CONTROLLER_OUTPUT.
        """
        if self.wp_seq is not None and self.lat is not None and self.mission:
            for seq in sorted(k for k in self.mission if k >= self.wp_seq):
                return (seq, *bearing_distance(self.lat, self.lon, *self.mission[seq]))
        if (self.target_bearing is not None and self.wp_dist is not None
                and time.monotonic() - self.nav_time < 2.0):
            return (self.wp_seq, self.target_bearing, self.wp_dist)
        return None


@dataclass
class UiState:
    """Driver selections made with the buttons (owned by the controller, main thread)."""
    speed_index: int = 0
    speed_levels: list[int] = field(default_factory=lambda: [25, 50, 75, 100])
    pending_mode: str | None = None
    guide_source: str = "HOME"        # "HOME" or "TARGET"
    trigger_held: bool = False

    @property
    def speed_pct(self) -> int:
        return self.speed_levels[self.speed_index]


def apply_trust(s: VehicleState, min_valid_v: float = 6.0, now: float | None = None) -> VehicleState:
    """Blank every value that is stale, missing or physically implausible.

    Called on a *copy* (a snapshot) by the link, so the HUD, controller and
    blackbox all see the same trusted values. Returns `s` for convenience.
    """
    now = time.monotonic() if now is None else now

    def fresh(msg: str) -> bool:
        return s.link_ok and now - s.rx.get(msg, -1e9) < STALE_S[msg]

    if not s.link_ok:
        s.mode, s.armed = "----", None
    if not fresh("VFR_HUD"):
        s.groundspeed = s.heading = s.throttle_pct = None
    if not fresh("ATTITUDE"):
        s.roll = s.pitch = None
    if not fresh("GPS_RAW_INT"):
        s.gps_fix = s.gps_sats = None
    has_fix = s.gps_fix is not None and s.gps_fix >= 3
    if not has_fix:
        s.groundspeed = None          # without GPS this is an EKF guess, not a measurement
    if not (has_fix and fresh("GLOBAL_POSITION_INT")):
        s.lat = s.lon = None

    s.heading_warn = ""
    if s.heading is not None:
        if s.roll is not None and max(abs(math.degrees(s.roll)), abs(math.degrees(s.pitch))) > TILT_WARN_DEG:
            s.heading_warn = "TILT"
        elif s.ahrs_ok is False:
            s.heading_warn = "AHRS"

    if not fresh("SYS_STATUS"):
        s.batt_status = "--"
    elif s.batt_monitor == 0 or s.batt_present is False:
        s.batt_status = "NO MON"
    elif s.batt_voltage is None or s.batt_voltage < min_valid_v:
        s.batt_status = "NO BATT"     # monitor configured but nothing (real) connected
    else:
        s.batt_status = "OK"
    if s.batt_status != "OK":
        s.batt_voltage = s.batt_current = s.batt_remaining = None
    return s


def bearing_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> tuple[float, float]:
    """Initial bearing (deg) and great-circle distance (m) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
    brg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0
    a = math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlon / 2) ** 2
    dist = 2 * 6371000.0 * math.asin(min(1.0, math.sqrt(a)))
    return brg, dist


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0
