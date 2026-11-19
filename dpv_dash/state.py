"""Shared data structures: what the vehicle reports, and what the driver has selected."""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field


@dataclass
class VehicleState:
    """Latest telemetry from the Pixhawk. Written by the link thread, read by the HUD."""
    link_ok: bool = False
    mode: str = "----"
    armed: bool = False

    groundspeed: float = 0.0          # m/s
    heading: float = 0.0              # deg, compass
    throttle_pct: int = 0
    roll: float = 0.0                 # rad
    pitch: float = 0.0                # rad

    batt_voltage: float | None = None  # V
    batt_current: float | None = None  # A
    batt_remaining: int | None = None  # %

    gps_fix: int = 0                  # 0-1 none, 2 = 2D, 3 = 3D, 4+ = DGPS/RTK
    gps_sats: int = 0
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
