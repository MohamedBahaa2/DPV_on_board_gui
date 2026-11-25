"""Blackbox: a per-run record of everything the Pixhawk said, what the driver saw
and what the driver did.

One folder per app start, under [blackbox] dir (default ~/dpv_blackbox):

  <seq>_<YYYYmmdd-HHMMSS>[_sim]/
    mavlink.tlog   every MAVLink message received AND sent, timestamped
                   (standard telemetry-log format: Mission Planner, MAVExplorer,
                   `mavlogdump.py mavlink.tlog`). Written by the link thread.
    hud.csv        csv_rate_hz rows of the trusted values on screen + driver
                   inputs + throttle PWM sent. Written by the Qt main thread.
    events.log     the app log: link up/down, button presses, mode/speed
                   commands and ACKs, throttle changes, autopilot messages.
    meta.json      start time, whether the clock was NTP-synced, config.

<seq> keeps sessions in order even when the Pi boots before NTP fixes the clock.
Files are flushed + fsync'ed every 2 s, so a power cut loses at most ~2 s.
The oldest sessions are deleted when the total exceeds max_total_mb.
"""
from __future__ import annotations

import csv
import json
import logging
import math
import os
import re
import shutil
import time
from datetime import datetime
from pathlib import Path

from .state import UiState, VehicleState

log = logging.getLogger(__name__)

SYNC_EVERY_S = 2.0
CSV_FIELDS = [
    "time", "t", "link_ok", "mode", "armed",
    "groundspeed_mps", "heading_deg", "heading_warn", "throttle_pct", "roll_deg", "pitch_deg",
    "batt_status", "batt_v", "batt_a", "batt_pct",
    "gps_fix", "gps_sats", "lat", "lon", "home_lat", "home_lon",
    "wp_seq", "guide_source", "guide_brg_deg", "guide_dist_m",
    "speed_pct", "pending_mode", "trigger_held", "thr_pwm_sent",
]


def _fmt(v, nd=None):
    if v is None:
        return ""
    if isinstance(v, bool):
        return int(v)
    if nd is not None:
        return round(v, nd)
    return v


class Blackbox:
    def __init__(self, cfg: dict, app_cfg: dict, sim: bool = False):
        self.root = Path(os.path.expanduser(cfg.get("dir", "~/dpv_blackbox")))
        self.root.mkdir(parents=True, exist_ok=True)
        self._prune(cfg.get("max_total_mb", 2000))
        seq = 1 + max((int(m.group(1)) for d in self.root.iterdir()
                       if (m := re.match(r"(\d+)_", d.name))), default=0)
        name = f"{seq:05d}_{datetime.now():%Y%m%d-%H%M%S}" + ("_sim" if sim else "")
        self.dir = self.root / name
        self.dir.mkdir()
        self._t0 = time.monotonic()
        self._last_sync = self._t0

        (self.dir / "meta.json").write_text(json.dumps({
            "started": datetime.now().astimezone().isoformat(timespec="seconds"),
            "clock_ntp_synced": Path("/run/systemd/timesync/synchronized").exists(),
            "boot_id": _read("/proc/sys/kernel/random/boot_id"),
            "sim": sim,
            "config": app_cfg,
        }, indent=2, default=str))

        # events.log: everything the app logs (root logger) goes here too
        self._handler = logging.FileHandler(self.dir / "events.log")
        self._handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s"))
        self._handler.setLevel(logging.INFO)
        logging.getLogger().addHandler(self._handler)

        self._csv_f = open(self.dir / "hud.csv", "w", newline="")
        self._csv = csv.DictWriter(self._csv_f, fieldnames=CSV_FIELDS)
        self._csv.writeheader()
        log.info("Blackbox recording to %s", self.dir)

    # ------------------------------------------------------------ link thread
    def open_tlog(self):
        """Opened by the MAVLink thread on each (re)connect; appends."""
        try:
            return open(self.dir / "mavlink.tlog", "ab")
        except OSError as e:
            log.error("Blackbox tlog unavailable: %s", e)
            return None

    # ------------------------------------------------------------ Qt main thread
    def write_row(self, s: VehicleState, ui: UiState, thr_pwm: int | None) -> None:
        if ui.guide_source == "HOME":
            bd = s.home_bearing_distance()
        else:
            t = s.target_bearing_distance()
            bd = t[1:] if t else None
        self._csv.writerow({
            "time": datetime.now().isoformat(timespec="milliseconds"),
            "t": round(time.monotonic() - self._t0, 2),
            "link_ok": int(s.link_ok), "mode": s.mode, "armed": _fmt(s.armed),
            "groundspeed_mps": _fmt(s.groundspeed, 2), "heading_deg": _fmt(s.heading, 1),
            "heading_warn": s.heading_warn, "throttle_pct": _fmt(s.throttle_pct),
            "roll_deg": _fmt(None if s.roll is None else math.degrees(s.roll), 1),
            "pitch_deg": _fmt(None if s.pitch is None else math.degrees(s.pitch), 1),
            "batt_status": s.batt_status, "batt_v": _fmt(s.batt_voltage, 2),
            "batt_a": _fmt(s.batt_current, 2), "batt_pct": _fmt(s.batt_remaining),
            "gps_fix": _fmt(s.gps_fix), "gps_sats": _fmt(s.gps_sats),
            "lat": _fmt(s.lat, 7), "lon": _fmt(s.lon, 7),
            "home_lat": _fmt(s.home_lat, 7), "home_lon": _fmt(s.home_lon, 7),
            "wp_seq": _fmt(s.wp_seq), "guide_source": ui.guide_source,
            "guide_brg_deg": _fmt(bd[0] if bd else None, 1), "guide_dist_m": _fmt(bd[1] if bd else None, 1),
            "speed_pct": ui.speed_pct, "pending_mode": ui.pending_mode or "",
            "trigger_held": int(ui.trigger_held), "thr_pwm_sent": _fmt(thr_pwm),
        })
        now = time.monotonic()
        if now - self._last_sync >= SYNC_EVERY_S:
            self._last_sync = now
            self._sync()

    def _sync(self) -> None:
        for f in (self._csv_f, self._handler.stream):
            try:
                f.flush()
                os.fsync(f.fileno())
            except (OSError, ValueError):
                pass

    def close(self) -> None:
        self._sync()
        self._csv_f.close()
        logging.getLogger().removeHandler(self._handler)
        self._handler.close()

    # ------------------------------------------------------------ retention
    def _prune(self, max_total_mb: float) -> None:
        sessions = sorted((d for d in self.root.iterdir() if d.is_dir() and re.match(r"\d+_", d.name)),
                          key=lambda d: int(d.name.split("_")[0]))
        sizes = {d: sum(f.stat().st_size for f in d.rglob("*") if f.is_file()) for d in sessions}
        total = sum(sizes.values())
        while sessions and total > max_total_mb * 1e6:
            d = sessions.pop(0)
            total -= sizes[d]
            shutil.rmtree(d, ignore_errors=True)
            log.info("Blackbox: deleted old session %s", d.name)


def _read(path: str) -> str:
    try:
        return Path(path).read_text().strip()
    except OSError:
        return ""
