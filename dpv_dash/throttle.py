"""Dead-man trigger + speed level -> RC throttle override.

The Pi acts as the vehicle's "RC transmitter" for the throttle channel: while the
link is up it sends RC_CHANNELS_OVERRIDE continuously at `rate_hz`:
  * MANUAL mode and trigger held -> neutral + level% * (full - neutral)
  * anything else                -> neutral (motor stopped)
If this app or the Pi dies, the stream stops, ArduPilot's override times out
after RC_OVERRIDE_TIME, and its throttle failsafe takes over (see README).
"""
from __future__ import annotations

from PyQt5.QtCore import QObject, QTimer

from .state import UiState


class ThrottleSender(QObject):
    def __init__(self, link, ui: UiState, cfg: dict, parent=None):
        super().__init__(parent)
        self.link, self.ui = link, ui
        self.enabled = cfg.get("enabled", True)
        self.channel = cfg.get("channel", 3)
        self.neutral = cfg.get("neutral_pwm", 1500)
        self.full = cfg.get("full_pwm", 2000)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(int(1000 / cfg.get("rate_hz", 10)))

    def pwm_for(self, mode: str) -> int:
        if self.ui.trigger_held and mode == "MANUAL":
            return int(round(self.neutral + (self.full - self.neutral) * self.ui.speed_pct / 100.0))
        return self.neutral

    def _tick(self) -> None:
        if not self.enabled:
            return
        s = self.link.snapshot()
        if s.link_ok:
            self.link.send_throttle(self.pwm_for(s.mode), self.channel)
