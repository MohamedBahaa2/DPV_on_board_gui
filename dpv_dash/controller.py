"""Turns button presses into actions.

To add a new button action:
  1. give the button a GPIO pin in config.toml [buttons] and add its name to
     controls.BUTTON_NAMES,
  2. write a method here and register it in `self.actions` (press) or
     `self.release_actions` (release),
  3. optionally map a keyboard key in hud.KEYMAP for desk testing.
"""
from __future__ import annotations

import logging

from PyQt5.QtCore import QObject, QTimer

from .state import UiState

log = logging.getLogger(__name__)

AUTO_SPEED_MODES = ("AUTO", "RTL")


class Controller(QObject):
    def __init__(self, link, cfg: dict, parent=None):
        super().__init__(parent)
        self.link = link
        sp = cfg.get("speed", {})
        levels = list(sp.get("levels", [25, 50, 75, 100]))
        self.ui = UiState(speed_levels=levels,
                          speed_index=min(sp.get("default_index", 0), len(levels) - 1))
        mcfg = cfg.get("modes", {})
        self.mode_cycle = list(mcfg.get("cycle", ["MANUAL", "AUTO", "RTL"]))

        self._commit = QTimer(self)
        self._commit.setSingleShot(True)
        self._commit.setInterval(int(1000 * mcfg.get("commit_delay_s", 1.5)))
        self._commit.timeout.connect(self._commit_mode)

        # Watch for mode changes (from us or the ground station) to re-apply speed.
        self._last_mode = None
        self._watch = QTimer(self)
        self._watch.timeout.connect(self._watch_mode)
        self._watch.start(200)

        self.actions = {
            "mode": self.cycle_mode,
            "speed": self.cycle_speed,
            "trigger": lambda: self.set_trigger(True),
            "guide": self.toggle_guide,
        }
        self.release_actions = {
            "trigger": lambda: self.set_trigger(False),
        }

    # ------------------------------------------------------------ dispatch
    def on_press(self, name: str) -> None:
        log.info("Button %s pressed", name)
        fn = self.actions.get(name)
        if fn:
            fn()

    def on_release(self, name: str) -> None:
        log.info("Button %s released", name)
        fn = self.release_actions.get(name)
        if fn:
            fn()

    # ------------------------------------------------------------ actions
    def cycle_mode(self) -> None:
        current = self.ui.pending_mode or self.link.snapshot().mode
        if current in self.mode_cycle:
            nxt = self.mode_cycle[(self.mode_cycle.index(current) + 1) % len(self.mode_cycle)]
        else:
            nxt = self.mode_cycle[0]
        self.ui.pending_mode = nxt
        self._commit.start()  # (re)start the countdown
        log.info("Pending mode %s", nxt)

    def _commit_mode(self) -> None:
        target, self.ui.pending_mode = self.ui.pending_mode, None
        if target and target != self.link.snapshot().mode:
            self.link.set_mode(target)

    def cycle_speed(self) -> None:
        self.ui.speed_index = (self.ui.speed_index + 1) % len(self.ui.speed_levels)
        log.info("Speed level %d%%", self.ui.speed_pct)
        self._apply_auto_speed()

    def set_trigger(self, held: bool) -> None:
        self.ui.trigger_held = held

    def toggle_guide(self) -> None:
        self.ui.guide_source = "TARGET" if self.ui.guide_source == "HOME" else "HOME"
        log.info("Guide source %s", self.ui.guide_source)

    # ------------------------------------------------------------ helpers
    def _apply_auto_speed(self) -> None:
        s = self.link.snapshot()
        if s.link_ok and s.mode in AUTO_SPEED_MODES and s.cruise_speed:
            self.link.change_speed(s.cruise_speed * self.ui.speed_pct / 100.0)

    def _watch_mode(self) -> None:
        mode = self.link.snapshot().mode
        if mode != self._last_mode:
            self._last_mode = mode
            if mode in AUTO_SPEED_MODES:
                self._apply_auto_speed()
