"""Hall-effect push buttons on Pi GPIO -> Qt signals.

gpiozero callbacks run on their own threads; emitting a Qt signal from there is
safe and delivers the event on the UI thread (queued connection).
"""
from __future__ import annotations

import logging

from PyQt5.QtCore import QObject, pyqtSignal

log = logging.getLogger(__name__)

BUTTON_NAMES = ("mode", "speed", "trigger", "guide")


class GpioButtons(QObject):
    pressed = pyqtSignal(str)
    released = pyqtSignal(str)

    def __init__(self, cfg: dict, parent=None):
        super().__init__(parent)
        self._buttons = []
        try:
            from gpiozero import Button
        except Exception as e:  # not on a Pi / gpiozero missing
            log.warning("gpiozero unavailable (%s); GPIO buttons disabled", e)
            return
        active_high = cfg.get("active_high", False)
        bounce = cfg.get("bounce_time", 0.03)
        for name in BUTTON_NAMES:
            pin = cfg.get(name)
            if pin is None:
                continue
            try:
                if active_high:  # sensor drives the line high; no internal pull
                    b = Button(pin, pull_up=None, active_state=True, bounce_time=bounce)
                else:            # open-drain, pulls low; use internal pull-up
                    b = Button(pin, pull_up=True, bounce_time=bounce)
            except Exception as e:
                log.error("Button %s on GPIO%s failed: %s", name, pin, e)
                continue
            b.when_pressed = lambda n=name: self.pressed.emit(n)
            b.when_released = lambda n=name: self.released.emit(n)
            self._buttons.append(b)
            log.info("Button %-7s on GPIO%d", name, pin)

    def close(self):
        for b in self._buttons:
            b.close()


def test_buttons(cfg: dict) -> None:
    """Print button edges to the terminal (python -m dpv_dash --test-buttons)."""
    import signal
    from gpiozero import Button

    active_high = cfg.get("active_high", False)
    for name in BUTTON_NAMES:
        pin = cfg[name]
        b = (Button(pin, pull_up=None, active_state=True, bounce_time=cfg.get("bounce_time", 0.03))
             if active_high else Button(pin, pull_up=True, bounce_time=cfg.get("bounce_time", 0.03)))
        b.when_pressed = lambda n=name, p=pin: print(f"{n:8s} GPIO{p:<3d} PRESSED", flush=True)
        b.when_released = lambda n=name, p=pin: print(f"{n:8s} GPIO{p:<3d} released", flush=True)
        print(f"{name:8s} GPIO{pin:<3d} currently {'PRESSED' if b.is_pressed else 'released'}")
        globals().setdefault("_keep", []).append(b)
    print("Press the buttons (Ctrl+C to quit)...", flush=True)
    signal.pause()
