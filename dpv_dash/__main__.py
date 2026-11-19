"""Entry point: python -m dpv_dash [--sim] [--windowed] [--conn URL] [--test-buttons]"""
from __future__ import annotations

import argparse
import logging
import signal
import sys
import tomllib
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config.toml"


def main() -> int:
    ap = argparse.ArgumentParser(prog="dpv_dash", description="DPV driver HUD")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--sim", action="store_true", help="fake vehicle instead of the Pixhawk")
    ap.add_argument("--windowed", action="store_true", help="normal window instead of full screen")
    ap.add_argument("--conn", help="override [mavlink] connection, e.g. udpin:0.0.0.0:14550")
    ap.add_argument("--test-buttons", action="store_true", help="print GPIO button edges and exit")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    with open(args.config, "rb") as f:
        cfg = tomllib.load(f)

    if args.test_buttons:
        from .controls import test_buttons
        test_buttons(cfg["buttons"])
        return 0

    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QApplication

    from .controller import Controller
    from .controls import GpioButtons
    from .hud import Hud
    from .throttle import ThrottleSender

    if args.sim:
        from .sim import SimLink
        link = SimLink()
    else:
        from .mavlink_link import MavlinkLink
        mcfg = dict(cfg["mavlink"])
        if args.conn:
            mcfg["connection"] = args.conn
        link = MavlinkLink(mcfg)

    app = QApplication(sys.argv)
    app.setApplicationName("dpv_dash")
    link.start()

    ctl = Controller(link, cfg)
    throttle = ThrottleSender(link, ctl.ui, cfg.get("throttle", {}))  # noqa: F841 (kept alive)
    buttons = GpioButtons(cfg.get("buttons", {}))
    buttons.pressed.connect(ctl.on_press)
    buttons.released.connect(ctl.on_release)

    hud = Hud(link, ctl, cfg.get("display", {}))
    if args.windowed or not cfg.get("display", {}).get("fullscreen", True):
        hud.resize(800, 480)
        hud.show()
    else:
        hud.showFullScreen()

    # Let Ctrl+C / SIGTERM close the app cleanly (Qt otherwise swallows them).
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    tick = QTimer()
    tick.timeout.connect(lambda: None)
    tick.start(250)

    rc = app.exec_()
    buttons.close()
    link.stop()
    return rc


if __name__ == "__main__":
    sys.exit(main())
