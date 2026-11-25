"""Entry point: python -m dpv_dash [--sim] [--windowed] [--conn URL] [--test-buttons] [--check]"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import tomllib
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config.toml"


def use_local_display() -> None:
    """When started from SSH (no display variables), draw on the Pi's own screen.

    The desktop is a Wayland session (labwc) whose socket lives in
    /run/user/<uid>/. Without this, Qt falls back to X11 and fails with
    'could not connect to display'.
    """
    if os.environ.get("WAYLAND_DISPLAY") or os.environ.get("DISPLAY"):
        os.environ.setdefault("QT_QPA_PLATFORM", "wayland" if os.environ.get("WAYLAND_DISPLAY") else "xcb")
        return
    runtime = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{os.getuid()}"
    sockets = sorted(Path(runtime).glob("wayland-[0-9]")) if Path(runtime).is_dir() else []
    if not sockets:
        sys.exit("No desktop session found on this Pi's screen (is the desktop logged in?).\n"
                 f"Looked for a Wayland socket in {runtime}.")
    os.environ["XDG_RUNTIME_DIR"] = runtime
    os.environ["WAYLAND_DISPLAY"] = sockets[0].name
    os.environ["QT_QPA_PLATFORM"] = "wayland"


def main() -> int:
    ap = argparse.ArgumentParser(prog="dpv_dash", description="DPV driver HUD")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    ap.add_argument("--sim", action="store_true", help="fake vehicle instead of the Pixhawk")
    ap.add_argument("--windowed", action="store_true", help="normal window instead of full screen")
    ap.add_argument("--conn", help="override [mavlink] connection, e.g. udpin:0.0.0.0:14550")
    ap.add_argument("--test-buttons", action="store_true", help="print GPIO button edges and exit")
    ap.add_argument("--check", action="store_true",
                    help="no HUD; print what the Pixhawk really sends (rates, raw values, params) and exit")
    ap.add_argument("--seconds", type=float, default=10.0, help="listening time for --check")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s")
    with open(args.config, "rb") as f:
        cfg = tomllib.load(f)

    if args.check:
        from .check import run_check
        mcfg = dict(cfg["mavlink"])
        if args.conn:
            mcfg["connection"] = args.conn
        return run_check(mcfg, args.seconds)

    if args.test_buttons:
        from .controls import test_buttons
        test_buttons(cfg["buttons"])
        return 0

    use_local_display()
    from PyQt5.QtCore import QTimer
    from PyQt5.QtWidgets import QApplication

    from .blackbox import Blackbox
    from .controller import Controller
    from .controls import GpioButtons
    from .hud import Hud
    from .throttle import ThrottleSender

    bbcfg = cfg.get("blackbox", {})
    bb = None
    if bbcfg.get("enabled", True):
        try:
            bb = Blackbox(bbcfg, cfg, sim=args.sim)
        except OSError as e:  # never let logging stop the dashboard
            logging.getLogger("dpv_dash").error("Blackbox disabled: %s", e)

    if args.sim:
        from .sim import SimLink
        link = SimLink(cfg.get("battery", {}))
    else:
        from .mavlink_link import MavlinkLink
        mcfg = dict(cfg["mavlink"])
        if args.conn:
            mcfg["connection"] = args.conn
        link = MavlinkLink(mcfg, cfg.get("battery", {}), blackbox=bb)

    app = QApplication(sys.argv)
    app.setApplicationName("dpv_dash")
    link.start()

    ctl = Controller(link, cfg)
    throttle = ThrottleSender(link, ctl.ui, cfg.get("throttle", {}))
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

    if bb is not None:  # record what the driver sees, csv_rate_hz times a second
        rec = QTimer()
        rec.timeout.connect(lambda: bb.write_row(link.snapshot(), ctl.ui, throttle.last_pwm))
        rec.start(int(1000 / bbcfg.get("csv_rate_hz", 5)))

    rc = app.exec_()
    buttons.close()
    link.stop()
    if bb is not None:
        bb.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
