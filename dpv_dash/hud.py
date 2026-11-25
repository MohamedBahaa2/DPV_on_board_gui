"""The HUD: one QWidget painted with QPainter at 30 fps.

Everything is drawn in a fixed 800x480 logical canvas (the LCD's native size)
and scaled to fit the window, so --windowed on a desktop looks the same.

The HUD never invents a value: whatever the trusted snapshot has as `None`
(stale, missing, implausible) is drawn as grey dashes.
"""
from __future__ import annotations

import math
import time
from typing import NamedTuple

from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPainterPath, QPen, QPolygonF
from PyQt5.QtWidgets import QApplication, QWidget

from .state import UiState, VehicleState, wrap180

W, H = 800, 480

BG = QColor(3, 7, 9)
PANEL = QColor(10, 20, 18)
GREEN = QColor(60, 255, 110)
GREEN_DIM = QColor(40, 140, 75)
CYAN = QColor(80, 220, 255)
MAGENTA = QColor(255, 92, 240)
AMBER = QColor(255, 176, 32)
RED = QColor(255, 64, 64)
WHITE = QColor(225, 240, 232)
GREY = QColor(90, 110, 100)

# Speed units: factor from m/s, label, pixels per unit on the tape, label step.
UNITS = {
    "kn": (1.943844, "KN", 30.0, 1.0),
    "m/s": (1.0, "M/S", 50.0, 0.5),
    "km/h": (3.6, "KM/H", 15.0, 2.0),
}

MODE_COLORS = {"MANUAL": GREEN, "AUTO": MAGENTA, "RTL": AMBER, "SMART_RTL": AMBER, "HOLD": AMBER}

# Keyboard fallback for desk testing: key -> button name.
KEYMAP = {Qt.Key_M: "mode", Qt.Key_S: "speed", Qt.Key_Space: "trigger", Qt.Key_G: "guide"}

FONT_FAMILY = "DejaVu Sans Mono"
ARRIVED_M = 3.0  # closer than this: bearing is meaningless, show "AT HOME" / "AT WP n"


class Guide(NamedTuple):
    label: str
    color: QColor
    bearing: float | None
    distance: float | None
    coords: tuple[float, float] | None


def fmt_dist(m: float) -> str:
    return f"{m:.0f} m" if m < 1000 else f"{m / 1000:.2f} km"


def fmt_coord(lat: float, lon: float) -> str:
    return f"{abs(lat):.5f}{'N' if lat >= 0 else 'S'}  {abs(lon):.5f}{'E' if lon >= 0 else 'W'}"


class Hud(QWidget):
    def __init__(self, link, controller, display_cfg: dict):
        super().__init__()
        self.link, self.ctl = link, controller
        self.units = display_cfg.get("speed_units", "kn")
        if self.units not in UNITS:
            self.units = "kn"
        self.setWindowTitle("DPV HUD")
        self.setCursor(Qt.BlankCursor)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setAttribute(Qt.WA_OpaquePaintEvent)
        self._fonts: dict[tuple[int, bool], QFont] = {}
        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(33)

    # ------------------------------------------------------------ input
    def keyPressEvent(self, e):
        if e.isAutoRepeat():
            return
        if e.key() == Qt.Key_Q and e.modifiers() & Qt.ControlModifier:
            QApplication.quit()
            return
        if e.key() in KEYMAP:
            self.ctl.on_press(KEYMAP[e.key()])

    def keyReleaseEvent(self, e):
        if not e.isAutoRepeat() and e.key() in KEYMAP:
            self.ctl.on_release(KEYMAP[e.key()])

    # ------------------------------------------------------------ helpers
    def font(self, px: int, bold: bool = True) -> QFont:
        key = (px, bold)
        if key not in self._fonts:
            f = QFont(FONT_FAMILY)
            f.setPixelSize(px)
            f.setBold(bold)
            f.setStyleHint(QFont.Monospace)
            self._fonts[key] = f
        return self._fonts[key]

    def text(self, p: QPainter, rect, s: str, color: QColor, px: int,
             align=Qt.AlignCenter, bold: bool = True) -> None:
        p.setFont(self.font(px, bold))
        p.setPen(color)
        p.drawText(QRectF(*rect), int(align), s)

    @staticmethod
    def box(p: QPainter, rect, border: QColor, fill: QColor = PANEL, width: float = 2) -> None:
        p.setPen(QPen(border, width))
        p.setBrush(fill)
        p.drawRoundedRect(QRectF(*rect), 6, 6)
        p.setBrush(Qt.NoBrush)

    @staticmethod
    def flash(hz: float = 2.0) -> bool:
        return int(time.monotonic() * hz * 2) % 2 == 0

    def guidance(self, s: VehicleState, ui: UiState) -> Guide:
        if ui.guide_source == "HOME":
            coords = (s.home_lat, s.home_lon) if s.home_lat is not None else None
            bd = s.home_bearing_distance()
            return Guide("HOME", CYAN, bd[0] if bd else None, bd[1] if bd else None, coords)
        t = s.target_bearing_distance()
        if t is None:
            return Guide("WP", MAGENTA, None, None, None)
        seq, brg, dist = t
        return Guide(f"WP {seq}" if seq is not None else "WP", MAGENTA, brg, dist, s.mission.get(seq))

    # ------------------------------------------------------------ paint
    def paintEvent(self, _ev):
        s = self.link.snapshot()
        ui = self.ctl.ui
        g = self.guidance(s, ui)
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.setRenderHint(QPainter.TextAntialiasing)
        p.fillRect(self.rect(), BG)
        scale = min(self.width() / W, self.height() / H)
        p.translate((self.width() - W * scale) / 2, (self.height() - H * scale) / 2)
        p.scale(scale, scale)

        self.draw_horizon(p, s)
        self.draw_heading_tape(p, s, g)
        self.draw_speed_tape(p, s, ui)
        self.draw_battery(p, s)
        if s.link_ok:
            self.draw_steer_cue(p, s, g)
        else:
            self.draw_no_link(p)
        self.draw_guide_box(p, s, ui, g)
        self.draw_statustext(p, s)
        self.draw_bottom(p, s, ui)
        p.end()

    # -- attitude: faint horizon behind the centre area
    def draw_horizon(self, p, s):
        if s.roll is None or s.pitch is None:
            return
        p.save()
        p.setClipRect(QRectF(110, 72, 580, 330))
        p.translate(400, 210)
        p.rotate(-math.degrees(s.roll))
        y = math.degrees(s.pitch) * 4.0
        c = QColor(GREEN_DIM)
        c.setAlpha(110)
        p.setPen(QPen(c, 2))
        p.drawLine(QPointF(-300, y), QPointF(-70, y))
        p.drawLine(QPointF(70, y), QPointF(300, y))
        for deg in (-10, 10):  # short pitch ladder rungs
            yy = y - deg * 4.0
            p.drawLine(QPointF(-40, yy), QPointF(40, yy))
        p.restore()

    # -- heading tape across the top, with guidance bug
    def draw_heading_tape(self, p, s, g: Guide):
        x0, x1, y0, y1, cx, ppd = 110.0, 690.0, 0.0, 64.0, 400.0, 4.5
        if s.heading is None:  # no (fresh) heading: empty tape, dashes
            p.fillRect(QRectF(x0, y0, x1 - x0, y1 - y0), PANEL)
            p.setPen(QPen(GREEN_DIM, 1))
            p.drawRect(QRectF(x0, y0, x1 - x0, y1 - y0))
            self.box(p, (cx - 46, 0, 92, 38), GREY, BG)
            self.text(p, (cx - 46, 0, 92, 38), "---°", GREY, 28)
            return
        hdg = s.heading % 360
        p.save()
        p.setClipRect(QRectF(x0, y0, x1 - x0, y1 - y0))
        p.fillRect(QRectF(x0, y0, x1 - x0, y1 - y0), PANEL)
        lo = hdg - (cx - x0) / ppd
        hi = hdg + (x1 - cx) / ppd
        p.setPen(QPen(GREEN, 2))
        for d in range(math.floor(lo / 5) * 5, math.ceil(hi) + 1, 5):
            x = cx + (d - hdg) * ppd
            dd = d % 360
            p.setPen(QPen(GREEN, 2))
            p.drawLine(QPointF(x, y1), QPointF(x, y1 - (14 if dd % 10 == 0 else 7)))
            if dd % 30 == 0:
                lbl = {0: "N", 90: "E", 180: "S", 270: "W"}.get(dd, f"{dd:03d}")
                self.text(p, (x - 30, 26, 60, 22), lbl, WHITE if len(lbl) == 1 else GREEN, 18)
        # guidance bug
        if g.bearing is not None:
            err = wrap180(g.bearing - hdg)
            bx = cx + err * ppd
            p.setPen(QPen(g.color, 3))
            if x0 + 14 <= bx <= x1 - 14:
                bug = QPolygonF([QPointF(bx - 13, y1), QPointF(bx - 13, y1 - 14), QPointF(bx - 5, y1 - 14),
                                 QPointF(bx, y1 - 7), QPointF(bx + 5, y1 - 14), QPointF(bx + 13, y1 - 14),
                                 QPointF(bx + 13, y1)])
                p.drawPolyline(bug)
            else:  # off-scale: arrow at the tape edge
                d = 1 if err > 0 else -1
                ex = x1 - 8 if d > 0 else x0 + 8
                p.setBrush(g.color)
                p.drawPolygon(QPolygonF([QPointF(ex, y1 - 10), QPointF(ex - d * 16, y1 - 20),
                                         QPointF(ex - d * 16, y1)]))
                p.setBrush(Qt.NoBrush)
        p.restore()
        p.setPen(QPen(GREEN_DIM, 1))
        p.drawRect(QRectF(x0, y0, x1 - x0, y1 - y0))
        # current heading readout (amber + reason when the AHRS can't vouch for it)
        hc = AMBER if s.heading_warn else GREEN
        self.box(p, (cx - 46, 0, 92, 38), hc, BG)
        self.text(p, (cx - 46, 0, 92, 38), f"{round(hdg) % 360:03d}°", hc, 28)
        if s.heading_warn:
            self.text(p, (cx + 48, 2, 60, 18), s.heading_warn, AMBER, 14, Qt.AlignLeft | Qt.AlignVCenter)
        p.setPen(Qt.NoPen)
        p.setBrush(hc)
        p.drawPolygon(QPolygonF([QPointF(cx - 7, 38), QPointF(cx + 7, 38), QPointF(cx, 47)]))
        p.setBrush(Qt.NoBrush)

    # -- speed tape on the left
    def draw_speed_tape(self, p, s, ui: UiState):
        factor, label, ppu, step = UNITS[self.units]
        x, y, w, h = 0.0, 72.0, 104.0, 330.0
        cy = y + h / 2
        known = s.groundspeed is not None
        v = s.groundspeed * factor if known else 0.0
        p.save()
        p.setClipRect(QRectF(x, y, w, h))
        p.fillRect(QRectF(x, y, w, h), PANEL)
        minor = step / 2
        i0 = math.floor((v - h / 2 / ppu) / minor)
        i1 = math.ceil((v + h / 2 / ppu) / minor)
        for i in (range(max(0, i0), i1 + 1) if known else ()):
            val = i * minor
            ty = cy - (val - v) * ppu
            major = i % 2 == 0
            p.setPen(QPen(GREEN, 2))
            p.drawLine(QPointF(w - 4 - (16 if major else 8), ty), QPointF(w - 4, ty))
            if major:
                self.text(p, (x, ty - 11, w - 26, 22), f"{val:g}", GREEN, 18, Qt.AlignRight | Qt.AlignVCenter)
        # target-speed bug in AUTO/RTL (level% of WP_SPEED)
        if known and s.mode in ("AUTO", "RTL") and s.cruise_speed:
            tv = s.cruise_speed * ui.speed_pct / 100.0 * factor
            ty = cy - (tv - v) * ppu
            p.setPen(QPen(MAGENTA, 3))
            p.drawPolyline(QPolygonF([QPointF(w - 2, ty - 10), QPointF(w - 14, ty - 10), QPointF(w - 14, ty - 4),
                                      QPointF(w - 8, ty), QPointF(w - 14, ty + 4), QPointF(w - 14, ty + 10),
                                      QPointF(w - 2, ty + 10)]))
        p.fillRect(QRectF(x, y, w, 24), PANEL)
        p.setPen(QPen(GREEN_DIM, 1))
        p.drawLine(QPointF(x, y + 24), QPointF(x + w, y + 24))
        self.text(p, (x, y + 3, w, 20), f"GS {label}", GREEN_DIM, 14)
        p.restore()
        p.setPen(QPen(GREEN_DIM, 1))
        p.drawRect(QRectF(x, y, w, h))
        # readout box with pointer
        path = QPainterPath()
        path.moveTo(2, cy - 24)
        path.lineTo(84, cy - 24)
        path.lineTo(84, cy - 9)
        path.lineTo(96, cy)
        path.lineTo(84, cy + 9)
        path.lineTo(84, cy + 24)
        path.lineTo(2, cy + 24)
        path.closeSubpath()
        col = GREEN if known else GREY
        p.setPen(QPen(col, 2))
        p.setBrush(BG)
        p.drawPath(path)
        p.setBrush(Qt.NoBrush)
        self.text(p, (2, cy - 24, 82, 48), f"{v:.1f}" if known else "--.-", col, 32)
        if not known and s.link_ok:
            self.text(p, (x, cy + 30, w, 20), "NEEDS GPS", GREY, 13, bold=False)

    # -- battery column on the right
    def draw_battery(self, p, s):
        x, y, w, h = 696.0, 72.0, 104.0, 330.0
        self.box(p, (x, y, w, h), GREEN_DIM)
        self.text(p, (x, y + 2, w, 20), "BATT", GREEN_DIM, 14)
        pct = s.batt_remaining
        col = GREY if pct is None else GREEN if pct > 40 else AMBER if pct > 20 else RED
        if pct is not None and pct <= 20 and not self.flash(1.0):
            col = QColor(col.red() // 2, col.green() // 2, col.blue() // 2)
        bx, by, bw, bh = x + 30, y + 34, 44.0, 190.0
        p.setPen(QPen(col, 2))
        p.drawRect(QRectF(bx, by, bw, bh))
        p.fillRect(QRectF(bx + 14, by - 6, bw - 28, 6), col)  # terminal nub
        if pct is not None:
            fh = (bh - 6) * max(0, min(100, pct)) / 100.0
            p.fillRect(QRectF(bx + 3, by + bh - 3 - fh, bw - 6, fh), col)
        if pct is not None:
            self.text(p, (x, by + bh + 4, w, 36), f"{pct}%", col, 30)
        else:  # "NO BATT" / "NO MON" / "--": say why there is no number
            self.text(p, (x, by + bh + 4, w, 36), "--%" if s.batt_status in ("--", "OK") else s.batt_status,
                      GREY, 30 if s.batt_status in ("--", "OK") else 20)
        vtxt = "--.-V" if s.batt_voltage is None else f"{s.batt_voltage:.1f}V"
        atxt = "--.-A" if s.batt_current is None else f"{s.batt_current:.1f}A"
        self.text(p, (x, by + bh + 42, w, 24), vtxt, WHITE, 20)
        self.text(p, (x, by + bh + 68, w, 24), atxt, WHITE, 20)

    # -- big steering cue in the middle
    def draw_steer_cue(self, p, s, g: Guide):
        cx, cy = 400.0, 168.0
        if g.bearing is None:
            if s.lat is None and (g.label == "HOME" or not s.mission):
                msg = "NO GPS POSITION"
            else:
                msg = "NO HOME POSITION" if g.label == "HOME" else "NO ACTIVE WAYPOINT"
            self.text(p, (150, cy - 20, 500, 40), msg, GREY, 22)
            return
        if g.distance is not None and g.distance < ARRIVED_M:
            self.text(p, (150, cy - 26, 500, 52), f"AT {g.label}", g.color, 40)
            return
        if s.heading is None:
            self.text(p, (150, cy - 20, 500, 40), "NO HEADING", GREY, 22)
            return
        err = wrap180(g.bearing - s.heading)
        if abs(err) < 5:
            p.setPen(QPen(g.color, 7, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            p.drawPolyline(QPolygonF([QPointF(cx - 34, cy + 6), QPointF(cx, cy - 26), QPointF(cx + 34, cy + 6)]))
            self.text(p, (cx - 150, cy + 18, 300, 34), "ON COURSE", g.color, 26)
            return
        d = 1 if err > 0 else -1
        n = 1 if abs(err) < 20 else 2 if abs(err) < 60 else 3
        p.setPen(QPen(g.color, 8, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        for i in range(n):
            x = cx + d * (110 + i * 42)
            p.drawPolyline(QPolygonF([QPointF(x - d * 14, cy - 30), QPointF(x + d * 14, cy),
                                      QPointF(x - d * 14, cy + 30)]))
        self.text(p, (cx - 90, cy - 30, 180, 50), f"{abs(err):.0f}°", g.color, 44)
        self.text(p, (cx - 90, cy + 20, 180, 24), "TURN RIGHT" if d > 0 else "TURN LEFT", g.color, 18)

    def draw_no_link(self, p):
        on = self.flash(1.5)
        self.box(p, (180, 112, 440, 110), RED if on else QColor(120, 30, 30), BG, 4)
        self.text(p, (180, 118, 440, 60), "NO LINK", RED if on else QColor(160, 40, 40), 52)
        self.text(p, (180, 176, 440, 34), "no heartbeat from Pixhawk", WHITE, 18, bold=False)

    # -- guidance box: HOME or WP target
    def draw_guide_box(self, p, s, ui: UiState, g: Guide):
        x, y, w, h = 186.0, 262.0, 428.0, 118.0
        self.box(p, (x, y, w, h), g.color)
        self.text(p, (x + 12, y + 6, 140, 36), g.label, g.color, 28, Qt.AlignLeft | Qt.AlignVCenter)
        brg = "---°" if g.bearing is None else f"{round(g.bearing) % 360:03d}°"
        self.text(p, (x + 140, y + 6, 120, 36), brg, WHITE, 28)
        dist = "----" if g.distance is None else fmt_dist(g.distance)
        self.text(p, (x + 250, y + 6, w - 262, 36), dist, WHITE, 28, Qt.AlignRight | Qt.AlignVCenter)
        coords = fmt_coord(*g.coords) if g.coords else "position unknown"
        self.text(p, (x + 12, y + 46, w - 24, 26), coords, WHITE if g.coords else GREY, 19,
                  Qt.AlignLeft | Qt.AlignVCenter, bold=False)
        # ETA, and which guidance source is selected (GUIDE button toggles)
        if g.distance is not None and s.groundspeed is not None and s.groundspeed > 0.2:
            eta = int(g.distance / s.groundspeed)
            eta_txt = f"ETA {eta // 60:d}:{eta % 60:02d}" if eta < 6000 else "ETA --:--"
        else:
            eta_txt = "ETA --:--"
        self.text(p, (x + 12, y + 80, 180, 28), eta_txt, GREEN, 20, Qt.AlignLeft | Qt.AlignVCenter)
        for i, (src, lbl, col) in enumerate((("HOME", "HOME", CYAN), ("TARGET", "WP", MAGENTA))):
            r = QRectF(x + w - 172 + i * 82, y + 80, 76, 28)
            if ui.guide_source == src:
                p.fillRect(r, col)
                self.text(p, r.getRect(), lbl, BG, 18)
            else:
                p.setPen(QPen(GREY, 1))
                p.drawRect(r)
                self.text(p, r.getRect(), lbl, GREY, 18)

    def draw_statustext(self, p, s):
        if not s.statustext or time.monotonic() - s.statustext_time > 10:
            return
        col = RED if s.statustext_severity <= 3 else AMBER if s.statustext_severity == 4 else GREEN
        self.text(p, (112, 74, 576, 24), s.statustext[:52], col, 18, bold=False)

    # -- annunciator strip along the bottom
    def draw_bottom(self, p, s, ui: UiState):
        y, h = 408.0, 68.0
        # MODE
        if ui.pending_mode:
            on = self.flash(2.0)
            self.box(p, (4, y, 196, h), AMBER if on else GREY, BG, 3)
            self.text(p, (12, y + 2, 180, 16), f"MODE  {s.mode} ▸", AMBER, 13, Qt.AlignLeft, bold=False)
            self.text(p, (4, y + 18, 196, h - 20), ui.pending_mode, AMBER if on else GREY, 30)
        else:
            col = MODE_COLORS.get(s.mode, WHITE) if s.link_ok else GREY
            self.box(p, (4, y, 196, h), col)
            self.text(p, (12, y + 2, 180, 16), "MODE", GREEN_DIM, 13, Qt.AlignLeft, bold=False)
            self.text(p, (4, y + 18, 196, h - 20), s.mode, col, 30)
        # SPEED level
        self.box(p, (206, y, 196, h), CYAN)
        sub = ""
        if s.mode in ("AUTO", "RTL") and s.cruise_speed:
            factor, label, *_ = UNITS[self.units]
            sub = f"  {s.cruise_speed * ui.speed_pct / 100 * factor:.1f}{label.lower()}"
        self.text(p, (214, y + 2, 180, 16), "SPEED SET" + sub, GREEN_DIM, 13, Qt.AlignLeft, bold=False)
        n = len(ui.speed_levels)
        segw = min(26.0, (110 - 4 * (n - 1)) / n)
        for i in range(n):
            r = QRectF(216 + i * (segw + 4), y + 28, segw, 30)
            if i <= ui.speed_index:
                p.fillRect(r, CYAN)
            else:
                p.setPen(QPen(GREY, 1))
                p.drawRect(r)
        self.text(p, (330, y + 20, 68, 44), f"{ui.speed_pct}%", CYAN, 26, Qt.AlignRight | Qt.AlignVCenter)
        # small status boxes
        if s.armed is None:
            motor = ("MOTOR", "--", GREY, False)
        else:
            motor = ("MOTOR", "ARMED" if s.armed else "SAFE", RED if s.armed else GREEN, False)
        thr = "--" if s.throttle_pct is None else f"{s.throttle_pct}%"
        boxes = [motor, ("THR", thr, BG if ui.trigger_held else GREEN if s.throttle_pct is not None else GREY,
                         ui.trigger_held)]
        sats = "" if s.gps_sats is None else f" {s.gps_sats}"
        if s.gps_fix is None:
            boxes.append(("GPS", "--", GREY, False))
        elif s.gps_fix >= 3:
            boxes.append(("GPS", f"{'3D' if s.gps_fix == 3 else 'RTK' if s.gps_fix >= 5 else 'DGPS'}{sats}", GREEN, False))
        elif s.gps_fix == 2:
            boxes.append(("GPS", f"2D{sats}", AMBER, False))
        elif s.gps_fix == 1:
            boxes.append(("GPS", "NO FIX", RED, False))
        else:
            boxes.append(("GPS", "NO GPS", RED, False))
        boxes.append(("LINK", "OK" if s.link_ok else "LOST", GREEN if s.link_ok else RED, False))
        bx = 408.0
        bw = (W - bx) / len(boxes)
        for (lbl, val, col, filled) in boxes:
            if filled:  # trigger held: lit box
                p.setPen(Qt.NoPen)
                p.setBrush(GREEN)
                p.drawRoundedRect(QRectF(bx, y, bw - 4, h), 6, 6)
                p.setBrush(Qt.NoBrush)
            else:
                self.box(p, (bx, y, bw - 4, h), col if col is not GREEN else GREEN_DIM)
            self.text(p, (bx, y + 4, bw - 4, 16), lbl, BG if filled else GREEN_DIM, 13, bold=False)
            self.text(p, (bx, y + 22, bw - 4, 40), val, col, 22)
            bx += bw
