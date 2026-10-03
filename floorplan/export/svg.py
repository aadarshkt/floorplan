"""Render rooms to a dimensioned SVG plan (self-contained, opens in any browser).

Every wall carries the same id as results.json (``r1-w3`` is drawn as ``w3``)
and its length; openings show type and width; each room shows its id, floor
area and ceiling height. That is what lets a tape/laser reading be written down
against the right wall when benchmarking.
"""
from __future__ import annotations

import math
from pathlib import Path

import svgwrite

from floorplan.geometry.room import Room

SCALE = 90          # px per metre
MARGIN = 90
WALL_COLOR = "#1a1a2e"
DIM_COLOR = "#c0392b"
ID_COLOR = "#2c3e50"
DOOR_COLOR = "#e67e22"
WIN_COLOR = "#2e86de"
FONT = "Helvetica Neue, Arial, sans-serif"
ROOM_COLORS = ["#eef1f7", "#e9f4ee", "#f4eef1", "#eef0f7", "#f7f3e8"]


def render(rooms: list[Room], out: str | Path, title: str = "Floor Plan",
           payload: dict | None = None) -> None:
    walls = [w for r in rooms for w in r.walls]
    xs = [w.start[0] for w in walls] + [w.end[0] for w in walls]
    ys = [w.start[1] for w in walls] + [w.end[1] for w in walls]
    if not xs:
        xs, ys = [0.0, 1.0], [0.0, 1.0]
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
    w_px = int((max_x - min_x) * SCALE + 2 * MARGIN) or 400
    h_px = int((max_y - min_y) * SCALE + 2 * MARGIN + 40) or 300

    def to_px(x, y):
        return ((x - min_x) * SCALE + MARGIN, (max_y - y) * SCALE + MARGIN)

    # interval lookup from the payload, keyed by wall id
    ci = {}
    if payload:
        for r in payload.get("rooms", []):
            for w in r.get("walls", []):
                ci[w["wall_id"]] = w["length_m"]

    dwg = svgwrite.Drawing(str(out), size=(f"{w_px}px", f"{h_px}px"))
    dwg.add(dwg.rect(insert=(0, 0), size=("100%", "100%"), fill="#fbfcfe"))
    dwg.add(dwg.text(title, insert=(20, 28), font_size="15px", font_family=FONT,
                     font_weight="bold", fill=WALL_COLOR))
    dwg.add(dwg.text("wN = wall id in results.json · lengths are interior, face to face · "
                     "D/W = door/window width", insert=(20, 48), font_size="11px",
                     font_family=FONT, fill="#555"))

    for ri, room in enumerate(rooms):
        rid = room.room_id or f"r{ri + 1}"
        if room.polygon:
            pts = [to_px(x, y) for x, y in room.polygon]
            dwg.add(dwg.polygon(points=pts, fill=ROOM_COLORS[ri % len(ROOM_COLORS)], stroke="none"))
            cx = sum(p[0] for p in pts) / len(pts)
            cy = sum(p[1] for p in pts) / len(pts)
            ceil = room.ceiling_height_m.value
            label = [f"{rid}", f"{room.floor_area_m2.value:.2f} m²",
                     f"ceiling {ceil:.2f} m" + ("" if room.ceiling_observed else " (not seen)")]
            for k, t in enumerate(label):
                dwg.add(dwg.text(t, insert=(cx, cy + (k - 1) * 15), font_size="12px",
                                 font_family=FONT, fill=ID_COLOR, text_anchor="middle",
                                 font_weight="bold" if k == 0 else "normal"))
        for w in room.walls:
            sx, sy = to_px(*w.start)
            ex, ey = to_px(*w.end)
            dwg.add(dwg.line(start=(sx, sy), end=(ex, ey), stroke=WALL_COLOR,
                             stroke_width=4, stroke_linecap="round"))
            mx, my = (sx + ex) / 2, (sy + ey) / 2
            dx, dy = ex - sx, ey - sy
            nl = math.hypot(dx, dy) or 1
            # label outside the room (walls run CCW, so the right side is outside)
            ox, oy = -dy / nl, dx / nl     # SVG y points down: this is outside
            m = ci.get(f"{rid}-w{w.id}")
            pm = ""
            if m and m.get("ci95"):
                pm = f" ±{(m['ci95'][1] - m['ci95'][0]) / 2 * 100:.0f}cm"
            dwg.add(dwg.text(f"w{w.id}: {w.length:.2f} m{pm}", insert=(mx + ox * 16, my + oy * 16),
                             font_size="11px", font_family=FONT, fill=DIM_COLOR,
                             text_anchor="middle", dominant_baseline="middle"))
            for o in w.openings:
                t0 = (o.center_along_wall - o.width_m / 2) / (w.length or 1)
                t1 = (o.center_along_wall + o.width_m / 2) / (w.length or 1)
                p0 = (sx + t0 * dx, sy + t0 * dy)
                p1 = (sx + t1 * dx, sy + t1 * dy)
                color = DOOR_COLOR if o.kind == "door" else WIN_COLOR
                dwg.add(dwg.line(start=p0, end=p1, stroke=color, stroke_width=7))
                tx, ty = (p0[0] + p1[0]) / 2 - ox * 14, (p0[1] + p1[1]) / 2 - oy * 14
                dwg.add(dwg.text(f"{o.kind[0].upper()} {o.width_m:.2f}", insert=(tx, ty),
                                 font_size="10px", font_family=FONT, fill=color,
                                 text_anchor="middle", dominant_baseline="middle"))

    # 1 m scale bar
    y0 = h_px - 25
    dwg.add(dwg.line(start=(20, y0), end=(20 + SCALE, y0), stroke=WALL_COLOR, stroke_width=3))
    dwg.add(dwg.text("1 m", insert=(20 + SCALE + 8, y0 + 4), font_size="11px",
                     font_family=FONT, fill=WALL_COLOR))
    dwg.save()
