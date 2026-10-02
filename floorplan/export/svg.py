"""Render one or more rooms to SVG (self-contained, opens in any browser)."""
from __future__ import annotations

import math
from pathlib import Path

import svgwrite

from floorplan.geometry.room import Room

SCALE = 90          # px per metre
MARGIN = 70
WALL_COLOR = "#1a1a2e"
DIM_COLOR = "#e94560"
DOOR_COLOR = "#f5a623"
WIN_COLOR = "#4ab8f5"
FILL = "#eef1f7"
FONT = "Inter, Helvetica Neue, Arial, sans-serif"
ROOM_COLORS = ["#eef1f7", "#e9f4ee", "#f4eef1", "#eef0f7", "#f7f3e8"]


def render(rooms: list[Room], out: str | Path, title: str = "Floor Plan") -> None:
    walls = [w for r in rooms for w in r.walls]
    xs = [w.start[0] for w in walls] + [w.end[0] for w in walls]
    ys = [w.start[1] for w in walls] + [w.end[1] for w in walls]
    if not xs:
        xs, ys = [0.0, 1.0], [0.0, 1.0]
    min_x, max_x, min_y, max_y = min(xs), max(xs), min(ys), max(ys)
    w_px = int((max_x - min_x) * SCALE + 2 * MARGIN) or 400
    h_px = int((max_y - min_y) * SCALE + 2 * MARGIN) or 300

    def to_px(x, y):
        return ((x - min_x) * SCALE + MARGIN, (max_y - y) * SCALE + MARGIN)

    dwg = svgwrite.Drawing(str(out), size=(f"{w_px}px", f"{h_px}px"))
    dwg.add(dwg.rect(insert=(0, 0), size=("100%", "100%"), fill="#fbfcfe"))
    dwg.add(dwg.text(title, insert=(MARGIN, 34), font_size="15px", font_family=FONT,
                     font_weight="bold", fill=WALL_COLOR))

    for ri, room in enumerate(rooms):
        if room.polygon:
            pts = [to_px(x, y) for x, y in room.polygon]
            dwg.add(dwg.polygon(points=pts, fill=ROOM_COLORS[ri % len(ROOM_COLORS)],
                                stroke="none"))
        for w in room.walls:
            sx, sy = to_px(*w.start)
            ex, ey = to_px(*w.end)
            dwg.add(dwg.line(start=(sx, sy), end=(ex, ey), stroke=WALL_COLOR,
                             stroke_width=5, stroke_linecap="round"))
            mx, my = (sx + ex) / 2, (sy + ey) / 2
            dx, dy = ex - sx, ey - sy
            nl = math.hypot(dx, dy) or 1
            lx, ly = mx + (-dy / nl) * 16, my + (dx / nl) * 16
            dwg.add(dwg.text(f"{w.length:.2f} m", insert=(lx, ly), font_size="12px",
                             font_family=FONT, fill=DIM_COLOR, text_anchor="middle",
                             dominant_baseline="middle"))
            for o in w.openings:
                t = o.center_along_wall / (w.length or 1)
                ox = sx + t * (ex - sx)
                oy = sy + t * (ey - sy)
                color = DOOR_COLOR if o.kind == "door" else WIN_COLOR
                dwg.add(dwg.circle(center=(ox, oy), r=7, fill=color, opacity=0.9))
                dwg.add(dwg.text(o.kind[0].upper(), insert=(ox, oy + 4), font_size="9px",
                                 font_family=FONT, fill="white", text_anchor="middle"))

    dwg.save()
