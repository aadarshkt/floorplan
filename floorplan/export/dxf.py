"""Render one or more rooms to DXF (dimensioned CAD, opens in AutoCAD/Shapr3D)."""
from __future__ import annotations

from pathlib import Path

import ezdxf

from floorplan.geometry.room import Room


def render(rooms: list[Room], out: str | Path) -> None:
    doc = ezdxf.new(dxfversion="R2010")
    msp = doc.modelspace()
    doc.layers.add("WALLS", color=7, lineweight=50)
    doc.layers.add("DIMENSIONS", color=1)
    doc.layers.add("OPENINGS", color=3)

    for room in rooms:
        for w in room.walls:
            p1 = (float(w.start[0]), float(w.start[1]))
            p2 = (float(w.end[0]), float(w.end[1]))
            msp.add_line(p1, p2, dxfattribs={"layer": "WALLS", "lineweight": 50})
            try:
                dim = msp.add_aligned_dim(p1=p1, p2=p2, distance=0.3,
                                          dxfattribs={"layer": "DIMENSIONS"})
                dim.render()
            except Exception:
                pass
            for o in w.openings:
                t = o.center_along_wall / (w.length or 1)
                ox = p1[0] + t * (p2[0] - p1[0])
                oy = p1[1] + t * (p2[1] - p1[1])
                color = 1 if o.kind == "door" else 4
                msp.add_circle((ox, oy), 0.05, dxfattribs={"layer": "OPENINGS", "color": color})
                msp.add_text(o.kind, dxfattribs={"layer": "OPENINGS", "height": 0.1,
                                                 "insert": (ox + 0.05, oy + 0.05)})

    doc.saveas(str(out))
