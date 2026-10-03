"""Top-down debug render of the free-space layout for a finished run.

    .venv/bin/python scripts/layout_debug.py <run_dir> <out.png>

grey: all points, black: wall evidence, blue: cameras, red: room polygons.
"""
import json
import sys

import numpy as np
import open3d as o3d
from PIL import Image, ImageDraw

from floorplan.config import Settings
from floorplan.geometry import layout as L
from floorplan.geometry import planes as P
from floorplan.geometry import room as Rm
from floorplan.geometry import walls as W

run, out = sys.argv[1], sys.argv[2]
cfg = Settings()
pts = np.asarray(o3d.io.read_point_cloud(run + "/scan_metric.ply").points)
cams = np.asarray(json.load(open(run + "/cameras.json"))["centre"])
pls = P.extract_planes(pts, cfg)
fz, cz, obs, fp, cp = Rm.floor_and_ceiling(pls, pts, cfg, cam_z=float(np.median(cams[:, 2])))
walls = W.snap_orthogonal(W.merge_double_walls(
    W.vectorize([p for p in pls if p.kind == "vertical"], fz, cfg), cfg), cfg)
theta = max(walls, key=lambda w: w.length).angle if walls else 0.0
band = L._band(pts, fz, cz, obs)
polys = L.rooms(pts, fz, cz, obs, cams[:, :2], theta, cfg)
print(f"floor {fz:.2f} ceil {cz:.2f} observed={obs} height {cz - fz:.2f}")
for p in polys:
    e = np.linalg.norm(np.roll(p, -1, 0) - p, axis=1)
    print(f"  room area {Rm.shoelace(p):.2f} m2, walls {np.round(e, 2).tolist()}")
lo = pts[:, :2].min(0) - 0.3
s = 100
H = int((pts[:, 1].max() - lo[1] + 0.3) * s)
Wd = int((pts[:, 0].max() - lo[0] + 0.3) * s)
img = Image.new("RGB", (Wd, H), "white")
dr = ImageDraw.Draw(img)


def px(xy):
    return [((x - lo[0]) * s, H - (y - lo[1]) * s) for x, y in xy]


for q in px(pts[::20, :2]):
    dr.point(q, fill=(205, 205, 205))
for q in px(band[:, :2]):
    dr.point(q, fill=(0, 0, 0))
for q in px(cams[::10, :2]):
    dr.point(q, fill=(0, 0, 255))
for p in polys:
    dr.line(px(np.vstack([p, p[:1]])), fill=(255, 0, 0), width=3)
img.save(out)
