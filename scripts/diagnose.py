"""Diagnostic: inspect planes / walls / floor-ceiling for a fused cloud.

Usage: python scripts/diagnose.py runs/<name>/scan_metric.ply
"""
from __future__ import annotations

import sys

import numpy as np
import open3d as o3d

from floorplan.config import Settings
from floorplan.geometry import planes as P
from floorplan.geometry import room as R
from floorplan.geometry import walls as W


def main() -> None:
    ply = sys.argv[1] if len(sys.argv) > 1 else "runs/single_room/scan_metric.ply"
    cfg = Settings()
    pts = np.asarray(o3d.io.read_point_cloud(ply).points)
    print(f"points={len(pts)} bbox_min={np.round(pts.min(0),2)} bbox_max={np.round(pts.max(0),2)}")
    pls = P.extract_planes(pts, cfg)
    horiz = [p for p in pls if p.kind == "horizontal"]
    vert = [p for p in pls if p.kind == "vertical"]
    print(f"planes={len(pls)}  horizontal={len(horiz)} vertical={len(vert)}")
    fz, cz, obs, fpts, cpts = R.floor_and_ceiling(pls, pts, cfg)
    print(f"floor_z={fz:.3f} ceil_z={cz:.3f} observed={obs} "
          f"height={cz - fz:.3f} (floor_pts={len(fpts)} ceil_pts={len(cpts)})")
    for p in sorted(vert, key=lambda p: -len(p.points))[:12]:
        print(f"  vertical plane {p.id:2d} n={len(p.points):8d} zmed={p.z_median:+.2f} "
              f"nrm={np.round(p.normal,2)}")
    walls = W.vectorize(vert, fz, cfg)
    walls = W.merge_double_walls(walls, cfg)
    walls = W.snap_orthogonal(walls, cfg)
    print(f"walls={len(walls)}")
    for w in walls:
        print(f"  wall {w.id} len={w.length:5.2f} start={np.round(w.start,2)} "
              f"end={np.round(w.end,2)} ang={np.degrees(w.angle):6.1f}")


if __name__ == "__main__":
    main()
