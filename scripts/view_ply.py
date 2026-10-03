"""Open a run's 3D output in an interactive window (Open3D).

    .venv/bin/python scripts/view_ply.py benchmark/runs/lidar/benchmark_2          # mesh if present, else cloud
    .venv/bin/python scripts/view_ply.py benchmark/runs/lidar/benchmark_2 --top    # look straight down
    .venv/bin/python scripts/view_ply.py some/scan_metric.ply

Idea from the lidar-optimized branch's viewer, without its Linux/Mesa GPU
selection. Mouse: drag to rotate, scroll to zoom, shift-drag to pan; Q quits.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", type=Path, help="run directory, scan_metric.ply or scan_mesh.ply")
    ap.add_argument("--top", action="store_true", help="start with a top-down view")
    ap.add_argument("--max-points", type=int, default=1_500_000)
    a = ap.parse_args()

    p = a.path
    if p.is_dir():
        p = p / "scan_mesh.ply" if (p / "scan_mesh.ply").exists() else p / "scan_metric.ply"
    mesh = o3d.io.read_triangle_mesh(str(p))
    if len(mesh.triangles):
        mesh.compute_vertex_normals()
        geom = mesh
    else:
        pcd = o3d.io.read_point_cloud(str(p))
        if len(pcd.points) > a.max_points:
            pcd = pcd.random_down_sample(a.max_points / len(pcd.points))
        if not pcd.has_colors():   # colour by height so walls/floor/ceiling read apart
            z = np.asarray(pcd.points)[:, 2]
            t = (z - np.percentile(z, 1)) / max(np.ptp(z), 1e-6)
            pcd.colors = o3d.utility.Vector3dVector(np.stack([t, 0.4 + 0 * t, 1 - t], axis=1).clip(0, 1))
        geom = pcd
    print(f"{p}: {len(getattr(geom, 'triangles', [])) or len(geom.points)} "
          f"{'triangles' if len(getattr(geom, 'triangles', [])) else 'points'}")
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=str(p), width=1280, height=900)
    vis.add_geometry(geom)
    if a.top:
        ctr = vis.get_view_control()
        ctr.set_front([0.0, 0.0, 1.0])
        ctr.set_up([0.0, 1.0, 0.0])
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
