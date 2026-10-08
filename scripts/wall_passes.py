"""Is a wall error pose drift or the room? Compare wall faces across passes of one walk.

For each edge of a run's room polygon, prints the outward offset (cm) of the densest
wall layer (0.9-2.35 m above the floor) in 0.3 m steps along the edge: once for the
whole cloud, then for each time slice of the walk fused on its own. A wall placed
differently by different slices is drift; a slope every slice agrees on is the room.

    python scripts/wall_passes.py <capture> <run_dir> [--parts 4] [--room 0]
"""
from __future__ import annotations

import argparse
import json
import math

import numpy as np

from floorplan import drift
from floorplan.config import Settings
from floorplan.fusion import depth_fusion as df
from floorplan.geometry import align
from floorplan.ingest import record3d


def profile(P: np.ndarray, poly: np.ndarray, k: int, floor_z: float) -> str:
    c = poly.mean(axis=0)
    a, b = poly[k], poly[(k + 1) % len(poly)]
    t = b - a
    L = float(np.linalg.norm(t))
    t /= L
    nrm = np.array([-t[1], t[0]])
    if np.dot(c - a, nrm) < 0:
        nrm = -nrm                                   # inward normal
    rel = P[:, :2] - a
    s, off, z = rel @ t, -(rel @ nrm), P[:, 2]       # off > 0: outward
    band = (z - floor_z > 0.9) & (z - floor_z < 2.35) & (off > -0.1) & (off < 0.3)
    row = []
    for s0 in np.arange(0.1, L - 0.1, 0.3):
        m = band & (s > s0) & (s < s0 + 0.3)
        if m.sum() < 100:
            row.append(" --")
            continue
        h, e = np.histogram(off[m], bins=np.arange(-0.10, 0.31, 0.01))
        row.append(f"{(e[h.argmax()] + 0.005) * 100:3.0f}")
    return " ".join(row)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("run_dir")
    ap.add_argument("--parts", type=int, default=4)
    ap.add_argument("--room", type=int, default=0)
    args = ap.parse_args()

    cfg = Settings()
    cap = record3d.load(args.capture)
    conv = df.detect_pose_convention(cap, cfg)
    if cfg.drift_correction:
        drift.correct(cap, cfg, conv)
    full = np.asarray(df.fuse(cap, cfg, convention=conv).points)
    # same upright rotation as the pipeline's LiDAR branch
    up = align.estimate_up(full, cfg)
    R = align.rotation_to_z(up) if 0.5 < up[2] < 1.0 - 1e-4 else np.eye(3)
    full = full @ R.T
    floor_z = float(np.percentile(full[:, 2], 2))
    poly = np.asarray(json.load(open(f"{args.run_dir}/results.json"))["rooms"][args.room]["polygon"])

    stride = max(1, math.ceil(cap.n_frames / cfg.max_frames))
    idxs = list(range(0, cap.n_frames, stride))
    intr = record3d.depth_intrinsics(cap)
    parts = [df._accumulate(cap, cfg, list(p), intr, conv, cfg.px_stride) @ R.T
             for p in np.array_split(idxs, args.parts)]
    for k in range(len(poly)):
        print(f"== edge {k}: wall-face offset (cm, + = outward) per 0.3 m along the edge")
        print(f"  all            : {profile(full, poly, k, floor_z)}")
        for p, P in zip(np.array_split(idxs, args.parts), parts):
            print(f"  frames {p[0]:>5}-{p[-1]:<5}: {profile(P, poly, k, floor_z)}")


if __name__ == "__main__":
    main()
