"""Score SfM settings on a Record3D clip against its ARKit poses (no depth stage).

    .venv/bin/python scripts/sfm_probe.py <record3d_dir> --out /tmp/probe \
        [--keyframes sharp] [--fps 2] [--features aliked] [--mapper global] [--focal 0.75]

Prints registration (largest model / keyframes), sub-model sizes, and the
trajectory error (ATE after a similarity alignment) of the registered cameras.
"""
from __future__ import annotations

import argparse
import json
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

from floorplan.config import Settings
from floorplan.eval.crosstier import robust_sim3
from floorplan.fusion import colmap_path, depth_fusion
from floorplan.ingest import record3d, video


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--out", required=True)
    ap.add_argument("--keyframes", default="sharp")
    ap.add_argument("--fps", type=float, default=2.0)
    ap.add_argument("--max-frames", type=int, default=300)
    ap.add_argument("--features", default="sift")
    ap.add_argument("--mapper", default="incremental")
    ap.add_argument("--focal", type=float, default=None)
    ap.add_argument("--max-size", type=int, default=None)
    ap.add_argument("--overlap", type=int, default=10)
    a = ap.parse_args()

    cfg = replace(Settings(), sfm_features=a.features, sfm_mapper=a.mapper,
                  sfm_focal_factor=a.focal, sfm_max_image_size=a.max_size,
                  sfm_seq_overlap=a.overlap, colmap_dense=False, mvs_enable=False)
    cap = record3d.load(a.capture)
    conv = depth_fusion.detect_pose_convention(cap, cfg)
    ref = depth_fusion.camera_centres(cap, conv)

    photos = video.load(Path(a.capture) / "rgb.mp4", fps=a.fps, mode=a.keyframes,
                        max_frames=a.max_frames)
    t0 = time.time()
    sfm = colmap_path.reconstruct(photos, cfg, Path(a.out), verbose=True)
    dt = time.time() - t0

    frames = np.array([photos.source_index[int(Path(im.name).stem)] for im in sfm.images])
    cent = np.array([im.centre for im in sfm.images])
    s, R, t, keep = robust_sim3(cent, ref[frames])
    res = np.linalg.norm(ref[frames] - (s * cent @ R.T + t), axis=1)
    row = {
        "capture": Path(a.capture).name, "keyframes": a.keyframes, "features": a.features,
        "mapper": a.mapper, "focal": a.focal, "n": photos.n_images,
        "registered": len(sfm.images), "models": sfm.model_sizes,
        "ate_rmse_cm": round(float(np.sqrt(np.mean(res[keep] ** 2))) * 100, 2),
        "ate_median_cm": round(float(np.median(res)) * 100, 2),
        "outliers": int((~keep).sum()),
        "focal_px": round(float(sfm.K[0, 0]), 1), "size": sfm.size,
        "true_focal_px": round(float(np.median(cap.intr[:, 0])) * sfm.size[0] / cap.rgb_size[0], 1),
        "sfm_s": round(dt, 1),
    }
    print(json.dumps(row))
    with open(Path(a.out).parent / "sfm_probe.jsonl", "a") as f:
        f.write(json.dumps(row) + "\n")


if __name__ == "__main__":
    main()
