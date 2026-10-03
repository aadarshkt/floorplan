"""RANSAC plane extraction (the "peel off the biggest flat sheets" stage).

Implemented in numpy with a fixed seed so results are deterministic across runs
(required for the repeatability gate). RANSAC samples from a capped subsample for
speed, then assigns inliers against the full cloud and refits by SVD.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from floorplan.config import Settings


@dataclass
class Plane:
    id: int
    model: np.ndarray      # (4,) a,b,c,d with unit normal (a,b,c)
    normal: np.ndarray     # (3,) unit
    kind: str              # "horizontal" | "vertical" | "slanted"
    points: np.ndarray     # (N, 3) inlier points
    z_median: float
    centroid: np.ndarray   # (3,)


def _ransac_plane(pts: np.ndarray, dist: float, iters: int,
                  rng: np.random.Generator, sample_cap: int = 20000) -> np.ndarray | None:
    n = len(pts)
    if n < 3:
        return None
    if n > sample_cap:
        sample = pts[np.linspace(0, n - 1, sample_cap).astype(int)]
    else:
        sample = pts
    m = len(sample)
    best_cnt, best = -1, None
    budget = iters
    for it in range(iters):
        if it >= budget:
            break
        tri = rng.choice(m, 3, replace=False)
        p0, p1, p2 = sample[tri]
        nrm = np.cross(p1 - p0, p2 - p0)
        ln = float(np.linalg.norm(nrm))
        if ln < 1e-9:
            continue
        nrm = nrm / ln
        d = -float(nrm @ p0)
        cnt = int(np.count_nonzero(np.abs(sample @ nrm + d) < dist))
        if cnt > best_cnt:
            best_cnt, best = cnt, np.concatenate([nrm, [d]])
            # stop once the inlier share gives 99.9 % confidence of a clean sample
            # (adaptive RANSAC, from the lidar-optimized branch); deterministic
            w3 = min((cnt / m) ** 3, 1.0 - 1e-12)
            if w3 > 0:
                budget = min(budget, max(64, int(np.ceil(np.log(0.001) / np.log1p(-w3)))))
    return best


def _refit(pts: np.ndarray) -> np.ndarray:
    """Total-least-squares plane fit on the inliers."""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    nrm = vt[2]
    ln = float(np.linalg.norm(nrm))
    if ln < 1e-12:
        return np.array([0.0, 0.0, 1.0, 0.0])
    nrm = nrm / ln
    return np.concatenate([nrm, [-float(nrm @ c)]])


def extract_planes(points: np.ndarray, cfg: Settings) -> list[Plane]:
    rng = np.random.default_rng(cfg.ransac_seed)
    remaining = np.asarray(points, dtype=np.float64)
    # A plane must be at least an absolute floor *or* a small fraction of the
    # cloud — otherwise RANSAC peels off tiny noise slabs forever.
    min_pts = max(cfg.min_plane_points, int(0.002 * len(points)))
    planes: list[Plane] = []
    for pid in range(cfg.max_planes):
        if len(remaining) < min_pts:
            break
        model = _ransac_plane(remaining, cfg.ransac_dist, cfg.ransac_iters, rng)
        if model is None:
            break
        inl = np.abs(remaining @ model[:3] + model[3]) < cfg.ransac_dist
        if int(inl.sum()) < min_pts:
            break
        model = _refit(remaining[inl])
        inl = np.abs(remaining @ model[:3] + model[3]) < cfg.ransac_dist
        inl_pts = remaining[inl]
        remaining = remaining[~inl]

        nrm = model[:3]
        nz = abs(float(nrm[2]))
        if nz >= cfg.horizontal_normal_z:
            kind = "horizontal"
        elif nz <= cfg.vertical_normal_z_max:
            kind = "vertical"
        else:
            kind = "slanted"
        planes.append(Plane(
            id=pid, model=model, normal=nrm, kind=kind, points=inl_pts,
            z_median=float(np.median(inl_pts[:, 2])), centroid=inl_pts.mean(axis=0),
        ))
    return planes
