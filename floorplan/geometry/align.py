"""Estimate the gravity/up axis of a fused cloud and rotate it upright.

Record3D `.r3d` poses are not always expressed in a gravity-aligned world frame
(e.g. if the world frame is fixed to the first camera pose). When that happens,
walls appear tilted and floor/ceiling detection degrades. We recover the up axis
from the largest horizontal surfaces (floor/ceiling show up as the biggest planes
and, being parallel, agree on the normal) and rotate the cloud so up = +Z.
"""
from __future__ import annotations

import numpy as np

from floorplan.config import Settings
from floorplan.geometry.planes import _ransac_plane, _refit


def estimate_up(points: np.ndarray, cfg: Settings) -> np.ndarray:
    n = len(points)
    if n < 1000:
        return np.array([0.0, 0.0, 1.0])
    ds = points[np.linspace(0, n - 1, min(n, 50000)).astype(int)]
    rng = np.random.default_rng(cfg.ransac_seed)

    models: list[tuple[int, np.ndarray]] = []
    remaining = ds
    for _ in range(3):
        if len(remaining) < 1000:
            break
        m = _ransac_plane(remaining, cfg.ransac_dist, cfg.ransac_iters, rng)
        if m is None:
            break
        inl = np.abs(remaining @ m[:3] + m[3]) < cfg.ransac_dist
        if int(inl.sum()) < 1000:
            break
        m = _refit(remaining[inl])
        models.append((int(inl.sum()), m))
        remaining = remaining[~inl]

    if not models:
        return np.array([0.0, 0.0, 1.0])

    models.sort(key=lambda t: -t[0])
    up = models[0][1][:3].copy()
    if len(models) > 1 and abs(float(up @ models[1][1][:3])) > 0.9:
        # the two biggest planes are near-parallel (floor + ceiling): average them
        n2 = models[1][1][:3]
        up = up + n2 * np.sign(float(up @ n2))
    up = up / (np.linalg.norm(up) or 1.0)

    # orient so the bulk of the cloud is on the +up side (i.e. up is a floor normal)
    if float(ds.mean(axis=0) @ up + models[0][1][3]) < 0:
        up = -up
    return up


def rotation_to_z(axis: np.ndarray) -> np.ndarray:
    """Rotation that maps ``axis`` onto +Z (Rodrigues)."""
    a = axis / (np.linalg.norm(axis) or 1.0)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, z)
    c = float(a @ z)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0.0, -v[2], v[1]],
                   [v[2], 0.0, -v[0]],
                   [-v[1], v[0], 0.0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))
