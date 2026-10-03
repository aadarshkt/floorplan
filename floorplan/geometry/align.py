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
from floorplan.geometry import planes as planes_mod
from floorplan.geometry.planes import _ransac_plane, _refit


def estimate_up(points: np.ndarray, cfg: Settings,
                above_points: np.ndarray | None = None) -> np.ndarray:
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

    # Orient the sign. ``above_points`` (e.g. camera centres, which sit inside the
    # room above the floor) disambiguate the flip for scale-free reconstructions
    # whose orientation is otherwise arbitrary; without them we fall back to the
    # bulk-of-points heuristic used for gravity-aligned LiDAR clouds.
    ref = above_points if (above_points is not None and len(above_points) > 0) else ds
    if float(ref.mean(axis=0) @ up + models[0][1][3]) < 0:
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


def choose_up(points: np.ndarray, cfg: Settings, hint: np.ndarray | None = None,
              above_points: np.ndarray | None = None) -> np.ndarray:
    """Pick the up axis that makes the most planes axis-aligned.

    Rotating the cloud by ``rotation_to_z(u)`` maps every plane normal ``n`` to
    ``R n``; a good up makes large planes either horizontal (|nz|~1) or vertical
    (|nz|~0). We score candidates (the biggest planes' normals + an optional
    hint, e.g. mean camera-up) by ``mean(min(|nz|, 1-|nz|))`` - cheap, because no
    re-extraction is needed - then fix the sign using the camera positions.
    """
    pls = planes_mod.extract_planes(points, cfg)
    big = sorted(pls, key=lambda p: -len(p.points))[:8]
    normals = [np.asarray(p.normal, dtype=float) for p in big]
    if hint is not None and float(np.linalg.norm(hint)) > 1e-6:
        normals.append(np.asarray(hint, dtype=float) / float(np.linalg.norm(hint)))
    if not normals:
        return np.array([0.0, 0.0, 1.0])

    def misalignment(u: np.ndarray) -> float:
        R = rotation_to_z(u)
        nz = np.array([abs(float((R @ m)[2])) for m in normals])
        return float(np.mean(np.minimum(nz, 1.0 - nz)))

    up = min(normals, key=misalignment)

    # Sign: the cameras sit above the floor, so with the right sign their mean
    # height clears the lowest 5th percentile of the cloud.
    if above_points is not None and len(above_points) > 0:
        z_cam = float(np.mean(above_points @ rotation_to_z(up).T[:, 2]))
        z_floor = float(np.percentile(points @ rotation_to_z(up).T[:, 2], 5))
        if z_cam < z_floor + 0.3:
            up = -up
    return up / (np.linalg.norm(up) or 1.0)
