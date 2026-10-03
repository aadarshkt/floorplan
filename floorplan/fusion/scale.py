"""Metric scale anchoring for scale-free reconstructions (photos / video).

A COLMAP (or learned-relative-depth) reconstruction is metric only up to an
unknown global scale. We recover it from, in order of preference:

  1. A paired metric capture of the same room (Record3D) -> similarity ICP
     recovers scale and pose.  scale_reference = "scaled_via_reference".
  2. An explicit known length (``--scale-ref``) interpreted as the room's
     floor-to-ceiling height -> scale so the observed vertical extent matches.
     scale_reference = "scaled_via_reference".
  3. Nothing -> a default-prior scale from an assumed ceiling height.
     scale_reference = "none_prior"  (the caller widens intervals and marks the
     plan non-metric).

Everything is reported, never silently applied.
"""
from __future__ import annotations

import numpy as np

from floorplan.config import Settings
from floorplan.fusion.ir import NONE_PRIOR, SCALED


def _vertical_extent(points: np.ndarray) -> float:
    z = points[:, 2]
    return float(np.percentile(z, 98) - np.percentile(z, 2))


def _similarity_icp(source: np.ndarray, target: np.ndarray,
                    cfg: Settings) -> tuple[float, np.ndarray]:
    """Return (scale, translation-aligned points) aligning source onto target."""
    import open3d as o3d

    c_s, c_t = source.mean(axis=0), target.mean(axis=0)
    s = source - c_s
    t = target - c_t
    diag_s = float(np.linalg.norm(s.max(axis=0) - s.min(axis=0)))
    diag_t = float(np.linalg.norm(t.max(axis=0) - t.min(axis=0)))
    rough = diag_t / diag_s if diag_s > 1e-9 else 1.0

    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(s * rough))
    tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(t))
    init = np.eye(4)
    est = o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True)
    res = o3d.pipelines.registration.registration_icp(
        src, tgt, cfg.anchor_icp_max_dist, init, est,
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60))
    scale = rough * float(np.cbrt(np.linalg.det(res.transformation[:3, :3])))
    aligned = np.asarray(src.transform(res.transformation).points) + c_t
    return scale, aligned


def anchor(points: np.ndarray, cfg: Settings,
           reference_points: np.ndarray | None = None
           ) -> tuple[np.ndarray, float, str, dict]:
    """Scale ``points`` to metres. Returns (scaled_points, scale, reference, notes)."""
    pts = np.asarray(points, dtype=np.float64)

    if reference_points is not None and len(reference_points) > 4:
        scale, aligned = _similarity_icp(pts, np.asarray(reference_points, dtype=np.float64), cfg)
        return aligned, float(scale), SCALED, {"anchor": "reference_capture", "scale": float(scale)}

    if cfg.scale_ref_m and cfg.scale_ref_kind in ("ceiling_height", "room_height", "door_height"):
        ext = _vertical_extent(pts)
        if ext > 1e-6:
            scale = cfg.scale_ref_m / ext
            return pts * scale, float(scale), SCALED, {
                "anchor": cfg.scale_ref_kind, "scale": float(scale), "ref_m": cfg.scale_ref_m}

    ext = _vertical_extent(pts)
    scale = (cfg.default_ceiling_m / ext) if ext > 1e-6 else 1.0
    return pts * scale, float(scale), NONE_PRIOR, {"anchor": "default_prior", "scale": float(scale)}
