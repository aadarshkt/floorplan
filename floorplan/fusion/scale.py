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
                    cfg: Settings) -> tuple[float, np.ndarray, np.ndarray]:
    """Align upright clouds with multiple yaw initializations and reject poor fits."""
    import open3d as o3d
    source = source[np.isfinite(source).all(axis=1)]
    target = target[np.isfinite(target).all(axis=1)]
    c_s, c_t = np.median(source, axis=0), np.median(target, axis=0)
    source_span = np.linalg.norm(np.percentile(source, 98, axis=0) - np.percentile(source, 2, axis=0))
    target_span = np.linalg.norm(np.percentile(target, 98, axis=0) - np.percentile(target, 2, axis=0))
    if min(source_span, target_span) < 1e-6:
        raise ValueError("Degenerate cloud cannot anchor metric scale")
    rough = target_span / source_span
    src = o3d.geometry.PointCloud(o3d.utility.Vector3dVector((source-c_s)*rough)).voxel_down_sample(0.05)
    tgt = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(target-c_t)).voxel_down_sample(0.05)
    estimator = o3d.pipelines.registration.TransformationEstimationPointToPoint(with_scaling=True)
    results = []
    for angle in (0.0, np.pi/2, np.pi, 3*np.pi/2):
        init = np.eye(4)
        c, t = np.cos(angle), np.sin(angle)
        init[:2, :2] = [[c, -t], [t, c]]
        coarse = o3d.pipelines.registration.registration_icp(
            src, tgt, cfg.anchor_icp_max_dist, init, estimator,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
        fine = o3d.pipelines.registration.registration_icp(
            src, tgt, 0.15, coarse.transformation, estimator,
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
        results.append(fine)
    best = max(results, key=lambda r: (r.fitness, -r.inlier_rmse))
    if best.fitness < 0.35 or best.inlier_rmse > 0.10:
        raise ValueError("Reference capture alignment has insufficient geometric overlap")
    transform = best.transformation.copy()
    transform[:3, :3] *= rough
    transform[:3, 3] += c_t - transform[:3, :3] @ c_s
    scale = float(np.cbrt(np.linalg.det(transform[:3, :3])))
    if not np.isfinite(scale) or not 0.5 <= scale / rough <= 2.0:
        raise ValueError("Reference alignment produced invalid metric scale")
    aligned = source @ transform[:3, :3].T + transform[:3, 3]
    return scale, aligned, transform


def anchor(points: np.ndarray, cfg: Settings,
           reference_points: np.ndarray | None = None,
           up_hint: np.ndarray | None = None,
           camera_centres: np.ndarray | None = None
           ) -> tuple[np.ndarray, float, str, dict]:
    """Orient before measuring height; transform camera metadata with the cloud."""
    from floorplan.geometry import align, planes
    pts = np.asarray(points, dtype=np.float64)
    if up_hint is not None and float(np.linalg.norm(up_hint)) > 1e-6:
        # The mean camera-up is the gravity estimate. choose_up() only scores plane
        # normals, and every axis of a box room scores equally, so it may return a
        # wall normal as "up"; trust the hint and use the cameras to fix its sign.
        up = np.asarray(up_hint, dtype=float)
        up = up / float(np.linalg.norm(up))
        if camera_centres is not None and len(camera_centres):
            z = pts @ align.rotation_to_z(up).T[:, 2]
            if float(np.mean(np.asarray(camera_centres) @ align.rotation_to_z(up).T[:, 2])) \
                    < float(np.percentile(z, 5)) + 0.3:
                up = -up
    else:
        up = align.choose_up(pts, cfg, above_points=camera_centres)
    rotation = align.rotation_to_z(up)
    upright = pts @ rotation.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    notes = {"prealigned": True, "up_hint": np.array([0.0, 0.0, 1.0])}
    if reference_points is not None and len(reference_points) > 4:
        factor, result, registration = _similarity_icp(upright, reference_points, cfg)
        transform = registration @ transform
        ref, kind = SCALED, "reference_capture"
    else:
        if cfg.scale_ref_m is not None:
            if cfg.scale_ref_m <= 0 or cfg.scale_ref_kind not in ("ceiling_height", "room_height"):
                raise ValueError("Scale anchoring currently supports a positive floor-to-ceiling height only")
            # A known ceiling height must match observed horizontal surfaces,
            # not the arbitrary raw-Z extent of an SfM cloud.
            horizontal = [p for p in planes.extract_planes(upright, cfg) if p.kind == "horizontal"]
            if not horizontal:
                raise ValueError("Height scale reference requires observed floor and ceiling planes")
            largest = max(len(p.points) for p in horizontal)
            levels = [p.z_median for p in horizontal if len(p.points) >= 0.15*largest]
            extent = max(levels) - min(levels)
            if extent <= 1e-6:
                raise ValueError("Cannot anchor ceiling height: two distinct horizontal planes are required")
            factor = cfg.scale_ref_m / extent
            ref, kind = SCALED, cfg.scale_ref_kind
        else:
            extent = _vertical_extent(upright)
            factor = cfg.default_ceiling_m / extent if extent > 1e-6 else 1.0
            ref, kind = NONE_PRIOR, "default_prior"
        result = upright * factor
        transform[:3, :3] *= factor
    if camera_centres is not None:
        centres = np.asarray(camera_centres)
        notes["camera_centres"] = centres @ transform[:3, :3].T + transform[:3, 3]
    notes.update(anchor=kind, scale=float(factor))
    return result, float(factor), ref, notes
