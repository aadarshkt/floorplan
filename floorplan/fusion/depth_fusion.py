"""Depth + pose fusion for the LiDAR tier.

Each depth frame is unprojected to camera-space points with its own intrinsics,
transformed into the world frame by its pose, then rotated into a canonical Z-up
metric frame. Frames are strided to a bounded count and the merged cloud is
voxel-downsampled + outlier-filtered.

Conventions (documented once, applied everywhere):
  * Record3D depth PNG values are millimetres along the camera +Z axis.
  * ARKit world is Y-up; we convert world (x, y, z) -> canonical Z-up (x, -z, y).
  * Pose convention: some exports store camera->world (c2w), others world->camera
    (w2c), and raw ARKit / .r3d poses use OpenGL camera axes (c2w_gl). We
    auto-detect the one that yields vertical walls / horizontal floors: a wrong
    choice shears the cloud or fans each frame around the camera.
"""
from __future__ import annotations

import math

import numpy as np
import open3d as o3d
from PIL import Image

from floorplan.config import Settings
from floorplan.geometry import planes as planes_mod
from floorplan.ingest import record3d


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Quaternion (qx,qy,qz,qw) -> 3x3 rotation matrix."""
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def _frame_points(depth_png: np.ndarray, conf_png: np.ndarray | None,
                  fx: float, fy: float, cx: float, cy: float,
                  conf_min: int, depth_min: float, depth_max: float,
                  px_stride: int) -> np.ndarray:
    """Unproject one depth frame to camera-space points (float metres)."""
    depth = depth_png.astype(np.float32) / 1000.0  # mm -> m
    h, w = depth.shape
    mask = (depth > depth_min) & (depth < depth_max)
    if conf_png is not None:
        mask &= conf_png >= conf_min
    if px_stride > 1:
        stride_mask = np.zeros_like(mask)
        stride_mask[::px_stride, ::px_stride] = True
        mask &= stride_mask
    if not mask.any():
        return np.zeros((0, 3), dtype=np.float64)

    v, u = np.nonzero(mask)
    z = depth[v, u].astype(np.float64)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x, y, z], axis=1)


def _to_world(pc: np.ndarray, R: np.ndarray, t: np.ndarray, convention: str) -> np.ndarray:
    if convention == "c2w_gl":
        # ARKit / raw .r3d poses use OpenGL camera axes (y up, z backward); the
        # unprojection is OpenCV (y down, z forward): flip y and z first.
        world = (pc * np.array([1.0, -1.0, -1.0])) @ R.T + t
    elif convention == "c2w":
        world = pc @ R.T + t
    else:  # w2c
        world = (pc - t) @ R
    # ARKit/Record3D world y-up (x,y,z) -> canonical Z-up (x, -z, y)
    return np.stack([world[:, 0], -world[:, 2], world[:, 1]], axis=1)


def camera_centres(capture, convention: str) -> np.ndarray:
    """Camera centres in the canonical Z-up frame (same transform as the cloud)."""
    out = np.empty((capture.n_frames, 3))
    for i in range(capture.n_frames):
        R = quat_to_matrix(capture.quat[i])
        t = capture.pos[i]
        out[i] = -R.T @ t if convention == "w2c" else t
    return np.stack([out[:, 0], -out[:, 2], out[:, 1]], axis=1)


def _accumulate(capture, cfg: Settings, idxs, intr, convention: str, px_stride: int) -> np.ndarray:
    chunks: list[np.ndarray] = []
    use_conf = len(capture.conf_paths) == capture.n_frames
    for i in idxs:
        depth_img = np.asarray(Image.open(capture.depth_paths[i]))
        conf_img = np.asarray(Image.open(capture.conf_paths[i])) if use_conf else None
        fx, fy, cx, cy = intr[i]
        pc = _frame_points(depth_img, conf_img, fx, fy, cx, cy,
                           cfg.conf_min, cfg.depth_min_m, cfg.depth_max_m, px_stride)
        if len(pc) == 0:
            continue
        chunks.append(_to_world(pc, quat_to_matrix(capture.quat[i]), capture.pos[i], convention))
    return np.concatenate(chunks, axis=0) if chunks else np.zeros((0, 3))


def _convention_score(points: np.ndarray, cfg: Settings) -> float:
    """Lower is better: occupied 5 cm voxels per point.

    With the right convention, overlapping frames land on the same surfaces and
    the cloud is crisp (few voxels); a wrong one smears every frame around its
    camera. Measured on the five benchmark captures this separates conventions
    by 1.2-3x, where a plane-orientation score picked the wrong one on a long
    multi-room scan.
    """
    if len(points) < 2000:
        return 1.0
    vox = np.unique(np.floor(points / 0.05).astype(np.int64), axis=0)
    return len(vox) / len(points)


def detect_pose_convention(capture, cfg: Settings, n_frames: int = 60) -> str:
    if getattr(capture, "pose_convention", None):
        return capture.pose_convention           # known from the file format
    stride = max(1, capture.n_frames // n_frames)
    idxs = list(range(0, capture.n_frames, stride))
    intr = record3d.depth_intrinsics(capture)
    best_conv, best_score = "c2w", None
    for conv in ("c2w", "w2c", "c2w_gl"):
        pts = _accumulate(capture, cfg, idxs, intr, conv, px_stride=2)
        score = _convention_score(pts, cfg)
        if best_score is None or score < best_score:
            best_conv, best_score = conv, score
    return best_conv


def fuse(capture: record3d.Record3DCapture, cfg: Settings, verbose: bool = False,
         convention: str = "auto") -> o3d.geometry.PointCloud:
    """Fuse a Record3D capture into a canonical Z-up metric point cloud."""
    n = capture.n_frames
    if n == 0:
        raise ValueError("Capture has no frames")

    if convention == "auto":
        convention = detect_pose_convention(capture, cfg)

    stride = max(1, math.ceil(n / cfg.max_frames))
    idxs = list(range(0, n, stride))
    intr = record3d.depth_intrinsics(capture)

    # Bound the raw point count. Voxel downsampling tens of millions of points
    # blows up memory, so if the frame budget is too dense we subsample pixels.
    px = cfg.px_stride
    if capture.depth_size is not None:
        dw, dh = capture.depth_size
        est = len(idxs) * dw * dh / (px * px)
        if est > cfg.max_raw_points:
            px = max(px, math.ceil(math.sqrt(len(idxs) * dw * dh / cfg.max_raw_points)))
    pts = _accumulate(capture, cfg, idxs, intr, convention, px)

    if len(pts) == 0:
        raise ValueError(
            "No usable depth points after confidence/range filtering. "
            "Try a lower --conf-min or check the capture."
        )

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd = pcd.voxel_down_sample(cfg.voxel_size)
    if len(pcd.points) >= cfg.stat_nb_neighbors:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=cfg.stat_nb_neighbors, std_ratio=cfg.stat_std_ratio
        )
    if verbose:
        bb = pcd.get_axis_aligned_bounding_box()
        print(f"[fusion] convention={convention} frames={len(idxs)} px_stride={px} "
              f"pts_raw={len(pts)} pts_clean={len(pcd.points)}")
        print(f"[fusion] bbox extent={np.round(bb.get_extent(), 3)} (Z is up)")
    return pcd
