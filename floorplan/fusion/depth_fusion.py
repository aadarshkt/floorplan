"""Depth + pose fusion for the LiDAR tier.

Each depth frame is unprojected to camera-space points with its own intrinsics,
transformed into the ARKit world frame by its pose, then rotated into a
canonical Z-up metric frame. Frames are strided to a bounded count and the
merged cloud is voxel-downsampled + outlier-filtered.

Conventions (documented once, applied everywhere):
  * ARKit world is Y-up; camera looks down -Z.
  * Record3D depth PNG values are millimetres along the camera +Z axis.
  * We convert ARKit world (x, y, z) -> canonical Z-up (x, -z, y).
"""
from __future__ import annotations

import math

import numpy as np
import open3d as o3d
from PIL import Image

from floorplan.config import Settings
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


def fuse(capture: record3d.Record3DCapture, cfg: Settings,
         verbose: bool = False) -> o3d.geometry.PointCloud:
    """Fuse a Record3D capture into a canonical Z-up metric point cloud."""
    n = capture.n_frames
    if n == 0:
        raise ValueError("Capture has no frames")

    stride = max(1, math.ceil(n / cfg.max_frames))
    intr = record3d.depth_intrinsics(capture)
    use_conf = len(capture.conf_paths) == n

    chunks: list[np.ndarray] = []
    for i in range(0, n, stride):
        depth_img = np.asarray(Image.open(capture.depth_paths[i]))
        conf_img = np.asarray(Image.open(capture.conf_paths[i])) if use_conf else None
        fx, fy, cx, cy = intr[i]
        pc = _frame_points(depth_img, conf_img, fx, fy, cx, cy,
                           cfg.conf_min, cfg.depth_min_m, cfg.depth_max_m, cfg.px_stride)
        if len(pc) == 0:
            continue
        R = quat_to_matrix(capture.quat[i])
        world = pc @ R.T + capture.pos[i]
        # ARKit y-up (x,y,z) -> canonical Z-up (x, -z, y)
        zup = np.stack([world[:, 0], -world[:, 2], world[:, 1]], axis=1)
        chunks.append(zup)

    if not chunks:
        raise ValueError(
            "No usable depth points after confidence/range filtering. "
            "Try a lower --conf-min or check the capture."
        )

    pts = np.concatenate(chunks, axis=0)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd = pcd.voxel_down_sample(cfg.voxel_size)
    if len(pcd.points) >= cfg.stat_nb_neighbors:
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=cfg.stat_nb_neighbors, std_ratio=cfg.stat_std_ratio
        )
    if verbose:
        bb = pcd.get_axis_aligned_bounding_box()
        print(f"[fusion] frames={len(chunks)} pts_raw={len(pts)} pts_clean={len(pcd.points)}")
        print(f"[fusion] bbox extent={np.round(bb.get_extent(), 3)} (Z is up)")
    return pcd
