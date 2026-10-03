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


def _smooth_mask(depth: np.ndarray, edge_rel: float) -> np.ndarray:
    """False at depth discontinuities: a pixel there mixes two surfaces and
    unprojects to a 'flying' point between them."""
    from scipy.ndimage import maximum_filter, minimum_filter
    local_range = maximum_filter(depth, 3) - minimum_filter(depth, 3)
    return local_range <= np.maximum(0.03, edge_rel * depth)


def _frame_points(depth_png: np.ndarray, conf_png: np.ndarray | None,
                  fx: float, fy: float, cx: float, cy: float,
                  conf_min: int, depth_min: float, depth_max: float,
                  px_stride: int, edge_rel: float = 0.0) -> np.ndarray:
    """Unproject one depth frame to camera-space points (float metres)."""
    depth = depth_png.astype(np.float32) / 1000.0  # mm -> m
    h, w = depth.shape
    mask = (depth > depth_min) & (depth < depth_max)
    if conf_png is not None:
        mask &= conf_png >= conf_min
    if edge_rel > 0:
        mask &= _smooth_mask(depth, edge_rel)
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
    c = np.stack([out[:, 0], -out[:, 2], out[:, 1]], axis=1)
    corr = getattr(capture, "world_corr", None)
    if corr is not None:
        c = np.einsum("nij,nj->ni", corr[:, :3, :3], c) + corr[:, :3, 3]
    return c


def _accumulate(capture, cfg: Settings, idxs, intr, convention: str, px_stride: int) -> np.ndarray:
    chunks: list[np.ndarray] = []
    use_conf = len(capture.conf_paths) == capture.n_frames
    for i in idxs:
        depth_img = np.asarray(Image.open(capture.depth_paths[i]))
        conf_img = np.asarray(Image.open(capture.conf_paths[i])) if use_conf else None
        fx, fy, cx, cy = intr[i]
        pc = _frame_points(depth_img, conf_img, fx, fy, cx, cy,
                           cfg.conf_min, cfg.depth_min_m, cfg.depth_max_m, px_stride,
                           cfg.depth_edge_rel)
        if len(pc) == 0:
            continue
        world = _to_world(pc, quat_to_matrix(capture.quat[i]), capture.pos[i], convention)
        corr = getattr(capture, "world_corr", None)
        if corr is not None:
            world = world @ corr[i, :3, :3].T + corr[i, :3, 3]
        chunks.append(world)
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


CONVENTIONS = ("c2w", "w2c", "c2w_gl")


def camera_to_world(capture, index: int, convention: str) -> np.ndarray:
    """4x4 pose: OpenCV camera axes -> canonical Z-up world (same as the cloud)."""
    R = quat_to_matrix(capture.quat[index])
    t = capture.pos[index]
    if convention == "w2c":
        R, t = R.T, -R.T @ t
    if convention == "c2w_gl":
        R = R @ np.diag([1.0, -1.0, -1.0])
    to_z = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])
    pose = np.eye(4)
    pose[:3, :3], pose[:3, 3] = to_z @ R, to_z @ t
    corr = getattr(capture, "world_corr", None)
    return corr[index] @ pose if corr is not None else pose


def _read_depth(capture, index: int, cfg: Settings) -> np.ndarray:
    """Depth in metres; invalid, low-confidence and edge pixels set to 0."""
    d = np.asarray(Image.open(capture.depth_paths[index]), dtype=np.float32) / 1000.0
    valid = np.isfinite(d) & (d > cfg.depth_min_m) & (d < cfg.depth_max_m)
    if len(capture.conf_paths) == capture.n_frames:
        valid &= np.asarray(Image.open(capture.conf_paths[index])) >= cfg.conf_min
    if cfg.depth_edge_rel > 0:
        valid &= _smooth_mask(d, cfg.depth_edge_rel)
    return np.where(valid, d, 0).astype(np.float32)


def _reproject(depth, intr, pose, other_depth, other_intr, other_pose, stride=1):
    """Project one frame's depth into another; return pixels, seen and agreement masks."""
    h, w = depth.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    valid = depth[::stride, ::stride] > 0
    u, v = u[valid], v[valid]
    z = depth[v, u]
    fx, fy, cx, cy = intr
    pc = np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z))
    rel = np.linalg.inv(other_pose) @ pose
    pc = pc @ rel[:3, :3].T + rel[:3, 3]
    zp = pc[:, 2]
    fx, fy, cx, cy = other_intr
    x = np.rint(fx * pc[:, 0] / np.maximum(zp, 1e-6) + cx).astype(int)
    y = np.rint(fy * pc[:, 1] / np.maximum(zp, 1e-6) + cy).astype(int)
    oh, ow = other_depth.shape
    inside = (zp > 0) & (x >= 0) & (x < ow) & (y >= 0) & (y < oh)
    target = other_depth[np.clip(y, 0, oh - 1), np.clip(x, 0, ow - 1)]
    seen = inside & (target > 0)
    tol = 0.03 + 0.015 * np.maximum(zp, 0)
    agree = seen & (np.abs(target - zp) <= tol)
    comparable = seen & (zp <= target + tol)     # a nearer surface may hide the point
    return v, u, seen, agree, comparable


def _reprojection_agreement(capture, cfg: Settings, convention: str, n_pairs: int = 16) -> float:
    """Median share of depth that lands on the same surface in a frame ~0.5 s later."""
    intr = record3d.depth_intrinsics(capture)
    starts = np.linspace(0, max(0, capture.n_frames - 2), n_pairs).astype(int)
    fr = []
    for i in starts:
        j = int(np.searchsorted(capture.ts, capture.ts[i] + 0.5))
        j = min(capture.n_frames - 1, max(i + 1, j))
        if j == i:
            continue
        di, dj = _read_depth(capture, i, cfg), _read_depth(capture, j, cfg)
        _, _, seen, agree, _ = _reproject(di, intr[i], camera_to_world(capture, i, convention),
                                          dj, intr[j], camera_to_world(capture, j, convention),
                                          stride=4)
        if seen.sum() >= 100:
            fr.append(float(agree.sum() / seen.sum()))
    return float(np.median(fr)) if fr else 0.0


def detect_pose_convention(capture, cfg: Settings, n_frames: int = 60) -> str:
    """Pick the pose convention; scores are kept in capture.pose_selection.

    Primary: cloud crispness (occupied voxels). When the two best are within
    10 %, cross-frame depth reprojection decides (from the lidar-optimized
    branch): with the right convention a frame's depth lands on the same
    surfaces in a nearby frame.
    """
    if getattr(capture, "pose_convention", None):
        capture.pose_selection = {"source": "file_format", "chosen": capture.pose_convention}
        return capture.pose_convention
    stride = max(1, capture.n_frames // n_frames)
    idxs = list(range(0, capture.n_frames, stride))
    intr = record3d.depth_intrinsics(capture)
    scores = {}
    for conv in CONVENTIONS:
        scores[conv] = _convention_score(_accumulate(capture, cfg, idxs, intr, conv, px_stride=2), cfg)
    ranked = sorted(scores, key=scores.get)
    best, second = ranked[0], ranked[1]
    sel = {"source": "voxel_crispness", "voxel_score": {k: round(v, 4) for k, v in scores.items()}}
    if scores[second] < 1.10 * scores[best]:
        agree = {c: _reprojection_agreement(capture, cfg, c) for c in (best, second)}
        best = max(agree, key=agree.get)
        sel.update(source="depth_reprojection", reprojection_agreement=agree)
    sel["chosen"] = best
    capture.pose_selection = sel
    return best


def fuse(capture: record3d.Record3DCapture, cfg: Settings, verbose: bool = False,
         convention: str = "auto", mesh_path=None) -> o3d.geometry.PointCloud:
    """Fuse a Record3D capture into a canonical Z-up metric point cloud."""
    n = capture.n_frames
    if n == 0:
        raise ValueError("Capture has no frames")

    if convention == "auto":
        convention = detect_pose_convention(capture, cfg)
    if cfg.lidar_fusion == "tsdf":
        return _fuse_tsdf(capture, cfg, convention, mesh_path, verbose)
    if cfg.lidar_fusion != "points":
        raise ValueError(f"unknown lidar_fusion '{cfg.lidar_fusion}' (points | tsdf)")

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


# ── optional TSDF fusion (ported from the lidar-optimized branch) ─────────────

def _fuse_tsdf(capture, cfg: Settings, convention: str, mesh_path, verbose: bool):
    """Signed-distance fusion (Open3D tensor VoxelBlockGrid, CPU) + optional mesh.

    Each frame's depth is first checked against its neighbours (reprojection):
    pixels contradicted by both neighbours are dropped as flying points, newly
    visible surfaces are kept. Measured on benchmark_2 its wall sheets are no
    thinner than point accumulation, so it is opt-in; its value is the mesh.
    The tensor API is used because the legacy ScalableTSDFVolume rescales
    metre depth a second time (Open3D 0.20).
    """
    from functools import lru_cache
    voxel = cfg.tsdf_voxel_m
    trunc = cfg.tsdf_trunc_m / voxel
    vol = o3d.t.geometry.VoxelBlockGrid(
        attr_names=("tsdf", "weight"),
        attr_dtypes=(o3d.core.Dtype.Float32, o3d.core.Dtype.Float32),
        attr_channels=((1,), (1,)), voxel_size=voxel, block_resolution=16,
        block_count=min(2048, cfg.tsdf_max_blocks), device=o3d.core.Device("CPU:0"))
    intr = record3d.depth_intrinsics(capture)
    idxs = np.unique(np.linspace(0, capture.n_frames - 1,
                                 min(capture.n_frames, cfg.max_frames)).astype(int))
    poses = {int(i): camera_to_world(capture, int(i), convention) for i in idxs}

    @lru_cache(maxsize=5)
    def depth_at(i):
        return _read_depth(capture, i, cfg)

    used = 0
    for k, i in enumerate(int(x) for x in idxs):
        d = depth_at(i).copy()
        checked = np.zeros(d.shape, np.uint8)
        agreeing = np.zeros(d.shape, np.uint8)
        for nb in (k - 1, k + 1):
            if 0 <= nb < len(idxs):
                j = int(idxs[nb])
                v, u, _, agree, comparable = _reproject(d, intr[i], poses[i], depth_at(j),
                                                        intr[j], poses[j])
                checked[v, u] += comparable
                agreeing[v, u] += agree
        d[(checked > 0) & (agreeing == 0)] = 0
        if np.count_nonzero(d) < 100:
            continue
        fx, fy, cx, cy = intr[i]
        K = o3d.core.Tensor([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=o3d.core.Dtype.Float64)
        E = o3d.core.Tensor(np.linalg.inv(poses[i]), dtype=o3d.core.Dtype.Float64)
        img = o3d.t.geometry.Image(o3d.core.Tensor(np.ascontiguousarray(d, dtype=np.float32)))
        blocks = vol.compute_unique_block_coordinates(img, K, E, depth_scale=1.0,
                                                      depth_max=cfg.depth_max_m,
                                                      trunc_voxel_multiplier=trunc)
        if vol.hashmap().size() + blocks.shape[0] > cfg.tsdf_max_blocks:
            raise RuntimeError(f"TSDF exceeded {cfg.tsdf_max_blocks} blocks; use a coarser "
                               "tsdf_voxel_m or lidar_fusion=points")
        vol.integrate(blocks, img, K, E, depth_scale=1.0, depth_max=cfg.depth_max_m,
                      trunc_voxel_multiplier=trunc)
        used += 1
    pcd = vol.extract_point_cloud(weight_threshold=0.0).to_legacy()
    if len(pcd.points) < 100:
        raise ValueError(f"TSDF produced {len(pcd.points)} points; use lidar_fusion=points")
    if mesh_path is not None:
        mesh = vol.extract_triangle_mesh(weight_threshold=0.0).to_legacy()
        mesh.remove_degenerate_triangles()
        mesh.remove_unreferenced_vertices()
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    del vol
    pcd = pcd.voxel_down_sample(cfg.voxel_size)
    if len(pcd.points) >= cfg.stat_nb_neighbors:
        pcd, _ = pcd.remove_statistical_outlier(cfg.stat_nb_neighbors, cfg.stat_std_ratio)
    if verbose:
        print(f"[fusion] convention={convention} method=tsdf voxel={voxel}m frames={used} "
              f"pts={len(pcd.points)}")
    return pcd
