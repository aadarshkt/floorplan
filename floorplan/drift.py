"""Drift correction for the LiDAR tier: keyframe pose graph with ICP-verified loops.

ARKit/Record3D poses come from visual-inertial odometry, which drifts over a long
walk. We do not trust them as-is. Instead:

  1. pick keyframes along the walk and build a small depth cloud per keyframe;
  2. chain consecutive keyframes with their odometry (relative VIO poses);
  3. find *revisits* (keyframes that look at the same place from a nearby pose
     after a real excursion) and verify each one by ICP between the two depth
     clouds, so a loop constraint is measured geometry, not a guess;
  4. optimise the pose graph (Open3D, Levenberg-Marquardt, uncertain loop edges
     are pruned by a line process) and write a smooth per-frame correction.

With no verified revisit (a single room, or a one-way walk) the correction is
the identity and the report says so. The correction is stored on the capture as
``world_corr`` (N,4,4, canonical Z-up world) and applied wherever poses are used.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d
from PIL import Image
from scipy.spatial.transform import Rotation, Slerp

from floorplan.config import Settings
from floorplan.fusion import depth_fusion as df
from floorplan.ingest import record3d

NO_DRIFT = {"method": "none", "n_loops": 0, "correction_applied": False,
            "ablation": {"on": None, "off": None}}


def _keyframe_cloud(cap, i: int, intr: np.ndarray, cfg: Settings):
    depth = np.asarray(Image.open(cap.depth_paths[i]))
    conf = (np.asarray(Image.open(cap.conf_paths[i]))
            if len(cap.conf_paths) == cap.n_frames else None)
    fx, fy, cx, cy = intr[i]
    pc = df._frame_points(depth, conf, fx, fy, cx, cy, cfg.conf_min, cfg.depth_min_m,
                          cfg.depth_max_m, 1, cfg.depth_edge_rel)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pc))
    pcd = pcd.voxel_down_sample(cfg.pg_voxel_m)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=cfg.pg_voxel_m * 3, max_nn=30))
    return pcd


def _rot_deg(R: np.ndarray) -> float:
    return float(np.degrees(np.linalg.norm(Rotation.from_matrix(R).as_rotvec())))


def _candidates(centres: np.ndarray, fwd: np.ndarray, cfg: Settings) -> list[tuple[int, int]]:
    """Revisit candidates: close in space, similar view, but far apart along the walk."""
    n = len(centres)
    step = np.linalg.norm(np.diff(centres, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(step)])
    out: list[tuple[float, int, int]] = []
    for b in range(n):
        best = []
        for a in range(0, b - cfg.pg_min_gap_kf + 1):
            if cum[b] - cum[a] < cfg.pg_min_travel_m:
                continue
            d = float(np.linalg.norm(centres[a] - centres[b]))
            if d > cfg.pg_loop_radius_m or float(fwd[a] @ fwd[b]) < cfg.pg_min_view_dot:
                continue
            best.append((d, a, b))
        out.extend(sorted(best)[:2])
    out.sort()
    return [(a, b) for _, a, b in out[:cfg.pg_max_candidates]]


def _icp_verify(src, tgt, init: np.ndarray, cfg: Settings):
    """Coarse-to-fine point-to-plane ICP; returns (T_src->tgt, fitness, rmse) or None."""
    T = init
    res = None
    for thr in (cfg.pg_icp_dist_m * 2, cfg.pg_icp_dist_m):
        res = o3d.pipelines.registration.registration_icp(
            src, tgt, thr, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=40))
        T = res.transformation
    delta = np.linalg.inv(init) @ T
    if (res.fitness < cfg.pg_min_fitness or res.inlier_rmse > cfg.pg_max_rmse_m
            or np.linalg.norm(delta[:3, 3]) > cfg.pg_max_jump_m
            or _rot_deg(delta[:3, :3]) > cfg.pg_max_rot_deg):
        return None
    return T, float(res.fitness), float(res.inlier_rmse)


def _loop_error(T_icp: np.ndarray, P_s: np.ndarray, P_t: np.ndarray) -> float:
    """Translation disagreement (m) between a graph's relative pose and the ICP measurement."""
    E = np.linalg.inv(T_icp) @ (np.linalg.inv(P_t) @ P_s)
    return float(np.linalg.norm(E[:3, 3]))


def _interpolate(corr_kf: np.ndarray, kf_idx: np.ndarray, n: int) -> np.ndarray:
    """Per-frame corrections: slerp rotation, lerp translation between keyframes."""
    slerp = Slerp(kf_idx.astype(float), Rotation.from_matrix(corr_kf[:, :3, :3]))
    clipped = np.clip(np.arange(n), kf_idx[0], kf_idx[-1]).astype(float)
    out = np.tile(np.eye(4), (n, 1, 1))
    out[:, :3, :3] = slerp(clipped).as_matrix()
    out[:, :3, 3] = np.stack([np.interp(clipped, kf_idx, corr_kf[:, k, 3]) for k in range(3)], axis=1)
    return out


def correct(cap: record3d.Record3DCapture, cfg: Settings, convention: str,
            verbose: bool = False) -> dict:
    """Optimise the trajectory; sets ``cap.world_corr`` when loops are verified."""
    n = cap.n_frames
    info = {**NO_DRIFT, "method": "pose_graph", "n_keyframes": 0, "n_candidates": 0}
    if n < 2 * cfg.pg_min_gap_kf:
        info["method"] = "pose_graph (capture too short for a loop)"
        return info
    kf = np.unique(np.linspace(0, n - 1, min(n, cfg.pg_max_keyframes)).astype(int))
    P = [df.camera_to_world(cap, int(i), convention) for i in kf]
    centres = np.array([p[:3, 3] for p in P])
    fwd = np.array([p[:3, 2] for p in P])                  # OpenCV camera +z in world
    info["n_keyframes"] = len(kf)

    cands = _candidates(centres, fwd, cfg)
    info["n_candidates"] = len(cands)
    if not cands:
        info["method"] = "pose_graph (no revisit: nothing to close)"
        return info

    intr = record3d.depth_intrinsics(cap)
    clouds = [_keyframe_cloud(cap, int(i), intr, cfg) for i in kf]

    verified = []
    for a, b in cands:
        init = np.linalg.inv(P[b]) @ P[a]                  # source a -> target b
        got = _icp_verify(clouds[a], clouds[b], init, cfg)
        if got is not None:
            verified.append((a, b, *got))
    # best-fitting loop first; one loop per keyframe so one place is not counted twice
    verified.sort(key=lambda l: -l[3])
    loops, used = [], set()
    for l in verified:
        if l[0] in used or l[1] in used:
            continue
        loops.append(l)
        used.update((l[0], l[1]))
    if verbose:
        print(f"[drift] {len(kf)} keyframes, {len(cands)} candidates, {len(loops)} ICP-verified loops")
    if not loops:
        info["method"] = "pose_graph (no ICP-verified loop)"
        return info

    reg = o3d.pipelines.registration
    graph = reg.PoseGraph()
    for p in P:
        graph.nodes.append(reg.PoseGraphNode(p))
    for k in range(len(kf) - 1):
        rel = np.linalg.inv(P[k + 1]) @ P[k]
        inf = reg.get_information_matrix_from_point_clouds(
            clouds[k], clouds[k + 1], cfg.pg_icp_dist_m * 2, rel)
        graph.edges.append(reg.PoseGraphEdge(k, k + 1, rel, inf, uncertain=False))
    for a, b, T, _, _ in loops:
        inf = reg.get_information_matrix_from_point_clouds(
            clouds[a], clouds[b], cfg.pg_icp_dist_m * 2, T)
        graph.edges.append(reg.PoseGraphEdge(a, b, T, inf, uncertain=True))

    reg.global_optimization(
        graph, reg.GlobalOptimizationLevenbergMarquardt(),
        reg.GlobalOptimizationConvergenceCriteria(),
        reg.GlobalOptimizationOption(max_correspondence_distance=cfg.pg_icp_dist_m * 2,
                                     edge_prune_threshold=cfg.pg_prune_threshold,
                                     reference_node=0))
    P2 = [np.array(node.pose) for node in graph.nodes]

    corr_kf = np.array([p2 @ np.linalg.inv(p) for p2, p in zip(P2, P)])
    moves = np.linalg.norm(corr_kf[:, :3, 3], axis=1)
    if float(moves.max()) > cfg.pg_max_correction_m:
        info.update(method="pose_graph (rejected: correction exceeds sanity bound)",
                    max_correction_m=round(float(moves.max()), 3))
        return info

    before = [_loop_error(T, P[a], P[b]) for a, b, T, _, _ in loops]
    after = [_loop_error(T, P2[a], P2[b]) for a, b, T, _, _ in loops]
    cap.world_corr = _interpolate(corr_kf, kf, n)
    info.update(
        method="pose_graph", n_loops=len(loops), correction_applied=True,
        max_correction_m=round(float(moves.max()), 3),
        mean_correction_m=round(float(moves.mean()), 3),
        loop_error_m={"before_median": round(float(np.median(before)), 3),
                      "before_max": round(float(np.max(before)), 3),
                      "after_median": round(float(np.median(after)), 3),
                      "after_max": round(float(np.max(after)), 3)},
        loops=[{"from_kf": int(kf[a]), "to_kf": int(kf[b]), "fitness": round(f, 3),
                "icp_rmse_m": round(r, 4)} for a, b, _, f, r in loops])
    if verbose:
        le = info["loop_error_m"]
        print(f"[drift] loop error median {le['before_median']} -> {le['after_median']} m, "
              f"max correction {info['max_correction_m']} m")
    return info
