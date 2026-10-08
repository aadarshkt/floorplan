"""SfM-aligned monocular depth fusion (photos / video tiers).

Measured against LiDAR on Record3D clips (scripts/depth_probe.py), a metric
monocular model gets the *shape* of each frame right (~2-6 % AbsRel after a
per-frame scale fit) but its absolute scale is biased by tens of percent and
wobbles 7-37 % from frame to frame. Stacking those frames as predicted (the
``raw`` path) turns every wall into a stack of offset sheets.

This path keeps the model for shape only and lets SfM fix the geometry:

  1. gravity    from the SfM camera axes (see ``gravity``); frames are rotated so
                gravity points down before inference — depth models are trained on
                upright images, and a phone held in portrait while the sensor
                records landscape produces sideways frames.
  2. align      each frame's depth is scaled to the SfM depth of the keypoints
                COLMAP triangulated in it (off depth edges), then all frame
                scales are solved jointly so neighbouring frames' dense depths
                agree, and their common level is set by wide-baseline parallax.
  3. metric     one global factor (metres per SfM unit) from the pooled
                model-vs-SfM depth ratio, with a bootstrap interval over frames.
  4. filter     depth-discontinuity (flying pixel) removal, then multi-view
                consistency: a pixel survives only if neighbouring views see the
                same surface at the same depth.
  5. fuse       unproject, voxel-downsample, statistical outlier removal.

The cloud is returned already rotated to Z-up (``prealigned``).
"""
from __future__ import annotations

import numpy as np

from floorplan.config import Settings
from floorplan.fusion.colmap_path import ColmapResult
from floorplan.fusion.ir import METRIC_MODEL, Reconstruction
from floorplan.ingest.photos import PhotoSet


# ── gravity ──────────────────────────────────────────────────────────────────

def gravity(sfm: ColmapResult) -> tuple[np.ndarray, str]:
    """World gravity (unit, pointing down) in the SfM frame, from camera axes.

    A handheld phone is held upright (image-down ~ gravity) or sideways (image
    x ~ gravity), and is rarely rolled. With no roll, the camera axis that stays
    horizontal is perpendicular to gravity in every frame, so gravity is the
    smallest-variance direction of that axis across frames — exact under any
    pitch, unlike averaging image-down. The axis whose mean is longest is the
    one carrying gravity; its mean fixes the sign. Returns (g, hold).
    """
    R = np.stack([im.R for im in sfm.images])           # world -> camera (COLMAP: x right, y down)
    xs = R[:, 0, :]                                      # camera x axis in world
    ys = R[:, 1, :]                                      # camera y axis (image down) in world
    mx, my = xs.mean(0), ys.mean(0)
    if np.linalg.norm(my) >= np.linalg.norm(mx):
        horiz, carrier, hold = xs, my, "upright"
    else:
        horiz, carrier, hold = ys, mx, "sideways"
    _, sv, vt = np.linalg.svd(horiz, full_matrices=False)
    g = vt[2]
    # if the horizontal axis barely swept (little yaw), its smallest-variance
    # direction is ill-posed: take the carrier axis made perpendicular to it
    # instead (pitch-biased; refine_with_floor fixes that)
    if sv[1] < 0.25 * sv[0]:
        h = horiz.mean(0)
        h /= np.linalg.norm(h)
        g = carrier - (carrier @ h) * h
    if float(g @ carrier) < 0:
        g = -g
    return g / np.linalg.norm(g), hold


def refine_with_floor(points: np.ndarray, g: np.ndarray, cfg: Settings,
                      max_deg: float = 35.0) -> np.ndarray:
    """Snap gravity to the normal of the largest horizontal-ish plane near it."""
    from floorplan.geometry import planes as planes_mod
    from dataclasses import replace
    sub = points[np.linspace(0, len(points) - 1, min(len(points), 150_000)).astype(int)]
    pls = planes_mod.extract_planes(sub, replace(cfg, max_planes=8))
    cos_min = np.cos(np.radians(max_deg))
    best = None
    for p in pls:
        c = float(p.normal @ g)
        if abs(c) >= cos_min and (best is None or len(p.points) > len(best[1].points)):
            best = (np.sign(c), p)
    if best is None:
        return g
    n = best[0] * best[1].normal
    return n / np.linalg.norm(n)


def upright_k(R_w2c: np.ndarray, g_world: np.ndarray) -> int:
    """np.rot90 k that turns the image so gravity points to image-down."""
    gc = R_w2c @ g_world                                # gravity in camera axes
    gx, gy = gc[0], gc[1]
    if abs(gy) >= abs(gx):
        return 0 if gy > 0 else 2
    return -1 if gx > 0 else 1                          # right edge -> bottom: clockwise


# ── per-frame inference ──────────────────────────────────────────────────────

def _predict_upright(pipe, path, k: int, max_dim: int) -> np.ndarray:
    from PIL import Image

    from floorplan.fusion.metric_depth import _predict
    if k == 0:
        return _predict(pipe, path, max_dim)
    import tempfile
    from pathlib import Path
    with Image.open(path) as im:
        arr = np.rot90(np.asarray(im.convert("RGB")), k).copy()
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as fh:
        tmp = Path(fh.name)
    try:
        Image.fromarray(arr).save(tmp, quality=95)
        d = _predict(pipe, tmp, max_dim)
    finally:
        tmp.unlink(missing_ok=True)
    return np.rot90(d, -k).copy()


def _shrink(d: np.ndarray, max_dim: int) -> np.ndarray:
    """float32 depth at <= max_dim long side (the model predicts at ~518 px anyway)."""
    from PIL import Image
    h, w = d.shape
    if max(h, w) <= max_dim:
        return d.astype(np.float32)
    s = max_dim / max(h, w)
    return np.asarray(Image.fromarray(d.astype(np.float32), mode="F")
                      .resize((round(w * s), round(h * s)), Image.NEAREST), dtype=np.float32)


def _edge_mask(d: np.ndarray, tol: float) -> np.ndarray:
    """True where depth is smooth: no relative jump > tol to any 4-neighbour."""
    ok = np.isfinite(d) & (d > 0)
    rel = np.zeros_like(d)
    with np.errstate(divide="ignore", invalid="ignore"):
        dx = np.abs(np.diff(d, axis=1)) / np.minimum(d[:, 1:], d[:, :-1])
        dy = np.abs(np.diff(d, axis=0)) / np.minimum(d[1:, :], d[:-1, :])
    rel[:, 1:] = np.maximum(rel[:, 1:], dx)
    rel[:, :-1] = np.maximum(rel[:, :-1], dx)
    rel[1:, :] = np.maximum(rel[1:, :], dy)
    rel[:-1, :] = np.maximum(rel[:-1, :], dy)
    return ok & (np.nan_to_num(rel, nan=np.inf) <= tol)


def _sfm_depth_ratio(im, d: np.ndarray, sfm_size, cfg: Settings,
                     smooth: np.ndarray | None = None) -> np.ndarray | None:
    """SfM depth / model depth at the keypoints COLMAP triangulated in this frame.

    With ``smooth``, keypoints on depth edges (where the model blurs) are skipped
    when enough remain.
    """
    if im.obs_xyz is None or im.obs_xy is None or len(im.obs_xyz) < 10:
        return None
    h, w = d.shape
    cw, ch = sfm_size if sfm_size else (w, h)
    zc = (im.obs_xyz @ im.R.T + im.t)[:, 2]
    ui = np.round(im.obs_xy[:, 0] * (w / cw)).astype(int)
    vi = np.round(im.obs_xy[:, 1] * (h / ch)).astype(int)
    inside = (zc > 1e-9) & (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
    if inside.sum() < 10:
        return None
    zc, ui, vi = zc[inside], ui[inside], vi[inside]
    zm = d[vi, ui]
    good = np.isfinite(zm) & (zm > cfg.depth_min_m) & (zm < cfg.depth_max_m)
    if smooth is not None and (good & smooth[vi, ui]).sum() >= 10:
        good &= smooth[vi, ui]
    if good.sum() < 10:
        return None
    return zc[good] / zm[good]


# ── joint per-frame scales ───────────────────────────────────────────────────
#
# Keypoint ratios fix each frame's scale independently and still disagree by ~11 %
# frame to frame (benchmark_1 vs LiDAR), so each frame lays its own copy of a wall
# 10-30 cm apart. Neighbouring frames see the same surfaces from nearly the same
# place, so their dense depths must agree: solve all log-scales jointly so they do,
# with a prior to the keypoint scales that fixes the slow, chain-wide component.

def _unproject(f: dict, a: float, stride: int) -> np.ndarray:
    d, (fx, fy, cx, cy), im = f["d"], f["K"], f["im"]
    h, w = d.shape
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    ok = f["mask"][0:h:stride, 0:w:stride]
    z = d[0:h:stride, 0:w:stride][ok] * a
    u, v = u[ok], v[ok]
    return (np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1) - im.t) @ im.R


def _pair_log_ratio(X: np.ndarray, g: dict, a: float, gross: float) -> np.ndarray | None:
    """log(depth of X in frame g / g's own scaled depth there), on g's smooth pixels."""
    im, d, (fx, fy, cx, cy) = g["im"], g["d"], g["K"]
    Xc = X @ im.R.T + im.t
    z = Xc[:, 2]
    f = z > 1e-6
    u = np.round(Xc[f, 0] / z[f] * fx + cx).astype(int)
    v = np.round(Xc[f, 1] / z[f] * fy + cy).astype(int)
    z = z[f]
    h, w = d.shape
    inb = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v, z = u[inb], v[inb], z[inb]
    ok = g["mask"][v, u]
    if ok.sum() < 200:
        return None
    lr = np.log(z[ok] / (d[v[ok], u[ok]] * a))
    lr = lr[np.abs(lr) < gross]
    return lr if len(lr) >= 200 else None


def joint_scales(frames: list[dict], order: list[int], a0: np.ndarray, cfg: Settings,
                 iters: int = 4, stride: int = 8, gross: float = 0.25) -> np.ndarray:
    n = len(frames)
    x = np.log(a0)
    pos = {fi: k for k, fi in enumerate(order)}
    lam = cfg.depth_joint_prior
    for _ in range(iters):
        rows, rhs, wts = [], [], []
        for i in range(n):
            X = _unproject(frames[i], float(np.exp(x[i])), stride)
            p = pos[i]
            for q in range(p - cfg.depth_joint_neighbours, p + cfg.depth_joint_neighbours + 1):
                if q == p or not 0 <= q < n:
                    continue
                j = order[q]
                lr = _pair_log_ratio(X, frames[j], float(np.exp(x[j])), gross)
                if lr is None:
                    continue
                # small baselines: frame i's depth moves with its own scale, so a
                # median log-disagreement e asks for dx_i - dx_j = -e
                rows.append((i, j))
                rhs.append(-float(np.median(lr)))
                wts.append(np.sqrt(len(lr)))
        if not rows:
            return a0
        w = np.asarray(wts) / np.median(wts)
        A = np.zeros((len(rows) + n, n))
        b = np.zeros(len(rows) + n)
        for k, ((i, j), r) in enumerate(zip(rows, rhs)):
            A[k, i], A[k, j], b[k] = w[k], -w[k], w[k] * r
        A[len(rows):, :] = lam * np.eye(n)
        b[len(rows):] = lam * (np.log(a0) - x)
        dx = np.linalg.lstsq(A, b, rcond=None)[0]
        x += dx
        if np.abs(dx).max() < 1e-3:
            break
    return np.exp(x)


def parallax_level(frames: list[dict], order: list[int], a: np.ndarray,
                   gaps: tuple[int, ...] = (5, 10, 15), stride: int = 8) -> float:
    """Overall depth multiplier that best agrees with the SfM poses.

    Between frames 5-15 keyframes apart the baseline is large enough that depth
    which is uniformly too large or too small no longer lands on the same surface.
    Returns the parabola-refined minimiser of the mean pairwise disagreement.
    """
    n = len(frames)
    pairs = [(order[p], order[p + g]) for p in range(0, n, 2) for g in gaps if p + g < n]
    pairs += [(j, i) for i, j in pairs]
    cs = np.linspace(0.88, 1.12, 9)
    cost = []
    for c in cs:
        errs = []
        pts = {i: _unproject(frames[i], a[i] * c, stride) for i in {i for i, _ in pairs}}
        for i, j in pairs:
            lr = _pair_log_ratio(pts[i], frames[j], a[j] * c, 0.3)
            if lr is not None:
                errs.append(float(np.median(np.abs(lr))))
        cost.append(np.mean(errs) if len(errs) >= 10 else np.nan)
    cost = np.asarray(cost)
    if np.isnan(cost).any():
        return 1.0
    k = int(np.clip(np.argmin(cost), 1, len(cs) - 2))
    p2 = np.polyfit(cs[k - 1:k + 2], cost[k - 1:k + 2], 2)
    c = -p2[1] / (2 * p2[0]) if p2[0] > 0 else cs[k]
    return float(np.clip(c, cs[0], cs[-1]))


# ── fusion ───────────────────────────────────────────────────────────────────

def reconstruct(photos: PhotoSet, cfg: Settings, sfm: ColmapResult, pipe,
                max_dim: int = 1036, verbose: bool = False) -> Reconstruction:
    from floorplan.fusion.metric_depth import _paths_for
    from floorplan.geometry import align

    g, hold = gravity(sfm)
    paths = _paths_for(photos, sfm.images)
    K = sfm.K
    sw, sh = sfm.size

    # 1-2. inference (upright) + per-frame alignment to SfM
    frames = []        # dicts: im, depth (SfM units), K at depth res, mask
    a_list, ratio_pool = [], []
    for im, path in zip(sfm.images, paths):
        if path is None:
            continue
        k = upright_k(im.R, g) if cfg.depth_upright else 0
        d = _shrink(_predict_upright(pipe, path, k, max_dim), cfg.depth_store_dim)
        mask = (_edge_mask(d, cfg.depth_edge_tol)
                & (d > cfg.depth_min_m) & (d < cfg.depth_max_m))
        r = _sfm_depth_ratio(im, d, sfm.size, cfg, smooth=mask)
        if r is None:
            continue
        # robust per-frame scale: median after trimming the worst 20 % of ratios
        med = float(np.median(r))
        dev = np.abs(np.log(r / med))
        a = float(np.median(r[dev <= np.percentile(dev, 80)]))
        h, w = d.shape
        Kd = (K[0, 0] * w / sw, K[1, 1] * h / sh, K[0, 2] * w / sw, K[1, 2] * h / sh)
        frames.append({"im": im, "d": d, "K": Kd, "mask": mask})
        a_list.append(a)
        ratio_pool.append(1.0 / a)
    if len(frames) < 2:
        raise RuntimeError("too few frames with SfM keypoints to align monocular depth")
    order = sorted(range(len(frames)), key=lambda i: frames[i]["im"].name)

    # 2b. frame scales that agree with each other, not only with sparse keypoints.
    # The metric scale below still comes from the keypoint ratios (ratio_pool),
    # which is what the focal-law correction was calibrated on.
    a_kp = np.asarray(a_list)
    a_fr, level = a_kp, 1.0
    if cfg.depth_joint_scale and len(frames) >= 3:
        a_fr = joint_scales(frames, order, a_kp, cfg)
        if cfg.depth_parallax_level and len(frames) >= 12:
            level = parallax_level(frames, order, a_fr)
            a_fr = a_fr * level
    for f, a in zip(frames, a_fr):
        f["d"] = a * f["d"]

    # 3. global metric scale: metres per SfM unit, with a bootstrap over frames
    inv = np.asarray(ratio_pool)
    S = float(np.median(inv))
    rng = np.random.default_rng(cfg.ransac_seed)
    boots = [float(np.median(inv[rng.integers(0, len(inv), len(inv))])) for _ in range(200)]
    s_lo, s_hi = np.percentile(boots, [2.5, 97.5])
    spread = float(np.std(np.log(inv)))
    # focal correction: the model's metric output assumes a camera; measured
    # against LiDAR its bias is ~ fc / (f / upright image height) for 4:3 frames.
    upright_h = sw if hold == "sideways" else sh
    upright_w = sh if hold == "sideways" else sw
    f_rel = float(K[1, 1] if hold == "upright" else K[0, 0]) / upright_h
    aspect = upright_h / upright_w
    calibrated = 1.25 <= aspect <= 1.42                 # 4:3 portrait, as calibrated
    corr = f_rel / cfg.depth_focal_canonical if cfg.depth_focal_canonical else 1.0
    S *= corr
    s_lo, s_hi = s_lo * corr, s_hi * corr
    # systematic part of the scale error, combined with the bootstrap spread
    sys_rel = cfg.depth_scale_sys_rel if calibrated else cfg.depth_scale_sys_rel_uncal
    half = float(np.hypot(sys_rel, (s_hi - s_lo) / (2 * S)))

    # 4. multi-view consistency against the nearest keyframes (in capture order)
    pos = {fi: n for n, fi in enumerate(order)}
    stride = max(1, int(np.ceil(np.sqrt(
        len(frames) * frames[0]["d"].size / max(cfg.metric_max_points * 4, 1)))))
    clouds = []
    n_kept = n_total = 0
    for i, f in enumerate(frames):
        im, d, (fx, fy, cx, cy) = f["im"], f["d"], f["K"]
        h, w = d.shape
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        z = d[0:h:stride, 0:w:stride]
        ok = f["mask"][0:h:stride, 0:w:stride]
        u, v, z = u[ok], v[ok], z[ok]
        pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1)
        Xw = (pc - im.t) @ im.R                                   # SfM world
        n_total += len(Xw)
        votes = np.zeros(len(Xw), dtype=int)
        p = pos[i]
        for q in (p - 2, p - 1, p + 1, p + 2):
            if q < 0 or q >= len(order):
                continue
            nb = frames[order[q]]
            jm, dj, (gx, gy, gcx, gcy) = nb["im"], nb["d"], nb["K"]
            Xc = Xw @ jm.R.T + jm.t
            zj = Xc[:, 2]
            front = zj > 1e-6
            uj = np.full(len(Xw), -1)
            vj = np.full(len(Xw), -1)
            uj[front] = np.round(Xc[front, 0] / zj[front] * gx + gcx).astype(int)
            vj[front] = np.round(Xc[front, 1] / zj[front] * gy + gcy).astype(int)
            hj, wj = dj.shape
            inb = front & (uj >= 0) & (uj < wj) & (vj >= 0) & (vj < hj)
            dref = np.zeros(len(Xw))
            dref[inb] = dj[vj[inb], uj[inb]]
            agree = inb & (np.abs(zj - dref) <= cfg.depth_consistency_tol * np.maximum(dref, 1e-6))
            votes += agree
        keep = votes >= cfg.depth_consistency_min
        n_kept += int(keep.sum())
        clouds.append(Xw[keep])
    pts = np.concatenate(clouds, axis=0) * S                      # metres, SfM orientation
    centres = np.asarray([f["im"].centre for f in frames]) * S

    # 5. upright (gravity -> -Z), voxel + statistical outlier removal
    g = refine_with_floor(pts, g, cfg)
    Rz = align.rotation_to_z(-g)
    pts = pts @ Rz.T
    centres = centres @ Rz.T
    import open3d as o3d
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
    pcd = pcd.voxel_down_sample(cfg.voxel_size)
    if len(pcd.points) >= cfg.stat_nb_neighbors:
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=cfg.stat_nb_neighbors,
                                                std_ratio=cfg.stat_std_ratio)
    pts = np.asarray(pcd.points)
    if verbose:
        print(f"[fuse] hold={hold} frames={len(frames)} scale={S:.4f} m/unit "
              f"(focal corr x{corr:.3f}, +-{half*100:.1f}%, frame spread {spread*100:.1f}%, "
              f"joint {np.std(np.log(a_fr / a_kp))*100:.1f}%, level x{level:.3f}) "
              f"consistent {n_kept}/{n_total} -> {len(pts)} pts")

    return Reconstruction(
        points=pts,
        tier="photos",
        path_chosen="metric_depth:sfm_aligned",
        scale_reference=METRIC_MODEL,
        conf=np.ones(len(pts)),
        notes={
            "backend": "metric_depth", "model": cfg.metric_model_id,
            "images": len(frames), "poses": "colmap_sfm", "hold": hold,
            "internal_scale": round(S, 6),
            "scale_ci95_rel": [round(1 - half, 4), round(1 + half, 4)],
            "scale_focal_correction": round(corr, 4), "scale_calibrated_framing": calibrated,
            "f_over_upright_height": round(f_rel, 4),
            "frame_scale_spread": round(spread, 4),
            "joint_scale_change": round(float(np.std(np.log(a_fr / a_kp))), 4),
            "parallax_level": round(level, 4),
            "consistent_fraction": round(n_kept / max(n_total, 1), 3),
            "camera_centres": centres,
            "camera_names": [f["im"].name for f in frames],
            "prealigned": True,
        },
    )
