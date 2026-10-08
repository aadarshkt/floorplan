"""CPU plane-sweep multi-view stereo.

COLMAP's dense MVS (``patch_match_stereo``) needs CUDA/HIP. When unavailable we
still have accurate SfM camera poses, so we densify the sparse cloud ourselves:
for each reference image we sweep a set of depth hypotheses, warp the grayscale
image into nearby views and keep, per pixel, the depth with the lowest
photometric cost (plus a confidence).

This is deliberately modest - a small working resolution, a handful of source
views - because its job is to give the planar-geometry stage enough surface
points to fit walls/floor/ceiling. Textureless walls remain ambiguous and are
rejected by the cost threshold, so confidence here is honest, not optimistic.

The output is in the same scale-free SfM frame as the poses; metric scale is
attached later by the scale anchor.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter, ImageOps
from scipy.ndimage import uniform_filter

from floorplan.config import Settings
from floorplan.fusion.colmap_path import ImagePose
from floorplan.ingest.photos import PhotoSet


def _load_gray(path: Path, max_dim: int) -> tuple[np.ndarray, float]:
    with Image.open(path) as source:
        im = ImageOps.exif_transpose(source).convert("L")
    w, h = im.size
    s = min(1.0, max_dim / max(w, h))
    if s < 1.0:
        im = im.resize((max(1, int(round(w * s))), max(1, int(round(h * s)))), Image.BILINEAR)
    im = im.filter(ImageFilter.GaussianBlur(0.8))
    return np.asarray(im, dtype=np.float32) / 255.0, s


def _paths_for(photos: PhotoSet, images: list[ImagePose]) -> list[Path | None]:
    """Map COLMAP image names back to source paths (copies are '<idx><suffix>')."""
    by_name: dict[str, Path] = {f"{i:06d}{p.suffix.lower()}": p
                                for i, p in enumerate(photos.image_paths)}
    return [im.image_path if im.image_path is not None else by_name.get(im.name) for im in images]


def _depth_range(images: list[ImagePose], ref: int, pts: np.ndarray,
                 cfg: Settings) -> tuple[float, float] | None:
    if len(pts) < 20:
        return None
    im = images[ref]
    points = im.obs_xyz if im.obs_xyz is not None and len(im.obs_xyz) >= 20 else pts
    cam = points @ im.R.T + im.t  # world -> camera (rows)
    z = cam[:, 2]
    z = z[z > 0]
    if len(z) < 20:
        return None
    lo, hi = np.percentile(z, 2), np.percentile(z, 98)
    span = max(hi - lo, 1e-6)
    return float(max(lo - 0.15 * span, 1e-4)), float(hi + 0.15 * span)


def _sample(img: np.ndarray, x: np.ndarray, y: np.ndarray,
            valid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    h, w = img.shape
    xi = np.clip(np.round(x).astype(np.int64), 0, w - 1)
    yi = np.clip(np.round(y).astype(np.int64), 0, h - 1)
    vals = img[yi, xi]
    ok = valid & (x >= 0) & (x <= w - 1) & (y >= 0) & (y <= h - 1)
    return vals, ok


def densify(photos: PhotoSet, images: list[ImagePose], K: np.ndarray,
            sparse_pts: np.ndarray, cfg: Settings,
            verbose: bool = False) -> tuple[np.ndarray, np.ndarray]:
    paths = _paths_for(photos, images)
    usable = [i for i, p in enumerate(paths) if p is not None]
    if len(usable) < 3:
        return np.zeros((0, 3)), np.zeros((0,))

    grays: list[np.ndarray | None] = [None] * len(images)
    intrinsics = {}
    for i in usable:
        g, scale = _load_gray(paths[i], cfg.mvs_max_dim)
        grays[i] = g
        calibration = (images[i].K if images[i].K is not None else K).copy()
        if images[i].size:
            cw, ch = images[i].size
            calibration[0] *= g.shape[1] / cw
            calibration[1] *= g.shape[0] / ch
        else:
            calibration[:2] *= scale
        intrinsics[i] = calibration

    centres = np.asarray([im.centre for im in images])
    refs = ([usable[i] for i in np.linspace(0, len(usable)-1, cfg.mvs_ref_max).astype(int)]
            if cfg.mvs_ref_max and len(usable) > cfg.mvs_ref_max else usable)
    n_depth = cfg.mvs_depth_samples

    all_pts: list[np.ndarray] = []
    all_conf: list[np.ndarray] = []
    for r in refs:
        rng = _depth_range(images, r, sparse_pts, cfg)
        if rng is None:
            continue
        zs = 1.0 / np.linspace(1.0/rng[0], 1.0/rng[1], n_depth)

        # pick the nearest source views (by camera-centre distance)
        d = np.linalg.norm(centres - centres[r], axis=1)
        order = [j for j in np.argsort(d) if j != r and j in usable and d[j] > 1e-5
                 and float(images[r].R[2] @ images[j].R[2]) > 0.1][:cfg.mvs_neighbors]
        if len(order) < 2:
            continue

        ref_img = grays[r]
        h, w = ref_img.shape
        kr = intrinsics[r]
        if kr[0, 0] <= 0 or kr[1, 1] <= 0:
            continue
        uu, vv = np.meshgrid(np.arange(w, dtype=np.float64), np.arange(h, dtype=np.float64))
        dx = (uu - kr[0, 2]) / kr[0, 0]
        dy = (vv - kr[1, 2]) / kr[1, 1]
        cost = np.full((n_depth, h, w), np.inf, dtype=np.float32)
        for k, z in enumerate(zs):
            pc = np.stack([dx * z, dy * z, np.full_like(dx, z)], axis=-1)   # (h,w,3)
            world = (pc - images[r].t) @ images[r].R                        # cam -> world
            acc = np.zeros((h, w), dtype=np.float32)
            cnt = np.zeros((h, w), dtype=np.float32)
            for j in order:
                cam = world @ images[j].R.T + images[j].t
                zj = cam[..., 2]
                valid = zj > 1e-6
                safe = np.where(valid, zj, 1.0)
                kj = intrinsics[j]
                x = kj[0, 0] * cam[..., 0] / safe + kj[0, 2]
                y = kj[1, 1] * cam[..., 1] / safe + kj[1, 2]
                vals, ok = _sample(grays[j], x, y, valid)
                acc += np.where(ok, np.abs(vals - ref_img), 0.0)
                cnt += ok
            with np.errstate(invalid="ignore"):
                # Aggregate a small patch; isolated matching pixels are ambiguous.
                patch_count = uniform_filter(cnt, size=3)
                patch_cost = uniform_filter(acc, size=3) / np.maximum(patch_count, 1e-6)
                cost[k] = np.where((cnt >= 2) & (patch_count >= 2), patch_cost, np.inf)

        best = np.argmin(cost, axis=0)
        best_cost = np.take_along_axis(cost, best[None], axis=0)[0]
        # A flat white wall can match at every depth. Require texture and a
        # distinct minimum outside the winning depth's immediate neighborhood.
        alternatives = np.abs(np.arange(n_depth)[:, None, None] - best[None]) > 2
        second = np.min(np.where(alternatives, cost, np.inf), axis=0)
        variance = np.maximum(0, uniform_filter(ref_img**2, 5) - uniform_filter(ref_img, 5)**2)
        accept = (np.isfinite(best_cost) & np.isfinite(second)
                  & (best_cost <= cfg.mvs_cost_tol) & (variance >= 0.0001)
                  & (second - best_cost >= np.maximum(0.005, 0.15 * second)))
        if not accept.any():
            continue
        z = zs[best]
        pc = np.stack([dx * z, dy * z, z], axis=-1)
        world = (pc - images[r].t) @ images[r].R
        all_pts.append(world[accept])
        all_conf.append(1.0 / (1.0 + best_cost[accept]))

    if not all_pts:
        return np.zeros((0, 3)), np.zeros((0,))

    pts = np.concatenate(all_pts, axis=0)
    conf = np.concatenate(all_conf, axis=0)

    # Bound the cloud: even subsample, then drop isolated speckle.
    if len(pts) > cfg.mvs_max_points:
        idx = np.linspace(0, len(pts) - 1, cfg.mvs_max_points).astype(int)
        pts, conf = pts[idx], conf[idx]
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))
        _, keep = pcd.remove_statistical_outlier(nb_neighbors=16, std_ratio=2.0)
        keep = np.asarray(keep)
        pts, conf = pts[keep], conf[keep]
    except Exception:
        pass

    if verbose:
        print(f"[mvs] CPU densify: {len(pts)} points from {len(refs)} reference views")
    return pts, conf
