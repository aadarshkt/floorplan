"""Metric monocular depth — the independent photos/video geometry source.

The photo and video tiers are specified as "no depth, no poses": the input files
carry neither metric scale nor camera positions. This backend recovers both from
the images alone:

  * a **metric** monocular-depth model (Depth Anything V2 *Metric*, Indoor)
    predicts absolute metres per pixel — so the tier needs neither a paired
    LiDAR capture nor an operator-measured length to become metric, and
  * **COLMAP SfM** recovers the camera poses and intrinsics from the same
    images (poses we compute, not poses we were given), which is what lets the
    per-image depth maps be fused into a single room cloud.

    photo folder / video --ffmpeg--> frames
        |-- COLMAP SfM .............. poses + intrinsics
        |-- metric monocular depth .... per-pixel metres
        `--> unproject(K, depth) -> pose -> one metric point cloud

The result keeps COLMAP's arbitrary world frame, so it is *metric* but not yet
Z-up; the shared geometry tail still aligns it. We never invent geometry: with
several images and no poses we raise rather than emit disconnected per-image
clouds.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np

from floorplan.config import Settings
from floorplan.fusion.colmap_path import ColmapResult
from floorplan.fusion.ir import METRIC_MODEL, Reconstruction
from floorplan.ingest.photos import PhotoSet


def available() -> bool:
    return all(importlib.util.find_spec(m) is not None
               for m in ("torch", "transformers", "PIL"))


def pick_device(cfg: Settings):
    """Resolve cfg.device to (torch.device, dtype). No CUDA required.

    auto -> cuda if present, else Apple-Silicon MPS, else CPU. MPS/CPU use
    float32 (float16 on MPS is numerically flaky for these ViT models).
    """
    import torch

    want = (cfg.device or "auto").lower()
    has_mps = bool(getattr(torch.backends, "mps", None)) and torch.backends.mps.is_available()
    if want == "auto":
        want = "cuda" if torch.cuda.is_available() else ("mps" if has_mps else "cpu")
    if want == "cuda" and torch.cuda.is_available():
        return torch.device("cuda"), torch.float16
    if want == "mps" and has_mps:
        return torch.device("mps"), torch.float32
    return torch.device("cpu"), torch.float32


def _predict(pipe, path: Path, max_dim: int) -> np.ndarray:
    """Per-pixel metric depth (metres) for one image, at its native resolution."""
    from PIL import Image

    with Image.open(path) as im:
        img = im.convert("RGB")
    w, h = img.size
    if max(w, h) > max_dim:  # bound inference cost on large phone frames
        s = max_dim / max(w, h)
        img = img.resize((max(1, round(w * s)), max(1, round(h * s))), Image.BILINEAR)
        w, h = img.size
    out = pipe(img)
    d = out.get("predicted_depth")
    if d is None:
        raise RuntimeError(
            "depth pipeline returned no 'predicted_depth' — "
            f"'{getattr(pipe, 'model', None)}' looks like a relative model; "
            "set cfg.metric_model_id to a metric variant")
    arr = d.squeeze().detach().to("cpu").float().numpy().astype(np.float64)
    if arr.shape != (h, w):
        arr = np.asarray(Image.fromarray(arr.astype(np.float32), mode="F")
                         .resize((w, h), Image.BILINEAR), dtype=np.float64)
    return arr


def _paths_for(photos: PhotoSet, images: list) -> list[Path | None]:
    """Map COLMAP image names back to source paths (copies are '<idx><suffix>')."""
    by_name = {f"{i:06d}{p.suffix.lower()}": p for i, p in enumerate(photos.image_paths)}
    return [by_name.get(im.name) for im in images]


def _px_stride(n_images: int, size: tuple[int, int] | None, budget: int) -> int:
    if not size or n_images <= 0:
        return 2
    w, h = size
    per_image = max(1, budget // n_images)
    if w * h <= per_image:
        return 1
    return max(1, int(np.ceil(np.sqrt((w * h) / per_image))))


def _load_pipe(cfg: Settings, verbose: bool):
    import torch
    from transformers import pipeline

    torch.set_num_threads(cfg.threads)

    dev, dtype = pick_device(cfg)
    if verbose:
        print(f"[metric] {cfg.metric_model_id} on {dev.type}")
    try:
        return pipeline("depth-estimation", model=cfg.metric_model_id, device=dev, dtype=dtype)
    except TypeError:  # older transformers
        return pipeline("depth-estimation", model=cfg.metric_model_id, device=dev, torch_dtype=dtype)


def reconstruct(photos: PhotoSet, cfg: Settings, workdir: Path,
                sfm: ColmapResult | None = None,
                max_dim: int = 1036, verbose: bool = False) -> Reconstruction:
    if cfg.depth_fusion == "aligned" and sfm is not None and len(sfm.images) >= 2:
        from floorplan.fusion import sfm_depth
        return sfm_depth.reconstruct(photos, cfg, sfm, _load_pipe(cfg, verbose),
                                     max_dim=max_dim, verbose=verbose)
    import torch
    from transformers import pipeline

    model_id = cfg.metric_model_id
    dev, dtype = pick_device(cfg)
    if verbose:
        print(f"[metric] {model_id} on {dev.type}")
    try:
        pipe = pipeline("depth-estimation", model=model_id, device=dev, dtype=dtype)
    except TypeError:  # older transformers
        pipe = pipeline("depth-estimation", model=model_id, device=dev, torch_dtype=dtype)

    poses = list(sfm.images) if sfm is not None else []
    if poses:
        paths = _paths_for(photos, poses)
        entries = [(im, p) for im, p in zip(poses, paths) if p is not None]
        if not entries:
            raise RuntimeError("COLMAP registered images do not map back to the input photos")
    else:
        # No poses (too few overlapping stills): fall back to one image so the tier
        # still answers; scale is uncalibrated and intervals are widened downstream.
        if verbose and photos.n_images > 1:
            print("[metric] no SfM poses: single-image fallback (wide intervals)")
        entries = [(None, photos.image_paths[photos.n_images // 2])]

    K = sfm.K if sfm is not None else None
    stride = _px_stride(len(entries), photos.size, cfg.metric_max_points)

    # Pass 1: per-image metric depth + intrinsics at that image's native size.
    frames: list[tuple] = []          # (pose|None, depth, fx, fy, cx, cy)
    for im, path in entries:
        d = _predict(pipe, path, max_dim)
        h, w = d.shape
        if K is not None and K[0, 0] > 0:
            fxv, fyv, cxv, cyv = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
            if photos.size:  # model may have resized; carry the intrinsics across
                sw, sh = photos.size
                fxv *= w / sw
                fyv *= h / sh
                cxv *= w / sw
                cyv *= h / sh
        else:
            f = photos.f_px or (0.5 * w / np.tan(np.radians((photos.fov_deg or 65.0) / 2.0)))
            fxv = fyv = f
            cxv, cyv = w / 2.0, h / 2.0
        if fxv <= 0 or fyv <= 0:
            continue
        frames.append((im, d, fxv, fyv, cxv, cyv))

    # COLMAP translations are scale-free while the depth is metric. Recover the
    # single scale mapping COLMAP units -> metres by comparing, per image, the
    # metric depth against the SfM depth of that image's *observed* keypoints
    # (the 2D-3D correspondences COLMAP actually triangulated).
    scale = 1.0
    if sfm is not None:
        cw, ch = sfm.size if sfm.size else (0, 0)
        ratios: list[float] = []
        for im, d, fxv, fyv, cxv, cyv in frames:
            if im.obs_xyz is None or im.obs_xy is None or len(im.obs_xyz) < 10:
                continue
            h, w = d.shape
            cam = im.obs_xyz @ im.R.T + im.t          # world -> camera
            zc = cam[:, 2]
            u = im.obs_xy[:, 0] * (w / cw if cw else 1.0)
            v = im.obs_xy[:, 1] * (h / ch if ch else 1.0)
            ui = np.round(u).astype(int)
            vi = np.round(v).astype(int)
            inside = (zc > 1e-6) & (ui >= 0) & (ui < w) & (vi >= 0) & (vi < h)
            if int(inside.sum()) < 10:
                continue
            zm = d[vi[inside], ui[inside]]
            zcl = zc[inside]
            good = (zm > cfg.depth_min_m) & (zm < cfg.depth_max_m)
            if int(good.sum()) < 10:
                continue
            ratios.append(float(np.median(zm[good] / zcl[good])))
        if ratios:
            scale = float(np.median(ratios))

    # Pass 2: unproject with metric depth and place at the (scaled) camera centre.
    clouds: list[np.ndarray] = []
    for im, d, fxv, fyv, cxv, cyv in frames:
        h, w = d.shape
        v, u = np.mgrid[0:h:stride, 0:w:stride]
        z = d[0:h:stride, 0:w:stride]
        ok = np.isfinite(z) & (z >= cfg.depth_min_m) & (z <= cfg.depth_max_m)
        if not ok.any():
            continue
        z = z[ok]
        pc = np.stack([(u[ok] - cxv) * z / fxv, (v[ok] - cyv) * z / fyv, z], axis=-1)
        if im is not None:
            centre = scale * im.centre                 # camera position in metres
            pc = centre + pc @ im.R                    # X_w = C + R^T X_c
        clouds.append(pc)

    if not clouds:
        raise RuntimeError("metric depth produced no points in range "
                           f"[{cfg.depth_min_m}, {cfg.depth_max_m}] m")

    pts = np.concatenate(clouds, axis=0)
    if len(pts) > cfg.metric_max_points:
        idx = np.linspace(0, len(pts) - 1, cfg.metric_max_points).astype(int)
        pts = pts[idx]
    if verbose:
        print(f"[metric] fused {len(clouds)}/{len(entries)} image(s) -> {len(pts)} metric points")

    return Reconstruction(
        points=pts,
        tier="photos",
        path_chosen="metric_depth:sfm_fused" if poses else "metric_depth:single_image",
        scale_reference=METRIC_MODEL,
        conf=np.ones(len(pts)),
        notes={
            "backend": "metric_depth",
            "model": model_id,
            "images": len(clouds),
            "poses": "colmap_sfm" if poses else "none",
            "internal_scale": round(float(scale), 6),
            "camera_centres": (np.asarray([scale * im.centre for im, _ in entries])
                               if poses else None),
            "camera_names": [im.name for im, _ in entries] if poses else None,
            "up_hint": sfm.up_hint if sfm is not None else None,
        },
    )
