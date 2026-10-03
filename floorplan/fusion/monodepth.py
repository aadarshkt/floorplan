"""Monocular-depth registry (the default photo/video geometry source).

Backends, in preference order for ``engine == "auto"``:

  depth_anything : a learned metric/relative-depth model (Depth Anything V2 via
                   transformers + torch). Relative predictions are median-scale
                   aligned to the scale anchor.
  cross_tier     : use the metric depth of a paired Record3D capture of the same
                   room as the geometry source (the documented cross-tier anchor).
  none           : unavailable -> the caller must fall back to COLMAP or fail.

We never fabricate geometry: if no backend can run we raise with guidance rather
than emitting an un-scaled, un-anchored plan.
"""
from __future__ import annotations

import importlib.util
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from floorplan.config import Settings
from floorplan.fusion.ir import NONE_PRIOR, SCALED, Reconstruction
from floorplan.ingest.photos import PhotoSet

_DA_MODEL = "depth-anything/Depth-Anything-V2-Small-hf"


def _has(mod: str) -> bool:
    return importlib.util.find_spec(mod) is not None


def available_backends() -> list[str]:
    b: list[str] = []
    if _has("torch") and _has("transformers"):
        b.append("depth_anything")
    # cross_tier is available whenever the caller supplies a reference capture
    return b


def _depth_anything(photos: PhotoSet, cfg: Settings, f_px: float) -> tuple[np.ndarray, np.ndarray]:
    """Best-effort per-image relative depth -> a single unprojected cloud."""
    import torch
    from PIL import Image
    from transformers import pipeline

    device = 0 if torch.cuda.is_available() else -1
    pipe = pipeline("depth-estimation", model=_DA_MODEL, device=device)
    cloud, conf = [], []
    for p in photos.image_paths:
        img = Image.open(p).convert("RGB")
        res = pipe(img)["depth"]
        d = np.asarray(res, dtype=np.float64)
        h, w = d.shape
        s = (photos.size[0] / w) if photos.size else 1.0
        fx = f_px / s
        cx, cy = w / 2.0, h / 2.0
        d = d / (np.median(d) + 1e-9)  # relative; anchored later
        v, u = np.mgrid[0:h, 0:w]
        z = d
        x = (u - cx) * z / fx
        y = (v - cy) * z / fx
        pts = np.stack([x, y, z], axis=-1).reshape(-1, 3)
        cloud.append(pts)
        conf.append(np.ones(len(pts)))
    return np.concatenate(cloud, axis=0), np.concatenate(conf)


def reconstruct(photos: PhotoSet, cfg: Settings, workdir: Path,
                reference_points: np.ndarray | None = None,
                verbose: bool = False) -> Reconstruction:
    """Produce a Reconstruction from still photos using the best available backend."""
    backends = available_backends()

    if "depth_anything" in backends and cfg.engine in ("auto", "monodepth"):
        if photos.size:
            w, _h = photos.size
            f_px = photos.f_px or (0.5 * w / np.tan(np.radians((photos.fov_deg or 65.0) / 2.0)))
            try:
                pts, c = _depth_anything(photos, cfg, f_px)
                return Reconstruction(points=pts, tier="photos",
                                      path_chosen="monodepth:depth_anything_v2",
                                      scale_reference=NONE_PRIOR, conf=c,
                                      notes={"backend": "depth_anything"})
            except Exception as exc:  # pragma: no cover - depends on optional deps
                if verbose:
                    print(f"[monodepth] depth_anything failed: {exc}")

    if reference_points is not None and len(reference_points) > 0:
        # Cross-tier anchor: the paired metric capture supplies the geometry.
        # It is already in the canonical Z-up metric frame, so do not re-orient.
        return Reconstruction(points=np.asarray(reference_points, dtype=np.float64),
                              tier="photos", path_chosen="monodepth:cross_tier_reference",
                              scale_reference=SCALED,
                              notes={"backend": "cross_tier", "prealigned": True})

    raise RuntimeError(
        "No monocular-depth backend is available. Install one of:\n"
        "  - Depth Anything V2:  pip install torch transformers  (see scripts/fetch_weights.sh)\n"
        "  - or supply a paired Record3D capture of the same room with --reference-capture\n"
        "  - or use --engine colmap to reconstruct from photos with COLMAP"
    )
