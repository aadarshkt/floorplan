"""Monocular photo/video geometry registry.

Backends, tried in order by the photo/video tiers:

  metric_depth : a *metric* monocular-depth model fused through COLMAP SfM poses
                 — the independent path, metres with no external anchor and
                 without borrowing any LiDAR data.
  cross_tier   : the metric geometry of a paired Record3D capture of the same
                 room (``--reference-capture``), for when a LiDAR twin exists.

Relative-depth backends are deliberately not offered: a relative model cannot
make a tier metric on its own, so it would only add a non-metric candidate that
the tiers are not allowed to ship.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

from floorplan.config import Settings
from floorplan.fusion import metric_depth
from floorplan.fusion.colmap_path import ColmapResult
from floorplan.fusion.ir import SCALED, Reconstruction
from floorplan.ingest.photos import PhotoSet


def available_backends() -> list[str]:
    b: list[str] = []
    if metric_depth.available():
        b.append("metric_depth")
    return b


def reconstruct(photos: PhotoSet, cfg: Settings, workdir: Path,
                reference_points: np.ndarray | None = None,
                sfm: ColmapResult | None = None,
                verbose: bool = False) -> Reconstruction:
    """Produce a Reconstruction from stills/video frames using the best backend."""
    if cfg.engine in ("auto", "monodepth") and metric_depth.available():
        try:
            return metric_depth.reconstruct(photos, cfg, workdir, sfm=sfm, verbose=verbose)
        except Exception as exc:
            if verbose:
                print(f"[monodepth] metric depth could not run: {exc}")

    if reference_points is not None and len(reference_points) > 0:
        # Cross-tier anchor: the paired metric capture supplies the geometry. It
        # is already in the canonical Z-up metric frame, so do not re-orient.
        return Reconstruction(points=np.asarray(reference_points, dtype=np.float64),
                              tier="photos", path_chosen="monodepth:cross_tier_reference",
                              scale_reference=SCALED,
                              notes={"backend": "cross_tier", "prealigned": True})

    raise RuntimeError(
        "No monocular-depth backend is available. Install one of:\n"
        "  - Depth Anything V2 Metric:  scripts/fetch_weights.sh   (torch + transformers)\n"
        "  - or supply a paired Record3D capture of the same room with --reference-capture\n"
        "  - or use --engine colmap to reconstruct from photos with COLMAP"
    )
