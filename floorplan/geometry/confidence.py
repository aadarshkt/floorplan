"""Confidence intervals via bootstrap resampling.

We report a 95% interval for every headline measurement. Bootstrap is used
because the error distribution from RANSAC/plane fitting is not analytic.
Sampling is seeded, so intervals are deterministic (repeatability gate).
"""
from __future__ import annotations

import numpy as np

from floorplan.config import Settings


def ci_from_samples(samples: np.ndarray, level: float) -> list[float]:
    lo = float(np.percentile(samples, (1.0 - level) / 2.0 * 100.0))
    hi = float(np.percentile(samples, (1.0 + level) / 2.0 * 100.0))
    return [round(lo, 4), round(hi, 4)]


def bracket(value: float, ci: list[float]) -> list[float]:
    """Guarantee an interval contains its own point estimate."""
    return [round(float(min(value, ci[0])), 4), round(float(max(value, ci[1])), 4)]


def interval(value: float, ci: list[float], min_half: float = 0.0) -> list[float]:
    """Bracket the value and widen to at least ``min_half`` on each side."""
    lo = min(float(value), float(ci[0]))
    hi = max(float(value), float(ci[1]))
    if float(value) - lo < min_half:
        lo = float(value) - min_half
    if hi - float(value) < min_half:
        hi = float(value) + min_half
    return [round(lo, 4), round(hi, 4)]


def bootstrap_length(points_xyz: np.ndarray, cfg: Settings,
                     sample_cap: int = 20000) -> list[float] | None:
    """Bootstrap the 2D extent (length) of a wall's inlier points."""
    pts = np.asarray(points_xyz, dtype=np.float64)
    n = len(pts)
    if n < 30:
        return None
    if n > sample_cap:
        pts = pts[np.linspace(0, n - 1, sample_cap).astype(int)]
    m = len(pts)
    rng = np.random.default_rng(cfg.ransac_seed + 1)
    lengths = np.empty(cfg.bootstrap_n)
    for k in range(cfg.bootstrap_n):
        s = pts[rng.integers(0, m, m)]
        xy = s[:, :2]
        c = xy.mean(axis=0)
        _, _, vt = np.linalg.svd(xy - c, full_matrices=False)
        d = vt[0]
        t = (xy - c) @ d
        lengths[k] = t.max() - t.min()
    return ci_from_samples(lengths, cfg.ci_level)


def bootstrap_scalar(values: np.ndarray, cfg: Settings,
                     sample_cap: int = 20000, stat=np.median) -> list[float] | None:
    """Bootstrap the CI of a scalar summary (e.g. plane height) from samples."""
    s = np.asarray(values, dtype=np.float64)
    if len(s) < 30:
        return None
    if len(s) > sample_cap:
        s = s[np.linspace(0, len(s) - 1, sample_cap).astype(int)]
    m = len(s)
    rng = np.random.default_rng(cfg.ransac_seed + 2)
    out = np.empty(cfg.bootstrap_n)
    for k in range(cfg.bootstrap_n):
        out[k] = stat(s[rng.integers(0, m, m)])
    return ci_from_samples(out, cfg.ci_level)


def bootstrap_hull_area(xy: np.ndarray, cfg: Settings,
                        sample_cap: int = 20000) -> list[float] | None:
    """Bootstrap the convex-hull area of a 2D point set (floor footprint)."""
    from scipy.spatial import ConvexHull

    pts = np.asarray(xy, dtype=np.float64)
    if len(pts) < 10:
        return None
    if len(pts) > sample_cap:
        pts = pts[np.linspace(0, len(pts) - 1, sample_cap).astype(int)]
    m = len(pts)
    rng = np.random.default_rng(cfg.ransac_seed + 3)
    areas = np.empty(cfg.bootstrap_n)
    for k in range(cfg.bootstrap_n):
        s = pts[rng.integers(0, m, m)]
        try:
            areas[k] = ConvexHull(s).volume
        except Exception:
            areas[k] = np.nan
    areas = areas[~np.isnan(areas)]
    if len(areas) < 10:
        return None
    return ci_from_samples(areas, cfg.ci_level)
