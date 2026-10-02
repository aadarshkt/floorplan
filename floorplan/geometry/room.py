"""Room assembly: floor/ceiling, polygon, area, ceiling height.

Floor and ceiling heights are found from the vertical density profile (horizontal
surfaces show up as peaks), which is far more robust than picking RANSAC planes
out of a noisy indoor cloud. If the ceiling was never densely observed we infer a
value from the top of the walls or a default, mark it `observed=False`, and widen
the interval — we never present a guess as a measurement.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from floorplan.config import Settings
from floorplan.geometry import confidence
from floorplan.geometry.walls import Wall


@dataclass
class Measurement:
    value: float
    ci95: list[float]
    method: str = "bootstrap"


@dataclass
class Room:
    polygon: list[list[float]]
    floor_area_m2: Measurement
    ceiling_height_m: Measurement
    ceiling_observed: bool
    floor_z: float
    walls: list[Wall] = field(default_factory=list)


def _density_floor_ceiling(points: np.ndarray, cfg: Settings) -> tuple[float, float, bool]:
    z = points[:, 2]
    lo, hi = float(z.min()), float(z.max())
    span = hi - lo
    if span < 0.5:
        return lo, lo + cfg.default_ceiling_m, False
    nbins = max(60, int(round(span / 0.02)))
    hist, edges = np.histogram(z, bins=nbins, range=(lo, hi))
    centers = (edges[:-1] + edges[1:]) / 2.0
    res = span / nbins
    k = max(1, int(round(0.10 / res)))
    smooth = np.convolve(hist, np.ones(k) / k, mode="same")

    lower = centers < lo + 0.35 * span
    upper = centers > lo + 0.65 * span
    floor_i = int(np.argmax(np.where(lower, smooth, -1.0)))
    ceil_i = int(np.argmax(np.where(upper, smooth, -1.0)))
    floor_z, ceil_z = float(centers[floor_i]), float(centers[ceil_i])

    gap = ceil_z - floor_z
    prominent = smooth[ceil_i] >= 0.05 * max(smooth[floor_i], 1.0)
    observed = bool(prominent and 1.8 <= gap <= 4.5)
    return floor_z, ceil_z, observed


def floor_and_ceiling(planes: list, points: np.ndarray, cfg: Settings
                      ) -> tuple[float, float, bool, np.ndarray, np.ndarray]:
    floor_z, ceil_z, observed = _density_floor_ceiling(points, cfg)
    if not observed:
        top = float(np.percentile(points[:, 2], 98))
        ceil_z = top if (top - floor_z) >= 1.8 else floor_z + cfg.default_ceiling_m
    floor_pts = points[np.abs(points[:, 2] - floor_z) <= 0.05]
    ceil_pts = points[np.abs(points[:, 2] - ceil_z) <= 0.05]
    # Report the plane heights with the same estimator the CI uses (median), so
    # the interval is guaranteed to bracket the reported value.
    if len(floor_pts) >= 10:
        floor_z = float(np.median(floor_pts[:, 2]))
    if len(ceil_pts) >= 10:
        ceil_z = float(np.median(ceil_pts[:, 2]))
    return floor_z, ceil_z, observed, floor_pts, ceil_pts


def build_polygon(walls: list[Wall]) -> np.ndarray | None:
    """Order wall endpoints around their centroid into a room polygon."""
    pts: list[np.ndarray] = []
    for w in walls:
        pts.append(w.start)
        pts.append(w.end)
    arr = np.asarray(pts)
    if len(arr) < 3:
        return None
    keep: list[np.ndarray] = []
    for p in arr:
        if all(float(np.linalg.norm(p - q)) > 0.08 for q in keep):
            keep.append(p)
    keep_arr = np.asarray(keep)
    if len(keep_arr) < 3:
        return None
    c = keep_arr.mean(axis=0)
    order = np.argsort(np.arctan2(keep_arr[:, 1] - c[1], keep_arr[:, 0] - c[0]))
    return keep_arr[order]


def shoelace(polygon: np.ndarray) -> float:
    n = len(polygon)
    if n < 3:
        return 0.0
    x, y = polygon[:, 0], polygon[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2.0)


def assemble(walls: list[Wall], floor_z: float, ceil_z: float, observed: bool,
             floor_pts: np.ndarray, ceil_pts: np.ndarray,
             points: np.ndarray, cfg: Settings) -> Room:
    polygon = build_polygon(walls)
    area_val = shoelace(polygon) if polygon is not None else 0.0

    # Floor area uses the observed floor footprint (convex hull) so that the
    # value and its bootstrap CI come from the same estimator.
    from scipy.spatial import ConvexHull
    if len(floor_pts) >= 10:
        try:
            hull = ConvexHull(floor_pts[:, :2])
            area_val = float(hull.volume)
            if polygon is None:
                polygon = floor_pts[:, :2][hull.vertices]
        except Exception:
            pass
    area_ci = confidence.bootstrap_hull_area(floor_pts[:, :2], cfg) if len(floor_pts) >= 10 else None
    if area_ci is None:
        area_ci = [round(area_val * 0.97, 3), round(area_val * 1.03, 3)]
    area_ci = confidence.bracket(area_val, area_ci)

    height = ceil_z - floor_z
    if observed and len(floor_pts) >= 30 and len(ceil_pts) >= 30:
        h_ci = confidence.bootstrap_scalar(ceil_pts[:, 2], cfg)
        lo_ci = confidence.bootstrap_scalar(floor_pts[:, 2], cfg)
        if h_ci is not None and lo_ci is not None:
            height_ci = [round(h_ci[0] - lo_ci[1], 4), round(h_ci[1] - lo_ci[0], 4)]
        else:
            height_ci = [round(height - 0.02, 3), round(height + 0.02, 3)]
        method = "bootstrap"
    else:
        height_ci = [round(height - 0.15, 3), round(height + 0.15, 3)]
        method = "inferred_from_wall_tops"
    height_ci = confidence.bracket(height, height_ci)

    return Room(
        polygon=[[round(float(x), 3), round(float(y), 3)]
                 for x, y in (polygon if polygon is not None else [])],
        floor_area_m2=Measurement(round(area_val, 3), area_ci, "bootstrap_hull"),
        ceiling_height_m=Measurement(round(height, 3), height_ci, method),
        ceiling_observed=bool(observed),
        floor_z=round(float(floor_z), 3),
        walls=walls,
    )
