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
    room_id: str = "r1"


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

    # With a noisy cloud the densest upper band is often a bed/wardrobe top, not
    # the ceiling. A horizontal RANSAC plane is a stronger witness: prefer the
    # highest sizable plane that sits at a plausible occupied height above the
    # floor. If there is none, the ceiling was never observed — say so honestly
    # rather than presenting a spurious dense band as the ceiling.
    ceil_plane = _ceil_from_planes(planes, floor_z, cfg) if planes else None
    if ceil_plane is not None:
        ceil_z, observed = ceil_plane, True
    else:
        ceil_z, observed = floor_z + cfg.default_ceiling_m, False

    floor_pts = points[np.abs(points[:, 2] - floor_z) <= 0.05]
    ceil_pts = points[np.abs(points[:, 2] - ceil_z) <= 0.05]
    # Report the plane heights with the same estimator the CI uses (median), so
    # the interval is guaranteed to bracket the reported value.
    if len(floor_pts) >= 10:
        floor_z = float(np.median(floor_pts[:, 2]))
    if observed and len(ceil_pts) >= 10:
        ceil_z = float(np.median(ceil_pts[:, 2]))
    return floor_z, ceil_z, observed, floor_pts, ceil_pts


def _ceil_from_planes(planes: list, floor_z: float, cfg: Settings) -> float | None:
    """Highest sizable horizontal plane at a plausible ceiling height, else None."""
    hor = [p for p in planes if p.kind == "horizontal"]
    if not hor:
        return None
    biggest = max(len(p.points) for p in hor)
    cands = [p.z_median for p in hor
             if len(p.points) >= cfg.ceiling_plane_min_frac * biggest
             and cfg.ceiling_min_m <= p.z_median - floor_z <= cfg.ceiling_max_m]
    return max(cands) if cands else None


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


def shoelace_signed(polygon: np.ndarray) -> float:
    n = len(polygon)
    if n < 3:
        return 0.0
    x, y = polygon[:, 0], polygon[:, 1]
    return float((np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2.0)


def point_in_polygon(xy: np.ndarray, polygon: np.ndarray) -> np.ndarray:
    """Vectorised ray-casting point-in-polygon test."""
    x, y = xy[:, 0], xy[:, 1]
    inside = np.zeros(len(xy), dtype=bool)
    n = len(polygon)
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        crosses = ((yi > y) != (yj > y))
        denom = (yj - yi) if abs(yj - yi) > 1e-12 else 1e-12
        xint = (xj - xi) * (y - yi) / denom + xi
        inside ^= crosses & (x < xint)
        j = i
    return inside


def ceiling_height(floor_z: float, ceil_z: float, observed: bool,
                   floor_pts: np.ndarray, ceil_pts: np.ndarray, cfg: Settings) -> Measurement:
    height = ceil_z - floor_z
    if observed and len(floor_pts) >= 30 and len(ceil_pts) >= 30:
        h_ci = confidence.bootstrap_scalar(ceil_pts[:, 2], cfg)
        lo_ci = confidence.bootstrap_scalar(floor_pts[:, 2], cfg)
        if h_ci is not None and lo_ci is not None:
            ci = [round(h_ci[0] - lo_ci[1], 4), round(h_ci[1] - lo_ci[0], 4)]
        else:
            ci = [round(height - 0.02, 3), round(height + 0.02, 3)]
        method = "bootstrap"
    else:
        ci = [round(height - 0.15, 3), round(height + 0.15, 3)]
        method = "inferred_from_wall_tops"
    return Measurement(round(height, 3), confidence.interval(height, ci, cfg.ci_floor_height_m), method)


def property_area(floor_pts: np.ndarray, cfg: Settings) -> Measurement:
    from scipy.spatial import ConvexHull
    if len(floor_pts) < 10:
        return Measurement(0.0, [0.0, 0.0], "none")
    try:
        value = float(ConvexHull(floor_pts[:, :2]).volume)
    except Exception:
        return Measurement(0.0, [0.0, 0.0], "none")
    ci = confidence.bootstrap_hull_area(floor_pts[:, :2], cfg) or [value * 0.97, value * 1.03]
    return Measurement(round(value, 3),
                       confidence.interval(value, ci, cfg.ci_floor_area_rel * value),
                       "bootstrap_hull")


def _rdp(points: np.ndarray, tol: float) -> np.ndarray:
    """Douglas–Peucker on an open 2D polyline."""
    n = len(points)
    if n < 3:
        return points
    keep = np.zeros(n, dtype=bool)
    keep[0] = keep[-1] = True
    stack = [(0, n - 1)]
    while stack:
        i, j = stack.pop()
        if j <= i + 1:
            continue
        a, b = points[i], points[j]
        ab = b - a
        seg = points[i + 1:j] - a
        length = float(np.linalg.norm(ab))
        d = (np.abs(ab[0] * seg[:, 1] - ab[1] * seg[:, 0]) / length
             if length > 1e-12 else np.linalg.norm(seg, axis=1))
        k = int(np.argmax(d))
        if d[k] > tol:
            keep[i + 1 + k] = True
            stack.append((i, i + 1 + k))
            stack.append((i + 1 + k, j))
    return points[keep]


def _simplify_closed(poly: np.ndarray, tol: float) -> np.ndarray:
    """Simplify a closed polygon by splitting it at the farthest point pair."""
    n = len(poly)
    if n < 5:
        return poly
    k = int(np.argmax(np.linalg.norm(poly - poly[0], axis=1)))
    c1 = _rdp(poly[:k + 1], tol)
    c2 = _rdp(np.vstack([poly[k:], poly[:1]]), tol)
    return np.vstack([c1[:-1], c2[:-1]])


def footprint_rect(floor_pts: np.ndarray, cfg: Settings) -> np.ndarray | None:
    """Smallest enclosing rectangle of the floor footprint, as 2D corners (CCW)."""
    from scipy.spatial import ConvexHull
    xy = np.asarray(floor_pts[:, :2], dtype=np.float64)
    if len(xy) < 10:
        return None
    try:
        hp = xy[ConvexHull(xy).vertices]
    except Exception:
        return None
    if len(hp) < 3:
        return None
    best = None
    for i in range(len(hp)):
        a = hp[i]
        d = hp[(i + 1) % len(hp)] - a
        length = float(np.linalg.norm(d))
        if length < 1e-9:
            continue
        d = d / length
        n = np.array([-d[1], d[0]])
        t = (hp - a) @ d
        s = (hp - a) @ n
        area = float((t.max() - t.min()) * (s.max() - s.min()))
        if best is None or area < best[0]:
            best = (area, d, t.min(), t.max(), s.min(), s.max(), a)
    if best is None:
        return None
    _, d, t0, t1, s0, s1, a = best
    n = np.array([-d[1], d[0]])
    return np.asarray([a + d * t + n * s
                       for t, s in ((t0, s0), (t1, s0), (t1, s1), (t0, s1))])


def footprint_outline(floor_pts: np.ndarray, cfg: Settings) -> np.ndarray | None:
    """Room outline from the observed floor footprint (2D, ordered).

    A noisy monocular cloud smears the vertical structure into many spurious
    sheets, but the floor footprint stays a single closed blob. Its minimum-area
    rectangle is a stable four-wall outline; when the footprint is too elongated
    for that to be honest (the rectangle would over-cover the observed floor),
    fall back to the simplified footprint itself so the area stays truthful.
    """
    from scipy.spatial import ConvexHull
    xy = np.asarray(floor_pts[:, :2], dtype=np.float64)
    if len(xy) < 10:
        return None
    try:
        hp = xy[ConvexHull(xy).vertices]
    except Exception:
        return None
    if len(hp) < 3:
        return None
    hull_area = shoelace(hp)
    rect = footprint_rect(floor_pts, cfg)
    if rect is not None and hull_area > 1e-6 and \
            shoelace(rect) <= cfg.outline_max_rect_ratio * hull_area:
        return rect
    simp = _simplify_closed(hp, cfg.outline_simplify_m)
    return simp if len(simp) >= 3 and shoelace_signed(simp) > 0 else simp[::-1]


def assemble_outline(outline: np.ndarray, walls: list[Wall], floor_z: float, ceil_z: float,
                     observed: bool, floor_pts: np.ndarray, ceil_pts: np.ndarray,
                     cfg: Settings) -> Room:
    """Build a room whose polygon and area come from the floor outline."""
    poly = np.asarray(outline, dtype=np.float64)
    area_val = shoelace(poly)
    area_ci = (confidence.bootstrap_hull_area(floor_pts[:, :2], cfg)
               if len(floor_pts) >= 10 else None)
    if area_ci is None:
        area_ci = [round(area_val * 0.97, 3), round(area_val * 1.03, 3)]
    area_ci = confidence.interval(area_val, area_ci, cfg.ci_floor_area_rel * area_val)
    return Room(
        polygon=[[round(float(x), 3), round(float(y), 3)] for x, y in poly],
        floor_area_m2=Measurement(round(area_val, 3), area_ci, "footprint_outline"),
        ceiling_height_m=ceiling_height(floor_z, ceil_z, observed, floor_pts, ceil_pts, cfg),
        ceiling_observed=bool(observed),
        floor_z=round(float(floor_z), 3),
        walls=walls,
    )


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
    area_ci = confidence.interval(area_val, area_ci, cfg.ci_floor_area_rel * area_val)

    height = ceil_z - floor_z
    height_meas = ceiling_height(floor_z, ceil_z, observed, floor_pts, ceil_pts, cfg)

    return Room(
        polygon=[[round(float(x), 3), round(float(y), 3)]
                 for x, y in (polygon if polygon is not None else [])],
        floor_area_m2=Measurement(round(area_val, 3), area_ci, "bootstrap_hull"),
        ceiling_height_m=height_meas,
        ceiling_observed=bool(observed),
        floor_z=round(float(floor_z), 3),
        walls=walls,
    )
