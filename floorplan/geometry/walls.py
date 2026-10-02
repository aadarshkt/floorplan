"""Wall vectorisation: vertical planes -> 2D wall segments.

Includes merging of the two faces of a single physical wall into one centreline
(otherwise every wall is counted twice) and orthogonal snapping.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from floorplan.config import Settings
from floorplan.geometry.planes import Plane
from floorplan.geometry.openings import Opening


@dataclass
class Wall:
    id: int
    start: np.ndarray              # (2,)
    end: np.ndarray                # (2,)
    normal: np.ndarray             # (2,) unit
    length: float
    plane_id: int
    points: np.ndarray             # (N, 3) full-height inliers (for openings)
    angle: float                   # [0, pi)
    band_points: np.ndarray | None = None  # (M, 3) points used for the line fit
    openings: list[Opening] = field(default_factory=list)


def _fit_line(xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (centroid, unit direction) of the best-fit 2D line."""
    c = xy.mean(axis=0)
    _, _, vt = np.linalg.svd(xy - c, full_matrices=False)
    d = vt[0]
    ln = float(np.linalg.norm(d))
    return c, (d / ln if ln > 1e-12 else np.array([1.0, 0.0]))


def _angle_diff(a: float, b: float) -> float:
    """Smallest signed difference between two undirected angles in [0, pi)."""
    return float((a - b + math.pi / 2) % math.pi - math.pi / 2)


def vectorize(vertical_planes: list[Plane], floor_z: float, cfg: Settings) -> list[Wall]:
    walls: list[Wall] = []
    for p in vertical_planes:
        pts = p.points
        band = pts[(pts[:, 2] >= floor_z + cfg.wall_band_low) &
                   (pts[:, 2] <= floor_z + cfg.wall_band_high)]
        if len(band) < 50:
            band = pts
        if len(band) < 30:
            continue
        xy = band[:, :2]
        c, d = _fit_line(xy)
        t = (xy - c) @ d
        t0, t1 = float(t.min()), float(t.max())
        length = t1 - t0
        if length < cfg.min_wall_len:
            continue
        start = c + d * t0
        end = c + d * t1
        nrm = np.array([-d[1], d[0]])
        walls.append(Wall(
            id=len(walls), start=start, end=end, normal=nrm, length=length,
            plane_id=p.id, points=pts, angle=math.atan2(d[1], d[0]) % math.pi,
            band_points=band,
        ))
    return walls


def merge_double_walls(walls: list[Wall], cfg: Settings) -> list[Wall]:
    """Merge near-parallel, near-coincident segments (the two faces of one wall)."""
    if not walls:
        return walls
    used = [False] * len(walls)
    merged: list[Wall] = []
    ang_tol = math.radians(cfg.wall_merge_angle_deg)

    for i, w in enumerate(walls):
        if used[i]:
            continue
        group = [w]
        used[i] = True
        wid = (w.end - w.start)
        wlen = float(np.linalg.norm(wid)) or 1.0
        wdir = wid / wlen
        wc = (w.start + w.end) / 2.0
        wperp = np.array([-wdir[1], wdir[0]])
        for j in range(i + 1, len(walls)):
            if used[j]:
                continue
            v = walls[j]
            if abs(_angle_diff(w.angle, v.angle)) > ang_tol:
                continue
            vc = (v.start + v.end) / 2.0
            perp = abs(float((vc - wc) @ wperp))
            if perp > cfg.wall_merge_max_gap:
                continue
            # overlap along wdir
            w_ts = np.sort([float((w.start - wc) @ wdir), float((w.end - wc) @ wdir)])
            v_ts = np.sort([float((v.start - wc) @ wdir), float((v.end - wc) @ wdir)])
            overlap = min(w_ts[1], v_ts[1]) - max(w_ts[0], v_ts[0])
            if overlap <= 0.2:
                continue
            group.append(v)
            used[j] = True

        if len(group) == 1:
            merged.append(w)
            continue

        # average direction (sign-aligned), centroid, and extent
        dirs = []
        for g in group:
            d = (g.end - g.start)
            ln = float(np.linalg.norm(d)) or 1.0
            d = d / ln
            if dirs and float(d @ dirs[0]) < 0:
                d = -d
            dirs.append(d)
        d = np.mean(dirs, axis=0)
        d = d / (float(np.linalg.norm(d)) or 1.0)
        mids = np.array([(g.start + g.end) / 2.0 for g in group])
        c = mids.mean(axis=0)
        ts = []
        for g in group:
            ts.extend([float((g.start - c) @ d), float((g.end - c) @ d)])
        t0, t1 = min(ts), max(ts)
        pts = np.concatenate([g.points for g in group], axis=0)
        bands = np.concatenate([g.band_points for g in group if g.band_points is not None], axis=0) \
            if any(g.band_points is not None for g in group) else None
        merged.append(Wall(
            id=len(merged), start=c + d * t0, end=c + d * t1,
            normal=np.array([-d[1], d[0]]), length=float(t1 - t0),
            plane_id=group[0].plane_id, points=pts,
            angle=math.atan2(d[1], d[0]) % math.pi, band_points=bands,
        ))
    for k, m in enumerate(merged):
        m.id = k
    return merged


def snap_orthogonal(walls: list[Wall], cfg: Settings) -> list[Wall]:
    """Snap walls to a dominant axis pair; keep (and flag) genuinely slanted walls."""
    if not cfg.snap_orthogonal or not walls:
        return walls
    primary = max(walls, key=lambda w: w.length).angle
    secondary = (primary + math.pi / 2) % math.pi
    tol = math.radians(cfg.ortho_snap_max_deg)
    for w in walls:
        dp, ds = abs(_angle_diff(w.angle, primary)), abs(_angle_diff(w.angle, secondary))
        if min(dp, ds) > tol:
            continue  # slanted: leave as-is (do not silently "correct")
        target = primary if dp <= ds else secondary
        mid = (w.start + w.end) / 2.0
        d = np.array([math.cos(target), math.sin(target)])
        w.start = mid - d * w.length / 2.0
        w.end = mid + d * w.length / 2.0
        w.normal = np.array([-d[1], d[0]])
        w.angle = target % math.pi
    return walls
