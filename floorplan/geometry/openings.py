"""Opening (door / window) detection along a wall.

Missed and phantom openings both count as misses, so an opening needs positive
evidence, not just missing points (a wall patch the scanner never looked at is
also "missing"):

  surface  only points on the wall face (|offset| <= opening_surface_m), on a
           grid of 2 cm along the wall x 10 cm in height
  door     columns empty from just above the floor to door height, AND the gap
           is seen through: the scan has points beyond the wall inside it
  window   columns empty at window height with wall below the gap, AND either
           wall above it (a lintel) or a view through it
  width    measured between the last surface points on either side (the jambs),
           not rounded to the grid
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from floorplan.config import Settings


@dataclass
class Opening:
    kind: str                  # "door" | "window"
    center_along_wall: float   # metres from wall start
    width_m: float
    sill_m: float              # height of the gap's lower edge above floor
    head_m: float              # height of the gap's upper edge above floor
    seen_through: bool = False


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    d = np.diff(np.concatenate([[0], mask.astype(np.int8), [0]]))
    return list(zip(np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]))   # [start, end)


def detect(wall_start: np.ndarray, wall_end: np.ndarray, wall_points: np.ndarray,
           floor_z: float, cfg: Settings, cloud: np.ndarray | None = None,
           inward: np.ndarray | None = None) -> list[Opening]:
    vec = np.asarray(wall_end) - np.asarray(wall_start)
    length = float(np.linalg.norm(vec))
    if length < cfg.min_window_width or len(wall_points) < 20:
        return []
    u = vec / length
    n = np.array([-u[1], u[0]])

    rel = wall_points[:, :2] - wall_start
    along, perp = rel @ u, rel @ n
    h = wall_points[:, 2] - floor_z
    face = (np.abs(perp) <= cfg.opening_surface_m) & (along >= 0) & (along <= length)
    a, hh = along[face], h[face]
    if len(a) < 20:
        return []

    bw, bh = 0.02, 0.10
    na = max(1, int(np.ceil(length / bw)))
    hb = np.arange(0.0, 2.4 + 1e-9, bh)
    occ = np.zeros((na, len(hb)), dtype=bool)
    ia = np.clip((a / bw).astype(int), 0, na - 1)
    ok = (hh >= 0) & (hh < hb[-1] + bh)
    occ[ia[ok], np.clip((hh[ok] / bh).astype(int), 0, len(hb) - 1)] = True

    def rows(lo, hi):
        return slice(int(lo / bh), int(np.ceil(hi / bh)))

    door_col = ~occ[:, rows(0.3, 1.8)].any(axis=1)
    win_col = (~occ[:, rows(1.0, 1.5)].any(axis=1)) & occ[:, rows(0.2, 0.8)].any(axis=1)
    lintel = occ[:, rows(1.9, 2.2)].any(axis=1)

    def close_small_holes(m):  # one stray point must not split an opening
        m = m.copy()
        for s, e in _runs(~m):
            if e - s <= 2 and s > 0 and e < len(m):
                m[s:e] = True
        return m

    door_col, win_col = close_small_holes(door_col), close_small_holes(win_col & ~door_col)

    def seen_through(s0, s1, h0, h1) -> bool:
        if cloud is None:
            return False
        r = cloud[:, :2] - wall_start
        al, pp = r @ u, r @ n
        z = cloud[:, 2] - floor_z
        out = -pp if inward is not None and float(inward @ n) > 0 else (
            pp if inward is not None else np.abs(pp))
        m = ((al > s0 + 0.05) & (al < s1 - 0.05) & (z > h0) & (z < h1)
             & (out > 0.2) & (out < 6.0))
        return int(m.sum()) >= cfg.opening_seen_min

    def jambs(s, e):
        """Gap edges from the last face points either side of columns [s, e)."""
        s0, s1 = s * bw, e * bw
        left = a[(a < s0 + bw) & (a > s0 - 0.3)]
        right = a[(a > s1 - bw) & (a < s1 + 0.3)]
        lo = float(left.max()) if len(left) and s > 0 else (0.0 if s == 0 else s0)
        hi = float(right.min()) if len(right) and e < na else (length if e == na else s1)
        return (lo, hi) if hi > lo else (s0, s1)

    out: list[Opening] = []
    for s, e in _runs(door_col):
        s0, s1 = jambs(s, e)
        if s1 - s0 < cfg.min_opening_width or s1 - s0 > cfg.max_opening_width:
            continue
        # touching a wall end: only a doorway edge (the layout closes a doorway
        # between rooms with a short wall whose ends are the jambs) qualifies
        if (s == 0 or e == na) and (e - s) < 0.7 * na:
            continue
        st = seen_through(s0, s1, 0.3, 1.8)
        if not st:
            continue
        seg = occ[s:e]
        filled = np.nonzero(seg.any(axis=0))[0]
        head = float(hb[filled[filled * bh >= 1.8].min()]) if (filled * bh >= 1.8).any() else 2.1
        out.append(Opening("door", (s0 + s1) / 2, s1 - s0, 0.0, round(head, 2), True))
    for s, e in _runs(win_col):
        s0, s1 = jambs(s, e)
        if s1 - s0 < cfg.min_window_width or s1 - s0 > cfg.max_opening_width:
            continue
        if (s == 0 or e == na) and (e - s) < 0.7 * na:
            continue
        st = seen_through(s0, s1, 1.0, 1.5)
        if not (st or lintel[s:e].mean() > 0.5):
            continue
        seg = occ[s:e]
        below = np.nonzero(seg[:, rows(0.0, 1.0)].any(axis=0))[0]
        sill = float((below.max() + 1) * bh) if len(below) else 0.9
        out.append(Opening("window", (s0 + s1) / 2, s1 - s0, round(sill, 2), 0.0, st))
    out.sort(key=lambda o: o.center_along_wall)
    return out
