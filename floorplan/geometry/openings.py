"""Opening (door / window) detection along a wall.

Openings are runs of absent wall surface along the wall axis. A run that reaches
the floor is a door; a run confined to upper height with points below and above
is a window. Missed and phantom openings are both scored as misses, so we guard
against false gaps (occlusion) by requiring a minimum width and a floor check.
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


def detect(wall_start: np.ndarray, wall_end: np.ndarray, wall_points: np.ndarray,
           floor_z: float, cfg: Settings) -> list[Opening]:
    vec = wall_end - wall_start
    length = float(np.linalg.norm(vec))
    if length < 1e-6 or len(wall_points) < 20:
        return []
    u = vec / length
    n = np.array([-u[1], u[0]])

    rel = wall_points[:, :2] - wall_start
    along = rel @ u
    perp = rel @ n
    height = wall_points[:, 2] - floor_z

    # keep points near this wall's face
    m = (along >= -0.15) & (along <= length + 0.15) & (np.abs(perp) <= 0.25)
    along, height = along[m], height[m]
    if len(along) < 20:
        return []

    bin_sz = cfg.opening_bin
    n_bins = max(1, int(round(length / bin_sz)))
    occupied = np.zeros(n_bins, dtype=bool)
    idx = np.clip((along / bin_sz).astype(int), 0, n_bins - 1)
    occupied[idx] = True

    openings: list[Opening] = []
    i = 0
    while i < n_bins:
        if occupied[i]:
            i += 1
            continue
        j = i
        while j < n_bins and not occupied[j]:
            j += 1
        s0, s1 = i * bin_sz, j * bin_sz
        width = s1 - s0
        if width >= cfg.min_opening_width:
            seg = (along >= s0 - bin_sz) & (along < s1 + bin_sz)
            hs = height[seg]
            if len(hs) > 0:
                lo = float(np.percentile(hs, 5))
                hi = float(np.percentile(hs, 95))
            else:
                lo, hi = 0.0, cfg.door_height
            # door: surface absent right down to the floor
            kind = "door" if lo <= 0.25 else "window"
            openings.append(Opening(
                kind=kind, center_along_wall=float((s0 + s1) / 2.0),
                width_m=float(width), sill_m=round(lo, 3), head_m=round(hi, 3),
            ))
        i = j
    return openings
