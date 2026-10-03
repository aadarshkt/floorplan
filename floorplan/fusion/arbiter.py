"""Path arbiter: pick (or note) the best reconstruction when several are available.

Each candidate cloud is scored by how much usable structure it contains: the
number of well-formed vertical (wall) and horizontal (floor/ceiling) planes plus
a mild density prior. The winner is recorded in provenance, so the benchmark and
the report can always say *which* code path produced a number.
"""
from __future__ import annotations

import math

import numpy as np

from floorplan.config import Settings
from floorplan.fusion.ir import Reconstruction
from floorplan.geometry import planes as planes_mod


def score(points: np.ndarray, cfg: Settings) -> dict:
    if len(points) < 100:
        return {"score": -1.0, "n_vertical": 0, "n_horizontal": 0, "n_points": len(points)}
    pls = planes_mod.extract_planes(points, cfg)
    n_v = sum(1 for p in pls if p.kind == "vertical")
    n_h = sum(1 for p in pls if p.kind == "horizontal")
    s = 3.0 * min(n_v, 8) + 2.0 * min(n_h, 2) + math.log10(max(len(points), 1))
    return {"score": round(float(s), 3), "n_vertical": n_v, "n_horizontal": n_h,
            "n_points": int(len(points))}


def choose(candidates: list[Reconstruction], cfg: Settings,
           verbose: bool = False) -> tuple[Reconstruction, list[dict]]:
    if not candidates:
        raise ValueError("arbiter got no candidates")
    scored = []
    for c in candidates:
        s = score(np.asarray(c.points), cfg)
        scored.append((s["score"], c, s))
    scored.sort(key=lambda t: -t[0])
    best_score, best, best_s = scored[0]
    table = [{"path": c.path_chosen, **s} for _, c, s in scored]
    if verbose:
        for row in table:
            print(f"[arbiter] {row['path']}: score={row['score']} "
                  f"(walls={row['n_vertical']}, horiz={row['n_horizontal']}, pts={row['n_points']})")
        print(f"[arbiter] chose {best.path_chosen}")
    best.notes = {**best.notes, "arbiter": table, "winner": best.path_chosen}
    return best, table
