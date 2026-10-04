"""Path arbiter: pick (or note) the best reconstruction when several are available.

Each candidate cloud is scored by how much usable structure it contains: the
number of well-formed vertical (wall) and horizontal (floor/ceiling) planes plus
a mild density prior. The winner is recorded in provenance, so the benchmark and
the report can always say *which* code path produced a number.
"""
from __future__ import annotations

import numpy as np

from floorplan.config import Settings
from floorplan.fusion.ir import Reconstruction
from floorplan.geometry import planes as planes_mod


def score(points: np.ndarray, cfg: Settings) -> dict:
    """Reward planarity *coverage*, not plane count.

    A noisy cloud yields many tiny spurious planes; the useful cloud has a few
    large ones (floor, ceiling, walls) that between them carry most of the
    points. So we score by the share of points lying in "large" planes (>=1% of
    the cloud) plus a small bonus for having enough of them.
    """
    n = len(points)
    if n < 100:
        return {"score": -1.0, "coverage": 0.0, "n_vertical": 0, "n_horizontal": 0,
                "n_points": n}
    pls = planes_mod.extract_planes(points, cfg)
    if not pls:
        return {"score": -1.0, "coverage": 0.0, "n_vertical": 0, "n_horizontal": 0,
                "n_points": n}
    big = [p for p in pls if len(p.points) >= 0.01 * n]
    coverage = sum(len(p.points) for p in big) / n
    n_v = sum(1 for p in big if p.kind == "vertical")
    n_h = sum(1 for p in big if p.kind == "horizontal")
    s = 10.0 * coverage + 3.0 * min(n_v, 6) + 2.0 * min(n_h, 2)
    return {"score": round(float(s), 3), "coverage": round(float(coverage), 3),
            "n_vertical": n_v, "n_horizontal": n_h, "n_points": int(n)}


def choose(candidates: list[Reconstruction], cfg: Settings,
           verbose: bool = False) -> tuple[Reconstruction, list[dict]]:
    if not candidates:
        raise ValueError("arbiter got no candidates")
    if len(candidates) == 1:
        best = candidates[0]
        table = [{"path": best.path_chosen, "n_points": len(best.points),
                  "selection": "only_successful_candidate"}]
        best.notes = {**best.notes, "arbiter": table, "winner": best.path_chosen}
        if verbose:
            print(f"[arbiter] chose {best.path_chosen}")
        return best, table
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
                  f"(walls={row['n_vertical']}, horiz={row['n_horizontal']}, "
                  f"coverage={row.get('coverage')}, pts={row['n_points']})")
        print(f"[arbiter] chose {best.path_chosen}")
    best.notes = {**best.notes, "arbiter": table, "winner": best.path_chosen}
    return best, table
