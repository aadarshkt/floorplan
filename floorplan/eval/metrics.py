"""Metric primitives: optimal matching of produced vs measured elements + errors.

Wall lengths and openings are unordered sets, so we solve a rectangular linear
assignment (Hungarian) that minimises total absolute error, then report the
per-pair error distribution. This means the harness never penalises the pipeline
merely for ordering differences.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import linear_sum_assignment


def assignment(cost: np.ndarray) -> list[tuple[int, int]]:
    if cost.size == 0:
        return []
    r, c = linear_sum_assignment(cost)
    return list(zip(r.tolist(), c.tolist()))


def wall_length_errors(produced: list[float], gt: list[float]) -> dict:
    p = np.asarray(produced, dtype=float)
    g = np.asarray(gt, dtype=float)
    out = {"n_produced": int(len(p)), "n_gt": int(len(g)), "n_matched": 0,
           "mean_abs_cm": None, "max_abs_cm": None, "errors_cm": [], "pairs": []}
    if len(p) == 0 or len(g) == 0:
        return out
    pairs = assignment(np.abs(p[:, None] - g[None, :]))
    errs = [abs(float(p[i]) - float(g[j])) * 100.0 for i, j in pairs]
    out.update(
        n_matched=len(pairs),
        mean_abs_cm=round(float(np.mean(errs)), 2),
        max_abs_cm=round(float(np.max(errs)), 2),
        errors_cm=[round(e, 2) for e in errs],
        pairs=[[int(i), int(j)] for i, j in pairs],
    )
    return out


def opening_errors(produced: list[dict], gt: list[dict], tol_cm: float) -> dict:
    out = {"n_produced": len(produced), "n_gt": len(gt), "n_matched": 0,
           "within_tol": 0, "pass_rate": None, "missed": len(gt),
           "phantom": len(produced), "errors_cm": []}
    if not gt:
        out["pass_rate"] = 1.0
        return out
    if not produced:
        out["pass_rate"] = 0.0
        return out
    P, G = len(produced), len(gt)
    cost = np.zeros((P, G))
    for i, po in enumerate(produced):
        for j, go in enumerate(gt):
            c = abs(float(po["width_m"]) - float(go["width_m"])) * 100.0
            if po.get("type") != go.get("type"):
                c += 50.0  # discourage type-mismatched pairings
            cost[i, j] = c
    pairs = assignment(cost)
    errs = [abs(float(produced[i]["width_m"]) - float(gt[j]["width_m"])) * 100.0
            for i, j in pairs]
    within = sum(1 for e in errs if e <= tol_cm)
    out.update(
        n_matched=len(pairs), within_tol=within,
        pass_rate=round(within / max(G, 1), 3),
        missed=G - len(pairs), phantom=P - len(pairs),
        errors_cm=[round(e, 2) for e in errs],
    )
    return out


def repeatability(a: dict, b: dict) -> dict:
    """Compare two runs of the same room (produced measurements only)."""
    wl = wall_length_errors(a["walls"], b["walls"])
    ceil_cm = (abs(a["ceiling"] - b["ceiling"]) * 100.0
               if a["ceiling"] is not None and b["ceiling"] is not None else None)
    area_rel = (abs(a["area"] - b["area"]) / max(a["area"], 1e-9) * 100.0
                if a["area"] and b["area"] else None)
    return {
        "max_wall_cm": wl["max_abs_cm"],
        "mean_wall_cm": wl["mean_abs_cm"],
        "ceiling_cm": round(ceil_cm, 2) if ceil_cm is not None else None,
        "area_rel_pct": round(area_rel, 3) if area_rel is not None else None,
    }
