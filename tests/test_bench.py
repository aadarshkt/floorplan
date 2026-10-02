import json

import numpy as np
import pytest

from floorplan.bench import evaluate_capture, evaluate_repeatability
from floorplan.eval import metrics
from floorplan.eval.groundtruth import GroundTruth


def _result(walls, ceiling, observed, area, openings):
    open_objs = [
        {"opening_id": f"o{k}", "type": o["type"], "position_along_wall_m": 0.0,
         "width_m": {"value": o["width_m"], "ci95": [o["width_m"] - 0.01, o["width_m"] + 0.01],
                     "method": "bin"}}
        for k, o in enumerate(openings)
    ]
    wall_objs = []
    for i, (ln, ci) in enumerate(walls):
        wall_objs.append({
            "wall_id": f"w{i}",
            "start": [0.0, 0.0], "end": [ln, 0.0],
            "length_m": {"value": ln, "ci95": ci, "method": "bootstrap"},
            "openings": open_objs if i == 0 else [],
        })
    return {"rooms": [{
        "room_id": "room1",
        "polygon": [],
        "floor_area_m2": {"value": area, "ci95": [area - 0.1, area + 0.1],
                          "method": "bootstrap_hull"},
        "ceiling_height_m": {"value": ceiling, "ci95": [ceiling - 0.01, ceiling + 0.01],
                             "method": "bootstrap", "observed": observed},
        "walls": wall_objs,
    }]}


GATES = {"ceiling_height_abs_cm": 1.5, "opening_width_abs_cm": 2.0,
         "opening_pass_rate": 0.85, "opening_phantom_max": 1,
         "wall_length_abs_cm": 3.0, "wall_length_max_abs_cm": 6.0,
         "area_rel_pct": 5.0, "repeatability_max_cm": 1.0,
         "repeatability_rel_pct": 0.5, "ci_coverage_min": 0.85}


def test_wall_matching_is_order_free():
    errs = metrics.wall_length_errors([3.0, 5.0, 4.0], [5.0, 4.0, 3.0])
    assert errs["max_abs_cm"] == pytest.approx(0.0, abs=0.01)


def test_opening_matching_type_penalty():
    produced = [{"type": "door", "width_m": 0.82}, {"type": "window", "width_m": 1.2}]
    gt = [{"type": "window", "width_m": 1.2}, {"type": "door", "width_m": 0.82}]
    out = metrics.opening_errors(produced, gt, tol_cm=2.0)
    assert out["pass_rate"] == 1.0


def test_capture_passes_when_within_gates():
    result = _result(
        walls=[(4.0, [3.98, 4.02]), (3.0, [2.98, 3.02])],
        ceiling=2.42, observed=True, area=12.0,
        openings=[{"type": "door", "width_m": 0.82}],
    )
    gt = GroundTruth("room1", 2.42, 12.0, [4.0, 3.0], [{"type": "door", "width_m": 0.82}])
    rc = evaluate_capture("c1", "room1", result, gt, GATES, [])
    assert rc["passed"], rc["checks"]


def test_capture_fails_on_bad_ceiling():
    result = _result(
        walls=[(4.0, [3.98, 4.02])], ceiling=2.50, observed=True, area=12.0,
        openings=[],
    )
    gt = GroundTruth("room1", 2.42, 12.0, [4.0], [])
    rc = evaluate_capture("c1", "room1", result, gt, GATES, [])
    assert not rc["passed"]
    assert not rc["checks"]["ceiling"]


def test_phantom_opening_fails():
    result = _result(
        walls=[(4.0, [3.98, 4.02])], ceiling=2.42, observed=True, area=12.0,
        openings=[{"type": "door", "width_m": 0.8},
                  {"type": "door", "width_m": 0.9},
                  {"type": "door", "width_m": 1.05}],
    )
    gt = GroundTruth("room1", 2.42, 12.0, [4.0], [{"type": "door", "width_m": 0.82}])
    rc = evaluate_capture("c1", "room1", result, gt, GATES, [])
    assert rc["openings"]["phantom"] == 2
    assert not rc["passed"]


def test_repeatability():
    a = _result([(4.0, [3.98, 4.02])], 2.42, True, 12.0, [])
    b = _result([(4.005, [3.985, 4.025])], 2.42, True, 12.0, [])
    r = evaluate_repeatability("room1", a, b, GATES)
    assert r["passed"] and r["max_wall_cm"] < 1.0
