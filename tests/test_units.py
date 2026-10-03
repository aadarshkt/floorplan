import math

import numpy as np
import pytest

from floorplan.config import Settings
from floorplan.fusion.depth_fusion import _frame_points, quat_to_matrix
from floorplan.geometry import openings, room


def test_quat_identity():
    assert np.allclose(quat_to_matrix([0, 0, 0, 1]), np.eye(3))


def test_quat_90_about_z():
    R = quat_to_matrix([0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    assert np.allclose(R @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-6)


def test_shoelace_unit_square():
    poly = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], float)
    assert room.shoelace(poly) == pytest.approx(1.0)


def test_unproject_center_pixel():
    depth = np.full((4, 4), 2000, np.uint16)
    conf = np.full((4, 4), 2, np.uint8)
    pts = _frame_points(depth, conf, fx=2.0, fy=2.0, cx=2.0, cy=2.0,
                        conf_min=1, depth_min=0.1, depth_max=8.0, px_stride=1)
    rounded = {tuple(np.round(p, 4)) for p in pts}
    assert (0.0, 0.0, 2.0) in rounded


def test_low_confidence_filtered():
    depth = np.full((4, 4), 2000, np.uint16)
    conf = np.zeros((4, 4), np.uint8)  # all low confidence
    pts = _frame_points(depth, conf, fx=2.0, fy=2.0, cx=2.0, cy=2.0,
                        conf_min=1, depth_min=0.1, depth_max=8.0, px_stride=1)
    assert len(pts) == 0


def _wall_with_gap():
    rng = np.random.default_rng(0)
    xs = rng.uniform(0.0, 4.0, 4000)
    keep = (xs < 1.5) | (xs > 2.5)
    xs = xs[keep]
    pts = np.stack([xs, np.zeros(len(xs)), rng.uniform(0.05, 2.3, len(xs))], axis=1)
    return pts, rng


def test_opening_detection_door():
    cfg = Settings()
    pts, rng = _wall_with_gap()
    # the scan saw the next room through the gap
    behind = np.stack([rng.uniform(1.6, 2.4, 200), np.full(200, -1.5),
                       rng.uniform(0.4, 1.7, 200)], axis=1)
    ops = openings.detect(np.array([0.0, 0.0]), np.array([4.0, 0.0]), pts,
                          floor_z=0.0, cfg=cfg, cloud=np.vstack([pts, behind]))
    assert len(ops) == 1
    assert ops[0].width_m == pytest.approx(1.0, abs=0.03)
    assert ops[0].kind == "door"


def test_unobserved_gap_is_not_a_door():
    """A wall patch nobody scanned looks like a gap; without a view through it
    it must not become a (phantom) door."""
    cfg = Settings()
    pts, _ = _wall_with_gap()
    ops = openings.detect(np.array([0.0, 0.0]), np.array([4.0, 0.0]), pts,
                          floor_z=0.0, cfg=cfg, cloud=pts)
    assert ops == []


def test_wall_matching_by_position_then_length():
    from floorplan.eval import metrics
    produced = [2.54, 0.60, 2.89]
    ids = ["r1-w0", "r1-w1", "r1-w2"]
    mids = [[0.0, 0.0], [1.0, 1.0], [3.0, 0.0]]
    # first measured wall is pinned to its plan position (renumbered as w9),
    # second has no id/position and pairs by length; w1 was never measured
    out = metrics.wall_errors_with_ids(produced, ids, [2.55, 2.88], ["r1-w9", None],
                                       produced_mids=mids, gt_mids=[[0.05, 0.0], None])
    assert out["pair_ids"] == [["r1-w0", "r1-w9"], ["r1-w2", None]]
    assert out["missed"] == 0 and out["phantom"] == 1
    assert out["max_abs_cm"] == pytest.approx(1.0, abs=1e-6)


def test_ground_truth_multi_room_and_ceiling_readings():
    from floorplan.eval.groundtruth import GroundTruth
    gt = GroundTruth.from_dict({"room_id": "r2", "ceiling_height_m": [2.80, 2.82, None],
                                "walls": [{"wall_id": "r2-w0", "length_m": 3.1},
                                          {"wall_id": "r2-w1", "length_m": None}]})
    assert gt.ceiling_height_m == pytest.approx(2.81)
    assert gt.wall_lengths_m == [3.1] and gt.wall_ids == ["r2-w0"]
