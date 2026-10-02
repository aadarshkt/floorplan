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


def test_opening_detection_door():
    cfg = Settings()
    xs = np.linspace(0.0, 4.0, 400)
    keep = (xs < 1.5) | (xs > 2.5)
    zs = np.random.default_rng(0).uniform(0.1, 2.0, int(keep.sum()))
    pts = np.stack([xs[keep], np.zeros(int(keep.sum())), zs], axis=1)
    ops = openings.detect(np.array([0.0, 0.0]), np.array([4.0, 0.0]), pts,
                          floor_z=0.0, cfg=cfg)
    assert len(ops) == 1
    assert ops[0].width_m == pytest.approx(1.0, abs=0.15)
    assert ops[0].kind == "door"
