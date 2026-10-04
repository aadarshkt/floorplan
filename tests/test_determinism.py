"""The same point set must give the same cloud, whatever order or float noise it arrives in."""
import numpy as np
import open3d as o3d

from floorplan.fusion.depth_fusion import canonical_order


def _cloud(p):
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))


def test_canonical_order_ignores_order_and_float_noise():
    rng = np.random.default_rng(0)
    pts = rng.uniform(-3, 3, size=(5000, 3))
    shuffled = pts[rng.permutation(len(pts))] + rng.normal(0, 1e-13, pts.shape)
    a = np.asarray(canonical_order(_cloud(pts)).points)
    b = np.asarray(canonical_order(_cloud(shuffled)).points)
    assert a.shape == b.shape
    assert np.array_equal(a, b)


def test_canonical_order_is_sorted_and_keeps_points():
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 1.0, 5.0], [0.0, 1.0, 4.0]])
    out = np.asarray(canonical_order(_cloud(pts)).points)
    assert out.tolist() == [[0.0, 1.0, 4.0], [0.0, 1.0, 5.0], [0.0, 2.0, 0.0], [1.0, 0.0, 0.0]]
