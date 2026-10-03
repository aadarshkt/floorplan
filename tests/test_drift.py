import numpy as np
import open3d as o3d

from floorplan import drift
from floorplan.config import Settings


def _loop_path(n=60):
    """A square walk that returns to its start, viewing direction along travel."""
    side = n // 4
    legs = [np.array(d, float) for d in ((1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0))]
    pos, fwd, p = [], [], np.zeros(3)
    for d in legs:
        for _ in range(side):
            pos.append(p.copy()); fwd.append(d)
            p = p + d * 0.5
    return np.array(pos), np.array(fwd)


def test_revisit_is_a_candidate_but_a_pause_is_not():
    pos, fwd = _loop_path()
    cfg = Settings(pg_min_gap_kf=10, pg_min_travel_m=3.0, pg_loop_radius_m=1.0, pg_min_view_dot=0.3)
    # walking in a square: the last keyframes come back near the first, facing it
    pos[-1] = pos[0] + [0.2, 0.0, 0.0]; fwd[-1] = fwd[0]
    assert (0, len(pos) - 1) in drift._candidates(pos, fwd, cfg)
    # standing still is not a loop (no real excursion)
    still = np.zeros((40, 3)); f = np.tile([1.0, 0, 0], (40, 1))
    assert drift._candidates(still, f, cfg) == []


def test_interpolated_correction_is_identity_without_motion_and_smooth_otherwise():
    kf = np.array([0, 10, 20])
    corr = np.tile(np.eye(4), (3, 1, 1))
    assert np.allclose(drift._interpolate(corr, kf, 21), np.eye(4))
    corr[2, :3, 3] = [0.0, 0.2, 0.0]
    out = drift._interpolate(corr, kf, 21)
    assert np.allclose(out[20, :3, 3], [0, 0.2, 0]) and np.allclose(out[10, :3, 3], 0)
    assert np.allclose(out[15, :3, 3], [0, 0.1, 0])


def test_icp_verification_recovers_an_offset_and_rejects_a_non_overlap():
    rng = np.random.default_rng(0)
    # an L-shaped corner (two walls + floor) so ICP is not degenerate
    wall_a = np.c_[rng.uniform(0, 3, 4000), np.zeros(4000), rng.uniform(0, 2, 4000)]
    wall_b = np.c_[np.zeros(4000), rng.uniform(0, 3, 4000), rng.uniform(0, 2, 4000)]
    floor = np.c_[rng.uniform(0, 3, 4000), rng.uniform(0, 3, 4000), np.zeros(4000)]
    pts = np.vstack([wall_a, wall_b, floor])

    def cloud(p):
        c = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(p))
        c = c.voxel_down_sample(0.05)
        c.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
        return c

    cfg = Settings()
    src, tgt = cloud(pts), cloud(pts + [0.08, -0.05, 0.02])      # target shifted by the "drift"
    got = drift._icp_verify(src, tgt, np.eye(4), cfg)
    assert got is not None
    assert np.allclose(got[0][:3, 3], [0.08, -0.05, 0.02], atol=0.02)
    far = cloud(pts + [40.0, 0, 0])
    assert drift._icp_verify(src, far, np.eye(4), cfg) is None
