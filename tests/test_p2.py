import numpy as np
import pytest

from floorplan.config import Settings
from floorplan.fusion import arbiter, scale
from floorplan.fusion.colmap_path import _parse_cameras, _quat_wxyz_to_R
from floorplan.fusion.ir import NATIVE_METRIC, NONE_PRIOR, SCALED
from floorplan.ingest import detect, photos


def _room_cloud():
    pts = []
    x, y = np.meshgrid(np.linspace(0, 4, 40), np.linspace(0, 3, 30))
    pts.append(np.stack([x.ravel(), y.ravel(), np.zeros(x.size)], 1))
    pts.append(np.stack([x.ravel(), y.ravel(), np.full(x.size, 2.5)], 1))
    for x0, y0, x1, y1 in [(0, 0, 4, 0), (4, 0, 4, 3), (4, 3, 0, 3), (0, 3, 0, 0)]:
        t = np.linspace(0, 1, 80)
        z = np.linspace(0, 2.5, 40)
        T, Z = np.meshgrid(t, z)
        pts.append(np.stack([x0 + (x1 - x0) * T.ravel(), y0 + (y1 - y0) * T.ravel(),
                             Z.ravel()], 1))
    return np.concatenate(pts, 0)


def test_quat_wxyz_identity():
    assert np.allclose(_quat_wxyz_to_R(1, 0, 0, 0), np.eye(3))


def test_parse_cameras_simple_radial(tmp_path):
    p = tmp_path / "cameras.txt"
    p.write_text("# Camera list\n1 SIMPLE_RADIAL 720 960 1152.0 360.0 480.0 -0.01\n")
    K, size = _parse_cameras(p)
    assert K[0, 0] == pytest.approx(1152.0)
    assert K[0, 2] == pytest.approx(360.0)
    assert size == (720, 960)


def test_detect_tier_photos_and_video(tmp_path):
    folder = tmp_path / "room_photos"
    folder.mkdir()
    for i in range(3):
        (folder / f"{i}.jpg").write_bytes(b"\xff\xd8\xff")  # not decoded, just listed
    assert detect.detect_tier(folder) == "photos"
    vid = tmp_path / "walk.mov"
    vid.write_bytes(b"\x00")
    assert detect.detect_tier(vid) == "video"


def test_photos_load_falls_back_to_fov(tmp_path):
    from PIL import Image
    folder = tmp_path / "imgs"
    folder.mkdir()
    for i in range(2):
        Image.new("RGB", (64, 48), (10, 20, 30)).save(folder / f"{i}.png")
    ps = photos.load(folder)
    assert ps.n_images == 2
    assert ps.size == (64, 48)
    assert ps.f_px is None and ps.fov_deg is not None


def test_scale_anchor_ceiling_height():
    cfg = Settings(scale_ref_m=2.5, scale_ref_kind="ceiling_height")
    pts = _room_cloud()  # vertical extent 2.5 already -> scale ~1
    # anchor() orients before measuring height; a 4x3x2.5 box is ambiguous without a hint
    scaled, s, ref, notes = scale.anchor(pts, cfg, up_hint=np.array([0.0, 0.0, 1.0]))
    assert ref == SCALED
    assert s == pytest.approx(1.0, abs=0.05)


def test_scale_anchor_none_prior():
    cfg = Settings()
    pts = np.stack([np.linspace(0, 1, 500), np.zeros(500), np.linspace(0, 2.0, 500)], 1)
    scaled, s, ref, notes = scale.anchor(pts, cfg)
    assert ref == NONE_PRIOR
    ext = np.percentile(scaled[:, 2], 98) - np.percentile(scaled[:, 2], 2)
    assert ext == pytest.approx(cfg.default_ceiling_m, rel=0.02)


def test_scale_anchor_via_reference_recovers_scale():
    cfg = Settings()
    target = _room_cloud()
    source = target * 0.5  # half-size copy
    scaled, s, ref, notes = scale.anchor(source, cfg, reference_points=target,
                                         up_hint=np.array([0.0, 0.0, 1.0]))
    assert ref == SCALED
    assert s == pytest.approx(2.0, rel=0.1)


def test_arbiter_prefers_planar_cloud():
    cfg = Settings()
    room = _room_cloud()
    blob = np.random.default_rng(1).normal(0, 3.0, (room.shape[0], 3))
    from floorplan.fusion.ir import Reconstruction
    a = Reconstruction(points=blob, tier="video", path_chosen="blob", scale_reference=NATIVE_METRIC)
    b = Reconstruction(points=room, tier="video", path_chosen="room", scale_reference=SCALED)
    winner, table = arbiter.choose([a, b], cfg)
    assert winner.path_chosen == "room"
    assert len(table) == 2


def test_monodepth_without_backend_raises(tmp_path):
    from PIL import Image
    from floorplan.fusion import monodepth
    folder = tmp_path / "imgs"
    folder.mkdir()
    Image.new("RGB", (32, 24), (5, 5, 5)).save(folder / "0.png")
    ps = photos.load(folder)
    if monodepth.available_backends():  # a learned model is installed; skip
        return
    with pytest.raises(RuntimeError):
        monodepth.reconstruct(ps, Settings(), tmp_path, reference_points=None)


def test_monodepth_cross_tier_returns_reference(tmp_path):
    from PIL import Image
    from floorplan.fusion import monodepth
    folder = tmp_path / "imgs"
    folder.mkdir()
    Image.new("RGB", (32, 24), (5, 5, 5)).save(folder / "0.png")
    ps = photos.load(folder)
    ref = _room_cloud()
    # Force the cross-tier branch (engine="colmap" skips the metric-depth backend,
    # which may or may not be installed).
    r = monodepth.reconstruct(ps, Settings(engine="colmap"), tmp_path, reference_points=ref)
    assert r.scale_reference == SCALED
    assert len(r.points) == len(ref)


def _box_frames(scales):
    """Cameras panning inside a 4 x 3 m room; each frame's depth is off by its own scale."""
    from floorplan.fusion.colmap_path import ImagePose
    h, w, f = 60, 80, 70.0
    frames = []
    for k, s in enumerate(scales):
        yaw = 0.06 * k
        C = np.array([0.3 * np.sin(0.4 * k), 0.2 * np.cos(0.3 * k), 0.0])
        # camera looks along world +x rotated by yaw; camera y = world -z (down)
        fwd = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        right = np.array([np.sin(yaw), -np.cos(yaw), 0.0])
        R = np.stack([right, [0.0, 0.0, -1.0], fwd])
        v, u = np.mgrid[0:h, 0:w]
        rays = np.stack([(u - w / 2) / f, (v - h / 2) / f, np.ones_like(u, float)], -1) @ R
        t_hits = []
        for ax, lim in ((0, 2.0), (0, -2.0), (1, 1.5), (1, -1.5), (2, 1.2), (2, -1.3)):
            with np.errstate(divide="ignore", invalid="ignore"):
                tt = (lim - C[ax]) / rays[..., ax]
            t_hits.append(np.where(tt > 0, tt, np.inf))
        z = np.min(t_hits, axis=0)            # ray length with unit z component = depth
        im = ImagePose(name=f"{k:06d}", R=R, t=-R @ C, centre=C)
        frames.append({"im": im, "d": z / s, "K": (f, f, w / 2, h / 2),
                       "mask": np.ones((h, w), bool)})
    return frames


def test_joint_scales_removes_frame_wobble():
    from floorplan.fusion import sfm_depth
    rng = np.random.default_rng(0)
    true = np.exp(rng.normal(0, 0.10, 24))       # ~10 % frame-to-frame wobble
    frames = _box_frames(true)
    order = list(range(len(frames)))
    noisy_kp = true * np.exp(rng.normal(0, 0.08, len(true)))  # keypoint scales: noisy
    a = sfm_depth.joint_scales(frames, order, noisy_kp, Settings(), stride=2)
    before = np.std(np.log(noisy_kp / true))
    after = np.std(np.log(a / true))
    assert after < 0.5 * before


def test_parallax_level_recovers_uniform_bias():
    from floorplan.fusion import sfm_depth
    frames = _box_frames(np.ones(30))
    a = np.full(len(frames), 1.07)               # every frame 7 % too deep
    c = sfm_depth.parallax_level(frames, list(range(len(frames))), a, stride=2)
    assert abs(1.07 * c - 1.0) < 0.02
