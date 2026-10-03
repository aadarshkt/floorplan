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
    scaled, s, ref, notes = scale.anchor(pts, cfg)
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
    scaled, s, ref, notes = scale.anchor(source, cfg, reference_points=target)
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
    r = monodepth.reconstruct(ps, Settings(), tmp_path, reference_points=ref)
    assert r.scale_reference == SCALED
    assert len(r.points) == len(ref)
