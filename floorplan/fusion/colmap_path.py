"""COLMAP reconstruction path (photos / video tiers).

Runs COLMAP SfM (features -> matching -> mapping) to recover camera poses and a
sparse point cloud, then:
  * uses dense MVS (``patch_match_stereo``) when the build has CUDA, else
  * falls back to a CPU plane-sweep MVS (``fusion.mvs``) driven by the SfM poses,
  * and finally to the raw sparse cloud.

The result is in an arbitrary (scale-free) frame; metric scale is attached later
by the scale anchor. The COLMAP workspace is always kept on disk so a run is
reproducible and inspectable.

Works with COLMAP 3.x and 4.x: the CPU/SIFT option names changed between
releases, so we probe the binary's own help text once and pick the right flags.
"""
from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from floorplan.config import Settings
from floorplan.ingest.photos import PhotoSet


@dataclass
class ImagePose:
    name: str
    R: np.ndarray      # (3,3) world -> camera
    t: np.ndarray      # (3,)  world -> camera:  X_cam = R X_world + t
    centre: np.ndarray  # (3,)  camera centre in world (-R^T t)
    obs_xy: np.ndarray | None = None    # (M,2) observed keypoint pixels
    obs_xyz: np.ndarray | None = None   # (M,3) matching sparse world points


@dataclass
class ColmapResult:
    points: np.ndarray            # (N, 3) raw, scale-free
    conf: np.ndarray | None
    n_images: int
    n_registered: int
    n_points: int
    dense: str                    # "mvs" | "cpu_mvs" | "sparse"
    model_dir: Path
    images: list[ImagePose] = field(default_factory=list)
    K: np.ndarray | None = None
    size: tuple[int, int] | None = None
    up_hint: np.ndarray | None = None   # world up from the camera orientations
    model_sizes: list[int] = field(default_factory=list)  # registered images per sub-model

    @property
    def centres(self) -> np.ndarray:
        if not self.images:
            return np.zeros((0, 3))
        return np.asarray([im.centre for im in self.images])


_FLAG_CACHE: dict[str, list[str]] = {}


def available() -> bool:
    return shutil.which("colmap") is not None


def _run(cmd: list[str], verbose: bool) -> subprocess.CompletedProcess:
    # NOTE: do NOT force QT_QPA_PLATFORM=offscreen here. On macOS that makes
    # COLMAP try (and fail) to create an offscreen OpenGL context and abort;
    # with CPU SIFT (use_gpu 0) it runs headless fine without the override.
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=3600)
    if verbose and res.returncode != 0:
        tail = (res.stderr or res.stdout or "").strip().splitlines()[-6:]
        print(f"[colmap] command failed ({' '.join(cmd[:2])}): " + " | ".join(tail))
    return res


def _cpu_flag(subcmd: str, new: str, old: str) -> list[str]:
    key = f"{subcmd}:{new}:{old}"
    if key in _FLAG_CACHE:
        return _FLAG_CACHE[key]
    help_txt = ""
    try:
        help_txt = subprocess.run(["colmap", subcmd, "-h"], capture_output=True,
                                  text=True, timeout=60).stdout
    except Exception:
        pass
    flag = new if new in help_txt else (old if old in help_txt else None)
    _FLAG_CACHE[key] = [flag, "0"] if flag else []
    return _FLAG_CACHE[key]


def _quat_wxyz_to_R(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    n = qw * qw + qx * qx + qy * qy + qz * qz
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (qy * qy + qz * qz), s * (qx * qy - qz * qw), s * (qx * qz + qy * qw)],
        [s * (qx * qy + qz * qw), 1 - s * (qx * qx + qz * qz), s * (qy * qz - qx * qw)],
        [s * (qx * qz - qy * qw), s * (qy * qz + qx * qw), 1 - s * (qx * qx + qy * qy)],
    ])


def _parse_cameras(txt: Path) -> tuple[np.ndarray, tuple[int, int]]:
    """Return (K, (w,h)) for the first camera (single_camera assumption)."""
    for line in txt.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        p = line.split()
        w, h = int(p[2]), int(p[3])
        params = [float(v) for v in p[4:]]
        if p[1] in ("SIMPLE_PINHOLE", "SIMPLE_RADIAL"):
            fx = fy = params[0]
            cx, cy = params[1], params[2]
        else:  # PINHOLE / OPENCV / RADIAL
            fx, fy, cx, cy = params[0], params[1], params[2], params[3]
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        return K, (w, h)
    return np.eye(3), (0, 0)


def _parse_txt_model(model_dir: Path, verbose: bool
                     ) -> tuple[np.ndarray, np.ndarray, list[ImagePose], np.ndarray, tuple[int, int]]:
    txt = model_dir.parent / (model_dir.name + "_txt")
    if txt.exists():
        shutil.rmtree(txt)
    txt.mkdir(parents=True)  # COLMAP's TXT writer requires the dir to already exist
    _run(["colmap", "model_converter", "--input_path", str(model_dir),
          "--output_path", str(txt), "--output_type", "TXT"], verbose)

    xyz, errs = [], []
    pts_by_id: dict[int, int] = {}
    p = txt / "points3D.txt"
    if p.exists():
        for line in p.read_text().splitlines():
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            pts_by_id[int(parts[0])] = len(xyz)
            xyz.append([float(parts[1]), float(parts[2]), float(parts[3])])
            errs.append(float(parts[7]))

    images: list[ImagePose] = []
    q = txt / "images.txt"
    if q.exists():
        # Pose lines (10 fields) and their following 2D-observation lines alternate;
        # we must keep blank observation lines to stay aligned.
        raw = q.read_text().splitlines()
        i = 0
        while i < len(raw):
            s = raw[i].strip()
            if not s or s.startswith("#"):
                i += 1
                continue
            parts = s.split()
            if len(parts) < 10:
                i += 1
                continue
            qw, qx, qy, qz, tx, ty, tz = (float(v) for v in parts[1:8])
            R = _quat_wxyz_to_R(qw, qx, qy, qz)
            t = np.array([tx, ty, tz])
            i += 1
            obs_xy, obs_xyz = [], []
            if i < len(raw):
                toks = raw[i].split()
                for k in range(0, len(toks) - 2, 3):
                    try:
                        px, py, pid = float(toks[k]), float(toks[k + 1]), int(toks[k + 2])
                    except ValueError:
                        break
                    j = pts_by_id.get(pid)
                    if j is not None:
                        obs_xy.append([px, py])
                        obs_xyz.append(xyz[j])
            i += 1
            images.append(ImagePose(
                name=parts[9], R=R, t=t, centre=-R.T @ t,
                obs_xy=np.asarray(obs_xy, dtype=np.float64) if obs_xy else None,
                obs_xyz=np.asarray(obs_xyz, dtype=np.float64) if obs_xyz else None,
            ))

    K, size = _parse_cameras(txt / "cameras.txt") if (txt / "cameras.txt").exists() else (np.eye(3), (0, 0))
    return (np.asarray(xyz, dtype=np.float64), np.asarray(errs, dtype=np.float64),
            images, K, size)


def _try_dense(workdir: Path, verbose: bool) -> Path | None:
    dense = workdir / "dense"
    steps = [
        ["colmap", "image_undistorter", "--image_path", str(workdir / "imgs"),
         "--input_path", str(workdir / "sparse" / "0"),
         "--output_path", str(dense), "--output_type", "COLMAP"],
        ["colmap", "patch_match_stereo", "--workspace_path", str(dense),
         "--workspace_format", "COLMAP", "--PatchMatchStereo.geom_consistency", "true"],
        ["colmap", "stereo_fusion", "--workspace_path", str(dense),
         "--workspace_format", "COLMAP", "--input_type", "geometric",
         "--output_path", str(dense / "fused.ply")],
    ]
    for cmd in steps:
        if _run(cmd, verbose).returncode != 0:
            return None
    fused = dense / "fused.ply"
    return fused if fused.exists() else None


def reconstruct(photos: PhotoSet, cfg: Settings, workdir: Path,
                verbose: bool = False) -> ColmapResult:
    if not available():
        raise RuntimeError("colmap not found on PATH")
    workdir.mkdir(parents=True, exist_ok=True)
    imgs = workdir / "imgs"
    if imgs.exists():
        shutil.rmtree(imgs)
    imgs.mkdir()
    for i, src in enumerate(photos.image_paths):
        shutil.copy(src, imgs / f"{i:06d}{src.suffix.lower()}")

    db = workdir / "database.db"
    if db.exists():
        db.unlink()
    n = photos.n_images
    aliked = cfg.sfm_features == "aliked"
    fe = ["colmap", "feature_extractor", "--database_path", str(db), "--image_path", str(imgs),
          "--ImageReader.single_camera", "1",
          "--FeatureExtraction.num_threads", str(cfg.threads),
          *_cpu_flag("feature_extractor", "FeatureExtraction.use_gpu", "SiftExtraction.use_gpu")]
    if aliked:
        fe += ["--FeatureExtraction.type", "ALIKED_N16ROT",
               "--AlikedExtraction.max_num_features", str(cfg.sfm_max_features)]
    else:
        fe += ["--SiftExtraction.max_num_features", "4096",
               "--SiftExtraction.peak_threshold", str(cfg.sfm_sift_peak)]
    if cfg.sfm_max_image_size:
        fe += ["--FeatureExtraction.max_image_size", str(cfg.sfm_max_image_size)]
    # Focal prior for frames without EXIF (all video keyframes). COLMAP's default
    # (1.2 x the long side) is a telephoto guess; a phone main camera is ~0.7-0.85,
    # and a bad initial focal is a common reason incremental mapping fragments.
    if cfg.sfm_focal_factor and not photos.f_px:
        fe += ["--ImageReader.default_focal_length_factor", str(cfg.sfm_focal_factor)]
    if _run(fe, verbose).returncode != 0:
        raise RuntimeError("COLMAP feature extraction failed")

    sequential = n > cfg.colmap_exhaustive_max
    matcher = (["colmap", "sequential_matcher", "--database_path", str(db),
                "--SequentialMatching.overlap", str(cfg.sfm_seq_overlap)]
               if sequential
               else ["colmap", "exhaustive_matcher", "--database_path", str(db)])
    matcher += ["--FeatureMatching.num_threads", str(cfg.threads)]
    if aliked:
        matcher += ["--FeatureMatching.type", "ALIKED_LIGHTGLUE"]
    matcher += _cpu_flag("exhaustive_matcher", "FeatureMatching.use_gpu", "SiftMatching.use_gpu")
    if _run(matcher, verbose).returncode != 0:
        raise RuntimeError("COLMAP feature matching failed")

    sparse = workdir / "sparse"
    if sparse.exists():
        shutil.rmtree(sparse)
    sparse.mkdir()
    # the global mapper needs a well-connected view graph; a handful of stills
    # (photos tier) initialise more reliably incrementally
    if cfg.sfm_mapper == "global" and n >= cfg.sfm_global_min_images:
        _run(["colmap", "global_mapper", "--database_path", str(db), "--image_path", str(imgs),
              "--output_path", str(sparse), "--GlobalMapper.random_seed", "0",
              "--GlobalMapper.gp_use_gpu", "0", "--GlobalMapper.ba_ceres_use_gpu", "0",
              "--GlobalMapper.num_threads", str(cfg.threads)], verbose)
    else:
        _run(["colmap", "mapper", "--database_path", str(db), "--image_path", str(imgs),
              "--output_path", str(sparse), "--Mapper.num_threads", str(cfg.threads),
              "--Mapper.init_min_tri_angle", str(cfg.sfm_init_min_tri_angle)], verbose)

    models = sorted([d for d in sparse.iterdir() if d.is_dir() and not d.name.endswith("_txt")])
    if not models:
        raise RuntimeError(
            f"COLMAP could not register images (0 models from {n} images). "
            "The photos likely lack overlap; retry with more/overlapping images.")

    # COLMAP can emit several disjoint sub-models. Keep the one that registered
    # the most images, and record what was dropped so it is never silent.
    parsed = [(*_parse_txt_model(m, verbose=False), m) for m in models]
    parsed.sort(key=lambda c: (-len(c[2]), -len(c[0])))
    best = parsed[0]
    model_sizes = [len(c[2]) for c in parsed]
    if verbose:
        print(f"[sfm] {len(models)} model(s), registered images per model: {model_sizes} of {n}")
    pts, errs, images, K, size, model_dir = best
    if len(pts) == 0:
        raise RuntimeError("COLMAP produced an empty sparse model")
    conf = 1.0 / (1.0 + errs) if len(errs) else None
    kind = "sparse"

    # Dense MVS (GPU) if available.
    if cfg.colmap_dense:
        fused = _try_dense(workdir, verbose)
        if fused is not None:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(str(fused))
            pts = np.asarray(pcd.points)
            conf = None
            kind = "mvs"

    # CPU plane-sweep MVS from the SfM poses (fallback densification).
    if kind == "sparse" and cfg.mvs_enable and len(images) >= 3:
        try:
            from floorplan.fusion import mvs
            dense_pts, dense_conf = mvs.densify(photos, images, K, pts, cfg, verbose)
            if len(dense_pts) > len(pts):
                pts, conf, kind = dense_pts, dense_conf, "cpu_mvs"
        except Exception as exc:
            if verbose:
                print(f"[mvs] CPU densification failed, using sparse: {exc}")

    # World "up" from the camera orientations: the camera's up axis (image -y)
    # mapped to world and averaged. Robust when the phone is held roughly upright.
    up_hint = None
    if images:
        ups = np.array([im.R.T @ np.array([0.0, -1.0, 0.0]) for im in images])
        m = ups.mean(axis=0)
        if float(np.linalg.norm(m)) > 1e-6:
            up_hint = m / float(np.linalg.norm(m))

    return ColmapResult(points=pts, conf=conf, n_images=n, n_registered=len(images),
                        n_points=len(pts), dense=kind, model_dir=model_dir,
                        images=images, K=K, size=size, up_hint=up_hint,
                        model_sizes=model_sizes)
