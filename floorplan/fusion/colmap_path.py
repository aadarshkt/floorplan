"""COLMAP reconstruction path (photos / video tiers).

Runs COLMAP SfM (features -> matching -> mapping) to recover camera poses and a
sparse point cloud, then:
  * uses dense MVS (``patch_match_stereo``) when the build has CUDA/HIP, else
  * falls back to a CPU plane-sweep MVS (``fusion.mvs``) driven by the SfM poses,
  * and finally to the raw sparse cloud.

The result is in an arbitrary (scale-free) frame; metric scale is attached later
by the scale anchor. The COLMAP workspace is always kept on disk so a run is
reproducible and inspectable.

Works with COLMAP 3.x and 4.x: the CPU/SIFT option names changed between
releases, so we probe the binary's own help text once and pick the right flags.
"""
from __future__ import annotations

import json
import os
import re
import shlex
import shutil
import signal
import subprocess
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from floorplan.config import Settings
from floorplan.fusion.colmap_workspace import StageCache, file_identity, fingerprint, model_identity
from floorplan.ingest.photos import PhotoSet


@dataclass
class ImagePose:
    name: str
    R: np.ndarray      # (3,3) world -> camera
    t: np.ndarray      # (3,)  world -> camera:  X_cam = R X_world + t
    centre: np.ndarray  # (3,)  camera centre in world (-R^T t)
    obs_xy: np.ndarray | None = None    # (M,2) observed keypoint pixels
    obs_xyz: np.ndarray | None = None   # (M,3) matching sparse world points
    K: np.ndarray | None = None
    size: tuple[int, int] | None = None
    image_path: Path | None = None      # rectified image matching K and obs_xy


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
    diagnostics: dict = field(default_factory=dict)

    @property
    def model_sizes(self) -> list[int]:
        return list(self.diagnostics.get("model_sizes", []))

    @property
    def centres(self) -> np.ndarray:
        if not self.images:
            return np.zeros((0, 3))
        return np.asarray([im.centre for im in self.images])


_FLAG_CACHE: dict[str, list[str]] = {}
_HELP_CACHE: dict[str, str] = {}


def available() -> bool:
    return shutil.which("colmap") is not None


def _run(cmd: list[str], verbose: bool, log: Path | None = None,
         env_overrides: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    # NOTE: do NOT force QT_QPA_PLATFORM=offscreen here. On macOS that makes
    # COLMAP try (and fail) to create an offscreen OpenGL context and abort;
    # with CPU SIFT (use_gpu 0) it runs headless fine without the override.
    environment = {**os.environ, **env_overrides} if env_overrides else None
    # Dense stereo may take hours for hundreds of reference views. Its completed
    # maps are resumable; an arbitrary two-hour limit discarded useful progress.
    stereo = len(cmd) > 1 and cmd[1] == "patch_match_stereo"
    timeout = None if stereo else 7200
    if log is None:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=environment)
    else:
        # Stream directly to disk: logs survive Ctrl+C and can be followed live.
        log.parent.mkdir(parents=True, exist_ok=True)
        with log.open("w", buffering=1) as stream:
            prefix = ["env", *(f"{k}={v}" for k, v in env_overrides.items())] if env_overrides else []
            stream.write(shlex.join(prefix + cmd) + "\n")
            process = subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT,
                                       start_new_session=os.name == "posix", env=environment)
            try:
                if stereo:
                    while True:
                        try:
                            code = process.wait(timeout=30)
                            break
                        except subprocess.TimeoutExpired:
                            if verbose:
                                _print_stereo_progress(log)
                else:
                    code = process.wait(timeout=timeout)
            except (KeyboardInterrupt, subprocess.TimeoutExpired) as exc:
                if process.poll() is None:
                    try:
                        if os.name == "posix":
                            os.killpg(process.pid, signal.SIGINT)
                        else:
                            process.terminate()
                    except ProcessLookupError:
                        pass
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        if os.name == "posix":
                            os.killpg(process.pid, signal.SIGKILL)
                        else:
                            process.kill()
                        process.wait()
                stream.write(f"\n{type(exc).__name__}: stage incomplete; completed earlier stages preserved\n")
                if isinstance(exc, KeyboardInterrupt):
                    raise
                code = 124
        with log.open("rb") as stream:
            stream.seek(max(0, log.stat().st_size - 16000))
            tail = stream.read().decode(errors="replace")
        res = subprocess.CompletedProcess(cmd, code, "", tail)
    if verbose and res.returncode != 0:
        print(f"[colmap] command failed ({' '.join(cmd[:2])}): {_failure_reason(res)}")
    return res


def _print_stereo_progress(log: Path) -> None:
    with log.open("rb") as stream:
        stream.seek(max(0, log.stat().st_size - 16000))
        tail = stream.read().decode(errors="replace")
    views = re.findall(r"Processing view (\d+) / (\d+)", tail)
    modes = re.findall(r"\bgeom_consistency: ([01])\b", tail)
    if views and modes:
        phase = "geometric pass 2/2" if modes[-1] == "1" else "photometric pass 1/2"
        current, total = views[-1]
        print(f"[colmap] stereo {phase}: processing view {current}/{total}", flush=True)
    else:
        print(f"[colmap] stereo running; details: {log}", flush=True)


def _failure_reason(result: subprocess.CompletedProcess) -> str:
    lines = (result.stderr or result.stdout or "").strip().splitlines()
    # Preserve the actual runtime fault instead of showing only libc abort frames.
    for needle in ("memory access fault", "illegal memory access", "hiperror", "cuda error",
                   "out of memory", "failed to", "error:", "sigabrt", "sigsegv"):
        for line in lines:
            if needle in line.lower():
                return line.strip()[:600]
    return f"exit {result.returncode}: " + " | ".join(lines[-3:])[:600]


class FeatureBackendError(RuntimeError):
    def __init__(self, stage: str, backend: str, reason: str, log: Path):
        self.stage, self.backend, self.reason, self.log = stage, backend, reason, log
        super().__init__(f"COLMAP {backend} {stage} failed: {reason}; see {log}. "
                         "Use --colmap-features auto for CPU feature fallback while keeping GPU stereo")


def _feature_runtime_env(cfg: Settings) -> dict[str, str]:
    if _feature_device(cfg) != "HIP":
        return {}
    # The installed SiftGPU port destroys temporary texture objects immediately
    # after async kernel launches. Serialize HIP launches/copies so their users
    # finish before those handles are released. Scope this to SIFT subprocesses,
    # not dense stereo, and include it in stage keys to discard old partial data.
    return {"AMD_SERIALIZE_KERNEL": "3", "AMD_SERIALIZE_COPY": "3"}


def _feature_failure(stage: str, cfg: Settings, result: subprocess.CompletedProcess,
                     log: Path, cache: StageCache, key: str) -> FeatureBackendError:
    reason = _failure_reason(result)
    cache.fail(stage, key, reason)
    # Matching failures also taint the shared DB. Do not reuse GPU-written pairs
    # or descriptors after a native abort, even if some rows were committed.
    if stage != "features" and "features" in cache.records:
        cache.fail("features", cache.records["features"]["key"], reason)
    backend = _feature_device(cfg)
    archived = log.with_name(f"{log.stem}.{backend}.failed.log")
    shutil.copy2(log, archived)
    return FeatureBackendError(stage, backend, reason, archived)


def _help(subcmd: str = "") -> str:
    if subcmd not in _HELP_CACHE:
        cmd = ["colmap", subcmd, "-h"] if subcmd else ["colmap", "-h"]
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        _HELP_CACHE[subcmd] = (res.stdout or "") + (res.stderr or "")
    return _HELP_CACHE[subcmd]


def gpu_backend() -> str:
    if not available():
        return "none"
    version = _help()
    return "HIP" if "with HIP" in version else "CUDA" if "with CUDA" in version else "none"


def _option(subcmd: str, names: tuple[str, ...], value) -> list[str]:
    name = next((name for name in names if "--" + name + " " in _help(subcmd)), None)
    return ["--" + name, str(value)] if name else []


def _feature_device_flags(subcmd: str, cfg: Settings, extraction: bool = False) -> list[str]:
    gpu = _feature_device(cfg) != "CPU"
    new = "FeatureExtraction" if extraction else "FeatureMatching"
    old = "SiftExtraction" if extraction else "SiftMatching"
    flags = _option(subcmd, (f"{new}.use_gpu", f"{old}.use_gpu"), int(gpu))
    if gpu:
        flags += _option(subcmd, (f"{new}.gpu_index", f"{old}.gpu_index"), cfg.colmap_gpu_index)
    return flags


def _feature_device(cfg: Settings) -> str:
    if cfg.colmap_features == "cpu":
        return "CPU"
    backend = gpu_backend()
    if backend == "none":
        if cfg.colmap_features == "gpu":
            raise RuntimeError("GPU features requested, but COLMAP has no HIP/CUDA build support")
        return "CPU"
    # Modern HIP builds include the SiftGPU compute kernels too. Do not force
    # CPU or enable affine/DSP extraction, which overrides use_gpu in COLMAP.
    return backend


def _cpu_flag(subcmd: str, new: str, old: str) -> list[str]:
    key = f"{subcmd}:{new}:{old}"
    if key in _FLAG_CACHE:
        return _FLAG_CACHE[key]
    help_txt = ""
    try:
        res = subprocess.run(["colmap", subcmd, "-h"], capture_output=True,
                             text=True, timeout=60)
        help_txt = (res.stdout or "") + "\n" + (res.stderr or "")
    except Exception:
        pass
    flag = new if new in help_txt else (old if old in help_txt else None)
    _FLAG_CACHE[key] = ["--" + flag, "0"] if flag else []
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


def _camera_map(txt: Path) -> dict[int, tuple[np.ndarray, tuple[int, int]]]:
    """Read calibration per camera; distorted models are only used during SfM.

    Depth backends receive the PINHOLE cameras exported by image_undistorter.
    """
    cameras = {}
    for line in txt.read_text().splitlines():
        if not line or line.startswith("#"):
            continue
        p = line.split()
        w, h = int(p[2]), int(p[3])
        params = [float(v) for v in p[4:]]
        if p[1] in ("SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL"):
            fx = fy = params[0]
            cx, cy = params[1], params[2]
        elif p[1] in ("PINHOLE", "OPENCV", "FULL_OPENCV"):
            fx, fy, cx, cy = params[:4]
        else:
            raise ValueError(f"Unsupported camera model for depth unprojection: {p[1]}")
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])
        cameras[int(p[0])] = K, (w, h)
    if not cameras:
        raise RuntimeError(f"No camera calibration in {txt}")
    return cameras


def _parse_cameras(txt: Path) -> tuple[np.ndarray, tuple[int, int]]:
    return next(iter(_camera_map(txt).values()))


def _parse_txt_model(model_dir: Path, verbose: bool
                     ) -> tuple[np.ndarray, np.ndarray, list[ImagePose], np.ndarray, tuple[int, int]]:
    txt = model_dir.parent / (model_dir.name + "_txt")
    if txt.exists():
        shutil.rmtree(txt)
    txt.mkdir(parents=True)  # COLMAP's TXT writer requires the dir to already exist
    res = _run(["colmap", "model_converter", "--input_path", str(model_dir),
                "--output_path", str(txt), "--output_type", "TXT"], verbose)
    if res.returncode != 0:
        raise RuntimeError(f"Could not read COLMAP model {model_dir}: {res.stderr[-1000:]}")
    cameras = _camera_map(txt / "cameras.txt")

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
                    if j is not None and errs[j] <= 2.0:
                        obs_xy.append([px, py])
                        obs_xyz.append(xyz[j])
            i += 1
            camera_K, camera_size = cameras[int(parts[8])]
            images.append(ImagePose(
                name=parts[9], R=R, t=t, centre=-R.T @ t,
                obs_xy=np.asarray(obs_xy, dtype=np.float64) if obs_xy else None,
                obs_xyz=np.asarray(obs_xyz, dtype=np.float64) if obs_xyz else None,
                K=camera_K, size=camera_size,
            ))

    K, size = next(iter(cameras.values()))
    return (np.asarray(xyz, dtype=np.float64).reshape(-1, 3), np.asarray(errs, dtype=np.float64),
            images, K, size)


def _undistort(workdir: Path, model_dir: Path, cfg: Settings, verbose: bool):
    dense = workdir / "dense"
    cmd = ["colmap", "image_undistorter", "--image_path", str(workdir / "imgs"),
           "--input_path", str(model_dir), "--output_path", str(dense),
           "--output_type", "COLMAP", "--max_image_size", str(cfg.colmap_max_image_size)]
    log = workdir / "logs" / "image_undistorter.log"
    cache = StageCache(workdir)
    key = fingerprint("rectification-v2", model_identity(model_dir),
                      cache.records.get("features"), cmd)
    if cache.complete("undistortion", key, [dense / "sparse" / "images.bin", dense / "images"]):
        if verbose:
            print("[colmap] reusing rectified images")
    else:
        cache.begin("undistortion", key)
        # Invalidate derived maps even if the regenerated calibration happens
        # to have identical bytes. The previous image workspace is being reset.
        cache.records.pop("patch_match_stereo", None)
        cache.records.pop("stereo_fusion", None)
        if dense.exists():
            shutil.rmtree(dense)
        if _run(cmd, verbose, log).returncode != 0:
            raise RuntimeError(f"Camera undistortion failed; see {log}")
        cache.finish("undistortion", key)
    parsed = _parse_txt_model(dense / "sparse", verbose)
    for im in parsed[2]:
        im.image_path = dense / "images" / im.name
        if not im.image_path.is_file():
            raise RuntimeError(f"Undistorted image missing: {im.image_path}")
    return parsed


def _try_dense(workdir: Path, verbose: bool, model_dir: Path | None = None,
               cfg: Settings | None = None) -> Path | None:
    cfg = cfg or Settings()
    if cfg.colmap_dense_mode == "cpu":
        return None
    # Build support is distinct from runtime device availability. Preserve full
    # failure logs so HIP architecture/driver errors are actionable.
    backend = gpu_backend()
    if backend == "none":
        if cfg.colmap_dense_mode == "gpu":
            raise RuntimeError("GPU stereo requested, but this COLMAP build has neither HIP nor CUDA")
        if verbose:
            print("[colmap] no GPU stereo backend in this build; using CPU MVS")
        return None
    if verbose:
        print(f"[colmap] dense backend={backend} device={cfg.colmap_gpu_index} "
              f"max_image_size={cfg.colmap_max_image_size}")
    dense = workdir / "dense"
    if not (dense / "sparse").exists():
        _undistort(workdir, model_dir or workdir / "sparse" / "0", cfg, verbose)
    steps = [
        ["colmap", "patch_match_stereo", "--workspace_path", str(dense),
          "--workspace_format", "COLMAP", "--PatchMatchStereo.geom_consistency", "true",
          "--PatchMatchStereo.gpu_index", cfg.colmap_gpu_index,
          "--PatchMatchStereo.max_image_size", str(cfg.colmap_max_image_size),
          "--PatchMatchStereo.window_radius", "7",
          "--PatchMatchStereo.filter", "true",
          "--PatchMatchStereo.cache_size", str(cfg.colmap_cache_gb)],
        ["colmap", "stereo_fusion", "--workspace_path", str(dense),
         "--workspace_format", "COLMAP", "--input_type", "geometric",
         "--StereoFusion.min_num_pixels", str(cfg.colmap_fusion_min_pixels),
         "--StereoFusion.max_reproj_error", "2",
         "--StereoFusion.max_depth_error", "0.01",
         "--StereoFusion.check_num_images", "50",
         "--StereoFusion.max_image_size", str(cfg.colmap_max_image_size),
         "--output_path", str(dense / "fused.ply")],
    ]
    logs = workdir / "logs"
    logs.mkdir(parents=True, exist_ok=True)
    cache = StageCache(workdir)
    dependency = cache.records.get("undistortion")
    for cmd in steps:
        log = logs / f"{cmd[1]}.log"
        key = fingerprint("dense-v2", dependency, cmd)
        output = dense / "stereo" / "depth_maps" if cmd[1] == "patch_match_stereo" else dense / "fused.ply"
        if cache.complete(cmd[1], key, [output]):
            if verbose:
                print(f"[colmap] reusing completed {cmd[1]}")
            dependency = key
            continue
        if cmd[1] == "patch_match_stereo":
            if not cache.same_inputs(cmd[1], key):
                # Parameter changes must not reuse maps COLMAP would otherwise
                # skip by filename. Matching retries with identical inputs can.
                for name in ("depth_maps", "normal_maps", "consistency_graphs"):
                    path = dense / "stereo" / name
                    if path.exists():
                        # Preserve the per-camera directory structure prepared
                        # by the undistorter; only remove computed maps.
                        for binary in path.rglob("*.bin"):
                            binary.unlink()
            cache.records.pop("stereo_fusion", None)
        else:
            (dense / "fused.ply").unlink(missing_ok=True)
        cache.begin(cmd[1], key)
        if verbose:
            print(f"[colmap] {cmd[1]} (log: {log})", flush=True)
        res = _run(cmd, verbose, log)
        if res.returncode != 0:
            tail = (res.stderr or res.stdout or "").strip().splitlines()
            sig = next((ln for ln in tail if "SIGSEGV" in ln or "hipError" in ln
                        or "HIP" in ln and "error" in ln.lower()), "")
            hint = (f" ({sig.strip()})" if sig else "")
            message = f"COLMAP {backend} {cmd[1]} failed{hint}; full log: {log}"
            if cfg.colmap_dense_mode == "gpu":
                raise RuntimeError(message)
            print(f"[colmap] {message}; falling back to CPU MVS")
            return None
        cache.finish(cmd[1], key)
        dependency = key
    fused = dense / "fused.ply"
    if not fused.exists():
        if cfg.colmap_dense_mode == "gpu":
            raise RuntimeError(f"COLMAP produced no dense cloud; see {logs}")
        print("[colmap] dense finished but fused.ply is missing, falling back to CPU MVS")
        return None
    return fused


def _prepare_images(photos: PhotoSet, imgs: Path) -> list[dict]:
    from PIL import Image, ImageOps

    groups = {}
    manifest = []
    for i, src in enumerate(photos.image_paths):
        with Image.open(src) as raw:
            exif = raw.getexif()
            camera_exif = exif.get_ifd(34665)
            orientation = exif.get(274, 1)
            im = ImageOps.exif_transpose(raw)
            # Share calibration only within the same lens/zoom/dimensions. Video
            # frames are one camera; unordered photos must not inherit the first
            # photo's calibration when cameras or orientations differ.
            signature = (im.size,) if photos.ordered else (
                im.size, str(src.parent.resolve()),
                *(str(camera_exif.get(tag, exif.get(tag, "")))
                  for tag in (271, 272, 42036, 37386, 41989, 41988)))
            group = groups.setdefault(signature, len(groups))
            folder = imgs / f"camera_{group:03d}"
            folder.mkdir(exist_ok=True)
            if src.suffix.lower() in (".jpg", ".jpeg") and orientation == 1:
                target = folder / f"{i:06d}.jpg"
                shutil.copy2(src, target)  # retain EXIF and avoid JPEG recompression
            else:
                target = folder / f"{i:06d}.png"
                # Rectify EXIF orientation once; SfM/depth now see identical pixels.
                im.convert("RGB").save(target, exif=im.getexif().tobytes())
            manifest.append({"name": target.relative_to(imgs).as_posix(),
                             "source": str(src.resolve()), "camera_group": group})
    return manifest


def _match(db: Path, subcmd: str, cfg: Settings, verbose: bool, log: Path,
           cache: StageCache, dependency: str) -> str:
    cmd = ["colmap", subcmd, "--database_path", str(db),
           *_feature_device_flags(subcmd, cfg),
           *_option(subcmd, ("FeatureMatching.max_num_matches", "SiftMatching.max_num_matches"), 8192),
           *_option(subcmd, ("FeatureMatching.guided_matching", "SiftMatching.guided_matching"), 1)]
    if subcmd == "sequential_matcher":
        cmd += ["--SequentialMatching.overlap", str(cfg.colmap_sequential_overlap),
                "--SequentialMatching.quadratic_overlap", "1"]
    if subcmd == "transitive_matcher":
        cmd += ["--TransitiveMatching.num_iterations", "2"]
    environment = _feature_runtime_env(cfg)
    key = fingerprint(dependency, cmd, environment)
    if cache.complete(log.stem, key, [db]):
        if verbose:
            print(f"[colmap] reusing completed {subcmd}")
        return key
    cache.begin(log.stem, key)
    if verbose:
        print(f"[colmap] {subcmd} with guided matching (log: {log})", flush=True)
    res = _run(cmd, verbose, log, env_overrides=environment)
    if res.returncode != 0:
        raise _feature_failure(log.stem, cfg, res, log, cache, key)
    cache.finish(log.stem, key)
    return key


def _model_dirs(output: Path) -> list[Path]:
    if (output / "images.bin").exists():
        return [output]
    if not output.exists():
        return []
    return [d for d in sorted(output.iterdir()) if d.is_dir() and (d / "images.bin").exists()]


def _map(db: Path, imgs: Path, output: Path, cfg: Settings, verbose: bool,
         log: Path, cache: StageCache, dependency: str, ordered: bool,
         seed: Path | None = None, global_mapping: bool = False) -> list:
    cmd = ["colmap", "mapper", "--database_path", str(db), "--image_path", str(imgs),
           "--output_path", str(output),
           "--Mapper.init_min_tri_angle", str(cfg.colmap_init_min_tri_angle),
           "--Mapper.max_reg_trials", "10", "--Mapper.ba_local_num_images", "10",
           "--Mapper.filter_max_reproj_error", "4.0",
           "--Mapper.tri_ignore_two_view_tracks", str(int(ordered)),
           "--Mapper.ba_refine_extra_params", str(int(not ordered)),
           "--Mapper.ba_local_min_tri_angle", str(cfg.colmap_init_min_tri_angle)]
    # Keep useful bridge tracks during mapping; the exported cloud still uses
    # the stricter 2px reprojection filter. Processed video's radial coefficient
    # stays at zero while focal length is estimated, avoiding a poorly
    # constrained focal/distortion trade-off during short-baseline motion.
    if global_mapping:
        cmd = ["colmap", "global_mapper", "--database_path", str(db),
               "--image_path", str(imgs), "--output_path", str(output),
               "--GlobalMapper.ba_refine_extra_params", str(int(not ordered)),
               "--GlobalMapper.track_min_num_views_per_track", "3",
               "--GlobalMapper.gp_use_gpu", "0", "--GlobalMapper.ba_ceres_use_gpu", "0"]
    if seed is not None:
        cmd += ["--input_path", str(seed), "--Mapper.multiple_models", "0"]
    key = fingerprint("mapping-v2", dependency, cmd, model_identity(seed) if seed else None)
    dirs = _model_dirs(output)
    if cache.complete(log.stem, key, [output]) and dirs:
        if verbose:
            print(f"[colmap] reusing completed {log.stem}")
        return [(*_parse_txt_model(d, verbose=False), d) for d in dirs]
    cache.begin(log.stem, key)
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    if verbose:
        stage = "global camera recovery" if global_mapping else "recovering cameras" if seed else "mapping cameras"
        print(f"[colmap] {stage} on CPU (log: {log})", flush=True)
    res = _run(cmd, verbose, log)
    dirs = _model_dirs(output)
    if not dirs and res.returncode != 0:
        raise RuntimeError(f"COLMAP mapping failed; see {log}")
    models = [(*_parse_txt_model(d, verbose=False), d) for d in dirs]
    if res.returncode == 0:
        cache.finish(log.stem, key)
    return models


def _save_diagnostics(workdir: Path, diagnostics: dict) -> None:
    temporary = workdir / "reconstruction.json.tmp"
    temporary.write_text(json.dumps(diagnostics, indent=2))
    temporary.replace(workdir / "reconstruction.json")


def reconstruct(photos: PhotoSet, cfg: Settings, workdir: Path,
                verbose: bool = False) -> ColmapResult:
    """Try the selected feature backend; auto can recover once using CPU SIFT.

    Feature failure never relaxes the separate GPU requirement for dense stereo.
    A recorded failure is specific to this capture, binary, device and runtime
    settings, so reruns can resume the CPU result without repeating a known crash.
    """
    if not available() or cfg.colmap_features == "cpu":
        return _reconstruct(photos, cfg, workdir, verbose)
    context = fingerprint("feature-runtime-v1", file_identity(Path(shutil.which("colmap"))),
                          [file_identity(p) for p in photos.image_paths],
                          cfg.colmap_gpu_index, cfg.colmap_sift_max_image_size, _feature_runtime_env(cfg),
                          {key: os.environ.get(key) for key in (
                              "ROCR_VISIBLE_DEVICES", "HIP_VISIBLE_DEVICES", "CUDA_VISIBLE_DEVICES",
                              "HSA_OVERRIDE_GFX_VERSION")})
    record = workdir / "feature_backend_failure.json"
    if cfg.colmap_features == "auto":
        try:
            previous = json.loads(record.read_text())
        except (OSError, ValueError):
            previous = {}
        if previous.get("context") == context and previous.get("backend") in ("HIP", "CUDA"):
            print(f"[colmap] prior {previous['backend']} SIFT failure; resuming CPU features. "
                  f"Stereo mode remains {cfg.colmap_dense_mode}; failure log: {previous['log']}")
            return _reconstruct(photos, replace(cfg, colmap_features="cpu"), workdir, verbose,
                                feature_fallback=previous)
    try:
        return _reconstruct(photos, cfg, workdir, verbose)
    except FeatureBackendError as exc:
        failure = {"context": context, "backend": exc.backend, "stage": exc.stage,
                   "reason": exc.reason, "log": str(exc.log)}
        temporary = record.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(failure, indent=2))
        temporary.replace(record)
        report = workdir / "reconstruction.json"
        if report.exists():
            diagnostics = json.loads(report.read_text())
            diagnostics.update(stage=f"{exc.stage}_failed", error=str(exc))
            _save_diagnostics(workdir, diagnostics)
        if cfg.colmap_features != "auto" or exc.backend == "CPU":
            raise
        print(f"[colmap] {exc.backend} SIFT failed: {exc.reason}. "
              f"Rebuilding features on CPU; stereo mode remains {cfg.colmap_dense_mode}. "
              f"Preserved log: {exc.log}")
        return _reconstruct(photos, replace(cfg, colmap_features="cpu"), workdir, verbose,
                            feature_fallback=failure)


def _reconstruct(photos: PhotoSet, cfg: Settings, workdir: Path,
                 verbose: bool = False, feature_fallback: dict | None = None) -> ColmapResult:
    if not available():
        raise RuntimeError("colmap not found on PATH")
    if cfg.colmap_dense_mode == "gpu" and gpu_backend() == "none":
        raise RuntimeError("GPU stereo requires a COLMAP build with HIP (AMD) or CUDA (NVIDIA)")
    if photos.n_images < 2:
        raise ValueError("Camera reconstruction needs at least two overlapping images")
    if not 0 < cfg.sfm_min_registered_fraction <= 1:
        raise ValueError("sfm_min_registered_fraction must be in (0, 1]")
    if cfg.colmap_max_image_size <= 0 or cfg.colmap_fusion_min_pixels < 3:
        raise ValueError("Stereo needs a positive image size and at least three consistent pixels")
    workdir.mkdir(parents=True, exist_ok=True)
    imgs = workdir / "imgs"
    cache = StageCache(workdir)
    logs = workdir / "logs"
    diagnostics = {"input_images": photos.n_images, "ordered_video": photos.ordered,
                   "gpu_backend_available": gpu_backend(), "gpu_index": cfg.colmap_gpu_index,
                   "dense_requested": cfg.colmap_dense_mode, "stage": "features",
                   "sift_device": _feature_device(cfg), "mapping_device": "CPU",
                   "video_distortion_fixed": photos.ordered,
                   "features_requested": "auto" if feature_fallback else cfg.colmap_features,
                   "feature_runtime_env": _feature_runtime_env(cfg)}
    if feature_fallback:
        diagnostics["feature_fallback"] = feature_fallback
    _save_diagnostics(workdir, diagnostics)

    db = workdir / "database.db"
    n = photos.n_images
    fe = ["colmap", "feature_extractor", "--database_path", str(db), "--image_path", str(imgs),
          "--ImageReader.single_camera_per_folder", "1", "--ImageReader.camera_model", "SIMPLE_RADIAL",
          "--SiftExtraction.max_num_features", "8192",
          "--SiftExtraction.peak_threshold", str(cfg.sfm_sift_peak),
          # Focal prior for EXIF-less frames (all video keyframes): COLMAP's
          # default is a telephoto guess; a phone main camera is ~0.7-0.85.
          *(["--ImageReader.default_focal_length_factor", str(cfg.sfm_focal_factor)]
            if cfg.sfm_focal_factor and not photos.f_px else []),
          *_option("feature_extractor", ("FeatureExtraction.max_image_size", "SiftExtraction.max_image_size"),
                   cfg.colmap_sift_max_image_size),
          *_feature_device_flags("feature_extractor", cfg, extraction=True)]
    # Affine/domain pooling explicitly forces the CPU extractor, even when
    # use_gpu=1. Keep it disabled for both HIP and CUDA.
    if diagnostics["sift_device"] == "CPU":
        fe += ["--SiftExtraction.estimate_affine_shape", "1", "--SiftExtraction.domain_size_pooling", "1"]
    else:
        fe += ["--SiftExtraction.estimate_affine_shape", "0", "--SiftExtraction.domain_size_pooling", "0"]
    feature_key = fingerprint("features-v2", photos.ordered,
                              [file_identity(p) for p in photos.image_paths],
                              file_identity(Path(shutil.which("colmap"))), fe, _feature_runtime_env(cfg))
    same_database = (cache.same_inputs("features", feature_key) and db.exists()
                     and not cache.records.get("features", {}).get("failed", False))
    if not same_database:
        # Invalidate dependent stages whenever a different feature set replaces
        # the database. A partial extraction with the same inputs can resume.
        cache.clear()
        for suffix in ("", "-wal", "-shm"):
            Path(str(db) + suffix).unlink(missing_ok=True)
    manifest_path = workdir / "input_images.json"
    manifest = json.loads(manifest_path.read_text()) if same_database and manifest_path.exists() else []
    if not manifest or not all((imgs / entry["name"]).exists() for entry in manifest):
        if imgs.exists():
            shutil.rmtree(imgs)
        imgs.mkdir()
        manifest = _prepare_images(photos, imgs)
        manifest_path.write_text(json.dumps(manifest, indent=2))
    if verbose:
        print(f"[colmap] {n} images, SIFT requested={diagnostics['sift_device']}, "
              f"stereo build={gpu_backend()} (log: {logs / 'features.log'})", flush=True)
        if diagnostics["sift_device"] == "HIP":
            print("[colmap] HIP SIFT uses synchronized kernels/copies for texture lifetime safety")
    if cache.complete("features", feature_key, [db]):
        if verbose:
            print("[colmap] reusing completed feature extraction")
    else:
        cache.begin("features", feature_key)
        res = _run(fe, verbose, logs / "features.log", env_overrides=_feature_runtime_env(cfg))
        if res.returncode != 0:
            diagnostics["stage"] = "features_failed"
            _save_diagnostics(workdir, diagnostics)
            raise _feature_failure("features", cfg, res, logs / "features.log", cache, feature_key)
        cache.finish("features", feature_key)

    # Sequential matching is appropriate only for video. Folder order says
    # nothing about overlap between photos, even in large photo sets.
    # Moderate video sets already need all-pairs matches across revisited walls.
    # Build that graph before choosing a seed, rather than first committing to a
    # fragment of the trajectory and adding loop matches too late.
    exhaustive_limit = max(cfg.colmap_exhaustive_max, cfg.colmap_recovery_exhaustive_max)
    matcher = "sequential_matcher" if photos.ordered and n > exhaustive_limit else "exhaustive_matcher"
    matching_key = _match(db, matcher, cfg, verbose, logs / "matching.log", cache, feature_key)

    sparse = workdir / "sparse"
    try:
        models = _map(db, imgs, sparse, cfg, verbose, logs / "mapping.log", cache,
                      matching_key, photos.ordered)
    except RuntimeError as exc:
        diagnostics["initial_mapping_error"] = str(exc)
        models = []
    ranking = lambda candidate: (len(candidate[2]), len(candidate[0]))
    best = max(models, key=ranking) if models else None
    diagnostics["initial_model_sizes"] = [len(model[2]) for model in models]
    diagnostics["stage"] = "registration"
    _save_diagnostics(workdir, diagnostics)
    # Rebuild from the expanded graph: extending only the largest old model can
    # preserve a bad seed/calibration and never reconsider the other components.
    if best is None or len(best[2]) < 0.90 * n:
        if verbose:
            print(f"[colmap] connected views={len(best[2]) if best else 0}/{n}; trying registration recovery")
        recovery_matcher = ("exhaustive_matcher"
                            if matcher != "exhaustive_matcher" and n <= cfg.colmap_recovery_exhaustive_max
                            else "transitive_matcher")
        recovered = workdir / "recovered"
        try:
            matching_key = _match(db, recovery_matcher, cfg, verbose, logs / "recovery_matching.log",
                                  cache, matching_key)
            extra = _map(db, imgs, recovered, cfg, verbose, logs / "recovery_mapping.log", cache,
                         matching_key, photos.ordered)
            models.extend(extra)
            if models:
                best = max(models, key=ranking)
        except FeatureBackendError:
            raise
        except RuntimeError as exc:
            diagnostics["recovery_error"] = str(exc)
            if verbose:
                print(f"[colmap] recovery failed: {exc}")
    # Global SfM jointly estimates a connected camera graph rather than growing
    # from one initial pair. It is a bounded alternative when incremental seeds
    # keep producing fragments; all geometric checks remain enabled.
    if (cfg.colmap_global_recovery and (best is None or len(best[2]) / n < cfg.sfm_min_registered_fraction)
            and "--GlobalMapper.gp_use_gpu " in _help("global_mapper")):
        try:
            extra = _map(db, imgs, workdir / "global", cfg, verbose, logs / "global_mapping.log",
                         cache, matching_key, photos.ordered, global_mapping=True)
            models.extend(extra)
            if models:
                best = max(models, key=ranking)
        except RuntimeError as exc:
            diagnostics["global_recovery_error"] = str(exc)
            if verbose:
                print(f"[colmap] global recovery failed: {exc}")
    diagnostics["model_sizes"] = [len(model[2]) for model in models]
    registered = {im.name for im in best[2]} if best else set()
    diagnostics.update(registered=len(registered), registered_fraction=len(registered) / n,
                       stage="registered" if len(registered) / n >= cfg.sfm_min_registered_fraction else "registration_incomplete",
                       unregistered=[entry for entry in manifest if entry["name"] not in registered])
    _save_diagnostics(workdir, diagnostics)
    if best is None:
        raise RuntimeError(f"COLMAP registered no connected cameras; see {workdir / 'reconstruction.json'}")
    pts, errs, images, K, size, model_dir = best
    good = np.isfinite(pts).all(axis=1) & np.isfinite(errs) & (errs <= 2.0)
    pts, errs = pts[good], errs[good]
    if len(pts) == 0:
        raise RuntimeError("COLMAP produced an empty sparse model")
    if len(images) < 2 or len(images) / n < cfg.sfm_min_registered_fraction:
        raise RuntimeError(f"Only {len(images)}/{n} images registered in one connected model; "
                           f"dense stereo skipped. See {workdir / 'reconstruction.json'} for missing views")
    # All depth methods use rectified pixels and their per-image calibration,
    # including learned depth and the CPU fallback.
    pts, errs, images, K, size = _undistort(workdir, model_dir, cfg, verbose)
    good = np.isfinite(pts).all(axis=1) & np.isfinite(errs) & (errs <= 2.0)
    pts, errs = pts[good], errs[good]
    conf = 1.0 / (1.0 + errs) if len(errs) else None
    kind = "sparse"

    # World "up" from the camera orientations: the camera's up axis (image -y)
    # mapped to world and averaged. Robust when the phone is held roughly upright.
    up_hint = None
    if images:
        ups = np.array([im.R.T @ np.array([0.0, -1.0, 0.0]) for im in images])
        m = np.median(ups, axis=0)
        if float(np.linalg.norm(m)) > 1e-6:
            up_hint = m / float(np.linalg.norm(m))

    diagnostics.update(stage="rectified", sparse_points=len(pts), selected_model=str(model_dir))
    _save_diagnostics(workdir, diagnostics)
    if verbose:
        print(f"[colmap] registered={len(images)}/{n}, sparse_points={len(pts)}, images rectified")
    result = ColmapResult(points=pts, conf=conf, n_images=n, n_registered=len(images),
                          n_points=len(pts), dense=kind, model_dir=model_dir,
                          images=images, K=K, size=size, up_hint=up_hint, diagnostics=diagnostics)
    return densify(result, photos, cfg, workdir, verbose)


def densify(result: ColmapResult, photos: PhotoSet, cfg: Settings, workdir: Path,
            verbose: bool = False) -> ColmapResult:
    """Densify existing poses, also usable after learned depth fails."""
    if result.dense != "sparse":
        return result
    pts, conf, kind = result.points, result.conf, result.dense
    images, K, model_dir = result.images, result.K, result.model_dir
    diagnostics = dict(result.diagnostics)
    diagnostics["stage"] = "densifying"
    _save_diagnostics(workdir, diagnostics)
    # Dense MVS (GPU) if available.
    if cfg.colmap_dense and cfg.colmap_dense_mode != "cpu":
        try:
            fused = _try_dense(workdir, verbose, model_dir=model_dir, cfg=cfg)
        except RuntimeError as exc:
            diagnostics.update(stage="dense_failed", error=str(exc))
            _save_diagnostics(workdir, diagnostics)
            raise
        if fused is not None:
            import open3d as o3d
            pcd = o3d.io.read_point_cloud(str(fused))
            dense_pts = np.asarray(pcd.points)
            dense_pts = dense_pts[np.isfinite(dense_pts).all(axis=1)]
            diagnostics["gpu_fused_points"] = len(dense_pts)
            # A dense surface cloud and a sparse feature cloud measure different
            # things. Point-count comparison is not a density/accuracy test.
            if len(dense_pts):
                pts, conf, kind = dense_pts, None, "mvs"
                diagnostics["dense_device"] = f"{gpu_backend()}:{cfg.colmap_gpu_index}"
            elif cfg.colmap_dense_mode == "gpu":
                message = f"GPU stereo produced no consistent surfaces; see {workdir / 'logs'}"
                diagnostics.update(stage="dense_failed", error=message)
                _save_diagnostics(workdir, diagnostics)
                raise RuntimeError(message)
            elif verbose:
                print("[colmap] GPU stereo produced no consistent surfaces; trying CPU MVS")

    # CPU plane-sweep MVS from the SfM poses (fallback densification).
    if kind == "sparse" and cfg.mvs_enable and len(images) >= 3:
        try:
            from floorplan.fusion import mvs
            dense_pts, dense_conf = mvs.densify(photos, images, K, pts, cfg, verbose)
            if len(dense_pts):
                pts, conf, kind = dense_pts, dense_conf, "cpu_mvs"
                diagnostics["dense_device"] = "CPU"
        except Exception as exc:
            if verbose:
                print(f"[mvs] CPU densification failed, using sparse: {exc}")

    diagnostics.update(stage="reconstructed", dense_method=kind, points_raw=len(pts))
    _save_diagnostics(workdir, diagnostics)
    if verbose:
        print(f"[colmap] method={kind}, device={diagnostics.get('dense_device', 'none')}, "
              f"points={len(pts)} (report: {workdir / 'reconstruction.json'})")
    return replace(result, points=pts, conf=conf, dense=kind, n_points=len(pts), diagnostics=diagnostics)
