"""Record3D export ingest (the LiDAR tier).

A Record3D export is a folder (or ZIP) named after the capture id containing:

    rgb.mp4                 colour video (intrinsics in odometry are for this resolution)
    depth/NNNNNN.png        256x192 uint16, millimetres
    confidence/NNNNNN.png   256x192 uint8, {0,1,2}
    odometry.csv            timestamp,frame,x,y,z,qx,qy,qz,qw,fx,fy,cx,cy
    imu.csv                 timestamp,a_x,a_y,a_z,alpha_x,alpha_y,alpha_z
    camera_matrix.csv       3x3 RGB intrinsics

This module parses the geometry-relevant parts (depth, poses, intrinsics,
confidence) into numpy arrays. Depth images stay on disk and are loaded lazily
during fusion.
"""
from __future__ import annotations

import csv
import hashlib
import json
import warnings
import math
import shutil
import subprocess
import tempfile
import zipfile
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from floorplan.ingest import r3d


@dataclass
class Record3DCapture:
    """Parsed Record3D capture (depth/confidence paths + per-frame poses)."""

    capture_id: str
    root: Path
    rgb_path: Path | None
    depth_paths: list[Path]
    conf_paths: list[Path]
    ts: np.ndarray            # (N,)
    pos: np.ndarray           # (N, 3) camera position in ARKit world (y-up)
    quat: np.ndarray          # (N, 4) qx,qy,qz,qw
    intr: np.ndarray          # (N, 4) fx,fy,cx,cy at RGB resolution
    K: np.ndarray             # (3, 3) RGB intrinsics from camera_matrix.csv
    rgb_size: tuple[int, int] | None      # (w, h)
    depth_size: tuple[int, int] | None    # (w, h)
    pose_convention: str | None = None    # known convention (.r3d: c2w_gl); None = auto-detect
    pose_selection: dict = field(default_factory=dict)  # how the convention was chosen (provenance)
    frame_ids: np.ndarray | None = None   # odometry frame number per entry (= rgb frame index)
    world_corr: np.ndarray | None = None  # (N, 4, 4) drift correction in the canonical Z-up world

    @property
    def n_frames(self) -> int:
        return len(self.depth_paths)


def _sha(path: Path) -> str:
    st = path.stat()
    key = f"{path.resolve()}:{st.st_size}:{int(st.st_mtime)}"
    return hashlib.sha1(key.encode()).hexdigest()[:16]


def _ensure_extracted(path: Path) -> Path:
    """Given a ZIP, extract once into a stable cache dir and return the root dir."""
    cache_root = Path(tempfile.gettempdir()) / "floorplan_cache" / _sha(path)
    marker = cache_root / ".extracted"
    if marker.exists():
        return cache_root
    cache_root.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path) as z:
        z.extractall(cache_root)
    marker.write_text("ok")
    return cache_root


def _find_capture_root(base: Path) -> Path:
    """Return the directory that directly contains odometry.csv."""
    if (base / "odometry.csv").exists():
        return base
    for child in sorted(base.iterdir()):
        if child.is_dir() and (child / "odometry.csv").exists():
            return child
    raise FileNotFoundError(f"No odometry.csv found under {base}")


def _read_camera_matrix(path: Path) -> np.ndarray:
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rows.append([float(v) for v in line.replace(",", " ").split()])
    return np.asarray(rows, dtype=np.float64)


def _read_odometry(path: Path) -> tuple[np.ndarray, ...]:
    """Parse odometry.csv into (frame_ids, ts, pos, quat, intr), validated per row.

    Rows are keyed by their ``frame`` column (not file order), quaternions are
    normalised, and a row with non-finite values, a zero quaternion, a
    non-positive focal length or a duplicate frame is skipped with a warning
    (row validation adapted from the lidar-optimized branch).
    """
    rows, seen = [], set()
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        col = {name: i for i, name in enumerate(header)}
        keys = ("timestamp", "x", "y", "z", "qx", "qy", "qz", "qw", "fx", "fy", "cx", "cy")
        for line, row in enumerate(reader, 2):
            if not row or not row[0].strip():
                continue
            try:
                vals = [float(row[col[k]].strip()) for k in keys]
                frame = int(row[col["frame"]].strip()) if "frame" in col else len(rows)
                qn = float(np.linalg.norm(vals[4:8]))
                if not np.all(np.isfinite(vals)) or qn < 1e-8:
                    raise ValueError("non-finite value or zero quaternion")
                if min(vals[8:10]) <= 0:
                    raise ValueError("non-positive focal length")
                if frame in seen:
                    raise ValueError(f"duplicate frame {frame}")
            except (KeyError, ValueError, TypeError, IndexError) as exc:
                warnings.warn(f"{path.name}: skipping row {line}: {exc}")
                continue
            vals[4:8] = list(np.asarray(vals[4:8]) / qn)
            rows.append((frame, vals))
            seen.add(frame)
    if not rows:
        raise ValueError(f"no valid poses in {path}")
    rows.sort(key=lambda r: r[0])
    ids = np.array([r[0] for r in rows], dtype=np.int64)
    a = np.array([r[1] for r in rows], dtype=np.float64)
    return ids, a[:, 0], a[:, 1:4], a[:, 4:8], a[:, 8:12]


def _probe_video_size(path: Path | None) -> tuple[int, int] | None:
    if path is None or not path.exists():
        return None
    try:
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0",
             "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", str(path)],
            capture_output=True, text=True, check=True, timeout=30,
        ).stdout.strip()
        w, h = out.split("x")
        return int(w), int(h)
    except Exception:
        return None


def _image_size(path: Path) -> tuple[int, int] | None:
    try:
        from PIL import Image
        with Image.open(path) as im:
            return im.size  # (w, h)
    except Exception:
        return None


def is_record3d_zip(path: Path) -> bool:
    if path.suffix.lower() != ".zip":
        return False
    try:
        with zipfile.ZipFile(path) as z:
            return any(n.endswith("odometry.csv") for n in z.namelist())
    except zipfile.BadZipFile:
        return False


def load(path: str | Path) -> Record3DCapture:
    """Load a Record3D capture from a directory or ZIP."""
    p = Path(path)
    if r3d.is_r3d(p):
        base = r3d.unpack(p)
    elif p.is_file() and p.suffix.lower() == ".zip":
        base = _ensure_extracted(p)
    else:
        base = p
    root = _find_capture_root(base)

    depth_dir = root / "depth"
    conf_dir = root / "confidence"
    depth_paths = sorted(depth_dir.glob("*.png"))
    conf_paths = sorted(conf_dir.glob("*.png")) if conf_dir.exists() else []
    if not depth_paths:
        raise FileNotFoundError(f"No depth frames under {depth_dir}")

    rgb_path = next((root / n for n in ("rgb.mp4", "rgb.mov") if (root / n).exists()), None)
    K = _read_camera_matrix(root / "camera_matrix.csv")
    ids, ts, pos, quat, intr = _read_odometry(root / "odometry.csv")

    # Pair poses with depth/confidence images by frame number, not by position:
    # a skipped odometry row must not shift every later depth onto the wrong pose.
    def by_frame(paths):
        out = {}
        for q in paths:
            if q.stem.isdigit():
                out[int(q.stem)] = q
        return out
    dmap, cmap = by_frame(depth_paths), by_frame(conf_paths)
    keep = [k for k, f in enumerate(ids) if int(f) in dmap]
    if not keep:
        raise ValueError("no depth image matches an odometry frame number")
    ids, ts, pos, quat, intr = ids[keep], ts[keep], pos[keep], quat[keep], intr[keep]
    depth_paths = [dmap[int(f)] for f in ids]
    conf_paths = ([cmap[int(f)] for f in ids] if cmap and all(int(f) in cmap for f in ids)
                  else [])
    if cmap and not conf_paths:
        warnings.warn("some frames have no confidence map: confidence filtering disabled")

    depth_size = _image_size(depth_paths[0]) if depth_paths else None
    src_meta = root / "source.json"          # written by ingest.r3d (no rgb.mp4 needed)
    meta_size = tuple(json.loads(src_meta.read_text())["rgb_size"]) if src_meta.exists() else None
    rgb_size = _probe_video_size(rgb_path) or meta_size or (1920, 1440)
    conv_file = root / "pose_convention.txt"
    pose_convention = conv_file.read_text().strip() if conv_file.exists() else None

    return Record3DCapture(
        capture_id=root.name,
        root=root,
        rgb_path=rgb_path,
        depth_paths=depth_paths,
        conf_paths=conf_paths,
        ts=ts,
        pos=pos,
        quat=quat,
        intr=intr,
        K=K,
        rgb_size=rgb_size,
        depth_size=depth_size,
        pose_convention=pose_convention,
        frame_ids=ids,
    )


def depth_intrinsics(capture: Record3DCapture) -> np.ndarray:
    """Per-frame intrinsics rescaled from RGB to depth resolution (x and y separately)."""
    if capture.depth_size is None or capture.rgb_size is None:
        return capture.intr.copy()
    dw, dh = capture.depth_size
    rw, rh = capture.rgb_size
    return capture.intr * np.array([dw / rw, dh / rh, dw / rw, dh / rh])
