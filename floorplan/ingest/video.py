"""Video ingest (the video tier).

A walkthrough clip is reduced to keyframes with ffmpeg, then handled exactly like
a photo set. Frame rate is configurable; the default (2 fps) is a practical
balance between overlap for SfM and runtime.
"""
from __future__ import annotations

import hashlib
import shutil
import subprocess
import tempfile
from pathlib import Path

from floorplan.ingest.photos import PhotoSet, _image_size


def _cache_dir(path: Path, fps: float) -> Path:
    st = path.stat()
    key = hashlib.sha1(f"{path.resolve()}:{st.st_size}:{int(st.st_mtime)}:{fps}".encode()).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / "floorplan_cache" / f"video_{key}"


def extract_frames(path: str | Path, fps: float = 2.0) -> Path:
    """Extract frames at ``fps`` into a stable cache dir; return the frames dir."""
    src = Path(path)
    out = _cache_dir(src, fps)
    marker = out / ".done"
    if marker.exists():
        return out
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to read video but was not found on PATH")
    out.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-v", "error", "-i", str(src), "-vf", f"fps={fps}", "-q:v", "3",
           str(out / "%06d.jpg")]
    subprocess.run(cmd, check=True, timeout=600)
    if not any(out.glob("*.jpg")):
        raise RuntimeError(f"ffmpeg produced no frames from {src}")
    marker.write_text("ok")
    return out


def load(path: str | Path, fps: float = 2.0) -> PhotoSet:
    src = Path(path)
    frames = extract_frames(src, fps)
    images = sorted(frames.glob("*.jpg"))
    size = _image_size(images[0]) if images else None
    return PhotoSet(
        capture_id=src.stem,
        root=src,
        image_paths=images,
        size=size,
        f_px=None,
        fov_deg=65.0,
    )
