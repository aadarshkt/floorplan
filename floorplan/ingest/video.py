"""Video ingest (the video tier).

A walkthrough clip is reduced to keyframes, then handled exactly like a photo
set. Two selection modes:

  uniform : the frame nearest each 1/fps tick (the original behaviour).
  sharp   : candidates every few source frames; within each window keep the
            sharpest one (variance of the Laplacian). Handheld clips are full of
            motion blur, and a blurred keyframe is a keyframe SfM cannot register.

Frames are always addressed by their *source frame index* (written to
``frames.json``), never by timestamp arithmetic: phone clips are often
variable-frame-rate, so ``ffmpeg -vf fps=N`` cannot tell you which frame it
picked. Exact indices are what lets the video tier be scored against the LiDAR
poses of the same clip.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np

from floorplan.ingest.photos import PhotoSet, _image_size


def _cache_dir(path: Path, key: str) -> Path:
    st = path.stat()
    h = hashlib.sha1(f"{path.resolve()}:{st.st_size}:{int(st.st_mtime)}:{key}".encode()).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / "floorplan_cache" / f"video_{h}"


def frame_times(path: Path) -> np.ndarray:
    """Presentation time (s) of every frame, in display order."""
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "packet=pts_time", "-of", "csv=p=0", str(path)],
        capture_output=True, text=True, check=True, timeout=300).stdout
    ts = np.array(sorted(float(s.strip().strip(",")) for s in out.split() if s.strip().strip(",")))
    if len(ts) == 0:
        raise RuntimeError(f"ffprobe found no video frames in {path}")
    return ts - ts[0]


def _extract_stride(src: Path, stride: int, n_frames: int, out: Path, max_dim: int) -> list[int]:
    """Decode every ``stride``-th source frame to ``out/<index:06d>.jpg``.

    Selecting by frame number (``not(mod(n,S))``) keeps the index exact even for
    variable-frame-rate clips, and needs a single decode pass.
    """
    tmp = out / "_tmp"
    if tmp.exists():
        shutil.rmtree(tmp)
    tmp.mkdir(parents=True)
    scale = (f",scale='if(gt(iw,ih),min({max_dim},iw),-2)':'if(gt(iw,ih),-2,min({max_dim},ih))'"
             if max_dim else "")
    cmd = ["ffmpeg", "-v", "error", "-i", str(src), "-vf", f"select='not(mod(n\\,{stride}))'{scale}",
           "-fps_mode", "vfr", "-q:v", "2", str(tmp / "%06d.jpg")]
    subprocess.run(cmd, check=True, timeout=3600)
    got = sorted(tmp.glob("*.jpg"))
    expect = list(range(0, n_frames, stride))
    # the container can list a trailing frame the decoder never emits; selection
    # is by frame number, so the frames we did get are still the first ones
    if not (len(expect) - 2 <= len(got) <= len(expect)):
        raise RuntimeError(f"ffmpeg returned {len(got)} frames, expected {len(expect)}")
    expect = expect[:len(got)]
    for p, i in zip(got, expect):
        p.rename(out / f"{i:06d}.jpg")
    shutil.rmtree(tmp)
    return expect


def _sharpness(path: Path) -> float:
    """Variance of the Laplacian on a ~480 px greyscale copy (blur score)."""
    from PIL import Image
    with Image.open(path) as im:
        g = im.convert("L")
        s = 480 / max(g.size)
        if s < 1:
            g = g.resize((max(1, round(g.size[0] * s)), max(1, round(g.size[1] * s))))
        a = np.asarray(g, dtype=np.float32)
    lap = (a[1:-1, :-2] + a[1:-1, 2:] + a[:-2, 1:-1] + a[2:, 1:-1] - 4.0 * a[1:-1, 1:-1])
    return float(lap.var())


def extract_frames(path: str | Path, fps: float = 2.0, mode: str = "sharp",
                   candidate_fps: float = 8.0, max_frames: int = 300,
                   max_dim: int = 1920) -> tuple[Path, list[int]]:
    """Pick keyframes; return (frames dir, source indices in order)."""
    src = Path(path)
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required to read video but was not found on PATH")
    out = _cache_dir(src, f"{mode}:{fps}:{candidate_fps}:{max_frames}:{max_dim}:v3")
    meta = out / "frames.json"
    if meta.exists():
        return out, json.loads(meta.read_text())["source_index"]
    if out.exists():
        shutil.rmtree(out)
    out.mkdir(parents=True)

    ts = frame_times(src)
    duration = float(ts[-1]) if len(ts) > 1 else 0.0
    # Bound the keyframe count: a long multi-room walk would otherwise swamp SfM.
    eff_fps = min(fps, max_frames / duration) if duration > 0 else fps

    if mode not in ("uniform", "sharp"):
        raise ValueError(f"unknown keyframe mode '{mode}'")
    # candidates: every S-th source frame, ~candidate_fps of them per second
    src_fps = (len(ts) - 1) / duration if duration > 0 else 30.0
    stride = max(1, int(round(src_fps / max(candidate_fps, eff_fps))))
    cands = _extract_stride(src, stride, len(ts), out, max_dim)
    ct = ts[cands]
    scores: dict[int, float] = {}
    if mode == "uniform":
        # the candidate nearest each 1/fps tick
        ticks = np.arange(0.0, duration + 1e-9, 1.0 / eff_fps)
        chosen = sorted({cands[int(np.argmin(np.abs(ct - tk)))] for tk in ticks})
    else:
        scores = {i: _sharpness(out / f"{i:06d}.jpg") for i in cands}
        # one keyframe per 1/eff_fps window: the sharpest candidate in it
        win = np.floor(ct * eff_fps).astype(int)
        chosen = []
        for w in np.unique(win):
            members = [c for c, wi in zip(cands, win) if wi == w]
            chosen.append(max(members, key=lambda c: scores[c]))
    keep = set(chosen)
    for c in cands:
        if c not in keep:
            (out / f"{c:06d}.jpg").unlink()

    meta.write_text(json.dumps({
        "source": str(src.resolve()), "mode": mode, "fps": eff_fps,
        "n_source_frames": int(len(ts)), "source_index": chosen,
        "time_s": [round(float(ts[i]), 4) for i in chosen],
        "sharpness": [round(scores[i], 2) for i in chosen] if scores else None,
    }, indent=1))
    return out, chosen


def load(path: str | Path, fps: float = 2.0, mode: str = "sharp",
         max_frames: int = 300, max_dim: int = 1920) -> PhotoSet:
    src = Path(path)
    frames, idx = extract_frames(src, fps, mode=mode, max_frames=max_frames, max_dim=max_dim)
    images = [frames / f"{i:06d}.jpg" for i in idx]
    size = _image_size(images[0]) if images else None
    return PhotoSet(
        capture_id=src.stem if src.stem != "rgb" else src.parent.name,
        root=src,
        image_paths=images,
        size=size,
        f_px=None,
        fov_deg=65.0,
        source_index=idx,
        ordered=True,
    )
