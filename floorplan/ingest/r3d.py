"""Record3D ``.r3d`` files, read directly (no manual unpack step).

A ``.r3d`` (Record3D -> Library -> Export -> "Shareable/Internal format") is a
ZIP holding ``metadata`` (JSON: intrinsics ``K``, per-frame ``poses``
``[qx,qy,qz,qw,tx,ty,tz]``, ``perFrameIntrinsicCoeffs``, ``frameTimestamps``,
sizes) and ``rgbd/<i>.jpg | .depth | .conf``; depth (float32 metres) and
confidence (uint8 0/1/2) are LZFSE-compressed.

``unpack`` decodes it once into a cache folder in the dataset layout the rest of
the pipeline reads (``depth/%06d.png`` uint16 mm, ``confidence/%06d.png``,
``odometry.csv``, ``camera_matrix.csv``). The RGB video is only encoded when
asked for (the video tier needs it, the LiDAR tier does not).

Poses in ``.r3d`` are ARKit camera-to-world with OpenGL camera axes (y up,
z backward); that is recorded in ``pose_convention.txt`` so fusion does not
have to guess it.
"""
from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path

import numpy as np
from PIL import Image

POSE_CONVENTION = "c2w_gl"


def is_r3d(path: Path) -> bool:
    if path.suffix.lower() != ".r3d" or not path.is_file():
        return False
    try:
        with zipfile.ZipFile(path) as z:
            return "metadata" in z.namelist()
    except zipfile.BadZipFile:
        return False


def _cache(path: Path) -> Path:
    st = path.stat()
    key = hashlib.sha1(f"{path.resolve()}:{st.st_size}:{int(st.st_mtime)}".encode()).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / "floorplan_cache" / f"r3d_{key}" / path.stem


def _decompress(buf: bytes) -> bytes:
    try:
        import lzfse
    except ImportError as exc:  # pragma: no cover - dependency is declared
        raise RuntimeError("reading .r3d needs the 'lzfse' package (pip install lzfse)") from exc
    return lzfse.decompress(buf)


def unpack(src: str | Path, out: str | Path | None = None, rgb: bool = False,
           crf: int = 18, verbose: bool = False) -> Path:
    """Decode a .r3d into a dataset folder (cached); return the folder."""
    src = Path(src)
    out = Path(out) if out else _cache(src)
    done = out / (".done_rgb" if rgb else ".done")
    if done.exists() or (not rgb and (out / ".done_rgb").exists()):
        return out
    out.mkdir(parents=True, exist_ok=True)

    with zipfile.ZipFile(src) as z:
        meta = json.loads(z.read("metadata"))
        dw, dh = int(meta["dw"]), int(meta["dh"])
        K = meta["K"]                      # column-major 3x3: fx,0,0, 0,fy,0, cx,cy,1
        poses = meta["poses"]
        stamps = meta.get("frameTimestamps") or [i / float(meta.get("fps", 60)) for i in range(len(poses))]
        intr = meta.get("perFrameIntrinsicCoeffs") or [[K[0], K[4], K[6], K[7]]] * len(poses)
        n = len(poses)
        ids = sorted(int(nm.split("/")[1].split(".")[0]) for nm in z.namelist()
                     if nm.startswith("rgbd/") and nm.endswith(".depth"))
        if len(ids) != n:
            raise RuntimeError(f"{src.name}: {len(ids)} depth frames but {n} poses")
        if verbose:
            print(f"[r3d] {src.name}: {n} frames, rgb {meta['w']}x{meta['h']}, depth {dw}x{dh}")

        (out / "depth").mkdir(exist_ok=True)
        (out / "confidence").mkdir(exist_ok=True)
        tmp_rgb = Path(tempfile.mkdtemp(prefix="r3d_rgb_")) if rgb else None
        try:
            for i, fid in enumerate(ids):
                d = np.frombuffer(_decompress(z.read(f"rgbd/{fid}.depth")), dtype="<f4").reshape(dh, dw)
                mm = np.clip(np.nan_to_num(d * 1000.0, nan=0.0, posinf=0.0, neginf=0.0), 0, 65535)
                Image.fromarray(mm.astype(np.uint16)).save(out / "depth" / f"{i:06d}.png")
                conf_name = f"rgbd/{fid}.conf"
                if conf_name in z.NameToInfo:
                    c = np.frombuffer(_decompress(z.read(conf_name)), dtype=np.uint8).reshape(dh, dw)
                    Image.fromarray(c, mode="L").save(out / "confidence" / f"{i:06d}.png")
                if tmp_rgb is not None:
                    (tmp_rgb / f"{i:06d}.jpg").write_bytes(z.read(f"rgbd/{fid}.jpg"))
            if tmp_rgb is not None:
                fps = int(meta.get("fps", 60))
                subprocess.run(["ffmpeg", "-y", "-v", "error", "-framerate", str(fps),
                                "-i", str(tmp_rgb / "%06d.jpg"), "-c:v", "libx264",
                                "-pix_fmt", "yuv420p", "-crf", str(crf), str(out / "rgb.mp4")],
                               check=True)
        finally:
            if tmp_rgb is not None:
                shutil.rmtree(tmp_rgb, ignore_errors=True)

    (out / "camera_matrix.csv").write_text(
        f"{K[0]}, 0.0, {K[6]}\n0.0, {K[4]}, {K[7]}\n0.0, 0.0, 1.0\n")
    with open(out / "odometry.csv", "w") as f:
        f.write("timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy\n")
        for i in range(n):
            qx, qy, qz, qw, tx, ty, tz = poses[i]
            fx, fy, cx, cy = intr[i]
            f.write(f"{stamps[i]}, {i:06d}, {tx}, {ty}, {tz}, {qx}, {qy}, {qz}, {qw}, "
                    f"{fx}, {fy}, {cx}, {cy}\n")
    (out / "pose_convention.txt").write_text(POSE_CONVENTION + "\n")
    (out / "source.json").write_text(json.dumps(
        {"source": str(src.resolve()), "frames": n, "rgb_size": [meta["w"], meta["h"]],
         "depth_size": [dw, dh], "fps": meta.get("fps")}, indent=1))
    done.write_text("ok")
    return out
