"""Unpack a Record3D .r3d file into the dataset-style folder used by this project.

Input (.r3d):  Record3D iOS app -> Library -> Export -> Shareable/Internal (.r3d),
               copied to PC via Files app / Share / iTunes File Sharing.
               A .r3d is a ZIP containing: sound.m4a, icon, metadata (JSON),
               rgbd/<i>.jpg + rgbd/<i>.depth (LZFSE float32 meters) + rgbd/<i>.conf (LZFSE uint8).

Output (<out>/): rgb.mp4, depth/%06d.png (uint16 mm), confidence/%06d.png (uint8),
               camera_matrix.csv, odometry.csv, imu.csv (zeros placeholder:
               .r3d contains no IMU), plus <out>.zip next to it.

Layout matches assignment/Assignment/c00a170fe1/.

Requires: python (>=3.10), numpy, pillow, lzfse, ffmpeg on PATH.
  uv pip install numpy pillow lzfse   # in this repo's .venv

Usage:
  python unpack_r3d.py <input.r3d> [-o <out_dir>]
  python unpack_r3d.py 2026-10-03--00-39-56.r3d
"""
import argparse
import io
import json
import shutil
import subprocess
import tempfile
import zipfile
from pathlib import Path

import lzfse
import numpy as np
from PIL import Image


def unpack(src: Path, out: Path, fps_override: int | None = None, crf: int = 18) -> Path:
    z = zipfile.ZipFile(src)
    meta = json.loads(z.read("metadata"))
    fps = fps_override or int(meta.get("fps", 60))
    dw, dh = int(meta["dw"]), int(meta["dh"])
    w, h = int(meta["w"]), int(meta["h"])
    K = meta["K"]  # [fx,0,0, 0,fy,0, cx,cy,1]
    poses = meta["poses"]  # [qx,qy,qz,qw,tx,ty,tz] per frame
    stamps = meta["frameTimestamps"]  # seconds, relative to start
    intr = meta["perFrameIntrinsicCoeffs"]  # [fx,fy,cx,cy] per frame
    n = len(poses)
    print(f"frames={n} rgb={w}x{h} depth={dw}x{dh} fps={fps}")

    names = z.namelist()
    jpg_ids = sorted(
        int(nm.split("/")[1].split(".")[0])
        for nm in names
        if nm.startswith("rgbd/") and nm.endswith(".jpg")
    )
    assert len(jpg_ids) == n, f"jpg count {len(jpg_ids)} != poses {n}"

    depth_dir = out / "depth"
    conf_dir = out / "confidence"
    depth_dir.mkdir(parents=True, exist_ok=True)
    conf_dir.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="r3d_rgb_") as tmp:
        tmp = Path(tmp)
        for i, fid in enumerate(jpg_ids):
            (tmp / f"{i:06d}.jpg").write_bytes(z.read(f"rgbd/{fid}.jpg"))

            raw_d = lzfse.decompress(z.read(f"rgbd/{fid}.depth"))
            assert len(raw_d) == dw * dh * 4, (fid, len(raw_d))
            dm = np.frombuffer(raw_d, dtype="<f4").reshape(dh, dw)
            mm = np.nan_to_num(dm * 1000.0, nan=0.0, posinf=0.0, neginf=0.0)
            mm = np.clip(mm, 0, 65535).astype(np.uint16)
            Image.fromarray(mm).save(depth_dir / f"{i:06d}.png")

            raw_c = lzfse.decompress(z.read(f"rgbd/{fid}.conf"))
            assert len(raw_c) == dw * dh, (fid, len(raw_c))
            cm = np.frombuffer(raw_c, dtype=np.uint8).reshape(dh, dw)
            Image.fromarray(cm, mode="L").save(conf_dir / f"{i:06d}.png")

            if (i + 1) % 300 == 0:
                print(f"  decoded {i + 1}/{n}", flush=True)

        rgb_mp4 = out / "rgb.mp4"
        cmd = ["ffmpeg", "-y", "-framerate", str(fps),
               "-i", str(tmp / "%06d.jpg"),
               "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", str(crf),
               str(rgb_mp4)]
        print("running:", " ".join(cmd), flush=True)
        subprocess.run(cmd, check=True, capture_output=True, text=True)

    with open(out / "camera_matrix.csv", "w") as f:
        f.write(f"{K[0]}, 0.0, {K[6]}\n0.0, {K[4]}, {K[7]}\n0.0, 0.0, 1.0\n")

    with open(out / "odometry.csv", "w") as f:
        f.write("timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy, distortion_center_x, distortion_center_y\n")
        for i in range(n):
            qx, qy, qz, qw, tx, ty, tz = poses[i]
            fx, fy, cx, cy = intr[i]
            f.write(f"{stamps[i]}, {i:06d}, {tx}, {ty}, {tz}, {qx}, {qy}, {qz}, {qw}, {fx}, {fy}, {cx}, {cy}, , \n")

    # .r3d has no IMU stream; keep the expected filename with honest zeros.
    with open(out / "imu.csv", "w") as f:
        f.write("timestamp, a_x, a_y, a_z, alpha_x, alpha_y, alpha_z\n")
        for i in range(n):
            f.write(f"{stamps[i]}, 0, 0, 0, 0, 0, 0\n")

    zip_path = out.parent / (out.name + ".zip")
    if zip_path.exists():
        zip_path.unlink()
    shutil.make_archive(str(out), "zip", root_dir=out.parent, base_dir=out.name)
    print(f"DONE folder={out} zip={zip_path}")
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Unpack Record3D .r3d to dataset folder.")
    ap.add_argument("input", type=Path, help="input .r3d file")
    ap.add_argument("-o", "--out", type=Path, default=None,
                    help="output dir (default: <input stem>/)")
    ap.add_argument("--fps", type=int, default=None, help="override video fps")
    ap.add_argument("--crf", type=int, default=18, help="ffmpeg quality (lower=better)")
    args = ap.parse_args()
    out = args.out or args.input.with_suffix("")
    unpack(args.input, out, fps_override=args.fps, crf=args.crf)


if __name__ == "__main__":
    main()
