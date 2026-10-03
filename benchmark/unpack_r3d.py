"""Unpack a Record3D .r3d into a dataset folder (rgb.mp4, depth/, confidence/, odometry.csv).

`floorplan run <file>.r3d` reads .r3d directly, so this is only needed when you
want the unpacked folder itself (e.g. to run the video tier on its rgb.mp4).

    ../.venv/bin/python unpack_r3d.py <input.r3d> [-o <out_dir>]
"""
import argparse
from pathlib import Path

from floorplan.ingest import r3d


def main() -> None:
    ap = argparse.ArgumentParser(description="Unpack Record3D .r3d to a dataset folder.")
    ap.add_argument("input", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=None, help="output dir (default: <input stem>/)")
    ap.add_argument("--crf", type=int, default=18, help="rgb.mp4 quality (lower = better)")
    a = ap.parse_args()
    out = r3d.unpack(a.input, a.out or a.input.with_suffix(""), rgb=True, crf=a.crf, verbose=True)
    print(f"DONE folder={out}")


if __name__ == "__main__":
    main()
