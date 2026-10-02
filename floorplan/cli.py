"""floorplan command line interface.

One command per capture:

    floorplan run <capture_path> --out <dir>
"""
from __future__ import annotations

import argparse
import sys

from floorplan import bench, pipeline
from floorplan.config import Settings


def _settings_from_args(a: argparse.Namespace) -> Settings:
    cfg = Settings()
    if getattr(a, "conf_min", None) is not None:
        cfg.conf_min = a.conf_min
    if getattr(a, "max_frames", None) is not None:
        cfg.max_frames = a.max_frames
    if getattr(a, "engine", None):
        cfg.engine = a.engine
    if getattr(a, "scale_ref", None) is not None:
        cfg.scale_ref_m = a.scale_ref
    return cfg


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="floorplan",
        description="Turn a phone capture (Record3D LiDAR / photos / video) into a "
                    "dimensioned floor plan.",
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    r = sub.add_parser("run", help="process one capture into artifacts")
    r.add_argument("input", help="Record3D export (folder or .zip), photo folder, or video file")
    r.add_argument("--out", required=True, help="output directory")
    r.add_argument("--tier", default="auto", choices=["auto", "lidar", "photos", "video"])
    r.add_argument("--engine", default=None, choices=["auto", "monodepth", "colmap"],
                   help="photo/video reconstruction engine (default auto)")
    r.add_argument("--scale-ref", type=float, default=None,
                   help="known real-world length (m) for photo/video scale anchoring")
    r.add_argument("--conf-min", type=int, default=None,
                   help="minimum depth confidence to keep (Record3D: 0/1/2)")
    r.add_argument("--max-frames", type=int, default=None,
                   help="cap on fused depth frames (default 500)")
    r.add_argument("--quiet", action="store_true")

    b = sub.add_parser("bench", help="run the benchmark manifest and score gates")
    b.add_argument("--manifest", required=True, help="benchmark manifest (.json)")
    b.add_argument("--out", required=True, help="output directory for the report")
    b.add_argument("--reuse", default=None,
                   help="directory of cached <capture_id>/results.json to reuse")
    b.add_argument("--conf-min", type=int, default=None)
    b.add_argument("--max-frames", type=int, default=None)
    b.add_argument("--quiet", action="store_true")

    args = ap.parse_args(argv)

    if args.cmd == "run":
        tier = None if args.tier == "auto" else args.tier
        cfg = _settings_from_args(args)
        try:
            pipeline.run(args.input, args.out, cfg=cfg, tier=tier, verbose=not args.quiet)
        except NotImplementedError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 2
        except Exception as exc:  # surface a clean message for the walk-in test
            print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1

    if args.cmd == "bench":
        cfg = _settings_from_args(args)
        try:
            bench.run(args.manifest, args.out, reuse_dir=args.reuse, cfg=cfg,
                      verbose=not args.quiet)
        except Exception as exc:
            print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
