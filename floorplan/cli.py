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
    if getattr(a, "device", None):
        cfg.device = a.device
    if getattr(a, "scale_ref", None) is not None:
        cfg.scale_ref_m = a.scale_ref
    if getattr(a, "scale_ref_kind", None):
        cfg.scale_ref_kind = a.scale_ref_kind
    if getattr(a, "reference_capture", None):
        cfg.reference_capture = a.reference_capture
    if getattr(a, "fps", None) is not None:
        cfg.video_fps = a.fps
    if getattr(a, "no_drift_correction", False):
        cfg.drift_correction = False
    return cfg


def _add_recon_flags(p: argparse.ArgumentParser) -> None:
    p.add_argument("--engine", default=None, choices=["auto", "monodepth", "colmap"],
                   help="photo/video reconstruction engine (default auto)")
    p.add_argument("--device", default=None, choices=["auto", "cpu", "mps", "cuda"],
                   help="inference device for the monocular-depth model (default auto)")
    p.add_argument("--scale-ref", type=float, default=None,
                   help="known real-world length (m) used to anchor photo/video scale")
    p.add_argument("--scale-ref-kind", default=None,
                   choices=["door_width", "ceiling_height", "room_height", "door_height"],
                   help="what --scale-ref measures (default ceiling_height for anchoring)")
    p.add_argument("--reference-capture", default=None,
                   help="paired Record3D capture of the same room (metric scale/geometry)")
    p.add_argument("--fps", type=float, default=None,
                   help="keyframe rate for the video tier (default 2)")


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
    _add_recon_flags(r)
    r.add_argument("--conf-min", type=int, default=None,
                   help="minimum depth confidence to keep (Record3D: 0/1/2)")
    r.add_argument("--max-frames", type=int, default=None,
                   help="cap on fused depth frames (default 500)")
    r.add_argument("--no-drift-correction", action="store_true",
                   help="use poses as-is (for the drift on/off ablation)")
    r.add_argument("--quiet", action="store_true")

    b = sub.add_parser("bench", help="run the benchmark manifest and score gates")
    b.add_argument("--manifest", required=True, help="benchmark manifest (.json)")
    b.add_argument("--out", required=True, help="report output directory (gates.json, report.md)")
    b.add_argument("--runs", default=None,
                   help="where per-capture pipeline outputs live (default: <out>/runs). "
                        "Existing results.json here are reused unless --force")
    b.add_argument("--force", action="store_true", help="re-run the pipeline for every capture")
    _add_recon_flags(b)
    b.add_argument("--conf-min", type=int, default=None)
    b.add_argument("--max-frames", type=int, default=None)
    b.add_argument("--no-drift-correction", action="store_true")
    b.add_argument("--ablate-drift", action="store_true",
                   help="also run each capture with drift correction OFF and report the delta")
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
            bench.run(args.manifest, args.out, runs_dir=args.runs, cfg=cfg,
                      verbose=not args.quiet, ablate_drift=args.ablate_drift,
                      force=args.force)
        except Exception as exc:
            print(f"error: {type(exc).__name__}: {exc}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
