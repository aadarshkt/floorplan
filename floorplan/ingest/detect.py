"""Tier autodetection: decide how to interpret an input path."""
from __future__ import annotations

import zipfile
from pathlib import Path

IMAGE_EXT = {".jpg", ".jpeg", ".png", ".heic", ".heif", ".webp"}
VIDEO_EXT = {".mp4", ".mov", ".m4v", ".avi"}


def detect_tier(path: str | Path) -> str:
    """Return one of 'lidar', 'photos', 'video'.

    lidar  : Record3D .r3d file, or an export folder / ZIP containing odometry.csv
    photos : a folder of still images
    video  : a single video file
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(p)

    if p.is_file():
        if p.suffix.lower() == ".r3d":
            return "lidar"
        if p.suffix.lower() == ".zip":
            with zipfile.ZipFile(p) as z:
                names = z.namelist()
            if any(n.endswith("odometry.csv") for n in names):
                return "lidar"
            raise ValueError(
                f"ZIP {p.name} has no odometry.csv; expected a Record3D export."
            )
        if p.suffix.lower() in VIDEO_EXT:
            return "video"
        if p.suffix.lower() in IMAGE_EXT:
            return "photos"
        raise ValueError(f"Unrecognised file type: {p.suffix}")

    # directory
    if (p / "odometry.csv").exists() or any(
        (c / "odometry.csv").exists() for c in p.iterdir() if c.is_dir()
    ):
        return "lidar"
    images = [f for f in p.rglob("*") if f.suffix.lower() in IMAGE_EXT]
    videos = [f for f in p.iterdir() if f.is_file() and f.suffix.lower() in VIDEO_EXT]
    if images:
        return "photos"
    if videos:
        return "video"
    raise ValueError(f"Could not detect tier for {p}")
