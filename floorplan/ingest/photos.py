"""Photo-set ingest (the photos tier).

A photo set is a folder of stills (2-8 for a sparse room; more if the walk
covers several rooms). We read the camera focal length from EXIF when present
(focal-length-in-35mm-film / 36 mm * image width) so a single-image depth model
can unproject pixels; otherwise we fall back to a horizontal-FOV prior.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from floorplan.ingest.detect import IMAGE_EXT

DEFAULT_HFOV_DEG = 65.0  # typical phone main camera


@dataclass
class PhotoSet:
    capture_id: str
    root: Path
    image_paths: list[Path]
    size: tuple[int, int] | None      # (w, h) of the first image
    f_px: float | None                # focal length in pixels (single-camera assumption)
    fov_deg: float | None
    source_index: list[int] | None = None   # video tier: source frame number per image

    @property
    def n_images(self) -> int:
        return len(self.image_paths)


def _exif_focal_px(path: Path, size: tuple[int, int]) -> float | None:
    try:
        from PIL import Image
        with Image.open(path) as im:
            exif = im.getexif()
    except Exception:
        return None
    w, _h = size
    f35 = exif.get(41989)  # FocalLengthIn35mmFilm
    if f35:
        try:
            return float(f35) / 36.0 * w
        except (TypeError, ValueError):
            pass
    # FocalLength (tag 37386) is the *physical* focal length (~6-7 mm on an iPhone);
    # without the sensor width it cannot be converted to pixels, so it is not used.
    return None


def _image_size(path: Path) -> tuple[int, int] | None:
    try:
        from PIL import Image
        with Image.open(path) as im:
            return im.size
    except Exception:
        return None


def _collect_images(path: Path) -> list[Path]:
    images = [p for p in sorted(path.rglob("*")) if p.suffix.lower() in IMAGE_EXT]
    # ignore tiny thumbnails / hidden files
    return [p for p in images if not p.name.startswith(".")]


def load(path: str | Path) -> PhotoSet:
    root = Path(path)
    images = _collect_images(root)
    if not images:
        raise FileNotFoundError(f"No images found under {root}")
    size = _image_size(images[0])
    f_px = _exif_focal_px(images[0], size) if size else None
    fov = None
    if size and not f_px:
        fov = DEFAULT_HFOV_DEG
    # stills named by frame number (e.g. sampled from a Record3D clip) keep that
    # number, so a photo run can be scored against the clip's LiDAR poses
    stems = [p.stem for p in images]
    source_index = [int(t) for t in stems] if all(t.isdigit() for t in stems) else None
    return PhotoSet(
        capture_id=root.name,
        root=root,
        image_paths=images,
        size=size,
        f_px=f_px,
        fov_deg=fov,
        source_index=source_index,
    )
