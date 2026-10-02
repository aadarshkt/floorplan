"""Central configuration for the pipeline.

All tunables live here so every stage is reproducible and the benchmark can
override parameters without code changes. Values are deliberately conservative
defaults for indoor handheld captures.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass
class Settings:
    # ── ingest / fusion ──────────────────────────────────────────────────────
    conf_min: int = 1            # keep depth pixels with confidence >= this (Record3D: 0/1/2)
    max_frames: int = 500        # cap frames fused (stride chosen to fit); plenty for a room
    px_stride: int = 1           # subsample depth pixels (1 = every pixel)
    depth_min_m: float = 0.10    # discard depths closer than this
    depth_max_m: float = 8.0     # discard depths beyond this (LiDAR range)
    voxel_size: float = 0.01     # voxel downsample size (metres)
    stat_nb_neighbors: int = 20  # statistical outlier removal
    stat_std_ratio: float = 2.0

    # ── plane extraction ─────────────────────────────────────────────────────
    ransac_dist: float = 0.02
    ransac_iters: int = 1000
    ransac_seed: int = 42
    max_planes: int = 40
    min_plane_points: int = 800
    horizontal_normal_z: float = 0.80   # |nz| >= -> floor/ceiling
    vertical_normal_z_max: float = 0.70  # |nz| <= -> wall

    # ── wall vectorisation ───────────────────────────────────────────────────
    wall_band_low: float = 0.5   # metres above floor
    wall_band_high: float = 1.5
    min_wall_len: float = 0.4
    wall_merge_max_gap: float = 0.35   # two faces of one wall are <= this apart
    wall_merge_angle_deg: float = 12.0
    snap_orthogonal: bool = True
    ortho_snap_max_deg: float = 20.0   # don't snap beyond this; flag slanted rooms

    # ── openings ─────────────────────────────────────────────────────────────
    opening_bin: float = 0.05
    min_opening_width: float = 0.5
    door_height: float = 2.0
    opening_height_band_top: float = 2.1

    # ── confidence intervals ─────────────────────────────────────────────────
    bootstrap_n: int = 200
    ci_level: float = 0.95

    # ── photo / video tiers ──────────────────────────────────────────────────
    engine: str = "auto"             # auto | monodepth | colmap
    scale_ref_m: float | None = None  # known real-world length for scale anchoring
    scale_ref_kind: str = "door_width"
    default_ceiling_m: float = 2.5    # fallback when the ceiling was never observed
    min_photos_for_colmap: int = 12   # below this, monocular depth is used

    def to_dict(self) -> dict:
        return asdict(self)
