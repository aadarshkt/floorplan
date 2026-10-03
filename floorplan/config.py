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
    max_raw_points: int = 6_000_000  # bound on unprojected points (memory/speed guard)
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

    # ── multi-room ───────────────────────────────────────────────────────────
    node_merge_tol: float = 0.15       # wall endpoints within this are one corner
    min_room_area: float = 2.0
    max_room_area: float = 120.0

    # ── single-room regularisation (noisy photo/video clouds) ─────────────────
    # A noisy monocular cloud peels into many spurious vertical sheets, so a
    # single room can come back with a dozen walls. Above this wall count the
    # room outline is rebuilt from the floor footprint instead of the sheet network.
    max_room_walls: int = 4
    ceiling_min_m: float = 2.1         # plausible ceiling range (habitable rooms are >= ~2.1 m)
    ceiling_max_m: float = 3.6
    ceiling_plane_min_frac: float = 0.10  # a ceiling plane needs this share of the floor plane's points
    wall_outline_band_m: float = 0.40  # cloud points this close to an outline edge belong to it
    outline_simplify_m: float = 0.25   # collar tolerance when simplifying the footprint hull
    outline_max_rect_ratio: float = 1.35  # reject a rectangle that over-covers the observed floor by more

    # ── free-space layout (geometry/layout.py) ───────────────────────────────
    layout: bool = True               # rooms from free space; else the plane-network path
    layout_cell: float = 0.02         # grid cell (m)
    layout_min_pts: int = 2           # band points for a cell to count as wall
    layout_open_m: float = 0.3        # opening radius: cuts leaks narrower than 2x this
    layout_door_close_m: float = 1.2  # wall gaps up to this wide (doors) are sealed along the wall
    layout_seal_min_run_m: float = 0.6  # ...when both sides are wall runs at least this long
    layout_smooth_m: float = 0.06     # close/open radius removing grid noise
    layout_min_edge: float = 0.25     # drop wall jogs shorter than this
    layout_refine_m: float = 0.40     # how far outward to look for the wall face when snapping an edge
    layout_multi_room: bool = True    # every camera-visited free-space component is a room

    # ── drift correction ─────────────────────────────────────────────────────
    drift_correction: bool = True
    auto_up: bool = True               # rotate cloud so estimated gravity = +Z

    # ── openings ─────────────────────────────────────────────────────────────
    opening_bin: float = 0.05
    min_opening_width: float = 0.55   # doors narrower than this are not doors
    min_window_width: float = 0.35
    max_opening_width: float = 3.0
    opening_surface_m: float = 0.12   # wall-face band for opening evidence
    opening_seen_min: int = 30        # points beyond the wall that prove a gap is open
    door_height: float = 2.0
    opening_height_band_top: float = 2.1

    # ── confidence intervals ─────────────────────────────────────────────────
    bootstrap_n: int = 200
    ci_level: float = 0.95
    # width floor: bootstrap captures sampling noise; unmodelled systematic error
    # (LiDAR depth bias, plane fit) needs a floor for honest calibration.
    ci_floor_length_m: float = 0.010
    ci_floor_height_m: float = 0.005
    ci_floor_area_rel: float = 0.010
    # systematic wall-length error per tier (95% half-width, metres): plane fit and
    # corner placement on that tier's cloud. Set from the xbench residuals.
    ci_sys_length_lidar: float = 0.02
    ci_sys_length_video: float = 0.10
    ci_sys_length_photos: float = 0.20

    # ── photo / video tiers ──────────────────────────────────────────────────
    engine: str = "auto"             # auto | monodepth | colmap
    device: str = "auto"             # auto | cpu | mps | cuda (monocular-depth inference)
    # Metric monocular depth: the independent photo/video geometry source. The
    # indoor metric variant returns metres directly, so no anchor is needed.
    metric_model_id: str = "depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf"
    metric_max_points: int = 4_000_000   # cap on the fused metric cloud
    # raw: stack each frame's depth as predicted (original). aligned: fit every
    # frame's depth to the SfM keypoints, filter, and fuse (sfm_depth.py).
    depth_fusion: str = "aligned"
    depth_upright: bool = True        # rotate frames so gravity points down before inference
    depth_consistency_tol: float = 0.04  # relative depth agreement with neighbour views
    depth_consistency_min: int = 1    # neighbour views that must agree
    # Depth-Anything-V2-Metric-Indoor scale calibration (scripts/depth_probe.py vs
    # LiDAR): bias ~ fc / (f / upright height) on 4:3 frames; fc 1.074-1.083 on two
    # camera configurations. Other framings are not calibrated: wide interval.
    depth_focal_canonical: float | None = 1.08
    depth_scale_sys_rel: float = 0.03       # 95% systematic scale error, calibrated framing
    depth_scale_sys_rel_uncal: float = 0.20 # 95% systematic scale error, other framings
    depth_store_dim: int = 640        # long side of each stored depth map (memory bound)
    depth_edge_tol: float = 0.06      # drop pixels whose depth jumps more than this per px (relative)
    scale_ref_m: float | None = None  # known real-world length for scale anchoring
    scale_ref_kind: str = "door_width"
    default_ceiling_m: float = 2.5    # fallback when the ceiling was never observed
    min_photos_for_colmap: int = 12   # below this, monocular depth is used
    video_fps: float = 4.0            # keyframe rate for the video tier (overlap for SfM)
    video_keyframes: str = "sharp"    # sharp (sharpest per window) | uniform
    video_max_frames: int = 120       # keyframe cap; long walks get a lower effective rate
    video_max_dim: int = 1280         # keyframe long side (memory/CPU bound on 8 GB laptops)
    threads: int = 4                  # CPU threads for COLMAP / torch (keep the laptop usable)
    colmap_exhaustive_max: int = 60   # exhaustive matching below this many images
    sfm_features: str = "sift"        # sift | aliked (ALIKED + LightGlue, COLMAP >= 3.13)
    sfm_sift_peak: float = 0.002      # SIFT contrast threshold (COLMAP 0.0067): more keypoints on plain walls
    sfm_max_features: int = 4096      # ALIKED keypoints per image
    sfm_mapper: str = "global"        # incremental | global (GLOMAP, COLMAP >= 3.13)
    sfm_focal_factor: float | None = 0.8  # initial focal / long side for EXIF-less frames
    sfm_max_image_size: int | None = None  # downscale for feature extraction
    sfm_seq_overlap: int = 20         # sequential matcher: neighbours per image
    sfm_global_min_images: int = 30   # below this, incremental mapping
    sfm_init_min_tri_angle: float = 8.0   # handheld clips have little parallax (COLMAP default 16)
    colmap_dense: bool = True         # attempt dense MVS (needs CUDA); else sparse
    mvs_enable: bool = True           # CPU plane-sweep densification from SfM poses
    mvs_max_dim: int = 192            # working resolution (longest image side)
    mvs_neighbors: int = 6            # source views per reference image
    mvs_depth_samples: int = 48       # plane-sweep depth hypotheses
    mvs_cost_tol: float = 0.06        # max photometric cost (0..1 scale) to accept
    mvs_ref_max: int = 40             # reference views to sweep (0 = all)
    mvs_max_points: int = 400_000     # cap on the densified cloud
    anchor_icp_max_dist: float = 1.0  # similarity-ICP correspondence distance (m)
    reference_capture: str | None = None  # paired Record3D capture for scale/geometry
    ci_none_prior_scale: float = 4.0  # widen intervals when scale_reference=none_prior

    def to_dict(self) -> dict:
        return asdict(self)
