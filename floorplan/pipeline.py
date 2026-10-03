"""Pipeline orchestration: capture path -> artifacts.

All three tiers converge on a common intermediate representation (a metric, Z-up
point cloud + provenance) and then share the exact same geometry, confidence and
export stages. That shared tail is what makes the tiers comparable and keeps the
JSON contract identical across them.
"""
from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import open3d as o3d

from floorplan import drift as drift_mod
from floorplan.config import Settings
from floorplan.export import dxf, json_export, provenance, svg
from floorplan.fusion import arbiter, colmap_path, depth_fusion, monodepth, scale
from floorplan.fusion.ir import NATIVE_METRIC, NONE_PRIOR, SCALED, Reconstruction
from floorplan.geometry import align
from floorplan.geometry import confidence
from floorplan.geometry import multiroom
from floorplan.geometry import openings as openings_mod
from floorplan.geometry import planes as planes_mod
from floorplan.geometry import room as room_mod
from floorplan.geometry import walls as walls_mod
from floorplan.ingest import detect, record3d
from floorplan.ingest import photos as photos_mod
from floorplan.ingest import video as video_mod

_NO_DRIFT = {"method": "none", "n_loops": 0, "correction_applied": False,
             "ablation": {"on": None, "off": None}}


def run(capture_path: str | Path, out_dir: str | Path, cfg: Settings | None = None,
        tier: str | None = None, verbose: bool = True) -> dict:
    cfg = cfg or Settings()
    tier = tier or detect.detect_tier(capture_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if tier == "lidar":
        return _run_lidar(capture_path, out, cfg, verbose)
    if tier in ("photos", "video"):
        return _run_photo_video(capture_path, out, cfg, verbose, tier)
    raise ValueError(f"unknown tier '{tier}'")


# ── LiDAR tier ───────────────────────────────────────────────────────────────

def _run_lidar(capture_path: str | Path, out: Path, cfg: Settings, verbose: bool) -> dict:
    timings: dict[str, float] = {}

    t0 = time.time()
    cap = record3d.load(capture_path)
    timings["ingest"] = time.time() - t0

    drift_info = dict(_NO_DRIFT)
    if cfg.drift_correction:
        loops = drift_mod.detect_loops(cap.pos)
        if loops:
            cap.pos, applied = drift_mod.correct_positions(cap.pos, loops)
            drift_info = {"method": "loop_closure_linear", "n_loops": len(loops),
                          "correction_applied": bool(applied),
                          "ablation": {"on": None, "off": None}}
    timings["drift"] = 0.0

    t0 = time.time()
    convention = depth_fusion.detect_pose_convention(cap, cfg)
    pcd = depth_fusion.fuse(cap, cfg, verbose=verbose, convention=convention)
    timings["fusion"] = time.time() - t0
    pts = np.asarray(pcd.points)

    recon = Reconstruction(points=pts, tier="lidar",
                           path_chosen=f"depth_fusion:{convention}",
                           scale_reference=NATIVE_METRIC)
    return _geometry_and_export(pts, cap.capture_id, "lidar", out, cfg, verbose,
                                timings, drift_info, recon, capture_path)


# ── Photos / video tiers ─────────────────────────────────────────────────────

def _run_photo_video(capture_path: str | Path, out: Path, cfg: Settings,
                     verbose: bool, tier: str) -> dict:
    timings: dict[str, float] = {}

    t0 = time.time()
    if tier == "video":
        photos = video_mod.load(capture_path, fps=cfg.video_fps)
    else:
        photos = photos_mod.load(capture_path)
    timings["ingest"] = time.time() - t0
    if verbose:
        print(f"[ingest] {tier}: {photos.n_images} images from {photos.capture_id}")

    # Optional cross-tier reference: a metric capture of the same room.
    reference_points = None
    if cfg.reference_capture:
        ref_cap = record3d.load(cfg.reference_capture)
        conv = depth_fusion.detect_pose_convention(ref_cap, cfg)
        ref_pcd = depth_fusion.fuse(ref_cap, cfg, verbose=False, convention=conv)
        reference_points = np.asarray(ref_pcd.points)
        if verbose:
            print(f"[reference] {ref_cap.capture_id}: {len(reference_points)} metric points")

    candidates: list[Reconstruction] = []
    failures: list[str] = []
    t0 = time.time()

    # COLMAP is opt-in (--engine colmap). On machines without CUDA its dense MVS
    # is unavailable and the CPU path is coarse, so the default photo/video path
    # is monocular depth (learned model if installed, else the paired reference).
    if cfg.engine == "colmap":
        if not colmap_path.available():
            failures.append("colmap: not on PATH")
        else:
            try:
                cr = colmap_path.reconstruct(photos, cfg, out / "colmap", verbose)
                pts, s, ref, notes = scale.anchor(cr.points, cfg, reference_points)
                candidates.append(Reconstruction(
                    points=pts, tier=tier,
                    path_chosen=f"colmap:{cr.dense}",
                    scale_reference=ref, scale_factor=s, conf=cr.conf,
                    notes={**notes, "registered": cr.n_registered, "images": cr.n_images,
                           "points_raw": cr.n_points, "camera_centres": cr.centres,
                           "up_hint": cr.up_hint}))
            except Exception as exc:
                failures.append(f"colmap: {exc}")
                if verbose:
                    print(f"[colmap] failed: {exc}")

    if cfg.engine in ("auto", "monodepth"):
        try:
            r = monodepth.reconstruct(photos, cfg, out, reference_points=reference_points,
                                      verbose=verbose)
            if r.scale_reference != SCALED:  # relative/unscaled: anchor it
                pts, s, ref, notes = scale.anchor(r.points, cfg, reference_points)
                r = replace(r, points=pts, scale_reference=ref, scale_factor=s,
                            notes={**r.notes, **notes})
            candidates.append(r)
        except Exception as exc:
            failures.append(f"monodepth: {exc}")
            if verbose:
                print(f"[monodepth] failed: {exc}")

    timings["fusion"] = time.time() - t0
    if not candidates:
        raise RuntimeError("no reconstruction path succeeded:\n  - " + "\n  - ".join(failures))

    recon, table = arbiter.choose(candidates, cfg, verbose)

    if recon.scale_reference == NONE_PRIOR:
        if verbose:
            print("[scale] no reference: intervals widened, plan marked non-metric")
        cfg = replace(cfg, ci_floor_length_m=cfg.ci_floor_length_m * cfg.ci_none_prior_scale,
                      ci_floor_height_m=cfg.ci_floor_height_m * cfg.ci_none_prior_scale,
                      ci_floor_area_rel=cfg.ci_floor_area_rel * cfg.ci_none_prior_scale)

    drift_info = {"method": "sfm_bundle_adjustment" if "colmap" in recon.path_chosen else "none",
                  "n_loops": 0, "correction_applied": False,
                  "ablation": {"on": None, "off": None}}
    above = recon.notes.get("camera_centres")
    up_hint = recon.notes.get("up_hint")
    return _geometry_and_export(recon.points, photos.capture_id, tier, out, cfg, verbose,
                                timings, drift_info, recon, capture_path,
                                above_points=np.asarray(above) if above is not None else None,
                                up_hint=np.asarray(up_hint) if up_hint is not None else None)


# ── Shared geometry + export tail ────────────────────────────────────────────

def _geometry_and_export(pts: np.ndarray, capture_id: str, tier: str, out: Path,
                         cfg: Settings, verbose: bool, timings: dict, drift_info: dict,
                         recon: Reconstruction, capture_path: str | Path,
                         above_points: np.ndarray | None = None,
                         up_hint: np.ndarray | None = None) -> dict:
    pts = np.asarray(pts, dtype=np.float64)
    if cfg.auto_up and len(pts) > 1000 and not recon.notes.get("prealigned"):
        if tier == "lidar":
            up = align.estimate_up(pts, cfg, above_points=above_points)
            apply = 0.5 < up[2] < 1.0 - 1e-4   # gravity-aligned: modest tilt only
        elif tier in ("photos", "video"):
            # arbitrary SfM frame: pick the up axis that best squares up the room
            up = align.choose_up(pts, cfg, hint=up_hint, above_points=above_points)
            apply = True
        else:
            up = align.estimate_up(pts, cfg, above_points=above_points)
            apply = up[2] < 1.0 - 1e-4         # arbitrary frame: rotate to +Z
        if apply:
            pts = pts @ align.rotation_to_z(up).T
            if verbose:
                print(f"[align] up axis {np.round(up, 3)} -> rotated upright")
    o3d.io.write_point_cloud(str(out / "scan_metric.ply"),
                             o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts)))

    t0 = time.time()
    pls = planes_mod.extract_planes(pts, cfg)
    floor_z, ceil_z, observed, floor_pts, ceil_pts = room_mod.floor_and_ceiling(pls, pts, cfg)
    vplanes = [p for p in pls if p.kind == "vertical"]
    walls = walls_mod.vectorize(vplanes, floor_z, cfg)
    walls = walls_mod.merge_double_walls(walls, cfg)
    walls = walls_mod.snap_orthogonal(walls, cfg)
    for w in walls:
        w.openings = openings_mod.detect(w.start, w.end, w.points, floor_z, cfg)
    timings["geometry"] = time.time() - t0

    t0 = time.time()
    wall_ci: dict[int, list[float]] = {}
    for w in walls:
        bp = w.band_points if w.band_points is not None and len(w.band_points) >= 30 else w.points
        ci = confidence.bootstrap_length(bp, cfg)
        if ci is not None:
            wall_ci[w.id] = ci

    rooms = multiroom.extract_rooms(walls, floor_z, ceil_z, observed,
                                    floor_pts, ceil_pts, cfg)
    if not rooms:
        rooms = [room_mod.assemble(walls, floor_z, ceil_z, observed,
                                   floor_pts, ceil_pts, pts, cfg)]
    adjacency = multiroom.adjacency(rooms) if len(rooms) > 1 else []
    prop = room_mod.property_area(floor_pts, cfg)
    timings["confidence"] = time.time() - t0

    if verbose:
        n_open = sum(len(w.openings) for r in rooms for w in r.walls)
        print(f"[geometry] planes={len(pls)} walls={len(walls)} rooms={len(rooms)} "
              f"openings={n_open} footprint={prop.value} m2 "
              f"ceiling={rooms[0].ceiling_height_m.value} m observed={rooms[0].ceiling_observed}")

    scale_payload = {"scale_reference": recon.scale_reference,
                     "scale_factor": round(float(recon.scale_factor), 4),
                     "metric": recon.scale_reference != NONE_PRIOR}
    payload = json_export.build_payload(
        capture_id, tier, rooms, wall_ci, cfg,
        property_footprint=json_export.measurement(prop.value, prop.ci95, prop.method),
        adjacency=adjacency, drift=drift_info, scale=scale_payload,
    )
    json_export.dump(payload, out / "results.json")
    label = {"lidar": "LiDAR", "photos": "Photos", "video": "Video"}.get(tier, tier)
    svg.render(rooms, out / "floor_plan.svg",
               title=f"{capture_id} — {label} ({len(rooms)} room{'s' if len(rooms) != 1 else ''})")
    dxf.render(rooms, out / "floor_plan.dxf")
    prov = provenance.write(out / "provenance.json", tier=tier, capture_id=capture_id,
                            cfg=cfg, timings=timings, path_chosen=recon.path_chosen,
                            input_path=capture_path,
                            extra={"scale_reference": recon.scale_reference,
                                   "scale_factor": round(float(recon.scale_factor), 4),
                                   "arbiter": recon.notes.get("arbiter", []),
                                   "reference_capture": cfg.reference_capture})
    if verbose:
        print(f"[done] {out}  (total {prov['total_s']}s)")
    return payload
