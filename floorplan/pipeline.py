"""Pipeline orchestration: capture path -> artifacts."""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import open3d as o3d

from floorplan import drift as drift_mod
from floorplan.config import Settings
from floorplan.export import dxf, json_export, provenance, svg
from floorplan.fusion import depth_fusion
from floorplan.geometry import confidence
from floorplan.geometry import multiroom
from floorplan.geometry import openings as openings_mod
from floorplan.geometry import planes as planes_mod
from floorplan.geometry import room as room_mod
from floorplan.geometry import walls as walls_mod
from floorplan.ingest import detect, record3d


def run(capture_path: str | Path, out_dir: str | Path, cfg: Settings | None = None,
        tier: str | None = None, verbose: bool = True) -> dict:
    cfg = cfg or Settings()
    tier = tier or detect.detect_tier(capture_path)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    if tier == "lidar":
        return _run_lidar(capture_path, out, cfg, verbose)
    raise NotImplementedError(
        f"tier '{tier}' is not implemented yet (photos/video land in P2). "
        f"Only 'lidar' (Record3D export) is available."
    )


def _run_lidar(capture_path: str | Path, out: Path, cfg: Settings, verbose: bool) -> dict:
    timings: dict[str, float] = {}

    t0 = time.time()
    cap = record3d.load(capture_path)
    timings["ingest"] = time.time() - t0

    drift_info = {"method": "none", "n_loops": 0, "correction_applied": False,
                  "ablation": {"on": None, "off": None}}
    if cfg.drift_correction:
        loops = drift_mod.detect_loops(cap.pos)
        if loops:
            cap.pos, applied = drift_mod.correct_positions(cap.pos, loops)
            drift_info = {"method": "loop_closure_linear", "n_loops": len(loops),
                          "correction_applied": bool(applied),
                          "ablation": {"on": None, "off": None}}
    timings["drift"] = 0.0

    t0 = time.time()
    pcd = depth_fusion.fuse(cap, cfg, verbose=verbose)
    timings["fusion"] = time.time() - t0
    o3d.io.write_point_cloud(str(out / "scan_metric.ply"), pcd)
    pts = np.asarray(pcd.points)

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
              f"ceiling={rooms[0].ceiling_height_m.value} m observed={rooms[0].ceiling_observed} "
              f"drift={drift_info['method']}({drift_info['n_loops']})")

    payload = json_export.build_payload(
        cap.capture_id, "lidar", rooms, wall_ci, cfg,
        property_footprint=json_export.measurement(prop.value, prop.ci95, prop.method),
        adjacency=adjacency, drift=drift_info,
    )
    json_export.dump(payload, out / "results.json")
    svg.render(rooms, out / "floor_plan.svg",
               title=f"{cap.capture_id} — LiDAR ({len(rooms)} room{'s' if len(rooms) != 1 else ''})")
    dxf.render(rooms, out / "floor_plan.dxf")
    prov = provenance.write(out / "provenance.json", tier="lidar", capture_id=cap.capture_id,
                            cfg=cfg, timings=timings, path_chosen="depth_fusion",
                            input_path=capture_path)
    if verbose:
        print(f"[done] {out}  (total {prov['total_s']}s)")
    return payload
