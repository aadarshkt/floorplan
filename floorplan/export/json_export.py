"""Build and write results.json (the published output contract).

results.json is deterministic (no wall-clock timestamps) so re-runs are
byte-identical. Run metadata lives in provenance.json.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from floorplan.config import Settings
from floorplan.geometry import confidence
from floorplan.geometry.room import Room


def measurement(value: float, ci95: list[float], method: str = "bootstrap") -> dict:
    return {"value": round(float(value), 4), "ci95": ci95, "method": method}


def widen(value: float, ci: list[float], rel: float = 0.0, floor: float = 0.0) -> list[float]:
    """Combine sampling CI with systematic error: scale (relative) and a geometry floor.

    Each side's half-width becomes hypot(sampling half, rel * value, floor), so an
    interval reflects how far the tier can really be off, not just point noise.
    """
    v = float(value)
    lo = (v - float(ci[0])) if ci else 0.0
    hi = (float(ci[1]) - v) if ci else 0.0
    sys = (rel * abs(v), floor)
    return [round(v - float(np.hypot(max(lo, 0.0), np.hypot(*sys))), 4),
            round(v + float(np.hypot(max(hi, 0.0), np.hypot(*sys))), 4)]


def build_payload(capture_id: str, tier: str, rooms: list[Room],
                  wall_ci: dict[int, list[float]], cfg: Settings,
                  property_footprint: dict | None = None,
                  adjacency: list[dict] | None = None,
                  drift: dict | None = None,
                  scale: dict | None = None,
                  scale_rel: float = 0.0) -> dict:
    # tier geometry floors (95% half-widths), calibrated on the xbench residuals
    floor_len = {"lidar": cfg.ci_sys_length_lidar, "video": cfg.ci_sys_length_video,
                 "photos": cfg.ci_sys_length_photos}.get(tier, 0.0)
    rooms_payload = []
    for room in rooms:
        rid = room.room_id or "r1"
        walls_payload = []
        for w in room.walls:
            ci = wall_ci.get(w.id) or [round(w.length * 0.995, 3), round(w.length * 1.005, 3)]
            ci = widen(w.length, confidence.interval(w.length, ci, cfg.ci_floor_length_m),
                       scale_rel, floor_len)
            openings_payload = [
                {
                    "opening_id": f"{rid}-w{w.id}-o{k}",
                    "type": o.kind,
                    "position_along_wall_m": round(o.center_along_wall, 3),
                    "width_m": measurement(o.width_m,
                                           widen(o.width_m, [o.width_m, o.width_m],
                                                 scale_rel, floor_len),
                                           "jamb_to_jamb"),
                    "seen_through": bool(getattr(o, "seen_through", False)),
                    "sill_m": o.sill_m,
                    "head_m": o.head_m,
                }
                for k, o in enumerate(w.openings)
            ]
            walls_payload.append({
                "wall_id": f"{rid}-w{w.id}",
                "start": [round(float(w.start[0]), 3), round(float(w.start[1]), 3)],
                "end": [round(float(w.end[0]), 3), round(float(w.end[1]), 3)],
                "length_m": measurement(w.length, ci),
                "openings": openings_payload,
            })

        rooms_payload.append({
            "room_id": rid,
            "polygon": room.polygon,
            "floor_area_m2": measurement(
                room.floor_area_m2.value,
                widen(room.floor_area_m2.value, room.floor_area_m2.ci95, 2 * scale_rel,
                      2 * floor_len * float(np.sqrt(max(room.floor_area_m2.value, 0.0)))),
                room.floor_area_m2.method),
            "ceiling_height_m": {
                **measurement(room.ceiling_height_m.value,
                              widen(room.ceiling_height_m.value, room.ceiling_height_m.ci95,
                                    scale_rel, floor_len / 2),
                              room.ceiling_height_m.method),
                "observed": room.ceiling_observed,
            },
            "walls": walls_payload,
            "damage_regions": [],
            "concealed_damage_flags": [],
            "scope_line_items": [],
        })

    if property_footprint is None:
        total = round(sum(r["floor_area_m2"]["value"] for r in rooms_payload), 3)
        property_footprint = measurement(total, [round(total * 0.97, 3), round(total * 1.03, 3)],
                                         "sum_of_rooms")

    return {
        "schema_version": "1.0",
        "capture_id": capture_id,
        "tier": tier,
        "units": "meters",
        "property": {
            "footprint_area_m2": property_footprint,
            "rooms": [r["room_id"] for r in rooms_payload],
            "adjacency": adjacency or [],
        },
        "rooms": rooms_payload,
        "drift": drift or {"method": "none", "n_loops": 0, "correction_applied": False,
                           "ablation": {"on": None, "off": None}},
        "scale": scale or {"scale_reference": "native_metric", "scale_factor": 1.0,
                           "metric": True},
        "confidence": {"calibration_factor": 1.0, "method": "bootstrap+propagation"},
        "artifacts": {
            "svg": "floor_plan.svg",
            "dxf": "floor_plan.dxf",
            "pointcloud": "scan_metric.ply",
        },
    }


def dump(payload: dict, path: str | Path) -> None:
    Path(path).write_text(json.dumps(payload, indent=2) + "\n")
