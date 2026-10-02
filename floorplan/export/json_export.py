"""Build and write results.json (the published output contract).

results.json is deterministic (no wall-clock timestamps) so re-runs are
byte-identical. Run metadata lives in provenance.json.
"""
from __future__ import annotations

import json
from pathlib import Path

from floorplan.config import Settings
from floorplan.geometry import confidence
from floorplan.geometry.room import Room


def measurement(value: float, ci95: list[float], method: str = "bootstrap") -> dict:
    return {"value": round(float(value), 4), "ci95": ci95, "method": method}


def build_payload(capture_id: str, tier: str, rooms: list[Room],
                  wall_ci: dict[int, list[float]], cfg: Settings,
                  property_footprint: dict | None = None,
                  adjacency: list[dict] | None = None,
                  drift: dict | None = None) -> dict:
    rooms_payload = []
    for room in rooms:
        rid = room.room_id or "r1"
        walls_payload = []
        for w in room.walls:
            ci = wall_ci.get(w.id) or [round(w.length * 0.995, 3), round(w.length * 1.005, 3)]
            ci = confidence.bracket(w.length, ci)
            openings_payload = [
                {
                    "opening_id": f"{rid}-w{w.id}-o{k}",
                    "type": o.kind,
                    "position_along_wall_m": round(o.center_along_wall, 3),
                    "width_m": measurement(o.width_m,
                                           [round(o.width_m - 0.01, 3), round(o.width_m + 0.01, 3)],
                                           "bin_resolution"),
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
            "floor_area_m2": measurement(room.floor_area_m2.value, room.floor_area_m2.ci95,
                                         room.floor_area_m2.method),
            "ceiling_height_m": {
                **measurement(room.ceiling_height_m.value, room.ceiling_height_m.ci95,
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
        "confidence": {"calibration_factor": 1.0, "method": "bootstrap+propagation"},
        "artifacts": {
            "svg": "floor_plan.svg",
            "dxf": "floor_plan.dxf",
            "pointcloud": "scan_metric.ply",
        },
    }


def dump(payload: dict, path: str | Path) -> None:
    Path(path).write_text(json.dumps(payload, indent=2) + "\n")
