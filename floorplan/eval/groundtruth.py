"""Ground-truth format and loader (what was measured with a laser / tape).

One file per capture. Each room lists what was measured; ``null`` (or a missing
field) means "not measured" and is simply not scored::

    {
      "capture": "benchmark_2",
      "instrument": "Bosch GLM 50 C laser, ±1.5 mm",
      "rooms": [
        {
          "room_id": "r1",
          "ceiling_height_m": [2.851, 2.848, 2.856],   # several readings: averaged
          "floor_area_m2": null,
          "walls": [
            {"wall_id": "r1-w0", "length_m": 2.553},     # id as drawn on floor_plan.svg
            {"length_m": 2.891}                          # no id: matched by length
          ],
          "openings": [
            {"type": "door", "width_m": 0.812, "wall_id": "r1-w6"}
          ]
        }
      ]
    }

The older single-room form ({"room_id", "ceiling_height_m", "wall_lengths_m",
"openings"}) is still accepted. ``floorplan gt-template <run_dir>`` writes a
file in this format with every wall and opening of a run, ready to fill in.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class GroundTruth:
    room_id: str
    ceiling_height_m: float | None
    floor_area_m2: float | None
    wall_lengths_m: list[float]
    openings: list[dict]
    wall_ids: list[str | None] = field(default_factory=list)   # parallel to wall_lengths_m
    wall_mids: list[list[float] | None] = field(default_factory=list)  # plan midpoints, if known
    ceiling_readings: list[float] = field(default_factory=list)

    @staticmethod
    def from_dict(d: dict, fallback_room_id: str = "r1") -> "GroundTruth":
        ceil = d.get("ceiling_height_m")
        readings = [float(v) for v in ceil if v is not None] if isinstance(ceil, list) else (
            [float(ceil)] if ceil is not None else [])
        walls, ids, mids = [], [], []
        for w in d.get("walls", []):
            if w.get("length_m") is not None:
                walls.append(float(w["length_m"]))
                ids.append(w.get("wall_id"))
                mids.append(w.get("_plan_mid"))
        for v in d.get("wall_lengths_m", []):
            if v is not None:
                walls.append(float(v))
                ids.append(None)
                mids.append(None)
        return GroundTruth(
            room_id=d.get("room_id", fallback_room_id),
            ceiling_height_m=(sum(readings) / len(readings)) if readings else None,
            floor_area_m2=d.get("floor_area_m2"),
            wall_lengths_m=walls,
            openings=[{"type": o.get("type", "door"), "width_m": float(o["width_m"]),
                       "wall_id": o.get("wall_id")}
                      for o in d.get("openings", []) if o.get("width_m") is not None],
            wall_ids=ids,
            wall_mids=mids,
            ceiling_readings=readings,
        )


def load_rooms(path: str | Path, fallback_room_id: str = "r1") -> list[GroundTruth]:
    d = json.loads(Path(path).read_text())
    if "rooms" in d:
        return [GroundTruth.from_dict(r, r.get("room_id", fallback_room_id)) for r in d["rooms"]]
    return [GroundTruth.from_dict(d, fallback_room_id)]


def load(path: str | Path, fallback_room_id: str = "r1") -> GroundTruth:
    return load_rooms(path, fallback_room_id)[0]


def template(results: dict, capture: str) -> dict:
    """A ground-truth skeleton listing every produced wall and opening (values null)."""
    rooms = []
    for r in results.get("rooms", []):
        rooms.append({
            "room_id": r["room_id"],
            "name": "",
            "ceiling_height_m": [None, None, None],
            "floor_area_m2": None,
            # _plan_mid locates the wall in the plan, so a measurement stays attached
            # to the right wall even if a later code version renumbers walls
            "walls": [{"wall_id": w["wall_id"], "length_m": None,
                       "_plan_mid": [round((w["start"][0] + w["end"][0]) / 2, 2),
                                     round((w["start"][1] + w["end"][1]) / 2, 2)]}
                      for w in r.get("walls", [])],
            "openings": [{"type": o["type"], "width_m": None, "wall_id": w["wall_id"]}
                         for w in r.get("walls", []) for o in w.get("openings", [])],
            "_missing_openings_help": "add {type, width_m, wall_id} for any door/window "
                                      "the plan did not show",
        })
    return {"capture": capture, "instrument": "", "measured_by": "", "date": "",
            "rooms": rooms}
