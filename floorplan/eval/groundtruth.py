"""Ground-truth format and loader.

Ground truth is what the operator measured with a laser/tape. It is deliberately
simple so it can be produced by hand at capture time:

    {
      "room_id": "room1",
      "ceiling_height_m": 2.42,
      "floor_area_m2": 13.1,
      "wall_lengths_m": [4.20, 3.15, 4.18, 3.10],
      "openings": [{"type": "door", "width_m": 0.82},
                   {"type": "window", "width_m": 1.20}]
    }

Wall lengths are an unordered multiset (order need not match the produced plan);
the harness matches them optimally. Optional fields may be omitted.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path


@dataclass
class GroundTruth:
    room_id: str
    ceiling_height_m: float | None
    floor_area_m2: float | None
    wall_lengths_m: list[float]
    openings: list[dict]

    @staticmethod
    def from_dict(d: dict, fallback_room_id: str = "r1") -> "GroundTruth":
        return GroundTruth(
            room_id=d.get("room_id", fallback_room_id),
            ceiling_height_m=d.get("ceiling_height_m"),
            floor_area_m2=d.get("floor_area_m2"),
            wall_lengths_m=[float(v) for v in d.get("wall_lengths_m", [])],
            openings=[{"type": o.get("type", "door"), "width_m": float(o["width_m"])}
                      for o in d.get("openings", [])],
        )


def load(path: str | Path, fallback_room_id: str = "r1") -> GroundTruth:
    return GroundTruth.from_dict(json.loads(Path(path).read_text()), fallback_room_id)
