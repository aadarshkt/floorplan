# Measuring ground truth for a LiDAR capture

The benchmark compares the plan against what you measure with a laser (or tape).
Measure the way the plan reports, or the comparison is meaningless.

## 1. Run the capture and print the sheet

```bash
./.venv/bin/floorplan run benchmark/benchmark_2.r3d --out benchmark/runs/lidar/benchmark_2
./.venv/bin/floorplan gt-template benchmark/runs/lidar/benchmark_2 \
    --out benchmark/ground_truth/benchmark_2.json
open benchmark/runs/lidar/benchmark_2/floor_plan.svg      # labels: w0, w1, ...
```

Take the SVG (printed or on a phone) into the room. Every wall in it has an id
(`w3` = `r1-w3` in the JSON).

## 2. What to measure (laser distance meter, mm resolution)

| Item | How | Notes |
|---|---|---|
| **Wall length** (every wall in the plan, including short jogs) | Laser held against one corner, aimed at the opposite corner of the *same* wall, **~1.0 m above the floor**, along the wall face | Interior, face to face. Skirting boards and furniture in the way: measure above them, parallel to the wall. If a wall can't be reached, leave `null` |
| **Ceiling height** | Laser on the floor pointing straight up, at **3 spots** (centre + two corners ~0.5 m from walls) | Enter all three in `ceiling_height_m: [a, b, c]`; their spread is reported |
| **Door width** | Clear opening, **jamb to jamb** (inside of the frame), at ~1.0 m height | Not the door leaf. Add `wall_id` of the wall it is in |
| **Window width** | Inside of the reveal/frame, jamb to jamb, at mid height | Add `wall_id` |
| **Missing openings** | Any door/window the plan does *not* show: add an entry anyway | That is how missed openings get scored |
| Floor area | Leave `null` | Derived from the walls; measuring it by hand is less accurate |

Write metres with 3 decimals (`2.553`). Fill only what you measured; `null`
fields are not scored.

## 3. Fill the JSON

```json
{"capture": "benchmark_2", "instrument": "Bosch GLM 50 C", "measured_by": "Aadarsh", "date": "2026-10-04",
 "rooms": [{"room_id": "r1", "name": "study",
   "ceiling_height_m": [2.851, 2.848, 2.856],
   "walls": [{"wall_id": "r1-w0", "length_m": 2.553, "_plan_mid": [0.52, -2.09]}, ...],
   "openings": [{"type": "door", "width_m": 0.812, "wall_id": "r1-w6"}]}]}
```

Keep the `_plan_mid` values the template wrote: they pin each measurement to a
wall position, so it stays attached to the right wall even if a later code
version renumbers walls.

## 4. Score

```bash
./.venv/bin/floorplan bench --manifest benchmark/manifest.lidar.json \
    --out benchmark/reports/lidar --runs benchmark/runs/lidar
cat benchmark/reports/lidar/report.md
```

## For the assignment's benchmark set

- **Repeatability:** capture at least one room **twice** (`room_a_1.r3d`,
  `room_a_2.r3d`) and list both under `"repeatability"` in the manifest.
- **Multi-room:** one capture of 3+ rooms and a corridor; measure every room
  (`r1`, `r2`, ... as labelled in its plan).
- **Damage room:** one furnished room with staged damage of two classes; note
  each damaged area's size with the tape.
- Keep the raw `.r3d`, the filled JSON and a photo of each laser reading: the
  assignment asks for raw sensor data and measurements.
