# Next steps

Status (4 Oct): the LiDAR tier reads `.r3d` directly and produces a plan for all
5 test captures (1–6 rooms each, 8–22 s per capture on an M1). Nothing is scored
yet because there is no laser ground truth. Video and photo tiers are parked.

## Running a new `.r3d`

Export from Record3D (Library → capture → Export → *Shareable/Internal (.r3d)*),
copy the file into `benchmark/captures/` (git-ignored), then:

```bash
cd floorplan
./.venv/bin/floorplan run benchmark/captures/<name>.r3d --out benchmark/runs/lidar/<name>
#   optional: add --fusion tsdf to also write a surface mesh (scan_mesh.ply)
```

The first run decodes the `.r3d` into `/tmp/floorplan_cache/r3d_<hash>/`. Later
runs on the same file reuse that cache.

### Where the output goes: `benchmark/runs/lidar/<name>/`

| File | What to look at |
|---|---|
| `floor_plan.svg` | Plan with wall ids (`w0…`), lengths ±95 % CI, door/window widths, room area and ceiling. `open` it |
| `results.json` | Every number with its `ci95`, plus rooms, openings, adjacency and footprint |
| `scan_metric.ply` | Fused point cloud (metres, Z-up) |
| `scan_mesh.ply` | Mesh, only written with `--fusion tsdf` |
| `cameras.json` | Camera path used by the layout |
| `provenance.json` | Pose convention and why it was chosen, drift, per-stage timings |
| `floor_plan.dxf` | CAD version of the plan |

Inspect:

```bash
open benchmark/runs/lidar/<name>/floor_plan.svg
./.venv/bin/python scripts/layout_debug.py benchmark/runs/lidar/<name> benchmark/runs/lidar/<name>/layout_debug.png
open benchmark/runs/lidar/<name>/layout_debug.png   # grey points, black walls, blue cameras, red outlines
./.venv/bin/python scripts/view_ply.py benchmark/runs/lidar/<name> --top      # 3D viewer
```

Things to check: does every red outline sit on the black wall evidence? Are the
doors you walked through present? Is the ceiling `observed: true`? If not,
sweep the ceiling next time.

### Scoring it against laser measurements

```bash
./.venv/bin/floorplan gt-template benchmark/runs/lidar/<name> --out benchmark/ground_truth/<name>.json
#   fill the nulls with laser readings, following benchmark/MEASURE.md
```

Add an entry to `captures` in `benchmark/manifest.lidar.json` (paths are relative to the manifest):

```json
{"id": "<name>", "path": "captures/<name>.r3d", "tier": "lidar",
 "ground_truth": "ground_truth/<name>.json"}
```

Then score all captures at once:

```bash
./.venv/bin/floorplan bench --manifest benchmark/manifest.lidar.json \
    --out benchmark/reports/lidar --runs benchmark/runs/lidar
cat benchmark/reports/lidar/report.md
```

`bench` reuses an existing `results.json` in `--runs`. Add `--force` after code
changes. It skips captures that have no ground-truth file.

## Captures to record (what the assignment's gates need)

1. **Repeatability:** one room captured twice (`room_a_1.r3d`, `room_a_2.r3d`).
   Needed for the 1 cm / 0.5 % per-wall gate and the ceiling-spread gate.
2. **Multi-room:** 3 or more rooms plus a corridor in one capture. Needed for
   the stitch, adjacency and drift gates.
3. **Damage room:** a furnished room with staged damage of two classes. Measure
   each damaged area with a tape.
4. **Hard surfaces:** a mirror, glass, and a low-light room, for the failure-mode
   section.

When capturing, walk slowly, sweep the ceiling once, and look *through* each
doorway (door detection needs points beyond the wall). Keep the raw `.r3d` and a
photo of each laser reading.

## Code work, in priority order

1. **Calibrate on laser truth:** once 2–3 rooms are measured, fit
   `ci_sys_length_lidar` and the opening rules in `config.py` to the real
   errors. Then check that about 95 % of truths fall inside `ci95`.
2. **Corridors and multi-room:** the layout doesn't trace corridors between
   rooms (c7d28f72c6 footprint undercounts). Adjacency is incomplete.
3. **Missed doors:** benchmark_2's doorway is not detected (it may sit in a wall
   notch).
4. **Drift:** add a real pose-graph correction and the on/off footprint ablation
   (the `bench --ablate-drift` flag exists but has nothing to ablate yet). Right now drift is only reported.
5. **Damage detection + scope line items:** currently `[]` in `results.json`.
6. **Capture protocol page** (`CAPTURE_PROTOCOL.md`), head-to-head against a
   consumer app on 2 rooms, the fix-loop write-up, and the technical report.
   See `COMPLIANCE.md`.
7. **Video / photo tiers:** resume after the LiDAR tier is scored. They are
   benchmarked against LiDAR with `floorplan xbench`.
