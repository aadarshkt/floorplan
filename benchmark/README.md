# Benchmark

Everything for scoring lives under this one folder: raw captures, ground truth,
pipeline runs, and reports.

## Layout

```
benchmark/
  manifest.json            all three tiers at once (hybrid) + gate thresholds
  manifest.photos.json     photos tier only
  manifest.video.json      video tier only
  CAPTURE.md               how to capture (Record3D) and unpack a .r3d
  unpack_r3d.py            .r3d -> dataset folder
  captures/<id>/           raw input (Record3D export folder, or a .zip)
  ground_truth/<id>.json   your laser/tape measurements   <-- EDIT THIS
  runs/<id>/               pipeline output per capture (results.json, svg, dxf, ply, provenance)
  reports/<tag>/           scores: gates.json + report.md
```

## The single accuracy number

`report.md` leads with:

```
Accuracy: NN%   (within/total measurements within tolerance)
```

That is **the** accuracy number. Across every scored quantity — each wall length,
the ceiling height, each opening width, and the floor area — it is the share that
lands inside its gate tolerance. 100% means every measurement is within spec.
The second line, `Overall: X/Y captures passed`, is the stricter per-capture
"all gates at once" view.

## Run it

```bash
# score the captures: writes runs/<id>/ and benchmark/{gates.json,report.md}
./.venv/bin/floorplan bench --manifest benchmark/manifest.json --out benchmark

# force a full pipeline re-run (ignore cached results.json)
./.venv/bin/floorplan bench --manifest benchmark/manifest.json --out benchmark --force

# fix loop: shared runs/, before and after a code fix
./.venv/bin/floorplan bench --manifest benchmark/manifest.json \
    --out benchmark/reports/before --runs benchmark/runs
#   ...ship a fix (a real commit)...
./.venv/bin/floorplan bench --manifest benchmark/manifest.json \
    --out benchmark/reports/after  --runs benchmark/runs --force
```

`--runs` (default `<out>/runs`) is where per-capture `results.json` live; they are
reused unless `--force`. `--ablate-drift` adds the drift on/off comparison.

## The three configurations

Every tier runs **independently** — none borrows another capture's data.

- **photos** — a folder of stills, "no depth, no poses".
- **video** — a handheld walkthrough clip.
- **hybrid** — the `.r3d` bundle, which carries all three streams (LiDAR depth +
  odometry, the RGB clip, and stills).

Photos and video run **metric monocular depth fused through COLMAP SfM poses**:
COLMAP recovers the camera poses and intrinsics from the images, and a *metric*
depth model (Depth Anything V2 Metric, Indoor) supplies absolute metres per
pixel. Both scale and poses therefore come from the images alone — no LiDAR
capture and no `--scale-ref` are needed. Provenance shows
`path_chosen: metric_depth:sfm_fused` and `scale_reference: metric_model`.

Install the model once (torch + transformers; a GPU is much faster but not
required):

```bash
scripts/fetch_weights.sh
```

```bash
# 1. photos only
./.venv/bin/floorplan bench --manifest benchmark/manifest.photos.json \
    --out benchmark/reports/photos --force

# 2. video only
./.venv/bin/floorplan bench --manifest benchmark/manifest.video.json \
    --out benchmark/reports/video --force

# 3. hybrid — all three streams from the one .r3d bundle
./.venv/bin/floorplan bench --manifest benchmark/manifest.json \
    --out benchmark/reports/hybrid --force
```

Or one capture at a time (`run` produces artifacts, `bench` scores them):

```bash
D=benchmark/captures/2026-10-03--00-39-56
./.venv/bin/floorplan run $D --tier lidar  --out benchmark/runs/lidar     # depth + odometry
./.venv/bin/floorplan run $D/rgb.mp4 --tier video --out benchmark/runs/video
./.venv/bin/floorplan run benchmark/captures/photos_2026-10-03--00-39-56 \
    --tier photos --out benchmark/runs/photos
```

Notes:

- `--engine auto` (default) is the metric-depth path above. `--engine colmap`
  swaps in COLMAP's dense MVS instead (provenance shows `colmap:mvs` /
  `colmap:cpu_mvs`); `--engine monodepth` forces the metric model or errors.
- `--reference-capture <r3d>` remains the cross-tier fallback when a LiDAR twin
  exists. The independent configs above do **not** use it — that is the whole
  point.
- Gate tolerances differ by tier in the assignment (photos ±8%, video ±3%). The
  defaults in `manifest.json → gates` are tighter absolute-cm values; override
  per manifest if you want the tier-specific numbers, e.g.
  `"gates": {"area_rel_pct": 8.0}`.

## Analyze a single capture

`floorplan run` produces the artifacts but does **not** score them. To see the
accuracy of one capture on its own, score it with a one-entry manifest — then
nothing is pooled with the other tiers.

```bash
# 1. one-entry manifest (tier can be lidar | photos | video)
cat > benchmark/manifest.lidar.json <<'JSON'
{
  "units": "meters",
  "captures": [
    { "id": "lidar", "path": "captures/2026-10-03--00-39-56", "tier": "lidar",
      "room_id": "room1", "ground_truth": "ground_truth/2026-10-03--00-39-56.json" }
  ]
}
JSON

# 2. score that one capture
./.venv/bin/floorplan bench --manifest benchmark/manifest.lidar.json \
    --out benchmark/reports/lidar-only --force

# 3. read it
cat benchmark/reports/lidar-only/report.md      # Accuracy line + gate table
cat benchmark/reports/lidar-only/gates.json     # same, machine-readable
cat benchmark/reports/lidar-only/runs/lidar/results.json   # raw measurements + ci95
```

Swap `path`/`tier` for the other tiers:

| tier | `path` | extra flags |
|---|---|---|
| photos | `captures/photos_2026-10-03--00-39-56` | `--reference-capture benchmark/captures/2026-10-03--00-39-56` |
| video | `captures/2026-10-03--00-39-56/rgb.mp4` | `--reference-capture benchmark/captures/2026-10-03--00-39-56` |

For artifacts only (no scoring): `./.venv/bin/floorplan run <capture> --out <dir>`.

## Ground-truth format (measure these with the laser/tape)

```json
{
  "room_id": "room1",
  "ceiling_height_m": 2.42,          // floor -> ceiling
  "floor_area_m2": 13.1,             // optional
  "wall_lengths_m": [4.20, 3.15],    // ONE number per wall, order-free
  "openings": [
    {"type": "door",   "width_m": 0.82},
    {"type": "window", "width_m": 1.20}
  ]
}
```

Wall lengths and openings are **unordered sets** — the harness matches produced
elements to measured ones with an optimal (Hungarian) assignment, so it never
penalises ordering. See `ground_truth/TEMPLATE.json`.

## Gate reference

| Gate | Produced by | Ground truth |
|---|---|---|
| `wall_length_*` | `rooms[].walls[].length_m.value` | `wall_lengths_m` |
| `ceiling_height_abs_cm` | `rooms[].ceiling_height_m.value` | `ceiling_height_m` |
| `opening_*` | `rooms[].walls[].openings[]` | `openings` |
| `area_rel_pct` | `rooms[].floor_area_m2.value` | `floor_area_m2` |
| `repeatability_*` | two runs of the same room | (none — self-comparison) |
| `ci_coverage_min` | all `ci95` intervals | whether the true value falls inside |
