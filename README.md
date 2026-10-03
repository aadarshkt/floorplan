# floorplan — phone capture → dimensioned floor plan

Turn a handheld phone capture into a **metric, dimensioned floor plan** with
confidence intervals: `results.json` (published schema), `floor_plan.svg`,
`floor_plan.dxf`, `scan_metric.ply`, `provenance.json`.

Local, deterministic, offline CLI. No server, no cloud.

## Install (clean machine)

```bash
# macOS
brew install ffmpeg colmap     # video frames; photos/video reconstruction (SfM)
brew install libusb            # required by open3d on macOS
uv venv --python 3.11 .venv    # or: python3.11 -m venv .venv
uv pip install -e .            # or: ./.venv/bin/pip install -e .

# Optional learned monocular-depth backend for the photos/video tiers:
scripts/fetch_weights.sh
```

## One command per capture

```bash
./.venv/bin/floorplan run <capture_path> --out <out_dir>
```

The tier is auto-detected:

| Input | Tier | Geometry source |
|---|---|---|
| Record3D export (folder or `.zip` with `odometry.csv`) | **lidar** | depth + pose fusion (native metres) |
| Folder of photos | **photos** | COLMAP SfM (+ CPU plane-sweep MVS), or monocular depth |
| Single `.mp4` / `.mov` walkthrough | **video** | ffmpeg keyframes → same as photos |

All three converge on one intermediate representation (metric Z-up cloud +
provenance) and share the **exact same** geometry/confidence/export stages — that
is what makes them comparable and keeps `results.json` identical in shape.

### Examples

```bash
# LiDAR
./.venv/bin/floorplan run /path/to/record3d.zip --out benchmark/runs/lidar

# Video (COLMAP), anchored to a paired LiDAR capture of the same room
./.venv/bin/floorplan run walkthrough.mov --out benchmark/runs/video \
    --engine colmap --reference-capture /path/to/record3d.zip

# Photos, scaled by a known ceiling height (no LiDAR needed)
./.venv/bin/floorplan run ./photos --out benchmark/runs/photos \
    --engine colmap --scale-ref 2.42 --scale-ref-kind ceiling_height
```

## Metric scale (photos / video)

A photo/video reconstruction is metric only up to an unknown scale. It is
resolved, in order of preference, and always recorded in `results.json → scale`
and `provenance.json`:

1. `--reference-capture <record3d>` — a paired metric capture of the same room;
   similarity ICP recovers scale+pose. `scale_reference = "scaled_via_reference"`.
2. `--scale-ref <m> --scale-ref-kind ceiling_height` — a known real length.
   `scale_reference = "scaled_via_reference"`.
3. nothing — a default prior is used, intervals are widened, and the plan is
   flagged **non-metric**. `scale_reference = "none_prior"`, `scale.metric=false`.

## Outputs

| File | Meaning |
|---|---|
| `results.json` | the output contract (see `schema/floorplan.schema.json`) |
| `floor_plan.svg` | rendered plan (opens in any browser) |
| `floor_plan.dxf` | dimensioned CAD |
| `scan_metric.ply` | fused point cloud (metres, Z-up) |
| `provenance.json` | how the run was produced, which path won, per-stage timings |

Every measurement is `{value, ci95, method}`. `results.json` is **deterministic**
(no wall-clock), so re-runs are byte-identical.

## Pipeline

```
Record3D ─┐
photos   ─┼─▶ INGEST (tier autodetect) ─▶ RECONSTRUCT ─▶ metric Z-up cloud (IR)
video    ─┘                                   ├ lidar : depth+pose fusion
                                              ├ photos: COLMAP SfM + CPU MVS
                                              └        (or monocular depth)
                                                        │  scale anchor
                                                        ▼
        RANSAC planes → walls → openings → room polygon/area/ceiling → JSON+SVG+DXF
```

## Test & analyse

```bash
# 1. score every capture in the manifest
./.venv/bin/floorplan bench --manifest benchmark/manifest.json --out benchmark

# 2. read the result
cat benchmark/report.md      # per-capture gate table + the single accuracy number
cat benchmark/gates.json     # the same, machine-readable
```

`report.md` leads with the one number that matters:

```
Accuracy: NN%   (within/total measurements within tolerance)
```

See `benchmark/README.md` for the full walkthrough. `IMPLEMENTATION_PLAN.md` has
the design, benchmark plan and fix-loop mechanics.

## Tests

```bash
./.venv/bin/python -m pytest -q
```
