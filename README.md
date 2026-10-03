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
| Folder of photos | **photos** | monocular depth: a learned model if installed, else the paired reference capture (COLMAP with `--engine colmap`) |
| Single `.mp4` / `.mov` walkthrough | **video** | ffmpeg keyframes → same as photos |

All three converge on one intermediate representation (metric Z-up cloud +
provenance) and share the **exact same** geometry/confidence/export stages — that
is what makes them comparable and keeps `results.json` identical in shape.

> COLMAP is **opt-in** (`--engine colmap`). On machines without CUDA its dense MVS
> is unavailable and the CPU path is coarse, so it is not the default. The
> monocular path uses a learned model when one is installed
> (`scripts/fetch_weights.sh`), otherwise the paired reference capture below.

### Examples

```bash
# LiDAR
./.venv/bin/floorplan run /path/to/record3d.zip --out benchmark/runs/lidar

# Photos/video, using a paired Record3D capture of the same room as the metric
# reference (monocular path, the default)
./.venv/bin/floorplan run ./photos --out benchmark/runs/photos \
    --reference-capture /path/to/record3d.zip
./.venv/bin/floorplan run walkthrough.mov --out benchmark/runs/video \
    --reference-capture /path/to/record3d.zip

# Optional: pure COLMAP reconstruction (slower; needs a good photo set)
./.venv/bin/floorplan run ./photos --out benchmark/runs/photos_colmap --engine colmap

# Scale a lone photo set from a known ceiling height (no reference capture)
./.venv/bin/floorplan run ./photos --out benchmark/runs/photos \
    --scale-ref 2.42 --scale-ref-kind ceiling_height
```

## Photos / video tier — exact commands (CPU or Apple Silicon, no NVIDIA)

`ffmpeg` (video frames) and `colmap` (SfM) are required; the learned
metric-depth backend is optional and only makes the tier metric without any
external reference:

```bash
brew install ffmpeg colmap libusb            # tools (libusb is for open3d)
uv venv --python 3.11 .venv && uv pip install -e .
scripts/fetch_weights.sh                     # optional: torch + transformers (MPS on Apple Silicon)
```

Run one capture (tier is auto-detected; `--fps` = video keyframes/second):

```bash
# video walkthrough
./.venv/bin/floorplan run benchmark/captures/room_rgb.mp4                  --out benchmark/runs/video_room
./.venv/bin/floorplan run benchmark/captures/2026-10-03--00-39-56/rgb.mp4  --out benchmark/runs/video

# photo folder
./.venv/bin/floorplan run benchmark/captures/photos_2026-10-03--00-39-56   --out benchmark/runs/photos

# LiDAR (Record3D export: folder or .zip containing odometry.csv)
./.venv/bin/floorplan run benchmark/captures/2026-10-03--00-39-56          --out benchmark/runs/lidar
```

Key options:

```bash
--device auto|mps|cpu|cuda   # monocular-depth device (auto: cuda > mps > cpu)
--engine auto|monodepth|colmap
                             #  auto (default): learned metric depth, else CPU COLMAP
                             #  colmap: pure-CPU SfM (+ CPU plane-sweep MVS), no torch
                             #  monodepth: force the learned metric-depth model
--fps 2                      # video keyframe rate
--scale-ref 2.42 --scale-ref-kind ceiling_height   # anchor a reference-free set
--reference-capture <record3d.zip>                 # paired metric capture of the same room
```

On a machine **without `torch`**, `--engine auto` transparently falls back to the
CPU COLMAP path — the tier still runs, no GPU required.

Score / compare:

```bash
./.venv/bin/floorplan bench --manifest benchmark/manifest.video.json  --out benchmark/reports/video
./.venv/bin/floorplan bench --manifest benchmark/manifest.photos.json --out benchmark/reports/photos
./.venv/bin/floorplan bench --manifest benchmark/manifest.json       --out benchmark --force   # LiDAR
```

What a successful run prints, and where to look:

```bash
# stderr, e.g.:
#   [ingest] video: 126 images from room_rgb
#   [metric] depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf on mps
#   [arbiter] chose metric_depth:sfm_fused
#   [geometry] planes=.. walls=.. rooms=1 openings=.. footprint=.. m2 ceiling=.. m observed=True

grep path_chosen benchmark/runs/video_room/provenance.json          # which code path won
python -c "import json;print(json.load(open('benchmark/runs/video_room/results.json'))['scale'])"
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
