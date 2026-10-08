# floorplan — phone capture → dimensioned floor plan

Turn a handheld phone capture into a **metric, dimensioned floor plan** with
confidence intervals: `results.json` (published schema), `floor_plan.svg`,
`floor_plan.dxf`, `scan_metric.ply`, `provenance.json`.

Local, deterministic, offline CLI. No server, no cloud.

## Quickstart: from clone to results

Tested on macOS (Apple Silicon). Needs Homebrew, `uv` and Python 3.11. No GPU, no cloud, no account.

**1. Install** (download time dominates)

```bash
git clone https://github.com/aadarshkt/floorplan && cd floorplan
brew install ffmpeg colmap libusb
uv venv --python 3.11 .venv && uv pip install -e .
```

**2. Get a capture.** Either record your own with *Stray Scanner* (see
[`CAPTURE_PROTOCOL.md`](CAPTURE_PROTOCOL.md): iPhone Pro, free app, zip the recording), or use ours:
download `raw_benchmark_data/` from the Drive link in the submission email, then

```bash
for c in kitchen_room_scan kitchen_scan_2 bedroom_2 study_room_friend; do
  unzip raw_benchmark_data/captures/$c.zip -d benchmark/captures/$c
done
```

**3. Run one command per capture**

```bash
./.venv/bin/floorplan run benchmark/captures/study_room_friend --out out/study     # a folder
./.venv/bin/floorplan run my_stray_scanner_capture.zip --out out/mine              # or a .zip
```

About 20 s on an M1. The last lines should read like (study room, tape: 9.98 m2, ceiling 2.836 m):

```
[drift] 120 keyframes, 80 candidates, 10 ICP-verified loops
[geometry] planes=40 walls=8 rooms=1 openings=1 footprint=9.747 m2 ceiling=2.845 m observed=True
[done] out/study  (total ~18s)
```

**4. Look at the result**

```bash
open out/study/floor_plan.svg          # the plan: wall lengths with 95% intervals, openings, area, ceiling
./.venv/bin/python - <<'EOF'
import json
r = json.load(open("out/study/results.json"))["rooms"][0]
print("ceiling", r["ceiling_height_m"]["value"], r["ceiling_height_m"]["ci95"])
print("area   ", r["floor_area_m2"]["value"], r["floor_area_m2"]["ci95"])
for w in r["walls"]:
    print(w["wall_id"], w["length_m"]["value"], w["length_m"]["ci95"], "openings:", len(w["openings"]))
EOF
```

**Skip the run:** precomputed outputs for 9 captures (our 4 tape-measured rooms, the 3 assignment samples as `assignment_1..3`, and 2 more) are on Drive as `stray.zip`. Unzip into `benchmark/runs/`, and each `benchmark/runs/stray/<capture>/` has the files above plus `layout_debug.png`, a top-down check of the layout (legend in [`BENCHMARK_REPORT.md`](BENCHMARK_REPORT.md) §1b). Added 2026-10-09; see [`SUBMISSION.md`](SUBMISSION.md).

Compare those numbers with your own laser or tape. [`TESTING.md`](TESTING.md) is the one-page
scoring guide (which field to compare, what counts as a pass).

**5. Reproduce the reported benchmark** (needs the unzipped captures from step 2; about 2 minutes)

```bash
./.venv/bin/python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/stray --runs benchmark/runs/stray --force
cat benchmark/reports/stray/report.md      # expect: Accuracy 26% (6/23), 0/4 captures pass all gates
```

Reproduce the **fix loop** (worst gate before and after, wall-length error 161.4 cm to 5.6 cm on `study_room_friend`):

```bash
git checkout fixloop-before   # code before the fix
./.venv/bin/python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/fixloop_before --runs benchmark/runs/fixloop_before --force
git checkout main             # code after the fix
./.venv/bin/python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/fixloop_after --runs benchmark/runs/fixloop_after --force
```

Run `python -m floorplan.cli` (not the installed `floorplan` script) when switching checkouts. Story and
post-mortem: [`fixloop/RESULTS.md`](fixloop/RESULTS.md).

Drift correction on vs off on the multi-room assignment sample:

```bash
./.venv/bin/python -m floorplan.cli drift-ablate <path>/Assignment/c7d28f72c6 --out out/ablation
open out/ablation/drift_overlay.svg
```

Tests: `./.venv/bin/python -m pytest -q` (32 pass).

**Video tier (optional, slower, needs a known ceiling height for scale)**

```bash
scripts/fetch_weights.sh      # torch + transformers; the depth model (~100 MB) downloads on first use
./.venv/bin/floorplan run walk.mov --out out/video --scale-ref 2.60 --scale-ref-kind ceiling_height
```

Add `--max-frames` or `--fps 1` for a quick smoke test. Video is **not** accurate yet (see below).

### What works, honestly

| Tier | State |
|---|---|
| LiDAR (Stray Scanner zip) | Runs end to end. Best wall error 5.6 cm; no capture passes all gates; doors and windows mostly missed |
| Video | Runs, but 0 of 4 rooms usable (wall errors 1 to 2 m) |
| Photos | Not delivered as a tier |
| Damage, scope, head-to-head | Not built |

Numbers and gates: [`BENCHMARK_REPORT.md`](BENCHMARK_REPORT.md). Requirement-by-requirement status:
[`COMPLIANCE.md`](COMPLIANCE.md). Architecture and fix-loop story: [`REPORT.md`](REPORT.md). Everything in order:
[`SUBMISSION.md`](SUBMISSION.md).

---

## Reference (older detail)

## Install (clean machine)

```bash
# macOS
brew install ffmpeg colmap     # video frames; photos/video reconstruction (SfM)
brew install libusb            # required by open3d on macOS
uv venv --python 3.11 .venv    # or: python3.11 -m venv .venv
uv pip install -e .            # or: ./.venv/bin/pip install -e .   (includes lzfse)

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
| Record3D `.r3d` file, or an export folder / `.zip` with `odometry.csv` | **lidar** | depth + pose fusion (native metres) |
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

## LiDAR tier — run, inspect, benchmark

New capture? See [`NEXT_STEPS.md`](NEXT_STEPS.md) for the full checklist: run it, where the output goes, scoring, and what's next.

```bash
# 1. run a Record3D capture (.r3d straight from the app's "Shareable/Internal" export)
./.venv/bin/floorplan run benchmark/benchmark_2.r3d --out benchmark/runs/lidar/benchmark_2

# 2. look at it
open benchmark/runs/lidar/benchmark_2/floor_plan.svg       # plan: wall ids (w0..), lengths ±95% CI,
                                                           # doors/windows with widths, area, ceiling
python -m json.tool benchmark/runs/lidar/benchmark_2/results.json | less   # every number + ci95
open -a "Preview" benchmark/runs/lidar/benchmark_2/scan_metric.ply        # or MeshLab / CloudCompare
grep -E '"path_chosen"|"total_s"' benchmark/runs/lidar/benchmark_2/provenance.json
./.venv/bin/python scripts/layout_debug.py benchmark/runs/lidar/benchmark_2 /tmp/layout.png && open /tmp/layout.png
#   grey = all points, black = wall evidence, blue = camera path, red = room outline

# 3. measure the room (benchmark/MEASURE.md), then score
./.venv/bin/floorplan gt-template benchmark/runs/lidar/benchmark_2 --out benchmark/ground_truth/benchmark_2.json
#   ...fill the nulls with laser readings...
./.venv/bin/floorplan bench --manifest benchmark/manifest.lidar.json \
    --out benchmark/reports/lidar --runs benchmark/runs/lidar
cat benchmark/reports/lidar/report.md      # gates, missed/phantom walls+openings, per-wall errors
```

What the numbers mean:

- wall lengths are **interior, face to face** (what a laser measures), from one
  room outline, so walls, polygon and area always agree;
- `ci95` combines how sharply each wall surface is defined in the scan with a
  per-tier systematic floor (`config.py: ci_sys_length_*`);
- an opening is reported only with evidence: a door must be *seen through*
  (scan points beyond the wall inside the gap), so unscanned wall patches do
  not become phantom doors;
- `ceiling_height_m.observed=false` means the ceiling was never scanned and the
  value is a prior with a wide interval — sweep the ceiling when capturing.

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
./.venv/bin/floorplan bench --manifest benchmark/manifest.json       --out benchmark/selftest --force   # harness self-test only (GT = pipeline output)
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
# score every capture in a manifest against tape/laser ground truth
./.venv/bin/python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/stray --runs benchmark/runs/stray --force
cat benchmark/reports/stray/report.md      # per-capture gate table + the single accuracy number
```

`benchmark/selftest/` is a harness self-test whose ground truth was copied from the pipeline's own
output; its 100 % is not an accuracy result. See `benchmark/README.md` for the full walkthrough.
`IMPLEMENTATION_PLAN.md` is the original design.

## Tests

```bash
./.venv/bin/python -m pytest -q
```
