# floorplan — phone capture → dimensioned floor plan

Turn a handheld phone capture into a **metric, dimensioned floor plan** with
confidence intervals: `results.json` (published schema), `floor_plan.svg`,
`floor_plan.dxf`, `scan_metric.ply`, `provenance.json`.

This is a local, deterministic, offline command-line tool. No server, no cloud.

## Install (clean machine)

```bash
# macOS
brew install ffmpeg            # frame extraction (video tier)
brew install libusb            # required by open3d on macOS
uv venv --python 3.11 .venv    # or: python3.11 -m venv .venv
uv pip install -e .            # or: ./.venv/bin/pip install -e .
```

## One command per capture

```bash
./.venv/bin/floorplan run <capture_path> --out <out_dir>
```

The tier is auto-detected:

| Input | Tier | Status |
|---|---|---|
| Record3D export (folder or `.zip` with `odometry.csv`) | LiDAR | ✅ implemented |
| Folder of 2–8 photos | Photos | 🔜 P2 |
| Single `.mp4` / `.mov` walkthrough | Video | 🔜 P2 |

### LiDAR example

```bash
./.venv/bin/floorplan run /path/to/record3d-export.zip --out runs/room1
```

Record3D export layout (the LiDAR tier): `rgb.mp4`, `depth/*.png` (256×192, mm),
`confidence/*.png`, `odometry.csv` (poses + intrinsics), `imu.csv`,
`camera_matrix.csv`.

## Outputs

| File | Meaning |
|---|---|
| `results.json` | the output contract (see `schema/floorplan.schema.json`) |
| `floor_plan.svg` | rendered plan (opens in any browser) |
| `floor_plan.dxf` | dimensioned CAD |
| `scan_metric.ply` | fused point cloud (metres, Z-up) |
| `provenance.json` | how the run was produced + per-stage timings |

Every measurement is `{value, ci95, method}`. `results.json` is **deterministic**
(no wall-clock), so re-runs are byte-identical.

## Pipeline

```
Record3D → depth+pose fusion (Z-up metres) → RANSAC planes → walls
        → openings → room polygon/area/ceiling → JSON + SVG + DXF
```

## Tests

```bash
./.venv/bin/python -m pytest -q
```

See `IMPLEMENTATION_PLAN.md` for the full design, tier roadmap, benchmark plan,
and the fix-loop mechanics.
