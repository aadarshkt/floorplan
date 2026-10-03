# Compliance matrix

Requirement → where it lives → artifact → status. Status is honest:
**done** (works and is evidenced), **partial** (works with stated limits),
**todo** (not built). Updated as work lands.

## Part 1 — capture route and tiers

| Requirement | File / command | Artifact | Status |
|---|---|---|---|
| Capture route: stock app + one-page protocol | `CAPTURE_PROTOCOL.md` (Record3D) | protocol page | todo |
| Device matrix (tier → hardware → honest accuracy) | technical report §2 | table | todo |
| LiDAR tier: depth + poses + intrinsics | `floorplan run <x>.r3d` · `floorplan/ingest/r3d.py`, `fusion/depth_fusion.py` | results.json, floor_plan.svg | partial: single rooms solid, multi-room layout rough |
| Video tier | `floorplan run <clip>.mp4` · `ingest/video.py`, `fusion/sfm_depth.py` | same contract | partial: works on slow captures (scale ±4–7 %), fragments on fast sweeps |
| Photos tier, 2–8 stills/room | `floorplan run <folder>` | same contract | partial: single-image fallback only; SfM on sparse stills fails |
| Photo folders per room → one stitched plan | — | — | todo |

## Part 2 — output contract per capture

| Requirement | File | Artifact | Status |
|---|---|---|---|
| Per-room plan: walls, ceiling, floor area, openings | `geometry/layout.py`, `geometry/room.py`, `geometry/openings.py` | results.json, floor_plan.svg | partial: openings need evidence-based tuning on real rooms |
| Stitched multi-room plan + adjacency | `geometry/layout.py` (one capture), `geometry/multiroom.py` | results.json `property.adjacency` | partial: adjacency only via shared walls |
| Damage regions (class + metric extent) | — | `damage_regions: []` | todo |
| Concealed-damage flags + rule fired | — | `concealed_damage_flags: []` | todo |
| Scope line items keyed to surfaces | — | `scope_line_items: []` | todo |
| Confidence interval on every measurement | `export/json_export.py` (`widen`), `geometry/layout.py` (edge spread) | `ci95` on every value | partial: needs calibration against laser truth |
| One command per capture | `floorplan run <input> --out <dir>` | — | done |
| JSON to published schema | `schema/floorplan.schema.json` | results.json | partial: schema must be re-checked against new fields |
| Rendered plan | `export/svg.py`, `export/dxf.py` | floor_plan.svg / .dxf | done |

## Part 2 — gates (benchmark)

| Gate | How it is measured | Status |
|---|---|---|
| Opening widths ≤2 cm on ≥85 %, missed + phantom count | `floorplan bench` (`eval/metrics.py` opening_errors) | scoring done; accuracy unmeasured (needs laser truth) |
| Ceiling ≤1.5 cm, repeat spread ≤1 cm | `bench` + repeatability block | scoring done; needs a room captured twice |
| Repeatability 1 cm / 0.5 % per wall | `bench` repeatability | todo: capture a room twice |
| Drift accountability + on/off ablation | `drift.py`, `bench --ablate-drift` | todo: real pose-graph correction + footprint ablation |
| Photo-tier whole-property stitch | — | todo |
| Video ±3 % / photos ±8 % with calibrated intervals | `floorplan xbench` (vs LiDAR), `bench` (vs laser) | partial |

## Parts 3–5 and deliverables

| Requirement | File | Status |
|---|---|---|
| Head-to-head vs consumer app on 2 rooms (beat/tie ≥70 %) | `benchmark/head_to_head/` + table in report | todo |
| Fix loop: worst gate, root cause, prediction, before/after regenerable | `fixloop/` (declaration, before/after reports, diff) | todo |
| Process evidence (incremental commits) | git history | ongoing |
| README: clean machine → running in <15 min | `README.md` | partial: update for .r3d, lzfse |
| Reproduction bundle (regenerate every number) | `benchmark/manifest*.json`, `floorplan bench`/`xbench` | partial |
| Benchmark report (3 tiers, repeatability, head-to-head, timing) | `benchmark/reports/` | partial |
| Technical report ≤6 pages | `REPORT.md` | todo |
| Raw benchmark data (sensor logs, ground truth, app exports) | `benchmark/` (+ external storage for .r3d) | partial: ground truth not yet measured |
| Mirrors, glass, wet surfaces, low light covered | report §failure modes + protocol | todo |
| Weights fetched by script | `scripts/fetch_weights.sh` | done |
