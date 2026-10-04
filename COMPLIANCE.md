# Compliance matrix

Requirement → file → artifact → status. **done** = works and is evidenced; **partial** = works with stated limits or fails its gate; **not done** = not built (a conscious scope cut, not an oversight).
Numbers: `BENCHMARK_REPORT.md`. Explanation: `REPORT.md`.

## Part 1: capture route and tiers

| Requirement | File | Artifact | Status |
|---|---|---|---|
| Capture route: stock app + one-page protocol | `CAPTURE_PROTOCOL.md` (Stray Scanner) | protocol page | done (not yet tried by a non-engineer) |
| Device matrix | `CAPTURE_PROTOCOL.md`, `REPORT.md` §2 | table | done |
| LiDAR tier | `ingest/r3d.py`, `ingest/record3d.py`, `fusion/depth_fusion.py` | `floorplan run x.zip` | partial: runs in about 20 s; gates mostly fail |
| Video tier | `ingest/video.py`, `fusion/sfm_depth.py`, `fusion/metric_depth.py` | `floorplan run x.mp4 --scale-ref` | partial: runs; 0 of 4 rooms usable; needs a known ceiling height |
| Photos tier (2 to 8 stills per room) | `ingest/photos.py` | `floorplan run <folder>` | **not done**: SfM registers 0 of 8 stills |
| Per-room photo folders stitched into one plan | none | none | **not done** |
| Mirrors, glass, wet surfaces, low light | `REPORT.md` §7 | stated as risks | **not done**: no test captures, no handling |

## Part 2: output contract

| Requirement | File | Artifact | Status |
|---|---|---|---|
| Per-room plan: walls, ceiling, area, openings | `geometry/layout.py`, `room.py`, `openings.py` | `results.json`, `floor_plan.svg` | partial: openings mostly missed |
| Stitched multi-room plan + adjacency | `geometry/layout.py`, `multiroom.py` | `results.json property.adjacency` | partial: one capture only; no tape truth |
| Damage regions | none | `damage_regions: []` | **not done** (out of scope by decision) |
| Concealed-damage flags + rule | none | `concealed_damage_flags: []` | **not done** |
| Scope line items | none | `scope_line_items: []` | **not done** |
| CI on every measurement | `export/json_export.py` | `ci95` | partial: coverage 40 % vs 85 % |
| One command per capture | `floorplan run <input> --out <dir>` | CLI | done |
| JSON to published schema | `schema/floorplan.schema.json` | `results.json` | done (damage arrays empty) |
| Rendered plan | `export/svg.py`, `export/dxf.py` | SVG, DXF | done |

## Part 2: benchmark set and gates

| Requirement | Evidence | Status |
|---|---|---|
| Multi-room capture, 3+ rooms + connector, tape truth | assignment samples only, no truth | **not done** |
| Furnished room with staged damage (2 classes) | none | **not done** |
| Same rooms at all three tiers | 4 single rooms at LiDAR + video | partial |
| Room captured twice, same tier | kitchen x2, LiDAR | done |
| Tape/laser truth + raw data | `benchmark/ground_truth/`, captures (see `REPRODUCE.md`) | partial: 4 rooms, tape not laser |
| Opening widths ≤2 cm on ≥85 % | `BENCHMARK_REPORT.md` §1 | **fail** (0 %) |
| Ceiling ≤1.5 cm; spread ≤1 cm; biased or unrepeatable stated | §1, §2 | **fail** (2 of 4 rooms; spread 2.4 cm; unrepeatable) |
| Repeatability 1 cm / 0.5 % | §2 | **fail** (25.2 cm) |
| Drift accountability + ablation | `drift.py`, `benchmark/ablation/c7d28f72c6/` | done as mechanism and ablation; accuracy gain unproven (no truth) |
| Photo-tier whole-property stitch ±8 % | §5 | **fail** |
| Video ±3 %, photo ±8 %, calibrated | §4 | **fail** |

## Parts 3 to 5

| Requirement | File | Status |
|---|---|---|
| Head-to-head vs consumer app on 2 rooms | none | **not done** (no export exists) |
| Fix loop declaration, before/after regenerable, diff | `fixloop/` (tag `fixloop-before`) | done: gate moved (161 to 5.6 cm worst wall) but capture-level gate not passed; prediction badly wrong, post-mortem written |
| Process evidence | git history (over 35 commits on main, 3 to 4 Oct) | done |

## Deliverables

| # | Deliverable | File | Status |
|---|---|---|---|
| 1 | Compliance matrix | `COMPLIANCE.md` | done |
| 2 | Capture route + device matrix | `CAPTURE_PROTOCOL.md` | done |
| 3 | Repo, README to running <15 min, one command | `README.md`, `TESTING.md` | done: native install (macOS), LiDAR tier runs in about 20 s; not timed on a second clean machine |
| 4 | Reproduction bundle | `REPRODUCE.md`, `benchmark/manifest*.json` | partial: captures are not in git (about 2 GB) |
| 5 | Benchmark report | `BENCHMARK_REPORT.md` | done |
| 6 | Fix loop bundle | `fixloop/` | done |
| 7 | Technical report ≤6 pages | `REPORT.md`, `REPORT.pdf` | done |
| 8 | Raw benchmark data | `benchmark/ground_truth/` + captures | partial: see `REPRODUCE.md` |
