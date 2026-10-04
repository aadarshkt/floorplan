# Technical report: phone capture to dimensioned floor plan

Scope statement first. This submission delivers a **working LiDAR tier** from Stray Scanner, a **video tier that runs but does not meet its gate**, and **no real photo tier**. Damage regions, concealed-damage flags, scope line items and the head-to-head against a consumer app are **not built**. Every number below is from tape-measured rooms (4 rooms, one author) and comes from files in this repo.

## What works

- The LiDAR tier runs end to end from a Stray Scanner zip in about 20 s on an M1 laptop, offline, with one command, and is deterministic (byte-identical `results.json` on rerun).
- On the cleanest room (study_room_friend, against tape): worst wall 5.6 cm, footprint 9.69 m² vs 9.98 m² (2.9 % under), ceiling 0.9 cm. Ceiling height is within the 1.5 cm gate on 2 of 4 rooms.
- Pose-graph drift correction cuts the loop-closure error on the multi-room sample from 25.5 cm to 2.1 cm (section 3).
- The fix loop moved the worst gate from 161.4 cm to 5.6 cm worst-wall error, with a declaration committed first and before/after runs that can be regenerated (section 6).
- These do not add up to a passing benchmark: 0 of 4 rooms pass every gate, and 26 % of measurements are within tolerance. The sections below say where and why.

## 1. Architecture

One command per capture: `floorplan run <capture> --out <dir>`. The tier is auto-detected from the input (`ingest/detect.py`): a Stray Scanner or Record3D zip/folder (contains `odometry.csv`) is LiDAR, a video file is video, a folder of images is photos.

```
ingest (tier autodetect)
  -> reconstruct: LiDAR depth+pose fusion | video: keyframes + SfM + monocular metric depth
  -> metric Z-up point cloud + provenance (shared intermediate representation)
  -> geometry: planes, free-space room outline, wall-support layout, openings, ceiling
  -> confidence: bootstrap CIs + systematic floor, widened per tier
  -> export: results.json (schema/floorplan.schema.json), floor_plan.svg, floor_plan.dxf
```

All tiers end in the same geometry, confidence and export stages, so `results.json` has one shape and tiers are directly comparable. Everything runs locally and offline; the only download is the monocular depth model (Depth Anything V2 Metric Indoor Small, via `scripts/fetch_weights.sh`). `results.json` is deterministic (byte-identical across runs; see section 6).

Key design decisions:
- **One free-space room outline, not independent planes.** Walls are the edges of a single polygon, so walls, area and adjacency always agree. This is also where our main errors come from (section 6).
- **Openings need evidence.** A door is reported only when scan points are seen beyond the wall inside the gap, to avoid phantom doors from unscanned patches. The cost is that many real openings are missed (section 5).
- **Capture format.** Stray Scanner exports `rgb.mp4`, `depth/*.png` (mm), `confidence/*.png`, `odometry.csv` (poses plus intrinsics) and `camera_matrix.csv`. This is the same layout the assignment samples use, so no converter is needed.

## 2. Tier design and device matrix

| Tier | Input | Hardware | Scale source | Status |
|---|---|---|---|---|
| LiDAR | Stray Scanner zip | iPhone Pro with LiDAR | native metres from depth | works; gates mostly fail |
| Video | any `.mp4` / `.mov` | iPhone 15 or newer | monocular depth model, anchored with `--scale-ref` (known ceiling height) | runs; 0 of 4 rooms usable |
| Photos | folder of stills | iPhone 15 or newer | same as video | **not delivered** |

**Video scale.** A video has no metric scale. The model's depth is biased high by 30 to 57 % on this data; we correct it with a focal-length law and anchor it with one known length. Without `--scale-ref` the plan is flagged non-metric (`scale.metric=false`) and its intervals are widened. For the walk-in test that means someone must read a ceiling height off a laser measurer and pass it in; we do not hide that.

**Photos.** A folder of stills goes through the same reconstruction as video. Structure-from-motion registered 0 of 8 images on all five test clips, so the tier falls back to a single-image prior. Per-room folders are not stitched into a property plan. This fails the photo-tier stitch gate outright.

## 3. Drift handling (multi-room)

Visual-inertial odometry drifts, and a multi-room walk returns to its start displaced. `drift.py` builds a **keyframe pose graph**: keyframes every few frames, loop candidates by position, each candidate **verified by point-to-plane ICP** (fitness and neighbour agreement), then a global pose-graph optimisation. With no verified revisit the correction is the identity and `results.json -> drift` says so. Switch: `--no-drift-correction`.

Ablation on the multi-room sample `c7d28f72c6` (`benchmark/ablation/c7d28f72c6/`):

| | ON | OFF |
|---|---|---|
| verified loops | 13 | 0 |
| loop-closure error median / max | 2.1 / 12.5 cm | 25.5 / 55.3 cm |
| stitched footprint | 69.2 m² | 60.0 m² |
| rooms found | 5 | 6 |

Correction closes a 25 cm gap and moves the footprint by 13 %. **We have no tape truth for that property**, so we show the loop gap shrinking, not that the footprint became more accurate. On our single rooms drift is small (study_room_friend: median loop error 2.7 to 0.8 cm) and not the dominant error.

## 4. Error budget (LiDAR tier)

Where the centimetres go, in order of size, based on evidence in this repo:

| Source | Size | Evidence |
|---|---|---|
| Obstacles in front of walls (full-height cabinets 64 and 31 cm deep, pillars 10 cm) | 10 to 60 cm | kitchen plan length = tape wall-to-wall minus protrusions to within 1 cm; `fixloop/DECLARATION.md` |
| Scan-to-scan layout differences | up to 25 cm | repeatability pair, section 6 |
| Ceiling observation and noise | 1 to 3.5 cm | ceiling errors 0.9, 0.9, 3.3, 3.5 cm |
| Depth noise and pose error (single room) | about 1 to 3 cm | fused points sit a median 1.2 cm from the ARKit mesh on an external check scene |
| Tape ambiguity (what counts as the wall) | up to a few cm | wall-to-wall vs clear length |

We have not run an error-propagation model; this table is empirical, not derived.

## 5. Calibration analysis

Intervals are bootstrap CIs plus a per-tier systematic floor, widened by the scale uncertainty. Against tape: **20 samples, 40 % inside the 95 % interval** (target 85 %). The intervals are **too narrow**. The dominant errors (obstacles, wrong layer chosen) are systematic and are not in the bootstrap, so the system is confident and wrong. We did not recalibrate; the floor should be fitted to the errors in section 4. The openings gate fails at 0 %: the evidence rule misses real doors and windows, and every opening that is produced is outside 2 cm. **Confident garbage on thin input is the failure mode the assignment caps; the video tier shows it** (14 phantom walls on one room) and its intervals do not widen enough to warn.

## 6. The fix loop

**Declaration (written first, commit `46b1413`, `fixloop/DECLARATION.md`).** Worst gate: wall length. Every capture fails; worst case study_room_friend at **161.4 cm** worst-wall error, with 6 phantom walls and a second phantom room.

**Root cause.** Walls were the edges of the free-space outline, not planes fitted to wall points. Three defects: (1) an edge snapped to the first dense layer within 40 cm, which is a cabinet face, not the wall; (2) a pillar or soffit near the ceiling inside the wall evidence band pulled the edge in; (3) outline edges with no wall points behind them were still exported as walls and rooms. Evidence: kitchen plan lengths equal tape wall-to-wall minus the protrusions to within 1 cm on both axes (2.282 = 3.23 - 0.639 - 0.309 m).

**Fix (`floorplan/geometry/layout.py`, `config.py`).** Cap the wall band below the ceiling; search farther and choose the outermost layer that covers most of the edge; drop short unsupported edges and rooms with weak perimeter support. Diff: `git diff fixloop-before..HEAD -- floorplan`.

**Prediction vs result.**

| | Predicted | Actual |
|---|---|---|
| study_room_friend worst wall | about 40 cm (15 to 70) | **5.6 cm** |
| phantom walls | at most 1 | 0 |
| rooms | 2 to 1 | 2 to 1 |
| kitchen long axis | stays at least 50 cm | 6 to 19 cm |
| gate passes | none | max-wall gate passes on 1 of 4 rooms; 3 cm per-wall gate on none |

By the assignment's rule this prediction is **badly wrong**, in the optimistic direction. I under-estimated how much of the 161 cm was phantom edges and the leaked second room, which one change removed. Accuracy went from 7 % (2 of 27) to 26 % (6 of 23). The root cause was confirmed; the capture-level wall gate is not passed, only the worst-wall limit on one room. bedroom_2 is still 62 cm off; not investigated.

**Side finding: non-determinism, fixed.** The same capture gave 18.8 cm then 65.8 cm worst-wall error with identical code, and 161.4, 161.7, 115.0 cm before the fix. My first suspicion (drift correction) was wrong: poses were byte-identical. The cause was Open3D's voxel downsampling returning the same points in a different order, which a 2 cm grid layout amplified. `canonical_order()` (round to 0.1 mm, sort) fixed it: `results.json` is byte-identical across five full runs (`tests/test_determinism.py`). It also removed the idea that the repeatability numbers were noise.

**Regenerate before and after:** `fixloop/RESULTS.md` (tag `fixloop-before`; use `python -m floorplan.cli` from the checkout under test, because the installed script imports whichever checkout was pip-installed). Captures are in `benchmark/captures/` (not in git; see `REPRODUCE.md`).

## 7. Known failure modes

- **Cabinets and furniture against walls**: the plan finds the cabinet face. Partly fixed; bedroom_2 (62 cm) is still wrong.
- **Doors and windows**: needs points seen beyond the wall. Closed doors and windows are mostly missed (openings gate 0 %).
- **Two scans of one room disagree** by up to 25 cm (kitchen).
- **Ceiling not swept**: `observed: false`, a prior with a wide interval.
- **Video**: whip-pans (about 78 degrees in 0.25 s) fragment SfM; monocular depth varies per frame by about 11 %, producing doubled parallel walls (diagnosed, a partial fix exists in a worktree, not shipped).
- **Mirrors and glass**: LiDAR returns reflections or passes through; **no test capture exists and nothing handles them**. Same for **wet or glossy floors and low light** (the LiDAR tier is light-independent, the video tier is not). These are risks stated, not covered.
- **Colmap non-determinism**: SfM can break across reruns (one rerun gave 38 cm pose error), so video results are not reproducible run to run.

## 8. Process, and what is missing

History is incremental: Over 35 commits on `main` from 3 to 4 October, including declaration-before-fix and a per-capture benchmark commit. AI coding tools were used throughout. Missing or weak, in order of damage to the score: photo tier; head-to-head (no magicplan or Polycam export); damage, concealed-damage and scope outputs (empty arrays, schema-valid); a tape-measured multi-room property and a damage-staged room; confidence calibration; opening detection; the install path was verified only on the author's machine, not on a clean one. See `COMPLIANCE.md`.
