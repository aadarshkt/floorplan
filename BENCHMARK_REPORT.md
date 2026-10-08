# Benchmark report

All ground truth is **tape measure, taken by the author** (`benchmark/ground_truth/*.json`, one room each, 4 rooms).
Captures: Stray Scanner, iPhone Pro, handheld. Machine: Apple M1, 8 GB, CPU/MPS only.
Gates are the assignment's (`benchmark/gates.json` keys): wall 3 cm (6 cm worst wall), ceiling 1.5 cm, opening 2 cm on 85 %, area 5 %, repeatability 1 cm, CI coverage 85 %.

**Not a result:** `benchmark/selftest/` scores the harness against ground truth that was copied from the pipeline's own output. Its 100 % is a harness self-test and means nothing about accuracy.

## 1. LiDAR tier (after the fix loop)

Regenerate: `floorplan bench --manifest benchmark/manifest.stray.json --out benchmark/reports/stray --runs benchmark/runs/stray --force`
Archived: `fixloop/after_deterministic_report.md`.

| Capture | Worst wall (cm) | Mean wall (cm) | Ceiling error (cm) | Openings in tolerance | Phantom walls |
|---|---|---|---|---|---|
| study_room_friend | **5.6** | 3.5 | 0.9 (pass) | 0 of 2 (1 missed) | 0 |
| kitchen_scan_2 | 7.6 | 7.0 | 3.3 (fail) | 0 of 3 (2 missed) | 0 |
| kitchen_room_scan | 18.8 | 15.5 | 0.9 (pass) | 0 of 3 (2 missed) | 0 |
| bedroom_2 | 62.0 | 36.6 | 3.5 (fail) | 0 of 1 (1 missed) | 0 |

- Overall accuracy **26 % (6 of 23 measurements within tolerance)**. Captures passing all gates: **0 of 4**.
- Wall gate: only study_room_friend meets the 6 cm worst-wall limit. No capture meets the 3 cm per-wall limit.
- Opening gate: **fails, 0 %.** Doors and windows are not detected as the right width or at all. We did not fix this.
- Ceiling gate: 2 of 4 rooms pass.
- Footprint, study_room_friend: 9.75 m² vs 9.98 m² tape (-2.3 %).

### 1a. Closer look: study_room_friend (no wardrobe or cabinets)

The one room with no full-height furniture against a wall, so it shows the pipeline's accuracy when obstacles are not the problem.
Output: `benchmark/runs/stray/study_room_friend/` (from the section 1 regenerate command; byte-identical to the archived deterministic run). Tape: `benchmark/ground_truth/study_room_friend.json`.

| Quantity | Tape | Plan | Error | Plan 95 % interval | Tape inside? | Gate |
|---|---|---|---|---|---|---|
| Long walls (w0, w2) | 3.400 m | 3.385 m | -1.5 cm (0.4 %) | 3.339 to 3.431 | yes | 3 cm: pass |
| Short walls (w1, w3) | 2.935 m | 2.879 m | **-5.6 cm** (1.9 %) | 2.820 to 2.938 | yes (barely) | 3 cm: **fail**; 6 cm worst wall: pass |
| Ceiling height | 2.836 m | 2.845 m | +0.9 cm | 2.834 to 2.856 | yes | 1.5 cm: pass |
| Floor area | 9.979 m² | 9.746 m² | -2.3 % | 9.50 to 9.99 | yes | 5 %: pass |
| Walls / rooms | 4 / 1 | 4 / 1 | no phantoms | | | pass |

Leaving openings aside, the room passes 4 of 5 gates. All four tape values fall inside their intervals here; the low overall calibration (40 %) comes from the other rooms. The plan is a rectangle, so opposite walls share one length: there are two independent wall errors, not four.

**Why the short walls are 5.6 cm short (diagnosed, not fixed).** The two long walls are 5.6 cm too close together, and the cause is the north long wall (edge 2), not furniture. In the fused cloud, that wall's face drifts from -3 cm to +11 cm outward along its 3.4 m length (about 2.4° off parallel), and the rectilinear layout snaps the edge to its inner end. Fusing each quarter of the walk separately shows that the slope is **pose drift, not the room**:

| Frames | Edge 2 wall-face offset along its length (cm, 0.3 m steps) |
|---|---|
| 0 to 1815 | 11 12 12 13 14 14 14 14 14 13 13 (straight, parallel) |
| 1830 to 3645 | -3 -2 0 1 2 3 6 7 9 10 11 (rotated) |
| 5475 to 7275 | partial view: -1 3 3 4 4 5 |

The opposite wall (edge 0) agrees across passes to about 1 cm. So the same wall is placed up to about 15 cm apart by different passes of one walk, even after the pose graph closed 10 ICP-verified loops (with drift correction off the plan breaks into 8 walls, so the correction helps, but does not finish the job). Consequences: (1) the -5.6 cm error is a compromise between inconsistent passes; (2) fitting a free-angle line to the wall would make it worse, because it would reproduce the drifted slope; (3) the fix belongs in drift correction (e.g. constrain surfaces seen in several passes to coincide), and the same effect is a candidate for the 25 cm kitchen repeatability gap. Regenerate: `python scripts/wall_passes.py benchmark/captures/study_room_friend benchmark/runs/stray/study_room_friend`.

### 1b. Precomputed runs, including captures without ground truth (added 2026-10-09)

All LiDAR-tier outputs are on Drive as `stray.zip` (about 190 MB). Unzip it into `benchmark/runs/` to get `benchmark/runs/stray/<capture>/`, with `results.json`, `floor_plan.svg`, `floor_plan.dxf`, `scan_metric.ply`, `cameras.json`, `provenance.json` and `layout_debug.png` for each capture. The four tape-measured rooms are byte-identical to what the section 1 command regenerates.

`layout_debug.png` is a top-down view: grey = all scan points, black = the wall band the layout uses (0.9 m above the floor to 0.5 m below the ceiling, vertical surfaces only), blue = camera path, red = chosen room outlines with edge ids `e0, e1, ...` (= walls `r1-w0, r1-w1, ...` in `results.json`), 1 m scale bar. A red edge should sit on the inner face of a black band; where it does not, the black points show what it snapped to. Regenerate: `python scripts/layout_debug.py <run_dir> <run_dir>/layout_debug.png`.

Five more captures were run with the same code. **None has tape ground truth, so they are not scored**; the notes come from visual inspection of `layout_debug.png`.

| Run | Source | Rooms | Area (m²) | Ceiling (m) | Drift loops | What the debug view shows |
|---|---|---|---|---|---|---|
| assignment_1 | assignment `c00a170fe1` (single_room.zip) | 1 | 26.1 | 2.49, not observed (prior) | 0 | Room plus corridor arm merged into one 10-wall outline; the arm's far edges have almost no wall evidence |
| assignment_2 | assignment `1a8384c3f6` (single_scan_floor_only.zip) | 3 | 63.9 | 2.48, not observed (prior) | 3 | **Room split fails**: one 52 m² "room" with 32 walls spans several rooms and the hall |
| assignment_3 | assignment `c7d28f72c6` (single_scan_with_ceiling.zip) | 5 | 70.6 | 2.38 | 13 | Residual drift doubles some walls 10 to 15 cm apart; **room outlines overlap** (stitch gate fails as is) |
| benchmark_1 | own capture | 1 | 18.8 | 2.83 | 0 (no revisit) | Main rectangle about 6.5 x 2.9 m is plausible; clutter adds short phantom walls (0.06, 0.25 m) |
| benchmark_2 | own capture | 1 | 9.3 | 2.84 | 0 (none verified) | Main rectangle about 3.0 x 2.9 m is plausible; notches around a corner object and a possible recess |

Regenerate the assignment runs: `floorplan run ../assignment/Assignment/<id> --out benchmark/runs/stray/assignment_<n>`. The assignment_3 footprint (70.6 m²) differs from the drift ablation in section 6 (69.2 m²) because the ablation was run before the fix loop.

What this adds: on single clean rooms the LiDAR tier is close to tape, but on multi-room captures it fails in ways the scored rooms do not exercise: room separation (assignment_2), overlapping outlines and doubled walls from residual drift (assignment_3), and clutter becoming short phantom walls (benchmark_1/2). The whole-property plan depends on exactly these.

## 2. Repeatability (one room, two captures, same tier)

Rooms: kitchen_room_scan and kitchen_scan_2, LiDAR.

| Quantity | Result | Gate |
|---|---|---|
| Max wall difference between the two captures | **25.2 cm** (mean 14.9 cm) | 1 cm or 0.5 % |
| Ceiling spread | **2.4 cm** | 1 cm |
| Same capture, run twice | byte-identical `results.json` (`tests/test_determinism.py`) | |

**Which failure do we have?** Unrepeatable between captures. Run-to-run noise is gone (same input, same bytes), so the 25 cm wall gap and 2.4 cm ceiling spread are real differences between the two scans (ceiling errors 0.9 vs 3.3 cm; walls: scan 1 18.8 cm worst, scan 2 7.6 cm). We did not establish why; cabinets blocking the wall is the leading suspect, not a tested cause. The author notes the kitchen was cluttered during both scans (not tidied beforehand); we did not re-scan a cleared kitchen, so whether clutter explains the gap is untested. The kitchen stays in the benchmark because it is the only repeatability pair. We cannot call the system biased: with one room twice there is not enough data to separate a bias from spread.

## 3. Confidence-interval calibration

20 samples, **40 % of tape values fall inside the 95 % interval** (target 85 %). Our intervals are too narrow: confident and wrong. We did not recalibrate.

## 4. Video tier

Source: the `rgb.mp4` of the same four Stray Scanner captures, scored against tape (`fixloop/video_tier_report.md`, run `benchmark/manifest.stray_video.json`).

| Capture | Worst wall (cm) | Ceiling error (cm) | Phantom walls |
|---|---|---|---|
| study_room_friend | 140 | 148 | 14 |
| bedroom_2 | 211 | 47 | 2 |
| kitchen_room_scan | 125 | 36 (ceiling unobserved) | 4 |
| kitchen_scan_2 | 99 | 162 | 12 |

Accuracy **9 % (2 of 22)**. The ±3 % video gate is **not met** on any room; the tier is not usable for sizing.
Cross-tier check (not tape; the LiDAR run of the same clip is the reference, `benchmark/reports/xbench_v3/`, regenerate with `floorplan xbench`): on the two clips that SfM registers fully (benchmark_1, benchmark_2) scale error is -6.8 % and +4.0 %, wall error 11.8 % and 5.7 %. On the assignment clips SfM fragments and fails (scale errors 508 % and 708 % on two of them).

## 5. Photo tier

Not delivered as a tier. A folder of stills runs through the same reconstruction as video, but SfM on 2 to 8 stills fails to register (`benchmark/reports/xbench_photos_v3/`: 0 of 8 images registered on all five clips). No per-room folder stitching exists. **Photo-tier whole-property stitch gate: fail.**

## 6. Drift accountability (multi-room)

Capture: assignment sample `c7d28f72c6`, a multi-room walk, no tape truth available.
Regenerate: `floorplan drift-ablate ../assignment/Assignment/c7d28f72c6 --out benchmark/ablation/c7d28f72c6`. Output: `benchmark/ablation/c7d28f72c6/drift_ablation.md`, `drift_overlay.svg`.

| | Correction ON (pose graph, ICP-verified loops) | OFF (poses as-is) |
|---|---|---|
| Verified loops | 13 | 0 |
| Loop-closure error, median / max | **2.1 / 12.5 cm** | 25.5 / 55.3 cm |
| Footprint | 69.2 m² | 60.0 m² (-13.2 %) |
| Rooms found | 5 | 6 |

Interpretation: drift correction removes a 25 cm loop gap and changes the footprint by 13 %. **We have no tape truth for this property, so we cannot show which footprint is right.** The loop-error reduction is real; the accuracy gain is unproven. Single-room Stray Scanner scans also verify loops (study_room_friend: 10 loops, median loop error 2.7 → 0.8 cm).

## 7. Timing (M1, 8 GB)

| Run | Time |
|---|---|
| LiDAR, study_room_friend (486 frames) | 17.6 s |
| LiDAR, typical 1 to 6 room capture | 8 to 22 s |
| Video, short clip, monocular depth + SfM | 32 to 54 s |
| Video, bedroom_2, COLMAP-heavy path | 687 s |

## 8. Head-to-head against a consumer app

**Not done.** No magicplan or Polycam export exists in this submission. Part 3 is therefore not scored and the 70 % tie-or-beat target is not claimed.

## 9. Not benchmarked

Multi-room property with tape truth; furnished room with staged damage; all rooms at all three tiers; mirrors, glass and low light (covered only as known failure modes in the report).
