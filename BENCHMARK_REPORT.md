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
- Footprint, study_room_friend: 9.69 m² vs 9.98 m² tape (-2.9 %).

## 2. Repeatability (one room, two captures, same tier)

Rooms: kitchen_room_scan and kitchen_scan_2, LiDAR.

| Quantity | Result | Gate |
|---|---|---|
| Max wall difference between the two captures | **25.2 cm** (mean 14.9 cm) | 1 cm or 0.5 % |
| Ceiling spread | **2.4 cm** | 1 cm |
| Same capture, run twice | byte-identical `results.json` (`tests/test_determinism.py`) | |

**Which failure do we have?** Unrepeatable between captures. Run-to-run noise is gone (same input, same bytes), so the 25 cm wall gap and 2.4 cm ceiling spread are real differences between the two scans (ceiling errors 0.9 vs 3.3 cm; walls: scan 1 18.8 cm worst, scan 2 7.6 cm). We did not establish why; cabinets blocking the wall is the leading suspect, not a tested cause. We cannot call the system biased: with one room twice there is not enough data to separate a bias from spread.

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
