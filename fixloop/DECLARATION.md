# Fix declaration (written before the fix)

Date: 2026-10-04. Before-run: git tag `fixloop-before` (commit `df71ed9`).
Benchmark: `benchmark/manifest.stray.json`, LiDAR tier, 4 Stray Scanner captures
with tape ground truth (kitchen x2, bedroom, study_room_friend).

## 1. Worst gate in my own benchmark

**Wall length** (gate: <= 3 cm per wall, <= 6 cm worst). Every capture fails it.

| capture | worst wall error | phantom walls | rooms found / real |
|---|---|---|---|
| study_room_friend | **161.4 cm** | 6 | 2 / 1 |
| bedroom_2 | 137.4 cm | 4 | 1 / 1 |
| kitchen_room_scan | 98.2 cm | 0 | 1 / 1 |
| kitchen_scan_2 | 96.2 cm | 0 | 1 / 1 |

Headline number: **study_room_friend worst wall error = 161.4 cm** (before report:
`benchmark/reports/fixloop_before/report.md`). Overall accuracy 7 % (2 of 27).

## 2. Root-cause hypothesis and evidence

The plan's walls are the edges of one free-space outline (`geometry/layout.py`),
not planes fitted to the black wall-evidence points. Three defects follow:

1. **Edges snap to the first obstacle, not the wall.** `_refine` takes the first
   dense layer within 0.40 m (0.20 m on the second pass). Kitchen evidence: the
   plan's clear lengths equal wall-to-wall minus protrusions to within 1 cm on both
   axes (2.282 = 3.23 - 0.639 - 0.309 m; 2.028 = 2.24 - 0.108 - 0.104 m, tape).
2. **Top-of-wall protrusions count as wall.** The evidence band runs from 0.9 m to
   ceiling - 0.08 m, so a pillar/soffit near the ceiling pulls the edge in (kitchen
   scan: layer at z 2.5-2.9 m sits ahead of the wall layer at z 0.4-2.1 m).
3. **Unsupported outline edges still become walls and rooms.** study_room_friend
   returns 10 edges for a rectangle plus a second 4.9 m2 "room" through the doorway.
   Every outline edge is exported as a wall, whether or not wall points back it.

## 3. Fix and prediction

Fix (all in `floorplan/geometry/layout.py` + `config.py`):
(a) cap the wall band below the ceiling (default ceiling - 0.5 m);
(b) search farther and choose the outermost layer that covers most of the edge;
(c) drop short edges with no wall support and rooms with weak perimeter support.

Prediction on study_room_friend, after the fix:

| number | before | predicted after |
|---|---|---|
| worst wall error | 161.4 cm | **~40 cm** (range 15-70 cm); gate still fails |
| phantom walls | 6 | <= 1 |
| rooms found | 2 | 1 |

Kitchen: short-axis error 20 cm -> <= 5 cm. Long-axis error (cabinets 64 cm /
31 cm deep) stays large (>= 50 cm) unless (b) works; I expect only partial
movement there. I do **not** expect the 3 cm gate to pass on any capture.
Openings are out of scope for this fix (they depend on correct walls).
