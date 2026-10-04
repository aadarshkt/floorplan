# Fix loop results and post-mortem

Declaration (committed first, `46b1413`): `fixloop/DECLARATION.md`.
Fix: `floorplan/geometry/layout.py` + `floorplan/config.py` (readable diff:
`git diff fixloop-before..HEAD -- floorplan`).

## Regenerate (both runs, same manifest)

```bash
# BEFORE: the code at tag fixloop-before (= df71ed9)
git checkout fixloop-before
python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/fixloop_before --runs benchmark/runs/fixloop_before --force
# AFTER: this branch
git checkout worktree-kitchen-benchmark
python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/fixloop_after --runs benchmark/runs/fixloop_after --force
```

Use `python -m floorplan.cli` from the checkout you want to test. The installed
`floorplan` script imports the editable install, which points at the main checkout,
not at a worktree (this cost one wasted "after" run during this fix).
Captures live in `benchmark/captures/` (not in git, too large); reports are in
`fixloop/before_report.md`, `after_report.md`, `after_run2_report.md`.

## Numbers (wall length gate, tape ground truth)

| capture | worst wall error before | after (run 1) | after (run 2) | phantom walls before / after |
|---|---|---|---|---|
| study_room_friend | **161.4 cm** | **5.6 cm** | 5.6 cm | 6 / 0 |
| kitchen_scan_2 | 96.2 cm | 7.7 cm | 7.6 cm | 0 / 0 |
| kitchen_room_scan | 98.2 cm | 18.8 cm | **65.8 cm** | 0 / 0 then 2 |
| bedroom_2 | 137.4 cm | 61.4 cm | 61.4 cm | 4 / 0 |

Other: rooms found on study_room_friend 2 -> 1; its footprint 13.49 -> 9.69 m2
(tape 9.98 m2). Overall accuracy 7 % -> 26 % (2/27 -> 6/23 measurements).
28 unit tests pass.

## Gate status

- Wall `wall_length_max_abs_cm` (6 cm): study_room_friend now passes (5.6 cm).
- Wall `wall_length_abs_cm` (3 cm per wall): still fails everywhere (study: 3.6 and 5.6).
  So the capture-level wall gate is **not** passed. Movement is large; the gate is not met.
- Repeatability (kitchen pair) got worse, not better: before 2.0 cm max (consistently
  wrong), after 25.2 cm in run 1 and larger in run 2. See below.
- Openings, ceiling, CI coverage (15 % -> 40 %, target 85 %) not fixed.

## Prediction vs result (honest)

| prediction | actual |
|---|---|
| study worst wall error ~40 cm (range 15-70 cm), gate still fails | 5.6 cm. **Outside my range, in the optimistic direction (7x better than predicted).** The max-wall gate passes. |
| phantom walls <= 1 | 0 (correct) |
| rooms 2 -> 1 | correct |
| kitchen short axis <= 5 cm | 6.4-7.7 cm (slightly over) |
| kitchen long axis stays >= 50 cm | 6-19 cm (wrong: better than predicted) |

My prediction was too pessimistic. I under-estimated how much of the 161 cm error was the
phantom edges and the leaked second room, which removing the edges and the room fixed
at once. By the scoring rule this counts as a badly wrong prediction.

## Root cause: confirmed, with caveats

Confirmed: edges snapped to the first obstacle layer (cabinet and pillar faces) and
unsupported edges were exported as walls. Fixing both moved the errors by an order of
magnitude on three of four captures.

Not fixed / found along the way:
1. **Non-determinism.** `kitchen_room_scan` gives 18.8 cm in one run and 65.8 cm (2 phantom
   walls) in the next, with identical code. Before the fix, `study_room_friend` also moved
   161.4 -> 161.7 -> 115.0 cm between runs that used the same code. Seed/threading
   sources are in `TODO.txt` (drift ICP). Until this is fixed the repeatability gate fails
   and single-run numbers carry noise.
2. **bedroom_2 still 61 cm off** on the long wall. Likely a wardrobe-type obstacle the
   coverage rule does not look past; not investigated.
3. **Tape ambiguity.** Ground truth is wall to wall; with cabinets in the way the real wall
   is only partly visible (coverage ~0.6), so the rule uses a threshold of 0.5 that I tuned
   on this data. It may over-fit these four rooms.
4. Kitchen footprint is now 7.63 m2 vs 7.24 m2 from the tape (+5.4 %).
