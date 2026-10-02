# Benchmark

This directory defines *what correct means*. The harness (`floorplan bench`)
runs every capture in a manifest, compares the produced plan to laser/tape ground
truth, and gates the result.

## Files

- `manifest.template.json` — copy this and fill it in for a real benchmark.
- `ground_truth/TEMPLATE.json` — the per-room measured truth.
- `manifest.example.json` + `ground_truth/example_*.json` — a runnable demo using
  the three provided scans with **placeholder** ground truth. The numbers are
  derived from the pipeline output, so they are NOT real measurements; replace
  them with tape/laser values. They exist only to exercise the harness.

## How to run

```bash
./.venv/bin/floorplan bench --manifest benchmark/manifest.json --out reports/before
```

Optional:

- `--reuse <dir>` — reuse cached `<capture_id>/results.json` (fast re-scoring;
  used by the fix loop's before/after runs).
- `--conf-min`, `--max-frames` — pipeline overrides applied to every capture.

## Ground-truth format (what to measure with the laser)

```json
{
  "room_id": "room1",
  "ceiling_height_m": 2.42,          // floor -> ceiling
  "floor_area_m2": 13.1,             // optional
  "wall_lengths_m": [4.20, 3.15],    // ONE number per wall, order-free
  "openings": [
    {"type": "door",   "width_m": 0.82},
    {"type": "window", "width_m": 1.20}
  ]
}
```

Wall lengths and openings are **unordered sets** — the harness matches produced
elements to measured ones with an optimal (Hungarian) assignment, so it never
penalises the model for ordering.

## What the harness measures, and how it maps to the code

| Gate | Produced by | Ground truth |
|---|---|---|
| `wall_length_*` | `rooms[].walls[].length_m.value` | `wall_lengths_m` |
| `ceiling_height_abs_cm` | `rooms[].ceiling_height_m.value` | `ceiling_height_m` |
| `opening_*` | `rooms[].walls[].openings[]` | `openings` |
| `area_rel_pct` | `rooms[].floor_area_m2.value` | `floor_area_m2` |
| `repeatability_*` | two runs of the same room | (none — self-comparison) |
| `ci_coverage_min` | all `ci95` intervals | whether the true value falls inside |

## Outputs

- `reports/<tag>/gates.json` — machine-readable: per-capture checks, summary.
- `reports/<tag>/report.md` — the human table for the write-up.
- `reports/<tag>/captures/<id>/` — every run's full artifact set.
