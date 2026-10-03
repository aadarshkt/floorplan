# Benchmark

Everything for scoring lives under this one folder: raw captures, ground truth,
pipeline runs, and reports.

## Layout

```
benchmark/
  manifest.json            which captures to score, and the gate thresholds
  CAPTURE.md               how to capture (Record3D) and unpack a .r3d
  unpack_r3d.py            .r3d -> dataset folder
  captures/<id>/           raw input (Record3D export folder, or a .zip)
  ground_truth/<id>.json   your laser/tape measurements   <-- EDIT THIS
  runs/<id>/               pipeline output per capture (results.json, svg, dxf, ply, provenance)
  reports/<tag>/           scores: gates.json + report.md
```

## The single accuracy number

`report.md` leads with:

```
Accuracy: NN%   (within/total measurements within tolerance)
```

That is **the** accuracy number. Across every scored quantity — each wall length,
the ceiling height, each opening width, and the floor area — it is the share that
lands inside its gate tolerance. 100% means every measurement is within spec.
The second line, `Overall: X/Y captures passed`, is the stricter per-capture
"all gates at once" view.

## Run it

```bash
# score the captures: writes runs/<id>/ and benchmark/{gates.json,report.md}
./.venv/bin/floorplan bench --manifest benchmark/manifest.json --out benchmark

# force a full pipeline re-run (ignore cached results.json)
./.venv/bin/floorplan bench --manifest benchmark/manifest.json --out benchmark --force

# fix loop: shared runs/, before and after a code fix
./.venv/bin/floorplan bench --manifest benchmark/manifest.json \
    --out benchmark/reports/before --runs benchmark/runs
#   ...ship a fix (a real commit)...
./.venv/bin/floorplan bench --manifest benchmark/manifest.json \
    --out benchmark/reports/after  --runs benchmark/runs --force
```

`--runs` (default `<out>/runs`) is where per-capture `results.json` live; they are
reused unless `--force`. `--ablate-drift` adds the drift on/off comparison.

## Ground-truth format (measure these with the laser/tape)

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
penalises ordering. See `ground_truth/TEMPLATE.json`.

## Gate reference

| Gate | Produced by | Ground truth |
|---|---|---|
| `wall_length_*` | `rooms[].walls[].length_m.value` | `wall_lengths_m` |
| `ceiling_height_abs_cm` | `rooms[].ceiling_height_m.value` | `ceiling_height_m` |
| `opening_*` | `rooms[].walls[].openings[]` | `openings` |
| `area_rel_pct` | `rooms[].floor_area_m2.value` | `floor_area_m2` |
| `repeatability_*` | two runs of the same room | (none — self-comparison) |
| `ci_coverage_min` | all `ci95` intervals | whether the true value falls inside |
