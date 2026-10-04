# How to test this (one page, for the evaluator)

Time: about 15 minutes after install. Capture rules are in `CAPTURE_PROTOCOL.md`.

## 1. Get it running

```bash
brew install ffmpeg colmap libusb
uv venv --python 3.11 .venv && uv pip install -e .
./.venv/bin/floorplan run capture.zip --out out/
# video tier only (adds torch + the depth model, about 100 MB download):
scripts/fetch_weights.sh
```

## 2. Capture and run (LiDAR tier, the one that works best)

1. Install Stray Scanner, record a room (60 to 120 s, slow, sweep ceiling, look through doors).
2. Zip the recording folder and move it to the computer.
3. `floorplan run capture.zip --out out/`. A 1 to 2 minute scan takes about 20 s on an M1 laptop.
4. Open `out/floor_plan.svg`.

## 3. Score it against your laser

Open `out/results.json`. Every number is `{value, ci95: [low, high]}`.

| What to compare | Where in `results.json` | Pass if |
|---|---|---|
| Ceiling height | `rooms[].ceiling_height_m.value` | within 1.5 cm; check `observed: true` |
| Each wall | `rooms[].walls[].length_m.value` | within 3 cm (walls are interior, face to face) |
| Door / window width | `rooms[].walls[].openings[].width_m.value` | within 2 cm; a missed or extra opening counts as a miss |
| Floor area | `rooms[].floor_area_m2.value` | within 5 % |
| Confidence | `ci95` on each | your laser value should fall inside `ci95` about 95 % of the time |

Or fill `benchmark/ground_truth/TEMPLATE.json` with your readings and score automatically:
```bash
floorplan bench --manifest your_manifest.json --out report/     # see benchmark/README.md
```

## 4. What to expect (so the result is read honestly)

- Our own tape-measured rooms: 0 of 4 pass all gates; best wall error 5.6 cm; doors and windows are mostly **not detected**; confidence intervals are too narrow (40 % coverage vs 85 % target). Details: `BENCHMARK_REPORT.md`.
- Video: runs, but 0.9 to 2.1 m wall errors. Pass `--scale-ref <ceiling m>`. Photo folders: not supported as a separate tier.
- Mirrors, glass, dark rooms, and fast pans degrade results; a plan built without an observed ceiling says `observed: false`.
