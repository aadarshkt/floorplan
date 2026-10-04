# Submission index

Repo: https://github.com/aadarshkt/floorplan (branch `main`)
Raw benchmark data (too large for git): Google Drive, `raw_benchmark_data/` (link in the email).

Order follows the eight deliverables in the brief.

| # | Deliverable | Where | Status |
|---|---|---|---|
| 1 | Compliance matrix | [COMPLIANCE.md](https://github.com/aadarshkt/floorplan/blob/main/COMPLIANCE.md) | done |
| 2 | Capture route (stock-app protocol) + device matrix | [CAPTURE_PROTOCOL.md](https://github.com/aadarshkt/floorplan/blob/main/CAPTURE_PROTOCOL.md) (Stray Scanner) | done |
| 3 | Repo, README, one command per capture | [README.md](https://github.com/aadarshkt/floorplan/blob/main/README.md), [TESTING.md](https://github.com/aadarshkt/floorplan/blob/main/TESTING.md) | done |
| 4 | Reproduction bundle | [REPRODUCE.md](https://github.com/aadarshkt/floorplan/blob/main/REPRODUCE.md), [benchmark/manifest.stray.json](https://github.com/aadarshkt/floorplan/blob/main/benchmark/manifest.stray.json) | partial: video tier not deterministic |
| 5 | Benchmark report | [BENCHMARK_REPORT.md](https://github.com/aadarshkt/floorplan/blob/main/BENCHMARK_REPORT.md) | done; head-to-head not done |
| 6 | Fix loop bundle | [fixloop/](https://github.com/aadarshkt/floorplan/tree/main/fixloop) (declaration, results, before/after reports), tag `fixloop-before` | done |
| 7 | Technical report, 6 pages max | [REPORT.md](https://github.com/aadarshkt/floorplan/blob/main/REPORT.md), [REPORT.pdf](https://github.com/aadarshkt/floorplan/blob/main/REPORT.pdf) | done |
| 8 | Raw benchmark data | Drive `raw_benchmark_data/` (4 Stray Scanner captures, tape ground truth, manifests, reports); ground truth also in [benchmark/ground_truth/](https://github.com/aadarshkt/floorplan/tree/main/benchmark/ground_truth) | partial: no consumer-app exports |

## Fastest path for a reviewer (about 15 minutes)

```bash
git clone https://github.com/aadarshkt/floorplan && cd floorplan
brew install ffmpeg colmap libusb
uv venv --python 3.11 .venv && uv pip install -e .
unzip study_room_friend.zip -d benchmark/captures/study_room_friend      # from the Drive folder
./.venv/bin/floorplan run benchmark/captures/study_room_friend --out out/
open out/floor_plan.svg
```

## Not done (stated, not hidden)

- Photo tier (per-room stills stitched into one plan).
- Head-to-head against magicplan or Polycam (Part 3).
- Damage regions, concealed-damage flags, scope line items (arrays are present and empty).
- A tape-measured multi-room property, and a staged-damage room.
- Walk-in readiness for the video tier needs a known ceiling height passed with `--scale-ref`.
- Gates not met: opening widths (0 %), per-wall 3 cm, repeatability (25 cm), CI coverage (40 %), video ±3 %.
