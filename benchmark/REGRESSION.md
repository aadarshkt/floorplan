# Regression runbook — `2026-10-03--00-39-56`

Re-run **all three tiers** (lidar, photos, video) on the same room and track the
scores over time. Everything lives under `benchmark/`.

## Run it

```bash
# from the repo root; tags the snapshot with the current timestamp
scripts/regression.sh

# or give the snapshot a name
scripts/regression.sh after-ceiling-fix
```

That one command:

1. runs `floorplan bench` over `benchmark/manifest.json` (3 tiers, `--force`),
2. writes `benchmark/reports/<tag>/{report.md, gates.json, runs/<tier>/}`,
3. appends a row to `benchmark/history.csv`.

Overrides (env vars): `REFERENCE_CAPTURE=…`, `ENGINE=auto|monodepth|colmap`.

## Read the result

| Look at | For |
|---|---|
| `benchmark/reports/<tag>/report.md` | the headline **Accuracy %**, the per-tier gate table, CI coverage |
| `benchmark/reports/<tag>/gates.json` | the same, machine-readable (`summary.accuracy`) |
| `benchmark/reports/<tag>/runs/<tier>/provenance.json` | which path won (`path_chosen`), how scale was set, arbiter scores, timings |
| `benchmark/reports/<tag>/runs/<tier>/floor_plan.svg` | eyeball the plan |

## Track over time

```bash
python scripts/regression_history.py show
```

```
tag                    commit   acc%   ci%   pass  detail
2026-10-03_0548        4c09cf6  100.0 100.0  3/3   lidar=PASS;photos=PASS;video=PASS
after-ceiling-fix      9f1a2b3  100.0 100.0  3/3   lidar=PASS;photos=PASS;video=PASS
```

Each row records the tag, git commit, accuracy %, CI coverage %, pass count and a
per-tier PASS/FAIL line — so a regression (or an improvement) is one line.

## Ground truth is the input that matters

`benchmark/ground_truth/2026-10-03--00-39-56.json` is still **pre-filled from an
earlier run**, so the scores above are self-referential. Replace it with real
tape/laser numbers (one entry per wall, ceiling height, each opening) and re-run —
that is when the accuracy becomes meaningful.

## Notes

- Inputs (`benchmark/captures/`) and outputs (`benchmark/runs/`, `benchmark/reports/`)
  are **gitignored** (large); `manifest.json`, `ground_truth/` and `history.csv`
  are tracked.
- The photos tier folder was made from the video:
  `ffmpeg -i benchmark/captures/2026-10-03--00-39-56/rgb.mp4 -vf fps=1 benchmark/captures/photos_2026-10-03--00-39-56/%03d.jpg`
- COLMAP is opt-in (`ENGINE=colmap scripts/regression.sh`); the default
  `auto` monocular path uses the paired capture above as the metric reference.

See `README.md` (repo root) for the CLI and `benchmark/README.md` for the
manifest / ground-truth format.
