# Reproduction bundle

Everything needed to regenerate the numbers in `BENCHMARK_REPORT.md`.

## Inputs (not in git)

Raw Stray Scanner captures are 200 to 850 MB each and are git-ignored. Place them at:

```
benchmark/captures/kitchen_room_scan/<id>/   (odometry.csv, depth/, confidence/, rgb.mp4, camera_matrix.csv)
benchmark/captures/kitchen_scan_2/<id>/
benchmark/captures/bedroom_2/<id>/
benchmark/captures/study_room_friend/<id>/
```

The four folders are supplied as zips on Drive (`raw_benchmark_data/captures/`, about 1.7 GB, see its README). Unzip each into `benchmark/captures/<name>/`. They are **not** in the repo. The tape ground truth **is** in git: `benchmark/ground_truth/*.json`. Multi-room drift sample: the assignment's own `Assignment/c7d28f72c6`.

## Precomputed outputs (Drive, `stray.zip`)

To inspect results without running anything, unzip `stray.zip` into `benchmark/runs/`: you get `benchmark/runs/stray/<capture>/` for 9 captures (the 4 above, `assignment_1..3` = assignment samples `c00a170fe1`, `1a8384c3f6`, `c7d28f72c6`, and `benchmark_1/2`), each with `layout_debug.png`. The four tape-measured ones are byte-identical to what the LiDAR benchmark command below writes. The `benchmark_1/2` raw captures are not on Drive, so those two runs can be inspected but not regenerated.

## Commands

```bash
# LiDAR benchmark (after fix), about 1 to 2 minutes
python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/stray --runs benchmark/runs/stray --force

# Fix loop before / after (see fixloop/RESULTS.md)
git checkout fixloop-before && python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/fixloop_before --runs benchmark/runs/fixloop_before --force
git checkout main          && python -m floorplan.cli bench --manifest benchmark/manifest.stray.json \
    --out benchmark/reports/fixloop_after  --runs benchmark/runs/fixloop_after  --force

# Video tier vs tape (slow; one heavy job at a time on 8 GB)
python -m floorplan.cli bench --manifest benchmark/manifest.stray_video.json --out benchmark/reports/video_stray

# Assignment samples, LiDAR tier (assignment_1..3 in stray.zip)
for p in 1:c00a170fe1 2:1a8384c3f6 3:c7d28f72c6; do
  python -m floorplan.cli run ../assignment/Assignment/${p#*:} --out benchmark/runs/stray/assignment_${p%%:*}
done

# Top-down layout debug view for every run
for d in benchmark/runs/stray/*/; do python scripts/layout_debug.py $d $d/layout_debug.png; done

# Drift ablation
python -m floorplan.cli drift-ablate ../assignment/Assignment/c7d28f72c6 --out benchmark/ablation/c7d28f72c6

# Tests
pytest -q
```

Use `python -m floorplan.cli` from the checkout under test: the installed `floorplan` script points at whichever checkout was pip-installed.

## What replays deterministically

- LiDAR tier: `results.json` is byte-identical across runs (`tests/test_determinism.py`). `scan_metric.ply` is not guaranteed byte-identical (1e-14 m float noise).
- Video tier: **not deterministic** (COLMAP SfM varies between runs). No cache is shipped, so video numbers may differ slightly on rerun.

## Weights

`scripts/fetch_weights.sh` installs torch + transformers; the Depth Anything V2 Metric Indoor Small model (about 100 MB) downloads on first use.
