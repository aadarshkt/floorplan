#!/usr/bin/env bash
# Install the learned monocular-depth backend used by the independent
# photos/video tiers (Depth Anything V2 *Metric*, Indoor).
#
#   scripts/fetch_weights.sh              # installs into ./.venv
#   scripts/fetch_weights.sh /path/to/python
#
# The model weights (~100 MB) download on first inference, not here. For GPU
# inference install the CUDA build of torch first, per https://pytorch.org
# (e.g. pip install torch --index-url https://download.pytorch.org/whl/cu124);
# this script's plain `pip install torch` gets the CPU wheel.
set -euo pipefail

PY="${1:-./.venv/bin/python}"

echo "Installing monocular-depth backend into: $PY"
if command -v uv >/dev/null 2>&1; then
  uv pip install --python "$PY" --upgrade torch transformers pillow
else
  "$PY" -m pip install --upgrade torch transformers pillow
fi

"$PY" - <<'PY'
import importlib.util as u
missing = [m for m in ("torch", "transformers", "PIL") if not u.find_spec(m)]
if missing:
    print(f"STILL MISSING: {missing}")
else:
    import torch
    print(f"monocular backend ready (cuda={torch.cuda.is_available()})")
PY

cat <<'TXT'

Done. The photo/video tiers now run metric-depth + COLMAP-SfM-poses on their own
(no LiDAR capture needed). Configurations:

  # 1. photos only (folder of 2-8+ stills)
  ./.venv/bin/floorplan run <photo_folder> --out benchmark/runs/photos --engine auto

  # 2. video only (handheld walkthrough clip)
  ./.venv/bin/floorplan run <clip.mp4> --out benchmark/runs/video --engine auto

  # 3. hybrid (.r3d bundle: LiDAR + video + photos streams)
  ./.venv/bin/floorplan run <capture.r3d|unpacked_folder> --tier lidar --out benchmark/runs/lidar

Score them:  ./.venv/bin/floorplan bench --manifest benchmark/manifest.photos.json \
                 --out benchmark/reports/photos --force

No --scale-ref / --reference-capture is required: the metric model supplies metres.
TXT
