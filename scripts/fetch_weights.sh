#!/usr/bin/env bash
# Optional: install the learned monocular-depth backend used by the photos/video
# tiers. COLMAP (SfM + CPU plane-sweep MVS) already gives a working photo/video
# path without this; the learned model is the "monocular depth" alternative.
#
#   scripts/fetch_weights.sh            # installs into ./.venv
#   scripts/fetch_weights.sh /path/to/python
#
# Model weights (Depth Anything V2 Small, ~100 MB) download on first inference.
set -euo pipefail

PY="${1:-./.venv/bin/python}"

echo "Installing monocular-depth backend into: $PY"
"$PY" -m pip install --upgrade torch transformers

"$PY" - <<'PY'
import importlib.util as u
missing = [m for m in ("torch", "transformers") if not u.find_spec(m)]
print("monocular backend ready" if not missing else f"STILL MISSING: {missing}")
PY

echo
echo "Done. Use it with:  floorplan run <photos|video> --engine monodepth"
echo "Without a scale reference the plan is marked non-metric (wide intervals);"
echo "add --reference-capture <record3d> or --scale-ref <m> --scale-ref-kind ceiling_height."
