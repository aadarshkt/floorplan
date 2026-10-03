#!/usr/bin/env bash
# Run the benchmark (all three tiers) and record the headline scores over time.
#
#   scripts/regression.sh                 # tag = timestamp
#   scripts/regression.sh my-fix-1        # named snapshot
#
# Env overrides:
#   REFERENCE_CAPTURE=benchmark/captures/2026-10-03--00-39-56
#   ENGINE=auto                            # auto | monodepth | colmap
set -euo pipefail
cd "$(dirname "$0")/.."

REF="${REFERENCE_CAPTURE:-benchmark/captures/2026-10-03--00-39-56}"
ENGINE="${ENGINE:-auto}"
TAG="${1:-$(date +%Y-%m-%d_%H%M%S)}"
OUT="benchmark/reports/$TAG"

./.venv/bin/floorplan bench --manifest benchmark/manifest.json --out "$OUT" \
    --engine "$ENGINE" --reference-capture "$REF" --force

./.venv/bin/python scripts/regression_history.py append "$TAG" "$OUT" >/dev/null
echo
echo "snapshot written to $OUT  (report.md + gates.json + runs/)"
./.venv/bin/python scripts/regression_history.py show
