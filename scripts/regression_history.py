"""Append a benchmark snapshot to benchmark/history.csv and print the trend.

    python scripts/regression_history.py append <tag> <report_dir>
    python scripts/regression_history.py show
"""
from __future__ import annotations

import csv
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HIST = ROOT / "benchmark" / "history.csv"
FIELDS = ["tag", "timestamp", "commit", "passed", "total",
          "accuracy_pct", "ci_coverage_pct", "detail"]


def _sha() -> str:
    try:
        return subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=ROOT,
                              capture_output=True, text=True).stdout.strip()
    except Exception:
        return ""


def append(tag: str, out_dir: str) -> dict:
    gates = json.loads((Path(out_dir) / "gates.json").read_text())
    s = gates["summary"]
    detail = ";".join(f"{c['capture_id']}={'PASS' if c['passed'] else 'FAIL'}"
                      for c in gates["per_capture"])
    row = {
        "tag": tag,
        "timestamp": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "commit": _sha(),
        "passed": s["captures_passed"],
        "total": s["captures_total"],
        "accuracy_pct": round(100.0 * s["accuracy"]["accuracy"], 1),
        "ci_coverage_pct": round(100.0 * s["ci"]["coverage"], 1),
        "detail": detail,
    }
    new = not HIST.exists()
    HIST.parent.mkdir(parents=True, exist_ok=True)
    with HIST.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)
    return row


def show() -> None:
    if not HIST.exists():
        print("no history yet — run scripts/regression.sh first")
        return
    rows = list(csv.DictReader(HIST.open()))
    header = f"{'tag':22} {'commit':8} {'acc%':>6} {'ci%':>5} {'pass':>6}  detail"
    print(header)
    print("-" * len(header))
    for r in rows:
        print(f"{r['tag']:22} {r['commit']:8} {r['accuracy_pct']:>6} "
              f"{r['ci_coverage_pct']:>5} {r['passed'] + '/' + r['total']:>6}  {r['detail']}")


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "show"
    if cmd == "append" and len(sys.argv) >= 4:
        append(sys.argv[2], sys.argv[3])
        show()
    else:
        show()
