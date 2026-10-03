"""Write run provenance (reproducibility metadata) and stage timings.

provenance.json answers "how was this produced": tool version, settings, which
code path won, input identity, and per-stage wall-clock timings. It is
deliberately NOT part of results.json, which must stay deterministic.
"""
from __future__ import annotations

import datetime as _dt
import hashlib
import json
import platform
from pathlib import Path

from floorplan import __version__
from floorplan.config import Settings


def _input_hash(path: str | Path) -> str:
    p = Path(path)
    try:
        st = p.stat()
        return hashlib.sha1(f"{p.resolve()}:{st.st_size}:{int(st.st_mtime)}".encode()).hexdigest()[:16]
    except OSError:
        return "unknown"


def write(out: str | Path, *, tier: str, capture_id: str, cfg: Settings,
          timings: dict, path_chosen: str, input_path: str | Path,
          extra: dict | None = None) -> dict:
    payload = {
        "floorplan_version": __version__,
        "generated_at": _dt.datetime.now(_dt.timezone.utc).isoformat(),
        "platform": platform.platform(),
        "tier": tier,
        "capture_id": capture_id,
        "path_chosen": path_chosen,
        "input": {"path": str(input_path), "hash": _input_hash(input_path)},
        "settings": cfg.to_dict(),
        "timings_s": {k: round(float(v), 3) for k, v in timings.items()},
        "total_s": round(float(sum(timings.values())), 3),
    }
    if extra:
        payload.update(extra)
    Path(out).write_text(json.dumps(payload, indent=2) + "\n")
    return payload
