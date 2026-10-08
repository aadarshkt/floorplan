"""Completion records for resumable COLMAP stages, keyed by their inputs."""
from __future__ import annotations

import hashlib
import json
from pathlib import Path


def fingerprint(*values) -> str:
    return hashlib.sha256(json.dumps(values, sort_keys=True, default=str).encode()).hexdigest()


def file_identity(path: Path) -> dict:
    st = path.stat()
    return {"path": str(path.resolve()), "bytes": st.st_size, "mtime_ns": st.st_mtime_ns}


def model_identity(path: Path) -> list:
    # Camera/point changes matter even when a model directory name stays fixed.
    return [(p.name, hashlib.sha256(p.read_bytes()).hexdigest())
            for p in sorted(path.glob("*.bin"))]


class StageCache:
    def __init__(self, root: Path):
        self.path = root / "stages.json"
        try:
            self.records = json.loads(self.path.read_text())
        except (OSError, ValueError):
            self.records = {}

    def same_inputs(self, stage: str, key: str) -> bool:
        return self.records.get(stage, {}).get("key") == key

    def complete(self, stage: str, key: str, outputs=()) -> bool:
        return (self.same_inputs(stage, key)
                and self.records[stage].get("complete", False)
                and all(p.exists() for p in outputs))

    def begin(self, stage: str, key: str) -> None:
        self.records[stage] = {"key": key, "complete": False}
        self._write()

    def finish(self, stage: str, key: str) -> None:
        self.records[stage] = {"key": key, "complete": True}
        self._write()

    def fail(self, stage: str, key: str, reason: str) -> None:
        # An interrupted extraction can resume. A failed native GPU process may
        # have written invalid descriptors; discard that database on retry.
        self.records[stage] = {"key": key, "complete": False, "failed": True, "reason": reason}
        self._write()

    def clear(self) -> None:
        self.records.clear()
        self._write()

    def _write(self) -> None:
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(self.records, indent=2))
        temporary.replace(self.path)
