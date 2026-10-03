"""Cross-tier benchmark: video tier vs LiDAR tier on the same Record3D clips.

`floorplan xbench --manifest benchmark/manifest.r3d.json --out benchmark/reports/<tag>`

For every capture (a Record3D export) it runs the LiDAR tier on the export and
the video tier on its ``rgb.mp4`` alone, then scores the video run against the
LiDAR run with :mod:`floorplan.eval.crosstier`. LiDAR runs are cached (they do
not change when the video tier does); video runs are redone with ``--force``.
"""
from __future__ import annotations

import json
import time
from pathlib import Path

from floorplan import pipeline
from floorplan.config import Settings
from floorplan.eval import crosstier


def photo_set(cap: Path, dest: Path, n: int = 8) -> Path:
    """n sharp stills spread over the clip, named by source frame (photos tier input)."""
    if dest.is_dir() and len(list(dest.glob("*.jpg"))) >= 2:
        return dest
    import shutil
    from floorplan.ingest import video
    ts = video.frame_times(cap / "rgb.mp4")
    frames, idx = video.extract_frames(cap / "rgb.mp4", fps=n / max(float(ts[-1]), 1e-3),
                                       mode="sharp", max_frames=n, max_dim=1920)
    dest.mkdir(parents=True, exist_ok=True)
    for i in idx[:n]:
        shutil.copy(frames / f"{i:06d}.jpg", dest / f"{i:06d}.jpg")
    return dest


def run(manifest_path: str | Path, out_dir: str | Path, cfg: Settings | None = None,
        lidar_runs: str | Path | None = None, force: bool = False,
        force_lidar: bool = False, only: list[str] | None = None,
        verbose: bool = True, tier: str = "video") -> list[dict]:
    cfg = cfg or Settings()
    manifest_path = Path(manifest_path)
    base = manifest_path.parent
    manifest = json.loads(manifest_path.read_text())
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    lidar_root = Path(lidar_runs) if lidar_runs else base / "runs" / "lidar_ref"

    rows: list[dict] = []
    for entry in manifest["captures"]:
        cid = entry["id"]
        if only and cid not in only:
            continue
        cap = Path(entry["path"])
        cap = cap if cap.is_absolute() else (base / cap)
        lrun, vrun = lidar_root / cid, out / "runs" / cid

        if force_lidar or not (lrun / "cameras.json").exists():
            if verbose:
                print(f"[xbench] {cid}: LiDAR tier")
            pipeline.run(cap, lrun, cfg=cfg, tier="lidar", verbose=False)

        t0 = time.time()
        err = None
        if force or not (vrun / "results.json").exists():
            src = (cap / "rgb.mp4" if tier == "video"
                   else photo_set(cap, base / "captures" / f"photos_{cid}"))
            if verbose:
                print(f"[xbench] {cid}: {tier} tier on {src}")
            try:
                pipeline.run(src, vrun, cfg=cfg, tier=tier, verbose=verbose)
            except Exception as exc:  # a failed run is a scored result, not a crash
                err = f"{type(exc).__name__}: {exc}"
                if verbose:
                    print(f"[xbench] {cid}: video tier failed: {err}")
        row = {"id": cid, "video_s": round(time.time() - t0, 1)}
        if err:
            row["error"] = err
        else:
            row.update(crosstier.evaluate(lrun, vrun))
        rows.append(row)
        if verbose:
            print(f"[xbench] {cid}: " + json.dumps({k: row.get(k) for k in
                  ("registration", "trajectory", "scale", "up_tilt_deg", "cloud")}))

    # merge with rows from earlier (e.g. --only) runs into the same report
    prev = out / "xbench.json"
    if prev.exists():
        fresh = {r["id"] for r in rows}
        old = [r for r in json.loads(prev.read_text()) if r["id"] not in fresh]
        order = [e["id"] for e in manifest["captures"]]
        rows = sorted(old + rows, key=lambda r: order.index(r["id"]) if r["id"] in order else 99)
    prev.write_text(json.dumps(rows, indent=2) + "\n")
    md = (f"# {tier.capitalize()} tier vs LiDAR — {manifest_path.name}\n\n"
          "LiDAR run of the same clip is the reference (not tape ground truth).\n\n"
          + crosstier.to_markdown(rows))
    (out / "xbench.md").write_text(md)
    if verbose:
        print(md)
    return rows
