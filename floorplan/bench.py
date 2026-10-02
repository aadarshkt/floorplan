"""Benchmark harness: run captures, compare to laser/tape ground truth, gate them.

`floorplan bench --manifest M --out D [--reuse R]`

For each capture the harness runs the pipeline (or reuses cached `results.json`),
matches produced measurements to ground truth, and writes `gates.json` +
`report.md`. This same command is the fix loop's before/after runner.
"""
from __future__ import annotations

import json
from pathlib import Path

from floorplan import pipeline
from floorplan.config import Settings
from floorplan.eval import groundtruth, metrics, report

DEFAULT_GATES = {
    "ceiling_height_abs_cm": 1.5,
    "opening_width_abs_cm": 2.0,
    "opening_pass_rate": 0.85,
    "opening_phantom_max": 1,
    "wall_length_abs_cm": 3.0,        # mean
    "wall_length_max_abs_cm": 6.0,    # worst matched wall
    "area_rel_pct": 5.0,
    "repeatability_max_cm": 1.0,
    "repeatability_rel_pct": 0.5,
    "ci_coverage_min": 0.85,
}


def _resolve(base: Path, p: str) -> Path:
    q = Path(p)
    return q if q.is_absolute() else (base / q)


def _pick_room(result: dict, room_id: str) -> dict:
    rooms = result.get("rooms", [])
    for r in rooms:
        if r.get("room_id") == room_id:
            return r
    return rooms[0] if rooms else {}


def room_measurements(room: dict) -> dict:
    ceil = room.get("ceiling_height_m") or {}
    area = room.get("floor_area_m2") or {}
    walls = room.get("walls", [])
    return {
        "walls": [w["length_m"]["value"] for w in walls],
        "wall_ci": [w["length_m"].get("ci95") for w in walls],
        "ceiling": ceil.get("value"),
        "ceil_ci": ceil.get("ci95"),
        "observed": bool(ceil.get("observed", False)),
        "area": area.get("value"),
        "area_ci": area.get("ci95"),
        "openings": [{"type": o["type"], "width_m": o["width_m"]["value"]}
                     for w in walls for o in w.get("openings", [])],
    }


def evaluate_capture(capture_id: str, room_id: str, result: dict,
                     gt: groundtruth.GroundTruth, gates: dict,
                     ci_samples: list) -> dict:
    m = room_measurements(_pick_room(result, room_id))
    wl = metrics.wall_length_errors(m["walls"], gt.wall_lengths_m)

    ceil_err = None
    ceil_pass = True
    if gt.ceiling_height_m is not None:
        if m["ceiling"] is None:
            ceil_pass = False
        else:
            ceil_err = round(abs(m["ceiling"] - gt.ceiling_height_m) * 100.0, 2)
            ceil_pass = ceil_err <= gates["ceiling_height_abs_cm"]
            if m["ceil_ci"]:
                ci_samples.append(("ceiling_height", gt.ceiling_height_m,
                                   m["ceil_ci"][0], m["ceil_ci"][1]))

    wall_pass = True
    if gt.wall_lengths_m:
        wall_pass = (
            wl["n_matched"] == len(gt.wall_lengths_m)
            and wl["mean_abs_cm"] is not None
            and wl["mean_abs_cm"] <= gates["wall_length_abs_cm"]
            and wl["max_abs_cm"] <= gates["wall_length_max_abs_cm"]
        )
        for i, j in wl["pairs"]:
            ci = m["wall_ci"][i]
            if ci:
                ci_samples.append(("wall_length", gt.wall_lengths_m[j], ci[0], ci[1]))

    op = metrics.opening_errors(m["openings"], gt.openings,
                                tol_cm=gates["opening_width_abs_cm"])
    open_pass = ((op["pass_rate"] or 0) >= gates["opening_pass_rate"]
                 and op["phantom"] <= gates["opening_phantom_max"])

    area_rel = None
    if m["area"] and gt.floor_area_m2:
        area_rel = round(abs(m["area"] - gt.floor_area_m2) / gt.floor_area_m2 * 100.0, 3)
        if m["area_ci"]:
            ci_samples.append(("floor_area", gt.floor_area_m2, m["area_ci"][0], m["area_ci"][1]))
    area_pass = area_rel is None or area_rel <= gates["area_rel_pct"]

    passed = bool(wall_pass and ceil_pass and open_pass and area_pass)
    return {
        "capture_id": capture_id,
        "room_id": room_id,
        "wall_length": wl,
        "ceiling_height": {"produced": m["ceiling"], "gt": gt.ceiling_height_m,
                           "abs_cm": ceil_err, "observed": m["observed"]},
        "openings": op,
        "area": {"produced": m["area"], "gt": gt.floor_area_m2, "rel_pct": area_rel},
        "passed": passed,
        "checks": {"wall": wall_pass, "ceiling": ceil_pass,
                   "openings": open_pass, "area": area_pass},
    }


def evaluate_repeatability(room_id: str, a: dict, b: dict, gates: dict) -> dict:
    ma = room_measurements(_pick_room(a, room_id))
    mb = room_measurements(_pick_room(b, room_id))
    r = metrics.repeatability(ma, mb)
    passed = ((r["max_wall_cm"] is None or r["max_wall_cm"] <= gates["repeatability_max_cm"])
              and (r["area_rel_pct"] is None
                   or r["area_rel_pct"] <= gates["repeatability_rel_pct"]))
    return {"room_id": room_id, **r, "passed": passed}


def run(manifest_path: str | Path, out_dir: str | Path,
        reuse_dir: str | Path | None = None, cfg: Settings | None = None,
        verbose: bool = True) -> dict:
    cfg = cfg or Settings()
    manifest_path = Path(manifest_path)
    base = manifest_path.parent
    manifest = json.loads(manifest_path.read_text())
    gates = {**DEFAULT_GATES, **manifest.get("gates", {})}

    out = Path(out_dir)
    cache = out / "captures"
    cache.mkdir(parents=True, exist_ok=True)

    per_capture: list[dict] = []
    results: dict[str, dict] = {}
    ci_samples: list = []

    for entry in manifest["captures"]:
        cid = entry["id"]
        cached = Path(reuse_dir) / cid / "results.json" if reuse_dir else None
        if cached and cached.exists():
            result = json.loads(cached.read_text())
        else:
            result = pipeline.run(_resolve(base, entry["path"]), cache / cid,
                                  cfg=cfg, tier=entry.get("tier"), verbose=False)
        results[cid] = result
        gt = groundtruth.load(_resolve(base, entry["ground_truth"]),
                              fallback_room_id=entry.get("room_id", "r1"))
        rc = evaluate_capture(cid, entry.get("room_id", "r1"), result, gt, gates, ci_samples)
        per_capture.append(rc)
        if verbose:
            print(f"[bench] {cid}: {'PASS' if rc['passed'] else 'FAIL'} {rc['checks']}")

    repeat: list[dict] = []
    for r in manifest.get("repeatability", []):
        ids = r["captures"]
        if all(i in results for i in ids):
            repeat.append(evaluate_repeatability(r["room_id"], results[ids[0]],
                                                 results[ids[1]], gates))

    n_ci = len(ci_samples)
    coverage = (sum(1 for _, gt_v, lo, hi in ci_samples if lo <= gt_v <= hi) / n_ci
                if n_ci else 0.0)
    passed_n = sum(1 for c in per_capture if c["passed"])
    summary = {
        "captures_total": len(per_capture),
        "captures_passed": passed_n,
        "pass_rate": round(passed_n / max(len(per_capture), 1), 3),
        "repeatability_passed": sum(1 for r in repeat if r["passed"]),
        "ci": {"n": n_ci, "coverage": round(coverage, 3),
               "meets_target": coverage >= gates["ci_coverage_min"] if n_ci else False},
    }

    (out / "gates.json").write_text(json.dumps(
        {"manifest": manifest_path.name, "gates": gates, "summary": summary,
         "per_capture": per_capture, "repeatability": repeat}, indent=2) + "\n")
    report.write(out, manifest_path.name, per_capture, repeat, summary, gates)

    if verbose:
        print(f"[bench] {passed_n}/{len(per_capture)} captures passed; "
              f"CI coverage {coverage*100:.0f}%. Wrote {out/'gates.json'}")
    return summary
