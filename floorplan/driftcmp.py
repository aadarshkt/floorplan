"""`floorplan drift-ablate`: run one capture with drift correction on and off.

Writes <out>/on, <out>/off (full pipeline outputs), drift_ablation.json,
drift_ablation.md and drift_overlay.svg (both footprints on one sheet).
"""
from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import svgwrite

from floorplan import pipeline
from floorplan.config import Settings


def _summary(res: dict) -> dict:
    pts = np.concatenate([np.asarray(r["polygon"], float) for r in res["rooms"]])
    ext = pts.max(axis=0) - pts.min(axis=0)
    walls = [w["length_m"]["value"] for r in res["rooms"] for w in r["walls"]]
    return {"footprint_m2": res["property"]["footprint_area_m2"]["value"],
            "rooms": len(res["rooms"]),
            "bbox_m": [round(float(ext[0]), 3), round(float(ext[1]), 3)],
            "walls": len(walls), "total_wall_m": round(float(sum(walls)), 3),
            "drift": res["drift"]}


def _overlay(path: Path, on: dict, off: dict) -> None:
    allp = np.concatenate([np.asarray(r["polygon"], float)
                           for res in (on, off) for r in res["rooms"]])
    lo, hi = allp.min(axis=0), allp.max(axis=0)
    s, pad = 80.0, 40.0
    w, h = (hi - lo) * s + 2 * pad
    dwg = svgwrite.Drawing(str(path), size=(f"{w:.0f}px", f"{h:.0f}px"))
    dwg.add(dwg.rect((0, 0), (w, h), fill="white"))

    def xy(p):
        return ((p[0] - lo[0]) * s + pad, (hi[1] - p[1]) * s + pad)   # y up

    for res, color, dash in ((off, "#d62728", "6,4"), (on, "#1f77b4", None)):
        for r in res["rooms"]:
            kw = {"stroke_dasharray": dash} if dash else {}
            dwg.add(dwg.polygon([xy(p) for p in r["polygon"]], fill="none",
                                stroke=color, stroke_width=2, **kw))
    dwg.add(dwg.text("blue: drift correction ON   red dashed: poses as-is (OFF)", insert=(pad, 22),
                     font_size="14px", font_family="sans-serif"))
    dwg.save()


def run(capture: str, out_dir: str, cfg: Settings, verbose: bool = True) -> dict:
    out = Path(out_dir)
    res = {}
    for name, flag in (("off", False), ("on", True)):
        if verbose:
            print(f"[drift-ablate] drift correction {name.upper()}")
        res[name] = pipeline.run(capture, out / name, cfg=replace(cfg, drift_correction=flag),
                                 verbose=verbose)
    s_on, s_off = _summary(res["on"]), _summary(res["off"])
    delta = {"footprint_m2": round(s_off["footprint_m2"] - s_on["footprint_m2"], 3),
             "footprint_rel_pct": round(100 * (s_off["footprint_m2"] - s_on["footprint_m2"])
                                        / max(s_on["footprint_m2"], 1e-9), 2),
             "bbox_m": [round(a - b, 3) for a, b in zip(s_off["bbox_m"], s_on["bbox_m"])],
             "total_wall_m": round(s_off["total_wall_m"] - s_on["total_wall_m"], 3)}
    report = {"capture": str(capture), "on": s_on, "off": s_off, "delta_off_minus_on": delta}
    (out / "drift_ablation.json").write_text(json.dumps(report, indent=2) + "\n")
    _overlay(out / "drift_overlay.svg", res["on"], res["off"])

    d = s_on["drift"]
    lines = [f"# Drift ablation: {Path(capture).name}", "",
             f"Method (ON): {d['method']}; verified loops: {d['n_loops']}", "",
             "| | ON | OFF (poses as-is) | OFF - ON |", "|---|---|---|---|",
             f"| footprint m2 | {s_on['footprint_m2']} | {s_off['footprint_m2']} | "
             f"{delta['footprint_m2']} ({delta['footprint_rel_pct']}%) |",
             f"| bbox (m) | {s_on['bbox_m']} | {s_off['bbox_m']} | {delta['bbox_m']} |",
             f"| rooms | {s_on['rooms']} | {s_off['rooms']} | |",
             f"| total wall length m | {s_on['total_wall_m']} | {s_off['total_wall_m']} | "
             f"{delta['total_wall_m']} |"]
    if d.get("loop_error_m"):
        le = d["loop_error_m"]
        lines += ["", f"Loop closure error (median / max, m): before {le['before_median']} / "
                      f"{le['before_max']}, after {le['after_median']} / {le['after_max']}; "
                      f"max pose correction {d['max_correction_m']} m"]
    (out / "drift_ablation.md").write_text("\n".join(lines) + "\n")
    if verbose:
        print("\n".join(lines))
        print(f"[drift-ablate] wrote {out}/drift_ablation.(json|md) and drift_overlay.svg")
    return report
