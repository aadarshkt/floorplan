"""Render the benchmark report (markdown) from evaluated capture results."""
from __future__ import annotations

from pathlib import Path


def _fmt(v, unit="", nd=2):
    if v is None:
        return "—"
    if isinstance(v, float):
        return f"{v:.{nd}f}{unit}"
    return f"{v}{unit}"


def to_markdown(manifest_name: str, per_capture: list[dict],
                repeatability: list[dict], summary: dict, gates: dict,
                ablations: list[dict] | None = None) -> str:
    lines: list[str] = []
    lines.append(f"# Benchmark report — {manifest_name}")
    lines.append("")
    lines.append(f"**Overall: {summary['captures_passed']}/{summary['captures_total']} captures "
                 f"passed all gates ({summary['pass_rate']*100:.0f}%)**")
    lines.append("")
    acc = summary.get("accuracy")
    if acc:
        lines.append(f"**Accuracy: {acc['accuracy']*100:.0f}%** "
                     f"({acc['within']}/{acc['total']} measurements within tolerance)")
    lines.append("")
    lines.append("## Per-capture gates")
    lines.append("")
    lines.append("| capture | room | walls max/mean (cm) | walls missed/phantom | ceiling (cm) | openings pass (missed/phantom) | area (%) | result |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for c in per_capture:
        wl = c["wall_length"]
        ce = c["ceiling_height"]
        op = c["openings"]
        ar = c["area"]
        lines.append(
            f"| {c['capture_id']} | {c['room_id']} | "
            f"{_fmt(wl['max_abs_cm'])} / {_fmt(wl['mean_abs_cm'])} | "
            f"{wl.get('missed', '—')}/{wl.get('phantom', '—')} | "
            f"{_fmt(ce['abs_cm'])}{'' if ce.get('observed', True) else ' (unobs)'} | "
            f"{_fmt((op['pass_rate'] or 0)*100, '', 0)}% ({op['missed']}/{op['phantom']}) | "
            f"{_fmt(ar['rel_pct'], '', 1)} | "
            f"{'PASS' if c['passed'] else 'FAIL'} |"
        )
    lines.append("")

    lines.append("## Per-wall errors")
    lines.append("")
    lines.append("| capture | room | wall (plan) | measured as | error (cm) |")
    lines.append("|---|---|---|---|---|")
    for c in per_capture:
        wl = c["wall_length"]
        for (pid, gid), e in zip(wl.get("pair_ids", []), wl.get("errors_cm", [])):
            lines.append(f"| {c['capture_id']} | {c['room_id']} | {pid} | {gid or 'by length'} | {e:.1f} |")
    lines.append("")

    if repeatability:
        lines.append("## Repeatability (same room, two captures)")
        lines.append("")
        lines.append("| room | max wall (cm) | mean wall (cm) | ceiling (cm) | area (%) | result |")
        lines.append("|---|---|---|---|---|---|")
        for r in repeatability:
            lines.append(
                f"| {r['room_id']} | {_fmt(r['max_wall_cm'])} | {_fmt(r['mean_wall_cm'])} | "
                f"{_fmt(r['ceiling_cm'])} | {_fmt(r['area_rel_pct'], '', 3)} | "
                f"{'PASS' if r['passed'] else 'FAIL'} |"
            )
        lines.append("")

    lines.append("## Confidence-interval calibration")
    lines.append("")
    lines.append(f"- samples: {summary['ci']['n']}, coverage: "
                 f"{summary['ci']['coverage']*100:.0f}% (target ≥ {gates['ci_coverage_min']*100:.0f}%)")
    lines.append("")

    if ablations:
        lines.append("## Drift ablation (correction ON vs OFF; delta = off − on, positive = ON is better)")
        lines.append("")
        lines.append("| capture | room | gate | ON | OFF | delta |")
        lines.append("|---|---|---|---|---|---|")
        for a in ablations:
            for key, nd in (("max_wall_cm", 2), ("ceiling_cm", 2), ("area_rel_pct", 3)):
                lines.append(
                    f"| {a['capture_id']} | {a['room_id']} | {key} | "
                    f"{_fmt(a['on'][key], '', nd)} | {_fmt(a['off'][key], '', nd)} | "
                    f"{_fmt(a['delta'][key], '', nd)} |"
                )
        lines.append("")

    lines.append("## Gates")
    lines.append("")
    lines.append("| gate | threshold |")
    lines.append("|---|---|")
    for k, v in gates.items():
        lines.append(f"| {k} | {v} |")
    lines.append("")
    return "\n".join(lines)


def write(out_dir: str | Path, manifest_name: str, per_capture: list[dict],
          repeatability: list[dict], summary: dict, gates: dict,
          ablations: list[dict] | None = None) -> None:
    out = Path(out_dir)
    (out / "report.md").write_text(
        to_markdown(manifest_name, per_capture, repeatability, summary, gates, ablations)
    )
