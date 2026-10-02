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
                repeatability: list[dict], summary: dict, gates: dict) -> str:
    lines: list[str] = []
    lines.append(f"# Benchmark report — {manifest_name}")
    lines.append("")
    lines.append(f"**Overall: {summary['captures_passed']}/{summary['captures_total']} captures "
                 f"passed all gates ({summary['pass_rate']*100:.0f}%)**")
    lines.append("")
    lines.append("## Per-capture gates")
    lines.append("")
    lines.append("| capture | room | walls max/mean (cm) | ceiling (cm) | openings pass | area (%) | result |")
    lines.append("|---|---|---|---|---|---|---|")
    for c in per_capture:
        wl = c["wall_length"]
        ce = c["ceiling_height"]
        op = c["openings"]
        ar = c["area"]
        lines.append(
            f"| {c['capture_id']} | {c['room_id']} | "
            f"{_fmt(wl['max_abs_cm'])} / {_fmt(wl['mean_abs_cm'])} | "
            f"{_fmt(ce['abs_cm'])}{'' if ce.get('observed', True) else ' (unobs)'} | "
            f"{_fmt((op['pass_rate'] or 0)*100, '', 0)}% | "
            f"{_fmt(ar['rel_pct'], '', 1)} | "
            f"{'PASS' if c['passed'] else 'FAIL'} |"
        )
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

    lines.append("## Gates")
    lines.append("")
    lines.append("| gate | threshold |")
    lines.append("|---|---|")
    for k, v in gates.items():
        lines.append(f"| {k} | {v} |")
    lines.append("")
    return "\n".join(lines)


def write(out_dir: str | Path, manifest_name: str, per_capture: list[dict],
          repeatability: list[dict], summary: dict, gates: dict) -> None:
    out = Path(out_dir)
    (out / "report.md").write_text(
        to_markdown(manifest_name, per_capture, repeatability, summary, gates)
    )
