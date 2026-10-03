"""Cross-tier evaluation: score a video-tier run against the LiDAR run of the same clip.

A Record3D export carries the RGB clip *and* LiDAR depth + ARKit poses for every
frame. Running the video tier on ``rgb.mp4`` alone and the LiDAR tier on the full
export gives a dense metric reference for every stage of the video pipeline,
without a tape measure:

  registration   share of keyframes that SfM placed at all
  trajectory     camera centres vs ARKit centres after a similarity alignment
                 (ATE, cm) -> are the poses right?
  scale          the similarity's scale -> is the "metric" cloud really metres?
  up axis        tilt between the two output frames' +Z -> is gravity right?
  cloud          accuracy (video points -> LiDAR surface) and completeness
                 (LiDAR surface covered by video points) after alignment
  measurements   results.json of both runs: walls, ceiling, area

The LiDAR run is a reference, not ground truth: its numbers carry its own error,
so measurement deltas are "agreement with LiDAR", reported as such. The
alignment is used only for scoring; nothing flows back into the video tier.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from floorplan.eval import metrics


def umeyama(src: np.ndarray, dst: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Similarity (s, R, t) minimising |dst - (s R src + t)|^2."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    cov = xd.T @ xs / len(src)
    U, D, Vt = np.linalg.svd(cov)
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (xs ** 2).sum() / len(src)
    s = float(np.trace(np.diag(D) @ S) / var_s) if var_s > 1e-12 else 1.0
    t = mu_d - s * R @ mu_s
    return s, R, t


def robust_sim3(src: np.ndarray, dst: np.ndarray, iters: int = 5
                ) -> tuple[float, np.ndarray, np.ndarray, np.ndarray]:
    """Umeyama with trimming of gross outliers (mis-registered cameras)."""
    keep = np.ones(len(src), dtype=bool)
    for _ in range(iters):
        s, R, t = umeyama(src[keep], dst[keep])
        res = np.linalg.norm(dst - (s * src @ R.T + t), axis=1)
        thr = max(3.0 * float(np.median(res[keep])), 0.05)
        new = res <= thr
        if new.sum() < 6 or (new == keep).all():
            break
        keep = new
    return s, R, t, keep


def _load_ply(path: Path) -> np.ndarray:
    import open3d as o3d
    return np.asarray(o3d.io.read_point_cloud(str(path)).points)


def _subsample(pts: np.ndarray, n: int) -> np.ndarray:
    if len(pts) <= n:
        return pts
    return pts[np.linspace(0, len(pts) - 1, n).astype(int)]


def _n_keyframes(video_run: Path) -> int | None:
    imgs = video_run / "colmap" / "imgs"
    return len(list(imgs.iterdir())) if imgs.is_dir() else None


def _room_summary(result: dict) -> dict:
    rooms = result.get("rooms", [])
    walls = [w["length_m"]["value"] for r in rooms for w in r.get("walls", [])]
    wci = [w["length_m"].get("ci95") for r in rooms for w in r.get("walls", [])]
    r0 = rooms[0] if rooms else {}
    return {
        "n_rooms": len(rooms),
        "walls": walls, "wall_ci": wci,
        "ceiling": (r0.get("ceiling_height_m") or {}).get("value"),
        "ceiling_ci": (r0.get("ceiling_height_m") or {}).get("ci95"),
        "ceiling_observed": (r0.get("ceiling_height_m") or {}).get("observed"),
        "area": sum((r.get("floor_area_m2") or {}).get("value", 0.0) for r in rooms),
        "n_openings": sum(len(w.get("openings", [])) for r in rooms for w in r.get("walls", [])),
    }


def evaluate(lidar_run: str | Path, video_run: str | Path,
             tol_m: float = 0.05, max_pts: int = 200_000) -> dict:
    lidar_run, video_run = Path(lidar_run), Path(video_run)
    out: dict = {"lidar_run": str(lidar_run), "video_run": str(video_run)}

    lc = json.loads((lidar_run / "cameras.json").read_text())
    vpath = video_run / "cameras.json"
    n_key = _n_keyframes(video_run)
    if not vpath.exists():
        out["note"] = "no camera poses recovered (single-image fallback): measurements only"
        out["registration"] = {"keyframes": n_key, "registered": 0, "rate": 0.0}
        out["measurements"] = _measurements(lidar_run, video_run)
        return out
    vc = json.loads(vpath.read_text())

    lcent = np.asarray(lc["centre"], dtype=float)
    vframes = np.asarray(vc["frame"], dtype=int)
    vcent = np.asarray(vc["centre"], dtype=float)
    ok = (vframes >= 0) & (vframes < len(lcent))
    vframes, vcent = vframes[ok], vcent[ok]
    ref = lcent[vframes]
    out["registration"] = {"keyframes": n_key, "registered": int(len(vframes)),
                           "rate": round(len(vframes) / n_key, 3) if n_key else None}
    if len(vframes) < 6:
        out["error"] = "too few registered cameras to align"
        return out

    s, R, t, keep = robust_sim3(vcent, ref)
    res = np.linalg.norm(ref - (s * vcent @ R.T + t), axis=1)
    # trajectory extent, to put the ATE in proportion
    extent = float(np.linalg.norm(ref.max(0) - ref.min(0)))
    out["trajectory"] = {
        "ate_rmse_cm": round(float(np.sqrt(np.mean(res[keep] ** 2))) * 100, 2),
        "ate_median_cm": round(float(np.median(res)) * 100, 2),
        "outlier_cameras": int((~keep).sum()),
        "extent_m": round(extent, 2),
    }
    # video units * s = metres. scale error > 0: video cloud is too large.
    out["scale"] = {"sim3_scale": round(s, 4), "error_pct": round((1.0 / s - 1.0) * 100, 2)}
    zv = R @ np.array([0.0, 0.0, 1.0])
    out["up_tilt_deg"] = round(float(np.degrees(np.arccos(np.clip(zv[2], -1, 1)))), 2)

    # cloud accuracy / completeness after the similarity alignment
    from scipy.spatial import cKDTree
    lp = _subsample(_load_ply(lidar_run / "scan_metric.ply"), max_pts)
    vp = _subsample(_load_ply(video_run / "scan_metric.ply"), max_pts)
    vp_al = s * vp @ R.T + t
    d_acc, _ = cKDTree(lp).query(vp_al, k=1)
    d_comp, _ = cKDTree(vp_al).query(lp, k=1)
    out["cloud"] = {
        "accuracy_median_cm": round(float(np.median(d_acc)) * 100, 2),
        "accuracy_p90_cm": round(float(np.percentile(d_acc, 90)) * 100, 2),
        "precision_pct": round(float(np.mean(d_acc <= tol_m)) * 100, 1),
        "completeness_pct": round(float(np.mean(d_comp <= tol_m)) * 100, 1),
        "tol_cm": tol_m * 100,
    }

    out["measurements"] = _measurements(lidar_run, video_run)
    return out


def _measurements(lidar_run: Path, video_run: Path) -> dict:
    """Agreement of the two runs' results.json (LiDAR run as reference)."""
    lr = json.loads((lidar_run / "results.json").read_text())
    vr = json.loads((video_run / "results.json").read_text())
    L, V = _room_summary(lr), _room_summary(vr)
    wl = metrics.wall_length_errors(V["walls"], L["walls"])
    rel = [abs(V["walls"][i] - L["walls"][j]) / L["walls"][j] * 100 for i, j in wl["pairs"]]
    covered = [1 if (V["wall_ci"][i] and V["wall_ci"][i][0] <= L["walls"][j] <= V["wall_ci"][i][1]) else 0
               for i, j in wl["pairs"]]
    return {
        "lidar": {k: L[k] for k in ("n_rooms", "walls", "ceiling", "ceiling_observed", "area", "n_openings")},
        "video": {k: V[k] for k in ("n_rooms", "walls", "ceiling", "ceiling_observed", "area", "n_openings")},
        "wall_count_delta": len(V["walls"]) - len(L["walls"]),
        "wall_mean_abs_cm": wl["mean_abs_cm"],
        "wall_mean_rel_pct": round(float(np.mean(rel)), 2) if rel else None,
        "wall_within_3pct": round(float(np.mean([r <= 3.0 for r in rel])), 3) if rel else None,
        "wall_ci_covers_lidar": round(float(np.mean(covered)), 3) if covered else None,
        "ceiling_abs_cm": (round(abs(V["ceiling"] - L["ceiling"]) * 100, 1)
                           if V["ceiling"] is not None and L["ceiling"] is not None else None),
        "area_rel_pct": (round(abs(V["area"] - L["area"]) / L["area"] * 100, 1)
                         if L["area"] else None),
    }


def to_markdown(rows: list[dict]) -> str:
    def g(d, *ks):
        for k in ks:
            if d is None:
                return "–"
            d = d.get(k)
        return "–" if d is None else d

    lines = [
        "| capture | reg | ATE cm | scale err % | tilt ° | acc med cm | prec % | compl % | walls V/L | wall rel % | ceil Δ cm | area Δ % |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        m = r.get("measurements") or {}
        reg = r.get("registration") or {}
        lines.append(
            f"| {r['id']} | {reg.get('registered', '–')}/{reg.get('keyframes', '–')} "
            f"| {g(r, 'trajectory', 'ate_rmse_cm')} | {g(r, 'scale', 'error_pct')} "
            f"| {r.get('up_tilt_deg', '–')} | {g(r, 'cloud', 'accuracy_median_cm')} "
            f"| {g(r, 'cloud', 'precision_pct')} | {g(r, 'cloud', 'completeness_pct')} "
            f"| {len(g(m, 'video', 'walls') or []) if m else '–'}/{len(g(m, 'lidar', 'walls') or []) if m else '–'} "
            f"| {m.get('wall_mean_rel_pct', '–')} | {m.get('ceiling_abs_cm', '–')} | {m.get('area_rel_pct', '–')} |")
    return "\n".join(lines) + "\n"
