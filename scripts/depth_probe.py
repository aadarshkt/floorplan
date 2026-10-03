"""Score a monocular metric-depth model against LiDAR depth on a Record3D clip.

    .venv/bin/python scripts/depth_probe.py <record3d_dir> [--model ID] [--n 24] [--upright]

For a sample of keyframes it compares the model's depth with the LiDAR depth of
the same frame (high-confidence pixels only) and reports:

  scale_bias_pct   median over frames of (pred / lidar - 1): the model's metric bias
  scale_spread_pct std over frames of the log ratio: frame-to-frame scale wobble,
                   which is what smears walls when frames are fused unaligned
  absrel_frame     AbsRel after a per-frame scale fit (shape quality)
  absrel_global    AbsRel at one global scale

``--upright`` rotates each frame so gravity points down (using ARKit poses — for
this probe only) before inference, to measure what sideways frames cost.
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
from PIL import Image

from floorplan.config import Settings
from floorplan.fusion import metric_depth
from floorplan.fusion.depth_fusion import quat_to_matrix
from floorplan.ingest import record3d, video


def upright_k(R_c2w: np.ndarray) -> int:
    """np.rot90 k that turns the image so ARKit gravity points to image-down."""
    g = R_c2w.T @ np.array([0.0, -1.0, 0.0])      # gravity in ARKit camera axes
    down, right = -g[1], g[0]                       # ARKit cam: x right, y up
    if abs(down) >= abs(right):
        return 0 if down > 0 else 2
    return -1 if right > 0 else 1                   # right edge -> bottom: clockwise


def frames_scale(cap, frames, f) -> float:
    """Keyframe JPGs may be downscaled from the clip; intrinsics are at clip size."""
    with Image.open(frames / f"{f:06d}.jpg") as im:
        return im.size[0] / cap.rgb_size[0]


def depthpro(pipe, path, f_px):
    """DepthPro metric depth; with f_px given, use it instead of the model's FOV."""
    import torch
    model, proc = pipe
    im = Image.open(path).convert("RGB")
    inp = proc(images=im, return_tensors="pt").to(model.device, model.dtype)
    with torch.no_grad():
        out = model(**inp)
    W, H = im.size
    raw = torch.nn.functional.interpolate(out.predicted_depth[:, None].float(), size=(H, W),
                                          mode="bilinear", align_corners=False)[0, 0].cpu().numpy()
    if f_px is None:
        fov = float(out.field_of_view.float().cpu().numpy().ravel()[0])
        f_px = 0.5 * W / np.tan(0.5 * np.deg2rad(fov))
    return raw * W / f_px


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--model", default="depth-anything/Depth-Anything-V2-Metric-Indoor-Small-hf")
    ap.add_argument("--n", type=int, default=24)
    ap.add_argument("--max-dim", type=int, default=1036)
    ap.add_argument("--upright", action="store_true")
    ap.add_argument("--crop", type=float, default=1.0, help="keep this centre fraction of each side")
    ap.add_argument("--aspect", type=float, default=None,
                    help="crop the height to width/aspect (e.g. 1.78 -> 16:9 landscape)")
    ap.add_argument("--focal", default="model", choices=["model", "true"],
                    help="DepthPro only: metric conversion from its own FOV or the true focal")
    a = ap.parse_args()

    cap = record3d.load(a.capture)
    frames, idx = video.extract_frames(Path(a.capture) / "rgb.mp4", 2.0, mode="sharp")
    pick = [idx[i] for i in np.linspace(0, len(idx) - 1, min(a.n, len(idx))).astype(int)]

    from transformers import pipeline
    dev, dtype = metric_depth.pick_device(Settings())
    if "DepthPro" in a.model:
        import torch
        from transformers import DepthProForDepthEstimation, DepthProImageProcessor
        proc = DepthProImageProcessor.from_pretrained(a.model)
        model = DepthProForDepthEstimation.from_pretrained(a.model, torch_dtype=torch.float16).to(dev).eval()
        pipe = (model, proc)
    else:
        pipe = pipeline("depth-estimation", model=a.model, device=dev, dtype=dtype)

    ratios, ab_frame, per_frame, geoms = [], [], [], []
    t0 = time.time()
    for f in pick:
        lid = np.asarray(Image.open(cap.depth_paths[f]), dtype=np.float64) / 1000.0
        conf = np.asarray(Image.open(cap.conf_paths[f])) if cap.conf_paths else np.full(lid.shape, 2)
        k = upright_k(quat_to_matrix(cap.quat[f])) if a.upright else 0
        img = np.rot90(np.asarray(Image.open(frames / f"{f:06d}.jpg").convert("RGB")), k)
        lid, conf = np.rot90(lid, k), np.rot90(conf, k)
        H, W = img.shape[:2]
        ch, cw = a.crop, a.crop
        if a.aspect:
            ch = min(1.0, cw * W / a.aspect / H)
        y0, y1 = int(H * (1 - ch) / 2), int(H * (1 + ch) / 2)
        x0, x1 = int(W * (1 - cw) / 2), int(W * (1 + cw) / 2)
        img = img[y0:y1, x0:x1]
        lh, lw = lid.shape
        lid = lid[int(lh * (1 - ch) / 2):int(lh * (1 + ch) / 2), int(lw * (1 - cw) / 2):int(lw * (1 + cw) / 2)]
        conf = conf[int(lh * (1 - ch) / 2):int(lh * (1 + ch) / 2), int(lw * (1 - cw) / 2):int(lw * (1 + cw) / 2)]
        f_px = float(cap.intr[f][0]) * frames_scale(cap, frames, f)
        geom = (f_px, img.shape[1], img.shape[0])
        tmp = Path("/tmp") / f"_probe_{f}.jpg"
        Image.fromarray(np.ascontiguousarray(img)).save(tmp, quality=95)
        if "DepthPro" in a.model:
            d = depthpro(pipe, tmp, f_px if a.focal == "true" else None)
        else:
            d = metric_depth._predict(pipe, tmp, a.max_dim)
        dh, dw = lid.shape
        pred = np.asarray(Image.fromarray(d.astype(np.float32), mode="F").resize((dw, dh), Image.BILINEAR),
                          dtype=np.float64)
        m = (conf >= 2) & (lid > 0.3) & (lid < 5.0) & np.isfinite(pred) & (pred > 0.1)
        if m.sum() < 500:
            continue
        r = float(np.median(pred[m] / lid[m]))
        ratios.append(r)
        geoms.append(geom)
        ab_frame.append(float(np.median(np.abs(pred[m] / r - lid[m]) / lid[m])))
        per_frame.append((f, pred[m], lid[m]))
    dt = time.time() - t0

    ratios = np.asarray(ratios)
    g = float(np.median(ratios))
    ab_global = float(np.median(np.concatenate([np.abs(p / g - l) / l for _, p, l in per_frame])))
    row = {
        "capture": Path(a.capture).name, "model": a.model.split("/")[-1], "upright": a.upright,
        "frames": len(ratios),
        "scale_bias_pct": round((g - 1) * 100, 2),
        "scale_spread_pct": round(float(np.std(np.log(ratios))) * 100, 2),
        "absrel_frame_pct": round(float(np.median(ab_frame)) * 100, 2),
        "absrel_global_pct": round(ab_global * 100, 2),
        "s_per_frame": round(dt / max(len(ratios), 1), 2),
        "crop": a.crop, "aspect": a.aspect, "focal": a.focal,
        "f_over_w": round(float(np.median([q[0] / q[1] for q in geoms])), 3),
        "f_over_h": round(float(np.median([q[0] / q[2] for q in geoms])), 3),
        # implied canonical focal if bias = f_c / f  (relative to width and height)
        "fc_w": round(g * float(np.median([q[0] / q[1] for q in geoms])), 3),
        "fc_h": round(g * float(np.median([q[0] / q[2] for q in geoms])), 3),
    }
    print(json.dumps(row))
    with open("benchmark/reports/depth_probe.jsonl", "a") as fh:
        fh.write(json.dumps(row) + "\n")


if __name__ == "__main__":
    main()
