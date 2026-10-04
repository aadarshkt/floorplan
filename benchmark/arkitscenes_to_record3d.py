"""Convert an ARKitScenes ``raw`` capture into the Record3D/Stray Scanner folder the LiDAR tier reads.

ARKitScenes (https://github.com/apple/ARKitScenes) ships iPhone/iPad LiDAR captures
with a Faro laser scan of the same room. Its ``raw`` assets differ from a Record3D
export only in packaging:

    ARKitScenes                         ->  Record3D folder (floorplan LiDAR tier)
    lowres_depth/<vid>_<ts>.png  uint16 mm  depth/NNNNNN.png      (copied as is)
    confidence/<vid>_<ts>.png    0/1/2      confidence/NNNNNN.png (copied as is)
    lowres_wide.traj  10 Hz, world->camera  odometry.csv          (per depth frame,
                      axis-angle + t, Z-up                         interpolated, c2w)
    lowres_wide_intrinsics/*.pincam         fx,fy,cx,cy columns + camera_matrix.csv

Poses are written camera-to-world with OpenCV camera axes (``pose_convention.txt``
= ``c2w``) in a y-up world, because fusion maps ARKit y-up to its Z-up frame.
Depth frames outside the trajectory's time span are dropped. No rgb.mp4 is
written (the LiDAR tier does not need it).

    .venv/bin/python benchmark/arkitscenes_to_record3d.py <raw/Training/47333462> [-o out_dir]
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

# ARKitScenes world is Z-up; fusion expects ARKit y-up and maps (x, y, z) -> (x, -z, y).
ZUP_TO_YUP = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]])


def _ts(path: Path) -> float:
    return float(path.stem.rsplit("_", 1)[1])


def _read_traj(path: Path) -> tuple[np.ndarray, Rotation, np.ndarray]:
    """Return (ts, R_c2w, t_c2w) from a world->camera axis-angle trajectory."""
    a = np.loadtxt(path, ndmin=2)
    r_w2c = Rotation.from_rotvec(a[:, 1:4])
    r_c2w = r_w2c.inv()
    t_c2w = -r_c2w.apply(a[:, 4:7])
    return a[:, 0], r_c2w, t_c2w


def convert(src: Path, out: Path) -> Path:
    depth = sorted((src / "lowres_depth").glob("*.png"), key=_ts)
    conf = {round(_ts(p), 3): p for p in (src / "confidence").glob("*.png")}
    pincams = sorted((src / "lowres_wide_intrinsics").glob("*.pincam"), key=_ts)
    if not depth or not pincams:
        raise FileNotFoundError(f"{src}: needs lowres_depth/ and lowres_wide_intrinsics/")
    tt, r_c2w, t_c2w = _read_traj(src / "lowres_wide.traj")
    slerp = Slerp(tt, r_c2w)
    pin_ts = np.array([_ts(p) for p in pincams])

    depth = [p for p in depth if tt[0] <= _ts(p) <= tt[-1]]
    if not depth:
        raise ValueError(f"{src}: no depth frame inside the trajectory time span")
    ts = np.array([_ts(p) for p in depth])
    R = ZUP_TO_YUP @ slerp(ts).as_matrix()
    t = np.stack([np.interp(ts, tt, t_c2w[:, k]) for k in range(3)], axis=1) @ ZUP_TO_YUP.T
    q = Rotation.from_matrix(R).as_quat()  # qx, qy, qz, qw

    if out.exists():
        shutil.rmtree(out)
    (out / "depth").mkdir(parents=True)
    (out / "confidence").mkdir()
    rows, size = [], None
    for i, p in enumerate(depth):
        shutil.copyfile(p, out / "depth" / f"{i:06d}.png")
        c = conf.get(round(ts[i], 3))
        if c is not None:
            shutil.copyfile(c, out / "confidence" / f"{i:06d}.png")
        w, h, fx, fy, cx, cy = np.loadtxt(pincams[int(np.abs(pin_ts - ts[i]).argmin())])
        size = (int(w), int(h))
        rows.append(f"{ts[i]}, {i:06d}, {t[i,0]}, {t[i,1]}, {t[i,2]}, "
                    f"{q[i,0]}, {q[i,1]}, {q[i,2]}, {q[i,3]}, {fx}, {fy}, {cx}, {cy}")
    (out / "odometry.csv").write_text(
        "timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy\n" + "\n".join(rows) + "\n")
    (out / "camera_matrix.csv").write_text(f"{fx}, 0.0, {cx}\n0.0, {fy}, {cy}\n0.0, 0.0, 1.0\n")
    (out / "pose_convention.txt").write_text("c2w\n")
    # Intrinsics are given at the 256x192 lowres_wide size, which is also the depth size.
    (out / "source.json").write_text(json.dumps(
        {"source": str(src.resolve()), "dataset": "ARKitScenes raw", "frames": len(depth),
         "rgb_size": list(size), "missing_confidence": len(depth) - len(list((out / "confidence").iterdir()))},
        indent=1))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="ARKitScenes raw capture -> Record3D dataset folder.")
    ap.add_argument("input", type=Path, help="raw/<split>/<video_id> folder")
    ap.add_argument("-o", "--out", type=Path, default=None, help="output dir (default: <input>_r3d/)")
    a = ap.parse_args()
    out = convert(a.input, a.out or a.input.with_name(a.input.name + "_r3d"))
    print(f"DONE folder={out}")


if __name__ == "__main__":
    main()
