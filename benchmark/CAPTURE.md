# Capture guide — Record3D (.r3d) and unpacked folders

Source of truth for re-running the `.r3d` -> dataset-folder conversion
(`.r3d` and unpacked exports live in `captures/`).

## 1. Capture (iPhone, Record3D app)

1. Record3D -> `Record` tab -> rear LiDAR mode -> scan the room -> Save.
2. `Library` -> swipe on the scan -> `Export` -> **`Shareable/Internal format (.r3d)`**.
   Do not use FBX / OBJ / PLY / glTF (fused mesh, loses per-frame depth+pose) or
   RGBD mp4 / preview exports (visualization only).
3. Copy the `.r3d` here via iOS `Files app > Record3D`, Share/AirDrop, or iTunes
   File Sharing. Double-clicking it on macOS shows "no application" — expected,
   do not open it; it is processed below.

Current capture: `2026-10-03--00-39-56.r3d` (1810 frames, RGB 720x960 @60fps,
depth 192x256, LiDAR).

## 2. Unpack (this folder)

Requires: python (>=3.10), `numpy pillow lzfse`, `ffmpeg` on PATH.

```bash
uv pip install --python ../.venv/bin/python numpy pillow lzfse
../.venv/bin/python unpack_r3d.py captures/2026-10-03--00-39-56.r3d
# custom output: ../.venv/bin/python unpack_r3d.py <in.r3d> -o captures/<out_dir>
```

Output (`captures/<stem>/`, same layout as `assignment/Assignment/c00a170fe1/`):

```
<stem>/rgb.mp4                  # h264, yuv420p, same fps as metadata
<stem>/depth/%06d.png           # uint16 millimeters (0 = missing/NaN)
<stem>/confidence/%06d.png      # uint8: 0 low, 1 medium, 2 high
<stem>/camera_matrix.csv        # 3x3 from metadata K
<stem>/odometry.csv             # timestamp,frame,x,y,z,qx,qy,qz,qw,fx,fy,cx,cy,,
<stem>/imu.csv                  # zeros placeholder
<stem>.zip                      # zipped copy of <stem>/
```

How it works: `.r3d` is a ZIP (`sound.m4a`, `icon`, `metadata` JSON,
`rgbd/<i>.jpg/.depth/.conf`). `.depth` (float32 meters) and `.conf` (uint8)
are LZFSE-compressed (`lzfse.decompress` on the whole file, magic `bvx2`
included); depth is x1000 to mm. Poses in `metadata` are
`[qx,qy,qz,qw,tx,ty,tz]`; intrinsics per frame are `[fx,fy,cx,cy]`.

## 3. Known differences vs the assignment reference set

- Portrait 720x960 here vs landscape 1920x1440 in the reference — capture
  quality setting, not a conversion bug.
- `odometry.csv` timestamps start at 0 (`.r3d frameTimestamps` are relative).
- `imu.csv` is all zeros: `.r3d` carries no IMU stream (verified: no IMU entry
  in the zip or `metadata`).
