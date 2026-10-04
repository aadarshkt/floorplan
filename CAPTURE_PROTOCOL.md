# Capture protocol (Route 2: stock app, Stray Scanner)

One page. Follow it literally; no engineering knowledge needed.

**App:** *Stray Scanner* (free, App Store). **Needs an iPhone Pro with LiDAR** (12 Pro or newer Pro/Pro Max).
Install time: about 2 minutes.

## Device matrix

| Tier | Input | Hardware | What we deliver today | Honest accuracy (tape truth, 4 rooms) |
|---|---|---|---|---|
| LiDAR | Stray Scanner `.zip` | iPhone Pro with LiDAR | Full plan: walls, ceiling, area, openings, CIs | Worst wall 5.6 to 62 cm; ceiling 0.9 to 3.5 cm; wall gate **not** met. See `BENCHMARK_REPORT.md` |
| Video | Any `.mp4` / `.mov` walkthrough | iPhone 15 or newer (any model) | Plan with CIs; needs one known length (ceiling height) for scale | Walls 0.9 to 2.1 m off, 0 of 4 rooms pass. **Not usable for sizing yet** |
| Photos | Folder of stills | iPhone 15 or newer | **Not delivered** (a folder is handled like a sparse video; no per-room stitching) | Not benchmarked |

## LiDAR capture (do this)

1. Open Stray Scanner. Tap the red record button. Stand in the doorway.
2. **Walk slowly** (about one step per second). Hold the phone **upright, steady, chest height**.
3. Point the phone at: every wall, the floor corners, **the ceiling once** (tilt up and sweep), and **look through each doorway** so the door is seen from both sides.
4. Take 60 to 120 seconds per room. For several rooms, walk through the doors in one continuous take and **return to where you started** (this lets drift correction work).
5. Tap stop. Open the recording, tap **share**, choose *Save to Files*, and zip the folder (long-press, *Compress*). AirDrop or email the `.zip`.
6. Measure with a laser or tape: ceiling height, each wall (wall to wall), each door/window width. Write them down for the comparison page.

**Avoid:** mirrors and large glass (LiDAR sees through or doubles them), wet or glossy floors, direct sun on the sensor, dark rooms, fast turns (more than a quarter turn per second), people walking through, covering the camera with fingers.

## Video capture (no LiDAR)

Use the native Camera app, 4K or 1080p, 30 fps. Same walking rules as above, slower. Say the ceiling height aloud into the video only if you know it; you must also give it to the command (`--scale-ref`), because video alone has no real-world scale.

## Run

```bash
floorplan run capture.zip --out out/                                   # LiDAR
floorplan run walk.mov --out out/ --scale-ref 2.60 --scale-ref-kind ceiling_height   # video
```

Output: `out/floor_plan.svg` (open in a browser), `out/results.json`.
