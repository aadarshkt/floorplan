# Cozmo AI — Phone-Capture-to-Floor-Plan: Implementation Plan

> **Purpose of this document.** A single, self-contained build spec that another LLM/engineer
> session can execute without prior context. It decodes the assignment, defines every term,
> locks the architecture decisions already made, and lays out phases with acceptance criteria.

---

## 0. TL;DR

Build a **local, deterministic, offline command-line tool** that turns a phone capture
(**Record3D ZIP** = LiDAR tier, **photo folder** = photos tier, **walkthrough clip** = video tier)
into a **metric, stitched, multi-room 2D floor plan + JSON (published schema) + SVG/DXF**, with
confidence intervals on every measurement. Ship a **benchmark harness** (so accuracy is
provable and the fix loop is regenerable), a **compliance matrix**, a **head-to-head vs a
consumer app**, and a **fix-loop before/after bundle**.

One command per capture: `floorplan run <capture_path> --out <dir>`.

---

## 1. The assignment, decoded

**Product:** "Cozmo AI" case study. Own the problem from the phone's sensors → produce a
stitched, dimensioned multi-room floor plan + damage/scope report, from **three input tiers**
(photos, video, LiDAR), each with confidence intervals, **one command per capture**, **JSON to a
published schema**, and a **rendered plan**.

**Why it is graded this way (the scoring map):**

| Weight | Component | What actually earns it |
|---|---|---|
| **30%** | Walk-in test | Cold run on the grader's unseen iPhone capture, scored against their laser measurements. Robustness + speed + **all three tiers runnable**, not accuracy on our own data. |
| **25%** | Fix loop delta | A reproducible **before/after** + a **shipped** fix + honest prediction. No fix ⇒ 0. No regenerable runs ⇒ 0. |
| **15%** | Verified benchmark accuracy | Our own benchmark across all 3 tiers, ground truth, gates, repeatability. |
| **10%** | Compliance matrix | requirement → file path → artifact → status. |
| **10%** | Head-to-head vs incumbent | 2 rooms vs magicplan/Polycam, per-dimension table, beat/tie ≥70%. |
| **5%** | Capture route quality | One-page protocol + device matrix. |
| **5%** | Process evidence | Real, incremental commit history (not one commit at the deadline). |

**Key insight:** 55% is robustness/reproducibility (walk-in + fix loop), ~30% is bookkeeping
(compliance, head-to-head, route, history), and only 15% is raw accuracy. Optimize accordingly:
**a working vertical slice + honest instrumentation + reproducible artifacts** beats a fancy
pipeline.

**Deliverables (all 8):** (1) compliance matrix; (2) capture route (Record3D one-pager) + device
matrix; (3) repo, README→running in <15 min, one command/capture; (4) reproduction bundle;
(5) benchmark report (3 tiers, repeatability, head-to-head, timing); (6) fix-loop bundle;
(7) technical report ≤6 pages; (8) raw benchmark data.

---

## 2. Glossary (plain-English definitions of the terms used throughout)

- **Tier** — one of the three capture types: **LiDAR** (depth+pose), **photos** (2–8 stills/room), **video** (walkthrough clip).
- **Depth map** — per-pixel distance from camera to surface. Ours: 256×192, 16-bit PNG, **millimetres**.
- **Confidence map** — per-pixel trust in the depth. Ours: 256×192, 8-bit, values 0/1/2 (low/med/high). "Black" because values are tiny (0–2 on a 0–255 display).
- **Intrinsics (`fx,fy,cx,cy`)** — camera lens spec in pixels; needed to unproject a pixel+depth to 3D.
- **Odometry / pose** — camera position `(x,y,z)` + orientation quaternion `(qx,qy,qz,qw)` per frame; "where the camera was pointing".
- **IMU** — accelerometer + gyroscope; gives gravity (which way is down) and helps smooth poses.
- **Unprojection** — turning (pixel, depth) into a 3D point: `X=(u−cx)Z/fx, Y=(v−cy)Z/fy, Z=depth`.
- **Fusion** — merging many posed depth frames into one point cloud. **TSDF fusion** = average depths into a voxel volume (cleaner) rather than pasting points.
- **Point cloud** — millions of 3D surface dots (the raw reconstruction).
- **RANSAC** — "random sample consensus": guess a plane from 3 random points, count agreeing points, keep the best, peel it off, repeat.
- **Plane / wall vectorization** — fitting a flat surface, then reducing a wall to a 2D segment `(start,end,length)`.
- **Orthogonal snapping** — forcing near-90° walls to exact perpendicular axes (flag genuinely slanted walls; never silently "fix").
- **Opening** — a door/window detected as a gap in wall point density; width = gap length.
- **SfM (Structure from Motion)** — COLMAP stage that solves camera poses **and** 3D points from overlapping photos by triangulating matched features.
- **MVS (Multi-View Stereo)** — COLMAP stage that turns solved cameras into dense per-pixel depth → dense cloud.
- **Monocular (metric) depth** — a neural net predicting depth from a single image (some metric, some relative).
- **Scale anchor** — a known real length in the scene used to scale a unit-less reconstruction to metres.
- **Confidence interval (CI)** — an error bar; e.g. "4.12 m ± 0.03 m (95%)". Methods: error propagation + bootstrap + calibration.
- **ICP** — Iterative Closest Point; aligns two clouds by matching nearest points (puzzle pieces).
- **Pose-graph optimization** — joint solve of all capture poses to distribute accumulated error.
- **Loop closure** — recognizing a revisited place and adding a constraint that ties the loop shut.
- **Plane-anchored correction** — force continuous floors/ceilings/walls to be shared between rooms, removing drift.
- **Drift** — accumulated pose error over a multi-room walk (walking in the dark counting steps).
- **Ablation** — running with a feature ON vs OFF and showing the difference (the gate demands drift-correction on/off).
- **Adjacency** — which rooms touch, through which opening.
- **Footprint** — the outer outline of the whole property.
- **Ground truth** — real laser/tape measurements used to score accuracy.
- **Gate** — a pass/fail accuracy threshold (see §7).
- **Repeatability** — same room captured twice must agree within 1 cm / 0.5%.
- **Head-to-head** — same rooms run through a consumer app (magicplan / Polycam), compared dimension-by-dimension.
- **Calibration** — (a) scale calibration (anchoring photogrammetry to metres); (b) interval calibration (making error bars honest).
- **Fix loop (25%)** — find worst gate → declare root cause + fix + predicted number → ship fix → submit before+after runs + diff.
- **Fix loop delta** — the before→after movement of the failing number (e.g. openings 60%→88%).
- **Compliance matrix** — a table mapping every requirement to the file/artifact that satisfies it + status.
- **Device matrix** — which tier runs on which hardware and the honest accuracy it delivers.
- **Vertical slice** — one thin but *complete* input→output path (one tier) before widening.

---

## 3. The data (what we actually have)

Three sample captures (each a `capture_id/` folder), all **LiDAR tier**, format = **Record3D export**:

```
<id>/
  rgb.mp4              # 1920x1440 @60fps, N frames (1715 for single_room)
  depth/000000.png     # 256x192, uint16, millimetres
  confidence/000000.png# 256x192, uint8, {0,1,2}
  odometry.csv         # timestamp,frame,x,y,z,qx,qy,qz,qw,fx,fy,cx,cy
  imu.csv              # timestamp,a_x,a_y,a_z,alpha_x,alpha_y,alpha_z
  camera_matrix.csv    # 3x3 RGB intrinsics (fx=fy≈1599.7, cx≈955.5, cy≈717.8)
```

- `single_room` (1715 frames), `single_scan_with_ceiling` (9745) — have ceilings.
- `single_scan_floor_only` (5251) — **ceiling not captured** → must infer/flag honestly.
- Depth intrinsics = RGB intrinsics × (256/1920) = ×0.1333.
- Three **different** room ids ⇒ three different rooms (not repeats).

**Decision:** the graders will capture with **Record3D** (App Store, Route 2). Our LiDAR ingest
must accept Record3D's export ZIP/folder. The prior `3DReconstruction` repo used RoomPlan JSON
for its LiDAR tier and therefore does **not** ingest this data — but its geometry/export code is
reusable (see §11).

---

## 4. Locked decisions

1. **Capture route = Route 2, stock app = Record3D.** One-page protocol + device matrix. No custom iOS app.
2. **LiDAR tier = depth+pose fusion** (Record3D → TSDF/point cloud).
3. **Photo/video tiers = monocular depth primary, COLMAP when available.** No learned pointmap model (no DUSt3R).
4. **Priority = vertical slice + fix loop first.** Damage detection is **deferred to v1+** (schema slots kept, benchmark staged so it can plug in).
5. **Local, deterministic, offline CLI.** No Postgres/Celery/cloud from the prior repo.
6. Hardware available to the author: **LiDAR iPhone Pro + laser/tape** ⇒ full benchmark is buildable.

---

## 5. Architecture

```
                       ┌─────────────────────────────┐
  Record3D ZIP ─────┐  │  INGEST (tier autodetect)    │
  photo folder ─────┼─▶│  record3d | photos | video   │
  video clip ───────┘  └──────────────┬──────────────┘
                                      ▼
        ┌──────────────────────────────────────────────────┐
        │ FUSION  (per tier → common point cloud)           │
        │  lidar : unproject+pose → TSDF/accumulate         │
        │  photos: monodepth (default) | COLMAP (if present)│
        │  video : ffmpeg frames → same as photos           │
        │  ARBITER: score paths, pick/fuse, log provenance  │
        └──────────────┬───────────────────────────────────┘
                       ▼  COMMON IR: {metric Z-up cloud, per-pt conf,
                       │              scale_confidence, provenance}
        ┌──────────────┴───────────────────────────────────┐
        │ GEOMETRY   planes → walls → openings → room       │
        │            polygon/area, ceiling height           │
        │ CONFIDENCE bootstrap + propagation + calibration  │
        └──────────────┬───────────────────────────────────┘
                       ▼
        ┌──────────────┴───────────────┐   ┌──────────────────────┐
        │ MULTI-ROOM (only multi-capture)│  │ DAMAGE (v1+: slots)  │
        │ ICP + pose-graph + plane-anchor│  │ classify + extent    │
        │ adjacency, no-overlap, ablation│  └──────────┬───────────┘
        └──────────────┬────────────────┘             │
                       ▼                               ▼
        ┌─────────────────────────────────────────────────────┐
        │ EXPORT   results.json (schema) + floor_plan.svg + .dxf│
        └─────────────────────────────────────────────────────┘

  EVAL (separate): bench harness → gates table, repeatability,
                   head-to-head, fix-loop before/after.
```

**Common IR (the seam that makes tiers swappable):**
`{ points: Nx3 float (metres, canonical Z-up), conf: Nx1 optional, scale_confidence: "native_metric"|"scaled_via_reference", reference_used, tier, provenance:{path_chosen, params} }`.

**Up-axis:** ARKit world = Y-up; convert `(x,y,z)→(x,−z,y)` to Z-up. Document once, apply everywhere.

---

## 6. Per-tier pipelines

### 6.1 LiDAR (backbone; we have data)
1. Ingest Record3D (ZIP or dir); read `odometry.csv`, `camera_matrix.csv`, depth + confidence PNGs.
2. Depth mm→m; scale intrinsics ×0.1333; keep pixels with confidence ≥ threshold (default: keep 2, drop 0; configurable).
3. Unproject per frame → camera points → world via `R(q)·p + t`.
4. **Fusion:** Open3D `ScalableTSDFVolume` (voxel ~1 cm, trunc ~4 cm) → extract cloud. (Fallback: accumulate + voxel downsample + statistical outlier removal.)
5. `scale_confidence = native_metric`.

### 6.2 Photos
- If `colmap` on PATH: SfM → poses; MVS → dense cloud; **scale anchor** via known length (door height 2.05 m / taped metre / printed ArUco). `scale_confidence = scaled_via_reference`.
- Else: monocular metric-depth model (Depth Anything V2 / Depth Pro / Metric3D v2) → per-image cloud; align relative models by median-scale to the anchor.

### 6.3 Video
- `ffmpeg -i clip -vf fps=3` (or blur-aware keyframe select) → frames → same as photos.

### 6.4 Arbiter
- When both paths available: score by (plane-fit residual, coverage, plausibility: room closes/wall count/ceiling present); pick winner or ICP-merge with confidence weighting. **Always record which path won** (feeds report + fix loop).

---

## 7. Geometry pipeline (tier-agnostic)

1. Clean cloud (statistical outlier removal + voxel downsample).
2. RANSAC plane peeling (dist thresh 1–2 cm); classify: `|nz|≥0.8` horizontal, `|nz|≤0.7` vertical.
3. Floor = lowest large horizontal; ceiling = highest; **ceiling_height = median(ceiling_z) − median(floor_z)**. If no ceiling: infer from top of walls (95th pct) else learned constant 2.5 m + `ceiling_observed:false` + wide CI.
4. Wall vectorization: inliers in band [floor+0.5, floor+1.5] → PCA line fit → segment. **Merge opposite faces of the same wall into one centerline** (avoid double-count).
5. Orthogonal snap; flag non-rectangular rooms.
6. **Openings:** 5 cm bins along wall over band [floor, floor+2.1]; empty runs ≥0.5 m = opening; floor-reaching = door, mid-height = window; suppress phantom gaps (require no points behind), recover missed (floor continuity). ← hardest gate.
7. Room polygon (angle-order segments) → shoelace area.
8. **Confidence intervals:** bootstrap (200× refit) + propagation; store per measurement.

**Gates (from the brief):**

| Metric | Gate |
|---|---|
| Opening widths | ≤2 cm on ≥85%; missed **and** phantom each = miss |
| Ceiling height | ≤1.5 cm/room; across repeated captures ≤1 cm |
| Repeatability | 2 captures of same room agree ≤1 cm or 0.5%/wall |
| Drift accountability | report method + **ablation on/off** footprint; "poses as-is" = fail |
| Photo-tier whole-property stitch | correct adjacency, no overlaps, footprint ±8% w/ calibrated CIs |
| Photo walls | ±8% w/ calibrated intervals |
| Video walls | ±3% w/ calibrated intervals |

---

## 8. Multi-room stitch + drift

- Input: multiple room captures (LiDAR poses; or per-room photo folders).
- **ICP** align clouds; **pose-graph** joint solve; **loop closure** when a room is revisited; **plane-anchored** correction (share continuous floor/ceiling/wall planes).
- Build **adjacency graph** (rooms touch via opening ids); enforce **no-overlap** constraints.
- Emit **ablation table**: footprint with correction ON vs OFF.
- Photo-tier multi-room is the explicitly-hardest gate → protects against "single-room-only" photo path.

---

## 9. Output contract & published JSON schema

`floorplan run <capture> --out <dir>` writes: `results.json`, `floor_plan.svg`, `floor_plan.dxf`,
`scan_metric.ply`, `provenance.json`, `timing.json`.

`results.json` (v1.0, units = metres; every measurement is `{value, ci95:[lo,hi], method}`):

```jsonc
{
  "schema_version": "1.0",
  "capture_id": "...", "tier": "lidar|photos|video", "generated_at": "...",
  "property": {
    "footprint_area_m2": {"value":0,"ci95":[0,0]},
    "rooms": ["r1","r2"], "adjacency": [{"a":"r1","b":"r2","via":"o4"}]
  },
  "rooms": [{
    "room_id":"r1", "polygon":[[x,y],...],
    "floor_area_m2": {"value":0,"ci95":[0,0]},
    "ceiling_height_m": {"value":0,"ci95":[0,0],"observed":true},
    "walls": [{"wall_id":"r1w1","start":[x,y],"end":[x,y],"length_m":{"value":0,"ci95":[0,0]},
               "openings":[{"opening_id":"o1","type":"door|window","position_along_wall_m":0,"width_m":{"value":0,"ci95":[0,0]}}]}],
    "damage_regions": [],           // v1+: {class, surface_id, extent_m2, polygon}
    "concealed_damage_flags": [],   // v1+: {rule, surfaces, rationale}
    "scope_line_items": []          // v1+: {item, surface_id, quantity, unit}
  }],
  "drift": {"method":"pose_graph+plane_anchor","ablation":{"on":0,"off":0}},
  "confidence": {"calibration_factor":1.0,"method":"bootstrap+propagation"},
  "artifacts": {"svg":"floor_plan.svg","dxf":"floor_plan.dxf","pointcloud":"scan_metric.ply"}
}
```

`schema/floorplan.schema.json` = the published JSON Schema. Damage/scope arrays exist and are
**structurally compliant but empty** in v1 (honest "not detected in v1").

---

## 10. Benchmark, evaluation, head-to-head, fix loop

### 10.1 Benchmark composition (required)
```
benchmark/
  manifest.yaml
  captures/  multiroom_L1/ multiroom_L2/ ...      # Record3D ZIPs (3+ rooms + connector)
  photos/    kitchen/ (2-8 jpg), living/ ...
  video/     walkthrough.mov (same rooms)
  repeat/    room_a_run1/ room_a_run2/            # same room, same tier
  staged_damage/ room_d/                          # v1+: 2 damage classes
  ground_truth/ *.csv          # entity_type,entity_id,dimension,value_m,unit,notes
  app_exports/ magicplan_*.json polycam_*.ply     # head-to-head evidence
```

### 10.2 Harness (`floorplan bench`)
Loads `manifest.yaml` + ground truth, runs each capture, computes **per-dimension error**, checks
**gates**, builds **repeatability table**, and writes `benchmark_report.md` + machine-readable
`gates.json`. Deterministic (fixed seeds).

### 10.3 Head-to-head
Run 2 benchmark rooms through **magicplan** and/or **Polycam** (free tier); save their export;
table our error vs theirs per shared dimension; report beat/tie %.

### 10.4 Fix loop (25%)
1. `floorplan bench` → worst failing gate + exact number.
2. One-page `fix_declaration.md`: worst gate+number; root cause+evidence; fix+**predicted** number.
3. Ship the code fix (real commit).
4. `fix_loop/` = `before/` + `after/` runs (regenerable) + `diff.patch` (readable).
Scoring: right cause + fix + pass = full; improvement-but-short = majority (explain why);
badly-wrong prediction = honesty only; no fix = 0; no reproducible runs = 0.

---

## 11. Reuse from the prior `3DReconstruction` repo

Reusable (copy + refactor, don't import the heavy stack):
- `backend/app/pipeline/common_backend.py` — RANSAC plane extraction, wall vectorization, orthogonal snap, opening detection, room area.
- `backend/app/pipeline/exporter.py` — DXF (ezdxf) + SVG (svgwrite) + JSON output.
- `floor_plan_pipeline_implementation.md` — geometry rationale, failure-mode catalogue.

Do **not** reuse: Postgres/Celery/Redis, auth_service, terraform, iOS app, the RoomPlan `tier_c`
(our LiDAR ingest is Record3D depth+pose, a new module). Keep the new tool a **local offline CLI**.

---

## 12. Repo layout (new, fresh)

```
floorplan/
  README.md                     # 15-min quickstart, one command per capture
  IMPLEMENTATION_PLAN.md        # this document
  compliance_matrix.md
  pyproject.toml                # uv-managed, py>=3.11
  floorplan/
    cli.py                      # floorplan run|bench|gates|headtohead
    config.py
    ingest/{detect,record3d,photos,video}.py
    fusion/{depth_fusion,monodepth,colmap_path,arbiter}.py
    geometry/{planes,walls,openings,room,confidence}.py
    multiroom/{register,pose_graph,adjacency}.py
    damage/{regions,classify}.py          # v1+: stubs
    export/{schema,json_export,svg,dxf}.py
    eval/{groundtruth,gates,repeatability,headtohead,report}.py
  schema/floorplan.schema.json
  data/                                   # gitignored big binaries; fetched/extracted by script
  benchmark/                              # manifest + captures + ground truth + app exports
  fix_loop/{before,after,diff.patch}
  scripts/{fetch_weights.sh,extract_samples.sh}
  tests/
```

**One command per capture:** `floorplan run <path>` autodetects tier (Record3D folder/ZIP → lidar;
folder of 2–8 images → photos; single .mp4/.mov → video).

**README requirement:** running on a fresh capture in **<15 min on a clean machine**; document
`uv sync`, optional `brew install colmap ffmpeg`, model-weight fetch by script.

---

## 13. Phased task plan (build order)

### P0 — End-to-end LiDAR vertical slice (highest priority; carries 30% walk-in LiDAR)
- [ ] Scaffold repo + `uv` env + deps (numpy, scipy, open3d, pillow, ezdxf, svgwrite, structlog).
- [ ] `ingest/record3d.py` (+ `detect.py`): parse ZIP/dir → frames, poses, intrinsics, confidence.
- [ ] `fusion/depth_fusion.py`: unproject+pose → TSDF/accumulate → canonical Z-up metric cloud.
- [ ] `geometry/{planes,walls,openings,room,confidence}.py`: planes → walls → openings → polygon/area, ceiling height, bootstrap CIs.
- [ ] `export/{json_export,schema,svg,dxf}.py`: results.json + published schema + SVG + DXF.
- [ ] `cli.py`: `floorplan run <capture>` end-to-end, deterministic, writes all artifacts.
- [ ] Validate on the 3 provided scans (`single_room`, `floor_only`, `with_ceiling`); eyeball SVGs; confirm ceiling-observed flags.
- **Acceptance:** one command on a Record3D capture yields a closed room polygon with walls, openings, area, ceiling height + CIs, JSON+SVG+DXF, byte-identical across reruns.

### P1 — Multi-room stitch + drift ablation (protects 30% + a named gate)
- [ ] `multiroom/{register,pose_graph,adjacency}.py`: ICP + pose-graph + plane-anchored; adjacency; no-overlap.
- [ ] Emit ON/OFF footprint **ablation table**.
- **Acceptance:** a 3-room + connector LiDAR capture stitches into one plan with correct adjacency and no overlaps; ablation table present.

### P1 — Benchmark harness + ground truth (prerequisite for 25% + 15%)
- [ ] `eval/{groundtruth,gates,repeatability,report}.py` + `floorplan bench`.
- [ ] Author `benchmark/manifest.yaml`; capture the required set with the iPhone Pro + laser/tape.
- [ ] Publish `benchmark_report.md` + `gates.json`.
- **Acceptance:** `floorplan bench` reproduces every reported number from raw inputs.

### P2 — Photos + video tiers (needed for "all three tiers ready")
- [ ] `fusion/monodepth.py` (depth model + scale anchor) and `fusion/colmap_path.py`.
- [ ] `ingest/{photos,video}.py`; hole-punching into the same IR; arbiter + provenance.
- [ ] Photo multi-room stitch (per-room folders).
- **Acceptance:** photo (±8%) and video (±3%) tiers run cold and hit/widen intervals honestly.

### P2 — Damage + scope (v1+ stretch; deferred by decision)
- [ ] `damage/*`: per-surface regions + class + metric extent; concealed-damage rules; scope line items keyed to surfaces.
- **Acceptance:** staged-damage room yields ≥2 classes with metric extents.

### P3 — Reports, compliance, head-to-head, fix loop, capture route
- [ ] `compliance_matrix.md` (requirement→file→artifact→status).
- [ ] `headtohead`: 2 rooms vs magicplan/Polycam; save app exports; per-dimension table.
- [ ] One-page Record3D capture protocol + `device_matrix.md`.
- [ ] Fix loop: bench → worst gate → declaration → ship fix → `fix_loop/{before,after,diff.patch}`.
- [ ] Technical report ≤6 pages; reproduction bundle; raw data committed.
- [ ] **Process evidence:** small, real commits throughout (from 2 Oct 20:00 onward).

---

## 14. Risks & mitigations

| Risk | Mitigation |
|---|---|
| COLMAP cold-install/slow in a live run | Monocular depth is the default path; COLMAP optional; pre-warm models; document install. |
| Opening gate (≤2 cm on 85%, no miss/phantom) | Best fix-loop candidate; instrument early; confidence-filtered depth helps LiDAR. |
| Photo multi-room stitch (±8%, no overlap) | Use LiDAR (same rooms) as cross-tier reference + anchors; multi-reference scale. |
| `floor_only` ceiling height | Infer from wall tops; else wide CI + explicit `observed:false`; capture protocol says "pan up". |
| Mirrors/glass/wet surfaces/low light | Confidence filtering; exclude low-confidence depth; document coverage + honest intervals. |
| Damage/scope unscoped | Deferred to v1+; schema slots reserved; benchmark room staged so it can plug in. |
| Walk-in "all three tiers ready" | Build photo/video paths as thin but working before polish; one command each. |
| "Confident garbage" capping score | Interval calibration on benchmark; widen when evidence is thin. |

---

## 15. Verification plan

- **Unit:** unprojection (synthetic depth+pose → known point), plane/opening detection on synthetic rooms, shoelace area, quaternion→R.
- **Golden:** all 3 provided scans run in CI; artifacts regenerate byte-identically (determinism test).
- **Benchmark:** `floorplan bench` prints gates table; every number in the report is regenerable.
- **Live rehearsal:** capture a fresh room with Record3D → `floorplan run` cold → compare to tape, in one sitting.
- **Repeatability:** same room twice → outputs within 1 cm / 0.5%.

---

## 16. Open questions / assumptions

- **Photos count vs gates:** "2–8 stills/room" is very sparse for COLMAP; the monocular path is expected to carry the photo tier. Confirm the grader's photo-tier protocol allows a reference object for scale.
- **Incumbent app choice:** magicplan vs Polycam (free tier) — decide at P3 based on export availability.
- **Reference object:** standardise on a printed ArUco marker of known size + door-height fallback.
- **Multi-room photo input:** confirm the grader supplies per-room photo folders (as stated).
- **Damage classes:** define the two classes (e.g. water stain + crack) before staging for v1+.

---

## 17. Review-note resolutions

- **Naming.** The tool is de-branded from `cozmo` to **`floorplan`** (package, CLI,
  repo directory). Run command: `floorplan run <capture> --out <dir>`.
- **Photo folder with no scale reference.** The photos path is unit-less without a
  reference. Resolution: `--scale-ref <metres>` (plus `--scale-ref-kind`) lets the
  operator supply a known length (door width 0.82 m / door height 2.05 m / taped
  metre / printed ArUco). If omitted, we fall back to a **default prior** and mark
  the result `scale_reference: "none_prior"` with **widened confidence intervals**;
  a photo plan with no reference is explicitly labelled non-metric. If a Record3D
  capture of the same room exists, its metric depth is used automatically as the
  scale reference (cross-tier anchor).
- **Photo-only fusion.** Tier autodetection routes a photo folder to the **photos**
  path (monocular metric depth by default), never to the LiDAR path. Engine is
  configurable via `--engine auto|monodepth|colmap`:
  `auto` → COLMAP when ≥ `min_photos_for_colmap` images **and** `colmap` is on
  PATH, otherwise monocular depth. A sparse set (2–8 stills) therefore uses
  monocular depth; a rich set can opt into COLMAP. When a Record3D capture of the
  same room is supplied, its depth is used as reference geometry.
- **`provenance.json` vs `timing.json`.** `provenance.json` = reproducibility
  metadata: tool version, settings snapshot, tier, which code path won
  (`path_chosen`), input identity hash, and `generated_at`. Timings are recorded
  per stage (`ingest`, `fusion`, `geometry`, `confidence`, …) inside provenance and
  drive the benchmark timing table and the <15-min guarantee. Neither belongs in
  `results.json`, which stays deterministic.
- **Fix loop — how it is implemented.**
  1. `floorplan bench --manifest benchmark/manifest.yaml --out reports/before`
     → `gates.json` + gate table.
  2. Choose the worst failing gate; write `fix_declaration.md` (gate + failing
     number, root-cause hypothesis + evidence, fix + predicted number).
  3. Ship the code fix (a real commit).
  4. `floorplan bench … --out reports/after`; `git diff <before>..HEAD >
     fix_loop/diff.patch`. Both `before/` and `after/` are committed → regenerable.
     **Delta = after − before per gate.** (No fix ⇒ 0; no regenerable runs ⇒ 0.)

## 18. Build status

**P0 — LiDAR vertical slice: DONE.**
- Record3D ingest; deterministic depth+pose fusion (Z-up metres, confidence-filtered);
  RANSAC planes; wall vectorisation + double-wall merge + orthogonal snap; opening
  detection; density-based floor/ceiling with an honest `observed` flag; bootstrap
  confidence intervals; JSON (schema-validated) + SVG + DXF + provenance; one-command CLI.
- Validated on all three provided scans (schema-valid, byte-identical across reruns;
  6/6 unit tests). `single_room` and `single_scan_floor_only` ceilings are reported
  `observed:false` (not densely captured) — honestly widened intervals.

**Next (P1→P3):** multi-room stitch + drift ablation; benchmark harness + ground
truth; photos/video tiers; damage/scope; reports, compliance matrix, head-to-head,
capture protocol, fix-loop bundle.
