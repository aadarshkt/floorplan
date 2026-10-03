"""Room layout from free space (shared by all tiers).

The earlier tail measured walls as the extent of RANSAC plane inliers and area as
the convex hull of floor points. Those disagree with each other (an inlier set
runs along a whole wall line, through doorways and into furniture; a hull spans
floor seen through doors), so lengths, polygon and area could not all be right.

Here one polygon carries every number:

  1. occupancy  points in a horizontal band near the ceiling (above furniture),
                or 1.3-2.0 m above the floor when the ceiling was not seen, on a
                grid aligned to the room's dominant wall direction
  2. free space flood fill from the camera positions (the cameras were inside
                the room) through cells that were observed and are not wall
  3. one room   a morphological opening cuts leaks through doorways; the
                component holding most cameras is the room; holes (furniture
                hiding the floor) are filled
  4. polygon    the region's outline is rectilinear in the aligned frame;
                consecutive collinear runs merge, tiny jogs are dropped
  5. refine     each edge moves to the median position of the wall points just
                behind it, so lengths are interior face-to-face (what a laser
                measures), not grid-quantised

Returns polygons in world XY, counter-clockwise.
"""
from __future__ import annotations

import math

import numpy as np
from scipy import ndimage

from floorplan.config import Settings


def _rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _band(points: np.ndarray, floor_z: float, ceil_z: float, observed: bool,
          cell: float = 0.02) -> np.ndarray:
    """Wall evidence: vertical-surface points above furniture height.

    Points whose normal is horizontal, from 0.9 m above the floor to just below
    the ceiling (or the top of the cloud when the ceiling was not seen). Most
    furniture (beds, tables, sofas) is lower; walls run the full height.
    """
    import open3d as o3d
    z = points[:, 2]
    top = ceil_z - 0.08 if observed else float(np.percentile(z, 99.5))
    sel = points[(z >= floor_z + 0.9) & (z <= top)]
    if len(sel) < 50:
        return sel
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(sel)).voxel_down_sample(cell)
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=4 * cell, max_nn=20))
    q = np.asarray(pcd.points)
    nrm = np.asarray(pcd.normals)
    keep = np.abs(nrm[:, 2]) < 0.3
    global _LAST_NORMALS
    _LAST_NORMALS = nrm[keep]
    return q[keep]


_LAST_NORMALS: np.ndarray | None = None


def dominant_angle(normals: np.ndarray) -> float | None:
    """Wall direction mod 90 deg: 4-fold circular mean of horizontal normal angles."""
    if normals is None or len(normals) < 50:
        return None
    phi = np.arctan2(normals[:, 1], normals[:, 0])
    m = np.exp(4j * phi).mean()
    if abs(m) < 0.05:
        return None
    return float(np.angle(m) / 4.0)


def _seal(occ: np.ndarray, max_gap: int, min_run: int) -> np.ndarray:
    """Fill row gaps <= max_gap cells lying between two occupied runs >= min_run."""
    out = occ.copy()
    for r in range(occ.shape[0]):
        row = occ[r]
        if row.sum() < 2 * min_run:
            continue
        d = np.diff(np.concatenate([[0], row.astype(np.int8), [0]]))
        starts, ends = np.nonzero(d == 1)[0], np.nonzero(d == -1)[0]   # runs [s, e)
        for k in range(len(starts) - 1):
            gap = starts[k + 1] - ends[k]
            if (gap <= max_gap and ends[k] - starts[k] >= min_run
                    and ends[k + 1] - starts[k + 1] >= min_run):
                out[r, ends[k]:starts[k + 1]] = True
    return out


def _trace(region: np.ndarray) -> list[tuple[int, int]]:
    """Outer boundary of a binary region as grid-corner vertices (CCW in (i, j))."""
    H, W = region.shape
    pad = np.zeros((H + 2, W + 2), dtype=bool)
    pad[1:-1, 1:-1] = region
    # directed boundary edges with the region on the left (corner coordinates)
    nxt: dict[tuple[int, int], list[tuple[int, int]]] = {}

    def add(a, b):
        nxt.setdefault(a, []).append(b)

    ii, jj = np.nonzero(pad)
    for i, j in zip(ii, jj):
        if not pad[i - 1, j]:
            add((i, j + 1), (i, j))
        if not pad[i + 1, j]:
            add((i + 1, j), (i + 1, j + 1))
        if not pad[i, j - 1]:
            add((i, j), (i + 1, j))
        if not pad[i, j + 1]:
            add((i + 1, j + 1), (i, j + 1))
    if not nxt:
        return []
    # walk every loop; keep the longest (the outer boundary after hole filling)
    best: list[tuple[int, int]] = []
    unused = {a: list(bs) for a, bs in nxt.items()}
    while unused:
        start = next(iter(unused))
        loop, cur = [start], start
        while True:
            outs = unused.get(cur)
            if not outs:
                break
            b = outs.pop()
            if not outs:
                del unused[cur]
            if b == start:
                break
            loop.append(b)
            cur = b
        if len(loop) > len(best):
            best = loop
    return [(i - 1, j - 1) for i, j in best]


def _rectilinear(verts: np.ndarray, min_edge: float) -> np.ndarray:
    """Merge collinear runs and drop jogs shorter than min_edge (axis-aligned input)."""
    v = verts
    for _ in range(len(verts)):
        n = len(v)
        if n < 4:
            return v
        # drop collinear vertices
        keep = []
        for k in range(n):
            a, b, c = v[k - 1], v[k], v[(k + 1) % n]
            if abs((b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])) > 1e-9:
                keep.append(k)
        v = v[keep]
        n = len(v)
        if n < 4:
            return v
        seg = np.linalg.norm(np.roll(v, -1, axis=0) - v, axis=1)
        k = int(np.argmin(seg))
        if seg[k] >= min_edge:
            return v
        # remove a short edge k (v[k] -> v[k+1]): collapse it onto its neighbours
        a, b = v[k], v[(k + 1) % n]
        p, q = v[k - 1], v[(k + 2) % n]
        horiz = abs(b[1] - a[1]) < 1e-9          # short edge runs along x
        if horiz:   # neighbours are vertical: put both at the length-weighted x
            lp, lq = abs(a[1] - p[1]), abs(q[1] - b[1])
            x = (a[0] * lp + b[0] * lq) / max(lp + lq, 1e-9)
            v[k - 1] = [x, p[1]]
            v[(k + 2) % n] = [x, q[1]]
        else:
            lp, lq = abs(a[0] - p[0]), abs(q[0] - b[0])
            y = (a[1] * lp + b[1] * lq) / max(lp + lq, 1e-9)
            v[k - 1] = [p[0], y]
            v[(k + 2) % n] = [q[0], y]
        v = np.delete(v, [k, (k + 1) % n], axis=0)
    return v


def _refine(v: np.ndarray, band_xy: np.ndarray, search: float) -> np.ndarray:
    """Move each axis-aligned edge to the median of the wall points just outside it."""
    n = len(v)
    v = v.copy()
    pos = []
    for k in range(n):
        a, b = v[k], v[(k + 1) % n]
        horiz = abs(b[1] - a[1]) < 1e-9
        ax, cross = (0, 1) if horiz else (1, 0)
        lo, hi = sorted([a[ax], b[ax]])
        c = a[cross]
        m = ((band_xy[:, ax] > lo + 0.1) & (band_xy[:, ax] < hi - 0.1)
             & (np.abs(band_xy[:, cross] - c) < search))
        pos.append(float(np.median(band_xy[m, cross])) if m.sum() >= 30 else c)
    for k in range(n):
        a_k = k
        b_k = (k + 1) % n
        horiz = abs(v[b_k][1] - v[a_k][1]) < 1e-9
        cross = 1 if horiz else 0
        v[a_k][cross] = pos[k]
        v[b_k][cross] = pos[k]
    return v


def rooms(points: np.ndarray, floor_z: float, ceil_z: float, observed: bool,
          cams_xy: np.ndarray | None, theta: float, cfg: Settings) -> list[np.ndarray]:
    band = _band(points, floor_z, ceil_z, observed)
    if len(band) < 200:
        return []
    th = dominant_angle(_LAST_NORMALS)
    theta = th if th is not None else theta
    R = _rot(-theta)
    bxy = band[:, :2] @ R.T
    allxy = points[:, :2] @ R.T
    cell = cfg.layout_cell
    # grid over the bulk of the cloud: a few points seen through a window can
    # lie tens of metres out and would blow up the grid
    lo = np.percentile(allxy, 0.5, axis=0) - 0.5
    hi = np.percentile(allxy, 99.5, axis=0) + 0.5
    inside = np.all((allxy >= lo) & (allxy <= hi), axis=1)
    allxy = allxy[inside]
    bxy = bxy[np.all((bxy >= lo) & (bxy <= hi), axis=1)]
    shape = tuple(np.ceil((hi - lo) / cell).astype(int) + 1)
    if shape[0] * shape[1] > 25_000_000:
        return []

    def cells(xy):
        ij = np.floor((xy - lo) / cell).astype(int)
        return ij[:, 0], ij[:, 1]

    occ = np.zeros(shape, dtype=np.int32)
    np.add.at(occ, cells(bxy), 1)
    wall = occ >= cfg.layout_min_pts
    wall = ndimage.binary_dilation(wall, iterations=1)
    # seal doorways: in the aligned grid a wall runs along a row or column; a gap
    # up to door width between two long wall runs on that line is filled
    L = max(3, int(cfg.layout_door_close_m / cell))
    min_run = max(3, int(cfg.layout_seal_min_run_m / cell))
    wall = _seal(wall, L, min_run) | _seal(wall.T, L, min_run).T

    seen = np.zeros(shape, dtype=bool)
    seen[cells(allxy)] = True
    seen = ndimage.binary_dilation(seen, iterations=max(1, int(0.10 / cell)))
    # seeds: camera positions (or the free cell nearest the cloud centre)
    if cams_xy is not None and len(cams_xy):
        ci, cj = cells(cams_xy @ R.T)
        ok = (ci >= 0) & (ci < shape[0]) & (cj >= 0) & (cj < shape[1])
        ci, cj = ci[ok], cj[ok]
    else:
        ci, cj = cells(np.median(allxy, axis=0, keepdims=True))
    # the sensor rarely sees the floor right around the phone: count enclosed
    # unobserved holes and the neighbourhood of the camera path as observed
    near = np.zeros(shape, dtype=bool)
    near[ci, cj] = True
    near = ndimage.binary_dilation(near, iterations=max(1, int(0.6 / cell)))
    seen = ndimage.binary_fill_holes(seen | near)
    free = seen & ~ndimage.binary_dilation(wall, iterations=1)
    lab, _ = ndimage.label(free)
    # a camera cell can sit on wall evidence (dilation, sealing): use the
    # nearest free cell instead
    _, (ni, nj) = ndimage.distance_transform_edt(lab == 0, return_indices=True)
    ci, cj = ni[ci, cj], nj[ci, cj]
    seed_labels = {int(lab[i, j]) for i, j in zip(ci, cj) if lab[i, j] > 0}
    if not seed_labels:
        return []
    region = np.isin(lab, list(seed_labels))

    # cut leaks through doorways / gaps, keep the components holding cameras
    r_open = max(1, int(cfg.layout_open_m / cell))
    st = ndimage.generate_binary_structure(2, 1)
    opened = ndimage.binary_opening(region, structure=st, iterations=r_open)
    lab2, n2 = ndimage.label(opened)
    out = []
    if n2 == 0:
        return []
    counts = np.bincount(lab2[ci, cj][lab2[ci, cj] > 0], minlength=n2 + 1)
    order = np.argsort(-counts)
    for lid in order:
        if lid == 0 or counts[lid] == 0:
            continue
        comp = lab2 == lid
        # restore the area the opening shaved off along walls, inside the region
        comp = ndimage.binary_dilation(comp, structure=st, iterations=r_open) & region
        comp = ndimage.binary_fill_holes(comp)
        r_s = max(1, int(cfg.layout_smooth_m / cell))
        comp = ndimage.binary_closing(comp, structure=st, iterations=r_s)
        comp = ndimage.binary_opening(comp, structure=st, iterations=r_s)
        area = comp.sum() * cell * cell
        if area < cfg.min_room_area:
            continue
        verts = np.asarray(_trace(comp), dtype=float) * cell + lo
        if len(verts) < 4:
            continue
        verts = _rectilinear(verts, cfg.layout_min_edge)
        if len(verts) < 4:
            continue
        verts = _refine(verts, bxy, cfg.layout_refine_m)
        verts = verts @ R                              # back to world XY
        x, y = verts[:, 0], verts[:, 1]
        if np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)) < 0:
            verts = verts[::-1]
        out.append(verts)
        if not cfg.layout_multi_room:
            break
    return out
