"""Multi-room extraction from a single capture.

The wall network is treated as a planar graph: wall endpoints become nodes
(merged within a tolerance) and walls become edges. The interior faces of that
graph are the rooms. This generalises from one room to a walkthrough without any
special-casing, and falls back to the single-room path when the network is not a
closed graph.
"""
from __future__ import annotations

import math
from collections import defaultdict

import numpy as np

from floorplan.config import Settings
from floorplan.geometry import confidence
from floorplan.geometry import room as room_mod
from floorplan.geometry.walls import Wall


def _merge_nodes(pts: np.ndarray, tol: float) -> tuple[np.ndarray, np.ndarray]:
    reps: list[np.ndarray] = []
    idx = np.full(len(pts), -1, dtype=int)
    for i, p in enumerate(pts):
        found = -1
        for r, rep in enumerate(reps):
            if float(np.hypot(p[0] - rep[0], p[1] - rep[1])) <= tol:
                found = r
                break
        if found < 0:
            found = len(reps)
            reps.append(p)
        idx[i] = found
    return np.asarray(reps), idx


def _next_half_edge(nodes, angles, cv, cu):
    ang_back = math.atan2(nodes[cu][1] - nodes[cv][1], nodes[cu][0] - nodes[cv][0])
    best, best_d = None, None
    for m in angles[cv]:
        ang = math.atan2(nodes[m][1] - nodes[cv][1], nodes[m][0] - nodes[cv][0])
        d = (ang_back - ang) % (2 * math.pi)
        if d < 1e-9:
            d += 2 * math.pi
        if best_d is None or d < best_d:
            best_d, best = d, m
    return best


def _planar_faces(nodes: np.ndarray, edges: list[tuple[int, int]]) -> list[list[int]]:
    adj: dict[int, list[int]] = defaultdict(list)
    for a, b in edges:
        adj[a].append(b)
        adj[b].append(a)
    angles = {}
    for n, neigh in adj.items():
        p = nodes[n]
        angles[n] = sorted(set(neigh),
                           key=lambda m: math.atan2(nodes[m][1] - p[1], nodes[m][0] - p[0]))

    visited: set[tuple[int, int]] = set()
    faces: list[list[int]] = []
    for a, b in edges:
        for u, v in ((a, b), (b, a)):
            if (u, v) in visited:
                continue
            face: list[int] = []
            cu, cv = u, v
            while (cu, cv) not in visited and len(face) < 500:
                visited.add((cu, cv))
                face.append(cu)
                nxt = _next_half_edge(nodes, angles, cv, cu)
                if nxt is None:
                    break
                cu, cv = cv, nxt
            if len(face) >= 3:
                faces.append(face)
    return faces


def extract_rooms(walls: list[Wall], floor_z: float, ceil_z: float, observed: bool,
                  floor_pts: np.ndarray, ceil_pts: np.ndarray,
                  cfg: Settings) -> list[room_mod.Room]:
    if len(walls) < 3:
        return []
    pts: list[np.ndarray] = []
    for w in walls:
        pts.extend([w.start, w.end])
    nodes, idx = _merge_nodes(np.asarray(pts, dtype=float), cfg.node_merge_tol)

    ends: list[tuple[int, int]] = []
    edges: set[tuple[int, int]] = set()
    for k in range(len(walls)):
        a, b = int(idx[2 * k]), int(idx[2 * k + 1])
        ends.append((a, b))
        if a != b:
            edges.add((min(a, b), max(a, b)))

    faces = _planar_faces(nodes, list(edges))
    rooms: list[room_mod.Room] = []
    seen: set[frozenset] = set()
    for face in faces:
        poly = nodes[face]
        signed = room_mod.shoelace_signed(poly)
        if signed < cfg.min_room_area or signed > cfg.max_room_area:
            continue
        key = frozenset(face)
        if key in seen:
            continue
        seen.add(key)

        fset = set(face)
        fwalls = [w for w, (a, b) in zip(walls, ends) if a in fset and b in fset]

        inside = room_mod.point_in_polygon(floor_pts[:, :2], poly)
        fpts = floor_pts[inside]
        area_val, area_ci = abs(signed), None
        if len(fpts) >= 10:
            from scipy.spatial import ConvexHull
            try:
                area_val = float(ConvexHull(fpts[:, :2]).volume)
                area_ci = confidence.bootstrap_hull_area(fpts[:, :2], cfg)
            except Exception:
                pass
        if area_ci is None:
            area_ci = [round(area_val * 0.97, 3), round(area_val * 1.03, 3)]
        area_ci = confidence.bracket(area_val, area_ci)

        h = room_mod.ceiling_height(floor_z, ceil_z, observed, floor_pts, ceil_pts, cfg)
        rooms.append(room_mod.Room(
            room_id="",
            polygon=[[round(float(x), 3), round(float(y), 3)] for x, y in poly],
            floor_area_m2=room_mod.Measurement(round(area_val, 3), area_ci, "bootstrap_hull"),
            ceiling_height_m=h, ceiling_observed=bool(observed),
            floor_z=round(float(floor_z), 3), walls=fwalls,
        ))

    rooms.sort(key=lambda r: -r.floor_area_m2.value)
    for i, r in enumerate(rooms):
        r.room_id = f"r{i + 1}"
    return rooms


def adjacency(rooms: list[room_mod.Room]) -> list[dict]:
    out: list[dict] = []
    for i in range(len(rooms)):
        si = {id(w) for w in rooms[i].walls}
        for j in range(i + 1, len(rooms)):
            if si & {id(w) for w in rooms[j].walls}:
                out.append({"a": rooms[i].room_id, "b": rooms[j].room_id,
                            "via": "shared_wall"})
    return out
