"""Trajectory drift correction (loop closure) for the LiDAR tier.

A handheld scan that revisits a place accumulates error. We detect revisits
(frames whose camera returns near an earlier position) and distribute a smooth
correction around each loop. When there is no revisit (a single room), the
correction is a no-op — and the ablation reports a zero delta honestly.
"""
from __future__ import annotations

import numpy as np


def detect_loops(pos: np.ndarray, min_gap: int = 200, radius: float = 0.15,
                 min_travel: float = 2.0, max_loops: int = 5) -> list[tuple[int, int]]:
    """Find frames that return to an earlier place after a real excursion.

    A pair only counts as a loop closure if the camera came within ``radius`` of
    an earlier position *and* travelled at least ``min_travel`` metres in between
    — otherwise a handheld scanner that merely pauses would be "corrected".
    """
    n = len(pos)
    if n < min_gap + 1:
        return []
    step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(step)])
    loops: list[tuple[int, int]] = []
    for i in range(n):
        if len(loops) >= max_loops:
            break
        for j in range(i + min_gap, n):
            if cum[j] - cum[i] < min_travel:
                continue
            if float(np.linalg.norm(pos[i] - pos[j])) < radius:
                loops.append((i, j))
                break
    return loops


def correct_positions(pos: np.ndarray, loops: list[tuple[int, int]],
                      max_correction: float = 0.05) -> tuple[np.ndarray, bool]:
    """Pull the trajectory back by the distributed closure error (smoothstep).

    The correction is capped at ``max_correction`` metres: ARKit VIO drift over a
    room is already a few centimetres, so a crude loop blend must only ever nudge
    the trajectory, never relocate walls.
    """
    if not loops:
        return pos.copy(), False
    offsets = np.zeros_like(pos)
    counts = np.zeros(len(pos))
    for i, j in loops:
        e = pos[j] - pos[i]
        norm = float(np.linalg.norm(e))
        if norm > max_correction:
            e = e * (max_correction / norm)
        span = max(j - i, 1)
        t = (np.arange(i, j + 1) - i) / span
        w = t * t * (3 - 2 * t)
        offsets[i:j + 1] -= w[:, None] * e
        counts[i:j + 1] += 1
    idx = counts > 0
    offsets[idx] /= counts[idx][:, None]
    return pos + offsets, True
