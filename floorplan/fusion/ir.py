"""Common intermediate representation (the seam that makes tiers swappable).

Every tier (LiDAR, photos, video) reduces to a single metric, Z-up point cloud
plus provenance describing how it was produced and how its metric scale was
established. Downstream geometry is tier-agnostic and only ever sees this.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# scale_reference values
NATIVE_METRIC = "native_metric"          # sensor gave metres directly (LiDAR)
METRIC_MODEL = "metric_model"            # learned metric depth: metres, no external anchor
SCALED = "scaled_via_reference"          # anchored to a known length / metric capture
NONE_PRIOR = "none_prior"                # no reference: default prior, non-metric


def is_metric(scale_reference: str) -> bool:
    """True when the cloud is already in metres (no pending anchor to apply)."""
    return scale_reference != NONE_PRIOR


@dataclass
class Reconstruction:
    points: np.ndarray                  # (N, 3) float, metres, canonical Z-up
    tier: str
    path_chosen: str                    # which code path produced this cloud
    scale_reference: str                # one of the constants above
    scale_factor: float = 1.0           # multiplier applied to raw units -> metres
    conf: np.ndarray | None = None      # (N,) optional per-point confidence
    notes: dict = field(default_factory=dict)
