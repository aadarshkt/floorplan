# Benchmark report — manifest.kitchen.json

**Overall: 0/1 captures passed all gates (0%)**

**Accuracy: 12%** (1/8 measurements within tolerance)

## Per-capture gates

| capture | room | walls max/mean (cm) | walls missed/phantom | ceiling (cm) | openings pass (missed/phantom) | area (%) | result |
|---|---|---|---|---|---|---|---|
| kitchen_room_scan | r1 | 119.18 / 69.52 | 0/0 | 0.10 | 0% (0/0) | — | FAIL |

## Per-wall errors

| capture | room | wall (plan) | measured as | error (cm) |
|---|---|---|---|---|
| kitchen_room_scan | r1 | r1-w0 | by length | 19.9 |
| kitchen_room_scan | r1 | r1-w1 | by length | 119.2 |
| kitchen_room_scan | r1 | r1-w2 | by length | 19.9 |
| kitchen_room_scan | r1 | r1-w3 | by length | 119.2 |

## Confidence-interval calibration

- samples: 5, coverage: 20% (target ≥ 85%)

## Gates

| gate | threshold |
|---|---|
| ceiling_height_abs_cm | 1.5 |
| opening_width_abs_cm | 2.0 |
| opening_pass_rate | 0.85 |
| opening_phantom_max | 1 |
| wall_length_abs_cm | 3.0 |
| wall_length_max_abs_cm | 6.0 |
| area_rel_pct | 5.0 |
| repeatability_max_cm | 1.0 |
| repeatability_rel_pct | 0.5 |
| ci_coverage_min | 0.85 |
