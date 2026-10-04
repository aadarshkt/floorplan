# Benchmark report — manifest.stray.json

**Overall: 0/4 captures passed all gates (0%)**

**Accuracy: 7%** (2/27 measurements within tolerance)

## Per-capture gates

| capture | room | walls max/mean (cm) | walls missed/phantom | ceiling (cm) | openings pass (missed/phantom) | area (%) | result |
|---|---|---|---|---|---|---|---|
| kitchen_room_scan | r1 | 98.18 / 59.01 | 0/0 | 0.10 | 0% (0/0) | — | FAIL |
| kitchen_scan_2 | r1 | 96.20 / 58.41 | 0/0 | 2.90 | 0% (1/0) | — | FAIL |
| bedroom_2 | r1 | 137.39 / 79.14 | 0/4 | 3.40 | 0% (1/0) | — | FAIL |
| study_room_friend | r1 | 161.37 / 79.78 | 0/6 | 0.90 | 0% (0/0) | — | FAIL |

## Per-wall errors

| capture | room | wall (plan) | measured as | error (cm) |
|---|---|---|---|---|
| kitchen_room_scan | r1 | r1-w0 | by length | 19.9 |
| kitchen_room_scan | r1 | r1-w1 | by length | 98.2 |
| kitchen_room_scan | r1 | r1-w2 | by length | 19.9 |
| kitchen_room_scan | r1 | r1-w3 | by length | 98.2 |
| kitchen_scan_2 | r1 | r1-w0 | by length | 20.6 |
| kitchen_scan_2 | r1 | r1-w1 | by length | 96.2 |
| kitchen_scan_2 | r1 | r1-w2 | by length | 20.6 |
| kitchen_scan_2 | r1 | r1-w3 | by length | 96.2 |
| bedroom_2 | r1 | r1-w4 | by length | 112.9 |
| bedroom_2 | r1 | r1-w5 | by length | 6.7 |
| bedroom_2 | r1 | r1-w6 | by length | 59.6 |
| bedroom_2 | r1 | r1-w7 | by length | 137.4 |
| study_room_friend | r1 | r1-w0 | by length | 56.5 |
| study_room_friend | r1 | r1-w5 | by length | 161.4 |
| study_room_friend | r1 | r1-w6 | by length | 43.8 |
| study_room_friend | r1 | r1-w9 | by length | 57.4 |

## Repeatability (same room, two captures)

| room | max wall (cm) | mean wall (cm) | ceiling (cm) | area (%) | result |
|---|---|---|---|---|---|
| r1 | 1.98 | 1.37 | 3.00 | 0.501 | FAIL |

## Confidence-interval calibration

- samples: 20, coverage: 15% (target ≥ 85%)

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
