# Benchmark report — manifest.stray.json

**Overall: 0/4 captures passed all gates (0%)**

**Accuracy: 26%** (6/23 measurements within tolerance)

## Per-capture gates

| capture | room | walls max/mean (cm) | walls missed/phantom | ceiling (cm) | openings pass (missed/phantom) | area (%) | result |
|---|---|---|---|---|---|---|---|
| kitchen_room_scan | r1 | 18.80 / 15.50 | 0/0 | 0.90 | 0% (2/0) | — | FAIL |
| kitchen_scan_2 | r1 | 7.66 / 7.04 | 0/0 | 3.30 | 0% (2/0) | — | FAIL |
| bedroom_2 | r1 | 61.43 / 36.29 | 0/0 | 3.50 | 0% (1/0) | — | FAIL |
| study_room_friend | r1 | 5.60 / 4.60 | 0/0 | 0.90 | 0% (1/0) | — | FAIL |

## Per-wall errors

| capture | room | wall (plan) | measured as | error (cm) |
|---|---|---|---|---|
| kitchen_room_scan | r1 | r1-w0 | by length | 12.2 |
| kitchen_room_scan | r1 | r1-w1 | by length | 18.8 |
| kitchen_room_scan | r1 | r1-w2 | by length | 12.2 |
| kitchen_room_scan | r1 | r1-w3 | by length | 18.8 |
| kitchen_scan_2 | r1 | r1-w0 | by length | 7.7 |
| kitchen_scan_2 | r1 | r1-w1 | by length | 6.4 |
| kitchen_scan_2 | r1 | r1-w2 | by length | 7.7 |
| kitchen_scan_2 | r1 | r1-w3 | by length | 6.4 |
| bedroom_2 | r1 | r1-w0 | by length | 61.4 |
| bedroom_2 | r1 | r1-w1 | by length | 11.2 |
| bedroom_2 | r1 | r1-w2 | by length | 61.4 |
| bedroom_2 | r1 | r1-w3 | by length | 11.2 |
| study_room_friend | r1 | r1-w0 | by length | 3.6 |
| study_room_friend | r1 | r1-w1 | by length | 5.6 |
| study_room_friend | r1 | r1-w2 | by length | 3.6 |
| study_room_friend | r1 | r1-w3 | by length | 5.6 |

## Repeatability (same room, two captures)

| room | max wall (cm) | mean wall (cm) | ceiling (cm) | area (%) | result |
|---|---|---|---|---|---|
| r1 | 25.23 | 14.88 | 2.40 | 6.207 | FAIL |

## Confidence-interval calibration

- samples: 20, coverage: 40% (target ≥ 85%)

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
