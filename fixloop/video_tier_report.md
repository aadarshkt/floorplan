# Benchmark report — manifest.stray_video.json

**Overall: 0/4 captures passed all gates (0%)**

**Accuracy: 9%** (2/22 measurements within tolerance)

## Per-capture gates

| capture | room | walls max/mean (cm) | walls missed/phantom | ceiling (cm) | openings pass (missed/phantom) | area (%) | result |
|---|---|---|---|---|---|---|---|
| study_room_friend | r1 | 140.12 / 88.51 | 0/14 | 148.40 | 0% (2/0) | — | FAIL |
| bedroom_2 | r1 | 210.70 / 113.05 | 0/2 | 47.00 | 0% (1/0) | — | FAIL |
| kitchen_room_scan | r1 | 124.67 / 67.83 | 0/4 | 35.80 (unobs) | 0% (2/0) | — | FAIL |
| kitchen_scan_2 | r1 | 99.31 / 32.75 | 0/12 | 162.00 | 0% (2/0) | — | FAIL |

## Per-wall errors

| capture | room | wall (plan) | measured as | error (cm) |
|---|---|---|---|---|
| study_room_friend | r1 | r1-w2 | by length | 104.6 |
| study_room_friend | r1 | r1-w7 | by length | 140.1 |
| study_room_friend | r1 | r1-w10 | by length | 91.9 |
| study_room_friend | r1 | r1-w16 | by length | 17.4 |
| bedroom_2 | r1 | r1-w2 | by length | 123.3 |
| bedroom_2 | r1 | r1-w3 | by length | 210.7 |
| bedroom_2 | r1 | r1-w4 | by length | 73.9 |
| bedroom_2 | r1 | r1-w5 | by length | 44.3 |
| kitchen_room_scan | r1 | r1-w0 | by length | 63.5 |
| kitchen_room_scan | r1 | r1-w3 | by length | 60.6 |
| kitchen_room_scan | r1 | r1-w4 | by length | 124.7 |
| kitchen_room_scan | r1 | r1-w5 | by length | 22.5 |
| kitchen_scan_2 | r1 | r1-w8 | by length | 23.0 |
| kitchen_scan_2 | r1 | r1-w10 | by length | 3.8 |
| kitchen_scan_2 | r1 | r1-w11 | by length | 4.9 |
| kitchen_scan_2 | r1 | r1-w14 | by length | 99.3 |

## Repeatability (same room, two captures)

| room | max wall (cm) | mean wall (cm) | ceiling (cm) | area (%) | result |
|---|---|---|---|---|---|
| r1 | 33.47 | 12.39 | 197.80 | 887.216 | FAIL |

## Confidence-interval calibration

- samples: 20, coverage: 30% (target ≥ 85%)

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
