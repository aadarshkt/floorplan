# Benchmark report — manifest.stray_video2.json

**Overall: 0/2 captures passed all gates (0%)**

**Accuracy: 8%** (1/13 measurements within tolerance)

## Per-capture gates

| capture | room | walls max/mean (cm) | walls missed/phantom | ceiling (cm) | openings pass (missed/phantom) | area (%) | result |
|---|---|---|---|---|---|---|---|
| study_room_friend | r1 | 93.73 / 39.77 | 0/2 | 87.30 | 33% (0/1) | — | FAIL |
| bedroom_2 | r1 | 95.68 / 64.70 | 0/0 | 21.70 | 0% (0/0) | — | FAIL |

## Per-wall errors

| capture | room | wall (plan) | measured as | error (cm) |
|---|---|---|---|---|
| study_room_friend | r1 | r1-w0 | by length | 17.7 |
| study_room_friend | r1 | r1-w1 | by length | 93.7 |
| study_room_friend | r1 | r1-w4 | by length | 26.9 |
| study_room_friend | r1 | r1-w5 | by length | 20.8 |
| bedroom_2 | r1 | r1-w0 | by length | 95.7 |
| bedroom_2 | r1 | r1-w1 | by length | 33.7 |
| bedroom_2 | r1 | r1-w2 | by length | 95.7 |
| bedroom_2 | r1 | r1-w3 | by length | 33.7 |

## Confidence-interval calibration

- samples: 10, coverage: 20% (target ≥ 85%)

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
