# Benchmark report — manifest.bedroom.json

**Overall: 0/2 captures passed all gates (0%)**

**Accuracy: 0%** (0/12 measurements within tolerance)

## Per-capture gates

| capture | room | walls max/mean (cm) | ceiling (cm) | openings pass | area (%) | result |
|---|---|---|---|---|---|---|
| bedroom_video | r1 | 66.60 / 37.62 | 87.00 | 100% | 15.5 | FAIL |
| room_rgb_video | r1 | 477.54 / 280.59 | 30.10 | 100% | 222.7 | FAIL |

## Repeatability (same room, two captures)

| room | max wall (cm) | mean wall (cm) | ceiling (cm) | area (%) | result |
|---|---|---|---|---|---|
| r1 | 569.19 | 386.66 | 56.90 | 281.764 | FAIL |

## Confidence-interval calibration

- samples: 12, coverage: 0% (target ≥ 85%)

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
