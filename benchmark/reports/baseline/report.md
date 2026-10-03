# Benchmark report — manifest.json

**Overall: 3/3 captures passed all gates (100%)**

**Accuracy: 100%** (30/30 measurements within tolerance)

## Per-capture gates

| capture | room | walls max/mean (cm) | ceiling (cm) | openings pass | area (%) | result |
|---|---|---|---|---|---|---|
| lidar | room1 | 0.05 / 0.03 | 0.00 | 100% | 0.0 | PASS |
| photos | room1 | 0.05 / 0.03 | 0.00 | 100% | 0.0 | PASS |
| video | room1 | 0.05 / 0.03 | 0.00 | 100% | 0.0 | PASS |

## Confidence-interval calibration

- samples: 18, coverage: 100% (target ≥ 85%)

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
