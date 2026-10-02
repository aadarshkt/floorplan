# Benchmark report — manifest.example.json

**Overall: 3/3 captures passed all gates (100%)**

## Per-capture gates

| capture | room | walls max/mean (cm) | ceiling (cm) | openings pass | area (%) | result |
|---|---|---|---|---|---|---|
| single_room | room1 | 0.00 / 0.00 | 0.00 (unobs) | 100% | 0.0 | PASS |
| single_scan_floor_only | room1 | 0.00 / 0.00 | 0.00 (unobs) | 100% | 0.0 | PASS |
| single_scan_with_ceiling | room1 | 0.00 / 0.00 | 0.00 | 100% | 0.0 | PASS |

## Confidence-interval calibration

- samples: 19, coverage: 95% (target ≥ 85%)

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
