# Video tier vs LiDAR — manifest.r3d.json

LiDAR run of the same clip is the reference (not tape ground truth).

| capture | reg | ATE cm | scale err % | tilt ° | acc med cm | prec % | compl % | walls V/L | wall rel % | ceil Δ cm | area Δ % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| c00a170fe1 | 27/120 | 1.91 | 0.27 | 5.54 | 5.02 | 49.9 | 24.8 | 14/30 | 9.19 | 0.3 | 74.7 |
| 1a8384c3f6 | 22/120 | 173.38 | 508.21 | 8.92 | 46.68 | 3.6 | 0.1 | 46/66 | 8.1 | 1.0 | 63.4 |
| c7d28f72c6 | 6/120 | 115.12 | 707.59 | 29.41 | 16.58 | 9.5 | 0.1 | 6/0 | None | 20.2 | 89.7 |
| benchmark_1 | 120/120 | 1.94 | -6.79 | 1.5 | 5.82 | 45.4 | 71.9 | 16/16 | 11.83 | 5.3 | 18.1 |
| benchmark_2 | 87/87 | 1.86 | 4.01 | 0.94 | 5.71 | 46.5 | 65.6 | 8/8 | 5.71 | 36.5 | 4.2 |
