# Video tier vs LiDAR — manifest.r3d.json

LiDAR run of the same clip is the reference (not tape ground truth).

| capture | reg | ATE cm | scale err % | tilt ° | acc med cm | prec % | compl % | walls V/L | wall rel % | ceil Δ cm | area Δ % |
|---|---|---|---|---|---|---|---|---|---|---|---|
| c00a170fe1 | 27/120 | 9.58 | -2.43 | 55.93 | 34.13 | 7.8 | 3.4 | 6/30 | 4.9 | 0.1 | 88.4 |
| 1a8384c3f6 | 21/120 | 4.49 | 9.91 | 102.81 | 10.66 | 26.5 | 7.9 | 30/66 | 4.12 | 10.4 | 69.6 |
| c7d28f72c6 | 6/120 | 108.39 | 302.4 | 51.99 | 14.94 | 12.3 | 0.1 | 6/0 | None | 20.4 | 35.9 |
| benchmark_1 | 120/120 | 1.96 | -6.79 | 1.53 | 5.77 | 45.7 | 71.8 | 16/16 | 9.94 | 15.3 | 15.5 |
| benchmark_2 | 87/87 | 1.66 | 4.1 | 0.63 | 5.3 | 48.5 | 68.5 | 8/8 | 5.36 | 36.6 | 4.7 |
