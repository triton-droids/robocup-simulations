# Localization scenario suite

Simulated suite: field `hsl_m_2026`, noise `default`, seed 1. Simulation evidence only: the noise and detector model are guesses until real field bags exist (LOC-14). `global_sideline` starts with no known pose (search of the own half). `kidnap_cross_alone` is expected to fail the side checks: without teammates nothing can tell the halves apart after a cross-field kidnap (the side confidence should drop instead). `team_kidnap_cross_*` runs three robots with real team messages over a lossy network (LOC-21); the healthy teammate must never be misled. Latency is this machine's compute time per cycle (not recorded for team runs).

Each ground-truth sample is matched to the nearest estimate within 0.050 s (no interpolation). Accuracy = position ≤ 0.50 m and heading ≤ 5.0°, over all samples; missing estimates count as failures. Percentiles are over usable samples only. Wrong half = estimate nearer the mirrored truth.

## Metrics

| Metric | tracking | global_sideline | penalty_reentry | falls | kidnap_same_half | kidnap_cross_team | kidnap_cross_alone | team_kidnap_cross_kidnapped_p3 | team_kidnap_cross_teammate_p2 |
|---|---|---|---|---|---|---|---|---|---|
| Accuracy within threshold | 1.000 | 0.947 | 1.000 | 0.999 | 0.916 | 0.912 | 0.334 | 0.688 | 1.000 |
| Availability | 1.000 | 0.950 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| Position p50 (m) | 0.013 | 0.016 | 0.018 | 0.016 | 0.020 | 0.018 | 2.787 | 0.020 | 0.013 |
| Position p95 (m) | 0.031 | 0.199 | 0.038 | 0.080 | 1.377 | 6.748 | 8.228 | 6.352 | 0.036 |
| Position max (m) | 0.050 | 6.067 | 0.101 | 0.416 | 1.505 | 7.451 | 8.312 | 7.200 | 0.092 |
| Heading p95 (°) | 0.507 | 2.093 | 0.498 | 0.913 | 34.985 | 167.873 | 179.973 | 179.940 | 0.533 |
| Wrong-half fraction | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.057 | 0.666 | 0.312 | 0.000 |
| Wrong-half episodes | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 1 | 0 |
| Longest wrong-half (s) | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 1.667 | 20.000 | 9.333 | 0.000 |
| Confidently wrong samples | 0 | 0 | 0 | 0 | 0 | 8 | 601 | 250 | 0 |
| Side confidence when right | 0.998 | 0.994 | 0.998 | 0.997 | 0.997 | 0.997 | 0.998 | 0.998 | 1.000 |
| Side confidence when wrong | N/A | N/A | N/A | N/A | N/A | 0.252 | 0.994 | 0.887 | N/A |
| Latency p50 (ms) | 1.149 | 8.359 | 1.018 | 7.711 | 1.029 | 1.794 | 1.360 | N/A | N/A |
| Latency p95 (ms) | 14.533 | 13.759 | 13.311 | 13.764 | 14.225 | 15.922 | 14.031 | N/A | N/A |

## Target checks (docs/roadmap.md §3.3)

| Check | tracking | global_sideline | penalty_reentry | falls | kidnap_same_half | kidnap_cross_team | kidnap_cross_alone | team_kidnap_cross_kidnapped_p3 | team_kidnap_cross_teammate_p2 |
|---|---|---|---|---|---|---|---|---|---|
| pos p50 ≤ 0.2 m | PASS | PASS | PASS | PASS | PASS | PASS | FAIL | PASS | PASS |
| pos p95 ≤ 0.5 m | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | FAIL | PASS |
| heading p95 ≤ 5.0° | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | FAIL | PASS |
| no wrong-half samples | PASS | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| every event converges ≤ 10.0 s | PASS | PASS | PASS | PASS | PASS | PASS | FAIL | PASS | PASS |

## Convergence after events

| Run | t (s) | Event | Converged after (s) |
|---|---|---|---|
| tracking | 0.00 | initial | 0.00 |
| global_sideline | 0.00 | initial | 1.07 |
| penalty_reentry | 0.00 | initial | 0.00 |
| falls | 0.00 | initial | 0.00 |
| falls | 15.03 | fall | 1.37 |
| falls | 30.07 | fall | 0.00 |
| kidnap_same_half | 0.00 | initial | 0.00 |
| kidnap_same_half | 10.03 | kidnap | 2.10 |
| kidnap_cross_team | 0.00 | initial | 0.00 |
| kidnap_cross_team | 10.03 | kidnap | 2.70 |
| kidnap_cross_alone | 0.00 | initial | 0.00 |
| kidnap_cross_alone | 10.03 | kidnap | N/A |
| team_kidnap_cross_kidnapped_p3 | 0.00 | initial | 0.00 |
| team_kidnap_cross_kidnapped_p3 | 10.03 | kidnap | 9.37 |
| team_kidnap_cross_teammate_p2 | 0.00 | initial | 0.00 |
