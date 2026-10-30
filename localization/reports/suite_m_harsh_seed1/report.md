# Localization scenario suite

Simulated suite: field `hsl_m_2026`, noise `harsh`, seed 1. Simulation evidence only: the noise and detector model are guesses until real field bags exist (LOC-14). `global_sideline` starts with no known pose (search of the own half). `kidnap_cross_alone` is expected to fail the side checks: without teammates nothing can tell the halves apart after a cross-field kidnap (the side confidence should drop instead). `team_kidnap_cross_*` runs three robots with real team messages over a lossy network (LOC-21); the healthy teammate must never be misled. Latency is this machine's compute time per cycle (not recorded for team runs).

Each ground-truth sample is matched to the nearest estimate within 0.050 s (no interpolation). Accuracy = position ≤ 0.50 m and heading ≤ 5.0°, over all samples; missing estimates count as failures. Percentiles are over usable samples only. Wrong half = estimate nearer the mirrored truth.

## Metrics

| Metric | tracking | global_sideline | penalty_reentry | falls | kidnap_same_half | kidnap_cross_team | kidnap_cross_alone | team_kidnap_cross_kidnapped_p3 | team_kidnap_cross_teammate_p2 |
|---|---|---|---|---|---|---|---|---|---|
| Accuracy within threshold | 1.000 | 0.947 | 1.000 | 0.988 | 0.717 | 0.943 | 0.334 | 0.688 | 1.000 |
| Availability | 1.000 | 0.947 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| Position p50 (m) | 0.152 | 0.222 | 0.138 | 0.191 | 0.211 | 0.182 | 2.744 | 0.175 | 0.152 |
| Position p95 (m) | 0.203 | 0.327 | 0.187 | 0.293 | 1.326 | 5.950 | 8.188 | 6.369 | 0.211 |
| Position max (m) | 0.240 | 0.351 | 0.201 | 0.459 | 1.460 | 7.352 | 8.278 | 7.261 | 0.232 |
| Heading p95 (°) | 1.971 | 4.000 | 3.175 | 2.900 | 34.269 | 168.631 | 179.884 | 179.919 | 3.148 |
| Wrong-half fraction | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 0.057 | 0.666 | 0.312 | 0.000 |
| Wrong-half episodes | 0 | 0 | 0 | 0 | 0 | 1 | 1 | 1 | 0 |
| Longest wrong-half (s) | 0.000 | 0.000 | 0.000 | 0.000 | 0.000 | 1.667 | 20.000 | 9.333 | 0.000 |
| Confidently wrong samples | 0 | 0 | 0 | 0 | 0 | 8 | 601 | 250 | 0 |
| Side confidence when right | 0.998 | 0.994 | 0.998 | 0.998 | 0.997 | 1.000 | 0.998 | 0.998 | 1.000 |
| Side confidence when wrong | N/A | N/A | N/A | N/A | N/A | 0.254 | 0.998 | 0.887 | N/A |
| Latency p50 (ms) | 1.062 | 8.176 | 1.064 | 7.465 | 1.043 | 1.264 | 1.196 | N/A | N/A |
| Latency p95 (ms) | 14.025 | 14.302 | 12.892 | 12.833 | 13.034 | 15.198 | 12.961 | N/A | N/A |

## Target checks (docs/roadmap.md §3.3)

| Check | tracking | global_sideline | penalty_reentry | falls | kidnap_same_half | kidnap_cross_team | kidnap_cross_alone | team_kidnap_cross_kidnapped_p3 | team_kidnap_cross_teammate_p2 |
|---|---|---|---|---|---|---|---|---|---|
| pos p50 ≤ 0.2 m | PASS | FAIL | PASS | PASS | FAIL | PASS | FAIL | PASS | PASS |
| pos p95 ≤ 0.5 m | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | FAIL | PASS |
| heading p95 ≤ 5.0° | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | FAIL | PASS |
| no wrong-half samples | PASS | PASS | PASS | PASS | PASS | FAIL | FAIL | FAIL | PASS |
| every event converges ≤ 10.0 s | PASS | PASS | PASS | PASS | PASS | PASS | FAIL | PASS | PASS |

## Convergence after events

| Run | t (s) | Event | Converged after (s) |
|---|---|---|---|
| tracking | 0.00 | initial | 0.00 |
| global_sideline | 0.00 | initial | 3.00 |
| penalty_reentry | 0.00 | initial | 0.00 |
| falls | 0.00 | initial | 0.00 |
| falls | 15.03 | fall | 1.97 |
| falls | 30.07 | fall | 0.00 |
| kidnap_same_half | 0.00 | initial | 0.00 |
| kidnap_same_half | 10.03 | kidnap | 7.10 |
| kidnap_cross_team | 0.00 | initial | 0.00 |
| kidnap_cross_team | 10.03 | kidnap | 3.50 |
| kidnap_cross_alone | 0.00 | initial | 0.00 |
| kidnap_cross_alone | 10.03 | kidnap | N/A |
| team_kidnap_cross_kidnapped_p3 | 0.00 | initial | 0.00 |
| team_kidnap_cross_kidnapped_p3 | 10.03 | kidnap | 9.37 |
| team_kidnap_cross_teammate_p2 | 0.00 | initial | 0.00 |
