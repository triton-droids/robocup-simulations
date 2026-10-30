# Handoff: 2026-10-26 19:47, LOC-05 particle filter core

Previous: [2026-10-26_1842_loc04-simulator.md](2026-10-26_1842_loc04-simulator.md).

## Rules in force
- No GitHub writes.
- New files only in `localization_work/`.
- Team repos untouched (all four clean on their branches).
- End each run with a handoff here.

## What was added: `localization_work/td_localization/`

- **`td_localization/pf.py`:** `ParticleFilter(field, likelihood, PFConfig, seed)`.
  - `init_poses(candidates)`: known placements, such as Initial or penalty re-entry.
  - `init_uniform(own_half_only=True)`: lost mode.
  - `predict(odom, odom_cov)`; `update(obs)` → `UpdateStats`; `estimate()` → `Estimate(pose, cov, weight, clusters)`.
  - `mirror()` for LOC-06; `lost` property.
  - Observations are duck-typed (`line_points`, `line_cov`, `landmarks[kind, xy, cov]`), so the sim's
    `Observations` fits.
- **`test/test_pf.py`:** 25 tests, about 70 s. **`test/helpers.py`:** `run()` scenario driver and `RunResult`
  metrics (a stand-in until LOC-07).
- Lint and format are clean (`td_perception/ruff.toml`).

## Mechanisms (all in the module docstring)

| Mechanism | What it does |
|---|---|
| Line update | Cauchy kernel on the likelihood field. Per-point σ is the largest standard deviation of its covariance. Tempered by 0.15 (points are correlated). |
| Landmark update | Same kernel, matched to the nearest map landmark of the same kind |
| Skip empty frames | Frames with < 10 points and no landmark are skipped (head turned off the field) |
| Recovery | Fast/slow quality ratio. Above a 30% drop, the filter roughens the cloud and injects poses pre-screened on the current frame (sensor resetting) |
| Lost mode | `init_uniform`: 3,000 particles and 10% screened proposals per frame. Ends after a cluster has ≥ 0.8 weight for 15 updates, then shrinks to 500. |
| Symmetry | Injected poses are folded toward `side_reference` (position + heading, 2 m/rad), or to x ≤ 0 while lost in our own half. The reference moves only when matches are healthy, confident and continuous (or after a 30-estimate streak). |
| Estimate | Densest-cell cluster seeding, plus hysteresis toward the previous cluster when none dominates |

## Results (simulation, M-Field unless noted)

**Scenarios with default noise** (pixel 1 px, 0.3° jitter, 2 false points/frame):

| Scenario | Result |
|---|---|
| 60 s tracking | p50 0.01 m, p95 0.03 m, heading p95 0.5° |
| Global localization from the sideline | converges in 0.7 s typical, ≤ 4 s across 18 seed/start combinations |
| Penalty re-entry | correct touchline at once |
| Fall recovery | ~0.1 s |
| Kidnap within the same half | recovers in 0.23 s |

**Seed sweep** (6 seeds, 60 s with a fall):

| Field | Noise | Worst p95 | Worst heading p95 | Side flips |
|---|---|---|---|---|
| M | default | 0.05 m | – | 0 |
| M | harsh | 0.26 m | 3.4° | 0 |
| L | default | 0.13 m | – | 0 |
| L | harsh | **0.60 m** | 5.6° | 0 |

Harsh noise = 1° pitch bias, 5 false points/frame, 20% dropout, 10% crossing confusion. L harsh is slightly over
the 0.5 m target, and its post-fall recovery takes up to 5.4 s.

**Cost:** update about 10 ms (p50) for 500 particles × 120 points on this laptop. **Lost mode is about 110 ms per
update** for about 1 s. Expect 3–5× slower on the Orin Nano, which is OK while standing in Initial but is a
performance item for LOC-20.

## Limits and decisions to know
- **A kidnap across the field is not recoverable by the filter alone.** If the robot is carried so that its mirror
  pose is nearer its old pose, continuity picks the mirror. That's correct given no other information. LOC-06 (side
  belief: GameController events, teammates) must handle it. Tests only cover same-half kidnaps.
- Tuned only in simulation. The simulator's noise and detector model are guesses (see the LOC-04 handoff). Retune
  on LOC-14 bags.
- `PFConfig` has about 30 parameters. Defaults are documented inline. The ones that mattered most:
  `line_temperature`, `drop_threshold`, `quality_alpha_slow`, `proposal_factor`, the lost-mode settings and
  `fold_heading_scale`.
- **History of bugs found and fixed this run** (useful for reviewers):
  1. Folding by position only flipped sides near the center.
  2. Global localization was too slow with 500 particles.
  3. Single-frame estimate spikes after resampling.
  4. Low-information frames triggered injection and wiped the cloud.
  5. The side reference dragged along with a wrong cloud after a kidnap.
  6. Uniform injection was too sparse to recover.
  7. Lost-mode shrink happened too early.

## How to run
```bash
cd <workspace>/localization_work
PYTHONPATH=td_field_model ../.venv-loc/Scripts/python -m pytest -q td_field_model/test
PYTHONPATH="td_field_model;td_localization;td_localization/test" ../.venv-loc/Scripts/python -m pytest -q td_localization/test
../.venv-loc/Scripts/ruff check --config ../td_perception/ruff.toml td_field_model td_localization
```
Run packages separately (duplicate `test_import.py` names, same as td_perception).

Exploration scripts (not part of the package) are in the session scratchpad: `explore_pf.py` (scenario table),
`sweep_pf.py` (seed sweep), `global_seeds.py`, `fall_recovery.py`. They're temporary; the important cases are now
in `test_pf.py`.

## Next steps
1. **LOC-06 side belief** (`td_localization/side.py`):
   - Own the binary half belief.
   - Inputs: GameController events (Initial seeding, penalty re-entry → `pf.init_poses(candidates)`), continuity,
     teammates' mirrored-ball check, goalkeeper weighting.
   - Act by setting `pf.side_reference` or calling `pf.mirror()`.
   - Test the cross-field kidnap case here.
2. **LOC-07 evaluation tool:** promote `test/helpers.py` metrics into `td_localization/eval.py` plus a CLI with a
   markdown report.
3. **LOC-08 GameController client:** read the official headers first.
4. Performance: profile lost mode. Options are fewer lost particles (1,500), fewer proposal points, or vectorizing
   further. Decide after a Jetson measurement.

## Open human decisions (unchanged)
LOC-01 IR policy confirmation, LOC-02 `td_msgs`, leg-model source, head encoders, which `td_perception` code base to
keep, when to move `localization_work` packages into a repo.
