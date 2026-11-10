# 01 Orientation

## What this is

Field localization for the Triton Droids humanoid (HSL 2026, Large division, M-Field 14 × 9 m).
- **Output:** the pose `(x, y, θ)` in the field frame, plus a side confidence.
- **Method:** MCL on field-line points, plus a separate side belief (the field is point-symmetric).
- **Status:** simulation and loopback only; nothing has run on hardware.

Repo: `triton-droids/robocup-simulations`, branch `localization`, folder `localization/`. Independent of the
ProtoMotions code in the rest of the repo.

## Layout

| Path | Contents |
|---|---|
| `td_field_model/td_field_model/field.py` | [`FieldModel`](../../../td_field_model/td_field_model/field.py), [`FieldDims`](../../../td_field_model/td_field_model/field.py), [`Segment`](../../../td_field_model/td_field_model/field.py), [`Arc`](../../../td_field_model/td_field_model/field.py), [`Landmark`](../../../td_field_model/td_field_model/field.py), [`LikelihoodField`](../../../td_field_model/td_field_model/field.py), `mirror_pose`. Fields: `fields/field_hsl_{s,m,l}_2026.json`. |
| `td_field_model/td_field_model/sim.py` | [`CameraModel`](../../../td_field_model/td_field_model/sim.py), [`NoiseModel`](../../../td_field_model/td_field_model/sim.py), [`Simulator`](../../../td_field_model/td_field_model/sim.py), [`Observations`](../../../td_field_model/td_field_model/sim.py), [`ObservedLandmark`](../../../td_field_model/td_field_model/sim.py); `project_floor`, `backproject_pixels`, `floor_covariances`, `robot_to_world`, `world_to_robot`. |
| `td_field_model/td_field_model/scenarios.py` | [`ScenarioBuilder`](../../../td_field_model/td_field_model/scenarios.py), [`Step`](../../../td_field_model/td_field_model/scenarios.py), [`OdometryNoise`](../../../td_field_model/td_field_model/scenarios.py), [`HeadMotion`](../../../td_field_model/td_field_model/scenarios.py); `compose`, `relative_motion`, `penalty_reentry_poses`, `kickoff_poses`, `sideline_start_poses`. |
| `td_localization/td_localization/pf.py` | [`ParticleFilter`](../../../td_localization/td_localization/pf.py), [`PFConfig`](../../../td_localization/td_localization/pf.py), [`Estimate`](../../../td_localization/td_localization/pf.py), [`Cluster`](../../../td_localization/td_localization/pf.py), [`UpdateStats`](../../../td_localization/td_localization/pf.py). |
| `td_localization/td_localization/side.py` | [`SideBelief`](../../../td_localization/td_localization/side.py), [`SideConfig`](../../../td_localization/td_localization/side.py), [`TeamReport`](../../../td_localization/td_localization/side.py), [`SideEvent`](../../../td_localization/td_localization/side.py). |
| `td_localization/td_localization/game_bridge.py` | [`GameBridge`](../../../td_localization/td_localization/game_bridge.py): GameController events → `SideBelief`; [`TeamMessage`](../../../td_game/td_game/team_msg.py) → `TeamReport`. |
| `td_localization/td_localization/eval.py` | CSV I/O ([`GroundTruth`](../../../td_localization/td_localization/eval.py), [`EstimateTrack`](../../../td_localization/td_localization/eval.py)), `evaluate`, `write_report`, CLI (`evaluate`, `suite`). |
| `td_localization/td_localization/suite.py` | Standard scenarios, `simulate`, `status_of`, [`BallTrack`](../../../td_localization/td_localization/suite.py), `run_suite`. |
| `td_localization/td_localization/team_sim.py` | Multi-robot simulation: [`RobotSpec`](../../../td_localization/td_localization/team_sim.py), [`LossyNetwork`](../../../td_localization/td_localization/team_sim.py), `run_team`. |
| `td_game/td_game/` | `gc_data.py` (wire formats, [`ControlData`](../../../td_game/td_game/gc_data.py)), `game_state.py` ([`GameStateTracker`](../../../td_game/td_game/game_state.py), [`GameEvent`](../../../td_game/td_game/game_state.py)), `team_msg.py` ([`TeamMessage`](../../../td_game/td_game/team_msg.py), `pack`/`unpack`, [`BudgetPacer`](../../../td_game/td_game/team_msg.py)), `team_comm.py` ([`TeamComm`](../../../td_game/td_game/team_comm.py)), `udp.py` ([`GameControllerLink`](../../../td_game/td_game/udp.py), [`TeamLink`](../../../td_game/td_game/udp.py)). |
| `td_bench/td_bench/` | `sources.py` (input sources), `vision.py` (renderer, line detector, frame sources), `record.py` (record/replay), `bench.py` ([`Bench`](../../../td_bench/td_bench/bench.py)), `__main__.py` (CLI). |
| `test_report/` | `make_results.py` → `index.html`; `visual_only.py` → `visual_only.html`; `raw/` is git-ignored. |
| `reports/` | Committed reference suite summaries (`suite_m_{default,harsh}_seed1/report.md`, `results.json`); everything else is ignored. |
| `docs/` | `roadmap.md` (rules, design), `implementation_plan.md` (tickets, §6 status, §9 decisions), `figures/`, `handoffs/`, `getting_started/`. |
| `run_tests.py`, `ruff.toml`, `pytest.ini`, `requirements.txt`, `.gitignore` | Tooling. |

Each package is ament_python (`package.xml`, `setup.py`, `setup.cfg`, `resource/`, `test/`).

## Environment

- Python ≥ 3.10; verified on 3.10 with numpy 1.21.5 (the robot's Ubuntu 22.04 versions). **3.11 is unverified.**
- Use a venv at `localization/.venv-localization` (git-ignored): `pip install -r requirements.txt`.
- Don't use the repo-root simulator venvs (`.venv-mujoco` etc.).
- No Docker, GPU or ROS needed.

## Commands (run from `localization/`)

| Task | Command |
|---|---|
| All tests + ruff | `python run_tests.py` (≈5 min; `td_localization` ≈4 min) |
| One package | `python run_tests.py td_game` |
| By hand | `PYTHONPATH=td_field_model:td_game:td_localization:td_bench python -m pytest -q td_bench/test` (`;` on Windows) |
| Lint / format | `ruff check .` / `ruff format .` |
| Scenario suite | `python -m td_localization.eval suite --out reports/<name>` (≈80 s) |
| Grade CSVs | `python -m td_localization.eval evaluate --gt gt.csv --est a.csv [--est b.csv] --out reports/<name>` |
| Bench | `python -m td_bench --camera … --odom … --head … --game … --team … --out reports/<name>` |
| Test report | `python test_report/make_results.py` (≈4 min), `python test_report/visual_only.py` (≈8 min) |

PYTHONPATH per package:
- `td_field_model`: `td_field_model`
- `td_game`: `td_game:td_game/test`
- `td_localization`: `td_field_model:td_game:td_localization:td_localization/test`
- `td_bench`: all four package folders

## Conventions

- **Field frame (`map`):**
  - origin at the center mark, +x toward the opponent goal, +y left;
  - **own goal at −x, in both halves of the game** (no half-time flip);
  - identical to the GameController status frame.
- **Robot frame (`base_footprint`):** x forward, y left, on the floor.
- **Camera optical frame:** z forward, x right, y down. Head pitch is positive looking down (REP-103).
- **Units:** SI only (m, rad, s). Angles wrapped to (−π, π] via `atan2(sin, cos)`.
- **Field geometry:** all lengths are between line **centerlines** (M-Field 13.95 × 8.95 m; the rule book's outer
  14 × 9).
- **Mirror:** `(x, y, θ) → (−x, −y, θ + π)`.
- **Pose distance** (folding, wrong-half): position plus heading at 2 m/rad.
- **Randomness:** `np.random.default_rng(seed)` everywhere; tests are seeded and deterministic (except the
  real-time UDP bench test, which is wall-clock sensitive).
- **Settings:** frozen dataclasses (`PFConfig`, `SideConfig`, `NoiseModel`, …); every default is commented.
- **Lint:** `localization/ruff.toml` (line 110, py310, rules E F I B UP); it overrides the repo root config (100).

## Invariants (don't break)

1. `ParticleFilter` never decides the half.
   - Every generated pose passes `_fold_to_reference`.
   - `side_reference` moves only while the filter is healthy (not lost, not degraded, best cluster ≥ 0.8) and
     either continuous (< 1.5 m pose-distance) or confident for 30 estimates in a row.
2. Only `SideBelief._flip` calls `pf.mirror()`. A flip requires all of:
   - ℓ < −4 held for 1 s;
   - no flip in the last 3 s;
   - consensus: the goalkeeper, or ≥ 2 distinct sources within 15 s.
3. `SideBelief.on_my_ball` is ignored unless the pose is trustworthy (not lost or degraded, weight ≥ 0.8).
4. While `GameBridge.paused`, no `predict`/`update` (the robot is being carried).
5. `Observations` is duck-typed:
   - `line_points` (N, 2) and `line_cov` (N, 2, 2), robot frame;
   - `landmarks` items with `kind`, `xy`, `cov`.
   Any detector must output this.
6. Team message format `TDLc` v1, 22 bytes. A change needs a version bump and simultaneous deployment.
7. The evaluation never interpolates: nearest estimate within 0.05 s; missing or unusable estimates count as
   failures in accuracy.
