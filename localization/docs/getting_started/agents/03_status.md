# 03 Status

As of 2026-11-10. The source of truth is [`implementation_plan.md`](../../implementation_plan.md) §6; update that, not
this file, when status changes. Then fix this summary if it's wrong.

## Code state

- Branch `localization` of `triton-droids/robocup-simulations`, commit `4d353ef` (one commit on top of `main`
  `4bfbefa`).
- **237 tests pass:** td_field_model 78, td_game 42, td_localization 86, td_bench 31. Ruff is clean.
- Verified on Python 3.10 + numpy 1.21.5 only.
- **Simulation and loopback only.** No robot, Jetson, D435, real GameController or field network has been used.

## Done

| Ticket | Deliverable |
|---|---|
| LOC-03 | Field model + likelihood field (`td_field_model/field.py`) |
| LOC-04 | [`Simulator`](../../../td_field_model/td_field_model/sim.py) + scenarios (`sim.py`, `scenarios.py`) |
| LOC-05 | Particle filter with recovery, drift test, lost mode (`pf.py`) |
| LOC-06 | Side belief (`side.py`) |
| LOC-07 | Evaluation + suite + reference reports (`eval.py`, `suite.py`, `reports/`) |
| LOC-08 | GameController v7 client + [`GameBridge`](../../../td_localization/td_localization/game_bridge.py) (`td_game/`, `game_bridge.py`) |
| LOC-21 | Team comms: 22 B messages, [`BudgetPacer`](../../../td_game/td_game/team_msg.py), [`TeamComm`](../../../td_game/td_game/team_comm.py), multi-robot sim, consensus rule. **The field test is still open.** |
| LOC-25 | Bench: any mix of real/recorded/sim inputs, record/replay, CLI, visual-only mix (cuVSLAM model, IMU head model), test report |

## Measured (simulation, M-Field)

Suite, seed 1, `default` noise (`harsh` noise in parentheses):

| Scenario | p50 / p95 | Convergence after the event | Wrong half |
|---|---|---|---|
| tracking | 1.3 / 3.1 cm (15 / 20 cm) | – | 0 |
| global_sideline (unknown start) | 1.6 / 20 cm (22 / 33 cm) | 1.1 s (3.0 s) | 0 |
| penalty_reentry | 1.8 / 3.8 cm | – | 0 |
| falls | 1.6 / 8.0 cm | 1.4 s (2.0 s) | 0 |
| kidnap_same_half | p95 1.38 m (includes recovery) | 2.1 s (7.1 s) | 0 |
| kidnap_cross_team | – | 2.7 s (3.5 s) | 1.7 s |
| kidnap_cross_alone | – | never (**by design**) | 20 s, confidently wrong |
| team_kidnap_cross, kidnapped / healthy teammate | – | 9.4 s / – | 9.3 s / **0** |

Cycle compute on a laptop: p50 ≈ 1 ms, p95 ≈ 14–16 ms (no Jetson numbers yet).

Visual-only (`test_report/visual_only.html`; 3 seeds × 30 s; cuVSLAM **model**):

| Variant | Result |
|---|---|
| VO + images + IMU head | 3/3 converge; last-10 s p50 3 cm, p95 ≤ 13 cm; 0 wrong half; VO lost 20% (head scan 1.26 rad/s > 1.2 rad/s gate) |
| Exact head angles | p50 1 cm, p95 ≤ 5 cm |
| No odometry at all | 2/3 converge; one seed ends mirrored (56% of the time on the wrong half) |

## Known limits

1. Mirror kidnap with no teammates: undetectable by design.
2. Three-robot kidnap recovery: ~9 s, and it recovered in only 2 of 4 seeds.
3. The substitute entry is assumed to be the same as a re-entry (`game_state.py`); confirm against the 2027 rules.
4. `td_bench/vision.py` detector is a stand-in: white next to green, no landmarks, untested on real images.
   [`RealSenseFrames`](../../../td_bench/td_bench/vision.py) has never run.
5. cuVSLAM is modeled ([`SimVisualOdometry`](../../../td_bench/td_bench/sources.py)); no GPU available. Its parameters are assumptions (roadmap §3.6).
6. Network code is tested on `127.0.0.1` only.
7. `line_range_ok` is produced but unused by the PF (far points are handled through σ).
8. The repo CI doesn't run `localization/`. Python 3.11 is unverified.
9. The `package.xml` maintainer is copied from `td_perception` (`parthtttrivedi@gmail.com`); confirm before review.
10. The packages' final home (`td_perception` vs here) is undecided.

## Open decisions (🧑)

- **LOC-01:** written TC confirmation of the IR policy and the division.
- **LOC-02:** `LocalizationState` message and frame contract.
- **The packages' home.**
- **The leg-odometry model source** (blocks LOC-15).

## Ready work

| Item | Label | Notes |
|---|---|---|
| LOC-18 feature projection | 🤖 | Deps done (LOC-03). Reuse `td_perception` `projection.py` (`project_to_ground`, `projection_covariance`); don't copy it. |
| CI job for `localization/` | 🤖 | `python run_tests.py` on PRs. Touches `.github/`; get the repo owner's OK. |
| Python 3.11 check | 🤖 | Run the suites; fix incompatibilities. |
| LOC-15 leg odometry in MuJoCo | 🤖/🔧 | Blocked on the model-source decision; can prototype. |
| Three-robot kidnap recovery | 🤖 | Improve beyond 2/4 seeds without breaking the consensus rule or the healthy-teammate result. |
| Hardware bench sessions | 🔧 | D435 tripod (`--camera realsense --odom stationary --head fixed:…`), GameController on the field network. Record with `--record`. |
| LOC-20 node skeleton | 🤖/🔧 | Blocked on LOC-02/11/17/18; the per-tick loop is defined in `02_architecture.md`. |

**Critical path:** LOC-13 (USB 3) + LOC-10 (head encoders) → LOC-14 (field bags) → LOC-17 (detector) → LOC-20
(node) → LOC-22 (on-robot validation).
