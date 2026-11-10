# 11. Where the project stands

**Series:** [index](../README.md) · [1 Welcome](01_welcome.md) · [2 Setup](02_setup.md) · [3 Geometry](03_math_geometry.md) ·
[4 Uncertainty](04_math_probability.md) · [5 Field](05_field_model.md) · [6 Camera](06_camera.md) ·
[7 Particle filter](07_particle_filter.md) · [8 Which half](08_side_belief.md) · [9 Game and team](09_game_and_team.md) ·
[10 Testing](10_testing_and_evaluation.md) · **11 Progress** · [12 Contributing](12_contributing.md)

*Status as of 2026-11-10. The live tracker is the status table in the [implementation plan](../../implementation_plan.md)
§6; when this chapter and the tracker disagree, the tracker wins.*

**In one paragraph:** the whole software core of localization exists and works **in simulation**:
- the field map and a camera simulator;
- the particle filter, with recovery from falls and pushes;
- the side belief that keeps the robot on the right half;
- the GameController client and team communication;
- an evaluation tool, a scenario suite and a bench for partial hardware.

All 237 tests pass. **None of it has run on the robot.** The next steps depend on hardware: measured head angles, a
working USB 3 camera link, recordings from a real field, and a real line detector.

---

## 11.1 Timeline

| Date | Milestone |
|---|---|
| 2026-10-26 | Review of all team repos: no localization of any kind existed. [Roadmap](../../roadmap.md) and [implementation plan](../../implementation_plan.md) written. |
| 2026-10-26 to 10-27 | Stage A, the offline core, built ticket by ticket: field model, simulator, particle filter, side belief, evaluation tool, GameController client, team communication. |
| 2026-10-27 | Team decision: the side consensus rule (goalkeeper or ≥ 2 teammates to flip). |
| 2026-10-27 to 10-29 | Bench harness for partial hardware, the visual-only (cuVSLAM) test, the test report. |
| 2026-10-29 | Moved to branch `localization` of `robocup-simulations` for team review. |

## 11.2 What's done

| Ticket | What | Where | Tests |
|---|---|---|---|
| LOC-03 | Field model: rule-book dimensions, lines, landmarks, likelihood field | `td_field_model/field.py` | part of 78 |
| LOC-04 | Synthetic simulator: camera projection, noise, scenarios (walks, falls, kidnaps, penalties) | `td_field_model/sim.py`, `scenarios.py` | part of 78 |
| LOC-05 | Particle filter with recovery, drift test and lost mode | `td_localization/pf.py` | part of 86 |
| LOC-06 | Side belief | `td_localization/side.py` | part of 86 |
| LOC-07 | Evaluation tool and scenario suite, with reference reports | `td_localization/eval.py`, `suite.py`, `reports/` | part of 86 |
| LOC-08 | GameController client (v7 wire format, events) and the bridge to localization | `td_game/`, `td_localization/game_bridge.py` | 42 + bridge tests |
| LOC-21 | Team communication: 22-byte messages, budget pacing, multi-robot simulation, consensus rule | `td_game/team_comm.py`, `td_localization/team_sim.py` | in both |
| LOC-25 | Bench harness: any mix of real, recorded and simulated inputs; record/replay; CLI; visual-only mix | `td_bench/` | 31 |

"Done" means **code + tests done, in simulation and on loopback networking**. The field test of team communication
(LOC-21) and everything hardware-related is still open.

## 11.3 Measured results (simulation)

All numbers are from simulation: they show the algorithm works under the modeled noise, not that it works on a real
field. The noise and detector models are educated guesses until real recordings exist (LOC-14).

### The scenario suite

M-Field, seed 1, from [`reports/suite_m_default_seed1/report.md`](../../../reports/suite_m_default_seed1/report.md)
and the `harsh` noise preset next to it:

| Scenario | Position p50 / p95 (default) | Converges after the event | Wrong half | Harsh noise p50 / p95 |
|---|---|---|---|---|
| `tracking` | 1.3 cm / 3.1 cm | (start known) | never | 15 cm / 20 cm |
| `global_sideline` (start unknown) | 1.6 cm / 20 cm | 1.1 s | never | 22 cm / 33 cm |
| `penalty_reentry` | 1.8 cm / 3.8 cm | (picks the right touchline) | never | 14 cm / 19 cm |
| `falls` | 1.6 cm / 8.0 cm | 1.4 s after a fall | never | 19 cm / 29 cm |
| `kidnap_same_half` | 2.0 cm / 1.38 m | 2.1 s | never | 21 cm / 1.33 m |
| `kidnap_cross_team` | 1.8 cm / 6.7 m | 2.7 s (teammates flip it) | 1.7 s in total | |
| `kidnap_cross_alone` | 2.8 m / 8.2 m | **never** (by design) | 20 s | |
| `team_kidnap_cross`, kidnapped robot | 2.0 cm / 6.4 m | 9.4 s | 9.3 s in total | |
| `team_kidnap_cross`, healthy teammate | 1.3 cm / 3.6 cm | | **never misled** | |

How to read it:

- **Tracking is far inside the targets** (p50 ≤ 20 cm, p95 ≤ 50 cm) with default noise. With harsh noise,
  including a 1° pitch calibration error, it's still inside them, but 10× worse: head-angle accuracy matters most
  ([chapter 6](06_camera.md)).
- **The kidnap scenarios "fail" p95** because their percentiles include the seconds right after the kidnap, before
  recovery. What matters there is the convergence time and wrong-half time.
- **The healthy teammate is never misled.** This was the point of the consensus rule.
- **`kidnap_cross_alone` is the documented limit:** a robot carried to its own mirror image, with no teammates, can't
  know it, and stays confidently wrong for the remaining 20 s ([chapter 8](08_side_belief.md), section 8.7).
- The kidnapped robot in the three-robot run recovers, but only after about 9 s. Across 4 seeds it recovered in 2.
  This is an open weakness (section 11.5).

### Speed

On a laptop, a full cycle (update + estimate + side) takes about 1 ms typically (median) and 14–16 ms at the 95th
percentile. The Jetson will be slower, and it shares the CPU with the ball detector, so this needs profiling there
(LOC-20).

### The visual-only test

From [`test_report/visual_only.html`](../../../test_report/visual_only.html): camera images through the line
detector, a **cuVSLAM model** as the only motion input, and head angles from an encoder + IMU model. 3 seeds × 30 s
each, start pose unknown:

| Variant | Converged | Error p50, last 10 s | Error p95, worst seed | Wrong half |
|---|---|---|---|---|
| Visual only (VO + camera + IMU head) | 3/3 | 3 cm | 13 cm | 0% |
| Slower head scan | 3/3 | 5 cm | 13 cm | 0% |
| Exact head angles | 3/3 | 1 cm | 5 cm | 0% |
| Camera + head, **no odometry** | **2/3** | 7 cm | **3.65 m** | **56%** (one seed ends mirrored) |
| Leg odometry instead of VO | 3/3 | 3 cm | 13 cm | 0% |

Takeaways, again about models, not hardware:

- some motion input is needed to stay on the right half;
- head-angle accuracy matters more than VO tracking losses;
- the default head scan (1.26 rad/s at its peak) is faster than the 1.2 rad/s at which the VO model loses
  tracking, so tracking drops about 20% of the time. On hardware, measure the real limit, and consider slowing the
  scan while VO is in use.

## 11.4 What isn't done

From the plan's status table. 🤖 = an agent can do it alone, 🔧 = needs hardware, 🧑 = needs a human decision.

| Ticket | What | Blocked by |
|---|---|---|
| LOC-01 🧑 | Confirm the IR policy and division with the HSL Technical Committee | People |
| LOC-02 🧑 | Interfaces and messages (`LocalizationState`, frames) | Team agreement |
| LOC-09 🤖/🔧 | Head joint filter (encoders + head IMU → head angles) | LOC-10 |
| LOC-10 🔧/🧑 | Head encoders and firmware | Parts and people |
| LOC-11 🤖/🔧 | Odometry EKF with fall/slip gating | LOC-15 |
| LOC-13 🔧 | USB 3 link for the camera (it currently enumerates as USB 2) | Hardware |
| LOC-14 🔧 | Field recordings with ground truth | LOC-13, LOC-10, a field |
| LOC-15 🤖/🔧 | Leg odometry (can start in MuJoCo) | Decision on the model source |
| LOC-16 🤖/🔧 | Floor-plane fit from depth | LOC-13 |
| LOC-17 🔧/🤖 | The real field-feature detector | LOC-14, LOC-16 |
| LOC-18 🤖 | Feature projection (reuse `td_perception`'s ground projection) | LOC-03 (done) |
| LOC-19 🔧 | cuVSLAM evaluation on the Jetson | LOC-09, LOC-11, LOC-13 |
| LOC-20 🤖/🔧 | The ROS 2 localization node and launch files | LOC-02, 11, 17, 18 |
| LOC-22 🔧 | Validation on the robot | LOC-20 |
| LOC-23 🧑/🤖 | Behavior integration (use the pose and side confidence) | LOC-20 |
| LOC-24 🧑/🤖 | Shared geometry module with the ball tracker | Ball v1 milestone |

**The critical path:** USB 3 (LOC-13) and head encoders (LOC-10) → field recordings (LOC-14) → feature detector
(LOC-17) → localization node (LOC-20) → validation on the robot (LOC-22).

## 11.5 Known limits and open questions

- **No hardware validation.** The noise models, the detector stand-in and the cuVSLAM model are all assumptions.
- **The mirror kidnap with no teammates** can't be detected (by design).
- **The three-robot kidnap recovery is slow and unreliable:** it recovers, but slowly, in 2 of 4 seeds.
- **The substitute rule is assumed:** whether a substitute enters like a penalty re-entry is unconfirmed (check the
  2027 rules).
- **Network code is tested on loopback only,** never against the official GameController or the field's access
  point.
- **The packages' final home is undecided:** they mirror `td_perception`'s layout and may move there.
- **Not tested on Python 3.11** (the rest of this repository's version), only on 3.10.
- **No continuous integration:** the repository's CI doesn't run these tests yet.
- **Rules interpretation:** the IR policy is stricter than the 2026 rule text, and is awaiting written confirmation
  (LOC-01).

## 11.6 What comes next

Work you could pick up now, roughly by how ready it is:

1. **LOC-18, feature projection** (🤖): ready, all its dependencies are done.
2. **A CI job for `localization/`** (🤖): run `python run_tests.py` on pull requests. It touches the repository's
   `.github/`, so agree it with the repo owners.
3. **A Python 3.11 check** (🤖): run the tests on 3.11 and fix anything that breaks.
4. **LOC-15, leg odometry in MuJoCo** (🤖/🔧): can start before the real encoders are calibrated.
5. **Bench sessions on hardware as parts arrive** (🔧): for example, a D435 on a tripod at a measured spot
   ([chapter 10](10_testing_and_evaluation.md), section 10.5) to exercise the real camera path.
6. **Decisions** (🧑): the interfaces (LOC-02), the TC confirmation (LOC-01), and where the packages finally live.

[Chapter 12](12_contributing.md) explains how to take one on.

---

**Next:** [12. How to contribute](12_contributing.md)
