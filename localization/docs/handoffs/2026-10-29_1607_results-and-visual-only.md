# Handoff: results report + visual-only test (cuVSLAM + camera + IMU head), 2026-10-29 16:07

Local only. Nothing was pushed, and no team repos were touched. The previous handoff is
`2026-10-27_0512_bench-harness.md`.

## Requests

1. "Show the test results in a new folder called 'results'. Show the test and result as a side by side
   comparison."
2. "Run a visual-only test (cuVSLAM+Camera+IMU/Head Angle)".

## What exists now

**`localization_work/results/`**
- `index.html`: every test (237 across 4 packages), with each test (docstring, assertions, full code) beside its
  result (status, time, measured values).
  - The td_bench hardware-mix runs get cards: inputs tagged real / sim / recorded, measured vs expected, the
    estimate-vs-truth path on the field, and an error-over-time chart.
  - The vision card shows the camera image beside the detected points on the field, plus a version with the head
    pitch 6° wrong.
- `visual_only.html`: the visual-only test plus 4 comparison variants, each on 3 seeds × 30 s. Plots show the
  worst seed.
- `make_results.py`:
  - Runs all suites; the subprocess packages run in parallel, and td_bench runs in-process with `Bench.run`
    wrapped to capture runs for plotting.
  - Writes `index.html` and `raw/*.xml`.
  - About 4 minutes.
- `visual_only.py`: about 8 minutes; also writes `raw/visual_only.json`.
- `README.md` explains all of the above.
- Measured values come from `show(record_property, ...)` in td_bench tests (`td_bench/test/bench_report.py`),
  through the JUnit XML properties (xunit1).
- Viewing: the browser pane could not open `file://`, so `CustomHumanoid/.claude/launch.json` defines a
  `results` server (python http.server on 127.0.0.1:8765, serving `localization_work/results`). Opening
  `index.html` directly in a normal browser also works; it is self-contained.

**New in `td_bench/sources.py`**
- `SimVisualOdometry`: the cuVSLAM stereo-inertial model, based on roadmap §3.6.
  - 3% scale error, 5% translation noise, 1.5°/min gyro bias.
  - Head motion is subtracted using the *believed* head angles, so head-angle error leaks into body yaw.
  - Tracking drops while the head pans faster than 1.2 rad/s, plus random 1 s losses (2% per second). While lost,
    it reports zero motion with walking-size covariance.
- `LiveVisualOdometry.push_pose(x, y, yaw, tracking)`: the input for real cuVSLAM output. It converts poses to
  increments and re-references after a loss.
- `ImuHead`: head angles from the encoder + head IMU model, with a per-run bias (σ 0.5°) and noise (0.2°).
- CLI: `--odom vo|none` and `--head imu`.

**Tests:** td_bench went from 25 to 31 tests.
- 5 source unit tests:
  - VO follows the motion while tracking.
  - VO drops tracking on fast pans.
  - Head error leaks into yaw.
  - Live VO increments and tracking loss.
  - ImuHead bias.
- 1 end-to-end test, `test_visual_only_cuvslam_camera_imu_head`.

The full run is 237/237 passing (td_bench 31, td_field_model 78, td_localization 86, td_game 42), and lint is
clean. Bug fixed along the way: VO head subtraction compared against the last *tracked* head angle, which gave
huge yaw jumps after a tracking loss.

## Visual-only results (3 seeds × 30 s, walking, start pose unknown, no legs / GameController / team / ball)

| Variant | Converged | Err p50, last 10 s (median) | Err p95 (worst seed) | Wrong half (worst) | VO lost |
|---|---|---|---|---|---|
| **Visual only (VO + camera + IMU head)** | 3/3 | 0.03 m | 0.13 m | 0% | 20% |
| Slower head scan | 3/3 | 0.05 m | 0.13 m | 0% | 0–3% |
| Exact head angles | 3/3 | 0.01 m | 0.05 m | 0% | 20% |
| Camera + head, no odometry | **2/3** | 0.07 m | **3.65 m** | **56%** (seed 2 ends mirrored, heading 180°) | – |
| Reference: leg odometry | 3/3 | 0.03 m | 0.13 m | 0% | – |

Takeaways (all are statements about the models; none is hardware evidence):
- **Visual-only localization meets the §3.3 targets** in simulation. Convergence takes 1.5 s (median), the
  heading p95 is at most 1.5°, and there is no wrong-half time.
- **Visual odometry is what prevents the mirror failure.** Without any motion input, 1 of 3 seeds locks onto the
  mirrored pose with high side confidence.
- **Head-angle error costs more than VO tracking loss.** Exact head angles cut the error 3×; a slower scan
  removes the losses but doesn't change accuracy much.
- **The default head scan (peak 1.26 rad/s) is above the assumed 1.2 rad/s VO pan gate**, so tracking drops
  about 20% of the time. On hardware: measure the real gate, and consider capping the pan rate while VO is used.

## Not done / caveats

- **cuVSLAM did not run.** This machine has no NVIDIA GPU and no Isaac ROS. Everything VO-related is a model, and
  its numbers (scale, noise, gate) are assumptions taken from §3.6, not measurements. LOC-19 on a Jetson is still
  required. Recording `/visual_slam/tracking/odometry` and feeding it through `LiveVisualOdometry` is the bridge.
- Leg odometry is also a model, so the VO-vs-legs comparison is only a sanity check.
- Camera images are rendered: no lighting, robots or crowd. The line detector is still the bench stand-in, not
  LOC-17.

## Docs updated

- `docs/localization_implementation_plan.md` §6: the LOC-19 note and the LOC-25 row (31 tests).
- `localization_work/README.md`: the results section, the visual-only section, and the new CLI options in the
  source table.
- `results/README.md`.
