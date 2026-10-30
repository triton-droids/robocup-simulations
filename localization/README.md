# localization

Field localization for the Triton Droids humanoid (RoboCup Humanoid Soccer League 2026, Large division):
where the robot is on the field, and which half it is on.

**Status: simulation and loopback only.** Everything here is tested against a synthetic simulator, rendered
camera images and UDP sockets on one machine. Nothing has run on the robot, a Jetson, a real D435, a real
GameController or the field network yet. The packages are pure Python (numpy only, no ROS imports) and use the
`td_perception` package layout (ament_python), so they can move there once the team decides where localization
lives. This folder is independent of the ProtoMotions training code in the rest of this repo.

## Why it's hard, and the approach

The HSL field is symmetric: from the lines alone, a robot can't tell its own half from the opponent's. Compasses
are banned, and the team policy keeps the D435's IR projector off. So the design
([docs/roadmap.md](docs/roadmap.md) §3.5) is:

- **A particle filter** (Monte Carlo localization) on field-line points projected from the camera, with
  odometry for motion. It keeps one side through continuity and never jumps sides on its own.
- **A side belief** decides which half the robot is on. It uses game events (initial positions, penalty
  re-entry from the GameController) and teammates' ball reports. A side flip needs the goalkeeper or ≥ 2
  teammates to agree; this is a team decision, see [docs/implementation_plan.md](docs/implementation_plan.md) §9.
- **Visual odometry** (cuVSLAM) is an optional extra motion input, never the field map.

## Layout

| Path | What |
|---|---|
| [`td_field_model/`](td_field_model/) | HSL field geometry from the rules JSON, likelihood field, mirror helpers, and the synthetic simulator: camera projection, scenarios with walks, falls, kidnaps and penalty re-entry. |
| [`td_localization/`](td_localization/) | Particle filter, side belief, evaluation tool and scenario suite, GameController bridge, multi-robot team simulation. |
| [`td_game/`](td_game/) | GameController client (v7 wire format), game events, team messages (22 bytes, budget pacer), UDP links. Standard library only. |
| [`td_bench/`](td_bench/) | Bench harness: runs the stack with each input (camera, odometry, head angles, GameController, teammates) real, recorded or simulated. Also record/replay and a CLI. |
| [`test_report/`](test_report/) | Generated HTML pages: every test beside its result, and the visual-only comparison. |
| [`reports/`](reports/) | Reference scenario-suite summaries (simulation, M field, seed 1). |
| [`docs/`](docs/) | Roadmap (findings, rules, design), implementation plan with tickets and status, figures, agent handoffs. |
| `run_tests.py` | Runs every package's tests and ruff. |

## Setup

Python 3.10 or newer; tested on 3.10 with numpy 1.21.5, to match Ubuntu 22.04 / ROS Humble. Use a venv of its
own, not the simulator envs at the repo root:

```bash
cd localization
python3 -m venv .venv-localization
source .venv-localization/bin/activate      # Windows: .venv-localization\Scripts\activate
pip install -r requirements.txt
python run_tests.py                         # 237 tests + ruff, about 5 minutes
```

`run_tests.py` runs each package in its own pytest process, because every package has a `test_import.py` and
pytest refuses duplicate module names in one run. To run one package by hand, put its dependencies on
`PYTHONPATH` (use `;` instead of `:` on Windows):

```bash
PYTHONPATH=td_field_model:td_game:td_localization:td_bench python -m pytest -q td_bench/test
```

Lint uses [`ruff.toml`](ruff.toml) here (line length 110, py310, the `td_perception` settings). It is not the
repo root config, and the repo's CI does not lint or test this folder yet.

## Results

- **[test_report/index.html](test_report/index.html):** every test, with its code and assertions beside its
  result. The bench runs include measured values against the pass conditions, the estimated path vs ground truth,
  and error over time. There is also a camera frame next to the line points detected from it. Regenerate it with
  `python test_report/make_results.py` (about 4 minutes).
- **[test_report/visual_only.html](test_report/visual_only.html):** the visual-only test.
  - **Setup:** camera images, a cuVSLAM model as the only motion input, and head angles from an encoder + head
    IMU model.
  - **Variants:** four more, which isolate head-scan speed, head-angle error and the value of VO.
  - **Result:** it meets the roadmap §3.3 targets on 3 of 3 seeds. Without any odometry, 1 of 3 seeds ends on
    the mirrored pose.
- **[reports/](reports/):** scenario-suite summaries (tracking, global localization from the sideline, falls,
  kidnaps, penalty re-entry, team recovery).

GitHub shows the HTML pages as source. Download them, or clone and open them locally.

## Bench testing with partial hardware

[`Bench`](td_bench/td_bench/bench.py) (CLI: `python -m td_bench`) drives the real localization code with each
input from its own source. Every input can be real, recorded or simulated, independently:

| Input | Simulated | Recorded | Real / live |
|---|---|---|---|
| camera | `sim` (simulator observations), `synthetic-images` (rendered frames → line detector) | `replay:FILE` | `realsense` (D435, projector forced off), `folder:DIR[@fps]` (saved frames, needs `--intrinsics`) |
| odometry | `sim` (legs), `vo` (cuVSLAM model), `none` (motion unknown) | `replay:FILE` | `stationary` (tripod); in Python, `LiveOdometry.push()` for encoders/IMU and `LiveVisualOdometry.push_pose()` for real cuVSLAM |
| head angles | `sim` (exact), `imu` (encoders + head IMU model) | `replay:FILE` | `fixed:YAW,PITCH` (tripod); `LiveHead.push()` |
| game events | `script:FILE.json` | `replay:FILE` | `udp:TEAM,PLAYER` (a real GameController) |
| teammates | `sim` | | `udp:TEAM,PLAYER` (real robots or another bench) |

The sources are in [`td_bench/td_bench/sources.py`](td_bench/td_bench/sources.py).

```bash
export PYTHONPATH=td_field_model:td_game:td_localization:td_bench
# Everything but the camera: real GameController + teammates on the network, simulated observations
python -m td_bench --camera sim --game udp:7,2 --team udp:7,2 --realtime --out reports/net
# Camera only: D435 on a tripod at a measured pose; record it for replay
python -m td_bench --camera realsense --odom stationary --head fixed:0,0.35 --world still --start known \
    --start-pose -3,1,0.3 --truth known:-3,1,0.3 --mount-height 1.2 --duration 60 \
    --record runs/tripod.npz --out reports/tripod
# Visual only: rendered images, cuVSLAM model, encoder/IMU head angles, start pose unknown
python -m td_bench --camera synthetic-images --odom vo --head imu --duration 30 --out reports/visual
# Replay a recording against changed code (bit-exact when nothing changed)
R=replay:runs/tripod.npz
python -m td_bench --camera $R --odom $R --head $R --game $R --truth $R --start known --world still --out reports/replay
```

Each run writes `est_bench.csv` and `bench_log.txt` to `--out`. With ground truth, it also writes `gt_bench.csv`
and an evaluation `report.md` / `results.json`. Recordings (`runs/`) and outputs (`reports/<name>`) are
git-ignored.

## Evaluating localization

```bash
export PYTHONPATH=td_field_model:td_game:td_localization
# Simulated scenario suite → reports/<dir>/report.md, results.json and per-scenario CSVs
python -m td_localization.eval suite --out reports/my_run
# Score estimate CSV(s) against a ground-truth CSV (formats in td_localization/td_localization/eval.py)
python -m td_localization.eval evaluate --gt gt.csv --est a.csv --est b.csv --out reports/cmp
```

## Known limits

- **No hardware validation yet:** no robot, Jetson, D435, GameController or field network. The bench is
  designed to make these checks easy once the hardware exists.
- **The image line detector** ([`td_bench/td_bench/vision.py`](td_bench/td_bench/vision.py)) is a bench
  stand-in that finds white pixels next to grass. The real field-feature detector is ticket LOC-17.
- **cuVSLAM is modelled**, not run: no GPU was available. Its parameters are assumptions from roadmap §3.6. Real
  output plugs in through `LiveVisualOdometry`.
- **A mirror kidnap with no teammates can't be detected,** by design: a symmetric field gives no evidence.
  Teammates or a game event resolve it.
- **No ROS nodes yet:** the localization node is ticket LOC-20.

## For agents and contributors

- Start with [docs/README.md](docs/README.md). Pick a ticket from the plan
  ([docs/implementation_plan.md](docs/implementation_plan.md)), check its dependencies, follow §0 and §7, and
  update the §6 status table.
- **Stop and ask a human** before:
  - changing a team decision (§9);
  - touching message definitions;
  - anything that commands motors;
  - turning the D435 projector on.
- **Handoffs:** read the newest one in [docs/handoffs/](docs/handoffs/) first. End each agent run with a new
  timestamped one there.
