# robocup_rl

Everything RoboCup-specific that ProtoMotions needs from us: robot configs, experiment
files, and the glue that registers them. Robot descriptions (MJCF, meshes) stay in
`../robots/`; this package only points at them. Installed editable by `uv sync` from the
repo root (see the top-level README).

```
robocup_rl/
  paths.py                 REPO_ROOT and ROBOTS_DIR, derived from this file's location
  robots/<robot>.py        one RobotConfig subclass per robot
  experiments/<name>/      one directory per experiment
    config.py              passed to --experiment-path
    results/<run>/         training outputs, written when you launch from inside <name>/ (gitignored)
```

Tests live in `../tests/` and run with `pytest` from the repo root.

## How a robot gets registered

ProtoMotions resolves `--robot-name` through `robot_config()` in
`third-party/protomotions/protomotions/robot_configs/factory.py`. Upstream's version only
knows its six built-in robots. Our local change to that file makes the fallback branch look
the name up in the `protomotions.robots` entry-point group, and the root `pyproject.toml`
declares our robots there:

```toml
[project.entry-points."protomotions.robots"]
triton_humanoid = "robocup_rl.robots.triton_humanoid:TritonHumanoidConfig"
booster_k1 = "robocup_rl.robots.booster_k1:BoosterK1Config"
```

The entry point is a zero-argument callable returning a [`RobotConfig`](../third-party/protomotions/protomotions/robot_configs/base.py); a dataclass subclass
with field defaults is exactly that. Entry points are written at install time, so after
adding or renaming one run `uv sync` (or `uv pip install --no-deps -e .`) again, or the
factory will report the robot as unknown.

## Adding a robot

1. Put the MJCF and meshes in `../robots/<robot>/`. ProtoMotions' MuJoCo backend needs one
   actuator per joint, a free root joint (`<freejoint/>` or `<joint type="free"/>`) on the only
   child of `<worldbody>`, and a `<worldbody>` element in the file it loads (it injects its
   floor and light there). Strip any floor, lights and sensors the vendor's file carries;
   `../robots/booster_k1/` shows the edits for a stock MJCF.
2. Write `robots/<robot>.py`, mirroring `robots/triton_humanoid.py`. Use an absolute
   `asset_root` (`paths.ROBOTS_DIR`). Fill all six `common_naming_to_robot_body_names` keys
   even if the robot lacks the body; set `non_termination_contact_bodies` to real body
   names or falls never terminate; declare PD gains with `override_control_info` regexes,
   because gains in the MJCF are ignored.
3. Add the entry-point line to the root `pyproject.toml` and re-sync.
4. Confirm `semantic_forward_axis_xy` with
   `python third-party/protomotions/scripts/identify_robot_facing_axis.py --robots <robot>`
   (it needs distinct left/right hand bodies and body frames that are not all coincident).
5. Add a test next to `../tests/test_triton_humanoid.py`.

## Experiments

An experiment file is plain Python loaded by path. ProtoMotions calls, in order:
`configure_robot_and_simulator` (optional), `terrain_config`, `scene_lib_config`,
`motion_lib_config`, `env_config`, `agent_config`, and at inference
`apply_inference_overrides` with eight positional arguments. `experiments/steering/config.py`
is the template: plain PPO, no motion data ([`MotionLibConfig(motion_file=None)`](../third-party/protomotions/protomotions/components/motion_lib.py)), a fall
termination and the steering reward;
[docs/tutorials/adding-an-experiment.md](../docs/tutorials/adding-an-experiment.md) walks
through adding one. Import from `robocup_rl` and `protomotions` with absolute paths only; the
loader executes the file under a throwaway module name.

ProtoMotions writes everything (configs, checkpoints, TensorBoard) to
`results/<experiment-name>` under the **current working directory**, with no flag to change
it. Launch from inside the experiment directory so each experiment keeps its own runs:

```sh
cd robocup_rl/experiments/<experiment>
protomotions-train-agent --robot-name triton_humanoid --simulator newton \
  --num-envs 1024 --batch-size 4096 --motion-file none \
  --experiment-path config.py --experiment-name <run> --headless
```

`--robot-name` is any registered robot (`triton_humanoid`, `booster_k1`); the experiment
file reads everything it needs from `robot_cfg`. Train on Newton or IsaacLab (GPU); the
MuJoCo env is single-env and for inspection and evaluation only. The CLI still requires
`--motion-file`; pass `none` for motion-free experiments and leave it out at inference. `batch_size` must divide `num_envs * num_steps`
(32 by default).
