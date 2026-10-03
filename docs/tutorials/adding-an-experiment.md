# Adding an experiment

This walks through `stand`, an experiment that rewards the robot for staying upright: plain
PPO, no motion data, no task command. It is the smallest useful experiment and the shape every
other one follows. The existing `robocup_rl/experiments/steering/` is the same thing plus a
steering command and reward.

## Anatomy

| File                                     | What                                                                                                     |
| ---------------------------------------- | -------------------------------------------------------------------------------------------------------- |
| `robocup_rl/experiments/<name>/__init__.py` | One-line docstring. Needed so tests can import the experiment as a package.                           |
| `.../<name>/config.py`                   | Passed to `--experiment-path`. Plain Python; ProtoMotions calls its hook functions, nothing is registered. |
| `.../<name>/rewards.py` (optional)       | Custom reward or termination kernels. See the pickling note below for why they do not go in `config.py`. |
| `.../<name>/results/<run>/`              | Checkpoints, configs, TensorBoard. Written when you launch from inside `<name>/`; already gitignored.    |

ProtoMotions loads `config.py` by path and calls, in order: `configure_robot_and_simulator`
(optional), `terrain_config`, `scene_lib_config`, `motion_lib_config`, `env_config`,
`agent_config`, and at inference `apply_inference_overrides`. The robot and the simulator are
not named in the file; they come from `--robot-name` and `--simulator`. Two more optional hooks,
`additional_experiment_arguments(parser)` and `configure_robot_and_simulator`, add CLI flags
and tweak the robot or sim config per experiment.

## 1. Create the directory

```sh
mkdir robocup_rl/experiments/stand
echo '"""Stand still: upright reward, fall termination, no motion data."""' > robocup_rl/experiments/stand/__init__.py
```

Import only with absolute paths (`from robocup_rl...`, `from protomotions...`). The loader
executes `config.py` under a throwaway module name, so relative imports do not resolve.

## 2. The reward: `rewards.py`

Rewards, terminations, and observations are all [`MdpComponent`](../../third-party/protomotions/protomotions/envs/mdp_component.py)s:

- `compute_func`: a pure tensor function returning `[num_envs]`.
- `dynamic_vars`: its tensor arguments, as [`EnvContext`](../../third-party/protomotions/protomotions/envs/context_views.py)`.<view>.<field>` paths
  resolved every step.
- `static_params`: constants, plus metadata the combiner reads and strips before the call:
  `weight`, `multiplicative`, `zero_during_grace_period`, `min_value`, `max_value`.

ProtoMotions pickles the resolved env config into `results/<run>/resolved_configs.pt`, and a
function defined in `config.py` cannot be pickled (its module is the throwaway name). Kernels
therefore live in a sibling module:

```python
"""Reward kernels for the stand experiment. Kept out of config.py on purpose: ProtoMotions
pickles the env config into results/<run>/resolved_configs.pt, and functions defined in
config.py cannot be pickled."""

import torch
from torch import Tensor

from protomotions.envs.obs import root_projected_gravity


def compute_upright_rew(anchor_rot: Tensor) -> Tensor:
    """1 when the torso's up axis points straight up, 0 when horizontal or worse.

    anchor_rot: anchor-body quaternion [num_envs, 4], xyzw. Returns [num_envs]."""
    gravity_local = root_projected_gravity(anchor_rot, w_last=True)  # (0, 0, -1) when upright
    return torch.clamp(-gravity_local[..., 2], min=0.0)
```

## 3. `config.py`

Copied from `steering/config.py`; the differences are the observation list, the reward, and
the missing steering control component. Everything else is unchanged.

```python
"""Stand still with plain PPO and no motion data.

The only reward is staying upright; the fall termination ends the episode when a non-foot body
touches the ground. Run from this directory so results/<run> lands next to this file:

    cd robocup_rl/experiments/stand
    protomotions-train-agent --robot-name triton_humanoid --simulator newton \
        --num-envs 1024 --batch-size 4096 --motion-file none \
        --experiment-path config.py --experiment-name <run> --headless

--motion-file is required by the CLI but ignored here. Do not pass it at inference.
"""

import argparse

from protomotions.agents.ppo.config import PPOAgentConfig
from protomotions.envs.base_env.config import EnvConfig
from protomotions.robot_configs.base import RobotConfig
from protomotions.simulator.base_simulator.config import SimulatorConfig

OBS_KEYS = ["max_coords_obs"]


def terrain_config(args: argparse.Namespace):
    from protomotions.components.terrains.config import TerrainConfig

    return TerrainConfig()


def scene_lib_config(args: argparse.Namespace):
    from protomotions.components.scene_lib import SceneLibConfig

    return SceneLibConfig(scene_file=None)


def motion_lib_config(args: argparse.Namespace):
    """Empty motion library on purpose; resets use the robot's default pose."""
    from protomotions.components.motion_lib import MotionLibConfig

    return MotionLibConfig(motion_file=None)


def env_config(robot_cfg: RobotConfig, args: argparse.Namespace) -> EnvConfig:
    from protomotions.envs.action import make_pd_action_config
    from protomotions.envs.component_factories import (
        fall_termination_factory,
        joint_limit_termination_factory,
        max_coords_obs_factory,
    )
    from protomotions.envs.context_views import EnvContext
    from protomotions.envs.mdp_component import MdpComponent

    from robocup_rl.experiments.stand.rewards import compute_upright_rew

    return EnvConfig(
        max_episode_length=300,
        num_state_history_steps=0,
        observation_components={
            "max_coords_obs": max_coords_obs_factory(
                local_obs=True, root_height_obs=True, observe_contacts=False
            ),
        },
        reward_components={
            "upright": MdpComponent(
                compute_func=compute_upright_rew,
                dynamic_vars={"anchor_rot": EnvContext.current.anchor_rot},
                static_params={"weight": 1.0},
            ),
        },
        termination_components={
            "fall": fall_termination_factory(termination_height=0.15),
            # A joint >1 rad past its limit means the simulator is diverging; reset that env.
            "joint_limit": joint_limit_termination_factory(
                dof_limits_lower=robot_cfg.kinematic_info.dof_limits_lower,
                dof_limits_upper=robot_cfg.kinematic_info.dof_limits_upper,
                max_violation=1.0,
            ),
        },
        action_config=make_pd_action_config(robot_cfg),
    )


def agent_config(
    robot_cfg: RobotConfig, env_cfg: EnvConfig, args: argparse.Namespace
) -> PPOAgentConfig:
    from protomotions.agents.common.config import MLPLayerConfig, MLPWithConcatConfig
    from protomotions.agents.ppo.config import PPOActorConfig, PPOModelConfig

    def mlp(out_keys, num_out):
        return MLPWithConcatConfig(
            in_keys=OBS_KEYS,
            out_keys=out_keys,
            num_out=num_out,
            normalize_obs=True,
            norm_clamp_value=5,
            layers=[MLPLayerConfig(units=256, activation="relu") for _ in range(2)],
        )

    return PPOAgentConfig(
        model=PPOModelConfig(
            in_keys=OBS_KEYS,
            actor=PPOActorConfig(
                num_out=robot_cfg.number_of_actions,
                actor_logstd=-2.9,
                in_keys=OBS_KEYS,
                mu_key="actor_trunk_out",
                mu_model=mlp(["actor_trunk_out"], robot_cfg.number_of_actions),
            ),
            critic=mlp(["value"], 1),
        ),
        batch_size=args.batch_size,
        training_max_steps=args.training_max_steps,
        gradient_clip_val=50.0,
        clip_critic_loss=True,
    )


def apply_inference_overrides(
    robot_cfg: RobotConfig,
    simulator_cfg: SimulatorConfig,
    env_cfg,
    agent_cfg,
    terrain_cfg,
    motion_lib_cfg,
    scene_lib_cfg,
    args: argparse.Namespace,
):
    if env_cfg is not None:
        env_cfg.max_episode_length = 1_000_000
```

- `weight` defaults to `0.0`. A term without one is logged but adds nothing to the reward.
- The termination is what makes "don't fall" learnable: reward stops accumulating at reset,
  so an upright robot collects more of it. Without `fall` the policy could lie down and still
  score whatever the kernel gives it.
- Each term shows up in W&B and TensorBoard as `raw_r/upright` and `scaled_r/upright`;
  terminations as `termination/fall` and `termination/joint_limit`.
- `--motion-file none` only satisfies the CLI. [`MotionLibConfig(motion_file=None)`](../../third-party/protomotions/protomotions/components/motion_lib.py) is what
  actually runs without motion data.
- `OBS_KEYS` is used twice: it names the observation components in `env_config` and the
  network inputs in `agent_config`. Keep them in sync or the agent will not find its inputs.

## 4. Smoke-check on CPU

Copy `test_steering_experiment_env_steps` in `tests/test_triton_humanoid.py`, swap the import
to `robocup_rl.experiments.stand`, and run it from the repo root in `.venv-mujoco` (dev extra):

```sh
pytest -k stand -v
```

It builds the env exactly as `config.py` configures it, steps random actions, and checks that
the observation keys exist, rewards are finite, the actor output size matches the robot, and
the fall termination fires when the robot folds. The same test's Newton case runs on the GPU
node and exercises the backend you train on.

## 5. Train and evaluate

Train on Newton (GPU) with `.venv-newton` active; the command is the one from the README:

```sh
cd robocup_rl/experiments/stand
protomotions-train-agent --robot-name triton_humanoid --simulator newton \
  --num-envs 1024 --batch-size 4096 --motion-file none \
  --experiment-path config.py --experiment-name <run> --headless \
  --training-max-iterations 1000 --use-wandb --wandb-project robocup-triton-humanoid
```

Pick a **new** `--experiment-name` after every edit to `config.py` or `rewards.py`. Re-using a
name resumes from the pickled config in `results/<run>/` and ignores the edit without failing.

Evaluate the checkpoint on CPU with `.venv-mujoco` active, from the same directory and without
`--motion-file`:

```sh
cd robocup_rl/experiments/stand
protomotions-inference-agent --checkpoint results/<run>/last.ckpt --simulator mujoco
```

## Going further

- More reward terms: `action_smoothness_factory` (needs `num_state_history_steps >= 1`),
  `pow_rew_factory`, and `steering_reward_factory` with a [`SteeringControlConfig`](../../third-party/protomotions/protomotions/envs/control/steering_control.py) control
  component, all in `third-party/protomotions/protomotions/envs/component_factories.py`.
- More inputs for a kernel: the `EnvContext.current.*` fields (`root_height`, `dof_vel`,
  `rigid_body_contacts`, ...) and the top-level `ground_heights`, `progress_buf`, `dt` in
  `third-party/protomotions/protomotions/envs/context_views.py`.
- Per-experiment robot or sim changes go in `configure_robot_and_simulator(robot_cfg,
  simulator_cfg, args)`; extra CLI flags in `additional_experiment_arguments(parser)`. Upstream's
  template is `third-party/protomotions/examples/experiments/format.py`.
