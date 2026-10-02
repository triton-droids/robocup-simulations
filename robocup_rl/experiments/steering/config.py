"""Steering with plain PPO and no motion data.

The robot walks toward a target direction at a target speed; both are re-sampled
periodically by ProtoMotions' steering control component. No reference motions and no
discriminator are involved, so this is the first thing that runs on triton_humanoid
before any retargeted data exists. Expect clumsy gaits: nothing here rewards style.

Run from this directory so results/<run> lands next to this file (ProtoMotions writes to
results/<experiment-name> under the current working directory). Train on Newton (GPU);
batch_size must divide num_envs * num_steps (32 by default):

    cd robocup_rl/experiments/steering
    protomotions-train-agent --robot-name triton_humanoid --simulator newton \
        --num-envs 1024 --batch-size 4096 --motion-file none \
        --experiment-path config.py --experiment-name <run> --headless

--motion-file is required by the CLI but ignored by this file. Do not pass it at
inference: protomotions-inference-agent would then override motion_file with it.
The MuJoCo env is single-env and for inspecting and evaluating checkpoints, not training.
"""

import argparse

from protomotions.agents.ppo.config import PPOAgentConfig
from protomotions.envs.base_env.config import EnvConfig
from protomotions.robot_configs.base import RobotConfig
from protomotions.simulator.base_simulator.config import SimulatorConfig

OBS_KEYS = ["max_coords_obs", "steering"]


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
        steering_obs_factory,
        steering_reward_factory,
    )
    from protomotions.envs.control.steering_control import SteeringControlConfig

    return EnvConfig(
        max_episode_length=300,
        num_state_history_steps=0,
        control_components={"steering": SteeringControlConfig()},
        observation_components={
            "max_coords_obs": max_coords_obs_factory(
                local_obs=True, root_height_obs=True, observe_contacts=False
            ),
            "steering": steering_obs_factory(),
        },
        reward_components={"heading_rew": steering_reward_factory(weight=1.0)},
        termination_components={
            "fall": fall_termination_factory(termination_height=0.15),
            # A joint driven >1 rad past its limit is the first sign of the simulator diverging
            # (upstream's words). Resetting that env beats the non-finite-state assertion that
            # otherwise kills the whole run a few steps later; seen on Newton with saturated actions.
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
