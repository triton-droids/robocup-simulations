"""Smoke tests for the triton_humanoid ProtoMotions integration. CPU only; run in .venv-mujoco."""

import pytest
import torch

mujoco = pytest.importorskip("mujoco")

from protomotions.robot_configs.factory import robot_config  # noqa: E402

from robocup_rl.paths import ROBOTS_DIR  # noqa: E402
from robocup_rl.robots.triton_humanoid import TritonHumanoidConfig  # noqa: E402

JOINTS = [
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("hip1", "hip2", "thigh", "knee", "ankle")
]


def test_entry_point_resolves_through_upstream_factory():
    assert isinstance(robot_config("triton_humanoid"), TritonHumanoidConfig)
    with pytest.raises(ValueError, match="triton_humanoid"):
        robot_config("no_such_robot")


def test_kinematics():
    cfg = TritonHumanoidConfig()
    assert cfg.kinematic_info.num_dofs == 10
    assert cfg.number_of_actions == 10
    assert cfg.kinematic_info.num_bodies == 13
    assert cfg.kinematic_info.dof_names == JOINTS
    assert cfg.contact_bodies == ["left_foot", "right_foot"]
    assert cfg.non_termination_contact_bodies == ["left_foot", "right_foot"]
    assert cfg.anchor_body_name in cfg.kinematic_info.body_names


def test_gains_match_actuated_xml():
    """The xml's kp/kv/forcerange are ignored by ProtoMotions; keep them equal to the config."""
    cfg = TritonHumanoidConfig()
    model = mujoco.MjModel.from_xml_path(
        str(ROBOTS_DIR / "triton_humanoid" / "triton_humanoid_actuated.xml")
    )
    assert model.nu == len(JOINTS)
    for i in range(model.nu):
        joint = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[i, 0])
        info = cfg.control.control_info[joint]
        assert model.actuator_gainprm[i, 0] == pytest.approx(info.stiffness), joint
        assert -model.actuator_biasprm[i, 2] == pytest.approx(info.damping), joint
        assert model.actuator_forcerange[i, 1] == pytest.approx(info.effort_limit), joint


def test_mujoco_simulator_steps_without_falling_through_the_floor():
    from protomotions.components.scene_lib import SceneLib
    from protomotions.components.terrains.config import TerrainConfig
    from protomotions.components.terrains.terrain import Terrain
    from protomotions.simulator.factory import simulator_config
    from protomotions.utils.hydra_replacement import get_class

    device = torch.device("cpu")
    cfg = TritonHumanoidConfig()
    sim_cfg = simulator_config("mujoco", cfg, headless=True, num_envs=1, experiment_name="smoke")
    sim = get_class(sim_cfg._target_)(
        config=sim_cfg,
        robot_config=cfg,
        terrain=Terrain(config=TerrainConfig(), num_envs=1, device=device),
        device=device,
        scene_lib=SceneLib.empty(num_envs=1, device=device),
    )
    try:
        sim._initialize_with_markers({})
        sim.reset_envs(sim.get_default_robot_reset_state(), env_ids=torch.arange(1))
        for _ in range(20):
            sim.step(torch.zeros(1, cfg.number_of_actions))
        root_z = sim.get_root_state().root_pos[0, 2].item()
    finally:
        sim.close()
    # 20 steps at 50 Hz is 0.4 s; without a floor the root would have fallen ~0.8 m.
    assert torch.isfinite(torch.tensor(root_z))
    assert root_z > 0.3, f"root height {root_z:.3f}: did the MuJoCo loader inject its floor?"


def test_steering_experiment_env_steps_on_mujoco():
    """Build the env exactly as robocup_rl/experiments/steering/config.py configures it and step it."""
    import argparse

    from protomotions.components.motion_lib import MotionLib
    from protomotions.components.scene_lib import SceneLib
    from protomotions.components.terrains.terrain import Terrain
    from protomotions.envs.base_env.env import BaseEnv
    from protomotions.simulator.factory import simulator_config
    from protomotions.utils.hydra_replacement import get_class

    from robocup_rl.experiments.steering import config as steering

    device = torch.device("cpu")
    args = argparse.Namespace(batch_size=32, training_max_steps=64)
    cfg = TritonHumanoidConfig()
    sim_cfg = simulator_config("mujoco", cfg, headless=True, num_envs=1, experiment_name="smoke")
    terrain = Terrain(config=steering.terrain_config(args), num_envs=1, device=device)
    scene_lib = SceneLib(
        config=steering.scene_lib_config(args), num_envs=1, device=device, terrain=terrain
    )
    motion_lib = MotionLib.empty(device=device)
    assert steering.motion_lib_config(args).motion_file is None
    simulator = get_class(sim_cfg._target_)(
        config=sim_cfg, robot_config=cfg, terrain=terrain, scene_lib=scene_lib, device=device
    )
    env = BaseEnv(
        config=steering.env_config(cfg, args),
        robot_config=cfg,
        device=device,
        terrain=terrain,
        scene_lib=scene_lib,
        motion_lib=motion_lib,
        simulator=simulator,
    )
    try:
        env.reset(torch.arange(1))
        obs = env.get_obs()
        assert set(steering.OBS_KEYS) <= set(obs)
        for _ in range(10):
            obs, rewards, dones, terminated, extras = env.step(
                0.1 * torch.randn(1, cfg.number_of_actions)
            )
        assert torch.isfinite(rewards).all()
        agent_cfg = steering.agent_config(cfg, env.config, args)
        assert agent_cfg.model.actor.num_out == cfg.number_of_actions
    finally:
        simulator.close()
