"""Smoke tests for the triton_humanoid ProtoMotions integration.

The config and kinematics tests are CPU-only and run in .venv-mujoco. The simulator tests are
parametrized over the backends: the mujoco case runs on CPU anywhere, the newton case needs
the `newton` extra and a CUDA device (run `pytest -v tests/` on a GPU machine) and skips
otherwise. The `backend` fixture and `build_simulator` live in conftest.py. IsaacLab has its own
file (tests/test_isaaclab.py) because it must start Isaac Sim before torch is imported.
"""

import argparse

import pytest
import torch

mujoco = pytest.importorskip("mujoco")

from protomotions.robot_configs.factory import robot_config  # noqa: E402

from robocup_rl.paths import ROBOTS_DIR  # noqa: E402
from robocup_rl.robots.triton_humanoid import TritonHumanoidConfig  # noqa: E402

from conftest import Backend, build_simulator  # noqa: E402

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
    # All bodies get contact sensors; only the feet may touch the ground without terminating.
    assert cfg.contact_bodies == cfg.kinematic_info.body_names
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


# --- simulator-backed tests -------------------------------------------------------------


def test_simulator_steps_without_falling_through_the_floor(backend: Backend):
    cfg = TritonHumanoidConfig()
    sim, _, _ = build_simulator(backend, cfg)
    try:
        sim._initialize_with_markers({})
        sim.reset_envs(
            sim.get_default_robot_reset_state(),
            env_ids=torch.arange(backend.num_envs, device=backend.device),
        )
        for _ in range(20):
            sim.step(torch.zeros(backend.num_envs, cfg.number_of_actions, device=backend.device))
        root_z = sim.get_root_state().root_pos[:, 2].cpu()
        if backend.name == "newton":
            _check_newton_dof_mapping(sim, cfg)
    finally:
        sim.close()
    # 20 steps at 50 Hz is 0.4 s; without a floor the root would have fallen ~0.8 m.
    assert torch.isfinite(root_z).all()
    assert (root_z > 0.3).all(), f"root heights {root_z.tolist()}: is the floor missing?"


def _check_newton_dof_mapping(sim, cfg):
    """Newton matches our DOF names to the MJCF joints by substring, then assumes the builder's
    DOF order equals ours. Check both the resolved mapping and the gains it wrote per DOF."""
    assert list(sim._newton_dof_names) == JOINTS
    assert list(sim.robot_view.joint_names) == JOINTS
    # Builder DOFs 0-5 are the free joint; 6.. are ours, in order.
    for i, joint in enumerate(JOINTS):
        info = cfg.control.control_info[joint]
        assert sim.robot.joint_target_ke[6 + i] == pytest.approx(info.stiffness), joint
        assert sim.robot.joint_target_kd[6 + i] == pytest.approx(info.damping), joint
        assert sim.robot.joint_effort_limit[6 + i] == pytest.approx(info.effort_limit), joint
    assert sim.get_dof_state().dof_pos.shape == (sim.num_envs, len(JOINTS))


def test_steering_experiment_env_steps(backend: Backend):
    """Build the env exactly as robocup_rl/experiments/steering/config.py configures it and step it."""
    from protomotions.components.motion_lib import MotionLib
    from protomotions.components.scene_lib import SceneLib
    from protomotions.components.terrains.terrain import Terrain
    from protomotions.envs.base_env.env import BaseEnv

    from robocup_rl.experiments.steering import config as steering

    n, device = backend.num_envs, backend.device
    args = argparse.Namespace(batch_size=32, training_max_steps=64)
    cfg = TritonHumanoidConfig()
    terrain = Terrain(config=steering.terrain_config(args), num_envs=n, device=device)
    scene_lib = SceneLib(
        config=steering.scene_lib_config(args), num_envs=n, device=device, terrain=terrain
    )
    motion_lib = MotionLib.empty(device=device)
    assert steering.motion_lib_config(args).motion_file is None
    simulator, _, _ = build_simulator(backend, cfg, terrain=terrain, scene_lib=scene_lib)
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
        env.reset(torch.arange(n, device=device))
        obs = env.get_obs()
        assert set(steering.OBS_KEYS) <= set(obs)
        for _ in range(10):
            obs, rewards, dones, terminated, extras = env.step(
                0.1 * torch.randn(n, cfg.number_of_actions, device=device)
            )
        assert rewards.shape == (n,)
        assert torch.isfinite(rewards).all()
        agent_cfg = steering.agent_config(cfg, env.config, args)
        assert agent_cfg.model.actor.num_out == cfg.number_of_actions

        # Zero actions target the middle of every joint range (knees at -1.05 rad), which folds
        # the robot onto the ground within ~40 control steps. Fall termination must notice:
        # it needs contact flags on non-foot bodies, which the GPU backends only provide for
        # bodies in robot_cfg.contact_bodies.
        fell = torch.zeros(n, dtype=torch.bool, device=device)
        for _ in range(150):
            _, _, _, terminated, _ = env.step(torch.zeros(n, cfg.number_of_actions, device=device))
            fell |= terminated.bool()
            if fell.all():
                break
        assert fell.all(), "fall termination never fired; are contact sensors on all bodies?"
    finally:
        simulator.close()
