"""Smoke tests for the booster_k1 ProtoMotions integration.

Same shape as tests/test_triton_humanoid.py: config and kinematics on CPU, the simulator
tests parametrized over mujoco (CPU) and newton (GPU, skips otherwise) through the fixtures
in conftest.py. IsaacLab: `pytest tests/test_isaaclab.py --robot booster_k1`.
"""

import argparse

import pytest
import torch

mujoco = pytest.importorskip("mujoco")

from protomotions.robot_configs.factory import robot_config  # noqa: E402

from robocup_rl.paths import ROBOTS_DIR  # noqa: E402
from robocup_rl.robots.booster_k1 import BoosterK1Config, pd_gains  # noqa: E402

from conftest import Backend, build_simulator  # noqa: E402

# Depth-first order of the MJCF: head, left arm, right arm, left leg, right leg. The `aa`
# prefixes are upstream's.
JOINTS = (
    ["aahead_yaw_joint", "aahead_pitch_joint"]
    + [
        f"{p}{side}_{joint}_joint"
        for side in ("left", "right")
        for p, joint in (
            ("aa", "shoulder_pitch"),
            ("", "shoulder_roll"),
            ("", "elbow_pitch"),
            ("", "elbow_yaw"),
        )
    ]
    + [
        f"{side}_{joint}_joint"
        for side in ("left", "right")
        for joint in ("hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch", "ankle_roll")
    ]
)
FEET = ["left_ankle_roll_link", "right_ankle_roll_link"]


def test_entry_point_resolves_through_upstream_factory():
    assert isinstance(robot_config("booster_k1"), BoosterK1Config)


def test_kinematics():
    cfg = BoosterK1Config()
    assert cfg.kinematic_info.num_dofs == 22
    assert cfg.number_of_actions == 22
    assert cfg.kinematic_info.num_bodies == 23
    assert cfg.kinematic_info.dof_names == JOINTS
    # All bodies get contact sensors; only the feet may touch the ground without terminating.
    assert cfg.contact_bodies == cfg.kinematic_info.body_names
    assert cfg.non_termination_contact_bodies == FEET
    assert set(FEET) <= set(cfg.kinematic_info.body_names)
    assert cfg.anchor_body_name in cfg.kinematic_info.body_names
    # Arms down at the sides, everything else at zero.
    default = dict(zip(JOINTS, cfg.default_dof_pos.tolist()))
    assert default.pop("left_shoulder_roll_joint") == pytest.approx(-1.3)
    assert default.pop("right_shoulder_roll_joint") == pytest.approx(1.3)
    assert all(v == 0.0 for v in default.values())


def test_every_joint_has_control_info():
    """A regex that misses the `aa`-prefixed joints would leave them without gains."""
    cfg = BoosterK1Config()
    for joint in JOINTS:
        info = cfg.control.control_info[joint]
        assert info.stiffness > 0 and info.damping > 0 and info.effort_limit > 0, joint
        assert info.velocity_limit > 0 and info.armature > 0, joint
    # Spot-check the derivation against Booster's published ankle numbers (24.98 / 7.14 Nm/rad).
    assert pd_gains(1.4 * 0.0282528, 4.0, 1.5)[0] == pytest.approx(24.98, abs=0.01)
    assert pd_gains(0.4 * 0.0282528, 4.0, 1.5)[0] == pytest.approx(7.14, abs=0.01)


def test_gains_match_actuated_xml():
    """The xml's kp/kv/forcerange are ignored by ProtoMotions; keep them equal to the config.
    The xml carries four decimals, hence the tolerance."""
    cfg = BoosterK1Config()
    model = mujoco.MjModel.from_xml_path(str(ROBOTS_DIR / "booster_k1" / "booster_k1_actuated.xml"))
    assert model.nu == len(JOINTS)
    for i in range(model.nu):
        joint = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, model.actuator_trnid[i, 0])
        info = cfg.control.control_info[joint]
        assert model.actuator_gainprm[i, 0] == pytest.approx(info.stiffness, rel=1e-3), joint
        assert -model.actuator_biasprm[i, 2] == pytest.approx(info.damping, rel=1e-3), joint
        assert model.actuator_forcerange[i, 1] == pytest.approx(info.effort_limit), joint


# --- simulator-backed tests -------------------------------------------------------------


def test_simulator_steps_without_falling_through_the_floor(backend: Backend):
    cfg = BoosterK1Config()
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
    """Newton matches our DOF names to the MJCF joints, then assumes the builder's DOF order
    equals ours. Check both the resolved mapping and the gains it wrote per DOF."""
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
    cfg = BoosterK1Config()
    terrain = Terrain(config=steering.terrain_config(args), num_envs=n, device=device)
    scene_lib = SceneLib(
        config=steering.scene_lib_config(args), num_envs=n, device=device, terrain=terrain
    )
    motion_lib = MotionLib.empty(device=device)
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

        # Zero actions target the middle of every joint range (knees at 1.16 rad, hips pitched
        # back), which folds the robot onto the ground. Fall termination must notice: it needs
        # contact flags on non-foot bodies, which the GPU backends only provide for bodies in
        # robot_cfg.contact_bodies.
        fell = torch.zeros(n, dtype=torch.bool, device=device)
        for _ in range(150):
            _, _, _, terminated, _ = env.step(torch.zeros(n, cfg.number_of_actions, device=device))
            fell |= terminated.bool()
            if fell.all():
                break
        assert fell.all(), "fall termination never fired; are contact sensors on all bodies?"
    finally:
        simulator.close()
