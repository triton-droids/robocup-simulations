"""ProtoMotions robot config for the Booster K1 (22 DoF).

The robot description is robots/booster_k1/ (see its README). ProtoMotions resolves this
class through the ``booster_k1`` entry point declared in the root pyproject.toml, so
``--robot-name booster_k1`` works in every upstream script.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from protomotions.components.pose_lib import ControlInfo
from protomotions.robot_configs.base import (
    ControlConfig,
    ControlType,
    RobotAssetConfig,
    RobotConfig,
    SimulatorParams,
)
from protomotions.simulator.isaaclab.config import IsaacLabSimParams
from protomotions.simulator.mujoco.config import MujocoSimParams
from protomotions.simulator.newton.config import NewtonSimParams

from robocup_rl.paths import ROBOTS_DIR

# PD gains follow Booster's own simulation defaults (BoosterRobotics/booster_train,
# assets/robots/booster_k1.py and actuator.py): each joint group is a second-order system
# with natural frequency f and damping ratio zeta on the motor armature,
#   kp = armature * (2 pi f)^2,   kd = 2 * zeta * armature * 2 pi f.
# Legs run at 4 Hz, arms and head at 10 Hz. The ankle is a parallel linkage on the real
# robot; the serial model here takes Booster's per-axis armature split (1.4, 0.4) of the
# E4310 motor armature. Effort limits are the <motor forcerange> of upstream's MJCF and
# velocity limits the <limit velocity> of its URDF. None of these are verified hardware
# specifications; they are what Booster trains with.
#
# ProtoMotions ignores the <position kp kv forcerange> values in booster_k1_actuated.xml and
# the joint armature in booster_k1.xml and uses these instead (MuJoCo rewrites the actuators
# at load; Newton and IsaacLab build their own). tests/test_booster_k1.py fails if the xml
# and this file drift apart.

ARMATURE_E4310 = 0.0282528  # hip yaw motor; also the base of the ankle split

# joint regex (fullmatch) -> (armature, natural frequency Hz, damping ratio, effort Nm, velocity rad/s)
JOINT_GROUPS: Dict[str, Tuple[float, float, float, float, float]] = {
    r".*_hip_pitch_joint": (0.0478125, 4.0, 1.5, 68.0, 14.66),
    r".*_hip_roll_joint": (0.0339552, 4.0, 1.5, 43.0, 12.57),
    r".*_hip_yaw_joint": (ARMATURE_E4310, 4.0, 1.5, 38.3, 17.59),
    r".*_knee_pitch_joint": (0.095625, 4.0, 1.0, 112.0, 12.57),
    r".*_ankle_pitch_joint": (1.4 * ARMATURE_E4310, 4.0, 1.5, 38.3, 17.59),
    r".*_ankle_roll_joint": (0.4 * ARMATURE_E4310, 4.0, 1.5, 38.3, 17.59),
    r".*_(shoulder|elbow)_.*_joint": (0.001, 10.0, 2.0, 14.0, 33.51),
    r"aahead_(yaw|pitch)_joint": (0.001, 10.0, 2.0, 6.0, 7.85),
}


def pd_gains(armature: float, natural_freq_hz: float, damping_ratio: float) -> Tuple[float, float]:
    """Booster's (stiffness, damping) for a joint, see the module comment."""
    omega = 2.0 * math.pi * natural_freq_hz
    return armature * omega * omega, 2.0 * damping_ratio * armature * omega


def _control_info() -> Dict[str, ControlInfo]:
    info = {}
    for pattern, (armature, freq, zeta, effort, velocity) in JOINT_GROUPS.items():
        stiffness, damping = pd_gains(armature, freq, zeta)
        info[pattern] = ControlInfo(
            stiffness=stiffness,
            damping=damping,
            effort_limit=effort,
            velocity_limit=velocity,
            armature=armature,
        )
    return info


@dataclass
class BoosterK1Config(RobotConfig):
    # The actuated file, not the bare one: ProtoMotions' MuJoCo backend needs one actuator
    # per joint. asset_root is absolute, so it is pickled into resolved_configs.pt as-is.
    asset: RobotAssetConfig = field(
        default_factory=lambda: RobotAssetConfig(
            asset_root=str(ROBOTS_DIR),
            asset_file_name="booster_k1/booster_k1_actuated.xml",
            self_collisions=True,
            replace_cylinder_with_capsule=True,
            thickness=0.01,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
        )
    )

    # Forward is +x: the hips are separated along y and the head cameras look along +x.
    # Confirmed by upstream's scripts/identify_robot_facing_axis.py --robots booster_k1.
    semantic_forward_axis_xy: Tuple[float, float] = (1.0, 0.0)

    # All six keys are mandatory (RobotConfig.__post_init__). The elbow-yaw links are the
    # last arm links (the K1 has no wrists or hands).
    common_naming_to_robot_body_names: Dict[str, List[str]] = field(
        default_factory=lambda: {
            "all_left_foot_bodies": ["left_ankle_roll_link"],
            "all_right_foot_bodies": ["right_ankle_roll_link"],
            "all_left_hand_bodies": ["left_elbow_yaw_link"],
            "all_right_hand_bodies": ["right_elbow_yaw_link"],
            "head_body_name": ["aahead_pitch_link"],
            "torso_body_name": ["trunk"],
        }
    )

    anchor_body_name: str = "trunk"

    trackable_bodies_subset: List[str] = field(
        default_factory=lambda: [
            "torso_body_name",
            "head_body_name",
            "all_left_foot_bodies",
            "all_right_foot_bodies",
            "all_left_hand_bodies",
            "all_right_hand_bodies",
        ]
    )

    # Every body: Newton and IsaacLab only create contact sensors for the bodies listed
    # here, and fall termination fires only when a body *outside*
    # non_termination_contact_bodies reports contact. 23 sensors is cheap.
    contact_bodies: str = "all"

    # Real body names: this field is not expanded from the common-naming keys.
    non_termination_contact_bodies: List[str] = field(
        default_factory=lambda: ["left_ankle_roll_link", "right_ankle_roll_link"]
    )

    # Trunk height with straight legs and the soles on the floor: the link offsets from
    # the trunk to the ankle roll link sum to 0.5137 m and the foot collision box reaches
    # 0.038 m below it (0.5517 m total). Booster's own init state drops it from 0.57 m.
    default_root_height: float = 0.552

    # Booster's default joint state: arms hanging at the sides instead of the model's
    # T-pose. Regex dict resolved against dof_names in RobotConfig.__post_init__;
    # unmatched joints are 0.
    default_dof_pos: Dict[str, float] = field(
        default_factory=lambda: {
            "left_shoulder_roll_joint": -1.3,
            "right_shoulder_roll_joint": 1.3,
        }
    )

    control: ControlConfig = field(
        default_factory=lambda: ControlConfig(
            control_type=ControlType.BUILT_IN_PD,
            override_control_info=_control_info(),
        )
    )

    # 50 Hz control, as Booster trains it (they use 500 Hz physics; 200/4 is what the other
    # robot here uses and what the backends are tuned for).
    simulation_params: SimulatorParams = field(
        default_factory=lambda: SimulatorParams(
            mujoco=MujocoSimParams(fps=200, decimation=4),
            newton=NewtonSimParams(fps=200, decimation=4),
            isaaclab=IsaacLabSimParams(fps=200, decimation=4),
        )
    )
