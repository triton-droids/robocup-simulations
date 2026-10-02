"""ProtoMotions robot config for the Triton Droids humanoid.

The robot description is robots/triton_humanoid/ (see its README). ProtoMotions resolves
this class through the ``triton_humanoid`` entry point declared in the root pyproject.toml,
so ``--robot-name triton_humanoid`` works in every upstream script.
"""

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

# PD gains and limits, per joint group. ProtoMotions ignores the <position kp kv forcerange>
# values in triton_humanoid_actuated.xml and uses these instead (MuJoCo rewrites the
# actuators at load; Newton and IsaacLab build their own). They mirror the xml, and
# tests/test_triton_humanoid.py fails if the two drift apart. None of them are verified
# hardware specifications. VELOCITY_LIMIT has no counterpart in the xml.
STIFFNESS = {"hip1": 100.0, "hip2": 100.0, "thigh": 100.0, "knee": 80.0, "ankle": 20.0}
DAMPING = 1.0
EFFORT_LIMIT = 120.0
VELOCITY_LIMIT = 20.0
ARMATURE = 0.01  # joint armature in triton_humanoid.xml


def _control_info() -> Dict[str, ControlInfo]:
    return {
        f".*_{joint}_joint": ControlInfo(
            stiffness=kp,
            damping=DAMPING,
            effort_limit=EFFORT_LIMIT,
            velocity_limit=VELOCITY_LIMIT,
            armature=ARMATURE,
        )
        for joint, kp in STIFFNESS.items()
    }


@dataclass
class TritonHumanoidConfig(RobotConfig):
    # The actuated file, not the bare one: ProtoMotions' MuJoCo backend needs one actuator
    # per joint. asset_root is absolute, so it is pickled into resolved_configs.pt as-is.
    asset: RobotAssetConfig = field(
        default_factory=lambda: RobotAssetConfig(
            asset_root=str(ROBOTS_DIR),
            asset_file_name="triton_humanoid/triton_humanoid_actuated.xml",
            self_collisions=True,
            replace_cylinder_with_capsule=True,
            thickness=0.01,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
        )
    )

    # Forward direction in the root frame (identity at the zero pose). Read off the model:
    # the hips are separated along x, knee and ankle hinge about x, the toes point to +y and
    # knee flexion swings the foot to -y. Upstream's identify_robot_facing_axis.py cannot
    # confirm it because every body frame in this xml sits at the same point (CAD-frame
    # model, see the robot README's known gaps).
    semantic_forward_axis_xy: Tuple[float, float] = (0.0, 1.0)

    # All six keys are mandatory (RobotConfig.__post_init__). The robot has no head or hands:
    # head is read only by the path-follower task, which needs exactly one body; hands only by
    # identify_robot_facing_axis.py, which needs two distinct left/right bodies. Training
    # never reads either, so they point at the torso and the hip-roll links.
    common_naming_to_robot_body_names: Dict[str, List[str]] = field(
        default_factory=lambda: {
            "all_left_foot_bodies": ["left_foot"],
            "all_right_foot_bodies": ["right_foot"],
            "all_left_hand_bodies": ["left_leg1"],
            "all_right_hand_bodies": ["right_leg1"],
            "head_body_name": ["torso"],
            "torso_body_name": ["torso"],
        }
    )

    anchor_body_name: str = "torso"

    trackable_bodies_subset: List[str] = field(
        default_factory=lambda: [
            "torso_body_name",
            "all_left_foot_bodies",
            "all_right_foot_bodies",
        ]
    )

    contact_bodies: List[str] = field(
        default_factory=lambda: ["all_left_foot_bodies", "all_right_foot_bodies"]
    )

    # Real body names: this field is not expanded from the common-naming keys. Leaving the
    # default ("all") would make fall termination never fire.
    non_termination_contact_bodies: List[str] = field(
        default_factory=lambda: ["left_foot", "right_foot"]
    )

    # Root height when standing straight on the floor: floating_base sits at z=0.6846 in the
    # xml and scene.xml puts the floor at z=-0.0271 for the soles to touch it.
    default_root_height: float = 0.71

    # None -> all zeros, the CAD pose (straight legs). For a crouch use a regex dict, e.g.
    # {".*_hip2_joint": -0.2, ".*_knee_joint": -0.4, ".*_ankle_joint": 0.2} (unmatched -> 0).
    default_dof_pos: None = None

    control: ControlConfig = field(
        default_factory=lambda: ControlConfig(
            control_type=ControlType.BUILT_IN_PD,
            override_control_info=_control_info(),
        )
    )

    simulation_params: SimulatorParams = field(
        default_factory=lambda: SimulatorParams(
            mujoco=MujocoSimParams(fps=200, decimation=4),
            newton=NewtonSimParams(fps=200, decimation=4),
            isaaclab=IsaacLabSimParams(fps=200, decimation=4),
        )
    )
