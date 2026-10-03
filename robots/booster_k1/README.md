# Booster K1

MuJoCo description of the [Booster Robotics K1](https://www.booster.tech/booster-k1/), the
22-DoF humanoid Booster lends to RoboCup Humanoid League teams. Vendored from Booster's
official [booster_assets](https://github.com/BoosterRobotics/booster_assets) (BSD-3-Clause,
see `LICENSE`). ProtoMotions loads it through `robocup_rl/robots/booster_k1.py` (see
"ProtoMotions integration").

## Files

| File                      | What it is                                                                                     |
| ------------------------- | ---------------------------------------------------------------------------------------------- |
| `booster_k1.xml`          | The robot: bodies, joints, inertials, meshes, collision geoms. Free root joint. No actuators.   |
| `booster_k1_actuated.xml` | Includes the robot and adds 22 position actuators. **This is the file ProtoMotions loads.** ProtoMotions ignores the kp/kv/forcerange here and uses the gains declared in `robocup_rl/robots/booster_k1.py`; a test keeps the two equal. Its empty `<worldbody/>` is where ProtoMotions' MuJoCo loader injects its own floor. |
| `scene.xml`               | Includes the actuated robot and adds a floor plane, a light and a `stand` keyframe, for standalone MuJoCo only. ProtoMotions never loads it. |
| `meshes/`                 | The 24 STL meshes the MJCF references, in metres, upstream names. 12 MB.                       |
| `LICENSE`                 | Booster's BSD-3-Clause license for the model and meshes.                                      |

View it (with `.venv-mujoco` active, from the repo root; pick the `stand` key, the model
spawns 1 m up):

```sh
python -m mujoco.viewer --mjcf robots/booster_k1/scene.xml
```

## Provenance and edits

`booster_k1.xml` is upstream `robots/K1/K1_22dof.xml` at commit `38a0ae84` (2026-08-28),
the serial-ankle model. Upstream also ships `K1_22dof_parallel.xml` (closed-chain ankle,
not RL-friendly), URDFs, a locomotion variant with fixed arms and head, and K1 motion CSVs;
none of those are vendored.

Edits versus upstream, all listed in the file header, so ProtoMotions can own the scene and
the actuation:

- model renamed `booster_k1`;
- the skybox and ground textures and material, the two lights and the ground plane removed
  (ProtoMotions adds its own floor and light, and IsaacLab would otherwise import the plane as
  part of the articulation);
- the two geomless, massless camera frame bodies removed: `head_realsense_rgb_link`
  (`pos="0.053617 -0.0115 0.10231" quat="0.42967577 -0.56158605 0.56158587 -0.42967563"`) and
  `head_booster_stereo_rgb_link` (`pos="0.060138 0.0351 0.092774" quat="0.50000016 -0.5
  0.49999984 -0.5"`), both children of `aahead_pitch_link`. They would get a contact sensor and
  a PhysX link with no mass on IsaacLab and add two useless slots to every body tensor. Put
  them back for perception work;
- the `<actuator>` block (22 torque motors) and the `<sensor>` block (IMU) removed;
  `booster_k1_actuated.xml` adds position actuators instead.

The `trunk` body tree (bodies, joints, inertials, geoms, meshes) is byte-identical to
upstream. Only the 24 meshes it references were copied; upstream's old feet, ZED head and
`*_Collision.STL` variants were not.

## Kinematic structure

```
world
└─ trunk                       free joint `world_joint`, 6.5 kg; site `imu`
   ├─ aahead_yaw_link          aahead_yaw_joint          axis z   [-1.012, 1.012]
   │  └─ aahead_pitch_link     aahead_pitch_joint        axis y   [-0.314, 0.794]
   ├─ aaleft_shoulder_pitch_link   aaleft_shoulder_pitch_joint   axis y  [-2.932, 1.196]
   │  └─ left_shoulder_roll_link   left_shoulder_roll_joint      axis x  [-1.642, 1.629]
   │     └─ left_elbow_pitch_link  left_elbow_pitch_joint        axis y  [-1.885, 1.885]
   │        └─ left_elbow_yaw_link left_elbow_yaw_joint          axis z  [-2.242, 0.816]
   ├─ aaright_shoulder_pitch_link  (mirror; right_elbow_yaw_joint [-0.816, 2.242])
   ├─ left_hip_pitch_link      left_hip_pitch_joint      axis y   [-2.958, 2.226]
   │  └─ left_hip_roll_link    left_hip_roll_joint       axis x   [-0.375, 1.536]
   │     └─ left_hip_yaw_link  left_hip_yaw_joint        axis z   [-1.012, 1.012]
   │        └─ left_knee_pitch_link    left_knee_pitch_joint    axis y   [0, 2.321]
   │           └─ left_ankle_pitch_link  left_ankle_pitch_joint  axis y  [-0.87, 0.345]
   │              └─ left_ankle_roll_link  left_ankle_roll_joint axis x  [-0.345, 0.345]   foot
   └─ right_hip_pitch_link     (mirror; right_hip_roll_joint [-1.536, 0.375])
```

22 hinge joints in that depth-first order (head 2, arms 4 × 2, legs 6 × 2), 23 bodies,
19.67 kg. Forward is +x: the hips are separated along y and the head cameras look along +x.
Visual meshes do not collide; collisions are primitive boxes and cylinders, the feet a
`0.18 × 0.07 × 0.036` m box whose sole is 0.038 m below the ankle roll link. With straight
legs the trunk origin sits 0.5517 m above the soles. The zero pose is a T-pose; Booster's
default joint state folds the arms down (`shoulder_roll` ∓1.3 rad).

Naming quirks, all upstream's: the head and shoulder-pitch names carry an `aa` prefix
(`aahead_yaw_joint`, `aaleft_shoulder_pitch_joint`), while Booster's `K1_JOINT_NAMES` for the
motion CSVs spell them without it (`left_shoulder_pitch_joint`); a retargeting step must map
names. Loading `scene.xml` gives 24 bodies incl. world, 23 joints, 22 actuators, 24 meshes.

## Gains and limits

Booster's own simulation defaults, as in
[booster_train](https://github.com/BoosterRobotics/booster_train) (`assets/robots/booster_k1.py`,
`actuator.py`): each joint is a second-order system on its motor armature,
`kp = armature · (2πf)²`, `kd = 2ζ · armature · 2πf`, with legs at f = 4 Hz and arms and head
at f = 10 Hz. Effort limits are upstream's `<motor forcerange>`, velocity limits the URDF's
`<limit velocity>`. Not verified hardware specifications.

| Joints                  | armature  | kp    | kd    | effort Nm | velocity rad/s |
| ----------------------- | --------- | ----- | ----- | --------- | -------------- |
| hip pitch (E6408)       | 0.0478125 | 30.20 | 3.605 | 68        | 14.66          |
| hip roll (E4315)        | 0.0339552 | 21.45 | 2.560 | 43        | 12.57          |
| hip yaw (E4310)         | 0.0282528 | 17.85 | 2.130 | 38.3      | 17.59          |
| knee pitch (E6416, ζ=1) | 0.095625  | 60.40 | 4.807 | 112       | 12.57          |
| ankle pitch             | 1.4 × E4310 | 24.98 | 2.982 | 38.3    | 17.59          |
| ankle roll              | 0.4 × E4310 | 7.14  | 0.852 | 38.3    | 17.59          |
| shoulders, elbows (R14) | 0.001     | 3.95  | 0.251 | 14        | 33.51          |
| head (HT4438)           | 0.001     | 3.95  | 0.251 | 6         | 7.85           |

The real ankle is a parallel linkage; Booster models its pitch and roll inertia on the serial
joints with the armature split above. `booster_k1.xml` still carries upstream's 0.0565 on both
ankle joints, but ProtoMotions applies the `ControlInfo.armature` from the Python config on
all three backends, so the split is what runs. Holding the default pose with these gains in
plain MuJoCo, the robot stays up but pitches a few degrees (they are soft on purpose; the
policy does the balancing).

## Known gaps

- Gains, limits and the ankle armature split are Booster's simulation values, not measured.
- `default_root_height` (0.552 m) is the geometric standing height; nothing has been settled
  under ProtoMotions on a GPU backend yet.
- No URDF is vendored, so the PyRoki retargeter is not set up for this robot either.
- Booster's K1 motion CSVs (50 Hz, root pose + 22 joint angles) are not vendored; they are
  the obvious source for the first mimic or AMP experiment.

## ProtoMotions integration

- **Config:** `robocup_rl/robots/booster_k1.py`
  ([`BoosterK1Config`](../../robocup_rl/robots/booster_k1.py)). It points at
  `booster_k1_actuated.xml` with an absolute `asset_root`, maps feet to the ankle roll links,
  hands to the elbow yaw links, head to `aahead_pitch_link` and torso and anchor to `trunk`,
  declares the gains above, `default_root_height` 0.552 m and the arms-down default pose.
  Contact sensors are on every body (`contact_bodies="all"`), as for the other robot.
- **Registration:** the `booster_k1` entry point in the root `pyproject.toml`, resolved by the
  hook in ProtoMotions' robot factory (see `docs/protomotions.md`). After `uv sync`,
  `--robot-name booster_k1` works everywhere.
- **Forward axis:** `(1, 0)`, confirmed by
  `python third-party/protomotions/scripts/identify_robot_facing_axis.py --robots booster_k1 --assert-declared`
  (hands and feet both give +X).
- **Tests:** `pytest tests/test_booster_k1.py` (entry point, kinematics, gains equal to the
  xml, MuJoCo and Newton steps, the steering experiment env); IsaacLab via
  `pytest tests/test_isaaclab.py --robot booster_k1`.
- **No USD needed.** IsaacLab converts the MJCF at runtime and caches it under `~/.cache`.
