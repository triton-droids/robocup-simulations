# Triton humanoid

MuJoCo description of the Triton Droids RoboCup humanoid: a 10-DOF, legs-only biped
with a floating base. This directory is the single source of truth for the robot model;
nothing under `third-party/` knows about it yet (see "What ProtoMotions still needs").

## Files

| File                           | What it is                                                                                     |
| ------------------------------ | ---------------------------------------------------------------------------------------------- |
| `triton_humanoid.xml`          | The robot: bodies, joints, inertials, meshes, collision geoms. Root `<freejoint/>`. No actuators. |
| `triton_humanoid_actuated.xml` | Includes the robot and adds 10 position actuators (kp 100/80/20, kv 1, ±120 Nm).                |
| `scene.xml`                    | Includes the actuated robot and adds a floor plane and a light.                                |
| `meshes/stl/`                  | All STL meshes, in millimetres (scaled by 0.001 in the MJCF). See inventory below.             |

Load `scene.xml` for a standalone MuJoCo test. Load `triton_humanoid.xml` when a framework
supplies its own ground and control, which is what ProtoMotions does.

View it (with `.venv-mujoco` active, from the repo root):

```sh
python -m mujoco.viewer --mjcf robots/triton_humanoid/scene.xml
```

Provenance: exported from CAD as `chrobot_16kg_candidate.xml`, `chrobot_16kg_actuated.xml`,
and `chrobot_16kg_standing_scene.xml`. Changes made here: files renamed, `model=` names
updated, `<include>` targets updated, and `meshdir` repointed from an mjlab checkout to
`meshes/stl`. Nothing else was touched.

## Kinematic structure

```
world
└─ floating_base            free joint; site `imu`
   └─ torso                 1 g placeholder inertial; site `top`
      └─ hip                3.46 kg; first real link
         ├─ left_leg1   hip1_joint    axis x   roll   [-1.57, 1.57]
         │  └─ left_leg2   hip2_joint    axis y   pitch  [-1.57, 0.44]
         │     └─ left_leg3   thigh_joint   axis -z  yaw    [-0.79, 0.79]
         │        └─ left_leg4   knee_joint    axis x          [-2.09, 0]
         │           └─ left_foot   ankle_joint   axis x          [-0.6, 0.6]   site `left_foot`
         └─ right_leg1  hip1_joint    axis x   roll   [-1.57, 1.57]
            └─ right_leg2  hip2_joint    axis y   pitch  [-0.44, 1.57]
               └─ right_leg3  thigh_joint   axis +z  yaw    [-0.79, 0.79]
                  └─ right_leg4  knee_joint    axis x          [-2.09, 0]
                     └─ right_foot  ankle_joint   axis x          [-0.6, 0.6]   site `right_foot`
```

Joint names are `{left,right}_{hip1,hip2,thigh,knee,ankle}_joint`. 10 actuated DOF, no arms,
no head. Feet collide with `box` geoms; every other link collides with a convex hull mesh.

Loading `scene.xml` gives 14 bodies, 11 joints (1 free + 10 hinge), 22 meshes, 10 actuators,
and a total mass of 16.00 kg.

Note on coordinates: every body has `pos="0 0 0"` and joints carry absolute positions, so
the model is written in the CAD assembly frame rather than in link-local frames. The export
header flags the rigid-link grouping and CAD frames as not yet validated.

## Mass and inertia state

From the export header, kept verbatim at the top of `triton_humanoid.xml`:

- CAD assembly masses were normalized to a 16 kg total. Hip 3.46 kg, each leg about 6.3 kg.
- Original centres of mass were retained; inertias were scaled per link.
- The torso is a 1 g placeholder with a 1e-6 diagonal inertia.
- Actuator gains and the 120 Nm force limits were copied from an older humanoid model and
  are not verified hardware specifications.

## Mesh inventory (`meshes/stl/`)

39 files, about 109 MB. Only 22 are referenced by the MJCF (5.6 MB).

| Group                   | Files                                                                                     | Used by MJCF | Size              |
| ----------------------- | ----------------------------------------------------------------------------------------- | ------------ | ----------------- |
| Collision hulls         | `hip_ch.stl`, `{left,right}_leg{1,2,3,4}_ch.stl`                                          | yes          | 20–75 KB each     |
| Decimated visuals       | `visual/hip_dumb2.stl`, `visual/{left,right}_leg{1,2,3,4}_dumb.stl`, `visual/{left,right}_foot.stl`, `visual/{left,right}_foot_sole.stl` | yes | ≤ 0.5 MB each |
| Foot collision hulls    | `{left,right}_foot_ch.stl`                                                                | no (feet use boxes) | 16 KB each  |
| High-resolution visuals | `visual/hip.stl`, `visual/{left,right}_leg{1,2,3,4}.stl`                                  | no (the `_dumb` versions are used) | 2.6–38.6 MB each, ~103 MB total |
| Extras                  | `battery.stl`, `visual/battery_box.stl`, `visual/imu_mount.stl`, `visual/torso_weight.stl`, `visual/{left,right}_foot_dumb.stl` | no | ≤ 0.3 MB each |

The high-resolution visuals are committed as plain git blobs (this repo does not use LFS).
They are kept for rendering or re-decimation; drop them from the MJCF's perspective at will.

## Known gaps

- Validate the rigid-link grouping and CAD frames; consider rewriting in link-local frames.
- Replace the placeholder torso inertial once the upper body is modelled.
- Verify actuator torque limits and gains against the hardware.
- Decide whether the ~103 MB of high-resolution meshes should stay in git.

## What ProtoMotions still needs

None of this is done yet. See `docs/protomotions.md` for where it goes.

- **Robot file:** `triton_humanoid.xml`. ProtoMotions adds its own ground and PD control;
  `triton_humanoid_actuated.xml` and `scene.xml` are for standalone MuJoCo only.
- **A `RobotConfig` subclass** with `semantic_forward_axis_xy` (find it with
  `third-party/protomotions/scripts/identify_robot_facing_axis.py`), PD gains per joint,
  trackable bodies, and default root height. The config asserts that
  `all_left_hand_bodies`, `all_right_hand_bodies`, and `head_body_name` exist even though
  this robot has none; map head to `torso` and hands to empty lists or `hip`.
- **One line** in `third-party/protomotions/protomotions/robot_configs/factory.py` to resolve
  `--robot-name triton_humanoid`.
- **A URDF** for the PyRoki retargeter (it does not read MJCF). Not present; generate it
  from `triton_humanoid.xml`, joint tree only, no visual meshes needed.
- **A legs-only retargeting script**, copied from
  `third-party/protomotions/pyroki/batch_retarget_to_g1_from_keypoints.py` with the
  shoulder, elbow, and wrist keypoints removed from the map.
- **No USD.** IsaacLab converts the MJCF at runtime and caches it.
