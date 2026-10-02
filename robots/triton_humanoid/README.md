# Triton humanoid

MuJoCo description of the Triton Droids RoboCup humanoid: a 10-DOF, legs-only biped
with a floating base. This directory is the single source of truth for the robot model.
ProtoMotions loads it through `robocup_rl/robots/triton_humanoid.py` (see "ProtoMotions
integration").

## Files

| File                           | What it is                                                                                     |
| ------------------------------ | ---------------------------------------------------------------------------------------------- |
| `triton_humanoid.xml`          | The robot: bodies, joints, inertials, meshes, collision geoms. Root `<freejoint/>`. No actuators. |
| `triton_humanoid_actuated.xml` | Includes the robot and adds 10 position actuators. **This is the file ProtoMotions loads.** ProtoMotions ignores the kp/kv/forcerange here and uses the gains declared in `robocup_rl/robots/triton_humanoid.py`; a test keeps the two equal. Its empty `<worldbody/>` is where ProtoMotions' MuJoCo loader injects its own floor. |
| `scene.xml`                    | Includes the actuated robot and adds a floor plane and a light, for standalone MuJoCo only. ProtoMotions never loads it and supplies its own floor at z=0. |
| `meshes/stl/`                  | All STL meshes, in millimetres (scaled by 0.001 in the MJCF). See inventory below.             |

Load `scene.xml` for a standalone MuJoCo test. `triton_humanoid.xml` on its own has no
actuators; ProtoMotions' MuJoCo backend needs one per joint, so it loads the actuated file.

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
and a total mass of 16.50 kg (16.00 kg from CAD plus the 0.5 kg base placeholder below).

Note on coordinates: every body has `pos="0 0 0"` and joints carry absolute positions, so
the model is written in the CAD assembly frame rather than in link-local frames. The export
header flags the rigid-link grouping and CAD frames as not yet validated.

## Mass and inertia state

From the export header, kept verbatim at the top of `triton_humanoid.xml`:

- CAD assembly masses were normalized to a 16 kg total. Hip 3.46 kg, each leg about 6.3 kg.
- Original centres of mass were retained; inertias were scaled per link.
- The torso is a 1 g placeholder with a 1e-6 diagonal inertia.
- `floating_base` carries a 0.5 kg placeholder inertial (added 2026-10-02, total now 16.5 kg):
  with the free-joint body massless, Newton's MuJoCo Warp solver diverged to NaN under
  saturated PD commands. Fold it into the real upper-body mass when that is modelled.
- Actuator gains were copied from an older humanoid model and are not verified hardware
  specifications. Its 120 Nm force limit was lowered to 40 Nm for the same stability reason
  (see `robocup_rl/robots/triton_humanoid.py`); 40 Nm is a guess too.

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

- **Every body frame sits at the same point.** All bodies have `pos="0 0 0"` and joints carry
  absolute positions, so at the zero pose every body's frame origin coincides with the base
  (`mj_forward` gives identical `xpos` for all 13 bodies; only the inertial COMs differ).
  ProtoMotions' observations, trackable bodies, contact heights and fall termination all use
  body frame positions, so until the model is rewritten in link-local frames those carry no
  per-limb position information, and upstream's `identify_robot_facing_axis.py` fails on it.
- **Left/right naming looks mirrored.** With the toes pointing to +y, the leg named `left_*`
  sits on the robot's right (+x). Harmless for symmetric locomotion rewards; matters the
  moment anything uses left/right semantics (retargeting, asymmetric gaits). Decide whether
  to rename in the xml or in the config's body mapping.
- Validate the rigid-link grouping and CAD frames; consider rewriting in link-local frames.
- Replace the placeholder torso inertial once the upper body is modelled.
- Verify actuator torque limits and gains against the hardware.
- Decide whether the ~103 MB of high-resolution meshes should stay in git.

## ProtoMotions integration

- **Config:** `robocup_rl/robots/triton_humanoid.py` (`TritonHumanoidConfig`). It points at
  `triton_humanoid_actuated.xml` with an absolute `asset_root`, maps feet to `left_foot` /
  `right_foot`, torso and head to `torso`, and the mandatory hand keys to the hip-roll links
  (the robot has none; training never reads them), and declares the PD gains, ±40 Nm
  limits, `default_root_height` 0.71 m and the zero (CAD) default pose. All of these are
  placeholders until verified against hardware. Contact sensors are on every body
  (`contact_bodies="all"`): Newton and IsaacLab only sense listed bodies, and fall
  termination needs a non-foot contact, so a feet-only list silently disables resets there.
- **Registration:** the `triton_humanoid` entry point in the root `pyproject.toml`, resolved
  by the hook in ProtoMotions' robot factory (see `docs/protomotions.md`). After `uv sync`,
  `--robot-name triton_humanoid` works everywhere.
- **Forward axis:** `(0, 1)`: the toes point to +y and knee flexion swings the foot to -y.
  Upstream's `identify_robot_facing_axis.py` cannot verify it on this model (see the first
  known gap below).
- **Tests:** `pytest` runs `tests/test_triton_humanoid.py` (entry point, kinematics, gains
  equal to the xml, a headless MuJoCo step).
- **First experiment:** `robocup_rl/experiments/steering/config.py`, plain PPO with no motion data.
- **No USD needed.** IsaacLab converts the MJCF at runtime and caches it.

Still missing: a URDF and a legs-only retargeting script for the PyRoki retargeter
(`third-party/protomotions/pyroki/batch_retarget_to_g1_from_keypoints.py` is G1-specific).
