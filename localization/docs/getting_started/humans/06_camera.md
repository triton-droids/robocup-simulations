# 6. Seeing the field: the camera

**Series:** [index](../README.md) · [1 Welcome](01_welcome.md) · [2 Setup](02_setup.md) · [3 Geometry](03_math_geometry.md) ·
[4 Uncertainty](04_math_probability.md) · [5 Field](05_field_model.md) · **6 Camera** ·
[7 Particle filter](07_particle_filter.md) · [8 Which half](08_side_belief.md) · [9 Game and team](09_game_and_team.md) ·
[10 Testing](10_testing_and_evaluation.md) · [11 Progress](11_progress.md) · [12 Contributing](12_contributing.md)

The particle filter never sees an image. It receives **points on the floor**, in the robot frame: "there's a line
point 3.2 m ahead and 0.4 m to the left". This chapter explains how a camera image becomes those points, and why
the hardest part isn't the image at all, but knowing **exactly where the camera is pointing**.

You'll learn:

- how a camera turns the 3-D world into pixels (the pinhole model);
- how the camera's position on the robot is described;
- how to turn a pixel back into a point on the floor;
- why a 1° error in head tilt moves a far point by half a meter;
- how each point's uncertainty is computed;
- how the simulator fakes all of this, and how the bench's simple line detector works.

The code is in [`td_field_model/td_field_model/sim.py`](../../../td_field_model/td_field_model/sim.py) and
[`td_bench/td_bench/vision.py`](../../../td_bench/td_bench/vision.py).

---

## 6.1 Pixels

An image is a grid of pixels. Pixel positions are written `(u, v)`:

- `u` counts columns from the **left** edge;
- `v` counts rows from the **top** edge (down is positive).

The robot's D435 color camera produces 848 × 480 images at the planned settings (u from 0 to 847, v from 0 to 479).
The bench uses half that resolution, 424 × 240, to save time.

## 6.2 The pinhole camera model

A camera is, to a good approximation, a box with a tiny hole: light from each point in the world travels in a
straight line through the hole and lands on the sensor. That's the **pinhole model**.

The camera has its own frame, the **optical frame**:

- z points straight out of the lens (forward);
- x points right in the image;
- y points down in the image.

A point at `(X, Y, Z)` in that frame (Z = how far in front) lands on pixel

```math
u = f_x \frac{X}{Z} + c_x, \qquad v = f_y \frac{Y}{Z} + c_y
```

- **Dividing by Z** is why far things look small: the same sideways offset X makes less of a difference when Z is
  large.
- **f_x, f_y** (the **focal length**, in pixels) scale the result. For the D435 at 848 × 480, f ≈ 615 pixels.
- **c_x, c_y** (the **principal point**) is the pixel straight down the lens's axis, about the image center
  (424, 240).

These four numbers are the camera's **intrinsics**.
[`CameraModel`](../../../td_field_model/td_field_model/sim.py) stores them, plus the camera's height above the floor
(`mount_height`, 1.2 m by default) and a maximum range (9 m).

## 6.3 Where the camera is: head angles

The camera sits in the head, 1.2 m above the floor, and the head can **yaw** (pan left/right) and **pitch** (tilt
down). Positive pitch means looking down. To use the pinhole model we need the camera's position and orientation in
the robot frame, which `CameraModel.pose_in_base(head_yaw, head_pitch)` computes as a rotation `R` and a
translation `t`:

- `t = (mount_forward, 0, h)`: the camera is h = 1.2 m above the robot's feet.
- `R = Rz(yaw) · Ry(pitch) · R_link_opt`, three rotations applied right to left:
  1. `R_link_opt` converts the optical frame's axes (z forward, x right, y down) into the robot's axes (x forward,
     y left, z up). It doesn't move anything; it just renames axes.
  2. `Ry(pitch)` tilts the camera down about the robot's left-pointing y axis.
  3. `Rz(yaw)` turns it left or right about the vertical z axis.

`Rz` is the 2-D rotation from [chapter 3](03_math_geometry.md) with an extra row and column for z, which it leaves
alone. `Ry` is the same idea about the y axis:

```math
R_z(\psi) = \begin{pmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{pmatrix},
\qquad
R_y(\varphi) = \begin{pmatrix} \cos\varphi & 0 & \sin\varphi \\ 0 & 1 & 0 \\ -\sin\varphi & 0 & \cos\varphi \end{pmatrix}
```

The robot doesn't measure head angles yet: the head servos have no position sensors. Adding encoders, fused with
the head IMU, is ticket LOC-10/LOC-09. Section 6.6 shows why that's on the critical path.

## 6.4 From a pixel to a point on the floor

This is **back-projection**, `backproject_pixels` in `sim.py`. It answers: "this pixel shows a line; where on the
floor is it?"

1. **Pixel → ray in the camera.** Undo the pinhole formula with Z = 1: the ray through pixel `(u, v)` points along

   ```math
   \mathbf{r}_{\text{cam}} = \left(\frac{u - c_x}{f_x},\; \frac{v - c_y}{f_y},\; 1\right)
   ```

2. **Rotate the ray into the robot frame:** `r_robot = R · r_cam`.
3. **Intersect with the floor.** The ray starts at the camera, `t = (0, 0, h)`, and points along `r_robot`. Points
   on it are `t + s · r_robot` for s ≥ 0. The floor is z = 0, so solve `h + s · r_z = 0`:

   ```math
   s = \frac{-h}{r_z}, \qquad \text{floor point} = (t_x + s\, r_x,\; t_y + s\, r_y)
   ```

   This only works if the ray points **down** (r_z < 0). A ray pointing up or level never reaches the floor: that
   pixel shows the sky, a wall or a spectator.

**Worked numbers** (camera 1.2 m up, pitch 0.35 rad ≈ 20° down, no yaw, 848 × 480 image):

| Pixel (u, v) | Where it is | Floor point (x ahead, y left) |
|---|---|---|
| (424, 240) | image center | (3.29, 0) m |
| (424, 479.5) | bottom center | (1.36, 0) m |
| (0, 479.5) | bottom-left corner | (1.36, 1.17) m |
| (424, 100) | upper middle | (9.46, 0) m |
| (424, 10) | near the top | doesn't hit the floor |

The center pixel looks along the camera's axis, 20° below horizontal, so it hits the floor at
h / tan(20°) = 1.2 / 0.364 = 3.29 m.

The **horizon** is the image row where rays become level. Above it, nothing is floor. For pitch φ it's at
`v = c_y − f_y · tan(φ)` = 240 − 615 × 0.364 ≈ 15.5. That's `horizon_row` in `vision.py`; the bench detector ignores
everything above it.

**The reverse** (floor point → pixel) is `project_floor`: convert the point into the camera frame with `R` and `t`,
then apply the pinhole formula. The simulator uses it to work out which field lines appear in the image. Projecting
`(3, 0.5)` gives pixel `(328.8, 258.8)`, and back-projecting that pixel gives `(3, 0.5)` again.

## 6.5 Why head angles matter so much

Back-projection uses the **believed** camera pose. If the believed pitch is off by a little, every floor point
moves, and far points move a lot.

Picture a point on the floor at distance `d` ahead. From the camera at height `h`, you look down at it at an angle
φ (the **depression angle**):

```math
\tan\varphi = \frac{h}{d} \quad\Longrightarrow\quad d = \frac{h}{\tan\varphi}
```

If the believed angle is off by a small δ, the believed distance changes by about δ times the slope of `d(φ)`.
Taking the derivative (or trusting the result):

```math
\left|\frac{\mathrm{d}d}{\mathrm{d}\varphi}\right| = \frac{h}{\sin^2\varphi} = \frac{h^2 + d^2}{h}
\qquad\Longrightarrow\qquad
\sigma_{\text{range}} \approx \frac{h^2 + d^2}{h}\,\sigma_{\text{pitch}}
```

(The second step uses sin²φ = h² / (h² + d²), from the right triangle with sides h and d.)

With h = 1.2 m and a 1° (0.01745 rad) pitch error:

| Distance | Formula (h² + d²)/h · δ | Exact, from the code |
|---|---|---|
| 2 m | 0.079 m | 0.077 m |
| 4 m | 0.254 m | 0.240 m |
| 6 m | 0.545 m | 0.501 m |

**One degree of tilt error moves a point 6 m away by half a meter.** The error grows with distance *squared*. And
it doesn't depend on where the camera is aimed, only on the angle down to the point. Two consequences shape the
whole design:

- **Head angles must be measured, not assumed.** With the servos' commanded angle alone (no feedback), a degree or
  two of error is normal. Hence the head-encoder tickets.
- **Far points must be trusted less.** The filter gives each point its own uncertainty, so a point 6 m away counts
  much less than one 2 m away (section 6.6 and [chapter 7](07_particle_filter.md)).

A yaw error δ does something gentler: it swings a point sideways by about √(h² + d²) · δ, which grows only linearly
with distance.

## 6.6 Each point's uncertainty

`floor_covariances` in `sim.py` gives every floor point a 2×2 covariance ([chapter 4](04_math_probability.md)),
built from the errors above.

1. **Pixel noise becomes an angle error:** σ_pixel / f. One pixel at f = 615 is 0.0016 rad (0.09°).
2. **Combine it in quadrature with the head-angle noise:** σ_pitch,total = hypot(σ_pixel / f, σ_pitch), and the
   same for yaw.
3. **Range and lateral sigmas:**

   ```math
   \sigma_r = \frac{h^2 + d^2}{h}\,\sigma_{\text{pitch,total}}, \qquad
   \sigma_t = \sqrt{h^2 + d^2}\;\sigma_{\text{yaw,total}}
   ```

4. **Rotate** that ellipse to point along the line of sight (bearing β), as in chapter 4, section 4.4.

With 1 px of pixel noise and 0.3° of head-angle noise, a point 3 m ahead gets σ_range ≈ 4.9 cm and σ_lateral ≈
1.9 cm. A point 6 m ahead gets σ_range ≈ 17 cm.

Points farther than 4 m are also flagged `range_ok = False` ("bearing only"). The current filter doesn't read that
flag: it handles far points through their large σ, which makes them count mostly for their direction, almost like
a pure bearing.

## 6.7 The simulator

Without the robot, we need fake observations that are realistic, including realistic **mistakes**.
[`Simulator`](../../../td_field_model/td_field_model/sim.py) produces them in
`render(pose, head_yaw, head_pitch)`:

```mermaid
flowchart LR
    A["Field lines<br/>(points every 3 cm)"] --> B["Project with the TRUE<br/>camera pose → pixels"]
    B --> C["Add pixel noise"]
    C --> D["Back-project with the BELIEVED<br/>pose (true + bias + jitter)"]
    D --> E["Drop some points, cap at 200,<br/>add false positives"]
    E --> F["Observations<br/>(robot-frame points + covariances)"]
```

The key trick is in the middle: project with the **true** pose and back-project with the **believed** pose. That's
exactly what happens on the robot, so the simulator doesn't have to invent range errors. They come out of the
geometry by themselves.

[`NoiseModel`](../../../td_field_model/td_field_model/sim.py) controls the mistakes:

| Field | Default | Meaning |
|---|---|---|
| `pixel_sigma` | 1 px | Detector position noise |
| `pitch_bias` | 0 | A constant calibration error in the believed pitch |
| `pitch_jitter`, `yaw_jitter` | 0.3° | Per-frame head-angle error |
| `point_dropout` | 0 | Chance each true line point is missed |
| `frame_dropout` | 0 | Chance a whole frame is lost |
| `false_positives` | 0 | Average number of fake "line" points per frame |
| `crossing_confusion` | 0 | Chance an L/T/X crossing is reported as the wrong type |
| `max_line_points` | 200 | Detector output cap per frame |

The test suite uses two presets: `default` (2 false positives per frame) and `harsh` (1° pitch bias, 5 false
positives, 20% dropout, 10% crossing confusion). The simulator can also hide floor behind **obstacles** (other
robots as cylinders).

The output is an [`Observations`](../../../td_field_model/td_field_model/sim.py) object, which is exactly what the
particle filter consumes:

| Field | Shape | Meaning |
|---|---|---|
| `line_points` | (N, 2) | Floor points in the robot frame, meters |
| `line_cov` | (N, 2, 2) | One covariance per point |
| `line_range_ok` | (N,) | False beyond 4 m |
| `landmarks` | list | Detected L/T/X crossings, marks and posts, if any |

## 6.8 The bench's line detector

To test with real images, the bench needs something that finds line pixels.
[`vision.py`](../../../td_bench/td_bench/vision.py) has a deliberately simple one, `detect_line_pixels`:

1. **White:** all three color channels bright (≥ 175) and similar (spread ≤ 45), i.e. white or light gray.
2. **Green nearby:** a field-green pixel within 8 pixels above or below. Lines lie on grass, so this rejects white
   walls, shirts against the crowd, and the sky.
3. **Below the horizon** of the believed camera pose (section 6.4).
4. Every 3rd pixel in each direction, at most 300 points.

Then `image_observations` back-projects those pixels with the believed head angles and attaches covariances, giving
the same `Observations` as the simulator.

For testing without a camera, `render_field_image` draws a synthetic image: every pixel is back-projected; if it
lands within 2.5 cm of a line it's painted white, otherwise green carpet, gray floor or sky, plus noise. Run
through the detector, the detected points land within a few centimeters of the true lines.

This detector is a **stand-in**. The real field-feature detector (ticket LOC-17) will need to handle real lighting,
shadows, robots standing on lines, and to classify crossings. Because it will output the same `Observations`, it
will plug into the same place.

## 6.9 Try it

```python
import math
import numpy as np
from td_field_model.sim import CameraModel, backproject_pixels, project_floor

cam = CameraModel()                          # D435 at 848×480, 1.2 m up
R, t = cam.pose_in_base(0.0, 0.35)           # no yaw, pitched 0.35 rad (20°) down

pts, hit = backproject_pixels(cam, np.array([[424, 240], [424, 479.5], [424, 10]]), R, t)
print(pts, hit)          # ≈ [[3.287 0] [1.364 0] [0 0]]  [ True  True False]: row 10 misses the floor

# A 1° pitch error, at 2, 4 and 6 m
R_bad, t_bad = cam.pose_in_base(0.0, 0.35 + math.radians(1))
for d in (2.0, 4.0, 6.0):
    uv, ok = project_floor(cam, np.array([[d, 0.0]]), R, t)       # where it really appears
    believed, _ = backproject_pixels(cam, uv, R_bad, t_bad)       # where we'd think it is
    print(d, round(d - believed[0, 0], 3))                        # 0.077, 0.24, 0.501
```

And the simulator:

```python
from td_field_model import FieldModel
from td_field_model.sim import NoiseModel, Simulator

fm = FieldModel.builtin("hsl_m_2026")
sim = Simulator(fm, noise=NoiseModel(false_positives=2.0), seed=0)
obs = sim.render((-3.0, 1.0, 0.3), head_yaw=0.0, head_pitch=0.35)
print(len(obs.line_points), "points,", len(obs.landmarks), "landmarks")
print(obs.line_points[:3])        # robot-frame floor points (x ahead, y left)
print(obs.line_cov[0])            # that point's 2×2 covariance
```

Try changing the pose to face the corner, or set `head_pitch=0.6` (looking more steeply down), and watch the number
of points change.

## 6.10 Summary

- The **pinhole model**: u = f·X/Z + c. Intrinsics are f_x, f_y, c_x, c_y.
- The camera pose comes from the **head angles**: R = Rz(yaw)·Ry(pitch)·R_link_opt, 1.2 m up.
- **Back-projection:** pixel → ray → rotate into the robot frame → intersect with the floor.
- Range error ≈ (h² + d²)/h × pitch error: **1° ≈ 0.5 m at 6 m**. Head angles must be measured.
- Each point gets a covariance, so far points count less.
- The simulator projects with the true pose and back-projects with the believed one; `NoiseModel` adds realistic
  mistakes. The bench's detector is a simple white-next-to-green stand-in.

---

**Next:** [7. The particle filter](07_particle_filter.md)
