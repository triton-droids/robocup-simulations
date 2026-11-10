# 3. Math toolkit, part 1: positions and angles

**Series:** [index](../README.md) · [1 Welcome](01_welcome.md) · [2 Setup](02_setup.md) · **3 Geometry** ·
[4 Uncertainty](04_math_probability.md) · [5 Field](05_field_model.md) · [6 Camera](06_camera.md) ·
[7 Particle filter](07_particle_filter.md) · [8 Which half](08_side_belief.md) · [9 Game and team](09_game_and_team.md) ·
[10 Testing](10_testing_and_evaluation.md) · [11 Progress](11_progress.md) · [12 Contributing](12_contributing.md)

Localization is mostly bookkeeping of positions and angles. This chapter builds every geometric tool the code uses,
from scratch:

- coordinate frames, and why the same point has different coordinates in different frames;
- angles in radians, and "wrapping" them;
- rotating a point;
- converting between the robot's view and the field map;
- adding up motions (odometry);
- the mirror pose that makes this field symmetric;
- measuring how far apart two poses are.

All you need is a little trigonometry: sine, cosine, and the idea of a right triangle. Everything else is built up
here.

---

## 3.1 Coordinate frames

A **coordinate frame** is a choice of three things: an origin (the zero point), an x-axis direction, and a y-axis
direction (90° counter-clockwise from x). Positions are measured as "how far along x, how far along y" from the
origin.

We use two frames all the time.

**The field frame** (also called `map`), fixed to the field:

- origin at the **center mark**;
- **+x points toward the opponent's goal**, so our own goal is at negative x;
- **+y points to the left** when you face the opponent's goal;
- angles are measured counter-clockwise from +x.

```text
                         +y (left)
                           ^
      our goal             |              their goal
       |-------------------+-------------------|
       |                   |                   |
  x = -6.975 ...... (0,0) center mark ...... x = +6.975  ---> +x
       |                   |                   |
       |-------------------+-------------------|
                           |
          our half: x < 0  |  their half: x > 0
```

On the M-Field, line centers run from x = −6.975 to +6.975 and from y = −4.475 to +4.475
([chapter 5](05_field_model.md)). **Our half is always x < 0**, in both halves of the game. This frame matches the
one the referee's GameController uses, so no conversion is ever needed when talking to it.

**The robot frame** (called `base_footprint` in ROS), fixed to the robot:

- origin on the floor, between the robot's feet;
- **+x points straight ahead** of the robot;
- **+y points to the robot's left**.

The camera reports what it sees in the robot frame: "a line point 2 m ahead and 0.5 m to the left" is `(2, 0.5)`
in the robot frame. To compare that with the map, we must convert it to the field frame. That conversion is most of
this chapter.

## 3.2 Angles and radians

Angles in the code are in **radians**, not degrees. A full turn is 2π radians (about 6.283) instead of 360°.

```math
\text{radians} = \text{degrees} \times \frac{\pi}{180}
\qquad
\text{degrees} = \text{radians} \times \frac{180}{\pi}
```

| Degrees | Radians |
|---|---|
| 0° | 0 |
| 30° | π/6 ≈ 0.524 |
| 90° | π/2 ≈ 1.571 |
| 180° | π ≈ 3.142 |
| 360° | 2π ≈ 6.283 |
| 1° | ≈ 0.01745 |

Why radians? Because math functions (`math.sin`, `np.cos`) expect them, and because a small angle in radians times
a distance gives the sideways offset directly: 0.01 rad at 6 m is about 0.06 m. That shortcut appears again in
[chapter 6](06_camera.md).

**Angle wrapping.** 350° and −10° are the same direction. So are 370° and 10°. To avoid confusion, the code keeps
every angle in the range −π to π (−180° to 180°). The trick it uses:

```python
wrapped = math.atan2(math.sin(a), math.cos(a))
```

Why it works: `sin` and `cos` don't care about extra full turns, and `atan2(y, x)` returns the angle of the point
`(x, y)` in the range −π to π. So 350° goes in and −10° comes out. You'll see this as `wrap()` in
[`scenarios.py`](../../../td_field_model/td_field_model/scenarios.py) and
[`pf.py`](../../../td_localization/td_localization/pf.py).

**Never subtract angles without wrapping.** The difference between 179° and −179° is 2°, not 358°. Every heading
error in the code is computed as `wrap(a - b)`.

## 3.3 A pose

A **pose** is a position plus a heading: `(x, y, θ)`. In code it's a tuple `(x, y, theta)` or a row of a numpy
array.

Example: the robot at `(−3, 1, 0.524)` stands 3 m toward our goal from the center, 1 m to the left, facing 30°
counter-clockwise from the +x direction (toward the opponent's goal, turned a little to the left).

## 3.4 Rotating a point

Suppose a point is at `(px, py)` and we rotate it counter-clockwise about the origin by an angle θ. Where does it
end up?

Start with the simplest cases:

- The point `(1, 0)` (one step along x) rotated by θ lands on the unit circle at `(cos θ, sin θ)`. That's the
  definition of sine and cosine.
- The point `(0, 1)` (one step along y) is 90° further around, at `(−sin θ, cos θ)`.

Any point is "px steps along x plus py steps along y", and rotating each part separately and adding them gives:

```math
\begin{aligned}
x' &= \cos\theta \cdot p_x - \sin\theta \cdot p_y \\
y' &= \sin\theta \cdot p_x + \cos\theta \cdot p_y
\end{aligned}
```

Written as a matrix (a grid of numbers that does this in one step), this is the **rotation matrix**:

```math
R(\theta) = \begin{pmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{pmatrix},
\qquad
\begin{pmatrix} x' \\ y' \end{pmatrix} = R(\theta) \begin{pmatrix} p_x \\ p_y \end{pmatrix}
```

Two facts you'll rely on:

- Rotating by θ and then by −θ gets you back where you started. `R(−θ)` is the **inverse** of `R(θ)`, and for
  rotation matrices that's also the **transpose** (swap rows and columns).
- Rotation never changes a point's distance from the origin.

## 3.5 From the robot's view to the field: `robot_to_world`

The robot is at pose `(x, y, θ)` and sees a point at `(px, py)` in its own frame. Where is that point on the field?

Two steps:

1. **Rotate** the point by the robot's heading θ. This turns "ahead and left of the robot" into "along the field's
   axes".
2. **Shift** it by the robot's position `(x, y)`.

```math
\begin{aligned}
x_{\text{field}} &= x + \cos\theta \cdot p_x - \sin\theta \cdot p_y \\
y_{\text{field}} &= y + \sin\theta \cdot p_x + \cos\theta \cdot p_y
\end{aligned}
```

**Worked example.** The robot is at `(−3, 1)` facing 30°, and sees a line point at `(2, 0.5)`: 2 m ahead, 0.5 m to
the left. With cos 30° ≈ 0.866 and sin 30° = 0.5:

```math
\begin{aligned}
x_{\text{field}} &= -3 + 0.866 \times 2 - 0.5 \times 0.5 = -3 + 1.732 - 0.25 = -1.518 \\
y_{\text{field}} &= 1 + 0.5 \times 2 + 0.866 \times 0.5 = 1 + 1 + 0.433 = 2.433
\end{aligned}
```

The point is at `(−1.518, 2.433)` on the field. This exact formula is `robot_to_world` in
[`sim.py`](../../../td_field_model/td_field_model/sim.py), and the particle filter does the same computation
inline for every particle and every point.

## 3.6 From the field to the robot's view: `world_to_robot`

Going backwards undoes both steps in reverse order: subtract the robot's position, then rotate by −θ.

```math
\begin{aligned}
d_x &= x_{\text{field}} - x, \qquad d_y = y_{\text{field}} - y \\
p_x &= \cos\theta \cdot d_x + \sin\theta \cdot d_y \\
p_y &= -\sin\theta \cdot d_x + \cos\theta \cdot d_y
\end{aligned}
```

Rotating by −θ just flips the sign of every `sin θ` (because sin(−θ) = −sin θ and cos(−θ) = cos θ). Feeding
`(−1.518, 2.433)` back in gives `(2, 0.5)` again. The simulator uses this to work out where field lines appear
relative to a robot.

## 3.7 Adding up motion: `compose` and `relative_motion`

Odometry reports motion **in the robot frame**: "since last time I moved `dx` forward, `dy` to the left, and turned
`dθ`". To update a pose, rotate that motion into the field frame and add it, just like a point, then add the turn:

```math
\text{compose}\big((x, y, \theta), (d_x, d_y, d_\theta)\big) =
\big(x + \cos\theta\, d_x - \sin\theta\, d_y,\;\; y + \sin\theta\, d_x + \cos\theta\, d_y,\;\; \text{wrap}(\theta + d_\theta)\big)
```

**Worked example.** The robot is at `(−3, 1)` facing 90° (straight toward the left touchline). It walks 0.3 m
forward and turns 0.1 rad (5.7°) left:

- forward is +y on the field when facing 90°, so x stays −3 and y becomes 1.3;
- the heading becomes 90° + 5.7° = 95.7°.

`compose((-3, 1, π/2), (0.3, 0, 0.1))` returns `(−3.0, 1.3, 1.671)` (1.671 rad = 95.7°).

`relative_motion(p0, p1)` is the reverse question: "what robot-frame motion takes me from `p0` to `p1`?" It's
`world_to_robot` applied to the position change, plus the wrapped heading change. The simulator uses it to produce
the odometry a robot *would* report between two true poses.

Both live in [`scenarios.py`](../../../td_field_model/td_field_model/scenarios.py). Note the **order matters**:
walking 1 m then turning 90° ends somewhere different from turning 90° then walking 1 m.

## 3.8 The mirror pose

Rotate the whole field 180° about the center mark. Every point `(x, y)` goes to `(−x, −y)`, and every direction
turns around by 180°. Because the field's lines are point-symmetric, the rotated field looks exactly like the
original. So a robot at `(x, y, θ)` and one at

```math
\text{mirror}(x, y, \theta) = (-x,\; -y,\; \text{wrap}(\theta + \pi))
```

see **identical** images. Example: `(−3, 1, 30°)` mirrors to `(3, −1, −150°)`.

This one formula is the root of the hardest problem in the project ([chapter 8](08_side_belief.md)). It's
`mirror_pose` in [`field.py`](../../../td_field_model/td_field_model/field.py).

A useful observation: near the center mark, a pose and its mirror are close in *position* (both near `(0, 0)`) but
face *opposite* ways. Far from the center, they're far apart in position. So to tell "which of the two is closer"
reliably, we need a distance that includes heading.

## 3.9 How far apart are two poses?

Position distance is the usual straight-line distance:

```math
d_{\text{pos}} = \sqrt{(x_1 - x_2)^2 + (y_1 - y_2)^2}
```

`math.hypot(a, b)` computes √(a² + b²) and is used everywhere for this.

But meters and radians can't be added directly. The code converts heading into meters with a scale factor,
2 m per radian, so a 1 rad heading difference "costs" as much as 2 m of position difference:

```math
d_{\text{pose}} = \sqrt{d_{\text{pos}}^2 + (2 \cdot |\text{wrap}(\theta_1 - \theta_2)|)^2}
```

This is `pose_distance` of [`ParticleFilter`](../../../td_localization/td_localization/pf.py) in [`pf.py`](../../../td_localization/td_localization/pf.py)
(`fold_heading_scale = 2.0`), and the same idea defines "wrong half" in the evaluation tool
([chapter 10](10_testing_and_evaluation.md)): an estimate is on the wrong half when it's closer to the mirror of the
truth than to the truth.

Example: the truth is `(−0.2, 0, 0°)`, just behind the center mark facing the opponent. An estimate of
`(0.2, 0, 180°)` is 0.4 m away in position, but its distance to the truth is 6.3 (mostly the 180° heading error),
while its distance to the mirror of the truth, `(0.2, 0, 180°)`, is exactly 0. It's on the wrong half, and only
the heading term reveals that.

## 3.10 Try it

With the venv active and `PYTHONPATH` set as in [chapter 2](02_setup.md), run:

```python
import math
from td_field_model import mirror_pose
from td_field_model.scenarios import compose, relative_motion
from td_field_model.sim import robot_to_world, world_to_robot

pose = (-3.0, 1.0, math.radians(30))
world = robot_to_world([[2.0, 0.5]], pose)
print(world)                                  # [[-1.51794919  2.4330127 ]]
print(world_to_robot(world, pose))            # [[2.  0.5]]

p = compose((-3.0, 1.0, math.pi / 2), (0.3, 0.0, 0.1))
print(p, math.degrees(p[2]))                  # (-3.0, 1.3, 1.67...) 95.7...
print(relative_motion((-3.0, 1.0, math.pi / 2), p))   # [0.3 ~0 0.1] (the 1.8e-17 is rounding noise)

m = mirror_pose(*pose)
print(m, math.degrees(m[2]))                  # (3.0, -1.0, -2.617...) -150.0...
```

Things to try:

1. Change the heading to 0° and predict `robot_to_world([[2, 0.5]], (-3, 1, 0))` before running it.
2. Compose 4 motions of `(1, 0, π/2)` starting at `(0, 0, 0)`. Where do you end up? (You walk a square.)
3. Mirror a pose twice. What do you get?

<details>
<summary>Answers</summary>

1. `(−1, 1.5)`: with θ = 0 there's no rotation, so just add.
2. Back at `(0, 0)`, facing 0 (up to tiny rounding, about 1e-16).
3. The original pose: mirroring is its own inverse.

</details>

## 3.11 Summary

- Two frames: **field** (center origin, +x toward the opponent goal) and **robot** (+x ahead, +y left).
- Angles are radians, always **wrapped** to (−π, π], and subtracted with wrapping.
- Robot → field: rotate by θ, then shift by `(x, y)`. Field → robot: shift back, then rotate by −θ.
- Odometry is a robot-frame motion; `compose` applies it to a pose.
- The **mirror pose** `(−x, −y, θ + π)` sees exactly the same field.
- Pose distance mixes position and heading at 2 m per radian.

---

**Next:** [4. Math toolkit, part 2: uncertainty](04_math_probability.md)
