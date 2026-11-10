# 5. The field model

**Series:** [index](../README.md) · [1 Welcome](01_welcome.md) · [2 Setup](02_setup.md) · [3 Geometry](03_math_geometry.md) ·
[4 Uncertainty](04_math_probability.md) · **5 Field** · [6 Camera](06_camera.md) ·
[7 Particle filter](07_particle_filter.md) · [8 Which half](08_side_belief.md) · [9 Game and team](09_game_and_team.md) ·
[10 Testing](10_testing_and_evaluation.md) · [11 Progress](11_progress.md) · [12 Contributing](12_contributing.md)

To recognize where it is, the robot compares what it sees with a **map**. For a soccer field the map is simple: a
set of white lines, a few special points and two goals. This chapter covers the package that holds that map,
[`td_field_model`](../../../td_field_model/), specifically the file
[`field.py`](../../../td_field_model/td_field_model/field.py):

- where the field's numbers come from;
- how lines are stored as geometry;
- the special points ("landmarks");
- how to measure the distance from any point to the nearest line;
- the **likelihood field**, a precomputed grid that makes that fast;
- how the field's symmetry shows up in code.

---

## 5.1 The real field

The HSL rule book defines the field. The Large division plays on the M-Field (or the bigger L-Field). The numbers
for the M-Field, as the code stores them:

| Quantity | Value | Meaning |
|---|---|---|
| `length` | 13.95 m | Goal line to goal line, measured between line **centers** |
| `width` | 8.95 m | Touchline to touchline, between line centers |
| `line_width` | 0.05 m | Every line is 5 cm wide |
| `penalty_area_length` × `width` | 2.95 × 5.95 m | The big box in front of each goal |
| `goal_area_length` × `width` | 0.95 × 3.95 m | The small box |
| `penalty_mark_distance` | 1.975 m | From the goal line to the penalty spot |
| `center_circle_diameter` | 2.95 m | |
| `corner_arc_radius` | 0.475 m | The quarter circles in the corners |
| `border_strip_width` | 1.025 m | Carpet outside the lines where robots may stand |
| `goal_inner_width` | 2.4 m | Between the goalposts |

Why 13.95 and not 14? The rule book quotes **outside edge to outside edge** (14.00 m). The field files measure
**between line centers**, which is half a line width less on each end: 14.00 − 0.05 = 13.95. The code works with
centerlines throughout, so a detected line pixel lies within 2.5 cm (half a line width) of a centerline.
[`FieldDims`](../../../td_field_model/td_field_model/field.py)`.outer_length` gives the rule-book number back.

**Where the numbers come from.** The official rule book repository ships the field definitions as JSON files. Copies
live in [`td_field_model/td_field_model/fields/`](../../../td_field_model/td_field_model/fields/), with their source
and checksum in `SOURCE.md`. Three are built in: `hsl_s_2026`, `hsl_m_2026` and `hsl_l_2026`. A measured venue can
be loaded from its own JSON with [`FieldModel`](../../../td_field_model/td_field_model/field.py)`.from_json(path)`.

## 5.2 Lines as geometry

`FieldModel` turns those dimensions into shapes, in the field
frame from [chapter 3](03_math_geometry.md) (origin at the center mark, +x toward the opponent's goal). There are
two kinds of shape:

- a [`Segment`](../../../td_field_model/td_field_model/field.py) is a straight line from point `a` to point `b`;
- an [`Arc`](../../../td_field_model/td_field_model/field.py) is part of a circle: a `center`, a `radius`, a
  `start` angle and a `sweep` (how far around it goes; 2π is a full circle).

The M-Field has **17 segments and 5 arcs**:

| Segments (17) | Arcs (5) |
|---|---|
| 2 touchlines, 1 halfway line, 2 goal lines | center circle (full) |
| each penalty area: left, front, right side (×2 = 6) | 4 corner arcs (quarter circles) |
| each goal area: left, front, right side (×2 = 6) | |

Each shape has a `name`, like `own_penalty_area_front`, which helps when debugging ("which line did that point
match?"). The method `_build_lines` builds them; reading it is a good exercise in turning a drawing into numbers.
For example, the own penalty area's front line runs along x = −(6.975 − 2.95) = −4.025, from y = −2.975 to
y = +2.975.

## 5.3 Landmarks

Some points on the field are more informative than a stretch of line, because they're distinctive. The code calls
them [`Landmark`](../../../td_field_model/td_field_model/field.py)s, each with a `kind`, a position `xy` and a
`name`. The M-Field has 31:

| Kind | Count | What it is |
|---|---|---|
| `L` | 12 | Two lines meeting at a corner, like the letter L: field corners, box corners |
| `T` | 10 | One line ending on another, like a T: halfway line meets touchline, boxes meet the goal line |
| `X` | 2 | Two lines crossing: the halfway line crosses the center circle |
| `center_mark` | 1 | The kick-off spot |
| `penalty_mark` | 2 | The penalty spots |
| `goalpost` | 4 | The posts (approximated at the goal line's center) |

The particle filter can use landmarks if a detector reports them (the simulator does), but today's bench image
detector only finds line pixels ([chapter 6](06_camera.md)).

## 5.4 Distance from a point to the nearest line

The core question localization asks the map is: **"how far is this point from the nearest line?"** If the robot
guesses its pose correctly, every line point it sees should land on a line, at distance ≈ 0. A wrong guess puts
them off the lines.

### Distance to a segment

Take a segment from `a` to `b` and a point `p`. The closest point on the segment is found in three steps.

1. **Project** `p` onto the infinite line through `a` and `b`. The projection is `a + t (b − a)`, where

   ```math
   t = \frac{(p - a) \cdot (b - a)}{(b - a) \cdot (b - a)}
   ```

   The **dot product** `u · v = u_x v_x + u_y v_y` measures how much two arrows point the same way. Dividing by
   the segment's squared length makes `t` a fraction: `t = 0` is at `a`, `t = 1` is at `b`.
2. **Clamp** `t` to the range 0 to 1, so the closest point stays on the segment (not beyond its ends).
3. The distance is the length of `p` − closest point.

**Worked example.** The point `(−3, 1)` and the own penalty area's front line, from `a = (−4.025, −2.975)` to
`b = (−4.025, 2.975)`:

- `b − a = (0, 5.95)`, `p − a = (1.025, 3.975)`;
- `t = (1.025 × 0 + 3.975 × 5.95) / (5.95²) = 23.65 / 35.40 = 0.668`, already between 0 and 1;
- closest point = `(−4.025, −2.975 + 0.668 × 5.95) = (−4.025, 1.0)`;
- distance = `|(−3, 1) − (−4.025, 1)| = 1.025 m`.

This is `_segment_distance` in `field.py`, written for many points at once with numpy.

### Distance to an arc

For a circle with center `c` and radius `R`, the distance from `p` is `| |p − c| − R |`: how far `p` is from the
center, minus the radius. For example, `(−3, 1)` is √10 ≈ 3.162 m from the center mark, so it's
|3.162 − 1.475| = 1.687 m from the center circle.

For a partial arc (the corners), that only holds if `p` lies within the arc's angular range; otherwise the nearest
point is one of the arc's two ends. `_arc_distance` handles both cases.

### The nearest of all lines

`FieldModel.distance(points)` simply computes the distance to every segment and arc and keeps the minimum. For
`(−3, 1)` the winner is `own_penalty_area_front` at 1.025 m.

## 5.5 The likelihood field: a lookup table for distance

`distance()` checks 22 shapes per point. The particle filter needs about 60,000 distances per camera frame (500
guesses × 120 points), many times a second. Computing them from scratch is too slow.

So the code computes distance **once**, on a grid, and looks it up afterwards. That grid is the
[`LikelihoodField`](../../../td_field_model/td_field_model/field.py), made by `FieldModel.likelihood_field()`:

- it covers the field plus the border strip: x from −8.0 to +8.0 m, y from −5.5 to +5.5 m;
- one cell every 2 cm: **801 × 551 = 441,351 cells**;
- each cell stores the distance (in meters) from that spot to the nearest line centerline.

It's called a "likelihood field" because distance-to-line is what the likelihood is computed from
([chapter 7](07_particle_filter.md)). It's built once at startup (a second or two) and can be saved to a file.

### Bilinear interpolation

A point rarely falls exactly on a grid corner. `LikelihoodField.lookup` blends the four surrounding cells, weighted
by how close the point is to each:

```math
v = g_{00}(1 - t_x)(1 - t_y) + g_{10}\,t_x(1 - t_y) + g_{01}(1 - t_x)\,t_y + g_{11}\,t_x t_y
```

Here `g₀₀` … `g₁₁` are the four corner values, and `t_x`, `t_y` (between 0 and 1) are how far the point is across
the cell in x and y. If the point is exactly on a corner, that corner gets weight 1. In the middle of a cell, each
corner gets 1/4. This is **bilinear interpolation**: linear in x, then linear in y.

Points off the grid (far outside the field) get the grid's maximum distance, about 1.88 m. "Far from everything"
is the right answer for a point that can't be near any line.

## 5.6 Symmetry, in code

[Chapter 3](03_math_geometry.md) introduced the mirror pose `(−x, −y, θ + π)`. In terms of the map, mirroring maps
every line onto a line:

- the own penalty area onto the opponent's;
- the left touchline onto the right touchline;
- the center circle onto itself.

So distance is symmetric too: `distance((−3, 1))` and `distance((3, −1))` are both 1.025 m.

That's why the particle filter can't tell a pose from its mirror by matching lines: every point lands at exactly
the same distance from a line. The functions `mirror_pose` and `mirror_points` in `field.py` are the only place
this symmetry is written down; [chapter 8](08_side_belief.md) is about breaking it with other evidence.

## 5.7 Code tour

Open [`field.py`](../../../td_field_model/td_field_model/field.py) and find:

| Part | What to look at |
|---|---|
| Module docstring | The frame definition and the "centerlines" note. |
| `FieldDims.from_dict` | How JSON keys map to fields. |
| `FieldModel.builtin` | How the built-in fields are loaded. |
| `_build_lines`, `_build_landmarks` | The drawing-to-numbers step. |
| `distance`, `_segment_distance`, `_arc_distance` | Section 5.4. |
| `likelihood_field`, `LikelihoodField.lookup` | Section 5.5. |
| `mirror_pose` | Section 5.6. |

Then look at the tests in [`td_field_model/test/test_field.py`](../../../td_field_model/test/test_field.py). They
check the dimensions against the rule book, that distances are zero on lines, that the field is symmetric, and that
the grid lookup matches the exact distance. Tests are often the clearest documentation of what code promises.

## 5.8 Try it

```python
from td_field_model import FieldModel, mirror_pose

fm = FieldModel.builtin("hsl_m_2026")
print(fm.dims.length, fm.dims.width, fm.dims.outer_length)   # 13.95 8.95 14.0
print(len(fm.segments), len(fm.arcs), len(fm.landmarks))     # 17 5 31

print(fm.distance((-3.0, 1.0)))       # [1.025]  (own penalty area front)
print(fm.distance((3.0, -1.0)))       # [1.025]  (the mirror point: same distance)
print(fm.distance((0.0, 0.0)))        # [0.]     (the center mark is on the halfway line)
print(fm.distance((2.0, -4.0)))       # [0.475]  (0.475 m from the right touchline)

lf = fm.likelihood_field()
print(lf.grid.shape)                  # (801, 551)
print(lf.lookup([(-3.0, 1.0), (-3.01, 1.013)]))   # ≈ [1.025 1.015] (the grid is float32)

for lm in fm.landmarks_of("X"):
    print(lm.name, lm.xy)             # the two halfway-line / center-circle crossings
```

Questions:

1. Which line is nearest to `(−5, 3)`, and how far is it?
2. Why does `lookup((-3.01, 1.013))` give 1.015, not 1.025?

<details>
<summary>Answers</summary>

1. The own penalty area's left side, at y = 2.975: 0.025 m away (`fm.distance((-5, 3))` → 0.025).
2. Moving 1 cm toward our goal brings the point 1 cm closer to the penalty-area front line at x = −4.025. The y
   change doesn't matter, because that line runs along y.

</details>

## 5.9 Summary

- The map is built from the rule book's JSON, in **centerline** coordinates.
- Lines are 17 **segments** and 5 **arcs**; there are 31 typed **landmarks**.
- Distance to a segment: project, clamp, measure. To a circle: |distance to center − radius|.
- The **likelihood field** precomputes distance on a 2 cm grid, with **bilinear** lookup, so the filter is fast.
- Distance is the same at a point and its mirror: lines alone can't tell the halves apart.

---

**Next:** [6. Seeing the field: the camera](06_camera.md)
