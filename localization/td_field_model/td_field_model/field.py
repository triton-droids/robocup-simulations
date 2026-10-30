"""HSL soccer field model: line geometry, typed landmarks and a line likelihood field (LOC-03).

Pure numpy, no ROS. Built from the rule book's field JSON (fields/SOURCE.md), or from a measured venue JSON.

Frame (docs/roadmap.md §3.2): origin at the center mark, +x toward the opponent goal, +y left,
z up. Our own goal is at x = -length/2, so our half is always x < 0. The field is point-symmetric:
mirror_pose() maps a pose to the one that sees exactly the same lines.

All lengths come straight from the JSON, which measures between line centers (see SOURCE.md), so every
segment and arc here is a line centerline. Distances are to the centerline. A detected line pixel lies
within line_width/2 of it.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FIELDS_DIR = Path(__file__).resolve().parent / "fields"
BUILTIN_FIELDS = {
    "hsl_s_2026": "field_hsl_s_2026.json",
    "hsl_m_2026": "field_hsl_m_2026.json",
    "hsl_l_2026": "field_hsl_l_2026.json",
}

# Landmark kinds. L/T/X are line crossings, named after the shape the lines make.
L_CROSSING = "L"
T_CROSSING = "T"
X_CROSSING = "X"
CENTER_MARK = "center_mark"
PENALTY_MARK = "penalty_mark"
GOALPOST = "goalpost"


@dataclass(frozen=True)
class FieldDims:
    length: float
    width: float
    line_width: float
    penalty_area_length: float
    penalty_area_width: float
    penalty_mark_distance: float
    center_circle_diameter: float
    border_strip_width: float
    goal_inner_width: float
    goal_post_diameter: float
    goal_height: float
    goal_depth: float
    goal_area_length: float | None = None
    goal_area_width: float | None = None
    corner_arc_radius: float | None = None
    penalty_mark_diameter: float | None = None

    @classmethod
    def from_dict(cls, d: dict) -> FieldDims:
        f, g = d["field"], d["goal"]
        return cls(
            length=f["length"],
            width=f["width"],
            line_width=f["lineWidth"],
            penalty_area_length=f["penaltyAreaLength"],
            penalty_area_width=f["penaltyAreaWidth"],
            penalty_mark_distance=f["penaltyMarkDistance"],
            center_circle_diameter=f["centerCircleDiameter"],
            border_strip_width=f["borderStripWidth"],
            goal_inner_width=g["innerWidth"],
            goal_post_diameter=g["postDiameter"],
            goal_height=g["height"],
            goal_depth=g["depth"],
            goal_area_length=f.get("goalAreaLength"),
            goal_area_width=f.get("goalAreaWidth"),
            corner_arc_radius=f.get("cornerArcRadius"),
            penalty_mark_diameter=f.get("penaltyMarkDiameter"),
        )

    @property
    def outer_length(self) -> float:
        """Outside edge to outside edge, as quoted in the rule book's dimension table."""
        return self.length + self.line_width

    @property
    def outer_width(self) -> float:
        return self.width + self.line_width


@dataclass(frozen=True)
class Segment:
    a: tuple[float, float]
    b: tuple[float, float]
    name: str

    @property
    def length(self) -> float:
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])


@dataclass(frozen=True)
class Arc:
    """Circular arc, counter-clockwise from start to start + sweep (rad). sweep = 2π is a full circle."""

    center: tuple[float, float]
    radius: float
    start: float
    sweep: float
    name: str

    @property
    def length(self) -> float:
        return self.radius * self.sweep


@dataclass(frozen=True)
class Landmark:
    kind: str
    xy: tuple[float, float]
    name: str


def mirror_pose(x: float, y: float, theta: float) -> tuple[float, float, float]:
    """The pose on the other half that sees an identical field: 180° rotation about the center mark."""
    return -x, -y, math.atan2(math.sin(theta + math.pi), math.cos(theta + math.pi))


def mirror_points(points: np.ndarray) -> np.ndarray:
    return -np.asarray(points, float)


def _segment_distance(p: np.ndarray, s: Segment) -> np.ndarray:
    a = np.array(s.a, float)
    ab = np.array(s.b, float) - a
    t = np.clip(((p - a) @ ab) / (ab @ ab), 0.0, 1.0)
    return np.linalg.norm(p - (a + t[:, None] * ab), axis=1)


def _arc_distance(p: np.ndarray, arc: Arc) -> np.ndarray:
    c = np.array(arc.center, float)
    d = p - c
    r = np.hypot(d[:, 0], d[:, 1])
    radial = np.abs(r - arc.radius)
    if arc.sweep >= 2 * math.pi - 1e-9:
        return radial
    rel = np.mod(np.arctan2(d[:, 1], d[:, 0]) - arc.start, 2 * math.pi)
    ends = [arc.start, arc.start + arc.sweep]
    end_d = [np.hypot(*(p - (c + arc.radius * np.array([math.cos(a), math.sin(a)]))).T) for a in ends]
    return np.where(rel <= arc.sweep, radial, np.minimum(*end_d))


class FieldModel:
    """Field lines (centerlines), typed landmarks and helpers for one field definition."""

    def __init__(self, dims: FieldDims, name: str = "custom"):
        self.dims = dims
        self.name = name
        self.segments, self.arcs = self._build_lines()
        self.landmarks = self._build_landmarks()

    @classmethod
    def from_dict(cls, d: dict, name: str = "custom") -> FieldModel:
        return cls(FieldDims.from_dict(d), name)

    @classmethod
    def from_json(cls, path: str | Path) -> FieldModel:
        path = Path(path)
        return cls.from_dict(json.loads(path.read_text()), path.stem)

    @classmethod
    def builtin(cls, name: str) -> FieldModel:
        """One of BUILTIN_FIELDS: hsl_s_2026, hsl_m_2026, hsl_l_2026."""
        if name not in BUILTIN_FIELDS:
            raise KeyError(f"unknown field {name!r}; choose from {sorted(BUILTIN_FIELDS)}")
        return cls.from_dict(json.loads((FIELDS_DIR / BUILTIN_FIELDS[name]).read_text()), name)

    # ---------------------------------------------------------------- geometry

    def _build_lines(self) -> tuple[tuple[Segment, ...], tuple[Arc, ...]]:
        d = self.dims
        a, b = d.length / 2, d.width / 2
        segs = [
            Segment((-a, -b), (a, -b), "touchline_right"),
            Segment((-a, b), (a, b), "touchline_left"),
            Segment((0.0, -b), (0.0, b), "halfway_line"),
        ]
        for s, side in ((-1, "own"), (1, "opp")):
            segs.append(Segment((s * a, -b), (s * a, b), f"{side}_goal_line"))
            boxes = [("penalty_area", d.penalty_area_length, d.penalty_area_width)]
            if d.goal_area_length is not None and d.goal_area_width is not None:
                boxes.append(("goal_area", d.goal_area_length, d.goal_area_width))
            for box, length, width in boxes:
                x, y = s * (a - length), width / 2
                segs += [
                    Segment((s * a, -y), (x, -y), f"{side}_{box}_right"),
                    Segment((x, -y), (x, y), f"{side}_{box}_front"),
                    Segment((x, y), (s * a, y), f"{side}_{box}_left"),
                ]
        arcs = [Arc((0.0, 0.0), d.center_circle_diameter / 2, 0.0, 2 * math.pi, "center_circle")]
        if d.corner_arc_radius is not None:
            hw = d.line_width / 2
            for sx in (-1, 1):
                for sy in (-1, 1):
                    inward = math.atan2(-sy, -sx)  # diagonal pointing into the field
                    arcs.append(
                        Arc(
                            (sx * (a + hw), sy * (b + hw)),
                            d.corner_arc_radius,
                            inward - math.pi / 4,
                            math.pi / 2,
                            f"corner_arc_{'opp' if sx > 0 else 'own'}_{'left' if sy > 0 else 'right'}",
                        )
                    )
        return tuple(segs), tuple(arcs)

    def _build_landmarks(self) -> tuple[Landmark, ...]:
        d = self.dims
        a, b, r = d.length / 2, d.width / 2, d.center_circle_diameter / 2
        lms = [
            Landmark(CENTER_MARK, (0.0, 0.0), "center_mark"),
            Landmark(T_CROSSING, (0.0, -b), "halfway_touchline_right"),
            Landmark(T_CROSSING, (0.0, b), "halfway_touchline_left"),
            Landmark(X_CROSSING, (0.0, -r), "halfway_circle_right"),
            Landmark(X_CROSSING, (0.0, r), "halfway_circle_left"),
        ]
        post_y = d.goal_inner_width / 2 + d.goal_post_diameter / 2
        for s, side in ((-1, "own"), (1, "opp")):
            lms += [
                Landmark(L_CROSSING, (s * a, -b), f"{side}_corner_right"),
                Landmark(L_CROSSING, (s * a, b), f"{side}_corner_left"),
                Landmark(PENALTY_MARK, (s * (a - d.penalty_mark_distance), 0.0), f"{side}_penalty_mark"),
                # Posts are centered on the goal-line centerline. The rules only say goals sit on the goal
                # line; the error is at most half a line width.
                Landmark(GOALPOST, (s * a, -post_y), f"{side}_goalpost_right"),
                Landmark(GOALPOST, (s * a, post_y), f"{side}_goalpost_left"),
            ]
            boxes = [("penalty_area", d.penalty_area_length, d.penalty_area_width)]
            if d.goal_area_length is not None and d.goal_area_width is not None:
                boxes.append(("goal_area", d.goal_area_length, d.goal_area_width))
            for box, length, width in boxes:
                x, y = s * (a - length), width / 2
                lms += [
                    Landmark(T_CROSSING, (s * a, -y), f"{side}_{box}_goal_line_right"),
                    Landmark(T_CROSSING, (s * a, y), f"{side}_{box}_goal_line_left"),
                    Landmark(L_CROSSING, (x, -y), f"{side}_{box}_corner_right"),
                    Landmark(L_CROSSING, (x, y), f"{side}_{box}_corner_left"),
                ]
        return tuple(lms)

    # ---------------------------------------------------------------- queries

    def distance(self, points) -> np.ndarray:
        """Distance (m) from each (x, y) to the nearest line centerline. Accepts (2,) or (N, 2)."""
        p = np.atleast_2d(np.asarray(points, float))
        best = np.full(len(p), np.inf)
        for s in self.segments:
            best = np.minimum(best, _segment_distance(p, s))
        for arc in self.arcs:
            best = np.minimum(best, _arc_distance(p, arc))
        return best

    def sample_lines(self, spacing: float = 0.05) -> np.ndarray:
        """Points along every line centerline, about `spacing` apart. (N, 2)."""
        pts = []
        for s in self.segments:
            n = max(2, int(math.ceil(s.length / spacing)) + 1)
            t = np.linspace(0.0, 1.0, n)[:, None]
            pts.append(np.array(s.a) + t * (np.array(s.b) - np.array(s.a)))
        for arc in self.arcs:
            n = max(2, int(math.ceil(arc.length / spacing)) + 1)
            ang = arc.start + np.linspace(0.0, arc.sweep, n)
            pts.append(np.array(arc.center) + arc.radius * np.stack([np.cos(ang), np.sin(ang)], axis=1))
        return np.concatenate(pts)

    def landmarks_of(self, kind: str) -> tuple[Landmark, ...]:
        return tuple(lm for lm in self.landmarks if lm.kind == kind)

    def in_bounds(self, points, margin: float = 0.0) -> np.ndarray:
        """True where a point lies within the field lines plus `margin` (m)."""
        p = np.atleast_2d(np.asarray(points, float))
        return (np.abs(p[:, 0]) <= self.dims.length / 2 + margin) & (
            np.abs(p[:, 1]) <= self.dims.width / 2 + margin
        )

    def likelihood_field(self, resolution: float = 0.02, margin: float | None = None) -> LikelihoodField:
        """Precomputed distance-to-nearest-line grid over the field plus border strip (plus `margin`)."""
        if margin is None:
            margin = self.dims.border_strip_width
        hx, hy = self.dims.length / 2 + margin, self.dims.width / 2 + margin
        # ceil so the grid always reaches +hx/+hy, even when the span isn't a whole number of cells.
        xs = -hx + resolution * np.arange(math.ceil(2 * hx / resolution - 1e-9) + 1)
        ys = -hy + resolution * np.arange(math.ceil(2 * hy / resolution - 1e-9) + 1)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        grid = self.distance(np.stack([gx.ravel(), gy.ravel()], axis=1)).reshape(gx.shape)
        return LikelihoodField((float(xs[0]), float(ys[0])), resolution, grid.astype(np.float32))


class LikelihoodField:
    """Distance-to-nearest-line on a regular grid, indexed [ix, iy], with vectorized bilinear lookup."""

    def __init__(self, origin: tuple[float, float], resolution: float, grid: np.ndarray):
        self.origin = (float(origin[0]), float(origin[1]))
        self.resolution = float(resolution)
        self.grid = np.asarray(grid, np.float32)
        self.outside = float(self.grid.max())

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """(x_min, x_max, y_min, y_max) covered by the grid."""
        nx, ny = self.grid.shape
        x0, y0 = self.origin
        return x0, x0 + (nx - 1) * self.resolution, y0, y0 + (ny - 1) * self.resolution

    def lookup(self, points, outside: float | None = None) -> np.ndarray:
        """Bilinear distance at each (x, y). Points off the grid get `outside` (default: the grid maximum)."""
        p = np.atleast_2d(np.asarray(points, float))
        fx = (p[:, 0] - self.origin[0]) / self.resolution
        fy = (p[:, 1] - self.origin[1]) / self.resolution
        nx, ny = self.grid.shape
        valid = (fx >= 0) & (fy >= 0) & (fx <= nx - 1) & (fy <= ny - 1)
        ix = np.clip(np.floor(fx).astype(int), 0, nx - 2)
        iy = np.clip(np.floor(fy).astype(int), 0, ny - 2)
        tx, ty = np.clip(fx - ix, 0, 1), np.clip(fy - iy, 0, 1)
        g = self.grid
        v = (
            g[ix, iy] * (1 - tx) * (1 - ty)
            + g[ix + 1, iy] * tx * (1 - ty)
            + g[ix, iy + 1] * (1 - tx) * ty
            + g[ix + 1, iy + 1] * tx * ty
        )
        return np.where(valid, v, self.outside if outside is None else outside)

    def save(self, path: str | Path) -> None:
        np.savez_compressed(path, origin=np.array(self.origin), resolution=self.resolution, grid=self.grid)

    @classmethod
    def load(cls, path: str | Path) -> LikelihoodField:
        with np.load(path) as z:
            return cls(tuple(z["origin"]), float(z["resolution"]), z["grid"])
