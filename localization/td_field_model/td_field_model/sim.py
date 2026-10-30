"""Synthetic field observations for testing localization without hardware (LOC-04).

Pure numpy, no ROS. It mimics the real pipeline:
1. Project field points into the D435 color image from the TRUE camera pose.
2. Add pixel noise.
3. Project back to the floor with the BELIEVED camera pose (true pose + pitch bias + per-frame jitter).
   This is what the robot actually does with imperfect head angles.

Range error growing with distance squared, (h² + d²)/h per radian of pitch error (docs/roadmap.md
§2.3), falls out of the geometry instead of being added by hand.

Frames: world = field frame (td_field_model.field). Robot = base_footprint: x forward, y left, on the floor.
Observations are in the robot frame, as the localization filter will receive them.
Camera = optical frame: z forward, x right, y down. Its pose uses REP-103 pitch, positive looking down.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from td_field_model.field import FieldModel

# camera_link (x fwd, y left, z up) → optical (z fwd, x right, y down)
R_LINK_OPT = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)


@dataclass(frozen=True)
class CameraModel:
    """Pinhole camera on a pan/tilt head. Defaults: D435 color at 848×480 on a ~1.3 m robot."""

    fx: float = 615.0
    fy: float = 615.0
    cx: float = 424.0
    cy: float = 240.0
    width: int = 848
    height_px: int = 480
    mount_height: float = 1.2  # camera optical center above the floor (m)
    mount_forward: float = 0.0  # camera ahead of base_footprint (m)
    max_range: float = 9.0  # ignore floor points farther than this (horizontal, from the camera foot)

    @property
    def focal(self) -> float:
        return 0.5 * (self.fx + self.fy)

    def pose_in_base(self, head_yaw: float, head_pitch: float, height: float | None = None):
        """(R, t): optical frame in base_footprint, for the given neck angles (rad, pitch + = down)."""
        cp, sp = math.cos(head_pitch), math.sin(head_pitch)
        cy, sy = math.cos(head_yaw), math.sin(head_yaw)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        h = self.mount_height if height is None else height
        return rz @ ry @ R_LINK_OPT, np.array([self.mount_forward, 0.0, h])


@dataclass(frozen=True)
class NoiseModel:
    """Error sources. Defaults are a realistic guess for encoders + head IMU.

    NoiseModel.zero() is noise-free.
    """

    pixel_sigma: float = 1.0  # px, per detected point
    pitch_bias: float = 0.0  # rad, constant calibration error in the believed pitch
    pitch_jitter: float = math.radians(0.3)  # rad, per-frame believed-pitch error (sync, backlash)
    yaw_jitter: float = math.radians(0.3)  # rad, per-frame believed-yaw error
    point_dropout: float = 0.0  # probability each line point is missed
    frame_dropout: float = 0.0  # probability the whole frame yields nothing
    false_positives: float = 0.0  # mean number of spurious line points per frame (Poisson)
    crossing_confusion: float = 0.0  # probability an L/T/X crossing is reported as another crossing type
    bearing_only_beyond: float = 4.0  # m; farther observations are flagged range_ok = False
    max_line_points: int | None = 200  # detector output cap per frame (random subset); None = no cap

    @classmethod
    def zero(cls) -> NoiseModel:
        """No noise and no output cap, so results are exact and complete."""
        return cls(pixel_sigma=0.0, pitch_jitter=0.0, yaw_jitter=0.0, max_line_points=None)

    @property
    def assumed_angle_sigma(self) -> float:
        """Angle sigma the estimator should assume: jitter plus bias, in quadrature."""
        return math.hypot(self.pitch_jitter, self.pitch_bias)


@dataclass(frozen=True)
class Obstacle:
    """A robot or person standing on the field, modeled as a tall vertical cylinder that hides the floor."""

    x: float
    y: float
    radius: float = 0.25


@dataclass
class ObservedLandmark:
    kind: str
    xy: np.ndarray  # robot frame, m
    cov: np.ndarray  # 2×2, m²
    range_ok: bool
    truth_name: str | None  # for tests and evaluation only; a real detector does not know it


@dataclass
class Observations:
    """One camera frame's detections in the robot frame (base_footprint)."""

    t: float
    line_points: np.ndarray  # (N, 2)
    line_cov: np.ndarray  # (N, 2, 2)
    line_range_ok: np.ndarray  # (N,) bool; False → use the bearing only
    landmarks: list[ObservedLandmark] = field(default_factory=list)
    line_is_false: np.ndarray | None = None  # (N,) bool, for tests only
    true_pose: tuple[float, float, float] | None = None  # for tests and evaluation only

    @property
    def bearings(self) -> np.ndarray:
        return np.arctan2(self.line_points[:, 1], self.line_points[:, 0])

    def landmarks_of(self, kind: str) -> list[ObservedLandmark]:
        return [lm for lm in self.landmarks if lm.kind == kind]


def world_to_robot(points: np.ndarray, pose) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    d = np.atleast_2d(points) - np.array([x, y])
    return np.stack([c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1]], axis=1)


def robot_to_world(points: np.ndarray, pose) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    p = np.atleast_2d(points)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], axis=1)


def project_floor(cam: CameraModel, pts_robot: np.ndarray, r_bc: np.ndarray, t_bc: np.ndarray):
    """Floor points (robot frame) → pixels (N, 2) and an in-image mask; (r_bc, t_bc) from cam.pose_in_base."""
    p = np.column_stack([pts_robot, np.zeros(len(pts_robot))])
    pc = (p - t_bc) @ r_bc  # = R^T (p - t), row-wise
    z = pc[:, 2]
    ok = z > 0.05
    zs = np.where(ok, z, 1.0)
    uv = np.stack([cam.fx * pc[:, 0] / zs + cam.cx, cam.fy * pc[:, 1] / zs + cam.cy], axis=1)
    ok &= (uv[:, 0] >= 0) & (uv[:, 0] < cam.width) & (uv[:, 1] >= 0) & (uv[:, 1] < cam.height_px)
    return uv, ok


def backproject_pixels(cam: CameraModel, uv: np.ndarray, r_bc: np.ndarray, t_bc: np.ndarray):
    """Pixels → floor points (robot frame) for a camera pose in base_footprint.

    Also returns a mask for rays that hit the floor. Used by the simulator and by real-image detectors
    (td_bench).
    """
    uv = np.asarray(uv, float).reshape(-1, 2)
    rays_c = np.stack([(uv[:, 0] - cam.cx) / cam.fx, (uv[:, 1] - cam.cy) / cam.fy, np.ones(len(uv))], axis=1)
    rays_b = rays_c @ r_bc.T
    down = rays_b[:, 2] < -1e-9
    s = np.where(down, -t_bc[2] / np.where(down, rays_b[:, 2], -1.0), 0.0)
    pts = t_bc[:2] + s[:, None] * rays_b[:, :2]
    return pts, down


def floor_covariances(
    cam: CameraModel,
    pts: np.ndarray,
    t_bc: np.ndarray,
    pixel_sigma: float,
    pitch_sigma: float,
    yaw_sigma: float,
) -> np.ndarray:
    """2×2 robot-frame covariance per floor point from pixel noise plus camera-angle noise.

    Polar about the camera foot:
    σ_range = (h² + d²)/h · σ_pitch_total, σ_lateral = √(h² + d²) · σ_yaw_total.
    """
    h = t_bc[2]
    rel = pts - t_bc[:2]
    d = np.hypot(rel[:, 0], rel[:, 1])
    pix = pixel_sigma / cam.focal
    s_pitch = math.hypot(pix, pitch_sigma)
    s_yaw = math.hypot(pix, yaw_sigma)
    s_r = (h * h + d * d) / h * s_pitch + 1e-3
    s_t = np.sqrt(h * h + d * d) * s_yaw + 1e-3
    b = np.arctan2(rel[:, 1], rel[:, 0])
    c, s = np.cos(b), np.sin(b)
    cov = np.empty((len(pts), 2, 2))
    cov[:, 0, 0] = c * c * s_r**2 + s * s * s_t**2
    cov[:, 1, 1] = s * s * s_r**2 + c * c * s_t**2
    cov[:, 0, 1] = cov[:, 1, 0] = c * s * (s_r**2 - s_t**2)
    return cov


def _occluded(origin: np.ndarray, pts: np.ndarray, obstacles) -> np.ndarray:
    """True where the 2-D sight line origin → point passes through an obstacle disc in front of the point."""
    hidden = np.zeros(len(pts), bool)
    for ob in obstacles:
        c = np.array([ob.x, ob.y])
        d = pts - origin
        dd = np.einsum("ij,ij->i", d, d)
        t = np.clip(((c - origin) @ d.T) / np.maximum(dd, 1e-12), 0.0, 1.0)
        closest = origin + t[:, None] * d
        hits = np.linalg.norm(closest - c, axis=1) < ob.radius
        # The obstacle must stand between camera and point, not behind the point.
        hidden |= hits & (t < 1.0 - 1e-9)
    return hidden


class Simulator:
    """Renders noisy field observations for a robot pose, head angles and obstacles."""

    def __init__(
        self,
        field_model: FieldModel,
        camera: CameraModel | None = None,
        noise: NoiseModel | None = None,
        seed: int = 0,
        line_spacing: float = 0.03,
    ):
        self.field = field_model
        self.camera = camera or CameraModel()
        self.noise = noise or NoiseModel()
        self.rng = np.random.default_rng(seed)
        self._line_world = field_model.sample_lines(spacing=line_spacing)

    # ---------------------------------------------------------------- camera geometry

    def _project(self, pts_robot: np.ndarray, r_bc: np.ndarray, t_bc: np.ndarray):
        return project_floor(self.camera, pts_robot, r_bc, t_bc)

    def _backproject(self, uv: np.ndarray, r_bc: np.ndarray, t_bc: np.ndarray):
        return backproject_pixels(self.camera, uv, r_bc, t_bc)

    def _covariances(self, pts: np.ndarray, t_bc: np.ndarray) -> np.ndarray:
        n = self.noise
        return floor_covariances(self.camera, pts, t_bc, n.pixel_sigma, n.assumed_angle_sigma, n.yaw_jitter)

    # ---------------------------------------------------------------- rendering

    def render(self, pose, head_yaw: float = 0.0, head_pitch: float = 0.35, obstacles=(), t: float = 0.0):
        """Observations for a robot at world `pose` = (x, y, θ) with the given neck angles (rad)."""
        n, cam, rng = self.noise, self.camera, self.rng
        r_true, t_true = cam.pose_in_base(head_yaw, head_pitch)
        r_bel, t_bel = cam.pose_in_base(
            head_yaw + rng.normal(0.0, n.yaw_jitter) if n.yaw_jitter else head_yaw,
            head_pitch + n.pitch_bias + (rng.normal(0.0, n.pitch_jitter) if n.pitch_jitter else 0.0),
        )
        empty = Observations(
            t, np.zeros((0, 2)), np.zeros((0, 2, 2)), np.zeros(0, bool), true_pose=tuple(pose)
        )
        if n.frame_dropout and rng.random() < n.frame_dropout:
            empty.line_is_false = np.zeros(0, bool)
            return empty

        cam_world = robot_to_world(t_true[:2], pose)[0]
        lines, line_false = self._observe_points(
            world_to_robot(self._line_world, pose), cam_world, pose, obstacles, r_true, t_true, r_bel, t_bel
        )
        keep = np.ones(len(lines), bool)
        if n.point_dropout:
            keep &= rng.random(len(lines)) >= n.point_dropout
        lines, line_false = lines[keep], line_false[keep]
        if n.max_line_points is not None and len(lines) > n.max_line_points:
            idx = rng.choice(len(lines), n.max_line_points, replace=False)
            lines, line_false = lines[idx], line_false[idx]
        fps = self._false_points(r_true, t_true, r_bel, t_bel)
        lines = np.vstack([lines, fps])
        line_false = np.concatenate([line_false, np.ones(len(fps), bool)])

        cov = self._covariances(lines, t_bel)
        range_ok = np.hypot(*(lines - t_bel[:2]).T) <= n.bearing_only_beyond
        obs = Observations(t, lines, cov, range_ok, line_is_false=line_false, true_pose=tuple(pose))
        obs.landmarks = self._observe_landmarks(pose, cam_world, obstacles, r_true, t_true, r_bel, t_bel)
        return obs

    def _observe_points(self, pts_robot, cam_world, pose, obstacles, r_true, t_true, r_bel, t_bel):
        uv, ok = self._project(pts_robot, r_true, t_true)
        rel = pts_robot - t_true[:2]
        ok &= np.hypot(rel[:, 0], rel[:, 1]) <= self.camera.max_range
        if obstacles:
            ok &= ~_occluded(cam_world, robot_to_world(pts_robot, pose), obstacles)
        uv = uv[ok]
        if self.noise.pixel_sigma:
            uv = uv + self.rng.normal(0.0, self.noise.pixel_sigma, uv.shape)
        pts, hit = self._backproject(uv, r_bel, t_bel)
        return pts[hit], np.zeros(int(hit.sum()), bool)

    def _false_points(self, r_true, t_true, r_bel, t_bel) -> np.ndarray:
        """Spurious detections (jerseys, shoes, other fields): random pixels in the lower image half."""
        k = self.rng.poisson(self.noise.false_positives) if self.noise.false_positives else 0
        if not k:
            return np.zeros((0, 2))
        cam = self.camera
        uv = np.column_stack(
            [self.rng.uniform(0, cam.width, k), self.rng.uniform(cam.height_px / 2, cam.height_px, k)]
        )
        pts, hit = self._backproject(uv, r_bel, t_bel)
        return pts[hit]

    def _observe_landmarks(self, pose, cam_world, obstacles, r_true, t_true, r_bel, t_bel):
        out = []
        lms = self.field.landmarks
        world = np.array([lm.xy for lm in lms])
        robot = world_to_robot(world, pose)
        uv, ok = self._project(robot, r_true, t_true)
        rel = robot - t_true[:2]
        ok &= np.hypot(rel[:, 0], rel[:, 1]) <= self.camera.max_range
        if obstacles:
            ok &= ~_occluded(cam_world, world, obstacles)
        crossing_kinds = ("L", "T", "X")
        for i in np.flatnonzero(ok):
            if self.noise.point_dropout and self.rng.random() < self.noise.point_dropout:
                continue
            px = uv[i] + (self.rng.normal(0.0, self.noise.pixel_sigma, 2) if self.noise.pixel_sigma else 0.0)
            pts, hit = self._backproject(px[None, :], r_bel, t_bel)
            if not hit[0]:
                continue
            kind = lms[i].kind
            if kind in crossing_kinds and self.noise.crossing_confusion:
                if self.rng.random() < self.noise.crossing_confusion:
                    kind = self.rng.choice([k for k in crossing_kinds if k != kind])
            cov = self._covariances(pts, t_bel)[0]
            d = float(np.hypot(*(pts[0] - t_bel[:2])))
            out.append(ObservedLandmark(kind, pts[0], cov, d <= self.noise.bearing_only_beyond, lms[i].name))
        return out
