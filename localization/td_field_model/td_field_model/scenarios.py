"""Ground-truth trajectories with noisy odometry and game events, for localization tests (LOC-04).

Pure numpy, no ROS. A ScenarioBuilder strings together walking, standing, falls, kidnaps and penalty
re-entries.
It yields Steps carrying:
- the true pose;
- what odometry would report since the previous step (robot frame, noisy);
- the head angles;
- any event.

Pair each Step with Simulator.render() to get camera observations.

Events and what the localization filter is allowed to know:
- "initial": start of the scenario. `known_poses` holds the placement candidates the game rules imply.
- "fall": the robot fell and got up. Odometry reports no motion with inflated covariance; the true heading
  changed.
- "kidnap": carried somewhere without notice (tests only). Odometry reports nothing. `known_poses` is empty.
- "penalty_reentry": placed by rule on its own-half touchline (docs/roadmap.md §2.4).
  `known_poses` holds both touchline candidates. Which one is true is for vision to decide.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from td_field_model.field import FieldModel

Pose = tuple[float, float, float]


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def relative_motion(p0: Pose, p1: Pose) -> np.ndarray:
    """Motion from p0 to p1 expressed in p0's robot frame: (dx, dy, dθ)."""
    c, s = math.cos(p0[2]), math.sin(p0[2])
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(p1[2] - p0[2])])


def compose(p: Pose, d) -> Pose:
    """Apply a robot-frame motion d = (dx, dy, dθ) to pose p."""
    c, s = math.cos(p[2]), math.sin(p[2])
    return (p[0] + c * d[0] - s * d[1], p[1] + s * d[0] + c * d[1], wrap(p[2] + d[2]))


def penalty_reentry_poses(fm: FieldModel, robot_clearance: float = 0.25) -> list[Pose]:
    """Both legal re-entry placements: own-half touchline level with the penalty mark, outside the field,
    facing the opposite touchline (HSL 2026 rules, standard removal penalty)."""
    d = fm.dims
    x = -(d.length / 2 - d.penalty_mark_distance)
    y = d.width / 2 + d.line_width / 2 + robot_clearance
    return [(x, y, -math.pi / 2), (x, -y, math.pi / 2)]


def kickoff_poses(fm: FieldModel) -> list[Pose]:
    """A plausible 5-robot own-half formation facing the opponent goal.

    Goalkeeper, 2 defenders, 2 forwards.
    """
    d = fm.dims
    a, b = d.length / 2, d.width / 2
    r = d.center_circle_diameter / 2
    return [
        (-a + 0.5, 0.0, 0.0),
        (-a / 2, b / 3, 0.0),
        (-a / 2, -b / 3, 0.0),
        (-r - 0.5, b / 4, 0.0),
        (-r - 0.5, -b / 4, 0.0),
    ]


def sideline_start_poses(fm: FieldModel, n: int = 5, robot_clearance: float = 0.25) -> list[Pose]:
    """Robots lined up on both own-half touchlines facing in, as handlers often place them in Initial."""
    d = fm.dims
    a = d.length / 2
    y = d.width / 2 + d.line_width / 2 + robot_clearance
    xs = np.linspace(-a + 1.0, -1.0, (n + 1) // 2)
    poses = [(float(x), y, -math.pi / 2) for x in xs] + [(float(x), -y, math.pi / 2) for x in xs]
    return poses[:n]


@dataclass(frozen=True)
class OdometryNoise:
    """Relative odometry error: sigma grows with distance and turn, plus a floor."""

    trans_frac: float = 0.05  # σ of translation error per meter travelled
    rot_frac: float = 0.05  # σ of rotation error per radian turned
    rot_per_meter: float = math.radians(2.0)  # σ of rotation error per meter travelled
    floor: float = 1e-4

    @classmethod
    def zero(cls) -> OdometryNoise:
        return cls(0.0, 0.0, 0.0, 0.0)

    def sigmas(self, delta: np.ndarray) -> np.ndarray:
        dist = math.hypot(delta[0], delta[1])
        st = self.trans_frac * dist + self.floor
        sr = self.rot_frac * abs(delta[2]) + self.rot_per_meter * dist + self.floor
        return np.array([st, st, sr])


@dataclass(frozen=True)
class HeadMotion:
    """Head scan: yaw sweeps sinusoidally, pitch is fixed."""

    yaw_amplitude: float = 0.8  # rad
    period: float = 4.0  # s
    pitch: float = 0.35  # rad, + = looking down

    def at(self, t: float) -> tuple[float, float]:
        if self.yaw_amplitude == 0.0:
            return 0.0, self.pitch
        return self.yaw_amplitude * math.sin(2 * math.pi * t / self.period), self.pitch


@dataclass
class Step:
    t: float
    true_pose: Pose
    odom: np.ndarray  # (dx, dy, dθ) since the previous step, robot frame, as odometry reports it
    odom_cov: np.ndarray  # 3×3
    head_yaw: float
    head_pitch: float
    event: str | None = None
    known_poses: list[Pose] = field(default_factory=list)


class ScenarioBuilder:
    """Chainable builder. Example:

    steps = (ScenarioBuilder(fm, seed=1).start(pose).walk(10).fall().walk(5)
             .penalty_reentry("left").walk(10).steps)
    """

    def __init__(
        self,
        fm: FieldModel,
        seed: int = 0,
        dt: float = 1 / 30,
        odom_noise: OdometryNoise | None = None,
        head: HeadMotion | None = None,
        fall_odom_sigma: float = 0.3,
    ):
        self.field = fm
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.odom_noise = odom_noise or OdometryNoise()
        self.head = head or HeadMotion()
        self.fall_odom_sigma = fall_odom_sigma
        self.steps: list[Step] = []
        self._turn_rate = 0.0

    @property
    def t(self) -> float:
        return self.steps[-1].t if self.steps else 0.0

    @property
    def pose(self) -> Pose:
        if not self.steps:
            raise RuntimeError("call start() first")
        return self.steps[-1].true_pose

    def _append(self, pose: Pose, odom, cov, event=None, known=(), t: float | None = None) -> None:
        t = self.t + self.dt if t is None else t
        yaw, pitch = self.head.at(t)
        self.steps.append(Step(t, pose, np.asarray(odom, float), cov, yaw, pitch, event, list(known)))

    def _still(self, pose: Pose, event: str, known=(), sigma: float = 1e-4) -> None:
        self._append(pose, np.zeros(3), np.diag([sigma**2] * 3), event, known)

    def start(self, pose: Pose, known_poses=None) -> ScenarioBuilder:
        if self.steps:
            raise RuntimeError("start() must come first")
        known = [pose] if known_poses is None else list(known_poses)
        yaw, pitch = self.head.at(0.0)
        self.steps.append(Step(0.0, pose, np.zeros(3), np.diag([1e-8] * 3), yaw, pitch, "initial", known))
        return self

    def _measured(self, true_delta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        sig = self.odom_noise.sigmas(true_delta)
        noisy = true_delta + self.rng.normal(0.0, 1.0, 3) * sig if np.any(sig) else true_delta.copy()
        return noisy, np.diag(np.maximum(sig, 1e-6) ** 2)

    def walk(self, duration: float, speed: float = 0.3, max_turn_rate: float = 0.6) -> ScenarioBuilder:
        """Wander at `speed` (m/s), with a smoothly varying turn rate that steers back inside the field lines.

        Steering back starts `speed / max_turn_rate` (the turning radius) plus 0.3 m inside the lines, so the
        robot never strays more than about 0.3 m outside them.
        """
        margin = speed / max_turn_rate + 0.3
        a, b = self.field.dims.length / 2 - margin, self.field.dims.width / 2 - margin
        for _ in range(int(round(duration / self.dt))):
            x, y, th = self.pose
            self._turn_rate += -0.5 * self._turn_rate * self.dt + 0.6 * math.sqrt(self.dt) * self.rng.normal()
            w = float(np.clip(self._turn_rate, -max_turn_rate, max_turn_rate))
            if abs(x) > a or abs(y) > b:  # head back toward the center
                w = max_turn_rate * float(np.sign(wrap(math.atan2(-y, -x) - th)) or 1.0)
            true_delta = np.array([speed * self.dt, 0.0, w * self.dt])
            new = compose(self.pose, true_delta)
            odom, cov = self._measured(relative_motion(self.pose, new))
            self._append(new, odom, cov)
        return self

    def turn(self, angle: float, rate: float = 0.5) -> ScenarioBuilder:
        n = max(1, int(round(abs(angle) / rate / self.dt)))
        for _ in range(n):
            new = compose(self.pose, (0.0, 0.0, angle / n))
            odom, cov = self._measured(relative_motion(self.pose, new))
            self._append(new, odom, cov)
        return self

    def stand(self, duration: float) -> ScenarioBuilder:
        for _ in range(int(round(duration / self.dt))):
            self._append(self.pose, np.zeros(3), np.diag([1e-8] * 3))
        return self

    def fall(self, heading_change: float | None = None, shift: float = 0.2) -> ScenarioBuilder:
        """Fall and get up: true pose shifts and turns; odometry reports nothing but a large covariance."""
        dth = self.rng.uniform(-math.pi / 3, math.pi / 3) if heading_change is None else heading_change
        x, y, th = self.pose
        ang = self.rng.uniform(-math.pi, math.pi)
        new = (x + shift * math.cos(ang), y + shift * math.sin(ang), wrap(th + dth))
        self._still(new, "fall", sigma=self.fall_odom_sigma)
        return self

    def kidnap(self, pose: Pose) -> ScenarioBuilder:
        """Teleport without telling the filter (robustness tests only)."""
        self._still(pose, "kidnap")
        return self

    def penalty_reentry(self, touchline: str = "left") -> ScenarioBuilder:
        """Removal penalty served: placed on the own-half `touchline` ("left" = +y).

        Both candidates are known to the filter.
        """
        candidates = penalty_reentry_poses(self.field)
        if touchline not in ("left", "right"):
            raise ValueError("touchline must be 'left' or 'right'")
        self._still(candidates[0 if touchline == "left" else 1], "penalty_reentry", known=candidates)
        return self

    def poses(self) -> np.ndarray:
        return np.array([s.true_pose for s in self.steps])
