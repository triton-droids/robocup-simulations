"""Pluggable inputs for bench-testing localization with any mix of real, recorded and simulated parts.

Every source answers `get(t)` for the bench clock time t (seconds from the start of the run):

| Input       | Simulated                     | Recorded       | Real / live                                |
|-------------|-------------------------------|----------------|--------------------------------------------|
| camera      | SimCamera, ImageCamera(Synth) | ReplayCamera   | ImageCamera + FolderFrames/RealSenseFrames |
| odometry    | SimOdometry                   | ReplayOdometry | StationaryOdometry (tripod), LiveOdometry  |
| head angles | SimHead                       | ReplayHead     | FixedHead (tripod), LiveHead               |
| game events | ScriptedGame                  | ReplayGame     | UdpGame (GameController over UDP)          |
| teammates   | SimTeam                       |                | UdpTeam (TeamLink + TeamComm)              |
| ball (mine) | SimBall                       |                | (needs a ball detector; not here)          |

"Live" sources are push-based (`push(...)` from another thread or a ROS callback) so real encoders, IMU or a
ROS bridge can feed the bench before the LOC-20 node exists. Simulated sources need a `World` (ground truth:
the scenario steps the simulated parts share).

Visual-only mixes: SimVisualOdometry (cuVSLAM model) / LiveVisualOdometry (real cuVSLAM poses) replace leg
odometry, and ImuHead models head angles from encoders + head IMU.
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field

import numpy as np
from td_field_model.scenarios import Step
from td_field_model.sim import Observations, Simulator, world_to_robot
from td_game.team_msg import remaining_game_seconds
from td_localization.side import TeamReport

from td_field_model import FieldModel
from td_game import GameEvent, GameStateTracker, TeamComm, TeamMessage

# ---------------------------------------------------------------- shared simulated world


@dataclass
class World:
    """Ground truth for the simulated parts: scenario steps on a fixed dt (td_field_model.scenarios)."""

    steps: list[Step]

    @property
    def dt(self) -> float:
        return self.steps[1].t - self.steps[0].t if len(self.steps) > 1 else 1 / 30

    def index(self, t: float) -> int:
        return int(min(max(round(t / self.dt), 0), len(self.steps) - 1))

    def step(self, t: float) -> Step:
        return self.steps[self.index(t)]

    @property
    def duration(self) -> float:
        return self.steps[-1].t


# ---------------------------------------------------------------- odometry


class SimOdometry:
    """Odometry from the scenario (noisy, as a walking robot would report it), accumulated between calls."""

    def __init__(self, world: World):
        self.world = world
        self._last = 0

    def get(self, t: float):
        i = self.world.index(t)
        d, cov = np.zeros(3), np.zeros((3, 3))
        for k in range(self._last + 1, i + 1):
            s = self.world.steps[k]
            d = _compose_rel(d, s.odom)
            cov = cov + s.odom_cov
        self._last = max(self._last, i)
        return d, cov


class StationaryOdometry:
    """A robot or tripod that doesn't move (camera-only bench): zero motion with a small floor covariance."""

    def __init__(self, sigma=(0.002, 0.002, 0.002)):
        self.cov = np.diag(np.square(sigma))

    def get(self, t: float):
        return np.zeros(3), self.cov


class LiveOdometry:
    """Push-based odometry for real encoders / IMU / a ROS callback (thread-safe).

    Call `push(dx, dy, dtheta, cov)` with each robot-frame increment. `get(t)` returns everything accumulated
    since
    the last call.
    """

    def __init__(self, floor_sigma=(0.002, 0.002, 0.002)):
        self._lock = threading.Lock()
        self._d = np.zeros(3)
        self._cov = np.zeros((3, 3))
        self._floor = np.diag(np.square(floor_sigma))

    def push(self, dx: float, dy: float, dtheta: float, cov=None) -> None:
        with self._lock:
            self._d = _compose_rel(self._d, np.array([dx, dy, dtheta]))
            self._cov = self._cov + (np.zeros((3, 3)) if cov is None else np.asarray(cov, float))

    def get(self, t: float):
        with self._lock:
            d, cov = self._d, self._cov + self._floor
            self._d, self._cov = np.zeros(3), np.zeros((3, 3))
        return d, cov


class SimVisualOdometry:
    """Simulated cuVSLAM stereo-inertial odometry (roadmap §3.6), reported as body motion.

    cuVSLAM tracks the *camera*; body motion = camera motion minus head motion, so errors in the head-angle
    source (`head`, e.g. ImuHead) leak into the body yaw. Model, per run / per tick:
    - translation scale error ~ N(0, scale_sigma) (turf: worse than the paper's 0.39% on EuRoC);
    - white noise on translation (`trans_noise` × distance) and yaw;
    - a constant gyro yaw-rate bias ~ N(0, yaw_bias_sigma) (rad/s);
    - tracking loss while the head pans faster than `pan_gate` (rad/s) and in random episodes
      (`loss_rate` per second, `loss_duration` s). While lost there is no visual motion: zero increment with
      the covariance of `blind_speed` (the robot may be walking), as the odometry filter would do without VO.
    """

    def __init__(
        self,
        world: World,
        head=None,
        seed: int = 0,
        scale_sigma: float = 0.03,
        trans_noise: float = 0.05,
        yaw_noise: float = math.radians(0.05),
        yaw_bias_sigma: float = math.radians(1.5) / 60,
        pan_gate: float = 1.2,
        loss_rate: float = 0.02,
        loss_duration: float = 1.0,
        blind_speed=(0.35, 0.35, 0.6),
    ):
        self.world, self.head = world, head
        self.rng = np.random.default_rng(seed)
        self.scale = 1.0 + self.rng.normal(0, scale_sigma)
        self.yaw_bias = self.rng.normal(0, yaw_bias_sigma)
        self.trans_noise, self.yaw_noise = trans_noise, yaw_noise
        self.pan_gate, self.loss_rate, self.loss_duration = pan_gate, loss_rate, loss_duration
        self.blind = np.square(np.asarray(blind_speed, float) * world.dt)
        self._last = 0
        self._lost_until = -1.0
        self.lost_ticks = 0
        self.ticks = 0

    def _tracking(self, t: float, pan_rate: float) -> bool:
        if t < self._lost_until:
            return False
        if self.rng.random() < self.loss_rate * self.world.dt:
            self._lost_until = t + self.loss_duration
            return False
        return abs(pan_rate) <= self.pan_gate

    def get(self, t: float):
        from td_field_model.scenarios import relative_motion

        i = self.world.index(t)
        d, cov = np.zeros(3), np.zeros((3, 3))
        for k in range(self._last + 1, i + 1):
            s0, s1 = self.world.steps[k - 1], self.world.steps[k]
            dt = s1.t - s0.t
            self.ticks += 1
            pan_rate = (s1.head_yaw - s0.head_yaw) / dt
            if not self._tracking(s1.t, pan_rate):
                self.lost_ticks += 1
                cov = cov + np.diag(self.blind)
                continue
            true = relative_motion(s0.true_pose, s1.true_pose)
            dist = float(np.hypot(true[0], true[1]))
            xy = true[:2] * self.scale + self.rng.normal(0, self.trans_noise * dist + 1e-4, 2)
            # head subtraction with the *believed* head yaw: its rate error leaks into the body yaw
            leak = 0.0
            if self.head is not None:
                believed = self.head.get(s1.t)[0] - self.head.get(s0.t)[0]
                leak = (s1.head_yaw - s0.head_yaw) - believed
            dth = true[2] + self.yaw_bias * dt + leak + self.rng.normal(0, self.yaw_noise)
            d = _compose_rel(d, np.array([xy[0], xy[1], dth]))
            sx = self.trans_noise * dist + 1e-3
            cov = cov + np.diag([sx**2, sx**2, (self.yaw_noise + 1e-3) ** 2])
        self._last = max(self._last, i)
        return d, cov

    @property
    def lost_fraction(self) -> float:
        return self.lost_ticks / max(1, self.ticks)


class LiveVisualOdometry:
    """Real cuVSLAM (or any VO) output: push the body pose in the VO's odom frame; get() returns increments.

    `push_pose(x, y, yaw, tracking=True)`, e.g. from /visual_slam/tracking/odometry after head-motion
    subtraction. While tracking is reported lost, increments are replaced by `blind_speed` covariance. The VO
    frame may jump on recovery, so the first pose after a loss is only used as the new reference.
    """

    def __init__(
        self,
        sigma_per_m: float = 0.05,
        yaw_sigma: float = math.radians(0.1),
        blind_speed=(0.35, 0.35, 0.6),
        dt: float = 1 / 30,
    ):
        self._lock = threading.Lock()
        self._ref = None
        self._d = np.zeros(3)
        self._cov = np.zeros((3, 3))
        self._lost = False
        self.sigma_per_m, self.yaw_sigma = sigma_per_m, yaw_sigma
        self._blind = np.diag(np.square(np.asarray(blind_speed, float) * dt))

    def push_pose(self, x: float, y: float, yaw: float, tracking: bool = True) -> None:
        with self._lock:
            if not tracking:
                self._lost, self._ref = True, None
                return
            p = np.array([x, y, yaw], float)
            if self._ref is not None:
                c, s = math.cos(self._ref[2]), math.sin(self._ref[2])
                dx, dy = p[0] - self._ref[0], p[1] - self._ref[1]
                inc = np.array(
                    [c * dx + s * dy, -s * dx + c * dy, math.remainder(p[2] - self._ref[2], 2 * math.pi)]
                )
                self._d = _compose_rel(self._d, inc)
                sx = self.sigma_per_m * float(np.hypot(dx, dy)) + 1e-3
                self._cov = self._cov + np.diag([sx**2, sx**2, self.yaw_sigma**2])
            self._ref, self._lost = p, False

    def get(self, t: float):
        with self._lock:
            d, cov = self._d, self._cov + (self._blind if self._lost else 0.0)
            self._d, self._cov = np.zeros(3), np.zeros((3, 3))
        return d, cov


def _compose_rel(a: np.ndarray, b) -> np.ndarray:
    """Robot-frame motion a followed by b, as one motion."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array(
        [
            a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            math.atan2(math.sin(a[2] + b[2]), math.cos(a[2] + b[2])),
        ]
    )


# ---------------------------------------------------------------- head angles


class SimHead:
    def __init__(self, world: World):
        self.world = world

    def get(self, t: float):
        s = self.world.step(t)
        return s.head_yaw, s.head_pitch


class ImuHead:
    """Head angles as the encoder + head-IMU filter would report them (roadmap §3.5 layer 1): the true angles
    plus a per-run calibration bias and white noise. pitch + = looking down."""

    def __init__(
        self,
        world: World,
        seed: int = 0,
        pitch_bias_sigma: float = math.radians(0.5),
        yaw_bias_sigma: float = math.radians(0.5),
        noise: float = math.radians(0.2),
    ):
        self.world = world
        self.rng = np.random.default_rng(seed + 7919)
        self.pitch_bias = self.rng.normal(0, pitch_bias_sigma)
        self.yaw_bias = self.rng.normal(0, yaw_bias_sigma)
        self.noise = noise
        self._cache: dict[int, tuple[float, float]] = {}

    def get(self, t: float):
        i = self.world.index(t)
        if i not in self._cache:  # the same tick reads the same sample (camera and VO both ask)
            s = self.world.steps[i]
            e = self.rng.normal(0, self.noise, 2)
            self._cache[i] = (s.head_yaw + self.yaw_bias + e[0], s.head_pitch + self.pitch_bias + e[1])
        return self._cache[i]


class FixedHead:
    """A camera on a tripod or a head that doesn't move: constant (yaw, pitch), pitch + = looking down."""

    def __init__(self, yaw: float = 0.0, pitch: float = 0.35):
        self.yaw, self.pitch = yaw, pitch

    def get(self, t: float):
        return self.yaw, self.pitch


class LiveHead:
    """Latest (yaw, pitch) pushed by real head encoders / the head filter (thread-safe)."""

    def __init__(self, yaw: float = 0.0, pitch: float = 0.35):
        self._lock = threading.Lock()
        self._v = (yaw, pitch)

    def push(self, yaw: float, pitch: float) -> None:
        with self._lock:
            self._v = (yaw, pitch)

    def get(self, t: float):
        with self._lock:
            return self._v


# ---------------------------------------------------------------- camera (observations)


class SimCamera:
    """Field observations straight from the simulator (no images), at the true pose and true head angles."""

    def __init__(self, world: World, simulator: Simulator, every: int = 2):
        self.world, self.sim, self.every = world, simulator, every

    def get(self, t: float) -> Observations | None:
        i = self.world.index(t)
        if i % self.every:
            return None
        s = self.world.steps[i]
        return self.sim.render(s.true_pose, s.head_yaw, s.head_pitch, t=t)


# ImageCamera, SyntheticFrames, FolderFrames, RealSenseFrames live in td_bench.vision; ReplayCamera in
# td_bench.record.


# ---------------------------------------------------------------- game events


class ScriptedGame:
    """Game events at given times, e.g. [(0, "initial"), (20, "penalized", "pushing"), (65, "reentry")]."""

    def __init__(self, script):
        self._events = sorted((float(e[0]), e[1], e[2] if len(e) > 2 else "") for e in script)
        self._i = 0

    def get(self, t: float) -> list[GameEvent]:
        out = []
        while self._i < len(self._events) and self._events[self._i][0] <= t:
            te, kind, detail = self._events[self._i]
            out.append(GameEvent(te, kind, detail, kind == "unpenalized_in_place"))
            self._i += 1
        return out


class UdpGame:
    """Live GameController: polls a td_game.udp.GameControllerLink and turns packets into events."""

    def __init__(self, link, team_number: int, player_number: int):
        self.link = link
        self.tracker = GameStateTracker(team_number, player_number)

    def get(self, t: float) -> list[GameEvent]:
        out = []
        for data in self.link.poll():
            out += self.tracker.update(t, data)
        return out + self.tracker.check_timeout(t)


# ---------------------------------------------------------------- my ball


class SimBall:
    """My own sightings of a simulated ball (td_localization.suite.BallTrack), within range and FOV."""

    def __init__(
        self, world: World, ball, seed: int = 0, see_range: float = 6.0, half_fov: float = math.radians(35)
    ):
        self.world, self.ball = world, ball
        self.rng = np.random.default_rng(seed)
        self.see_range, self.half_fov = see_range, half_fov
        self._t = 0.0
        self.position = np.array(ball.xy, float)

    def get(self, t: float):
        """(robot-frame xy, sigma) if seen now, else None. Advances the ball to t."""
        if t > self._t:
            self.position = self.ball.step(t - self._t)
            self._t = t
        s = self.world.step(t)
        rel = world_to_robot(self.position, s.true_pose)[0]
        d = float(np.hypot(*rel))
        bearing = math.atan2(rel[1], rel[0]) - s.head_yaw
        if d > self.see_range or abs(math.atan2(math.sin(bearing), math.cos(bearing))) > self.half_fov:
            return None
        sigma = 0.1 + 0.05 * d
        return rel + self.rng.normal(0, sigma, 2), sigma


# ---------------------------------------------------------------- teammates


class NoTeam:
    def get(self, t: float, my_message: TeamMessage | None = None) -> list[TeamReport]:
        return []


@dataclass
class SimTeammate:
    robot_id: int
    flipped: bool = False
    side_confidence: float = 0.98
    is_goalkeeper: bool = False
    ball_sigma: float = 0.2


@dataclass
class SimTeam:
    """Scripted teammates that see the simulated ball (oracles; may be flipped), reporting at `rate_hz`."""

    ball: SimBall
    teammates: list[SimTeammate] = field(
        default_factory=lambda: [SimTeammate(1, is_goalkeeper=True), SimTeammate(3)]
    )
    rate_hz: float = 2.0
    seed: int = 0

    def __post_init__(self):
        self.rng = np.random.default_rng(self.seed)
        self._next = {tm.robot_id: 0.0 for tm in self.teammates}

    def get(self, t: float, my_message: TeamMessage | None = None) -> list[TeamReport]:
        out = []
        for tm in self.teammates:
            if t >= self._next[tm.robot_id]:
                self._next[tm.robot_id] = t + 1.0 / self.rate_hz
                seen = self.ball.position + self.rng.normal(0, tm.ball_sigma, 2)
                xy = -seen if tm.flipped else seen
                out.append(
                    TeamReport(t, tm.robot_id, tuple(xy), tm.ball_sigma, tm.side_confidence, tm.is_goalkeeper)
                )
        return out


class UdpTeam:
    """Live team communication: TeamLink (UDP) + TeamComm (pacing, teammate table) + GameBridge conversion.

    `get(t, my_message)` sends ours when the pacer allows and returns reports from received teammate messages.
    The budget comes from the GameController if a tracker is given, else `budget` is used as a fixed value.
    """

    def __init__(
        self,
        link,
        comm: TeamComm,
        bridge,
        tracker: GameStateTracker | None = None,
        budget: int = 12000,
        robots: int = 3,
    ):
        self.link, self.comm, self.bridge, self.tracker = link, comm, bridge, tracker
        self.budget, self.robots = budget, robots

    def get(self, t: float, my_message: TeamMessage | None = None) -> list[TeamReport]:
        if my_message is not None:
            tr = self.tracker
            budget = tr.message_budget if tr is not None and tr.message_budget is not None else self.budget
            counts = tr.budget_counts if tr is not None else True
            secs = tr.data.secs_remaining if tr is not None and tr.data is not None else 600
            first = tr.data.first_half if tr is not None and tr.data is not None else True
            robots = tr.active_teammates if tr is not None and tr.data is not None else self.robots
            payload = self.comm.outgoing(
                t, my_message, budget, counts, remaining_game_seconds(secs, first), robots
            )
            if payload is not None:
                self.link.send(payload)
        out = []
        for payload, _host in self.link.poll():
            msg = self.comm.incoming(t, payload)
            rep = self.bridge.team_report(t, msg) if msg is not None else None
            if rep is not None:
                out.append(rep)
        return out


def default_world(
    fm: FieldModel,
    kind: str = "walk",
    seed: int = 1,
    duration: float = 30.0,
    start=(-3.0, 1.0, 0.3),
    head_pitch: float = 0.35,
) -> World:
    """Ready-made worlds: "walk" (wandering robot), "tripod" (stationary camera, scanning head), "still"
    (stationary, fixed head)."""
    from td_field_model.scenarios import HeadMotion, ScenarioBuilder

    if kind == "walk":
        return World(ScenarioBuilder(fm, seed=seed).start(start).walk(duration).steps)
    if kind == "tripod":
        head = HeadMotion(yaw_amplitude=0.8, period=6.0, pitch=head_pitch)
        return World(ScenarioBuilder(fm, seed=seed, head=head).start(start).stand(duration).steps)
    if kind == "still":
        head = HeadMotion(yaw_amplitude=0.0, pitch=head_pitch)
        return World(ScenarioBuilder(fm, seed=seed, head=head).start(start).stand(duration).steps)
    raise ValueError(f"unknown world {kind!r} (walk, tripod, still)")
