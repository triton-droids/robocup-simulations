"""Multi-robot simulation with real team communication (LOC-21). numpy only, no ROS.

Every robot runs its own particle filter (LOC-05), side belief (LOC-06), GameBridge (LOC-08) and TeamComm
(LOC-21). Messages are real packed TeamMessage bytes, broadcast over `LossyNetwork` (random loss and fixed
delay). The team's message budget is shared and decremented as the GameController would, and pacing uses it.
This replaces the scripted `suite.Teammate` oracle: teammates' ball reports now come from their own (possibly
wrong) localization.

Simplifications (simulation only):
- All robots see the same ball (`suite.BallTrack`) when it is within `see_range` and inside the camera's
  horizontal field of view around the head yaw. No occlusion by robots.
- No opponents. The game is always Playing (every message counts).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from td_field_model.sim import NoiseModel, Simulator
from td_game.team_msg import remaining_game_seconds

from td_field_model import FieldModel
from td_game import TeamComm, TeamMessage
from td_localization.eval import EstimateTrack, GroundTruth
from td_localization.game_bridge import GameBridge
from td_localization.pf import ParticleFilter
from td_localization.side import SideBelief
from td_localization.suite import BallTrack, my_ball_obs, status_of, to_field


@dataclass
class RobotSpec:
    player: int
    steps: list  # td_field_model.scenarios.Step
    is_goalkeeper: bool = False


class LossyNetwork:
    """Broadcast with independent per-recipient loss and a fixed delay."""

    def __init__(self, loss: float = 0.05, delay: float = 0.05, seed: int = 0):
        self.loss = loss
        self.delay = delay
        self.rng = np.random.default_rng(seed)
        self._queue: list[tuple[float, int, int, bytes]] = []  # (deliver_at, sender, recipient, payload)
        self.sent = 0
        self.lost = 0

    def broadcast(self, t: float, sender: int, recipients: list[int], payload: bytes) -> None:
        self.sent += 1
        for r in recipients:
            if r == sender:
                continue
            if self.rng.random() < self.loss:
                self.lost += 1
                continue
            self._queue.append((t + self.delay, sender, r, payload))

    def deliver(self, t: float, recipient: int) -> list[bytes]:
        due = [q for q in self._queue if q[2] == recipient and q[0] <= t]
        self._queue = [q for q in self._queue if not (q[2] == recipient and q[0] <= t)]
        return [q[3] for q in due]


@dataclass
class RobotRun:
    player: int
    gt: GroundTruth
    est: EstimateTrack
    side: SideBelief
    comm: TeamComm
    flip_times: list[float] = field(default_factory=list)


@dataclass
class TeamRun:
    robots: list[RobotRun]
    budget_start: int
    budget_end: int
    network: LossyNetwork

    def robot(self, player: int) -> RobotRun:
        return next(r for r in self.robots if r.player == player)


def _visible(true_pose, head_yaw: float, ball_xy, see_range: float, half_fov: float) -> bool:
    x, y, th = true_pose
    dx, dy = ball_xy[0] - x, ball_xy[1] - y
    if math.hypot(dx, dy) > see_range:
        return False
    bearing = math.atan2(dy, dx) - th - head_yaw
    return abs(math.atan2(math.sin(bearing), math.cos(bearing))) <= half_fov


def run_team(
    fm: FieldModel,
    lf,
    specs: list[RobotSpec],
    noise: NoiseModel | None = None,
    seed: int = 0,
    ball: BallTrack | None = None,
    loss: float = 0.05,
    delay: float = 0.05,
    budget: int = 12000,
    secs_remaining: float = 600.0,
    first_half: bool = True,
    update_every: int = 2,
    see_range: float = 6.0,
    half_fov: float = math.radians(35),
) -> TeamRun:
    """Run all robots in lockstep over their scenario steps (trimmed to the shortest)."""
    noise = noise or NoiseModel(false_positives=2.0)
    ball = ball or BallTrack(fm, seed=seed, start=(-1.5, 0.5))
    rng = np.random.default_rng(seed + 1000)
    net = LossyNetwork(loss, delay, seed + 2000)
    n = min(len(s.steps) for s in specs)
    players = [s.player for s in specs]
    dt = specs[0].steps[1].t - specs[0].steps[0].t
    budget_start = budget

    state = []
    for k, s in enumerate(specs):
        pf = ParticleFilter(fm, lf, seed=seed + k)
        side = SideBelief(pf)
        state.append(
            dict(
                spec=s,
                pf=pf,
                side=side,
                bridge=GameBridge(side, fm),
                comm=TeamComm(s.player),
                sim=Simulator(fm, noise=noise, seed=seed + 10 * k),
                ball_seen=None,  # (t, field xy, sigma)
                gt=np.zeros((n, 3)),
                est=np.zeros((n, 3)),
                status=np.empty(n, dtype=object),
                conf=np.zeros(n),
                events=[],
                flips=[],
            )
        )

    ts = np.zeros(n)
    for i in range(n):
        b = ball.step(dt)
        for r in state:
            s = r["spec"].steps[i]
            pf, side = r["pf"], r["side"]
            if i:
                pf.predict(s.odom, s.odom_cov)
            if s.event:
                r["events"].append((s.t, s.event))
            if s.event == "initial":
                side.on_initial(s.t, s.known_poses)
            elif s.event == "penalty_reentry":
                side.on_penalty_reentry(s.t, s.known_poses)
            if i % update_every == 0:
                pf.update(r["sim"].render(s.true_pose, s.head_yaw, s.head_pitch, t=s.t))
            e = pf.estimate()
            if side.step(s.t, e):
                r["flips"].append(s.t)

            if _visible(s.true_pose, s.head_yaw, b, see_range, half_fov):
                rel, sig = my_ball_obs(s.true_pose, b, rng)
                field_xy = to_field(e.pose, rel)
                side.on_my_ball(s.t, field_xy, sig)
                r["ball_seen"] = (s.t, field_xy, sig)

            # Incoming team messages → teammate table → side-belief evidence.
            for payload in net.deliver(s.t, r["spec"].player):
                msg = r["comm"].incoming(s.t, payload)
                rep = r["bridge"].team_report(s.t, msg) if msg is not None else None
                if rep is not None:
                    side.on_team_report(rep)

            # Outgoing: paced against the shared budget.
            seen = r["ball_seen"]
            ball_xy, ball_age, ball_sigma = None, 0.0, 0.3
            if seen is not None and s.t - seen[0] <= 3.0:
                ball_xy, ball_age, ball_sigma = tuple(seen[1]), s.t - seen[0], seen[2]
            status = status_of(pf, e.weight)
            msg = TeamMessage(
                player_number=r["spec"].player,
                pose=e.pose,
                pose_sigma=float(max(0.05, math.sqrt(max(e.cov[0, 0], e.cov[1, 1])))),
                side_confidence=side.confidence,
                ball_xy=ball_xy,
                ball_sigma=ball_sigma,
                ball_age=ball_age,
                is_goalkeeper=r["spec"].is_goalkeeper,
                localized=status == "localized",
            )
            remaining = remaining_game_seconds(int(secs_remaining - s.t), first_half)
            payload = r["comm"].outgoing(s.t, msg, budget, True, remaining, len(specs))
            if payload is not None:
                budget -= 1
                net.broadcast(s.t, r["spec"].player, players, payload)

            r["gt"][i] = s.true_pose
            r["est"][i] = e.pose
            r["status"][i] = status
            r["conf"][i] = side.confidence
        ts[i] = specs[0].steps[i].t

    runs = []
    for r in state:
        gt = GroundTruth(ts.copy(), r["gt"], r["events"])
        est = EstimateTrack(ts.copy(), r["status"], r["est"], r["conf"], None)
        runs.append(RobotRun(r["spec"].player, gt, est, r["side"], r["comm"], r["flips"]))
    return TeamRun(runs, budget_start, budget, net)
