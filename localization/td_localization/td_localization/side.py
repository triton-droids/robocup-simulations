"""Side belief: which half of the symmetric field the particle filter is on (LOC-06). Pure numpy, no ROS.

The field is point-symmetric, so line and landmark matching score a pose and its mirror identically
(docs/roadmap.md §3.5). The particle filter keeps one side through continuity. This module
decides whether that side is right, using evidence vision cannot give.

State: `logodds` = log P(filter's half is correct) / P(mirrored half is correct).
`confidence` = sigmoid(logodds) is what behavior reads (for example, no shots below 0.9).

Evidence, most reliable first:
1. Known placements from the rules:
   - Initial-state placement in our own half.
   - Penalty re-entry on our own-half touchline.
   These re-seed the filter (`pf.init_poses` / `pf.init_uniform`) and set logodds high.
2. Teammates' ball reports: my ball in the field frame vs theirs.
   - A match supports my side; a match with x and y negated contradicts it.
   - Log-likelihood ratio = (d_mirror² - d_direct²) / (2σ²), clipped. A ball near the center mark gives ≈ 0.
   - Gated: if neither the direct nor the mirrored ball matches within 3σ, a pose is wrong in some other way,
     and that's no evidence about the side.
   - Weighted by the teammate's own confidence. The goalkeeper (rarely far from its goal) counts double.
3. Weak cue: opponents stand in their own half (x > 0) during Set.

Uncertainty:
- While the filter is lost or degraded, logodds decays toward 0.
- When the estimate jumps (relocalization after a kidnap), logodds is capped near 1 (P ≈ 0.73). Behavior then
  avoids shooting, and fresh evidence decides quickly.

Decision:
- When logodds stays below `flip_threshold` for `flip_hold` seconds, the filter is mirrored (`pf.mirror()`).
- The new side starts at moderate confidence, and no further flip is allowed for `flip_refractory` seconds.
- Every flip is logged in `events`.

My ball reports are only accepted while my own pose is trustworthy: not lost, not degraded, and a
confident cluster.

Known limit: with no teammates and no game event, a kidnap across the field can leave the filter on the
mirrored side. Confidence drops after the jump, but only external evidence can fix the side.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from td_localization.pf import Estimate, ParticleFilter


@dataclass(frozen=True)
class SideConfig:
    known_placement_logodds: float = 6.0  # P ≈ 0.998 after a rule-defined placement
    max_logodds: float = 8.0
    flip_threshold: float = -4.0  # P(current side) ≈ 0.018
    flip_hold: float = 1.0  # s the evidence must stay below the threshold
    degraded_tau: float = 6.0  # s, logodds decay time constant while the filter is lost or degraded
    report_llr_clip: float = 2.0  # max evidence from a single teammate report
    pose_sigma: float = 0.3  # m, added for both robots' pose uncertainty in the ball comparison
    ball_sigma_floor: float = 0.15  # m
    # Max time between my sighting and the teammate's that may be compared. Long gaps are still informative
    # when the ball is far from the center (direct and mirrored positions are 2|ball| apart); the widened σ
    # (ball_speed × gap) makes the evidence fade by itself near the center or for long gaps.
    ball_max_age: float = 3.0  # s
    ball_speed: float = 1.0  # m/s, assumed ball motion between the two sightings (widens the comparison)
    ball_history_s: float = 4.0  # s of my own sightings kept for matching against delayed reports
    # Consensus before a flip: evidence against my side must come from the goalkeeper or from at least this
    # many different teammates within flip_sources_window. One confidently-wrong teammate (for example a
    # robot that was itself just kidnapped) must not flip a healthy robot.
    # TEAM DECISION 2026-10-27 (docs/implementation_plan.md §9): keep at 2. Ask before changing.
    min_flip_sources: int = 2
    flip_sources_window: float = 15.0  # s
    min_pose_weight: float = 0.8  # my best-cluster weight needed to trust my ball in the field frame
    goalkeeper_weight: float = 2.0
    min_teammate_confidence: float = 0.6  # reports from less sure teammates are ignored
    opponent_margin: float = 0.5  # m past the halfway line before an opponent counts as evidence
    opponent_llr: float = 0.5  # evidence per opponent seen at Set
    avoid_shooting_below: float = 0.9
    jump_distance: float = 1.5  # m of position change between consecutive estimates that counts as a jump
    jump_heading: float = math.pi / 2  # rad; a heading change this large also counts (falls turn ≤ ~60°)
    after_jump_logodds: float = 1.0  # cap after a jump: P ≈ 0.73, so no shots until evidence returns
    match_gate_sigma: float = (
        3.0  # a report counts only if the direct or mirrored match is within this many σ
    )
    post_flip_logodds: float = 3.0  # confidence right after a flip (the new side still has to prove itself)
    flip_refractory: float = 3.0  # s with no further flip after one
    post_lost_grace: float = (
        2.0  # s after the filter leaves lost mode when jumps don't count (estimate settling)
    )


@dataclass(frozen=True)
class TeamReport:
    """One teammate message (it fits easily in the 512 B budget; see LOC-21)."""

    t: float
    robot_id: int
    ball_xy: tuple[float, float]  # in the field frame as the teammate believes it
    ball_sigma: float  # m
    side_confidence: float  # teammate's own SideBelief.confidence
    is_goalkeeper: bool = False


@dataclass
class SideEvent:
    t: float
    kind: str  # "initial", "penalty_reentry", "flip", "report", "opponents"
    detail: str = ""
    logodds: float = 0.0


class SideBelief:
    def __init__(self, pf: ParticleFilter, config: SideConfig | None = None):
        self.pf = pf
        self.cfg = config or SideConfig()
        self.logodds = 0.0
        self.flips = 0
        self.events: list[SideEvent] = []
        self._t_last: float | None = None
        self._below_since: float | None = None
        self._pose_ok = False
        self._my_ball: tuple[float, np.ndarray, float] | None = None  # latest (t, xy, sigma)
        self._ball_hist: list[tuple[float, np.ndarray, float]] = []  # recent sightings, oldest first
        self._pending: list[TeamReport] = []  # teammate reports waiting for a sighting of mine to match
        self._against: dict[
            object, tuple[float, bool]
        ] = {}  # source → (time, is_goalkeeper) of evidence against
        self._last_pose: tuple[float, float, float] | None = None
        self._last_flip: float | None = None
        self._grace_until: float | None = None

    # ---------------------------------------------------------------- outputs

    @property
    def confidence(self) -> float:
        """P(the filter's half is correct)."""
        return 1.0 / (1.0 + math.exp(-self.logodds))

    @property
    def avoid_shooting(self) -> bool:
        return self.confidence < self.cfg.avoid_shooting_below

    # ---------------------------------------------------------------- game events

    def on_initial(self, t: float, known_poses=None) -> None:
        """Initial state: robot placed by a handler in our own half.

        Pass the known starting spots, or None to search the whole own half.
        """
        if known_poses:
            self.pf.init_poses(known_poses)
        else:
            self.pf.init_uniform(own_half_only=True)
        self._set(t, self.cfg.known_placement_logodds, "initial", f"{len(known_poses or [])} known poses")

    def on_penalty_reentry(self, t: float, candidates) -> None:
        """Penalty served: placed on our own-half touchline level with the penalty mark.

        Both touchline candidates are given.
        """
        self.pf.init_poses(candidates)
        self._set(t, self.cfg.known_placement_logodds, "penalty_reentry", f"{len(candidates)} candidates")

    def _set(self, t: float, logodds: float, kind: str, detail: str) -> None:
        self.logodds = logodds
        self._below_since = None
        self._my_ball = None
        self._ball_hist = []
        self._pending = []
        self._against = {}
        self._last_pose = None
        # A re-seed from several known candidates (both touchlines, several starting spots) makes the estimate
        # switch between them while vision decides. That switch is expected, not a kidnap, so it gets the same
        # grace period as leaving lost mode.
        self._grace_until = t + self.cfg.post_lost_grace
        self.events.append(SideEvent(t, kind, detail, self.logodds))

    # ---------------------------------------------------------------- evidence

    def on_my_ball(self, t: float, ball_xy_field, sigma: float) -> None:
        """My current ball estimate in the field frame (from my pose). Ignored while my pose is unreliable."""
        if not self._pose_ok:
            return
        self._my_ball = (t, np.asarray(ball_xy_field, float), float(sigma))
        self._ball_hist.append(self._my_ball)
        cutoff = t - self.cfg.ball_history_s
        while self._ball_hist and self._ball_hist[0][0] < cutoff:
            self._ball_hist.pop(0)
        # A teammate's report that arrived before I saw the ball can be matched now. Each report is used once;
        # reports too old to ever match are dropped.
        still = []
        for rep in self._pending:
            if self._try_apply(rep) is None and t - rep.t <= self.cfg.ball_max_age:
                still.append(rep)
        self._pending = still

    def report_llr(self, my_xy: np.ndarray, my_sigma: float, rep: TeamReport) -> float:
        """Evidence from one teammate report, before weighting.

        Log-likelihood ratio of current side vs mirrored.
        """
        cfg = self.cfg
        theirs = np.asarray(rep.ball_xy, float)
        var = (
            max(my_sigma, cfg.ball_sigma_floor) ** 2
            + max(rep.ball_sigma, cfg.ball_sigma_floor) ** 2
            + 2 * cfg.pose_sigma**2
        )
        d_direct2 = float(np.sum((my_xy - theirs) ** 2))
        d_mirror2 = float(np.sum((my_xy + theirs) ** 2))
        if min(d_direct2, d_mirror2) > cfg.match_gate_sigma**2 * var:
            # Neither the direct nor the mirrored ball matches: one pose is wrong in some other way.
            # That says nothing about the side.
            return 0.0
        llr = (d_mirror2 - d_direct2) / (2 * var)
        return float(np.clip(llr, -cfg.report_llr_clip, cfg.report_llr_clip))

    def on_team_report(self, rep: TeamReport) -> float:
        """Apply one teammate's ball report. Returns the evidence applied (0 if unusable or not yet matched).

        A report with no matching sighting of mine yet is kept and matched when I next see the ball
        (on_my_ball), within ball_max_age of the teammate's sighting.
        """
        if rep.side_confidence < self.cfg.min_teammate_confidence:
            return 0.0
        llr = self._try_apply(rep)
        if llr is None:
            self._pending.append(rep)
            return 0.0
        return llr

    def _try_apply(self, rep: TeamReport) -> float | None:
        """Match a report with my sighting nearest in time and apply its evidence.

        None if I have no sighting within ball_max_age of theirs.
        """
        cfg = self.cfg
        if not self._ball_hist:
            return None
        # Teammates report at ~2 Hz with network delay, and each robot sees the ball only part of the time, so
        # exact coincidences are rare. Widen my uncertainty by how far the ball could have moved in between.
        t_mine, my_xy, my_sigma = min(self._ball_hist, key=lambda b: abs(b[0] - rep.t))
        gap = abs(rep.t - t_mine)
        if gap > cfg.ball_max_age:
            return None
        my_sigma = math.hypot(my_sigma, cfg.ball_speed * gap)
        # Teammate reliability: 0 at min_teammate_confidence, 1 at certainty, doubled for the goalkeeper.
        weight = (rep.side_confidence - cfg.min_teammate_confidence) / (1.0 - cfg.min_teammate_confidence)
        if rep.is_goalkeeper:
            weight *= cfg.goalkeeper_weight
        llr = weight * self.report_llr(my_xy, my_sigma, rep)
        self._add(llr)
        if llr < 0:
            self._against[rep.robot_id] = (rep.t, rep.is_goalkeeper)
        if llr:
            self.events.append(
                SideEvent(rep.t, "report", f"robot {rep.robot_id} llr {llr:+.2f}", self.logodds)
            )
        return llr

    def on_opponents_at_set(self, t: float, opponents_xy_field) -> float:
        """Weak cue: during Set, opponents stand in their own half (x > 0 in our field frame)."""
        if not self._pose_ok:
            return 0.0
        cfg = self.cfg
        xs = np.atleast_2d(np.asarray(opponents_xy_field, float))[:, 0] if len(opponents_xy_field) else []
        llr = cfg.opponent_llr * (
            int(np.sum(xs > cfg.opponent_margin)) - int(np.sum(xs < -cfg.opponent_margin))
        )
        self._add(llr)
        if llr < 0:
            self._against["opponents"] = (t, False)
        if llr:
            self.events.append(SideEvent(t, "opponents", f"llr {llr:+.2f}", self.logodds))
        return llr

    def _consensus(self, t: float) -> bool:
        """Is recent evidence against my side corroborated (goalkeeper, or enough distinct sources)?"""
        recent = [gk for (ts, gk) in self._against.values() if t - ts <= self.cfg.flip_sources_window]
        return any(recent) or len(recent) >= self.cfg.min_flip_sources

    def _add(self, llr: float) -> None:
        m = self.cfg.max_logodds
        self.logodds = float(np.clip(self.logodds + llr, -m, m))

    # ---------------------------------------------------------------- per-cycle step

    def step(self, t: float, estimate: Estimate) -> bool:
        """Call once per cycle after pf.estimate(). Decays uncertainty and flips the filter when warranted.

        Returns True if a flip happened.
        """
        cfg = self.cfg
        dt = 0.0 if self._t_last is None else max(0.0, t - self._t_last)
        self._t_last = t
        unreliable = self.pf.lost or self.pf.degraded
        self._pose_ok = not unreliable and estimate.weight >= cfg.min_pose_weight
        if unreliable and dt:
            self.logodds *= math.exp(-dt / cfg.degraded_tau)
        if self.pf.lost:
            self._grace_until = t + cfg.post_lost_grace
        in_grace = self._grace_until is not None and t < self._grace_until
        if self._last_pose is not None and not self.pf.lost and not in_grace:
            # Position, or a large heading change. A fall turns the robot but can't put it on the other half,
            # so moderate heading changes alone must not cost side confidence.
            x0, y0, th0 = self._last_pose
            jump = math.hypot(estimate.pose[0] - x0, estimate.pose[1] - y0)
            dth = abs(math.atan2(math.sin(estimate.pose[2] - th0), math.cos(estimate.pose[2] - th0)))
            if (jump > cfg.jump_distance or dth > cfg.jump_heading) and self.logodds > cfg.after_jump_logodds:
                # The estimate relocated (kidnap, recovery): continuity no longer vouches for the side.
                self.logodds = cfg.after_jump_logodds
                self.events.append(
                    SideEvent(
                        t,
                        "jump",
                        f"estimate moved {jump:.1f} m, turned {math.degrees(dth):.0f}°",
                        self.logodds,
                    )
                )
        # While lost (global search), the estimate hops between hypotheses by design. Start comparing afresh
        # once it settles.
        self._last_pose = None if self.pf.lost else estimate.pose
        refractory = self._last_flip is not None and t - self._last_flip < cfg.flip_refractory
        if self.logodds < cfg.flip_threshold and not refractory and self._consensus(t):
            if self._below_since is None:
                self._below_since = t
            elif t - self._below_since >= cfg.flip_hold:
                self._flip(t)
                return True
        else:
            self._below_since = None
        return False

    def _flip(self, t: float) -> None:
        self.pf.mirror()
        self.logodds = min(-self.logodds, self.cfg.post_flip_logodds)
        self.flips += 1
        self._last_flip = t
        self._below_since = None
        self._against = {}  # that evidence was about the old side
        if self._my_ball is not None:
            tb, xy, s = self._my_ball
            self._my_ball = (tb, -xy, s)
        # My past sightings were mapped through the old (mirrored) pose.
        self._ball_hist = [(tb, -xy, s) for tb, xy, s in self._ball_hist]
        if self._last_pose is not None:
            # Our own flip is not a jump.
            x, y, th = self._last_pose
            self._last_pose = (-x, -y, math.atan2(math.sin(th + math.pi), math.cos(th + math.pi)))
        self.events.append(SideEvent(t, "flip", "mirrored the particle filter", self.logodds))
