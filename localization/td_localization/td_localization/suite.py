"""Simulated scenario suite for localization (LOC-07). numpy only, no ROS.

Runs the particle filter (LOC-05) and side belief (LOC-06) through td_field_model scenarios with a moving ball
and reporting teammates. Records ground truth and estimates in the eval.py CSV formats, and writes one
comparison report. `python -m td_localization.eval suite --out DIR` is the entry point.

Estimate status mapping (until the LocalizationState message exists, LOC-02):
- filter lost → "initializing"
- degraded, drift detected, or best cluster < 0.8 → "uncertain"
- otherwise → "localized"

`published_s` = stamp + measured compute time of that cycle (update + estimate + side step), so the
latency metric reflects this machine's speed.

This is simulation evidence only. The simulator's noise model is a guess until field bags exist (LOC-14).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from td_field_model.scenarios import ScenarioBuilder, compose, penalty_reentry_poses, sideline_start_poses
from td_field_model.sim import NoiseModel, Simulator

from td_field_model import FieldModel, mirror_pose
from td_localization.eval import (
    EstimateTrack,
    GroundTruth,
    evaluate,
    write_estimates,
    write_ground_truth,
    write_report,
)
from td_localization.pf import ParticleFilter
from td_localization.side import SideBelief, TeamReport

NOISE = {
    "default": NoiseModel(false_positives=2.0),
    "harsh": NoiseModel(
        pitch_bias=math.radians(1.0), false_positives=5.0, point_dropout=0.2, crossing_confusion=0.1
    ),
}


class BallTrack:
    """A ball wandering the field: smooth random velocity, bounced back inside the lines."""

    def __init__(self, fm: FieldModel, seed: int = 0, start=(2.0, 1.5), speed: float = 0.6):
        self.rng = np.random.default_rng(seed)
        self.a, self.b = fm.dims.length / 2 - 0.3, fm.dims.width / 2 - 0.3
        self.xy = np.array(start, float)
        self.v = self.rng.normal(0, speed, 2)
        self.speed = speed

    def step(self, dt: float) -> np.ndarray:
        self.v += -0.3 * self.v * dt + self.speed * math.sqrt(dt) * self.rng.normal(0, 1, 2)
        self.xy += self.v * dt
        for i, lim in ((0, self.a), (1, self.b)):
            if abs(self.xy[i]) > lim:
                self.xy[i] = math.copysign(lim, self.xy[i])
                self.v[i] *= -1
        return self.xy.copy()


@dataclass
class Teammate:
    robot_id: int
    flipped: bool = False  # this teammate itself believes the mirrored side
    side_confidence: float = 0.98
    is_goalkeeper: bool = False
    ball_sigma: float = 0.2
    phase: float = 0.0  # s, offset of its 2 Hz report schedule


def my_ball_obs(true_pose, ball_xy, rng, sigma0: float = 0.1, sigma_per_m: float = 0.05):
    """My robot-frame ball observation with range-dependent noise."""
    x, y, th = true_pose
    dx, dy = ball_xy[0] - x, ball_xy[1] - y
    c, s = math.cos(th), math.sin(th)
    rel = np.array([c * dx + s * dy, -s * dx + c * dy])
    sigma = sigma0 + sigma_per_m * float(np.hypot(*rel))
    return rel + rng.normal(0, sigma, 2), sigma


def to_field(pose, rel) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.array([x + c * rel[0] - s * rel[1], y + s * rel[0] + c * rel[1]])


@dataclass
class SimRun:
    gt: GroundTruth
    est: EstimateTrack
    side: SideBelief
    flip_times: list[float] = field(default_factory=list)
    update_ms: list[float] = field(default_factory=list)


def status_of(pf: ParticleFilter, weight: float, min_weight: float = 0.8) -> str:
    if pf.lost:
        return "initializing"
    if pf.degraded or pf.drift or weight < min_weight:
        return "uncertain"
    return "localized"


def simulate(
    steps,
    sim: Simulator,
    pf: ParticleFilter,
    side: SideBelief,
    ball: BallTrack | None = None,
    teammates: list[Teammate] = (),
    seed: int = 0,
    update_every: int = 2,
    report_period: float = 0.5,
) -> SimRun:
    """Drive filter + side belief through scenario steps.

    Game events "initial" and "penalty_reentry" go to the side belief with their known poses. Kidnaps and
    falls are not announced; they are only recorded in the ground truth for convergence scoring.
    """
    rng = np.random.default_rng(seed)
    n = len(steps)
    gt_pose, est_pose = np.zeros((n, 3)), np.zeros((n, 3))
    ts, status, conf, pub = np.zeros(n), np.empty(n, dtype=object), np.zeros(n), np.zeros(n)
    events, flips, upd_ms = [], [], []
    next_report = {tm.robot_id: tm.phase for tm in teammates}
    dt = steps[1].t - steps[0].t if n > 1 else 1 / 30
    for i, s in enumerate(steps):
        if i:
            pf.predict(s.odom, s.odom_cov)
        if s.event:
            events.append((s.t, s.event))
        if s.event == "initial":
            side.on_initial(s.t, s.known_poses)
        elif s.event == "penalty_reentry":
            side.on_penalty_reentry(s.t, s.known_poses)
        obs = sim.render(s.true_pose, s.head_yaw, s.head_pitch, t=s.t) if i % update_every == 0 else None
        t0 = time.perf_counter()
        if obs is not None:
            pf.update(obs)
        e = pf.estimate()
        if side.step(s.t, e):
            flips.append(s.t)
        compute = time.perf_counter() - t0
        if obs is not None:
            upd_ms.append(1000 * compute)
        if ball is not None:
            b = ball.step(dt)
            rel, sig = my_ball_obs(s.true_pose, b, rng)
            side.on_my_ball(s.t, to_field(e.pose, rel), sig)
            for tm in teammates:
                if s.t >= next_report[tm.robot_id]:
                    next_report[tm.robot_id] += report_period
                    seen = b + rng.normal(0, tm.ball_sigma, 2)
                    xy = -seen if tm.flipped else seen
                    side.on_team_report(
                        TeamReport(
                            s.t, tm.robot_id, tuple(xy), tm.ball_sigma, tm.side_confidence, tm.is_goalkeeper
                        )
                    )
        ts[i] = s.t
        gt_pose[i] = s.true_pose
        est_pose[i] = e.pose
        status[i] = status_of(pf, e.weight)
        conf[i] = side.confidence
        pub[i] = s.t + compute
    gt = GroundTruth(ts, gt_pose, events)
    est = EstimateTrack(ts.copy(), status, est_pose, conf, pub)
    return SimRun(gt, est, side, flips, upd_ms)


# ---------------------------------------------------------------- standard scenarios

TEAM = (Teammate(2), Teammate(3, phase=0.25), Teammate(1, is_goalkeeper=True, phase=0.1))


def kidnap_same_half(sb: ScenarioBuilder, move=(1.5, 0.0, 0.6)) -> ScenarioBuilder:
    """Kidnap relative to the current pose (a push or short carry).

    Continuity still points to the true side.
    """
    return sb.kidnap(compose(sb.pose, move))


def kidnap_to_mirror(sb: ScenarioBuilder, offset=(0.5, 0.0, 0.2)) -> ScenarioBuilder:
    """Adversarial cross-field kidnap: carried to near the mirror image of the current pose.

    The new pose's own mirror is then almost exactly the old pose, so continuity confidently picks the wrong
    half. Only external evidence (teammates, game events) can correct it.
    """
    return sb.kidnap(compose(mirror_pose(*sb.pose), offset))


def scenarios(fm: FieldModel, seed: int) -> dict[str, dict]:
    """name → {steps, team}. Each mirrors a case from docs/implementation_plan.md LOC-05/06."""
    start = (-3.0, 1.0, 0.3)
    side_start = sideline_start_poses(fm, 3)[1]
    cands = penalty_reentry_poses(fm)

    def sb():
        return ScenarioBuilder(fm, seed=seed)

    return {
        "tracking": {"steps": sb().start(start).walk(40.0).steps, "team": ()},
        "global_sideline": {
            "steps": sb().start(side_start, known_poses=[]).stand(8.0).walk(12.0).steps,
            "team": (),
        },
        "penalty_reentry": {
            "steps": sb().start(cands[1], known_poses=cands).stand(3.0).walk(15.0).steps,
            "team": (),
        },
        "falls": {
            "steps": sb().start(start).walk(15.0).fall().walk(15.0).fall().walk(10.0).steps,
            "team": (),
        },
        "kidnap_same_half": {
            "steps": kidnap_same_half(sb().start((-2.0, -1.0, 0.0)).walk(10.0)).walk(15.0).steps,
            "team": (),
        },
        "kidnap_cross_team": {
            "steps": kidnap_to_mirror(sb().start((-2.0, -1.0, 0.0)).walk(10.0)).walk(20.0).steps,
            "team": TEAM,
        },
        "kidnap_cross_alone": {
            "steps": kidnap_to_mirror(sb().start((-2.0, -1.0, 0.0)).walk(10.0)).walk(20.0).steps,
            "team": (),
        },
        # Three real robots with team communication (LOC-21): goalkeeper, a field player, and player 3 who
        # is kidnapped to its mirror at t = 10 s. Reported as two runs: the kidnapped robot and a healthy
        # teammate.
        "team_kidnap_cross": {"team_specs": team_kidnap_specs(fm, seed)},
    }


def team_kidnap_specs(fm: FieldModel, seed: int, duration: float = 30.0):
    from td_field_model.scenarios import HeadMotion

    from td_localization.team_sim import RobotSpec

    a = fm.dims.length / 2
    gk_head = HeadMotion(yaw_amplitude=1.0, period=5.0, pitch=0.3)
    gk = ScenarioBuilder(fm, seed=seed, head=gk_head).start((-a + 0.6, 0.0, 0.0)).stand(duration)
    p2 = ScenarioBuilder(fm, seed=seed + 1).start((-3.0, -1.5, 0.2)).walk(duration)
    p3 = kidnap_to_mirror(ScenarioBuilder(fm, seed=seed + 2).start((-2.0, 1.0, 0.0)).walk(10.0))
    p3.walk(duration - 10.0)
    return [RobotSpec(1, gk.steps, True), RobotSpec(2, p2.steps), RobotSpec(3, p3.steps)]


def run_suite(
    out_dir, field_name: str = "hsl_m_2026", noise: str = "default", seed: int = 1, only=None
) -> Path:
    """Run every standard scenario, write gt/estimate CSVs per scenario, and one comparison report."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    fm = FieldModel.builtin(field_name)
    lf = fm.likelihood_field()
    results = []
    for name, sc in scenarios(fm, seed).items():
        if only and name not in only:
            continue
        if "team_specs" in sc:
            from td_localization.team_sim import run_team

            tr = run_team(fm, lf, sc["team_specs"], noise=NOISE[noise], seed=seed)
            for player, label in ((3, "kidnapped_p3"), (2, "teammate_p2")):
                r = tr.robot(player)
                run_name = f"{name}_{label}"
                write_ground_truth(out / f"gt_{run_name}.csv", r.gt)
                write_estimates(out / f"est_{run_name}.csv", r.est)
                results.append(evaluate(r.gt, r.est, run_name))
            continue
        pf = ParticleFilter(fm, lf, seed=seed)
        side = SideBelief(pf)
        run = simulate(
            sc["steps"], Simulator(fm, noise=NOISE[noise], seed=seed), pf, side,
            BallTrack(fm, seed=seed), list(sc["team"]), seed=seed,
        )  # fmt: skip
        write_ground_truth(out / f"gt_{name}.csv", run.gt)
        write_estimates(out / f"est_{name}.csv", run.est)
        results.append(evaluate(run.gt, run.est, name))
    notes = (
        f"Simulated suite: field `{field_name}`, noise `{noise}`, seed {seed}. Simulation evidence only: the "
        "noise and detector model are guesses until real field bags exist (LOC-14). "
        "`global_sideline` starts with no known pose (search of the own half). "
        "`kidnap_cross_alone` is expected to fail the side checks: without teammates nothing can tell the "
        "halves apart after a cross-field kidnap (the side confidence should drop instead). "
        "`team_kidnap_cross_*` runs three robots with real team messages over a lossy network (LOC-21); the "
        "healthy teammate must never be misled. Latency is this machine's compute time per cycle "
        "(not recorded for team runs)."
    )
    return write_report(results, out, title="Localization scenario suite", notes=notes)
