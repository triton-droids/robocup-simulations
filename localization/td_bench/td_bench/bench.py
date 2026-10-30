"""Bench runner: the real localization stack driven by any mix of real, recorded and simulated inputs.

Stack per tick (the same order the LOC-20 ROS node will use):
1. game events → GameBridge (re-seed on initial/re-entry; pause while penalized). With no game source, the run
   starts with an "initial" event: known start poses if given, else a search of our own half.
2. odometry → ParticleFilter.predict (skipped while paused).
3. camera observations → ParticleFilter.update (skipped while paused).
4. estimate → SideBelief.step.
5. my ball (if a ball source) → SideBelief.on_my_ball.
6. team: send our TeamMessage, receive teammates' reports → SideBelief.on_team_report.
7. record (optional) and log.

Ground truth, when available (a simulated World, a recording with truth, or a known fixed pose for a tripod),
lets
the run be scored with td_localization.eval exactly like the scenario suite.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from td_localization.eval import (
    EstimateTrack,
    GroundTruth,
    evaluate,
    write_estimates,
    write_ground_truth,
    write_report,
)
from td_localization.game_bridge import GameBridge
from td_localization.pf import ParticleFilter, PFConfig
from td_localization.side import SideBelief
from td_localization.suite import status_of, to_field

from td_field_model import FieldModel
from td_game import GameEvent, TeamMessage


@dataclass
class BenchResult:
    est: EstimateTrack
    gt: GroundTruth | None
    events: list[GameEvent]
    side: SideBelief
    pf: ParticleFilter
    frames_without_obs: int = 0
    ticks: int = 0
    log: list[str] = field(default_factory=list)

    def report(self, out_dir, name: str = "bench", notes: str = "") -> Path:
        """Write estimates (+ ground truth and an eval report if truth is known). Returns the main file."""
        out = Path(out_dir)
        out.mkdir(parents=True, exist_ok=True)
        write_estimates(out / f"est_{name}.csv", self.est)
        (out / "bench_log.txt").write_text("\n".join(self.log) + "\n", encoding="utf-8")
        if self.gt is None:
            return out / f"est_{name}.csv"
        write_ground_truth(out / f"gt_{name}.csv", self.gt)
        return write_report(
            [evaluate(self.gt, self.est, name)], out, title="Localization bench run", notes=notes
        )


class Bench:
    def __init__(
        self,
        fm: FieldModel,
        camera,
        odometry,
        head=None,
        game=None,
        team=None,
        ball=None,
        truth=None,
        start_poses=None,
        player: int = 2,
        goalkeeper: bool = False,
        pf_config: PFConfig | None = None,
        seed: int = 0,
        recorder=None,
    ):
        """truth: a sources.World, a record.Recording (with truth), a fixed (x, y, θ), or None."""
        self.fm = fm
        self.camera, self.odometry, self.head, self.game, self.team, self.ball = (
            camera,
            odometry,
            head,
            game,
            team,
            ball,
        )
        self.truth = truth
        self.start_poses = start_poses
        self.player, self.goalkeeper = player, goalkeeper
        self.pf = ParticleFilter(fm, fm.likelihood_field(), pf_config, seed=seed)
        self.side = SideBelief(self.pf)
        self.bridge = GameBridge(self.side, fm, start_poses)
        self.recorder = recorder

    def _truth_at(self, t: float):
        tr = self.truth
        if tr is None:
            return None
        if hasattr(tr, "step"):  # World
            return tuple(tr.step(t).true_pose)
        if hasattr(tr, "observations"):  # Recording
            i = tr.index(t)
            truth = tr.truth
            return None if truth is None or i is None else tuple(truth[i])
        return tuple(tr)  # fixed pose

    def run(self, duration: float, dt: float = 1 / 30, realtime: bool = False) -> BenchResult:
        n = int(round(duration / dt)) + 1
        ts = np.zeros(n)
        est = np.zeros((n, 3))
        gt = np.full((n, 3), np.nan)
        status = np.empty(n, dtype=object)
        conf = np.zeros(n)
        events_all: list[GameEvent] = []
        no_obs = 0
        last_ball = None
        wall0 = time.perf_counter()
        for i in range(n):
            t = i * dt
            if realtime:
                lag = t - (time.perf_counter() - wall0)
                if lag > 0:
                    time.sleep(lag)
            events = self.game.get(t) if self.game is not None else []
            if i == 0 and self.game is None:
                events = [GameEvent(0.0, "initial")]
            self.bridge.on_events(events)
            events_all += events

            d, cov = self.odometry.get(t)
            if i and not self.bridge.paused:
                self.pf.predict(d, cov)
            obs = self.camera.get(t)
            if obs is not None and not self.bridge.paused:
                self.pf.update(obs)
            elif obs is None:
                no_obs += 1
            e = self.pf.estimate()
            self.side.step(t, e)

            if self.ball is not None:
                seen = self.ball.get(t)
                if seen is not None:
                    rel, sig = seen
                    xy = to_field(e.pose, rel)
                    self.side.on_my_ball(t, xy, sig)
                    last_ball = (t, xy, sig)
            if self.team is not None:
                ball_xy, age, sig = None, 0.0, 0.3
                if last_ball is not None and t - last_ball[0] <= 3.0:
                    ball_xy, age, sig = tuple(last_ball[1]), t - last_ball[0], last_ball[2]
                st = status_of(self.pf, e.weight)
                msg = TeamMessage(
                    self.player,
                    e.pose,
                    float(max(0.05, math.sqrt(max(e.cov[0, 0], e.cov[1, 1])))),
                    self.side.confidence,
                    ball_xy,
                    sig,
                    age,
                    self.goalkeeper,
                    False,
                    st == "localized",
                )
                for rep in self.team.get(t, msg):
                    self.side.on_team_report(rep)

            head = self.head.get(t) if self.head is not None else (math.nan, math.nan)
            truth = self._truth_at(t)
            if self.recorder is not None:
                self.recorder.add(t, d, cov, head, obs, events, truth)
            ts[i] = t
            est[i] = e.pose
            status[i] = "lost" if self.bridge.paused else status_of(self.pf, e.weight)
            conf[i] = self.side.confidence
            if truth is not None:
                gt[i] = truth
        has_gt = not np.all(np.isnan(gt))
        gtrack = None
        if has_gt:
            # Game-event names → evaluation event names (td_localization.eval.EVENTS).
            names = {"initial": "initial", "replaced": "initial", "reentry": "penalty_reentry"}
            gtrack = GroundTruth(
                ts.copy(),
                np.nan_to_num(gt, nan=0.0),
                [(e.t, names[e.kind]) for e in events_all if e.kind in names],
            )
        result = BenchResult(
            EstimateTrack(ts, status, est, conf, None),
            gtrack,
            events_all,
            self.side,
            self.pf,
            no_obs,
            n,
            list(self.bridge.log),
        )
        return result
