"""Shared scenario runner for the particle-filter tests (stand-in until LOC-07's evaluation tool exists)."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from td_field_model import mirror_pose


@dataclass
class RunResult:
    t: np.ndarray
    truth: np.ndarray  # (T, 3)
    est: np.ndarray  # (T, 3)
    weight: np.ndarray
    update_ms: np.ndarray

    def pos_err(self) -> np.ndarray:
        return np.hypot(*(self.est[:, :2] - self.truth[:, :2]).T)

    def head_err(self) -> np.ndarray:
        return np.abs(
            np.arctan2(np.sin(self.est[:, 2] - self.truth[:, 2]), np.cos(self.est[:, 2] - self.truth[:, 2]))
        )

    def mirror_closer(self, heading_scale: float = 2.0) -> np.ndarray:
        """True where the estimate is nearer the mirror of the truth than the truth itself: a side flip.

        Distance is in (x, y, θ) with heading weighted by heading_scale m/rad, so near-center cases count.
        """
        mir = np.array([mirror_pose(*p) for p in self.truth])

        def dist(ref):
            dth = np.arctan2(np.sin(self.est[:, 2] - ref[:, 2]), np.cos(self.est[:, 2] - ref[:, 2]))
            return np.hypot(np.hypot(*(self.est[:, :2] - ref[:, :2]).T), heading_scale * dth)

        return dist(mir) < dist(self.truth)

    def after(self, t0: float) -> RunResult:
        k = self.t >= t0
        return RunResult(self.t[k], self.truth[k], self.est[k], self.weight[k], self.update_ms[k])


def run(steps, sim, pf, update_every: int = 2, on_event=None) -> RunResult:
    """Drive the filter through scenario steps.

    Predicts every step; renders and updates every `update_every` steps.

    on_event(step, pf) is called for steps carrying an event (after predict), so a test can play the
    GameController (for example re-seed on penalty re-entry).
    """
    ts, truth, est, weight, ms = [], [], [], [], []
    for i, s in enumerate(steps):
        if i:
            pf.predict(s.odom, s.odom_cov)
        if s.event and on_event:
            on_event(s, pf)
        if i % update_every == 0:
            obs = sim.render(s.true_pose, s.head_yaw, s.head_pitch, t=s.t)
            t0 = time.perf_counter()
            pf.update(obs)
            ms.append(1000 * (time.perf_counter() - t0))
        e = pf.estimate()
        ts.append(s.t)
        truth.append(s.true_pose)
        est.append(e.pose)
        weight.append(e.weight)
    pad = np.full(len(ts), np.nan)
    pad[: len(ms)] = ms
    return RunResult(np.array(ts), np.array(truth), np.array(est), np.array(weight), pad)


def p(x: np.ndarray, q: float) -> float:
    return float(np.percentile(x, q))


def deg(x: float) -> float:
    return math.degrees(x)
