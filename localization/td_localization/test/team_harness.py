"""Side-belief test harness (LOC-06). The ball/teammate simulation lives in td_localization.suite."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from helpers import RunResult
from td_localization.suite import BallTrack, Teammate, simulate

__all__ = ["BallTrack", "SideRun", "Teammate", "run_with_side"]


@dataclass
class SideRun(RunResult):
    confidence: np.ndarray = None
    flip_times: list = None


def run_with_side(steps, sim, pf, side, ball, teammates, seed=0, update_every=2) -> SideRun:
    """Drive filter + side belief through a scenario with a moving ball and reporting teammates."""
    run = simulate(steps, sim, pf, side, ball, list(teammates), seed=seed, update_every=update_every)
    n = len(run.gt.t)
    return SideRun(
        run.gt.t,
        run.gt.pose,
        run.est.pose,
        np.full(n, np.nan),
        np.full(n, np.nan),
        confidence=run.est.side_confidence,
        flip_times=run.flip_times,
    )
