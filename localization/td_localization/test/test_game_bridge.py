"""GameController → localization bridge (LOC-08).

Unit checks and an end-to-end penalty cycle in simulation.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "td_game" / "test"))
from gc_fixtures import US, control  # noqa: E402
from td_field_model.scenarios import ScenarioBuilder, penalty_reentry_poses  # noqa: E402
from td_field_model.sim import NoiseModel, Simulator  # noqa: E402
from td_game.gc_data import PENALTY_PUSHING, STATE_PLAYING  # noqa: E402
from td_localization.game_bridge import GameBridge  # noqa: E402

from td_field_model import FieldModel  # noqa: E402
from td_game import GameEvent, GameStateTracker, TeamMessage  # noqa: E402
from td_localization import ParticleFilter, SideBelief  # noqa: E402

ME = 2


@pytest.fixture(scope="module")
def fm():
    return FieldModel.builtin("hsl_m_2026")


@pytest.fixture(scope="module")
def lf(fm):
    return fm.likelihood_field()


def make(fm, lf, start_poses=None):
    pf = ParticleFilter(fm, lf, seed=0)
    side = SideBelief(pf)
    return pf, side, GameBridge(side, fm, start_poses)


def test_initial_with_known_and_unknown_start(fm, lf):
    pf, side, br = make(fm, lf, start_poses=[(-3.0, 4.7, -math.pi / 2)])
    br.on_events([GameEvent(0.0, "initial")])
    assert not pf.lost and side.logodds == side.cfg.known_placement_logodds
    pf2, _, br2 = make(fm, lf)
    br2.on_events([GameEvent(0.0, "initial")])
    assert pf2.lost  # own-half search


def test_penalty_pauses_and_reentry_reseeds(fm, lf):
    pf, side, br = make(fm, lf)
    br.on_events([GameEvent(1.0, "penalized", "pushing")])
    assert br.paused
    br.on_events([GameEvent(40.0, "reentry", "pushing")])
    assert not br.paused and side.events[-1].kind == "penalty_reentry"
    cands = np.array(penalty_reentry_poses(fm))
    d = np.min(np.hypot(*(pf.p[:, None, :2] - cands[None, :, :2]).transpose(2, 0, 1)), axis=1)
    assert np.median(d) < 0.3  # particles sit on the two touchline candidates


def test_in_place_penalty_does_not_pause(fm, lf):
    _, _, br = make(fm, lf)
    br.on_events([GameEvent(1.0, "penalized", "motion_in_set", in_place=True)])
    assert not br.paused
    br.on_events([GameEvent(16.0, "unpenalized_in_place", "motion_in_set", True)])
    assert not br.paused


def test_team_report_conversion(fm, lf):
    _, _, br = make(fm, lf)
    m = TeamMessage(3, (0.0, 0.0, 0.0), 0.1, 0.95, (2.0, -1.0), 0.2, 0.2, is_goalkeeper=True)
    rep = br.team_report(10.0, m)
    assert rep.t == pytest.approx(9.8) and rep.ball_xy == (2.0, -1.0) and rep.is_goalkeeper
    assert br.team_report(10.0, TeamMessage(3, (0, 0, 0), 0.1, 0.95, None)) is None
    assert br.team_report(10.0, TeamMessage(3, (0, 0, 0), 0.1, 0.95, (1, 1), ball_age=4.0)) is None


def test_end_to_end_penalty_cycle_from_gamecontroller_packets(fm, lf):
    """Walk → GameController says Pushing → carried off (paused) → placed on the touchline → penalty ends.

    The filter then re-seeds from the rule-defined candidates and converges on the right touchline.
    """
    start = (-3.0, 1.0, 0.3)
    pre = ScenarioBuilder(fm, seed=4).start(start).walk(5.0)
    walk1 = pre.steps
    true_reentry = penalty_reentry_poses(fm)[0]  # left touchline
    post = ScenarioBuilder(fm, seed=4).start(true_reentry).stand(1.0).walk(6.0).steps
    sim = Simulator(fm, noise=NoiseModel(false_positives=2.0), seed=4)
    pf, side, br = make(fm, lf, start_poses=[start])
    tr = GameStateTracker(US, ME)

    br.on_events(tr.update(0.0, control(0, STATE_PLAYING)) + [GameEvent(0.0, "initial")])
    for i, s in enumerate(walk1):
        if i:
            pf.predict(s.odom, s.odom_cov)
        if i % 2 == 0:
            pf.update(sim.render(s.true_pose, s.head_yaw, s.head_pitch))
        side.step(s.t, pf.estimate())

    t = walk1[-1].t
    br.on_events(tr.update(t + 0.1, control(1, STATE_PLAYING, our_penalties={ME: PENALTY_PUSHING})))
    assert br.paused
    # While paused the node feeds nothing (the robot is carried); 45 s later the penalty ends.
    br.on_events(tr.update(t + 45.0, control(2, STATE_PLAYING)))
    assert not br.paused

    errs = []
    for i, s in enumerate(post[1:], 1):
        pf.predict(s.odom, s.odom_cov)
        if i % 2 == 0:
            pf.update(sim.render(s.true_pose, s.head_yaw, s.head_pitch))
        e = pf.estimate()
        side.step(t + 45.0 + s.t, e)
        errs.append(math.hypot(e.pose[0] - s.true_pose[0], e.pose[1] - s.true_pose[1]))
    errs = np.array(errs)
    assert np.all(errs[90:] < 0.3)  # converged on the correct touchline within 3 s and stayed
    assert side.confidence > 0.9 and side.flips == 0
