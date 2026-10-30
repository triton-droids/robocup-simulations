"""Team communication in a multi-robot simulation (LOC-21), plus the side-belief rules it relies on."""

import numpy as np
import pytest
from td_field_model.scenarios import HeadMotion, ScenarioBuilder
from td_localization.eval import evaluate
from td_localization.suite import kidnap_to_mirror
from td_localization.team_sim import LossyNetwork, RobotSpec, run_team

from td_field_model import FieldModel
from td_localization import Estimate, ParticleFilter, SideBelief, TeamReport

DUR = 30.0


@pytest.fixture(scope="module")
def fm():
    return FieldModel.builtin("hsl_m_2026")


@pytest.fixture(scope="module")
def lf(fm):
    return fm.likelihood_field()


# ---------------------------------------------------------------- network


def test_lossy_network_delivers_after_delay_and_not_to_sender():
    net = LossyNetwork(loss=0.0, delay=0.1)
    net.broadcast(0.0, 1, [1, 2, 3], b"x")
    assert net.deliver(0.05, 2) == []
    assert net.deliver(0.1, 2) == [b"x"] and net.deliver(0.1, 3) == [b"x"]
    assert net.deliver(0.2, 1) == []  # never to the sender
    assert net.deliver(0.2, 2) == []  # delivered once


def test_lossy_network_drops():
    net = LossyNetwork(loss=1.0, delay=0.0)
    net.broadcast(0.0, 1, [1, 2], b"x")
    assert net.deliver(1.0, 2) == [] and net.lost == 1


# ---------------------------------------------------------------- side-belief rules used by team comm


@pytest.fixture
def side(fm, lf):
    s = SideBelief(ParticleFilter(fm, lf, seed=0))
    s.on_initial(0.0, [(-3.0, 1.0, 0.3)])
    s.step(0.0, Estimate((-3.0, 1.0, 0.3), np.eye(3) * 0.01, 1.0, []))
    return s


def rep(t, xy, rid=2, gk=False):
    return TeamReport(t, rid, xy, 0.2, 0.98, gk)


def test_report_before_my_sighting_is_matched_later(side):
    assert side.on_team_report(rep(1.0, (-3.0, -2.0))) == 0.0  # I haven't seen the ball yet: kept pending
    before = side.logodds
    side.on_my_ball(1.4, (3.0, 2.0), 0.2)  # now I see it, mirrored relative to the report
    assert side.logodds < before


def test_old_sightings_are_compared_with_widened_uncertainty(side):
    side.on_my_ball(1.0, (3.0, 2.0), 0.2)
    near = side.report_llr(np.array([3.0, 2.0]), 0.2, rep(1.0, (3.0, 2.0)))
    side2_llr = side.on_team_report(rep(3.5, (3.0, 2.0)))  # 2.5 s later: still informative far from center
    assert 0 < side2_llr <= near
    assert side.on_team_report(rep(9.0, (3.0, 2.0))) == 0.0  # beyond ball_max_age


def _drive_below_threshold(side, sources):
    side.on_my_ball(5.0, (3.0, 2.0), 0.2)
    for rid, gk in sources * 10:
        side.on_team_report(rep(5.0, (-3.0, -2.0), rid, gk))
    assert side.logodds < side.cfg.flip_threshold
    pose = (-3.0, 1.0, 0.3)
    side.step(6.0, Estimate(pose, np.eye(3) * 0.01, 1.0, []))
    return side.step(7.5, Estimate(pose, np.eye(3) * 0.01, 1.0, []))


def test_single_teammate_cannot_flip_me(side):
    assert not _drive_below_threshold(side, [(3, False)])
    assert side.flips == 0 and side.avoid_shooting  # but I know I'm unsure


def test_two_teammates_can_flip_me(side):
    assert _drive_below_threshold(side, [(3, False), (4, False)])


def test_goalkeeper_alone_can_flip_me(side):
    assert _drive_below_threshold(side, [(1, True)])


# ---------------------------------------------------------------- multi-robot scenarios


def team_specs(fm, seed):
    a = fm.dims.length / 2
    gk_head = HeadMotion(yaw_amplitude=1.0, period=5.0, pitch=0.3)
    gk = ScenarioBuilder(fm, seed=seed, head=gk_head).start((-a + 0.6, 0.0, 0.0)).stand(DUR)
    p2 = ScenarioBuilder(fm, seed=seed + 1).start((-3.0, -1.5, 0.2)).walk(DUR)
    p3 = kidnap_to_mirror(ScenarioBuilder(fm, seed=seed + 2).start((-2.0, 1.0, 0.0)).walk(10.0)).walk(
        DUR - 10.0
    )
    return [RobotSpec(1, gk.steps, True), RobotSpec(2, p2.steps), RobotSpec(3, p3.steps)]


def _wrong_half(run):
    r = evaluate(run.gt, run.est, f"p{run.player}")
    return r.samples["wrong_half"].astype(bool), r.samples["timestamp_s"]


def test_teammates_bring_a_kidnapped_robot_back(fm, lf):
    tr = run_team(fm, lf, team_specs(fm, 1), seed=1)
    p3 = tr.robot(3)
    wrong, t = _wrong_half(p3)
    assert wrong.any() and t[wrong][-1] - 10.0 <= 15.0  # back on the correct half within 15 s
    assert len(p3.flip_times) == 1 and p3.side.confidence > 0.9
    for p in (1, 2):
        w, _ = _wrong_half(tr.robot(p))
        assert not w.any() and not tr.robot(p).flip_times  # healthy robots never misled
    used = tr.budget_start - tr.budget_end
    assert 0.8 * 3 * 2.0 * DUR <= used <= 3 * 2.0 * DUR + 3  # ~2 Hz each, within the cap


def test_confidently_wrong_teammate_does_not_flip_a_healthy_robot(fm, lf):
    """Regression for a bug found while building LOC-21.

    Before the consensus rule, player 2 was flipped by kidnapped player 3's mirrored reports.
    """
    tr = run_team(fm, lf, team_specs(fm, 3), seed=3)
    for p in (1, 2):
        w, _ = _wrong_half(tr.robot(p))
        assert not w.any() and not tr.robot(p).flip_times
    # The kidnapped robot couldn't get two corroborating sources in time here, but it knows it is wrong.
    p3 = tr.robot(3)
    if _wrong_half(p3)[0][-1]:
        assert p3.side.avoid_shooting
