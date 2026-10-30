"""Side-belief tests (LOC-06): evidence math, decisions, and full scenarios with a ball and teammates."""

import math

import numpy as np
import pytest
from td_field_model.scenarios import ScenarioBuilder, penalty_reentry_poses
from td_field_model.sim import NoiseModel, Simulator
from td_localization.suite import kidnap_to_mirror
from team_harness import BallTrack, Teammate, run_with_side

from td_field_model import FieldModel
from td_localization import Estimate, ParticleFilter, SideBelief, SideConfig, TeamReport

DEFAULT = NoiseModel(false_positives=2.0)
TEAM = [Teammate(2), Teammate(3, phase=0.25), Teammate(1, is_goalkeeper=True, phase=0.1)]


@pytest.fixture(scope="module")
def fm() -> FieldModel:
    return FieldModel.builtin("hsl_m_2026")


@pytest.fixture(scope="module")
def lf(fm):
    return fm.likelihood_field()


@pytest.fixture
def side(fm, lf) -> SideBelief:
    pf = ParticleFilter(fm, lf, seed=0)
    s = SideBelief(pf)
    s.on_initial(0.0, [(-3.0, 1.0, 0.3)])
    s.step(0.0, est((-3.0, 1.0, 0.3)))  # pose is trustworthy
    return s


def est(pose, weight=1.0) -> Estimate:
    return Estimate(tuple(pose), np.eye(3) * 0.01, weight, [])


def report(t, xy, conf=0.98, gk=False, sigma=0.2, rid=2):
    return TeamReport(t, rid, tuple(xy), sigma, conf, gk)


# ---------------------------------------------------------------- evidence math


def test_report_llr_signs(side):
    me = np.array([3.0, 2.0])
    assert side.report_llr(me, 0.2, report(0, (3.1, 2.0))) > 1.0  # same ball: supports our side
    assert side.report_llr(me, 0.2, report(0, (-3.1, -2.0))) < -1.0  # mirrored: contradicts it
    # Near the center mark a ball and its mirror are almost the same point, so the evidence is weak.
    near_center = np.array([0.1, 0.05])
    weak = side.report_llr(near_center, 0.2, report(0, (0.12, 0.04)))
    strong = side.report_llr(me, 0.2, report(0, (3.1, 2.0)))
    assert 0 < weak < 0.1 * strong


def test_report_llr_is_clipped_and_gated(side):
    me = np.array([6.0, 4.0])
    assert side.report_llr(me, 0.2, report(0, (6.0, 4.0))) == pytest.approx(side.cfg.report_llr_clip)
    # Neither the direct nor the mirrored ball is anywhere close: a wrong pose, not a wrong side.
    assert side.report_llr(me, 0.2, report(0, (0.0, -4.0))) == 0.0


def test_team_report_weighting(side):
    side.logodds = 0.0
    side.on_my_ball(1.0, (3.0, 2.0), 0.2)
    unsure = side.on_team_report(report(1.0, (-3.0, -2.0), conf=0.55))
    assert unsure == 0.0  # below min_teammate_confidence
    field = side.on_team_report(report(1.0, (-3.0, -2.0), conf=1.0))
    gk = side.on_team_report(report(1.0, (-3.0, -2.0), conf=1.0, gk=True, rid=1))
    assert field < 0 and gk == pytest.approx(side.cfg.goalkeeper_weight * field)


def test_stale_ball_and_untrusted_pose_are_ignored(side):
    side.on_my_ball(1.0, (3.0, 2.0), 0.2)
    assert side.on_team_report(report(5.0, (-3.0, -2.0))) == 0.0  # my sighting is 4 s older than theirs
    side.step(2.0, est((-3.0, 1.0, 0.3), weight=0.3))  # not confident in my own pose
    side.on_my_ball(2.0, (-9.0, -9.0), 0.2)
    side.on_my_ball(2.0, (3.0, 2.0), 0.2)
    assert side._my_ball[0] == 1.0  # neither accepted


def test_opponents_at_set(side):
    side.logodds = 0.0
    assert side.on_opponents_at_set(5.0, [(2.0, 1.0), (3.0, -1.0), (0.1, 0.0)]) == pytest.approx(1.0)
    assert side.on_opponents_at_set(5.0, [(-2.0, 1.0)]) == pytest.approx(-0.5)


# ---------------------------------------------------------------- decisions


def test_events_seed_the_filter(fm, lf):
    pf = ParticleFilter(fm, lf, seed=0)
    s = SideBelief(pf)
    s.on_initial(0.0, None)
    assert pf.lost and s.logodds == s.cfg.known_placement_logodds
    s.on_penalty_reentry(5.0, penalty_reentry_poses(fm))
    assert not pf.lost and pf.side_reference[0] < 0
    assert [e.kind for e in s.events] == ["initial", "penalty_reentry"]


def test_flip_needs_sustained_evidence_then_waits(side):
    """Timing only; the consensus requirement is tested in test_team_sim.py."""
    side.cfg = SideConfig(min_flip_sources=0)
    pose = (-3.0, 1.0, 0.3)
    side.logodds = -5.0
    assert not side.step(1.0, est(pose))  # below threshold, hold timer starts
    assert not side.step(1.5, est(pose))
    assert side.step(2.1, est(pose))  # held for ≥ flip_hold
    assert side.flips == 1
    assert side.logodds == pytest.approx(side.cfg.post_flip_logodds)
    assert side.pf.side_reference[0] > 0  # the filter was mirrored
    side.logodds = -8.0
    mirrored = (3.0, -1.0, 0.3 - math.pi)
    for t in (2.2, 3.5, 4.5, 5.0):
        assert not side.step(t, est(mirrored))  # refractory period
    assert not side.step(5.2, est(mirrored))  # refractory over: the hold timer starts now
    assert side.step(6.3, est(mirrored))


def test_jump_caps_confidence(side):
    t0 = side.cfg.post_lost_grace + 1.0  # after the grace period that follows the fixture's re-seed
    side.logodds = 8.0
    side.step(t0, est((-3.0, 1.0, 0.3)))
    side.step(t0 + 0.1, est((2.0, -2.0, 1.5)))  # relocated 5+ m in 0.1 s
    assert side.logodds == pytest.approx(side.cfg.after_jump_logodds)
    assert side.avoid_shooting
    assert side.events[-1].kind == "jump"


def test_no_jump_cap_while_settling_after_a_reseed(side):
    """Right after a re-seed from several candidates, switching between them is expected (grace period)."""
    side.logodds = 6.0
    side.step(0.5, est((-4.0, 4.7, -1.57)))
    side.step(0.6, est((-4.0, -4.7, 1.57)))  # switched to the other touchline candidate
    assert side.logodds == 6.0


def test_decay_while_degraded(side):
    side.logodds = 6.0
    side.pf._degraded = True
    tau = side.cfg.degraded_tau
    side.step(tau, est((-3.0, 1.0, 0.3)))  # the fixture's last step was at t = 0
    assert side.logodds == pytest.approx(6.0 / math.e, rel=0.01)
    side.pf._degraded = False
    side.step(2 * tau, est((-3.0, 1.0, 0.3)))
    assert side.logodds == pytest.approx(6.0 / math.e, rel=0.01)  # no decay while healthy


def test_confidence_and_avoid_shooting():
    cfg = SideConfig()
    s = SideBelief.__new__(SideBelief)
    s.cfg, s.logodds = cfg, 0.0
    assert s.confidence == pytest.approx(0.5) and s.avoid_shooting
    s.logodds = 4.0
    assert s.confidence > 0.98 and not s.avoid_shooting


# ---------------------------------------------------------------- scenarios


def _kidnap_steps(fm, seed, offset=(0.5, 0.0, 0.2)):
    """Adversarial cross-field kidnap: carried to near the mirror of the current pose.

    Continuity then points to the wrong half.
    """
    return (
        kidnap_to_mirror(ScenarioBuilder(fm, seed=seed).start((-2.0, -1.0, 0.0)).walk(10.0), offset)
        .walk(20.0)
        .steps
    )


def _run(fm, lf, steps, team, seed, noise=DEFAULT):
    pf = ParticleFilter(fm, lf, seed=seed)
    side = SideBelief(pf)
    r = run_with_side(
        steps, Simulator(fm, noise=noise, seed=seed), pf, side, BallTrack(fm, seed=seed), team, seed=seed
    )
    return r, side


@pytest.mark.parametrize("seed, offset", [(5, (0.5, 0.0, 0.2)), (2, (1.0, 0.5, -0.3)), (4, (0.3, -0.8, 0.4))])
def test_cross_field_kidnap_resolved_by_teammates(fm, lf, seed, offset):
    r, side = _run(fm, lf, _kidnap_steps(fm, seed, offset), TEAM, seed)
    wrong = r.mirror_closer() & (r.t > 10.0)
    assert wrong.any()  # the scenario really does put the filter on the wrong half first
    settle = r.t[wrong][-1] - 10.0
    assert settle <= 6.0
    assert side.flips == 1
    assert r.pos_err()[-1] < 0.3
    assert r.confidence[-1] > 0.9


def test_cross_field_kidnap_without_teammates_lowers_confidence_when_it_relocates(fm, lf):
    """Alone, a cross-field kidnap that also moves the robot (the estimate jumps).

    It stays on the mirrored side, but it must know it is unsure.
    """
    r, side = _run(fm, lf, _kidnap_steps(fm, 5, offset=(2.0, 1.0, 0.5)), [], 5)
    assert r.mirror_closer()[-1]  # wrong side (continuity picked the mirror)
    assert side.avoid_shooting  # but confidence dropped after the jump


def test_exact_mirror_kidnap_alone_is_undetectable(fm, lf):
    """Known limitation, pinned on purpose.

    Carried to the exact mirror image (+0.5 m), nothing the robot sees changes, so it stays on the wrong side
    and may stay confident. Only teammates or a game event can fix it (see
    test_cross_field_kidnap_resolved_by_teammates and test_penalty_reentry_event_restores_the_side).
    If this starts failing because the robot became unsure, that's an improvement: update the test.
    """
    r, side = _run(fm, lf, _kidnap_steps(fm, 5), [], 5)
    assert r.mirror_closer()[-1]
    assert side.flips == 0


def test_normal_play_with_mixed_team_never_flips(fm, lf):
    team = [
        Teammate(2),
        Teammate(3, flipped=True, side_confidence=0.55),
        Teammate(1, is_goalkeeper=True, phase=0.1),
    ]
    steps = ScenarioBuilder(fm, seed=8).start((-3.0, 1.0, 0.3)).walk(20.0).fall().walk(15.0).steps
    r, side = _run(fm, lf, steps, team, 8)
    assert side.flips == 0
    assert not r.mirror_closer()[r.t > 1.0].any()


def test_one_confidently_wrong_teammate_is_outvoted(fm, lf):
    team = [
        Teammate(2, flipped=True, side_confidence=0.95),
        Teammate(3, phase=0.25),
        Teammate(1, is_goalkeeper=True, phase=0.1),
    ]
    steps = ScenarioBuilder(fm, seed=9).start((-3.0, 1.0, 0.3)).walk(30.0).steps
    r, side = _run(fm, lf, steps, team, 9)
    assert side.flips == 0
    assert r.confidence[-1] > 0.9


def test_penalty_reentry_event_restores_the_side(fm, lf):
    """After an unresolved cross-field kidnap, a penalty re-entry puts the robot back on the right side."""
    steps = (
        kidnap_to_mirror(ScenarioBuilder(fm, seed=5).start((-2.0, -1.0, 0.0)).walk(10.0))
        .walk(8.0)
        .penalty_reentry("right")
        .walk(8.0)
        .steps
    )
    r, side = _run(fm, lf, steps, [], 5)
    t_pen = next(s.t for s in steps if s.event == "penalty_reentry")
    after = r.t > t_pen + 2.0
    assert not r.mirror_closer()[after].any()
    assert side.confidence > 0.9
