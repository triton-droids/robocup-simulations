"""Unit tests of the pluggable bench inputs (td_bench.sources)."""

import math
import threading

import numpy as np
import pytest
from td_bench.sources import (
    FixedHead,
    ImuHead,
    LiveHead,
    LiveOdometry,
    LiveVisualOdometry,
    NoTeam,
    ScriptedGame,
    SimBall,
    SimHead,
    SimOdometry,
    SimTeam,
    SimTeammate,
    SimVisualOdometry,
    StationaryOdometry,
    _compose_rel,
    default_world,
)
from td_field_model.scenarios import relative_motion
from td_localization.suite import BallTrack

from td_field_model import FieldModel


@pytest.fixture(scope="module")
def fm():
    return FieldModel.builtin("hsl_m_2026")


def test_world_index_clamps(fm):
    w = default_world(fm, "walk", duration=2.0)
    assert w.index(-1.0) == 0 and w.index(99.0) == len(w.steps) - 1
    assert w.step(1.0).t == pytest.approx(1.0, abs=w.dt)


def test_compose_rel_turn_then_forward():
    d = _compose_rel(np.zeros(3), (1.0, 0.0, math.pi / 2))
    d = _compose_rel(d, (1.0, 0.0, 0.0))
    assert np.allclose(d, (1.0, 1.0, math.pi / 2))


def test_sim_odometry_accumulates_between_calls(fm):
    w = default_world(fm, "walk", seed=3, duration=3.0)
    one_shot = SimOdometry(w).get(2.0)[0]
    inc = SimOdometry(w)
    d = np.zeros(3)
    for k in range(1, 61):
        d = _compose_rel(d, inc.get(k * w.dt)[0])
    assert np.allclose(d, one_shot, atol=1e-9)
    # without noise this would equal the true motion; with it, it's close
    true = relative_motion(w.steps[0].true_pose, w.step(2.0).true_pose)
    assert np.hypot(*(one_shot[:2] - true[:2])) < 0.2
    assert SimOdometry(w).get(0.0)[0].tolist() == [0.0, 0.0, 0.0]


def test_stationary_odometry():
    d, cov = StationaryOdometry().get(5.0)
    assert not d.any() and cov[0, 0] > 0


def test_live_odometry_push_from_thread():
    odo = LiveOdometry()

    def producer():
        for _ in range(100):
            odo.push(0.01, 0.0, 0.0, np.eye(3) * 1e-6)

    th = threading.Thread(target=producer)
    th.start()
    th.join()
    d, cov = odo.get(0.0)
    assert d[0] == pytest.approx(1.0) and cov[0, 0] == pytest.approx(1e-4 + 0.002**2)
    d2, _ = odo.get(0.1)  # drained
    assert not d2.any()


def test_heads(fm):
    assert FixedHead(0.1, 0.4).get(3.0) == (0.1, 0.4)
    h = LiveHead()
    h.push(-0.2, 0.5)
    assert h.get(0.0) == (-0.2, 0.5)
    w = default_world(fm, "tripod", duration=6.0)
    yaws = [SimHead(w).get(t)[0] for t in np.arange(0, 6, 0.5)]
    assert max(yaws) > 0.5 and min(yaws) < -0.5  # scanning
    still = default_world(fm, "still", duration=1.0, head_pitch=0.5)
    assert SimHead(still).get(0.5) == (0.0, 0.5)


def test_scripted_game_emits_each_event_once():
    g = ScriptedGame([(5.0, "penalized", "pushing"), (0.0, "initial"), (8.0, "unpenalized_in_place")])
    assert [e.kind for e in g.get(0.0)] == ["initial"]
    assert g.get(1.0) == []
    ev = g.get(9.0)
    assert [e.kind for e in ev] == ["penalized", "unpenalized_in_place"]
    assert ev[0].detail == "pushing" and ev[1].in_place
    assert g.get(20.0) == []


def test_sim_ball_and_team(fm):
    w = default_world(fm, "still", duration=5.0, start=(-1.0, 0.0, 0.0))
    ball = SimBall(w, BallTrack(fm, seed=1, start=(0.5, 0.0), speed=0.0), seed=1)
    rel, sig = ball.get(0.0)
    assert np.hypot(*(rel - (1.5, 0.0))) < 5 * sig  # straight ahead, 1.5 m
    team = SimTeam(ball, [SimTeammate(1, is_goalkeeper=True), SimTeammate(3, flipped=True)], rate_hz=2.0)
    reps = team.get(0.0)
    assert {r.robot_id for r in reps} == {1, 3}
    flipped = next(r for r in reps if r.robot_id == 3)
    assert flipped.ball_xy[0] < 0  # mirrored report
    assert team.get(0.1) == []  # rate limited
    assert len(team.get(0.6)) == 2
    assert NoTeam().get(0.0) == []


def test_ball_out_of_view(fm):
    w = default_world(fm, "still", duration=1.0, start=(-1.0, 0.0, 0.0))
    behind = SimBall(w, BallTrack(fm, seed=1, start=(-3.0, 0.0), speed=0.0))
    assert behind.get(0.0) is None


def test_unknown_world(fm):
    with pytest.raises(ValueError):
        default_world(fm, "swim")


# ---------------------------------------------------------------- visual odometry (cuVSLAM model), IMU head


def _dead_reckon(src, w):
    from td_field_model.scenarios import compose

    p = w.steps[0].true_pose
    for s in w.steps[1:]:
        p = compose(p, src.get(s.t)[0])
    return np.array(p)


def test_visual_odometry_follows_motion_when_tracking(fm):
    w = default_world(fm, "walk", seed=3, duration=10.0)
    vo = SimVisualOdometry(w, SimHead(w), seed=3, pan_gate=math.inf, loss_rate=0.0)
    end = _dead_reckon(vo, w)
    true = np.array(w.steps[-1].true_pose)
    assert vo.lost_fraction == 0.0
    assert np.hypot(*(end[:2] - true[:2])) < 0.3  # ~3 m walked, a few % drift
    assert abs(math.remainder(end[2] - true[2], 2 * math.pi)) < math.radians(3)


def test_visual_odometry_drops_on_fast_pans(fm):
    fast = default_world(fm, "walk", seed=3, duration=10.0)  # head scan peaks at 1.26 rad/s
    vo = SimVisualOdometry(fast, ImuHead(fast), seed=3, loss_rate=0.0)
    d, cov = vo.get(10.0)
    assert vo.lost_fraction > 0.1 and cov[0, 0] > 0
    still = default_world(fm, "still", seed=3, duration=10.0)
    vo2 = SimVisualOdometry(still, ImuHead(still), seed=3, loss_rate=0.0)
    vo2.get(10.0)
    assert vo2.lost_fraction == 0.0


def test_visual_odometry_head_error_leaks_into_yaw(fm):
    w = default_world(fm, "tripod", seed=4, duration=6.0)  # robot still, head scanning
    exact = SimVisualOdometry(
        w, SimHead(w), seed=4, pan_gate=math.inf, loss_rate=0.0, yaw_noise=0.0, yaw_bias_sigma=0.0
    )
    noisy = SimVisualOdometry(
        w,
        ImuHead(w, noise=math.radians(1.0)),
        seed=4,
        pan_gate=math.inf,
        loss_rate=0.0,
        yaw_noise=0.0,
        yaw_bias_sigma=0.0,
    )
    yaw_exact = [exact.get(s.t)[0][2] for s in w.steps[1:]]
    yaw_noisy = [noisy.get(s.t)[0][2] for s in w.steps[1:]]
    assert np.std(yaw_exact) < 1e-9  # body still: head motion fully subtracted
    assert np.std(yaw_noisy) > math.radians(0.5)  # head-angle noise shows up as body rotation


def test_live_visual_odometry_increments_and_tracking_loss():
    vo = LiveVisualOdometry()
    vo.push_pose(1.0, 1.0, math.pi / 2)  # reference
    vo.push_pose(1.0, 2.0, math.pi / 2)  # 1 m forward (along +y in the VO frame)
    d, cov = vo.get(0.0)
    assert np.allclose(d, (1.0, 0.0, 0.0), atol=1e-9) and cov[0, 0] > 0
    vo.push_pose(0, 0, 0, tracking=False)
    d, cov_lost = vo.get(0.1)
    assert not d.any() and cov_lost[0, 0] > 1e-5  # blind while lost
    vo.push_pose(50.0, -3.0, 2.0)  # VO frame jumped on recovery: only a new reference
    vo.push_pose(50.0, -3.0, 2.1)
    d, _ = vo.get(0.2)
    assert np.allclose(d, (0.0, 0.0, 0.1), atol=1e-9)


def test_imu_head_bias_and_consistency(fm):
    w = default_world(fm, "still", seed=5, duration=4.0, head_pitch=0.4)
    h = ImuHead(w, seed=5, pitch_bias_sigma=math.radians(1.0), noise=math.radians(0.2))
    assert h.get(1.0) == h.get(1.0)  # one sample per tick
    pitch = np.array([h.get(s.t)[1] for s in w.steps])
    assert abs(pitch.mean() - (0.4 + h.pitch_bias)) < math.radians(0.1)
    assert abs(h.pitch_bias) < math.radians(4)
