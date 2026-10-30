import math

import numpy as np
import pytest
from td_field_model.scenarios import (
    HeadMotion,
    OdometryNoise,
    ScenarioBuilder,
    compose,
    kickoff_poses,
    penalty_reentry_poses,
    relative_motion,
    sideline_start_poses,
)
from td_field_model.sim import NoiseModel, Simulator

from td_field_model import FieldModel


@pytest.fixture(scope="module")
def fm() -> FieldModel:
    return FieldModel.builtin("hsl_m_2026")


def test_compose_and_relative_motion_are_inverse():
    rng = np.random.default_rng(0)
    for _ in range(50):
        p0 = tuple(rng.uniform([-6, -4, -math.pi], [6, 4, math.pi]))
        d = rng.uniform([-0.5, -0.5, -1], [0.5, 0.5, 1])
        assert relative_motion(p0, compose(p0, d)) == pytest.approx(d)


def test_penalty_reentry_poses_follow_the_rules(fm):
    d = fm.dims
    for x, y, th in penalty_reentry_poses(fm):
        assert x == pytest.approx(-(d.length / 2 - d.penalty_mark_distance))  # level with own penalty mark
        assert abs(y) > d.width / 2 + d.line_width / 2  # fully outside the field
        assert math.cos(th - math.atan2(-y, 0.0)) == pytest.approx(1.0)  # facing the opposite touchline


def test_start_poses_are_in_own_half(fm):
    for x, _, _ in kickoff_poses(fm) + sideline_start_poses(fm, 5):
        assert x < 0


def test_noise_free_odometry_integrates_to_truth(fm):
    sb = ScenarioBuilder(fm, seed=1, odom_noise=OdometryNoise.zero()).start((-3.0, 0.0, 0.0))
    sb.walk(20.0).turn(1.0).walk(5.0)
    pose = sb.steps[0].true_pose
    for s in sb.steps[1:]:
        pose = compose(pose, s.odom)
    assert pose[:2] == pytest.approx(sb.pose[:2], abs=1e-6)
    assert math.cos(pose[2] - sb.pose[2]) == pytest.approx(1.0)


def test_walk_stays_near_the_field(fm):
    sb = ScenarioBuilder(fm, seed=2).start((0.0, 0.0, 0.0)).walk(120.0, speed=0.5)
    p = sb.poses()
    assert np.all(np.abs(p[:, 0]) < fm.dims.length / 2 + 0.5)
    assert np.all(np.abs(p[:, 1]) < fm.dims.width / 2 + 0.5)


def test_odometry_noise_scales_with_distance(fm):
    sb = ScenarioBuilder(fm, seed=3, odom_noise=OdometryNoise(trans_frac=0.1)).start((-4.0, 0.0, 0.0))
    sb.walk(30.0)
    err = [
        s.odom[:2] - relative_motion(a.true_pose, s.true_pose)[:2]
        for a, s in zip(sb.steps[:-1], sb.steps[1:], strict=True)
    ]
    step = 0.3 * sb.dt
    assert np.std(np.array(err)) == pytest.approx(0.1 * step, rel=0.3)


def test_events_and_known_poses(fm):
    sb = ScenarioBuilder(fm, seed=4).start((-3.0, 2.0, 0.0)).walk(2.0).fall(heading_change=0.5)
    sb.walk(1.0).kidnap((2.0, -1.0, 1.0)).walk(1.0).penalty_reentry("right").stand(1.0)
    events = [(s.event, s) for s in sb.steps if s.event]
    assert [e for e, _ in events] == ["initial", "fall", "kidnap", "penalty_reentry"]
    by = dict(events)
    assert by["initial"].known_poses == [(-3.0, 2.0, 0.0)]
    assert by["fall"].odom_cov[0, 0] > 0.01 and not np.any(by["fall"].odom)
    assert by["kidnap"].known_poses == [] and by["kidnap"].true_pose == (2.0, -1.0, 1.0)
    assert len(by["penalty_reentry"].known_poses) == 2
    assert by["penalty_reentry"].true_pose == penalty_reentry_poses(fm)[1]


def test_fall_changes_heading_but_odometry_does_not(fm):
    sb = ScenarioBuilder(fm, seed=5).start((-3.0, 0.0, 0.0)).fall(heading_change=0.7)
    before, after = sb.steps[-2].true_pose, sb.steps[-1].true_pose
    assert after[2] - before[2] == pytest.approx(0.7)
    assert not np.any(sb.steps[-1].odom)


def test_head_motion_sweeps():
    h = HeadMotion(yaw_amplitude=0.8, period=4.0, pitch=0.3)
    assert h.at(1.0)[0] == pytest.approx(0.8)
    assert h.at(3.0)[0] == pytest.approx(-0.8)
    assert h.at(0.5)[1] == 0.3


def test_rules_and_builder_errors(fm):
    with pytest.raises(RuntimeError):
        ScenarioBuilder(fm).walk(1.0)
    sb = ScenarioBuilder(fm).start((0.0, 0.0, 0.0))
    with pytest.raises(RuntimeError):
        sb.start((0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        sb.penalty_reentry("middle")


def test_scenario_drives_the_simulator(fm):
    """End-to-end smoke: 20 s at 30 Hz renders quickly and every frame has a true pose attached."""
    sb = ScenarioBuilder(fm, seed=6).start((-4.0, 0.0, 0.0)).walk(10.0).fall().walk(10.0)
    sim = Simulator(fm, noise=NoiseModel(false_positives=1.0), seed=6)
    frames = [sim.render(s.true_pose, s.head_yaw, s.head_pitch, t=s.t) for s in sb.steps]
    assert len(frames) == len(sb.steps)
    assert sum(len(f.line_points) > 0 for f in frames) > 0.8 * len(frames)
