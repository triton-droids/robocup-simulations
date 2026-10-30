import math
import time

import numpy as np
import pytest
from td_field_model.field import GOALPOST, PENALTY_MARK
from td_field_model.sim import (
    CameraModel,
    NoiseModel,
    Obstacle,
    Simulator,
    robot_to_world,
    world_to_robot,
)

from td_field_model import FieldModel, mirror_pose

POSE = (-3.0, 1.0, 0.3)  # own half, looking roughly at the center circle


@pytest.fixture(scope="module")
def fm() -> FieldModel:
    return FieldModel.builtin("hsl_m_2026")


def sim(fm, noise=None, seed=0, **cam):
    return Simulator(fm, CameraModel(**cam), noise or NoiseModel.zero(), seed=seed)


# ---------------------------------------------------------------- frames


def test_world_robot_round_trip():
    pts = np.random.default_rng(0).uniform(-5, 5, (20, 2))
    back = robot_to_world(world_to_robot(pts, POSE), POSE)
    assert np.allclose(back, pts)


def test_robot_frame_x_is_forward():
    p = world_to_robot(np.array([[POSE[0] + math.cos(POSE[2]), POSE[1] + math.sin(POSE[2])]]), POSE)
    assert p[0] == pytest.approx([1.0, 0.0])


def test_camera_looks_forward_and_down():
    r, t = CameraModel().pose_in_base(0.0, 0.35)
    optical_z_in_base = r[:, 2]
    assert optical_z_in_base[0] > 0.9 and optical_z_in_base[2] < 0  # forward, down
    assert t[2] == pytest.approx(1.2)


# ---------------------------------------------------------------- noise-free geometry


def test_noise_free_points_lie_on_lines(fm):
    obs = sim(fm).render(POSE)
    assert len(obs.line_points) > 50
    world = robot_to_world(obs.line_points, POSE)
    assert np.max(fm.distance(world)) < 1e-6


def test_noise_free_landmarks_match_truth(fm):
    obs = sim(fm).render(POSE)
    assert obs.landmarks
    for lm in obs.landmarks:
        truth = next(t for t in fm.landmarks if t.name == lm.truth_name)
        assert lm.kind == truth.kind
        assert np.allclose(robot_to_world(lm.xy, POSE)[0], truth.xy, atol=1e-6)


def test_points_respect_field_of_view_and_range(fm):
    cam = CameraModel(max_range=6.0)
    s = Simulator(fm, cam, NoiseModel.zero())
    obs = s.render(POSE)
    rel = obs.line_points - np.array([cam.mount_forward, 0.0])
    rng = np.hypot(rel[:, 0], rel[:, 1])
    assert np.all(rng <= 6.0 + 1e-9)
    assert np.all(rel[:, 0] > 0)  # nothing behind the camera
    # Every observed point projects inside the image from the true camera pose.
    r, t = cam.pose_in_base(0.0, 0.35)
    _, inside = s._project(obs.line_points, r, t)
    assert np.all(inside)
    # And the field of view is actually limiting: some line points at the same range are left out.
    assert (
        len(obs.line_points)
        < len(Simulator(fm, cam, NoiseModel.zero()).render(POSE, head_yaw=0.0).line_points) + 1
    )
    wide = Simulator(fm, CameraModel(max_range=6.0, fx=300.0, fy=300.0), NoiseModel.zero()).render(POSE)
    assert len(wide.line_points) > len(obs.line_points)


def test_head_yaw_turns_the_view(fm):
    s = sim(fm)
    left = s.render(POSE, head_yaw=0.8)
    right = s.render(POSE, head_yaw=-0.8)
    assert np.mean(left.bearings) > 0.3 and np.mean(right.bearings) < -0.3


def test_looking_at_goal_sees_both_posts(fm):
    a = fm.dims.length / 2
    obs = sim(fm).render((a - 4.0, 0.0, 0.0), head_pitch=0.2)
    names = {lm.truth_name for lm in obs.landmarks_of(GOALPOST)}
    assert names == {"opp_goalpost_left", "opp_goalpost_right"}
    assert obs.landmarks_of(PENALTY_MARK)


def test_mirror_pose_sees_identical_field(fm):
    """The core symmetry problem: lines alone cannot tell a pose from its mirror."""
    s = sim(fm)
    mirrored = mirror_pose(*POSE)
    a, b = s.render(POSE), s.render(mirrored)
    # What the robot sees from POSE fits the field perfectly when placed at the mirrored pose instead.
    assert np.max(fm.distance(robot_to_world(a.line_points, mirrored))) < 1e-6
    # Same number of points up to sampling phase along arcs, and the same landmark sightings.
    assert abs(len(a.line_points) - len(b.line_points)) <= 0.01 * len(a.line_points)
    assert sorted(lm.kind for lm in a.landmarks) == sorted(lm.kind for lm in b.landmarks)
    for lm in a.landmarks:
        twin = min(b.landmarks, key=lambda o: np.linalg.norm(o.xy - lm.xy))
        assert twin.kind == lm.kind and np.allclose(twin.xy, lm.xy, atol=1e-6)


def test_bearing_only_flag(fm):
    obs = sim(fm).render(POSE, head_pitch=0.15)
    d = np.hypot(*obs.line_points.T)
    assert np.all(obs.line_range_ok == (d <= 4.0 + 1e-9))
    assert obs.line_range_ok.any() and (~obs.line_range_ok).any()


# ---------------------------------------------------------------- noise


def _range_errors(fm, noise, n=200):
    s = Simulator(fm, CameraModel(max_range=9.0), noise, seed=3)
    d_all, err_all = [], []
    for _ in range(n):
        obs = s.render(POSE, head_pitch=0.2)
        world = robot_to_world(obs.line_points, POSE)
        # The nearest true line point along the camera ray is hard to recover, so measure the error as the
        # distance to the nearest line. It underestimates the radial error but keeps its growth with distance.
        e = fm.distance(world)
        d_all.append(np.hypot(*obs.line_points.T))
        err_all.append(e)
    return np.concatenate(d_all), np.concatenate(err_all)


def test_error_grows_with_distance(fm):
    noise = NoiseModel(pixel_sigma=0.0, pitch_jitter=math.radians(0.5), yaw_jitter=0.0)
    d, e = _range_errors(fm, noise, n=100)
    near, far = e[(d > 1.5) & (d < 2.5)], e[(d > 5.5) & (d < 6.5)]
    assert len(near) > 100 and len(far) > 100
    assert np.mean(far) > 3 * np.mean(near)


def test_pitch_bias_shifts_points_systematically(fm):
    base = sim(fm).render(POSE, head_pitch=0.2)
    biased = Simulator(fm, CameraModel(), NoiseModel(0.0, math.radians(1.0), 0.0, 0.0)).render(
        POSE, head_pitch=0.2
    )
    # The believed camera looks further down than it really is, so every point is reported closer.
    assert np.mean(np.hypot(*biased.line_points.T)) < np.mean(np.hypot(*base.line_points.T))


def test_covariance_is_consistent_with_noise(fm):
    """Normalized error should match the model's own covariance (single-point check)."""
    noise = NoiseModel(pixel_sigma=1.0, pitch_jitter=math.radians(0.5), yaw_jitter=math.radians(0.5))
    s = Simulator(fm, CameraModel(), noise, seed=4)
    mark = next(lm for lm in fm.landmarks if lm.name == "own_penalty_mark")
    pose = (mark.xy[0] - 3.0, 0.0, 0.0)  # penalty mark 3 m straight ahead
    truth = world_to_robot(np.array(mark.xy), pose)[0]
    z = []
    for _ in range(400):
        lm = next(x for x in s.render(pose, head_pitch=0.3).landmarks if x.truth_name == "own_penalty_mark")
        e = lm.xy - truth
        z.append(e @ np.linalg.solve(lm.cov, e))
    # Mean NEES of a correct 2-D covariance is 2. Allow for the first-order approximation.
    assert 1.2 < np.mean(z) < 3.0


def test_same_seed_is_deterministic(fm):
    noise = NoiseModel(point_dropout=0.1, false_positives=3.0)
    a = Simulator(fm, CameraModel(), noise, seed=7).render(POSE)
    b = Simulator(fm, CameraModel(), noise, seed=7).render(POSE)
    assert np.array_equal(a.line_points, b.line_points)


def test_dropout_and_false_positives(fm):
    noise = NoiseModel.zero()
    full = len(Simulator(fm, CameraModel(), noise).render(POSE).line_points)
    dropped = Simulator(
        fm, CameraModel(), NoiseModel(0.0, 0.0, 0.0, 0.0, point_dropout=0.5, max_line_points=10_000)
    ).render(POSE)
    assert 0.3 * full < len(dropped.line_points) < 0.7 * full
    fp = Simulator(
        fm, CameraModel(), NoiseModel(0.0, 0.0, 0.0, 0.0, false_positives=20.0, max_line_points=None), seed=1
    ).render(POSE)
    assert fp.line_is_false.sum() > 5
    assert np.max(fm.distance(robot_to_world(fp.line_points[~fp.line_is_false], POSE))) < 1e-6


def test_frame_dropout(fm):
    s = Simulator(fm, CameraModel(), NoiseModel(frame_dropout=1.0))
    obs = s.render(POSE)
    assert len(obs.line_points) == 0 and not obs.landmarks


def test_max_line_points_cap(fm):
    s = Simulator(fm, CameraModel(), NoiseModel(0.0, 0.0, 0.0, 0.0, max_line_points=50))
    assert len(s.render(POSE).line_points) == 50


def test_crossing_confusion_changes_kinds(fm):
    s = Simulator(
        fm, CameraModel(), NoiseModel(0.0, 0.0, 0.0, 0.0, crossing_confusion=1.0, max_line_points=None)
    )
    obs = s.render(POSE)
    crossings = [lm for lm in obs.landmarks if lm.kind in ("L", "T", "X")]
    truth = {t.name: t.kind for t in fm.landmarks}
    assert crossings and all(lm.kind != truth[lm.truth_name] for lm in crossings)


# ---------------------------------------------------------------- occlusion


def test_obstacle_hides_points_behind_it(fm):
    s = sim(fm)
    clear = s.render(POSE)
    # A robot 1.5 m straight ahead of the camera.
    ahead = robot_to_world(np.array([[1.5, 0.0]]), POSE)[0]
    blocked = s.render(POSE, obstacles=[Obstacle(ahead[0], ahead[1], radius=0.4)])
    assert len(blocked.line_points) < len(clear.line_points)
    rel = blocked.line_points
    behind_and_central = (rel[:, 0] > 1.6) & (np.abs(np.arctan2(rel[:, 1], rel[:, 0])) < math.atan2(0.3, 1.5))
    assert not behind_and_central.any()


def test_obstacle_behind_the_camera_hides_nothing(fm):
    s = sim(fm)
    behind = robot_to_world(np.array([[-1.0, 0.0]]), POSE)[0]
    assert len(s.render(POSE, obstacles=[Obstacle(*behind)]).line_points) == len(s.render(POSE).line_points)


# ---------------------------------------------------------------- performance


def test_render_speed(fm):
    s = Simulator(fm, CameraModel(), NoiseModel(false_positives=2.0), seed=0)
    t0 = time.perf_counter()
    for i in range(300):
        s.render(POSE, head_yaw=0.5 * math.sin(i / 10))
    per_frame = (time.perf_counter() - t0) / 300
    assert per_frame < 0.02  # 10 s of 30 Hz simulated video renders in well under 10 s
