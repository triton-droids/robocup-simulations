"""Particle filter tests (LOC-05).

Unit tests check each mechanism. Scenario tests drive the filter with td_field_model's simulator and check the
docs/roadmap.md §3.3 targets:
- position p50 ≤ 0.2 m, p95 ≤ 0.5 m;
- heading p95 ≤ 5°;
- converge ≤ 10 s;
- zero side flips.
"""

import math

import numpy as np
import pytest
from helpers import deg, p, run
from td_field_model.scenarios import ScenarioBuilder, penalty_reentry_poses, sideline_start_poses
from td_field_model.sim import NoiseModel, Simulator
from td_localization.suite import kidnap_same_half

from td_field_model import FieldModel, mirror_pose
from td_localization import ParticleFilter, PFConfig

HARSH = NoiseModel(
    pitch_bias=math.radians(1.0), false_positives=5.0, point_dropout=0.2, crossing_confusion=0.1
)
DEFAULT = NoiseModel(false_positives=2.0)


@pytest.fixture(scope="module")
def fm() -> FieldModel:
    return FieldModel.builtin("hsl_m_2026")


@pytest.fixture(scope="module")
def lf(fm):
    return fm.likelihood_field()


def make_pf(fm, lf, seed=0, **cfg) -> ParticleFilter:
    return ParticleFilter(fm, lf, PFConfig(**cfg), seed=seed)


def converge_time(r, after: float, tol: float = 0.3, hold: float = 2.0) -> float | None:
    """Seconds after `after` until position error stays below tol for `hold` seconds."""
    a = r.after(after)
    ok = a.pos_err() < tol
    for i in range(len(ok)):
        j = np.searchsorted(a.t, a.t[i] + hold)
        if j <= len(ok) and ok[i:j].all():
            return float(a.t[i] - after)
    return None


# ---------------------------------------------------------------- unit: initialization and motion


def test_init_poses_splits_particles_and_sets_reference(fm, lf):
    pf = make_pf(fm, lf, n_particles=400)
    cands = penalty_reentry_poses(fm)
    pf.init_poses(cands, sigma_xy=0.01, sigma_theta=0.01)
    near = [np.sum(np.hypot(*(pf.p[:, :2] - np.array(c[:2])).T) < 0.1) for c in cands]
    assert near == [200, 200]
    assert pf.side_reference[0] < 0  # candidates are both in the own half


def test_init_uniform_own_half(fm, lf):
    pf = make_pf(fm, lf, n_particles=300, n_particles_lost=2000)
    pf.init_uniform(own_half_only=True)
    assert len(pf.p) == 2000
    assert np.all(pf.p[:, 0] <= 0)
    assert pf.w.sum() == pytest.approx(1.0)


def test_predict_without_noise_moves_exactly(fm, lf):
    pf = make_pf(fm, lf, n_particles=10, odom_noise_scale=0.0, odom_sigma_floor=(0.0, 0.0, 0.0))
    pf.init_poses([(1.0, 2.0, math.pi / 2)], sigma_xy=0.0, sigma_theta=0.0)
    pf.predict((1.0, 0.0, 0.1), np.zeros((3, 3)))
    assert np.allclose(pf.p, [1.0, 3.0, math.pi / 2 + 0.1])


def test_predict_spreads_with_covariance(fm, lf):
    pf = make_pf(fm, lf, n_particles=2000, odom_noise_scale=1.0)
    pf.init_poses([(0.0, 0.0, 0.0)], sigma_xy=0.0, sigma_theta=0.0)
    pf.predict((0.0, 0.0, 0.0), np.diag([0.1**2, 0.1**2, 0.2**2]))
    assert np.std(pf.p[:, 0]) == pytest.approx(0.1, rel=0.15)
    assert np.std(pf.p[:, 2]) == pytest.approx(0.2, rel=0.15)


def test_mirror_flips_particles_and_reference(fm, lf):
    pf = make_pf(fm, lf, n_particles=50)
    pf.init_poses([(-3.0, 1.0, 0.3)])
    before = pf.p.copy()
    pf.mirror()
    assert np.allclose(pf.p[:, :2], -before[:, :2])
    assert pf.side_reference[0] > 0


def test_fold_uses_heading_near_center(fm, lf):
    """Near the center mark a pose and its mirror are centimeters apart; heading must decide."""
    pf = make_pf(fm, lf)
    pf.side_reference = (0.1, 0.0, 0.0)
    poses = np.array(
        [[0.1, 0.0, 0.05], [-0.1, 0.0, math.pi]]
    )  # second is the mirror of a near-reference pose
    folded = pf._fold_to_reference(poses)
    assert np.allclose(folded[0], poses[0])
    assert np.allclose(folded[1], [0.1, 0.0, 0.0], atol=1e-9)


# ---------------------------------------------------------------- unit: update and estimate


def test_update_prefers_true_pose(fm, lf):
    pose = (-3.0, 1.0, 0.3)
    obs = Simulator(fm, noise=NoiseModel.zero()).render(pose)
    pf = make_pf(fm, lf, n_particles=2, resample_ess_fraction=0.0, drop_threshold=1.0)
    pf.p = np.array([pose, (pose[0] + 0.5, pose[1], pose[2])])
    pf.w = np.full(2, 0.5)
    pf.update(obs)
    assert pf.w[0] > 0.99


def test_update_skips_frames_with_little_to_see(fm, lf):
    pf = make_pf(fm, lf, n_particles=100)
    pf.init_poses([(-3.0, 1.0, 0.3)])
    w, p0 = pf.w.copy(), pf.p.copy()

    class Few:
        line_points = np.array([[1.0, 0.0], [2.0, 0.0]])
        line_cov = np.tile(np.eye(2) * 0.01, (2, 1, 1))
        landmarks = []

    stats = pf.update(Few())
    assert not stats.resampled and np.array_equal(pf.w, w) and np.array_equal(pf.p, p0)


def test_clusters_find_two_hypotheses(fm, lf):
    pf = make_pf(fm, lf, n_particles=600)
    pf.init_poses([(-3.0, 1.0, 0.3), (2.0, -2.0, -1.0)], sigma_xy=0.05, sigma_theta=0.03)
    pf.w = np.where(pf.p[:, 0] < 0, 0.7, 0.3)
    pf.w /= pf.w.sum()
    ks = pf.clusters()
    assert len(ks) == 2
    assert ks[0].weight == pytest.approx(0.7, abs=0.02) and ks[0].pose[0] < 0
    assert ks[1].weight == pytest.approx(0.3, abs=0.02)


def test_estimate_hysteresis_keeps_previous_cluster(fm, lf):
    pf = make_pf(fm, lf, n_particles=600)
    pf.init_poses([(-3.0, 1.0, 0.3), (2.0, -2.0, -1.0), (0.0, 3.0, 2.0)], sigma_xy=0.05, sigma_theta=0.03)
    pf._last_estimate = (-3.0, 1.0, 0.3)
    # No cluster is dominant; the one near the last estimate should be reported though it is not the heaviest.
    pf.w = np.select([pf.p[:, 0] < -1, pf.p[:, 0] > 1], [0.30, 0.40], 0.30)
    pf.w /= pf.w.sum()
    assert pf.estimate(update_reference=False).pose[0] == pytest.approx(-3.0, abs=0.1)


def test_proposals_beat_random_poses(fm, lf):
    pose = (-3.0, 1.0, 0.3)
    obs = Simulator(fm, noise=NoiseModel.zero()).render(pose)
    pf = make_pf(fm, lf)
    pf.side_reference = pose
    sig = np.full(len(obs.line_points), 0.1)
    proposed = pf._propose(20, obs.line_points, sig)
    random = pf._fold_to_reference(pf._random_poses(20))
    ll_p = pf._line_loglik(proposed, obs.line_points, sig)
    ll_r = pf._line_loglik(random, obs.line_points, sig)
    assert np.median(ll_p) > np.median(ll_r)


# ---------------------------------------------------------------- scenarios (roadmap §3.3 targets)


@pytest.mark.parametrize("noise_name", ["default", "harsh"])
def test_tracking_meets_targets(fm, lf, noise_name):
    noise = DEFAULT if noise_name == "default" else HARSH
    start = (-3.0, 1.0, 0.3)
    sb = ScenarioBuilder(fm, seed=11).start(start).walk(40.0)
    pf = make_pf(fm, lf, seed=11)
    pf.init_poses([start])
    r = run(sb.steps, Simulator(fm, noise=noise, seed=11), pf).after(1.0)
    assert p(r.pos_err(), 50) <= 0.2
    assert p(r.pos_err(), 95) <= 0.5
    assert deg(p(r.head_err(), 95)) <= 5.0
    assert not r.mirror_closer().any()


@pytest.mark.parametrize(
    "seed, start_index", [(12, 0), (21, 1), (22, 2)]
)  # regressions before lost mode existed
def test_global_localization_from_sideline(fm, lf, seed, start_index):
    start = sideline_start_poses(fm, 3)[start_index]
    sb = ScenarioBuilder(fm, seed=seed).start(start).stand(8.0).walk(4.0)
    pf = make_pf(fm, lf, seed=seed)
    pf.init_uniform(own_half_only=True)
    assert pf.lost
    r = run(sb.steps, Simulator(fm, noise=DEFAULT, seed=seed), pf)
    t = converge_time(r, 0.0)
    assert t is not None and t <= 10.0
    assert not r.after(t).mirror_closer().any()
    assert not pf.lost
    assert len(pf.p) == pf.cfg.n_particles  # shrank back from the lost-mode particle count


@pytest.mark.parametrize("touchline", ["left", "right"])
def test_penalty_reentry_picks_the_right_touchline(fm, lf, touchline):
    cands = penalty_reentry_poses(fm)
    true = cands[0 if touchline == "left" else 1]
    sb = ScenarioBuilder(fm, seed=13).start(true, known_poses=cands).stand(3.0).walk(10.0)
    pf = make_pf(fm, lf, seed=13)
    pf.init_poses(cands)
    r = run(sb.steps, Simulator(fm, noise=DEFAULT, seed=13), pf)
    t = converge_time(r, 0.0)
    assert t is not None and t <= 3.0
    assert not r.mirror_closer().any()


def test_fall_recovery(fm, lf):
    start = (-2.0, -1.0, 0.0)
    sb = ScenarioBuilder(fm, seed=14).start(start).walk(10.0).fall(heading_change=0.7).walk(10.0)
    pf = make_pf(fm, lf, seed=14)
    pf.init_poses([start])
    r = run(sb.steps, Simulator(fm, noise=DEFAULT, seed=14), pf)
    t_fall = next(s.t for s in sb.steps if s.event == "fall")
    t = converge_time(r, t_fall)
    assert t is not None and t <= 3.0
    assert not r.mirror_closer().any()


@pytest.mark.parametrize(
    "seed", [1, 4, 5]
)  # seeds 1 and 4 drifted ~1 m for ~10 s before the drift test (LOC-07)
def test_kidnap_within_same_half_recovers(fm, lf, seed):
    """A push or a short carry (relative move, so continuity holds). Covers the drift case the suite found."""
    start = (-2.0, -1.0, 0.0)
    sb = kidnap_same_half(ScenarioBuilder(fm, seed=seed).start(start).walk(10.0)).walk(15.0)
    pf = make_pf(fm, lf, seed=seed)
    pf.init_poses([start])
    r = run(sb.steps, Simulator(fm, noise=DEFAULT, seed=seed), pf)
    t_kid = next(s.t for s in sb.steps if s.event == "kidnap")
    t = converge_time(r, t_kid)
    assert t is not None and t <= 5.0
    assert not r.after(t_kid).mirror_closer().any()


def test_drift_test_fires_on_offset_cloud_not_on_good_one(fm, lf):
    """Unit check of the local drift test: an offset cloud is detected within drift_window frames, a good
    cloud is not."""
    pose = (-3.0, 1.0, 0.3)
    sim = Simulator(fm, noise=NoiseModel(pixel_sigma=0.5, pitch_jitter=0.0, yaw_jitter=0.0), seed=3)
    good = make_pf(fm, lf, seed=3)
    good.init_poses([pose], sigma_xy=0.02, sigma_theta=0.01)
    off = make_pf(fm, lf, seed=3)
    off.init_poses([(pose[0] + 0.8, pose[1] - 0.3, pose[2])], sigma_xy=0.02, sigma_theta=0.01)
    good_fired = off_fired = 0
    for i in range(8):
        obs = sim.render(pose, head_yaw=0.3 * math.sin(i), head_pitch=0.35)
        good_fired += good.update(obs).drift
        off_fired += off.update(obs).drift
    assert good_fired == 0
    assert off_fired >= 1


def test_long_run_with_falls_never_flips(fm, lf):
    start = (-4.0, 0.0, 0.0)
    sb = ScenarioBuilder(fm, seed=16).start(start)
    for _ in range(3):
        sb.walk(15.0).fall()
    sb.walk(10.0)
    pf = make_pf(fm, lf, seed=16)
    pf.init_poses([start])
    r = run(sb.steps, Simulator(fm, noise=HARSH, seed=16), pf)
    assert not r.mirror_closer().any()
    assert p(r.pos_err(), 95) <= 0.5


def test_l_field_tracking():
    fm_l = FieldModel.builtin("hsl_l_2026")
    start = (-5.0, 2.0, 0.3)
    sb = ScenarioBuilder(fm_l, seed=17).start(start).walk(30.0)
    pf = ParticleFilter(fm_l, fm_l.likelihood_field(), seed=17)
    pf.init_poses([start])
    r = run(sb.steps, Simulator(fm_l, noise=DEFAULT, seed=17), pf).after(1.0)
    assert p(r.pos_err(), 95) <= 0.5
    assert not r.mirror_closer().any()


def test_update_time_budget(fm, lf):
    """500 particles × 120 points. Laptop numpy is ~10 ms; the bound is loose so slower boxes don't flake."""
    start = (-3.0, 1.0, 0.3)
    sb = ScenarioBuilder(fm, seed=18).start(start).walk(5.0)
    pf = make_pf(fm, lf, seed=18)
    pf.init_poses([start])
    r = run(sb.steps, Simulator(fm, noise=DEFAULT, seed=18), pf)
    assert np.nanpercentile(r.update_ms, 50) < 40.0


def test_mirror_symmetry_of_pose_is_used_correctly():
    assert mirror_pose(1.0, 2.0, 0.0)[:2] == (-1.0, -2.0)
