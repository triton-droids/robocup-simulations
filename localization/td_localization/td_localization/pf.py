"""Monte Carlo localization on the HSL field (LOC-05). Pure numpy, no ROS.

Design (docs/roadmap.md §3.5, layer 3):
- State: N particles (x, y, θ) in the field frame (own goal at -x), with normalized weights.
- Predict: compose each particle with the odometry motion (robot frame), plus noise drawn from the odometry
  covariance (scaled) and a small floor. A fall shows up as a large covariance and spreads the cloud.
- Update:
  - Line points: map each robot-frame point into the field with each particle. Score its distance to the
    nearest line (LikelihoodField) with a robust Cauchy kernel. Per-point σ is the largest standard deviation
    of its 2×2 covariance (the range direction), so far points constrain little and act almost as bearings.
  - Landmarks (L/T/X crossings, marks, posts): distance to the nearest map landmark of the same kind, with
    the same kernel.
  - Points along a line are strongly correlated, so the summed line log-likelihood is tempered.
  - Frames with too little to see (head turned off the field) are skipped.
- Resample: low-variance, when the effective sample size drops.
- Recovery (augmented MCL with sensor resetting):
  - A fast/slow match-quality ratio measures how much matches have degraded beyond normal scene variation.
  - In proportion, the filter roughens the cloud and injects random poses, pre-screened against the current
    frame's lines.
- Lost mode (init_uniform): more particles, screened proposals every frame, until one cluster stays confident.
- **Symmetry: the filter never chooses the half.** Every injected pose is folded to whichever of
  {p, mirror(p)} is closer to `side_reference` (position and heading), or to x ≤ 0 while lost in our own half.
  - The reference follows the estimate only while matches are healthy, confident and continuous.
  - The side-belief module (LOC-06) may set it.
  - Line matching scores a pose and its mirror identically, so this is the only place the half is decided.

Observations are duck-typed: any object with `line_points` (N, 2), `line_cov` (N, 2, 2) and `landmarks` whose
items have `kind`, `xy` and `cov`. td_field_model.sim.Observations fits; the future ROS node can pass its own.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from td_field_model import FieldModel, LikelihoodField, mirror_pose

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class PFConfig:
    n_particles: int = 500
    max_points: int = 120  # line points used per update (random subset); cost scales with particles × points
    point_sigma_floor: float = 0.05  # m, added in quadrature to each point's σ (line half-width, map error)
    line_temperature: float = 0.15  # scales the summed line log-likelihood (points are correlated)
    landmark_sigma_floor: float = 0.15  # m
    landmark_temperature: float = 1.0
    odom_noise_scale: float = 1.5  # inflate reported odometry σ (be conservative)
    odom_sigma_floor: tuple[float, float, float] = (0.003, 0.003, 0.003)  # per predict (m, m, rad)
    resample_ess_fraction: float = 0.5
    inject_max_fraction: float = 0.1
    roughen_xy: float = (
        0.15  # m, extra diffusion per update at zero match quality (scaled by the quality drop)
    )
    roughen_theta: float = 0.08  # rad
    quality_alpha_slow: float = (
        0.005  # ~13 s time constant at 15 Hz, so a wrong-but-stable fit stays "degraded"
    )
    proposal_factor: int = 40  # random candidates screened per injected particle (sensor resetting)
    proposal_points: int = 40  # line points used to screen candidates
    drift_candidates: int = 200  # local poses tested around the estimate each informative frame
    drift_sigma_xy: float = 0.6  # m, spread of the local drift test
    drift_sigma_theta: float = 0.25  # rad
    drift_wide_sigma_xy: float = 1.5  # m, second (wide) half of the candidates
    drift_wide_sigma_theta: float = 0.6  # rad
    drift_margin_per_point: float = 0.5  # untempered log-likelihood per point a local pose must win by
    drift_window: int = 4  # informative frames considered...
    drift_votes: int = 3  # ...of which this many must show the gap (one noisy frame never fires)
    drift_inject_fraction: float = 0.05  # best local candidates injected when the test fires
    quality_alpha_fast: float = 0.3
    cluster_radius: float = 0.5  # m
    cluster_angle: float = 0.5  # rad
    min_cluster_weight: float = 0.05  # clusters below this weight are not counted
    confident_cluster_weight: float = 0.8  # best cluster above this may update side_reference
    reference_jump: float = (
        1.5  # pose distance (m, heading via fold_heading_scale) the reference may move at once
    )
    reference_streak: int = 30  # ...unless confident this many estimates in a row (about 1 s at 30 Hz)
    min_obs_for_quality: int = 20  # fewer line points and no landmarks → frame does not count toward quality
    min_points_for_update: int = 10  # fewer line points and no landmarks → frame is skipped entirely
    drop_threshold: float = 0.3  # quality drop (1 - fast/slow) below this is normal scene variation
    keep_previous_min_weight: float = 0.2  # report the cluster near the last estimate if it holds this much
    random_margin: float = 0.5  # m beyond the field lines where random particles may appear
    fold_heading_scale: float = (
        2.0  # m per rad, heading weight when folding random poses to the reference side
    )
    n_particles_lost: int = 3000  # particle count during global localization (lost mode)
    lost_until_cluster_weight: float = 0.8  # lost mode ends after the best cluster holds this much weight...
    lost_confident_streak: int = 15  # ...for this many consecutive informative updates (~1 s at 15 Hz)
    lost_inject_fraction: float = 0.1  # proposals injected per informative update while lost
    lost_proposal_factor: int = (
        10  # candidates screened per injected particle while lost (cheaper, many more)
    )


@dataclass
class UpdateStats:
    n_points: int
    n_landmarks: int
    ess: float
    quality: float
    injected: int
    resampled: bool
    drift: bool = False  # the local drift test fired on this update


@dataclass
class Cluster:
    pose: Pose
    cov: np.ndarray  # 3×3 (x, y, θ)
    weight: float
    size: int


@dataclass
class Estimate:
    pose: Pose
    cov: np.ndarray
    weight: float  # weight of the best cluster
    clusters: list[Cluster] = field(default_factory=list)

    @property
    def n_clusters(self) -> int:
        return len(self.clusters)


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _lambda_max(cov: np.ndarray) -> np.ndarray:
    """Largest eigenvalue of each 2×2 covariance in (N, 2, 2)."""
    a, b, c = cov[:, 0, 0], cov[:, 0, 1], cov[:, 1, 1]
    return 0.5 * (a + c) + np.sqrt(0.25 * (a - c) ** 2 + b * b)


class ParticleFilter:
    def __init__(
        self,
        field_model: FieldModel,
        likelihood: LikelihoodField | None = None,
        config: PFConfig | None = None,
        seed: int = 0,
    ):
        self.field = field_model
        self.lf = likelihood or field_model.likelihood_field()
        self.cfg = config or PFConfig()
        self.rng = np.random.default_rng(seed)
        n = self.cfg.n_particles
        self.p = np.zeros((n, 3))
        self.w = np.full(n, 1.0 / n)
        self.side_reference: Pose | None = None
        self._last_estimate: Pose | None = None
        self._confident_streak = 0
        self._degraded = False
        self._lost = False  # global localization in progress (see init_uniform)
        self._own_half_only = False  # while lost: constrain every hypothesis to x ≤ 0
        self._lost_streak = 0
        self._drift = False
        self._drift_hist: list[bool] = []
        self._q_slow = 0.0
        self._q_fast = 0.0
        self._lm_by_kind: dict[str, np.ndarray] = {}
        for lm in field_model.landmarks:
            self._lm_by_kind.setdefault(lm.kind, []).append(lm.xy)
        self._lm_by_kind = {k: np.array(v) for k, v in self._lm_by_kind.items()}

    # ---------------------------------------------------------------- initialization

    def init_poses(self, poses, sigma_xy: float = 0.15, sigma_theta: float = 0.1) -> None:
        """Gaussian clouds around known candidate poses (Initial placement, penalty re-entry).

        Particles are split evenly across the candidates.
        """
        poses = np.atleast_2d(np.asarray(poses, float))
        n = self.cfg.n_particles
        idx = np.arange(n) % len(poses)
        self.p = poses[idx] + self.rng.normal(0.0, 1.0, (n, 3)) * [sigma_xy, sigma_xy, sigma_theta]
        self.p[:, 2] = wrap(self.p[:, 2])
        self.w = np.full(n, 1.0 / n)
        self._reset_quality()
        self._lost = self._own_half_only = False
        # The candidates are known to be on the correct half, so their centroid sets the side reference.
        mean = poses.mean(axis=0)
        self.side_reference = (float(mean[0]), float(mean[1]), 0.0)

    def init_uniform(self, own_half_only: bool = True) -> None:
        """Global localization ("lost mode").

        With own_half_only, every hypothesis stays in our half (x ≤ 0), as the Initial state guarantees.

        Lost mode uses n_particles_lost particles and injects screened proposals every informative frame.
        It ends once one cluster has been confident for lost_confident_streak updates: the filter shrinks to
        n_particles and hands the side decision to side_reference.
        """
        n = max(self.cfg.n_particles_lost, self.cfg.n_particles)
        self._reset_quality()
        self._lost, self._own_half_only, self._lost_streak = True, own_half_only, 0
        self.side_reference = None
        self.p = self._fold_to_reference(self._random_poses(n))
        self.w = np.full(n, 1.0 / n)

    def _reset_quality(self) -> None:
        self._q_slow = self._q_fast = 0.0
        self._last_estimate = None
        self._confident_streak = 0
        self._degraded = False
        self._drift = False
        self._drift_hist = []

    def _random_poses(self, n: int) -> np.ndarray:
        m = self.cfg.random_margin
        a, b = self.field.dims.length / 2 + m, self.field.dims.width / 2 + m
        return np.column_stack(
            [self.rng.uniform(-a, a, n), self.rng.uniform(-b, b, n), self.rng.uniform(-math.pi, math.pi, n)]
        )

    def pose_distance(self, poses: np.ndarray, ref) -> np.ndarray:
        """Distance in (x, y, θ) with heading weighted by fold_heading_scale (m per rad).

        Heading matters most near the center mark, where a pose and its mirror are close in position but face
        opposite ways.
        """
        ref = np.asarray(ref, float)
        dpos2 = (poses[:, 0] - ref[0]) ** 2 + (poses[:, 1] - ref[1]) ** 2
        dth = wrap(poses[:, 2] - ref[2])
        return np.sqrt(dpos2 + (self.cfg.fold_heading_scale * dth) ** 2)

    def _fold_to_reference(self, poses: np.ndarray) -> np.ndarray:
        """Put each pose on the believed side.

        - Lost in our own half: mirror poses with x > 0.
        - Otherwise: mirror where the mirror is closer to side_reference (position and heading).
        """
        mirrored = poses.copy()
        mirrored[:, :2] *= -1
        mirrored[:, 2] = wrap(mirrored[:, 2] + math.pi)
        if self._lost and self._own_half_only:
            flip = poses[:, 0] > 0
        elif self.side_reference is None:
            return poses
        else:
            flip = self.pose_distance(mirrored, self.side_reference) < self.pose_distance(
                poses, self.side_reference
            )
        out = poses.copy()
        out[flip, :2] *= -1
        out[flip, 2] = wrap(out[flip, 2] + math.pi)
        return out

    # ---------------------------------------------------------------- predict

    def predict(self, odom, odom_cov) -> None:
        """Move particles by a robot-frame motion (dx, dy, dθ) with 3×3 covariance."""
        cfg = self.cfg
        sigma = np.sqrt(np.clip(np.diag(np.asarray(odom_cov, float)), 0.0, None)) * cfg.odom_noise_scale
        sigma = np.hypot(sigma, cfg.odom_sigma_floor)
        d = np.asarray(odom, float) + self.rng.normal(0.0, 1.0, (len(self.p), 3)) * sigma
        c, s = np.cos(self.p[:, 2]), np.sin(self.p[:, 2])
        self.p[:, 0] += c * d[:, 0] - s * d[:, 1]
        self.p[:, 1] += s * d[:, 0] + c * d[:, 1]
        self.p[:, 2] = wrap(self.p[:, 2] + d[:, 2])

    # ---------------------------------------------------------------- update

    def update(self, obs) -> UpdateStats:
        cfg = self.cfg
        pts = np.asarray(obs.line_points, float).reshape(-1, 2)
        cov = np.asarray(obs.line_cov, float).reshape(-1, 2, 2)
        landmarks = list(getattr(obs, "landmarks", []) or [])
        if len(pts) > cfg.max_points:
            idx = self.rng.choice(len(pts), cfg.max_points, replace=False)
            pts, cov = pts[idx], cov[idx]
        n_pts, n_lm = len(pts), len(landmarks)
        if n_lm == 0 and n_pts < cfg.min_points_for_update:
            # Too little to see (head turned off the field). A few points, possibly false positives,
            # would only add noise.
            return UpdateStats(n_pts, 0, self.ess, self._q_fast, 0, False)

        c, s = np.cos(self.p[:, 2])[:, None], np.sin(self.p[:, 2])[:, None]
        loglik = np.zeros(len(self.p))
        score = np.zeros(len(self.p))
        sig_pts = None
        if n_pts:
            sig_pts = np.sqrt(_lambda_max(cov) + cfg.point_sigma_floor**2)
            wx = self.p[:, 0:1] + c * pts[:, 0] - s * pts[:, 1]
            wy = self.p[:, 1:2] + s * pts[:, 0] + c * pts[:, 1]
            d = self.lf.lookup(np.column_stack([wx.ravel(), wy.ravel()])).reshape(wx.shape)
            r2 = (d / sig_pts) ** 2
            loglik -= cfg.line_temperature * np.log1p(r2).sum(axis=1)
            score += (1.0 / (1.0 + r2)).sum(axis=1)
        for lm in landmarks:
            refs = self._lm_by_kind.get(lm.kind)
            if refs is None:
                continue
            xy = np.asarray(lm.xy, float)
            sig = math.sqrt(
                float(_lambda_max(np.asarray(lm.cov, float)[None])[0]) + cfg.landmark_sigma_floor**2
            )
            wx = self.p[:, 0] + c[:, 0] * xy[0] - s[:, 0] * xy[1]
            wy = self.p[:, 1] + s[:, 0] * xy[0] + c[:, 0] * xy[1]
            e2 = ((wx[:, None] - refs[:, 0]) ** 2 + (wy[:, None] - refs[:, 1]) ** 2).min(axis=1)
            r2 = e2 / sig**2
            loglik -= cfg.landmark_temperature * np.log1p(r2)
            score += 1.0 / (1.0 + r2)

        # Match quality in [0, 1]: weighted mean per-observation score, before reweighting.
        # A frame with little to see (for example the head turned off the field) says nothing
        # about match quality, so it doesn't count.
        quality = float(self.w @ score) / (n_pts + n_lm)
        informative = bool(n_lm) or n_pts >= cfg.min_obs_for_quality
        if informative:
            self._update_quality(quality)

        logw = np.log(np.maximum(self.w, 1e-300)) + loglik
        logw -= logw.max()
        self.w = np.exp(logw)
        self.w /= self.w.sum()

        # Recovery strength in [0, 1]: zero for normal scene-to-scene variation, rising once matches
        # clearly degrade. Applied only on informative frames, so a few frames looking off the field
        # never erode the cloud.
        drop = max(0.0, 1.0 - self._q_fast / self._q_slow) if self._q_slow > 0 else 0.0
        recovery = 0.0
        if informative and cfg.drop_threshold < 1.0:
            recovery = min(1.0, max(0.0, (drop - cfg.drop_threshold) / (1.0 - cfg.drop_threshold)))
        inject_fraction = cfg.inject_max_fraction * recovery
        if informative:
            self._degraded = drop > cfg.drop_threshold
        if self._lost:
            inject_fraction = cfg.lost_inject_fraction if informative else 0.0
            recovery = 0.0  # no roughening while lost; proposals do the searching
            if informative:
                self._update_lost_state()
        local = None
        self._drift = False
        if informative and not self._lost and n_pts:
            local = self._drift_test(pts, sig_pts)
            self._drift = local is not None
        resampled = False
        n_inject = 0
        if self.ess < cfg.resample_ess_fraction * len(self.p) or inject_fraction > 0 or self._drift:
            self._resample()
            resampled = True
            n_inject = int(round(inject_fraction * len(self.p)))
            if recovery > 0:
                # Roughening: when matches get worse, let the cloud slide so a small offset can be corrected.
                self.p[:, :2] += self.rng.normal(0.0, cfg.roughen_xy * recovery, (len(self.p), 2))
                self.p[:, 2] = wrap(
                    self.p[:, 2] + self.rng.normal(0.0, cfg.roughen_theta * recovery, len(self.p))
                )
            if n_inject:
                slots = self.rng.choice(len(self.p), n_inject, replace=False)
                self.p[slots] = self._propose(n_inject, pts, sig_pts if n_pts else None)
            if local is not None:
                slots = self.rng.choice(len(self.p), len(local), replace=False)
                self.p[slots] = local
                n_inject += len(local)
        return UpdateStats(n_pts, n_lm, self.ess, quality, n_inject, resampled, self._drift)

    def _drift_test(self, pts: np.ndarray, sig: np.ndarray) -> np.ndarray | None:
        """Is there a clearly better pose near the current estimate? If so, return the best local candidates.

        The global quality-drop trigger misses a cloud that has collapsed 0.5–1.5 m off (for example after a
        push near the touchline): matches are mediocre but not dramatically worse than the slow baseline, and
        the tight cloud creeps back at ~0.1 m/s. This test is self-calibrating: it compares the estimate with
        poses around it on the same frame, so scene content and noise level cancel out. In normal tracking the
        estimate is already the local best and nothing happens. Candidates stay near the estimate, so they
        can't move the robot to the other half.
        """
        cfg = self.cfg
        center = np.array(self.clusters(max_clusters=1)[0].pose)
        k = min(len(pts), cfg.proposal_points)
        idx = self.rng.choice(len(pts), k, replace=False)
        p, s = pts[idx], sig[idx]
        # Two scales: half near (small drifts), half wide (a 1–2 m push leaves the cloud well away from
        # the truth).
        half = cfg.drift_candidates // 2
        scale = np.empty((cfg.drift_candidates, 3))
        scale[:half] = [cfg.drift_sigma_xy, cfg.drift_sigma_xy, cfg.drift_sigma_theta]
        scale[half:] = [cfg.drift_wide_sigma_xy, cfg.drift_wide_sigma_xy, cfg.drift_wide_sigma_theta]
        cand = center + self.rng.normal(0.0, 1.0, (cfg.drift_candidates, 3)) * scale
        cand[:, 2] = wrap(cand[:, 2])
        ll = self._line_loglik(np.vstack([center[None], cand]), p, s)
        # Per-point gap, with persistence. Measured in simulation: clean tracking has a p99 of 0.4–0.7 on
        # single frames, while a cloud 0.8 m off has a median of 0.7–1.6, and it stays high frame after frame.
        self._drift_hist.append((ll[1:].max() - ll[0]) / k > cfg.drift_margin_per_point)
        self._drift_hist = self._drift_hist[-cfg.drift_window :]
        if sum(self._drift_hist) < cfg.drift_votes:
            return None
        # Keep firing on every frame while the gap persists. Recovery needs continuous local injection;
        # clearing the history made it stop-and-go and let the slower global recovery take over.
        n = max(1, int(round(cfg.drift_inject_fraction * len(self.p))))
        best = np.argsort(-ll[1:])[:n]
        return cand[best]

    def _update_lost_state(self) -> None:
        best = self.clusters(max_clusters=1)[0]
        self._lost_streak = self._lost_streak + 1 if best.weight >= self.cfg.lost_until_cluster_weight else 0
        if self._lost_streak >= self.cfg.lost_confident_streak:
            self._lost = self._own_half_only = False
            self.side_reference = best.pose  # hand the side decision over to continuity

    @property
    def lost(self) -> bool:
        return self._lost

    @property
    def drift(self) -> bool:
        """The last informative update found a clearly better pose near the estimate (estimate is off)."""
        return self._drift

    @property
    def degraded(self) -> bool:
        """Match quality has clearly dropped (for example after a kidnap). The estimate may be wrong."""
        return self._degraded

    def _line_loglik(self, poses: np.ndarray, pts: np.ndarray, sig: np.ndarray) -> np.ndarray:
        c, s = np.cos(poses[:, 2])[:, None], np.sin(poses[:, 2])[:, None]
        wx = poses[:, 0:1] + c * pts[:, 0] - s * pts[:, 1]
        wy = poses[:, 1:2] + s * pts[:, 0] + c * pts[:, 1]
        d = self.lf.lookup(np.column_stack([wx.ravel(), wy.ravel()])).reshape(wx.shape)
        return -np.log1p((d / sig) ** 2).sum(axis=1)

    def _propose(self, n: int, pts: np.ndarray, sig: np.ndarray | None) -> np.ndarray:
        """Sensor-resetting proposals: score many random (folded) poses on this frame's lines.

        Keeps the best n. Uniform random poses almost never land near the truth (about 1 in 10⁴ on an
        M-Field). Screening `proposal_factor` × n candidates on a subset of the current points makes
        recovery take seconds, not minutes.
        """
        cfg = self.cfg
        factor = cfg.lost_proposal_factor if self._lost else cfg.proposal_factor
        cand = self._fold_to_reference(self._random_poses(n * factor))
        if sig is None or len(pts) == 0:
            return cand[:n]
        k = min(len(pts), cfg.proposal_points)
        idx = self.rng.choice(len(pts), k, replace=False)
        ll = self._line_loglik(cand, pts[idx], sig[idx])
        best = np.argpartition(-ll, n - 1)[:n]
        return cand[best]

    def _update_quality(self, q: float) -> None:
        cfg = self.cfg
        if self._q_slow == 0.0:
            self._q_slow = self._q_fast = q
        else:
            self._q_slow += cfg.quality_alpha_slow * (q - self._q_slow)
            self._q_fast += cfg.quality_alpha_fast * (q - self._q_fast)

    def _resample(self) -> None:
        """Low-variance resampling. Shrinks back to n_particles once lost mode has ended."""
        n = len(self.p) if self._lost else self.cfg.n_particles
        positions = (self.rng.random() + np.arange(n)) / n
        idx = np.minimum(np.searchsorted(np.cumsum(self.w), positions), n - 1)
        self.p = self.p[idx].copy()
        self.w = np.full(n, 1.0 / n)

    @property
    def ess(self) -> float:
        return float(1.0 / np.sum(self.w**2))

    # ---------------------------------------------------------------- symmetry

    def mirror(self) -> None:
        """Flip every particle to the other half (for the side belief, LOC-06)."""
        self.p[:, :2] *= -1
        self.p[:, 2] = wrap(self.p[:, 2] + math.pi)
        if self.side_reference is not None:
            self.side_reference = mirror_pose(*self.side_reference)
        if self._last_estimate is not None:
            self._last_estimate = mirror_pose(*self._last_estimate)

    # ---------------------------------------------------------------- estimate

    def _densest_seed(self, mask: np.ndarray) -> np.ndarray:
        """Weighted mean pose of the heaviest cell among `mask`.

        Cells are cluster_radius × cluster_radius × cluster_angle.

        Seeding from a cell's total weight, not the single heaviest particle, keeps a lone injected particle
        from becoming the estimate when weights are uniform right after resampling.
        """
        cfg = self.cfg
        idx = np.flatnonzero(mask)
        p, w = self.p[idx], self.w[idx]
        keys = np.column_stack(
            [
                np.floor(p[:, 0] / cfg.cluster_radius),
                np.floor(p[:, 1] / cfg.cluster_radius),
                np.floor((p[:, 2] + math.pi) / cfg.cluster_angle),
            ]
        ).astype(np.int64)
        _, inv = np.unique(keys, axis=0, return_inverse=True)
        inv = inv.ravel()
        cell = int(np.argmax(np.bincount(inv, weights=w)))
        m = inv == cell
        wn = w[m] / w[m].sum() if w[m].sum() > 0 else np.full(int(m.sum()), 1.0 / m.sum())
        th = math.atan2(float(wn @ np.sin(p[m, 2])), float(wn @ np.cos(p[m, 2])))
        return np.array([float(wn @ p[m, 0]), float(wn @ p[m, 1]), th])

    def clusters(self, max_clusters: int = 8) -> list[Cluster]:
        cfg = self.cfg
        unassigned = np.ones(len(self.p), bool)
        out = []
        while unassigned.any() and len(out) < max_clusters:
            seed = self._densest_seed(unassigned)
            dx = np.hypot(*(self.p[:, :2] - seed[:2]).T)
            da = np.abs(wrap(self.p[:, 2] - seed[2]))
            members = unassigned & (dx < cfg.cluster_radius) & (da < cfg.cluster_angle)
            if not members.any():
                break
            unassigned &= ~members
            w = self.w[members]
            total = float(w.sum())
            if total < cfg.min_cluster_weight:
                if not out:
                    out.append(self._cluster_stats(members, total))
                break
            out.append(self._cluster_stats(members, total))
        return sorted(out, key=lambda k: -k.weight)

    def _cluster_stats(self, members: np.ndarray, total: float) -> Cluster:
        p, w = self.p[members], self.w[members]
        wn = w / w.sum() if w.sum() > 0 else np.full(len(w), 1.0 / len(w))
        x, y = float(wn @ p[:, 0]), float(wn @ p[:, 1])
        th = math.atan2(float(wn @ np.sin(p[:, 2])), float(wn @ np.cos(p[:, 2])))
        d = np.column_stack([p[:, 0] - x, p[:, 1] - y, wrap(p[:, 2] - th)])
        cov = (d * wn[:, None]).T @ d
        return Cluster((x, y, th), cov, total, int(members.sum()))

    def estimate(self, update_reference: bool = True) -> Estimate:
        """Best cluster, with hysteresis.

        Also moves side_reference when the estimate is confident and continuous.
        """
        cfg = self.cfg
        clusters = self.clusters()
        best = clusters[0]
        if self._last_estimate is not None and best.weight < cfg.lost_until_cluster_weight:
            near = [
                k
                for k in clusters
                if k.weight >= cfg.keep_previous_min_weight
                and self.pose_distance(np.array([k.pose]), self._last_estimate)[0] < cfg.reference_jump
            ]
            if near:
                best = near[0]
        self._last_estimate = best.pose

        # The side reference may only move while matches are healthy. A wrong cloud sliding
        # continuously after a kidnap must not drag the reference (and so the folding of new
        # random particles) with it.
        if (
            update_reference
            and not self._lost
            and not self._degraded
            and best.weight >= cfg.confident_cluster_weight
        ):
            self._confident_streak += 1
            continuous = (
                self.side_reference is None
                or self.pose_distance(np.array([best.pose]), self.side_reference)[0] < cfg.reference_jump
            )
            if continuous or self._confident_streak >= cfg.reference_streak:
                self.side_reference = best.pose
        else:
            self._confident_streak = 0
        return Estimate(best.pose, best.cov, best.weight, clusters)
