"""Evaluation tool tests (LOC-07): alignment, metrics, CSV contracts and reports, on synthetic tracks."""

import json
import math

import numpy as np
import pytest
from td_localization.eval import (
    EstimateTrack,
    EvalConfig,
    GroundTruth,
    align,
    evaluate,
    main,
    read_estimates,
    read_ground_truth,
    write_estimates,
    write_ground_truth,
    write_report,
)

DT = 1 / 30


def straight(n=300, x0=-3.0, v=0.3, theta=0.0, events=()):
    t = np.arange(n) * DT
    pose = np.column_stack([x0 + v * t, np.zeros(n), np.full(n, theta)])
    return GroundTruth(t, pose, list(events))


def est_from(gt, offset=(0.0, 0.0, 0.0), status="localized", conf=0.99, dt_shift=0.0, published=None):
    n = len(gt.t)
    st = np.array([status] * n, dtype=object)
    return EstimateTrack(
        gt.t + dt_shift,
        st,
        gt.pose + np.asarray(offset, float),
        np.full(n, conf),
        None if published is None else gt.t + dt_shift + published,
    )


# ---------------------------------------------------------------- alignment


def test_align_nearest_within_tolerance_ties_to_earlier():
    gt = GroundTruth(np.array([0.0, 1.0, 2.0, 3.0]), np.zeros((4, 3)))
    est = EstimateTrack(
        np.array([0.02, 0.98, 1.02, 2.5]), np.array(["localized"] * 4, dtype=object), np.zeros((4, 3))
    )
    idx = align(gt, est, tolerance_s=0.05)
    assert list(idx) == [0, 1, -1, -1]  # 1.0: tie between 0.98 and 1.02 → earlier; 2.0 and 3.0 too far


# ---------------------------------------------------------------- core metrics


def test_perfect_run():
    gt = straight()
    r = evaluate(gt, est_from(gt))
    m = r.metrics
    assert m["accuracy_within_threshold"] == 1.0 and m["availability"] == 1.0
    assert m["pos_p95_m"] == pytest.approx(0.0) and m["heading_p95_deg"] == pytest.approx(0.0)
    assert m["wrong_half_fraction"] == 0.0 and m["wrong_half_episodes"] == 0
    assert all(v is True for k, v in r.checks.items() if "converge" not in k)


def test_constant_offset_errors():
    gt = straight()
    m = evaluate(gt, est_from(gt, offset=(0.3, 0.4, math.radians(10)))).metrics
    assert m["pos_p50_m"] == pytest.approx(0.5)
    assert m["heading_p95_deg"] == pytest.approx(10.0)
    assert m["accuracy_within_threshold"] == 0.0  # heading over the 5° threshold


def test_missing_estimates_count_against_accuracy_not_percentiles():
    gt = straight()
    est = est_from(gt)
    est.status[::2] = "lost"
    est.pose[::2] = np.nan
    m = evaluate(gt, est).metrics
    assert m["availability"] == pytest.approx(0.5)
    assert m["accuracy_within_threshold"] == pytest.approx(0.5)
    assert m["pos_p95_m"] == pytest.approx(0.0)  # over usable samples only


def test_nothing_usable_gives_none_not_zero():
    gt = straight(n=30)
    est = est_from(gt, status="initializing")
    est.pose[:] = np.nan
    r = evaluate(gt, est)
    assert r.metrics["pos_p50_m"] is None and r.metrics["wrong_half_fraction"] is None
    assert r.checks["no wrong-half samples"] is None


# ---------------------------------------------------------------- wrong half and side confidence


def test_wrong_half_detection_and_episodes():
    gt = straight(n=300, x0=-3.0, v=0.0, theta=0.3)
    est = est_from(gt)
    mirror = np.column_stack([-gt.pose[:, 0], -gt.pose[:, 1], gt.pose[:, 2] + math.pi])
    est.pose[60:90] = mirror[60:90]  # 1 s on the wrong half
    est.pose[200:215] = mirror[200:215]  # 0.5 s again
    est.side_confidence[200:215] = 0.3  # the second time it knew it was unsure
    m = evaluate(gt, est).metrics
    assert m["wrong_half_fraction"] == pytest.approx(45 / 300)
    assert m["wrong_half_episodes"] == 2
    assert m["wrong_half_longest_s"] == pytest.approx(29 * DT)
    assert m["confidently_wrong_samples"] == 30
    assert m["side_confidence_mean_when_wrong"] == pytest.approx((30 * 0.99 + 15 * 0.3) / 45)


def test_near_center_heading_decides_the_half():
    """At the center mark a pose and its mirror share a position; only heading separates them."""
    gt = GroundTruth(np.array([0.0]), np.array([[0.05, 0.0, 0.0]]))
    right = EstimateTrack(
        np.array([0.0]), np.array(["localized"], dtype=object), np.array([[0.0, 0.0, 0.05]])
    )
    wrong = EstimateTrack(
        np.array([0.0]), np.array(["localized"], dtype=object), np.array([[0.0, 0.0, math.pi]])
    )
    assert evaluate(gt, right).metrics["wrong_half_episodes"] == 0
    assert evaluate(gt, wrong).metrics["wrong_half_episodes"] == 1


# ---------------------------------------------------------------- convergence after events


def test_convergence_per_event():
    gt = straight(n=600, events=[(0.0, "initial"), (10.0, "kidnap")])
    est = est_from(gt)
    k = int(round(10.0 / DT))
    est.pose[k : k + 45, 0] += 2.0  # 1.5 s off after the kidnap
    r = evaluate(gt, est)
    conv = {e["event"]: e["converge_s"] for e in r.events}
    assert conv["initial"] == pytest.approx(0.0)
    assert conv["kidnap"] == pytest.approx(1.5, abs=DT + 1e-9)
    assert r.checks["every event converges ≤ 10.0 s"] is True


def test_never_converging_event_is_none_and_fails():
    gt = straight(n=300, events=[(0.0, "initial"), (5.0, "fall")])
    est = est_from(gt)
    est.pose[150:, 1] += 1.0
    r = evaluate(gt, est)
    assert r.events[1]["converge_s"] is None
    assert r.checks["every event converges ≤ 10.0 s"] is False


# ---------------------------------------------------------------- latency


def test_latency_from_published():
    gt = straight(n=100)
    m = evaluate(gt, est_from(gt, published=0.012)).metrics
    assert m["latency_p50_ms"] == pytest.approx(12.0)
    assert evaluate(gt, est_from(gt)).metrics["latency_p50_ms"] is None


# ---------------------------------------------------------------- CSV contracts


def test_csv_round_trip(tmp_path):
    gt = straight(n=50, events=[(0.0, "initial"), (1.0, "fall")])
    est = est_from(gt, offset=(0.1, 0.0, 0.0), published=0.01)
    est.status[5] = "lost"
    est.pose[5] = np.nan
    write_ground_truth(tmp_path / "gt.csv", gt)
    write_estimates(tmp_path / "est.csv", est)
    gt2, est2 = read_ground_truth(tmp_path / "gt.csv"), read_estimates(tmp_path / "est.csv")
    assert np.allclose(gt2.pose, gt.pose, atol=1e-5) and [k for _, k in gt2.events] == ["initial", "fall"]
    assert est2.status[5] == "lost" and np.isnan(est2.pose[5]).all()
    a, b = evaluate(gt, est).metrics, evaluate(gt2, est2).metrics
    assert a["pos_p50_m"] == pytest.approx(b["pos_p50_m"], abs=1e-5)


@pytest.mark.parametrize(
    "header, row, message",
    [
        ("timestamp_s,status,x_m,y_m,theta_rad", "0.0,flying,1,2,0", "unknown status"),
        ("timestamp_s,status,x_m,y_m,theta_rad", "0.0,localized,,2,0", "missing x_m"),
        (
            "timestamp_s,status,x_m,y_m,theta_rad,side_confidence",
            "0.0,localized,1,2,0,1.5",
            "side_confidence",
        ),
        ("timestamp_s,status,x_m,y_m,theta_rad,published_s", "1.0,localized,1,2,0,0.5", "precedes"),
        ("timestamp_s,status,x_m", "0.0,localized,1", "required columns"),
    ],
)
def test_estimate_csv_validation(tmp_path, header, row, message):
    p = tmp_path / "e.csv"
    p.write_text(f"{header}\n{row}\n")
    with pytest.raises(ValueError, match=message):
        read_estimates(p)


def test_lost_rows_may_omit_pose(tmp_path):
    p = tmp_path / "e.csv"
    p.write_text("timestamp_s,status,x_m,y_m,theta_rad\n0.0,lost,,,\n0.1,localized,1,2,0\n")
    est = read_estimates(p)
    assert np.isnan(est.pose[0]).all() and est.status[1] == "localized"


def test_ground_truth_validation(tmp_path):
    p = tmp_path / "g.csv"
    p.write_text("timestamp_s,x_m,y_m,theta_rad\n0.1,0,0,0\n0.1,0,0,0\n")
    with pytest.raises(ValueError, match="strictly increasing"):
        read_ground_truth(p)
    p.write_text("timestamp_s,x_m,y_m,theta_rad,event\n0.0,0,0,0,teleport\n")
    with pytest.raises(ValueError, match="unknown event"):
        read_ground_truth(p)


# ---------------------------------------------------------------- reports and CLI


def test_report_files(tmp_path):
    gt = straight(n=60, events=[(0.0, "initial")])
    results = [evaluate(gt, est_from(gt), "good"), evaluate(gt, est_from(gt, offset=(1.0, 0, 0)), "bad")]
    md = write_report(results, tmp_path, notes="synthetic")
    text = md.read_text(encoding="utf-8")
    assert "| Metric | good | bad |" in text and "FAIL" in text and "PASS" in text and "synthetic" in text
    data = json.loads((tmp_path / "results.json").read_text(encoding="utf-8"))
    assert data["bad"]["metrics"]["pos_p50_m"] == pytest.approx(1.0)
    assert (tmp_path / "samples_good.csv").exists()


def test_cli_evaluate(tmp_path):
    gt = straight(n=60)
    write_ground_truth(tmp_path / "gt.csv", gt)
    write_estimates(tmp_path / "a.csv", est_from(gt))
    write_estimates(tmp_path / "b.csv", est_from(gt, offset=(0.2, 0, 0)))
    rc = main(
        ["evaluate", "--gt", str(tmp_path / "gt.csv"), "--est", str(tmp_path / "a.csv"),
         "--est", str(tmp_path / "b.csv"), "--out", str(tmp_path / "out")]
    )  # fmt: skip
    assert rc == 0
    data = json.loads((tmp_path / "out" / "results.json").read_text(encoding="utf-8"))
    assert set(data) == {"a", "b"}


def test_config_thresholds_are_used():
    gt = straight()
    est = est_from(gt, offset=(0.3, 0.0, 0.0))
    assert evaluate(gt, est, cfg=EvalConfig(pos_threshold_m=0.2)).metrics["accuracy_within_threshold"] == 0.0
    assert evaluate(gt, est, cfg=EvalConfig(pos_threshold_m=0.5)).metrics["accuracy_within_threshold"] == 1.0
