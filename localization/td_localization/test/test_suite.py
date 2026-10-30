"""Scenario-suite tests (LOC-07): scenario construction and an end-to-end smoke run."""

import json
import math

import numpy as np
from td_field_model.scenarios import ScenarioBuilder
from td_localization.suite import kidnap_same_half, kidnap_to_mirror, run_suite, scenarios

from td_field_model import FieldModel, mirror_pose


def _pose_dist(a, b, heading_scale=2.0):
    dth = math.atan2(math.sin(a[2] - b[2]), math.cos(a[2] - b[2]))
    return math.hypot(math.hypot(a[0] - b[0], a[1] - b[1]), heading_scale * dth)


def test_kidnap_to_mirror_makes_continuity_point_the_wrong_way():
    fm = FieldModel.builtin("hsl_m_2026")
    for seed in range(1, 6):
        sb = ScenarioBuilder(fm, seed=seed).start((-2.0, -1.0, 0.0)).walk(10.0)
        before = sb.pose
        after = kidnap_to_mirror(sb).pose
        assert _pose_dist(mirror_pose(*after), before) < _pose_dist(after, before)


def test_kidnap_same_half_keeps_continuity():
    fm = FieldModel.builtin("hsl_m_2026")
    for seed in range(1, 6):
        sb = ScenarioBuilder(fm, seed=seed).start((-2.0, -1.0, 0.0)).walk(10.0)
        before = sb.pose
        after = kidnap_same_half(sb).pose
        assert _pose_dist(after, before) < _pose_dist(mirror_pose(*after), before)


def test_standard_scenarios_have_expected_events():
    sc = scenarios(FieldModel.builtin("hsl_m_2026"), seed=1)
    kinds = {name: [s.event for s in d["steps"] if s.event] for name, d in sc.items() if "steps" in d}
    team = sc["team_kidnap_cross"]["team_specs"]
    assert [r.player for r in team] == [1, 2, 3] and team[0].is_goalkeeper
    assert [s.event for s in team[2].steps if s.event] == ["initial", "kidnap"]
    assert kinds["falls"] == ["initial", "fall", "fall"]
    assert kinds["kidnap_cross_team"] == ["initial", "kidnap"]
    assert sc["global_sideline"]["steps"][0].known_poses == []  # unknown start: own-half search
    assert len(sc["kidnap_cross_team"]["team"]) == 3 and not sc["kidnap_cross_alone"]["team"]


def test_suite_smoke(tmp_path):
    md = run_suite(tmp_path, only=["penalty_reentry", "falls"])
    text = md.read_text(encoding="utf-8")
    assert "penalty_reentry" in text and "falls" in text and "tracking" not in text
    data = json.loads((tmp_path / "results.json").read_text(encoding="utf-8"))
    for name in ("penalty_reentry", "falls"):
        assert all(v is True for v in data[name]["checks"].values()), (name, data[name]["checks"])
        assert (tmp_path / f"gt_{name}.csv").exists() and (tmp_path / f"est_{name}.csv").exists()
    assert np.isfinite(data["falls"]["metrics"]["latency_p50_ms"])
