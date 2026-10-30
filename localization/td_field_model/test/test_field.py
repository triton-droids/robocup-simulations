import math
import time

import numpy as np
import pytest
from td_field_model.field import (
    CENTER_MARK,
    FIELDS_DIR,
    GOALPOST,
    L_CROSSING,
    PENALTY_MARK,
    T_CROSSING,
    X_CROSSING,
)

from td_field_model import FieldModel, LikelihoodField, mirror_points, mirror_pose

ALL = ["hsl_s_2026", "hsl_m_2026", "hsl_l_2026"]


def src_dir():
    return FIELDS_DIR


@pytest.fixture(scope="module", params=ALL)
def field(request) -> FieldModel:
    return FieldModel.builtin(request.param)


@pytest.fixture(scope="module")
def m_field() -> FieldModel:
    return FieldModel.builtin("hsl_m_2026")


@pytest.fixture(scope="module")
def m_lf(m_field) -> LikelihoodField:
    return m_field.likelihood_field(resolution=0.02)


# ---------------------------------------------------------------- dimensions vs the rule book table


@pytest.mark.parametrize(
    "name, outer_length, outer_width, penalty_area, goal_area, circle",
    [
        ("hsl_s_2026", 9.0, 6.0, (2.0, 4.0), (1.0, 3.0), 1.5),
        ("hsl_m_2026", 14.0, 9.0, (3.0, 6.0), (1.0, 4.0), 3.0),
    ],
)
def test_s_and_m_match_rule_table(name, outer_length, outer_width, penalty_area, goal_area, circle):
    d = FieldModel.builtin(name).dims
    lw = d.line_width
    assert d.outer_length == pytest.approx(outer_length)
    assert d.outer_width == pytest.approx(outer_width)
    assert (d.penalty_area_length + lw, d.penalty_area_width + lw) == pytest.approx(penalty_area)
    assert (d.goal_area_length + lw, d.goal_area_width + lw) == pytest.approx(goal_area)
    assert d.center_circle_diameter + lw == pytest.approx(circle)


def test_l_field_inner_dimensions_match_rule_table():
    d = FieldModel.builtin("hsl_l_2026").dims
    lw = d.line_width
    assert lw == pytest.approx(0.12)
    assert (d.penalty_area_length + lw, d.penalty_area_width + lw) == pytest.approx((3.5, 7.0))
    assert (d.goal_area_length + lw, d.goal_area_width + lw) == pytest.approx((1.0, 5.0))
    assert d.center_circle_diameter + lw == pytest.approx(4.0)
    assert d.corner_arc_radius + lw / 2 == pytest.approx(1.0)


def test_l_field_length_width_discrepancy_is_pinned():
    """Upstream L JSON gives 21.95 x 13.95 between line centers → 22.07 x 14.07 outside, not 22 x 14.

    Pinned on purpose (fields/SOURCE.md). If this fails, the upstream file changed:
    re-vendor it and update SOURCE.md.
    """
    d = FieldModel.builtin("hsl_l_2026").dims
    assert (d.length, d.width) == pytest.approx((21.95, 13.95))
    assert (d.outer_length, d.outer_width) == pytest.approx((22.07, 14.07))


def test_unknown_builtin_raises():
    with pytest.raises(KeyError):
        FieldModel.builtin("hsl_xl_2026")


def test_from_json_round_trip(tmp_path):
    src = FieldModel.builtin("hsl_m_2026")
    p = tmp_path / "venue.json"
    p.write_text((src_dir() / "field_hsl_m_2026.json").read_text())
    venue = FieldModel.from_json(p)
    assert venue.name == "venue"
    assert venue.dims.length == src.dims.length
    assert len(venue.segments) == len(src.segments)


# ---------------------------------------------------------------- geometry and landmarks


def test_line_counts(field):
    has_goal_area = field.dims.goal_area_length is not None
    # 2 touchlines + halfway + 2 goal lines + 3 per penalty area (+3 per goal area), per side.
    assert len(field.segments) == 5 + 2 * 3 * (2 if has_goal_area else 1)
    assert len(field.arcs) == 1 + (4 if field.dims.corner_arc_radius is not None else 0)


def test_s_field_has_no_corner_arcs():
    assert len(FieldModel.builtin("hsl_s_2026").arcs) == 1


def test_crossings_and_marks_lie_on_lines(field):
    on_lines = [lm for lm in field.landmarks if lm.kind in (L_CROSSING, T_CROSSING, X_CROSSING, CENTER_MARK)]
    d = field.distance(np.array([lm.xy for lm in on_lines]))
    assert np.all(d < 1e-9), [lm.name for lm, v in zip(on_lines, d, strict=True) if v >= 1e-9]


def test_penalty_marks_are_off_lines_and_in_penalty_area(field):
    d = field.dims
    for lm in field.landmarks_of(PENALTY_MARK):
        x, y = lm.xy
        assert y == 0.0
        assert d.length / 2 - d.penalty_area_length < abs(x) < d.length / 2
        assert field.distance(lm.xy)[0] > 0.3


def test_landmark_counts(field):
    boxes = 2 if field.dims.goal_area_length is not None else 1
    assert len(field.landmarks_of(CENTER_MARK)) == 1
    assert len(field.landmarks_of(X_CROSSING)) == 2
    assert len(field.landmarks_of(T_CROSSING)) == 2 + 2 * 2 * boxes
    assert len(field.landmarks_of(L_CROSSING)) == 4 + 2 * 2 * boxes
    assert len(field.landmarks_of(PENALTY_MARK)) == 2
    assert len(field.landmarks_of(GOALPOST)) == 4


def test_goalposts_frame_the_goal(field):
    d = field.dims
    posts = np.array([lm.xy for lm in field.landmarks_of(GOALPOST)])
    assert np.allclose(np.abs(posts[:, 0]), d.length / 2)
    gap = 2 * np.abs(posts[:, 1]) - d.goal_post_diameter  # inner edge to inner edge
    assert np.allclose(gap, d.goal_inner_width)


def test_own_goal_is_negative_x(field):
    own = [lm for lm in field.landmarks if lm.name.startswith("own_")]
    opp = [lm for lm in field.landmarks if lm.name.startswith("opp_")]
    assert own and all(lm.xy[0] < 0 for lm in own)
    assert len(opp) == len(own) and all(lm.xy[0] > 0 for lm in opp)


def test_corner_arcs_are_inside_the_field():
    f = FieldModel.builtin("hsl_m_2026")
    for arc in f.arcs[1:]:
        mid = arc.start + arc.sweep / 2
        p = (arc.center[0] + arc.radius * math.cos(mid), arc.center[1] + arc.radius * math.sin(mid))
        assert f.in_bounds(p)[0]


# ---------------------------------------------------------------- distance


def test_distance_spot_values(m_field):
    a = m_field.dims.length / 2
    r = m_field.dims.center_circle_diameter / 2
    pts = np.array([[0.0, 0.0], [1.0, 0.5], [a + 0.5, 0.0], [-2.0, 0.0]])
    expected = [
        0.0,  # center mark on the halfway line
        min(1.0, abs(math.hypot(1.0, 0.5) - r)),  # halfway line vs center circle
        0.5,  # behind the opponent goal line
        min(2.0 - r, a - m_field.dims.penalty_area_length - 2.0),  # circle vs own penalty-area front
    ]
    assert m_field.distance(pts) == pytest.approx(expected, abs=1e-9)


def test_distance_accepts_single_point(m_field):
    assert m_field.distance((0.0, 0.0)).shape == (1,)


def test_sampled_line_points_are_on_lines(field):
    pts = field.sample_lines(spacing=0.1)
    assert len(pts) > 100
    assert np.max(field.distance(pts)) < 1e-9


def test_partial_arc_distance_uses_endpoints_outside_the_sweep():
    f = FieldModel.builtin("hsl_m_2026")
    corner = next(arc for arc in f.arcs if arc.name == "corner_arc_opp_left")
    # A point on the arc's full circle but outside the quarter sweep must not read as distance 0.
    outside = (corner.center[0] + corner.radius, corner.center[1])  # pointing +x, away from the field
    assert f.distance(outside)[0] > 0.1


# ---------------------------------------------------------------- symmetry


def test_mirror_pose_is_involution():
    rng = np.random.default_rng(0)
    for x, y, th in rng.uniform([-7, -4.5, -math.pi], [7, 4.5, math.pi], size=(50, 3)):
        mx, my, mth = mirror_pose(*mirror_pose(x, y, th))
        assert (mx, my) == pytest.approx((x, y))
        assert math.cos(mth - th) == pytest.approx(1.0)


def test_field_lines_are_point_symmetric(field):
    pts = field.sample_lines(spacing=0.1)
    assert np.max(field.distance(mirror_points(pts))) < 1e-9


def test_landmarks_map_onto_same_kind_under_mirror(field):
    for lm in field.landmarks:
        mx, my = mirror_points(np.array(lm.xy))
        assert any(
            o.kind == lm.kind and math.isclose(o.xy[0], mx) and math.isclose(o.xy[1], my)
            for o in field.landmarks
        ), lm.name


# ---------------------------------------------------------------- likelihood field


def test_likelihood_field_covers_border_strip(m_field, m_lf):
    x0, x1, y0, y1 = m_lf.extent
    d = m_field.dims
    assert x0 <= -(d.length / 2 + d.border_strip_width) + 1e-9
    assert x1 >= d.length / 2 + d.border_strip_width - 1e-9
    assert y0 <= -(d.width / 2 + d.border_strip_width) + 1e-9
    assert y1 >= d.width / 2 + d.border_strip_width - 1e-9


def test_likelihood_field_matches_exact_distance(m_field, m_lf):
    rng = np.random.default_rng(1)
    x0, x1, y0, y1 = m_lf.extent
    pts = rng.uniform([x0, y0], [x1, y1], size=(5000, 2))
    err = np.abs(m_lf.lookup(pts) - m_field.distance(pts))
    # Bilinear interpolation of a distance field is exact to about one grid cell.
    assert np.max(err) <= m_lf.resolution


def test_likelihood_field_outside_value(m_lf):
    far = np.array([[100.0, 0.0], [0.0, -100.0]])
    assert np.all(m_lf.lookup(far) == m_lf.outside)
    assert np.all(m_lf.lookup(far, outside=7.0) == 7.0)


def test_likelihood_field_save_load(tmp_path, m_lf):
    path = tmp_path / "m.npz"
    m_lf.save(path)
    back = LikelihoodField.load(path)
    assert back.origin == pytest.approx(m_lf.origin)
    assert back.resolution == m_lf.resolution
    assert np.array_equal(back.grid, m_lf.grid)


def test_likelihood_field_build_time_l_field():
    f = FieldModel.builtin("hsl_l_2026")
    t0 = time.perf_counter()
    lf = f.likelihood_field(resolution=0.02)
    elapsed = time.perf_counter() - t0
    d = f.dims
    nx = math.ceil((d.length + 2 * d.border_strip_width) / 0.02 - 1e-9) + 1
    ny = math.ceil((d.width + 2 * d.border_strip_width) / 0.02 - 1e-9) + 1
    assert lf.grid.shape == (nx, ny)
    x0, x1, y0, y1 = lf.extent
    assert x1 >= d.length / 2 + d.border_strip_width - 1e-9
    assert y1 >= d.width / 2 + d.border_strip_width - 1e-9
    # About 2.5 s on the dev laptop for ~1 M cells. Build once and cache with save()/load().
    # Loose bound so slower CI boxes don't flake.
    assert elapsed < 10.0


def test_lookup_is_vectorized_and_fast(m_lf):
    pts = np.random.default_rng(2).uniform(-7, 7, size=(100_000, 2))
    t0 = time.perf_counter()
    m_lf.lookup(pts)
    assert time.perf_counter() - t0 < 0.5  # 500 particles × 200 points per filter update
