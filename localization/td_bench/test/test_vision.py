"""The image path: rendered frames → line-pixel detector → floor observations."""

import math

import numpy as np
import pytest
from bench_report import show
from td_bench.sources import FixedHead, default_world
from td_bench.vision import (
    D435_HALF,
    FolderFrames,
    ImageCamera,
    SyntheticFrames,
    detect_line_pixels,
    green_mask,
    horizon_row,
    image_observations,
    load_frame,
    render_field_image,
    save_frame,
    white_mask,
)
from td_field_model.sim import robot_to_world

from td_field_model import FieldModel

POSE = (-2.5, 1.0, 0.3)


@pytest.fixture(scope="module")
def fm():
    return FieldModel.builtin("hsl_m_2026")


@pytest.fixture(scope="module")
def frame(fm):
    rng = np.random.default_rng(0)
    return render_field_image(fm, D435_HALF, POSE, 0.0, 0.4, fm.likelihood_field(), rng)


def test_render_shape_and_colours(frame):
    assert frame.shape == (D435_HALF.height_px, D435_HALF.width, 3) and frame.dtype == np.uint8
    assert green_mask(frame).mean() > 0.3  # mostly grass when looking down
    assert 0.0 < white_mask(frame).mean() < 0.2  # some lines


def test_detected_points_lie_on_lines(fm, frame, record_property):
    obs = image_observations(frame, D435_HALF, 0.0, 0.4, rng=np.random.default_rng(0))
    world = robot_to_world(obs.line_points[obs.line_range_ok], POSE)
    d = fm.distance(world)
    show(record_property, "Line points detected", str(len(obs.line_points)), "> 30")
    show(record_property, "Median distance to nearest line", f"{100 * np.median(d):.1f} cm", "< 5 cm")
    show(record_property, "Points within 15 cm of a line", f"{100 * np.mean(d < 0.15):.0f}%", "> 90%")
    assert len(obs.line_points) > 30
    assert np.median(d) < 0.05 and np.mean(d < 0.15) > 0.9


def test_horizon_rows_skipped(frame):
    v = horizon_row(D435_HALF, 0.0, 0.0)  # looking level: horizon mid-image
    assert v == pytest.approx(D435_HALF.cy, abs=2)
    pts = detect_line_pixels(frame, horizon_v=150)
    assert len(pts) == 0 or pts[:, 1].min() >= 150


def test_no_lines_no_points():
    grass = np.zeros((240, 424, 3), np.uint8)
    grass[:] = (40, 130, 50)
    assert len(detect_line_pixels(grass)) == 0
    wall = np.full((240, 424, 3), 235, np.uint8)  # all white, no grass next to it → rejected
    assert len(detect_line_pixels(wall)) == 0


def test_frame_files_round_trip(tmp_path, frame):
    save_frame(tmp_path / "f000.npy", frame)
    save_frame(tmp_path / "f001.npy", frame[::-1])
    assert np.array_equal(load_frame(tmp_path / "f000.npy"), frame)
    frames = FolderFrames(tmp_path, fps=10)
    assert np.array_equal(frames.get(0.0), frame)
    assert frames.get(0.05) is None  # same frame slot: not delivered twice
    assert np.array_equal(frames.get(0.1), frame[::-1])
    assert frames.get(0.2) is None  # ran out, no loop


def test_image_camera_with_synthetic_frames(fm, record_property):
    w = default_world(fm, "still", duration=1.0, start=POSE, head_pitch=0.4)
    cam = ImageCamera(SyntheticFrames(fm, D435_HALF, w, every=2, seed=0), D435_HALF, FixedHead(0.0, 0.4))
    got = [cam.get(i / 30) for i in range(4)]
    assert got[1] is None and got[3] is None  # every 2nd tick
    assert got[0] is not None and len(got[0].line_points) > 30 and got[0].t == 0.0
    # wrong believed pitch → points land off the lines (what a bad head-angle source looks like)
    off = ImageCamera(
        SyntheticFrames(fm, D435_HALF, w, seed=0), D435_HALF, FixedHead(0.0, 0.4 + math.radians(6))
    )
    o2 = off.get(0.0)
    d_good = np.median(fm.distance(robot_to_world(got[0].line_points, POSE)))
    d_bad = np.median(fm.distance(robot_to_world(o2.line_points, POSE)))
    show(record_property, "Median line distance, correct pitch", f"{100 * d_good:.1f} cm", "(baseline)")
    show(record_property, "Median line distance, pitch 6° wrong", f"{100 * d_bad:.1f} cm", "> 2 × baseline")
    assert d_bad > 2 * d_good
