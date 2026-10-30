"""Image path for the bench: frames → field-line pixels → floor points → Observations. numpy only.

This is a deliberately simple CLASSICAL detector so localization can be tested with a real camera before the
learned
detector (LOC-17) exists:
- line pixel = bright and unsaturated (white), with field green just above or below it within `edge_px`;
- pixels above the horizon of the believed camera pose are ignored (a white wall isn't on the floor).

It has no crossing/landmark classification and no goalposts, and it has a hard time in odd lighting. The point
is a
working end-to-end path, not competition perception.

Frame sources:
- SyntheticFrames: renders a field image from the simulated world (exercises the whole image path with no
camera).
- FolderFrames: a folder of frames recorded on the bench (`.npy` H×W×3 uint8; `.png`/`.jpg` if Pillow is
installed).
- RealSenseFrames: a live D435 color stream via pyrealsense2 (optional import). The IR projector is switched
OFF
  (team policy, docs/roadmap.md §2.2). NOT TESTED here: no camera on this machine.

The camera model must match the frames: use `CameraModel(fx, fy, cx, cy, width, height_px, mount_height)` from
the camera's real intrinsics and height.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
from td_field_model.sim import (
    CameraModel,
    Observations,
    backproject_pixels,
    floor_covariances,
    robot_to_world,
)

from td_field_model import FieldModel

D435_HALF = CameraModel(fx=307.5, fy=307.5, cx=212.0, cy=120.0, width=424, height_px=240)  # 848×480 halved

# ---------------------------------------------------------------- rendering (synthetic camera)


def render_field_image(
    fm: FieldModel,
    cam: CameraModel,
    pose,
    head_yaw: float,
    head_pitch: float,
    lf=None,
    rng=None,
    noise_level: float = 6.0,
    green=(40, 130, 50),
    white=(235, 235, 235),
    outside=(110, 110, 110),
    sky=(170, 190, 210),
) -> np.ndarray:
    """H×W×3 uint8 image of the field from a camera at `pose` (field frame) with the given neck angles."""
    lf = lf or fm.likelihood_field()
    h, w = cam.height_px, cam.width
    vv, uu = np.mgrid[0:h, 0:w]
    uv = np.column_stack([uu.ravel() + 0.5, vv.ravel() + 0.5])
    r_bc, t_bc = cam.pose_in_base(head_yaw, head_pitch)
    pts_robot, hit = backproject_pixels(cam, uv, r_bc, t_bc)
    pts_world = robot_to_world(pts_robot, pose)
    img = np.empty((h * w, 3), np.float32)
    img[:] = sky
    a = fm.dims.length / 2 + fm.dims.border_strip_width
    b = fm.dims.width / 2 + fm.dims.border_strip_width
    on_carpet = hit & (np.abs(pts_world[:, 0]) <= a) & (np.abs(pts_world[:, 1]) <= b)
    img[hit & ~on_carpet] = outside
    img[on_carpet] = green
    dist = lf.lookup(pts_world[on_carpet])
    line = np.zeros(h * w, bool)
    line[np.flatnonzero(on_carpet)[dist <= fm.dims.line_width / 2]] = True
    img[line] = white
    if noise_level:
        rng = rng or np.random.default_rng(0)
        img += rng.normal(0, noise_level, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8).reshape(h, w, 3)


# ---------------------------------------------------------------- classical line detector


def white_mask(img: np.ndarray, min_value: int = 175, max_spread: int = 45) -> np.ndarray:
    c = img.astype(np.int16)
    return (c.min(axis=2) >= min_value) & (c.max(axis=2) - c.min(axis=2) <= max_spread)


def green_mask(img: np.ndarray, margin: int = 20) -> np.ndarray:
    c = img.astype(np.int16)
    r, g, b = c[..., 0], c[..., 1], c[..., 2]
    return (g > r + margin) & (g > b + margin)


def detect_line_pixels(
    img: np.ndarray,
    step: int = 3,
    edge_px: int = 8,
    max_points: int = 300,
    horizon_v: float | None = None,
    rng=None,
) -> np.ndarray:
    """(N, 2) pixel coordinates (u, v) of field-line pixels, sampled every `step` px.

    A white pixel counts only if green is within `edge_px` above or below it (lines lie on grass), which
    rejects
    white jerseys against a non-green background, walls and sky. If `horizon_v` is given, rows above it are
    skipped.
    """
    white = white_mask(img)
    green = green_mask(img)
    near_green = np.zeros_like(green)
    for k in range(1, edge_px + 1):
        near_green[k:] |= green[:-k]
        near_green[:-k] |= green[k:]
    cand = white & near_green
    sub = np.zeros_like(cand)
    sub[::step, ::step] = cand[::step, ::step]
    if horizon_v is not None:
        sub[: max(0, int(math.ceil(horizon_v)))] = False
    vs, us = np.nonzero(sub)
    uv = np.column_stack([us + 0.5, vs + 0.5]).astype(float)
    if len(uv) > max_points:
        rng = rng or np.random.default_rng(0)
        uv = uv[rng.choice(len(uv), max_points, replace=False)]
    return uv


def horizon_row(cam: CameraModel, head_yaw: float, head_pitch: float) -> float:
    """Image row of the horizon (rays at and above it never hit the floor)."""
    r_bc, _ = cam.pose_in_base(head_yaw, head_pitch)
    # The ray through the optical center at row v has base-frame z component ∝ r[2,1]·(v-cy)/fy + r[2,2].
    a, c = r_bc[2, 1], r_bc[2, 2]
    if abs(a) < 1e-9:
        return 0.0
    return float(cam.cy - c * cam.fy / a)


def image_observations(
    img: np.ndarray,
    cam: CameraModel,
    head_yaw: float,
    head_pitch: float,
    t: float = 0.0,
    pixel_sigma: float = 1.5,
    angle_sigma: float = math.radians(0.5),
    bearing_only_beyond: float = 4.0,
    **detect_kw,
) -> Observations:
    """Detect line pixels and project them to the floor with the believed head angles → Observations."""
    r_bc, t_bc = cam.pose_in_base(head_yaw, head_pitch)
    uv = detect_line_pixels(img, horizon_v=horizon_row(cam, head_yaw, head_pitch) + 2, **detect_kw)
    pts, hit = backproject_pixels(cam, uv, r_bc, t_bc)
    pts = pts[hit]
    rel = pts - t_bc[:2]
    keep = np.hypot(rel[:, 0], rel[:, 1]) <= cam.max_range
    pts = pts[keep]
    cov = floor_covariances(cam, pts, t_bc, pixel_sigma, angle_sigma, angle_sigma)
    range_ok = np.hypot(*(pts - t_bc[:2]).T) <= bearing_only_beyond if len(pts) else np.zeros(0, bool)
    return Observations(t, pts, cov, range_ok)


class ImageCamera:
    """Observation source from frames: frame source → classical detector → floor projection.

    `head` supplies the BELIEVED neck angles (encoders, a fixed tripod angle, or simulation).
    """

    def __init__(self, frames, cam: CameraModel, head, **obs_kw):
        self.frames, self.cam, self.head, self.obs_kw = frames, cam, head, obs_kw
        self.frames_used = 0

    def get(self, t: float) -> Observations | None:
        img = self.frames.get(t)
        if img is None:
            return None
        self.frames_used += 1
        yaw, pitch = self.head.get(t)
        return image_observations(img, self.cam, yaw, pitch, t=t, **self.obs_kw)


# ---------------------------------------------------------------- frame sources


class SyntheticFrames:
    """Rendered frames from a simulated World (td_bench.sources.World), every `every` ticks."""

    def __init__(
        self, fm: FieldModel, cam: CameraModel, world, every: int = 2, seed: int = 0, noise_level: float = 6.0
    ):
        self.fm, self.cam, self.world, self.every = fm, cam, world, every
        self.lf = fm.likelihood_field()
        self.rng = np.random.default_rng(seed)
        self.noise_level = noise_level

    def get(self, t: float) -> np.ndarray | None:
        i = self.world.index(t)
        if i % self.every:
            return None
        s = self.world.steps[i]
        return render_field_image(
            self.fm, self.cam, s.true_pose, s.head_yaw, s.head_pitch, self.lf, self.rng, self.noise_level
        )


class FolderFrames:
    """Frames from a folder, played back at `fps` (sorted by name).

    `.npy` always works; `.png`/`.jpg` need Pillow.

    Record bench frames as `frame_000123.npy` (H×W×3 uint8, RGB) with `save_frame`.
    """

    def __init__(self, folder, fps: float = 15.0, loop: bool = False):
        self.files = sorted(
            p for p in Path(folder).iterdir() if p.suffix.lower() in (".npy", ".png", ".jpg", ".jpeg")
        )
        if not self.files:
            raise FileNotFoundError(f"no frames in {folder}")
        self.fps, self.loop = fps, loop
        self._last = -1

    def get(self, t: float) -> np.ndarray | None:
        k = int(t * self.fps)
        if self.loop:
            k %= len(self.files)
        if k == self._last or k >= len(self.files):
            return None
        self._last = k
        return load_frame(self.files[k])


def save_frame(path, img: np.ndarray) -> None:
    np.save(path, np.asarray(img, np.uint8))


def load_frame(path) -> np.ndarray:
    path = Path(path)
    if path.suffix.lower() == ".npy":
        return np.load(path)
    try:
        from PIL import Image  # optional
    except ImportError as e:
        raise ImportError("loading .png/.jpg frames needs Pillow; save frames as .npy instead") from e
    return np.asarray(Image.open(path).convert("RGB"))


class RealSenseFrames:  # pragma: no cover - needs a D435 and pyrealsense2
    """Live D435 color frames (RGB) via pyrealsense2. The IR projector is turned OFF (team policy).

    Untested on this machine. Use the real intrinsics with `camera_model()`.
    """

    def __init__(self, width: int = 848, height: int = 480, fps: int = 30):
        import pyrealsense2 as rs  # optional dependency

        self.rs = rs
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, width, height, rs.format.rgb8, fps)
        self.profile = self.pipe.start(cfg)
        depth = self.profile.get_device().first_depth_sensor()
        if depth.supports(rs.option.emitter_enabled):
            depth.set_option(rs.option.emitter_enabled, 0)
            if depth.get_option(rs.option.emitter_enabled) != 0:
                self.pipe.stop()
                raise RuntimeError("D435 projector could not be turned off; refusing to run (team IR policy)")

    def camera_model(self, mount_height: float) -> CameraModel:
        intr = self.profile.get_stream(self.rs.stream.color).as_video_stream_profile().get_intrinsics()
        return CameraModel(intr.fx, intr.fy, intr.ppx, intr.ppy, intr.width, intr.height, mount_height)

    def get(self, t: float) -> np.ndarray | None:
        frames = self.pipe.poll_for_frames()
        if not frames:
            return None
        color = frames.get_color_frame()
        return None if not color else np.asanyarray(color.get_data()).copy()

    def close(self) -> None:
        self.pipe.stop()
