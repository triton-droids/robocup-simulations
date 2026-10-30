"""Run every localization test suite and write test_report/index.html: each test beside its result.

    cd localization
    python test_report/make_results.py

- td_field_model, td_game and td_localization run as subprocesses (in parallel), with JUnit XML in test_report/raw/.
- td_bench runs in-process so every bench run the tests make can be captured and plotted (estimate vs ground
  truth). Its tests log measured values next to their pass conditions (td_bench/test/bench_report.py).

Standard library + numpy only (no matplotlib): plots are inline SVG, images are PNG data URIs.
"""
# ruff: noqa: E501  (long lines are HTML/CSS templates)

from __future__ import annotations

import ast
import base64
import datetime as dt
import html
import math
import os
import platform
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import zlib
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]  # localization/
OUT = Path(__file__).resolve().parent
RAW = OUT / "raw"
PATHS = {
    "td_field_model": ["td_field_model"],
    "td_game": ["td_game", "td_game/test"],
    "td_localization": ["td_field_model", "td_game", "td_localization", "td_localization/test"],
    "td_bench": ["td_field_model", "td_game", "td_localization", "td_bench"],
}
ORDER = ["td_bench", "td_field_model", "td_localization", "td_game"]
BLURB = {
    "td_bench": "Bench harness: localization driven by any mix of real, recorded and simulated inputs (LOC-25).",
    "td_field_model": "Field model and synthetic simulator (LOC-03, LOC-04).",
    "td_localization": "Particle filter, side belief, evaluation, game bridge, team simulation (LOC-05 to 08, 21).",
    "td_game": "GameController client, team messages, budget pacer, UDP links (LOC-08, LOC-21).",
}


def pytest_args(pkg: str) -> list[str]:
    xml = RAW / f"{pkg}.xml"
    return [f"{pkg}/test", f"--rootdir={ROOT}", f"--junitxml={xml}", "-o", "junit_family=xunit1", "-q",
            "-p", "no:cacheprovider"]  # fmt: skip


# ---------------------------------------------------------------- running


class Capture:
    """pytest plugin: remembers which test is running so captured bench runs can be attributed to it."""

    def __init__(self):
        self.current = None
        self.runs: dict[str, list] = defaultdict(list)

    def pytest_runtest_setup(self, item):
        self.current = item.nodeid


def run_all() -> tuple[dict, Capture, dict]:
    RAW.mkdir(parents=True, exist_ok=True)
    procs, t0 = {}, time.perf_counter()
    for pkg in ("td_localization", "td_field_model", "td_game"):
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(str(ROOT / p) for p in PATHS[pkg]))
        log = open(RAW / f"{pkg}.log", "w", encoding="utf-8")  # noqa: SIM115
        procs[pkg] = (subprocess.Popen([sys.executable, "-m", "pytest", *pytest_args(pkg)], cwd=ROOT, env=env,
                                       stdout=log, stderr=subprocess.STDOUT), log, time.perf_counter())  # fmt: skip
    wall = {}

    # td_bench in-process, with Bench.run wrapped to keep every run's result
    for p in PATHS["td_bench"]:
        sys.path.insert(0, str(ROOT / p))
    import pytest
    from td_bench.bench import Bench

    cap = Capture()
    orig = Bench.run

    def run(self, *a, **k):
        res = orig(self, *a, **k)
        cap.runs[cap.current].append((self, res))
        return res

    Bench.run = run
    os.chdir(ROOT)
    tb = time.perf_counter()
    pytest.main(pytest_args("td_bench"), plugins=[cap])
    wall["td_bench"] = time.perf_counter() - tb
    Bench.run = orig

    while len(wall) < 1 + len(procs):  # poll so each package gets its own finish time
        for pkg, (p, log, ts) in procs.items():
            if pkg not in wall and p.poll() is not None:
                log.close()
                wall[pkg] = time.perf_counter() - ts
        time.sleep(0.2)
    wall["total"] = time.perf_counter() - t0
    for pkg in ORDER:  # pytest's own suite time (the poll loop above only notices exits after td_bench)
        suite = ET.parse(RAW / f"{pkg}.xml").getroot()
        suite = suite if suite.tag == "testsuite" else suite.find("testsuite")
        wall[pkg] = float(suite.get("time", wall[pkg]))
    return {pkg: parse_junit(RAW / f"{pkg}.xml") for pkg in ORDER}, cap, wall


def parse_junit(path: Path) -> list[dict]:
    out = []
    for tc in ET.parse(path).getroot().iter("testcase"):
        status, msg = "passed", ""
        for tag in ("failure", "error", "skipped"):
            el = tc.find(tag)
            if el is not None:
                status = {"failure": "failed", "error": "error", "skipped": "skipped"}[tag]
                msg = (el.get("message") or "") + "\n" + (el.text or "")
        props = [(p.get("name"), p.get("value")) for p in tc.iter("property")]
        out.append({"classname": tc.get("classname"), "name": tc.get("name"), "time": float(tc.get("time", 0)),
                    "status": status, "message": msg.strip(), "props": props})  # fmt: skip
    return out


# ---------------------------------------------------------------- test sources


_ast_cache: dict[Path, tuple[str, ast.Module]] = {}


def test_source(classname: str, name: str) -> tuple[str, str, str]:
    """(file label, docstring, source) of a test function."""
    parts = classname.split(".")
    for k in range(len(parts), 0, -1):
        f = ROOT.joinpath(*parts[:k]).with_suffix(".py")
        if f.exists():
            break
    else:
        return "?", "", ""
    if f not in _ast_cache:
        src = f.read_text(encoding="utf-8")
        _ast_cache[f] = (src, ast.parse(src))
    src, tree = _ast_cache[f]
    fname = name.split("[")[0]
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == fname:
            seg = ast.get_source_segment(src, node) or ""
            return f.relative_to(ROOT).as_posix(), ast.get_docstring(node) or "", seg
    return f.relative_to(ROOT).as_posix(), "", ""


def assertions(src: str) -> list[str]:
    """The assert statements of a test (joined over continuation lines)."""
    out = []
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return []
    for node in ast.walk(tree):
        if isinstance(node, ast.Assert):
            out.append((node.lineno, " ".join(ast.get_source_segment(src, node).split())))
        elif isinstance(node, ast.With):
            ctx = ast.get_source_segment(src, node.items[0].context_expr) or ""
            if "raises" in ctx:
                out.append((node.lineno, "with " + ctx))
    return [s for _, s in sorted(out)]


# ---------------------------------------------------------------- images and plots


def png_uri(img: np.ndarray) -> str:
    img = np.ascontiguousarray(img, dtype=np.uint8)
    h, w = img.shape[:2]
    raw = b"".join(b"\x00" + img[y].tobytes() for y in range(h))

    def chunk(t: bytes, d: bytes) -> bytes:
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))  # fmt: skip
    return "data:image/png;base64," + base64.b64encode(png).decode()


class FieldPic:
    """Top view of the field as a PNG, placed in an SVG whose user units are field metres (y up)."""

    RES = 0.03  # m per pixel

    def __init__(self, fm):
        d = fm.dims
        self.hx, self.hy = d.length / 2 + d.border_strip_width, d.width / 2 + d.border_strip_width
        xs = np.arange(-self.hx, self.hx, self.RES) + self.RES / 2
        ys = np.arange(self.hy, -self.hy, -self.RES) - self.RES / 2
        gx, gy = np.meshgrid(xs, ys)
        dist = fm.distance(np.column_stack([gx.ravel(), gy.ravel()])).reshape(gx.shape)
        img = np.zeros(gx.shape + (3,), np.uint8)
        img[:] = (46, 125, 60)
        inside = (np.abs(gx) <= d.length / 2) & (np.abs(gy) <= d.width / 2)
        img[~inside] = (40, 110, 52)
        img[dist <= d.line_width / 2 + self.RES / 2] = (240, 240, 240)
        own = (
            (gx < -d.length / 2) & (np.abs(gy) < d.goal_inner_width / 2) & (gx > -d.length / 2 - d.goal_depth)
        )
        img[own] = (70, 120, 200)  # own goal (blue), the side the robot defends
        opp = (gx > d.length / 2) & (np.abs(gy) < d.goal_inner_width / 2) & (gx < d.length / 2 + d.goal_depth)
        img[opp] = (200, 90, 70)
        self.uri = png_uri(img)

    def svg(self, layers: str, view=None, width: int = 560, label: str = "") -> str:
        x0, y0, x1, y1 = view or (-self.hx, -self.hy, self.hx, self.hy)
        vb = f"{x0:.3f} {-y1:.3f} {x1 - x0:.3f} {y1 - y0:.3f}"
        img = (f'<image href="{self.uri}" x="{-self.hx}" y="{-self.hy}" width="{2 * self.hx}" '
               f'height="{2 * self.hy}" preserveAspectRatio="none" style="image-rendering:pixelated"/>')  # fmt: skip
        return (f'<svg class="field" viewBox="{vb}" width="{width}" role="img" aria-label="{html.escape(label)}">'
                f"{img}{layers}</svg>")  # fmt: skip


def path_d(xy: np.ndarray) -> str:
    xy = xy[np.isfinite(xy).all(axis=1)]
    if len(xy) == 0:
        return ""
    return "M" + " L".join(f"{x:.3f},{-y:.3f}" for x, y in xy)


def pose_marker(p, color: str, r: float = 0.14, cls: str = "") -> str:
    x, y, th = p
    tip = (x + 2.2 * r * math.cos(th), y + 2.2 * r * math.sin(th))
    return (f'<g class="{cls}"><circle cx="{x:.3f}" cy="{-y:.3f}" r="{r}" fill="{color}" stroke="#111" '
            f'stroke-width="1.2" vector-effect="non-scaling-stroke"/>'
            f'<line x1="{x:.3f}" y1="{-y:.3f}" x2="{tip[0]:.3f}" y2="{-tip[1]:.3f}" stroke="#111" '
            f'stroke-width="2" vector-effect="non-scaling-stroke"/></g>')  # fmt: skip


def line_chart(
    t, ys: list[tuple[str, np.ndarray, str]], limit=None, ymax=None, yl="m", w=560, h=170, shade=None
):
    """Simple SVG line chart. ys: (label, values, css colour var)."""
    pl, pr, pt, pb = 46, 12, 10, 26
    t = np.asarray(t, float)
    allv = np.concatenate([v[np.isfinite(v)] for _, v, _ in ys]) if ys else np.zeros(1)
    top = ymax or max(1e-3, float(np.nanmax(allv)) * 1.1, (limit or 0) * 1.4)
    t1 = float(t[-1]) if len(t) else 1.0

    def X(v):
        return pl + (w - pl - pr) * v / t1

    def Y(v):
        return pt + (h - pt - pb) * (1 - min(v, top) / top)

    g = []
    if shade is not None:  # boolean mask over t → shaded spans
        m = np.asarray(shade, bool)
        i = 0
        while i < len(m):
            if m[i]:
                j = i
                while j + 1 < len(m) and m[j + 1]:
                    j += 1
                g.append(f'<rect x="{X(t[i]):.1f}" y="{pt}" width="{max(1.0, X(t[j]) - X(t[i])):.1f}" '
                         f'height="{h - pt - pb}" class="shade"/>')  # fmt: skip
                i = j + 1
            else:
                i += 1
    for k in range(5):
        v = top * k / 4
        g.append(f'<line x1="{pl}" x2="{w - pr}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" class="grid"/>')
        g.append(f'<text x="{pl - 6}" y="{Y(v) + 4:.1f}" class="tick" text-anchor="end">{v:.2g}</text>')
    for k in range(6):
        v = t1 * k / 5
        g.append(f'<text x="{X(v):.1f}" y="{h - 8}" class="tick" text-anchor="middle">{v:.0f} s</text>')
    if limit is not None:
        g.append(f'<line x1="{pl}" x2="{w - pr}" y1="{Y(limit):.1f}" y2="{Y(limit):.1f}" class="limit"/>')
        g.append(f'<text x="{w - pr - 4}" y="{Y(limit) - 4:.1f}" class="tick lim" text-anchor="end">'
                 f"pass limit {limit:g} {yl}</text>")  # fmt: skip
    for _label, v, color in ys:
        pts = " ".join(f"{X(a):.1f},{Y(b):.1f}" for a, b in zip(t, v, strict=True) if np.isfinite(b))
        g.append(f'<polyline points="{pts}" fill="none" stroke="var({color})" stroke-width="1.8"/>')
    g.append(f'<text x="8" y="{pt + 4}" class="tick" transform="rotate(-90 8 {pt + 4})" text-anchor="end">'
             f"{yl}</text>")  # fmt: skip
    return f'<svg class="chart" viewBox="0 0 {w} {h}" width="{w}" role="img">{"".join(g)}</svg>'


# ---------------------------------------------------------------- describing bench runs

KIND = {"real": "live / real interface", "sim": "simulated", "rec": "recorded"}


def describe_inputs(bench) -> list[tuple[str, str, str]]:
    """(input, description, kind) for the sources a Bench was built with."""

    def name(o):
        return type(o).__name__ if o is not None else "None"

    cam, odo, head, game, team, ball, truth = (bench.camera, bench.odometry, bench.head, bench.game, bench.team,
                                               bench.ball, bench.truth)  # fmt: skip
    rows = []
    c = name(cam)
    if c == "ImageCamera":
        fr = name(cam.frames)
        desc = {"SyntheticFrames": "Images (rendered) → line detector", "FolderFrames": "Image files → line detector",
                "RealSenseFrames": "D435 images → line detector"}.get(fr, fr)  # fmt: skip
        rows.append(("Camera", desc, "real" if fr == "RealSenseFrames" else "sim"))
    else:
        rows.append(("Camera", {"SimCamera": "Simulated observations (no images)", "ReplayCamera": "Recording"}
                     .get(c, c), "rec" if c.startswith("Replay") else "sim"))  # fmt: skip
    o = name(odo)
    od = {
        "SimOdometry": "Simulated leg odometry (noisy)",
        "StationaryOdometry": "None: stationary",
        "LiveOdometry": "Live push from a producer thread",
        "ReplayOdometry": "Recording",
        "SimVisualOdometry": "Simulated cuVSLAM stereo-inertial VO (head motion subtracted)",
        "LiveVisualOdometry": "cuVSLAM poses (live push)",
    }.get(o, o)
    if o == "StationaryOdometry" and odo.cov[0, 0] > 1e-4:
        od = "None: motion unknown (walking assumed)"
    rows.append(("Odometry", od, {"LiveOdometry": "real", "StationaryOdometry": "real", "ReplayOdometry": "rec",
                                  "LiveVisualOdometry": "real"}.get(o, "sim")))  # fmt: skip
    h = name(head)
    hd = {
        "SimHead": "Simulated scan (exact angles)",
        "ImuHead": "Encoders + head IMU model (bias + noise)",
        "LiveHead": "Live push",
        "ReplayHead": "Recording",
        "None": "None",
    }.get(h)
    if h == "FixedHead":
        hd = f"Fixed: yaw {math.degrees(head.yaw):.0f}°, pitch {math.degrees(head.pitch):.0f}°"
    rows.append(
        ("Head angles", hd or h, {"FixedHead": "real", "LiveHead": "real", "ReplayHead": "rec"}.get(h, "sim"))
    )
    g = name(game)
    rows.append(("GameController", {"None": "None (automatic start)", "ScriptedGame": "Scripted events",
                                    "UdpGame": "Real UDP packets (loopback)", "ReplayGame": "Recording"}.get(g, g),
                 {"UdpGame": "real", "ReplayGame": "rec", "None": "none"}.get(g, "sim")))  # fmt: skip
    tm = name(team)
    rows.append(("Teammates", {"None": "None", "SimTeam": "Simulated teammates (2)",
                               "UdpTeam": "Teammate over real UDP (loopback)"}.get(tm, tm),
                 {"UdpTeam": "real", "None": "none"}.get(tm, "sim")))  # fmt: skip
    b = name(ball)
    rows.append(("Ball", "Simulated ball" if b == "SimBall" else "None", "sim" if b == "SimBall" else "none"))
    tr = "None" if truth is None else ("Simulated world" if hasattr(truth, "step") else
                                       "Recording" if hasattr(truth, "observations") else
                                       f"Known fixed pose ({truth[0]:.1f}, {truth[1]:.1f}, {math.degrees(truth[2]):.0f}°)")  # fmt: skip
    rows.append(("Ground truth", tr, "none" if truth is None else "info"))
    return rows


def tag(kind: str) -> str:
    label = {
        "real": "real",
        "sim": "sim",
        "rec": "recorded",
        "none": "off",
        "info": "truth",
        "set": "setting",
    }[kind]
    return f'<span class="tag {kind}">{label}</span>'


def run_figures(fpic: FieldPic, runs, limit=None) -> str:
    """Field plot (truth vs estimate) and error-over-time chart for the bench runs of one test."""
    colors = ["#2b6fd6", "#f08c00"]
    layers, series = [], []
    first = runs[0][1]
    t = first.est.t
    gt = first.gt.pose if first.gt is not None else None
    if gt is not None:
        layers.append(f'<path d="{path_d(gt[:, :2])}" fill="none" stroke="#111" stroke-width="5" '
                      f'stroke-opacity="0.55" vector-effect="non-scaling-stroke"/>')  # fmt: skip
        layers.append(f'<path d="{path_d(gt[:, :2])}" fill="none" stroke="#fff" stroke-width="2" '
                      f'stroke-dasharray="5 4" vector-effect="non-scaling-stroke"/>')  # fmt: skip
    # Before the first "initial" (e.g. waiting for a GameController packet) the filter has no pose yet.
    starts = [next((float(e.t) for e in r.events if e.kind in ("initial", "replaced")), 0.0) for _, r in runs]
    for k, (_b, r) in enumerate(runs[:2]):
        s = int(np.searchsorted(r.est.t, starts[k] - 1e-9))
        layers.append(f'<path d="{path_d(r.est.pose[s:, :2])}" fill="none" stroke="{colors[k]}" '
                      f'stroke-width="{2.4 if k == 0 else 1.4}" vector-effect="non-scaling-stroke"/>')  # fmt: skip
        layers.append(pose_marker(r.est.pose[-1], colors[k]))
        if r.gt is not None:
            e = np.hypot(*(r.est.pose[:, :2] - r.gt.pose[:, :2]).T)
            e[:s] = np.nan
            series.append(("error", e, "--est" if k == 0 else "--est2"))
    if gt is not None:
        layers.insert(2, pose_marker(gt[0], "#ffffff", 0.12))
        layers.append(pose_marker(gt[-1], "#ffffff", 0.12))
    pts = [
        r.est.pose[np.searchsorted(r.est.t, s0 - 1e-9) :, :2] for (_, r), s0 in zip(runs, starts, strict=True)
    ]
    pts += [gt[:, :2]] if gt is not None else []
    allp = np.concatenate(pts)
    allp = allp[np.isfinite(allp).all(axis=1)]
    lo, hi = allp.min(axis=0) - 1.2, allp.max(axis=0) + 1.2
    span = max(hi[0] - lo[0], (hi[1] - lo[1]) * 1.6, 4.0)
    cx, cy = (lo + hi) / 2
    view = (cx - span / 2, cy - span / 3.2, cx + span / 2, cy + span / 3.2)
    field = fpic.svg("".join(layers), view, label="Estimated path vs ground truth")
    lost = first.est.status == "lost"
    chart = line_chart(t, series, limit=limit, ymax=None if not series else min(3.0, max(
        0.5, float(np.nanpercentile(series[0][1], 99)) * 1.15)), shade=lost if lost.any() else None)  # fmt: skip
    legend = ['<span class="key truth"></span>ground truth (start ○ and end ○ white)',
              '<span class="key est"></span>estimate']  # fmt: skip
    if starts[0] > 0:
        legend.append(f"estimate starts at the first GameController 'initial' ({starts[0]:.2f} s)")
    if len(runs) > 1:
        legend.append('<span class="key est2"></span>replay of the recording')
    if lost.any():
        legend.append('<span class="key shade"></span>paused (penalized)')
    return (f'<figure>{field}<figcaption>{" ".join(f"<span>{x}</span>" for x in legend)}</figcaption></figure>'
            f'<figure>{chart}<figcaption>Position error over time</figcaption></figure>')  # fmt: skip


def vision_card(fm, fpic, props_detect, props_pitch) -> str:
    from td_bench.vision import D435_HALF, image_observations, render_field_image
    from td_field_model.sim import robot_to_world

    pose = (-2.5, 1.0, 0.3)  # same as td_bench/test/test_vision.py
    img = render_field_image(fm, D435_HALF, pose, 0.0, 0.4, fm.likelihood_field(), np.random.default_rng(0))
    obs = image_observations(img, D435_HALF, 0.0, 0.4, rng=np.random.default_rng(0))
    wp = robot_to_world(obs.line_points[obs.line_range_ok], pose)
    d = fm.distance(wp)
    bad = robot_to_world(image_observations(img, D435_HALF, 0.0, 0.4 + math.radians(6),
                                            rng=np.random.default_rng(0)).line_points, pose)  # fmt: skip
    dots = "".join(f'<circle cx="{x:.3f}" cy="{-y:.3f}" r="0.045" fill="{"#1f9d55" if e < 0.05 else "#e8a317" if e < 0.15 else "#d63031"}" '
                   f'stroke="#000" stroke-width="0.6" vector-effect="non-scaling-stroke"/>'
                   for (x, y), e in zip(wp, d, strict=True))  # fmt: skip
    bdots = "".join(f'<rect x="{x - 0.035:.3f}" y="{-y - 0.035:.3f}" width="0.07" height="0.07" fill="#ff4fd8" '
                    f'opacity="0.85"/>' for x, y in bad)  # fmt: skip
    allp = np.vstack([wp, bad, [pose[:2]]])
    lo, hi = allp.min(axis=0) - 0.6, allp.max(axis=0) + 0.6
    view = (lo[0], lo[1], hi[0], hi[1])
    top = fpic.svg(
        dots + pose_marker(pose, "#ffffff", 0.12), view, width=520, label="Detected points, top view"
    )
    top_bad = fpic.svg(bdots + pose_marker(pose, "#ffffff", 0.12), view, width=520, label="Wrong pitch")
    return f"""
<section class="card" id="vision">
  <h3>Camera image vs detected field lines</h3>
  <div class="pair">
    <div class="side test">
      <div class="label">Input: what the camera sees</div>
      <p>A rendered D435 frame (424 × 240), robot at (−2.5, 1.0) m facing 17°, head pitched 23° down. This is
      what <code>--camera synthetic-images</code> feeds the detector; a real D435 or image folder goes through the
      same path.</p>
      <figure><img class="frame" src="{png_uri(img)}" alt="Rendered camera frame of the field"></figure>
      <div class="label">Pass conditions</div>
      {props_table(props_detect + props_pitch)}
    </div>
    <div class="side result">
      <div class="label">Result: detected line pixels projected onto the field (top view)</div>
      <figure>{top}<figcaption><span><span class="dot g"></span>&lt; 5 cm from a line</span>
      <span><span class="dot y"></span>5–15 cm</span><span><span class="dot r"></span>&gt; 15 cm</span>
      <span>○ robot</span></figcaption></figure>
      <div class="label">Same image, head pitch believed 6° wrong (what a bad head-angle source looks like)</div>
      <figure>{top_bad}<figcaption><span><span class="dot m"></span>projected points miss the lines</span>
      </figcaption></figure>
    </div>
  </div>
</section>"""


# ---------------------------------------------------------------- HTML


def esc(s: str) -> str:
    return html.escape(s, quote=True)


def humanize(name: str) -> str:
    s = name.split("[")[0].removeprefix("test_").replace("_", " ")
    return s[:1].upper() + s[1:]


def props_table(props) -> str:
    if not props:
        return ""
    rows = []
    for k, v in props:
        meas, _, exp = v.partition(" | ")
        rows.append(
            f"<tr><td>{esc(k)}</td><td class='exp'>{esc(exp)}</td><td class='meas'>{esc(meas)}</td></tr>"
        )
    return ("<table class='props'><thead><tr><th>Check</th><th>Expected</th><th>Measured</th></tr></thead>"
            f"<tbody>{''.join(rows)}</tbody></table>")  # fmt: skip


def badge(status: str) -> str:
    return f'<span class="badge {status}">{status.upper()}</span>'


SCENARIO_TITLES = {
    "test_all_simulated_search_converges": "All simulated: find itself from scratch",
    "test_camera_only_tripod": "Camera only: tripod, no odometry, no head encoders",
    "test_everything_but_camera_live_odometry_and_udp": "Everything but the camera: live odometry + real UDP",
    "test_scripted_penalty_pauses_and_reseeds": "Scripted GameController: penalty, pause, re-entry",
    "test_ball_and_simulated_team": "Ball + simulated teammates: side confidence",
    "test_visual_only_cuvslam_camera_imu_head": "Visual only: cuVSLAM (model) + camera + IMU head angles",
    "test_record_then_replay_is_identical": "Record a session, replay it: identical result",
}
LIMITS = {
    "test_all_simulated_search_converges": 0.3,
    "test_camera_only_tripod": 0.2,
    "test_everything_but_camera_live_odometry_and_udp": 0.3,
    "test_visual_only_cuvslam_camera_imu_head": 0.3,
}


def scenario_card(tc, cap_runs, fpic) -> str:
    fn = tc["name"]
    _, doc, src = test_source(tc["classname"], fn)
    bench, _ = cap_runs[0]
    inputs = "".join(f"<tr><td>{esc(a)}</td><td>{esc(b)}</td><td>{tag(k)}</td></tr>"
                     for a, b, k in describe_inputs(bench))  # fmt: skip
    asserts = "\n".join(assertions(src))
    figs = run_figures(fpic, cap_runs, LIMITS.get(fn))
    dur = cap_runs[0][1].est.t[-1]
    msg = f"<pre class='fail'>{esc(tc['message'])}</pre>" if tc["status"] != "passed" else ""
    return f"""
<section class="card" id="{esc(fn)}">
  <h3>{esc(SCENARIO_TITLES.get(fn, humanize(fn)))}</h3>
  <div class="pair">
    <div class="side test">
      <div class="label">Test</div>
      <p class="fn"><code>{esc(fn)}</code> · {dur:.0f} s of robot time</p>
      {f"<p>{esc(doc)}</p>" if doc else ""}
      <table class="inputs"><thead><tr><th>Input</th><th>Source</th><th></th></tr></thead><tbody>{inputs}</tbody></table>
      <div class="label">Assertions</div>
      <pre class="code">{esc(asserts)}</pre>
      <details><summary>Full test code</summary><pre class="code">{esc(src)}</pre></details>
    </div>
    <div class="side result">
      <div class="label">Result {badge(tc["status"])} <span class="time">{tc["time"]:.1f} s</span></div>
      {props_table(tc["props"])}
      {msg}
      {figs}
    </div>
  </div>
</section>"""


def test_row(tc) -> str:
    file, doc, src = test_source(tc["classname"], tc["name"])
    asserts = assertions(src)
    left = (f"<div class='tname'>{esc(humanize(tc['name']))}</div><div class='file'>{esc(file)} · "
            f"<code>{esc(tc['name'])}</code></div>"
            + (f"<p class='doc'>{esc(doc)}</p>" if doc else "")
            + (f"<pre class='code small'>{esc(chr(10).join(asserts))}</pre>" if asserts else "")
            + f"<details><summary>Full test code</summary><pre class='code'>{esc(src)}</pre></details>")  # fmt: skip
    right = (f"{badge(tc['status'])} <span class='time'>{tc['time']:.2f} s</span>" + props_table(tc["props"])
             + (f"<pre class='fail'>{esc(tc['message'])}</pre>" if tc["message"] else ""))  # fmt: skip
    return f"<tr class='{tc['status']}'><td class='lt'>{left}</td><td class='rt'>{right}</td></tr>"


CSS = """
:root{--bg:#f6f7f9;--card:#fff;--fg:#1d2330;--muted:#5d6675;--line:#dfe3ea;--code:#f1f3f7;--pass:#18794e;
--passbg:#e3f5ea;--fail:#c62828;--failbg:#fdeaea;--est:#2b6fd6;--est2:#f08c00;--limit:#c62828;--accent:#2b6fd6}
@media (prefers-color-scheme: dark){:root:not([data-theme="light"]){--bg:#12151b;--card:#1b2029;--fg:#e6e9ef;
--muted:#9aa3b2;--line:#2c3340;--code:#141922;--pass:#4cc38a;--passbg:#163626;--fail:#ff6b6b;--failbg:#3a1a1a}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);
font:15px/1.5 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
main{max-width:1280px;margin:0 auto;padding:24px 16px 64px}h1{font-size:26px;margin:0 0 4px}
h2{font-size:20px;margin:40px 0 8px}h3{font-size:17px;margin:0 0 12px}p{margin:6px 0}
.sub{color:var(--muted)}code{font:13px/1.4 ui-monospace,Consolas,monospace}
.summary{display:grid;grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:12px;margin:18px 0}
.stat{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:12px 14px}
.stat .n{font-size:24px;font-weight:650;font-variant-numeric:tabular-nums}.stat .k{color:var(--muted);font-size:13px}
table{border-collapse:collapse;width:100%}th,td{text-align:left;padding:6px 8px;border-bottom:1px solid var(--line);
vertical-align:top}th{font-size:12px;text-transform:uppercase;letter-spacing:.04em;color:var(--muted)}
.pkgs td:nth-child(n+2){font-variant-numeric:tabular-nums}
.card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:18px;margin:16px 0}
.pair{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1.25fr);gap:20px}
.side{min-width:0}.side.test{border-right:1px solid var(--line);padding-right:20px}
.label{font-size:12px;text-transform:uppercase;letter-spacing:.05em;color:var(--muted);margin:12px 0 6px;
font-weight:600}.label:first-child{margin-top:0}
.fn{color:var(--muted)}pre.code{background:var(--code);border:1px solid var(--line);border-radius:8px;padding:10px;
overflow-x:auto;font:12.5px/1.45 ui-monospace,Consolas,monospace;margin:4px 0;white-space:pre}
pre.small{font-size:12px}pre.fail{background:var(--failbg);color:var(--fail);border-radius:8px;padding:10px;
white-space:pre-wrap;font:12px/1.4 ui-monospace,Consolas,monospace}
details summary{cursor:pointer;color:var(--accent);font-size:13px;margin-top:6px}
.badge{display:inline-block;font-size:12px;font-weight:700;border-radius:999px;padding:1px 10px;letter-spacing:.03em}
.badge.passed{background:var(--passbg);color:var(--pass)}.badge.failed,.badge.error{background:var(--failbg);
color:var(--fail)}.badge.skipped{background:var(--code);color:var(--muted)}.time{color:var(--muted);font-size:13px;
font-variant-numeric:tabular-nums;text-transform:none;letter-spacing:0;font-weight:400}
.props{margin:6px 0 10px}.props td{font-size:13.5px}.props .exp{color:var(--muted);white-space:nowrap}
.props .meas{font-weight:650;font-variant-numeric:tabular-nums}
.inputs td{font-size:13.5px}.tag{font-size:11px;font-weight:700;border-radius:4px;padding:1px 6px;white-space:nowrap}
.tag.real{background:#dff3e4;color:#17663a}.tag.sim{background:#e8ecf3;color:#475569}.tag.rec{background:#efe4fb;
color:#6b3fa0}.tag.none{background:transparent;color:var(--muted);border:1px solid var(--line)}
.tag.info{background:#fff3d6;color:#8a5a00}.tag.set{background:transparent;color:var(--muted);border:1px dashed var(--line)}
.scroll{overflow-x:auto;max-width:100%}figure{margin:10px 0;overflow:hidden}svg{overflow:hidden}figcaption{font-size:12.5px;color:var(--muted);display:flex;flex-wrap:wrap;gap:4px 14px;
margin-top:4px}svg.field,svg.chart,img.frame{max-width:100%;height:auto;display:block;border-radius:6px}
img.frame{image-rendering:pixelated;width:100%;max-width:520px;border:1px solid var(--line)}
svg.chart{background:var(--code);border:1px solid var(--line)}
svg .grid{stroke:var(--line);stroke-width:1}svg .tick{fill:var(--muted);font:11px system-ui,sans-serif}
svg .limit{stroke:var(--limit);stroke-dasharray:6 4;stroke-width:1.4}svg .lim{fill:var(--limit)}
svg .shade{fill:var(--muted);opacity:.18}
.key{display:inline-block;width:22px;height:0;border-top:3px solid;vertical-align:middle;margin-right:6px}
.key.truth{border-top:3px dashed #888}.key.est{border-color:var(--est)}.key.est2{border-color:var(--est2)}
.key.shade{height:10px;border:0;background:var(--muted);opacity:.3}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:5px;vertical-align:middle}
.dot.g{background:#1f9d55}.dot.y{background:#e8a317}.dot.r{background:#d63031}.dot.m{background:#ff4fd8;
border-radius:1px}
.tests{background:var(--card);border:1px solid var(--line);border-radius:12px;overflow:hidden}
.tests>thead th{background:var(--code);padding:10px 12px}.tests td.lt{width:58%;border-right:1px solid var(--line)}
.tests td{padding:10px 12px}.tname{font-weight:620}.file{font-size:12.5px;color:var(--muted)}
.doc{font-size:13.5px;color:var(--muted)}tr.failed td,tr.error td{background:var(--failbg)}
nav{display:flex;flex-wrap:wrap;gap:6px 16px;font-size:14px;margin:6px 0}a{color:var(--accent)}
@media (max-width:820px){.pair{grid-template-columns:1fr}.side.test{border-right:0;padding-right:0;
border-bottom:1px solid var(--line);padding-bottom:12px}.tests td.lt{width:auto}.tests tr{display:grid}
.tests td.lt{border-right:0}}
"""


def build_html(results, cap: Capture, wall) -> str:
    from td_field_model import FieldModel

    fm = FieldModel.builtin("hsl_m_2026")
    fpic = FieldPic(fm)
    total = sum(len(v) for v in results.values())
    passed = sum(tc["status"] == "passed" for v in results.values() for tc in v)
    failed = total - passed - sum(tc["status"] == "skipped" for v in results.values() for tc in v)
    pkg_rows = "".join(
        f"<tr><td><a href='#pkg-{p}'><code>{p}</code></a><div class='file'>{esc(BLURB[p])}</div></td>"
        f"<td>{len(results[p])}</td><td>{sum(t['status'] == 'passed' for t in results[p])}</td>"
        f"<td>{sum(t['status'] in ('failed', 'error') for t in results[p])}</td><td>{wall[p]:.0f} s</td></tr>"
        for p in ORDER
    )
    by_name = {tc["name"]: tc for tc in results["td_bench"]}
    cards = []
    for fn in SCENARIO_TITLES:
        tc = by_name.get(fn)
        runs = next((v for k, v in cap.runs.items() if k.endswith("::" + fn)), None)
        if tc and runs:
            cards.append(scenario_card(tc, runs, fpic))
    vis = vision_card(fm, fpic, by_name.get("test_detected_points_lie_on_lines", {}).get("props", []),
                      by_name.get("test_image_camera_with_synthetic_frames", {}).get("props", []))  # fmt: skip
    sections = []
    for p in ORDER:
        rows = "".join(test_row(tc) for tc in results[p])
        npass = sum(t["status"] == "passed" for t in results[p])
        sections.append(
            f"<h2 id='pkg-{p}'><code>{p}</code> <span class='sub'>· {npass}/{len(results[p])} passed</span></h2>"
            f"<p class='sub'>{esc(BLURB[p])}</p><table class='tests'><thead><tr><th>Test (what it checks)</th>"
            f"<th>Result</th></tr></thead><tbody>{rows}</tbody></table>"
        )
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    verdict = "All tests passed" if failed == 0 else f"{failed} test(s) failed"
    nav = "".join(f"<a href='#{fn}'>{esc(t.split(':')[0])}</a>" for fn, t in SCENARIO_TITLES.items())
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Localization Test Results</title><style>{CSS}</style></head>
<body><main>
<h1>Localization test results</h1>
<p class="sub">robocup-simulations/localization · generated {now} by <code>test_report/make_results.py</code> · Python
{platform.python_version()}, numpy {np.__version__}, {esc(platform.system())} {esc(platform.release())} ·
simulation and loopback only, no robot hardware</p>
<div class="summary">
  <div class="stat"><div class="n">{passed} / {total}</div><div class="k">{verdict}</div></div>
  <div class="stat"><div class="n">{len(ORDER)}</div><div class="k">packages</div></div>
  <div class="stat"><div class="n">{wall["total"] / 60:.1f} min</div><div class="k">wall time (run in parallel)</div></div>
  <div class="stat"><div class="n">{len(cards)}</div><div class="k">hardware-mix runs plotted</div></div>
</div>
<div class="scroll"><table class="pkgs"><thead><tr><th>Package</th><th>Tests</th><th>Passed</th><th>Failed</th><th>Time</th></tr>
</thead><tbody>{pkg_rows}</tbody></table></div>

<h2>Hardware-mix runs (td_bench)</h2>
<p class="sub">Each card shows the test on the left (which inputs are real, simulated or recorded, and what it
asserts) and the result on the right (measured values against the pass conditions, the estimated path against
ground truth, and the position error over time). Tags: <span class="tag real">real</span> a real interface
(UDP sockets, a live push API, a fixed tripod) · <span class="tag sim">sim</span> simulated ·
<span class="tag rec">recorded</span> replayed from a file.</p>
<nav>{nav}<a href="#vision">Vision</a></nav>
<p class="sub">Visual-only comparison (cuVSLAM model + camera + IMU head angles, several variants and seeds):
<a href="visual_only.html">visual_only.html</a></p>
{"".join(cards)}
{vis}

<h2>Every test, side by side</h2>
<p class="sub">Left: the test's name, docstring and assertions (expand for the full code). Right: the outcome,
time, and any measured values it logged.</p>
{"".join(sections)}
</main></body></html>
"""


def main() -> int:
    results, cap, wall = run_all()
    (OUT / "index.html").write_text(build_html(results, cap, wall), encoding="utf-8")
    total = sum(len(v) for v in results.values())
    bad = [f"{p}::{t['name']}" for p, v in results.items() for t in v if t["status"] in ("failed", "error")]
    print(f"\nwrote {OUT / 'index.html'}: {total - len(bad)}/{total} passed")
    for b in bad:
        print("FAILED", b)
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
