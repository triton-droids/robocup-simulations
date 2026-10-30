"""Visual-only localization test: cuVSLAM (model) + camera images + head angles from encoders/IMU.

    cd localization
    python test_report/visual_only.py [--seeds 1 2 3] [--duration 30]

Writes test_report/visual_only.html (each variant: test setup beside its result) and
test_report/raw/visual_only.json.

No leg odometry, GameController, teammates or ball: the robot starts with "search our own half" and must find
itself from the camera alone, with visual odometry as its only motion input. cuVSLAM itself cannot run on this
machine (it needs an NVIDIA GPU + Isaac ROS on Linux/Jetson), so it is the stereo-inertial model in
td_bench.sources.SimVisualOdometry (roadmap §3.6). Every variant renders real images and runs them through the
line detector (td_bench.vision).
"""
# ruff: noqa: E501  (long lines are HTML templates)

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent
for p in ("td_field_model", "td_game", "td_localization", "td_bench"):
    sys.path.insert(0, str(ROOT / p))
sys.path.insert(0, str(OUT))

import make_results as mr  # noqa: E402
from td_bench.sources import (  # noqa: E402
    ImuHead,
    SimHead,
    SimOdometry,
    SimVisualOdometry,
    StationaryOdometry,
    World,
)
from td_bench.vision import D435_HALF, ImageCamera, SyntheticFrames  # noqa: E402
from td_field_model.scenarios import HeadMotion, ScenarioBuilder  # noqa: E402
from td_localization.eval import evaluate  # noqa: E402

from td_bench import Bench  # noqa: E402
from td_field_model import FieldModel  # noqa: E402

START = (-3.0, 1.0, 0.3)

# name, title, what it isolates, head scan period (s), head source, odometry source
VARIANTS = [
    ("visual_only", "Visual only: cuVSLAM + camera + encoder/IMU head angles",
     "The requested test. Motion comes only from visual odometry; head angles carry calibration bias and noise.",
     4.0, "imu", "vo"),
    ("visual_only_slow_scan", "Visual only, slower head scan",
     "Same, but the head scans at half speed (peak 0.6 rad/s), below the 1.2 rad/s rate at which VO tracking "
     "is dropped. Isolates the effect of tracking loss during fast pans.",
     8.0, "imu", "vo"),
    ("visual_only_exact_head", "Visual only, exact head angles",
     "Same as the first, but with perfect head angles. Isolates the cost of head-angle bias and noise.",
     4.0, "exact", "vo"),
    ("camera_no_odometry", "Camera + head angles, no odometry at all",
     "No motion input: the filter only knows the robot may be walking. Shows what visual odometry adds.",
     4.0, "imu", "none"),
    ("legs_reference", "Reference: leg odometry instead of VO",
     "The planned normal setup (legs + gyro), with the same camera and head. Both odometry sources are models, so "
     "compare them as a sanity check, not as a verdict on cuVSLAM vs legs.",
     4.0, "imu", "legs"),
]  # fmt: skip


def world_for(fm, seed: int, duration: float, period: float) -> World:
    head = HeadMotion(yaw_amplitude=0.8, period=period, pitch=0.35)
    return World(ScenarioBuilder(fm, seed=seed, head=head).start(START).walk(duration).steps)


def run_variant(fm, v, seed: int, duration: float):
    name, _, _, period, head_kind, odom_kind = v
    w = world_for(fm, seed, duration, period)
    head = ImuHead(w, seed=seed) if head_kind == "imu" else SimHead(w)
    cam = ImageCamera(SyntheticFrames(fm, D435_HALF, w, seed=seed), D435_HALF, head)
    odom = {
        "vo": lambda: SimVisualOdometry(w, head, seed=seed),
        "none": lambda: StationaryOdometry(sigma=(0.012, 0.012, 0.02)),
        "legs": lambda: SimOdometry(w),
    }[odom_kind]()
    bench = Bench(fm, cam, odom, head, truth=w, seed=seed)
    t0 = time.perf_counter()
    r = bench.run(duration)
    wall = time.perf_counter() - t0
    ev = evaluate(r.gt, r.est, f"{name}_s{seed}")
    m = ev.metrics
    e = np.hypot(*(r.est.pose[:, :2] - r.gt.pose[:, :2]).T)
    he = np.degrees(np.abs(np.remainder(r.est.pose[:, 2] - r.gt.pose[:, 2] + np.pi, 2 * np.pi) - np.pi))
    tail = r.est.t >= r.est.t[-1] - 10
    first_ok = next((float(t) for t, x in zip(r.est.t, e, strict=True) if x < 0.3), None)
    out = {
        "seed": seed,
        "pos_p50_m": m["pos_p50_m"],
        "pos_p95_m": m["pos_p95_m"],
        "tail_p50_m": float(np.median(e[tail])),
        "tail_p95_m": float(np.percentile(e[tail], 95)),
        "tail_heading_p95_deg": float(np.percentile(he[tail], 95)),
        "first_within_0_3m_s": first_ok,
        "converge_s": ev.events[0]["converge_s"] if ev.events else None,
        "wrong_half_fraction": m.get("wrong_half_fraction"),
        "side_confidence_end": float(r.side.confidence),
        "vo_lost_fraction": getattr(odom, "lost_fraction", None),
        "head_pitch_bias_deg": math.degrees(head.pitch_bias) if hasattr(head, "pitch_bias") else 0.0,
        "wall_s": wall,
    }
    return out, (bench, r)


def fmt(v, unit="", nd=2, pct=False):
    if v is None:
        return "–"
    return f"{100 * v:.0f}%" if pct else f"{v:.{nd}f}{unit}"


def summary_rows(rows: list[dict]):
    def med(k):
        vals = [r[k] for r in rows if r[k] is not None]
        return float(np.median(vals)) if vals else None

    def worst(k):
        vals = [r[k] for r in rows if r[k] is not None]
        return float(np.max(vals)) if vals else None

    conv_ok = sum(r["converge_s"] is not None for r in rows)
    return [
        ("Converged (within 0.3 m, held 2 s)", f"{conv_ok} of {len(rows)} seeds", "all"),
        ("Time to converge (median)", fmt(med("converge_s"), " s", 1), "≤ 10 s target"),
        ("Position error, last 10 s, median", fmt(med("tail_p50_m"), " m"), "≤ 0.2 m target"),
        ("Position error, last 10 s, p95 (worst seed)", fmt(worst("tail_p95_m"), " m"), "≤ 0.5 m target"),
        (
            "Heading error, last 10 s, p95 (worst seed)",
            fmt(worst("tail_heading_p95_deg"), "°", 1),
            "≤ 5° target",
        ),
        ("Wrong-half time (worst seed)", fmt(worst("wrong_half_fraction"), pct=True), "0%"),
        ("Side confidence at end (median)", fmt(med("side_confidence_end")), "high"),
        ("VO tracking lost (median)", fmt(med("vo_lost_fraction"), pct=True), "(info)"),
    ]


def per_seed_table(rows: list[dict]) -> str:
    head = ("<tr><th>Seed</th><th>Converge</th><th>Err p50 / p95, last 10 s</th><th>Heading p95</th>"
            "<th>Wrong half</th><th>VO lost</th><th>Pitch bias</th></tr>")  # fmt: skip
    body = "".join(
        f"<tr><td>{r['seed']}</td><td>{fmt(r['converge_s'], ' s', 1)}</td>"
        f"<td>{fmt(r['tail_p50_m'])} / {fmt(r['tail_p95_m'])} m</td><td>{fmt(r['tail_heading_p95_deg'], '°', 1)}</td>"
        f"<td>{fmt(r['wrong_half_fraction'], pct=True)}</td><td>{fmt(r['vo_lost_fraction'], pct=True)}</td>"
        f"<td>{r['head_pitch_bias_deg']:+.2f}°</td></tr>"
        for r in rows
    )
    return f"<div class='scroll'><table class='props seeds'><thead>{head}</thead><tbody>{body}</tbody></table></div>"


def describe(bench, v) -> list[tuple[str, str, str]]:
    _, _, _, period, head_kind, odom_kind = v
    return [
        ("Camera", "Rendered images (424×240) → line detector", "sim"),
        ("Motion", {"vo": "Simulated cuVSLAM stereo-inertial VO (head motion subtracted)",
                    "none": "None: motion unknown (walking assumed)",
                    "legs": "Simulated leg odometry + gyro"}[odom_kind], "sim" if odom_kind != "none" else "none"),
        ("Head angles", "Encoders + head IMU: bias ~0.5°, noise 0.2°" if head_kind == "imu" else "Exact",
         "sim"),
        ("Head scan", f"±46°, period {period:g} s (peak {0.8 * 2 * math.pi / period:.2f} rad/s)", "set"),
        ("GameController", "None (automatic start: search own half)", "none"),
        ("Teammates / ball", "None", "none"),
    ]  # fmt: skip


def page(results, plots, fpic, seeds, duration, total_wall) -> str:
    cards, overview = [], []
    for v in VARIANTS:
        name, title, why = v[0], v[1], v[2]
        rows = results[name]
        _, plot_seed, (bench, r) = plots[name]
        srows = summary_rows(rows)
        inputs = "".join(f"<tr><td>{mr.esc(a)}</td><td>{mr.esc(b)}</td><td>{mr.tag(k)}</td></tr>"
                         for a, b, k in describe(bench, v))  # fmt: skip
        checks = "".join(f"<tr><td>{mr.esc(a)}</td><td class='exp'>{mr.esc(c)}</td><td class='meas'>{mr.esc(b)}</td></tr>"
                         for a, b, c in srows)  # fmt: skip
        figs = mr.run_figures(fpic, [(bench, r)], limit=0.3)
        conv = sum(x["converge_s"] is not None for x in rows)
        ok = conv == len(rows) and max(x["tail_p95_m"] for x in rows) <= 0.5
        overview.append(f"<tr><td><a href='#{name}'>{mr.esc(title)}</a></td><td>{conv}/{len(rows)}</td>"
                        f"<td>{srows[2][1]}</td><td>{srows[3][1]}</td><td>{srows[5][1]}</td><td>{srows[7][1]}</td>"
                        f"<td>{mr.badge('passed' if ok else 'failed').replace('PASSED', 'MEETS TARGETS').replace('FAILED', 'MISSES TARGETS')}</td></tr>")  # fmt: skip
        cards.append(f"""
<section class="card" id="{name}">
  <h3>{mr.esc(title)}</h3>
  <div class="pair">
    <div class="side test">
      <div class="label">Test</div>
      <p>{mr.esc(why)}</p>
      <table class="inputs"><thead><tr><th>Input</th><th>Source</th><th></th></tr></thead><tbody>{inputs}</tbody></table>
      <div class="label">Run</div>
      <p class="fn">{len(seeds)} seeds × {duration:g} s walking from ({START[0]:g}, {START[1]:g}), start pose unknown
      to the filter. Plots show seed {plot_seed}, the worst by p95 error.</p>
    </div>
    <div class="side result">
      <div class="label">Result</div>
      <table class='props'><thead><tr><th>Check</th><th>Expected</th><th>Measured</th></tr></thead><tbody>{checks}</tbody></table>
      {figs}
      <details><summary>Per-seed numbers</summary>{per_seed_table(rows)}</details>
    </div>
  </div>
</section>""")
    now = dt.datetime.now().strftime("%Y-%m-%d %H:%M")
    return f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Visual-Only Localization Test</title><style>{mr.CSS}
.seeds td,.seeds th{{font-size:12.5px;white-space:nowrap}}.note{{background:var(--code);border:1px solid var(--line);
border-radius:10px;padding:10px 14px;margin:12px 0}}</style></head>
<body><main>
<p><a href="index.html">← All test results</a></p>
<h1>Visual-only localization test</h1>
<p class="sub">cuVSLAM (model) + camera + head angles from encoders/IMU · generated {now} by
<code>test_report/visual_only.py</code> · {len(seeds)} seeds × {len(VARIANTS)} variants × {duration:g} s ·
{total_wall / 60:.1f} min</p>
<div class="note"><b>What is real and what is modelled.</b> The localization code (particle filter, side belief),
the image renderer and the line detector are the real bench code. <b>cuVSLAM is a model</b>
(<code>SimVisualOdometry</code>): it can't run on this machine, which has no NVIDIA GPU and no Isaac ROS. The model
is stereo-inertial VO with a 3% scale error, 5% translation noise, 1.5°/min gyro bias and head-motion subtraction
using the believed head angles. Tracking drops while the head pans faster than 1.2 rad/s and in random 1 s episodes
(2% per second). The head angles are also a model (<code>ImuHead</code>). Real cuVSLAM output plugs in through
<code>LiveVisualOdometry.push_pose()</code>. Targets are the roadmap §3.3 ones.</div>
<div class="scroll"><table class="pkgs"><thead><tr><th>Variant</th><th>Converged</th><th>Err p50, last 10 s</th><th>Err p95, worst</th>
<th>Wrong half, worst</th><th>VO lost</th><th></th></tr></thead><tbody>{"".join(overview)}</tbody></table></div>
{"".join(cards)}
</main></body></html>
"""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[1, 2, 3])
    ap.add_argument("--duration", type=float, default=30.0)
    a = ap.parse_args(argv)
    fm = FieldModel.builtin("hsl_m_2026")
    results, plots = {}, {}
    t0 = time.perf_counter()
    for v in VARIANTS:
        results[v[0]] = []
        for seed in a.seeds:
            out, run = run_variant(fm, v, seed, a.duration)
            results[v[0]].append(out)
            # plot the worst seed (largest p95 error in the last 10 s), so failures are visible
            if v[0] not in plots or out["tail_p95_m"] > plots[v[0]][0]:
                plots[v[0]] = (out["tail_p95_m"], seed, run)
            print(f"{v[0]:24s} seed {seed}: converge {fmt(out['converge_s'], ' s', 1):>7s}  "
                  f"tail p50 {out['tail_p50_m']:.2f} p95 {out['tail_p95_m']:.2f} m  "
                  f"wrong-half {fmt(out['wrong_half_fraction'], pct=True)}  "
                  f"VO lost {fmt(out['vo_lost_fraction'], pct=True)}  ({out['wall_s']:.0f} s)", flush=True)  # fmt: skip
    total = time.perf_counter() - t0
    (OUT / "raw").mkdir(exist_ok=True)
    (OUT / "raw" / "visual_only.json").write_text(json.dumps(
        {"seeds": a.seeds, "duration": a.duration, "results": results}, indent=1), encoding="utf-8")  # fmt: skip
    (OUT / "visual_only.html").write_text(page(results, plots, mr.FieldPic(fm), a.seeds, a.duration, total),
                                          encoding="utf-8")  # fmt: skip
    print(f"wrote {OUT / 'visual_only.html'} in {total / 60:.1f} min")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
