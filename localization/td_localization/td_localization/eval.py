"""Localization evaluation (LOC-07). numpy only, no ROS.

Scores estimated robot poses against ground truth and writes report.md, results.json and per-sample CSVs.
Conventions follow td_perception's evaluation_perception/motion_eval.py:
- Each ground-truth sample is matched to the nearest estimate within `tolerance_s` (ties go to the earlier
  one). No interpolation and no guessed time offset.
- Missing or unusable estimates count as failures in "accuracy within threshold", so selective output can't
  look accurate. Accuracy-only statistics (p50/p95) are over usable samples, with availability reported
  beside them.
- Metrics that can't be computed are None (N/A in reports), never a silent zero.

Localization-specific metrics:
- Wrong half: an estimate nearer the mirror of the truth than the truth itself, with heading weighted
  `heading_scale` m/rad. Fraction, episode count, longest episode.
- "Confidently wrong": wrong-half samples whose side_confidence ≥ `confident_side`. This is the dangerous case
  (own goals).
- Convergence after each ground-truth event (initial, fall, kidnap, penalty_reentry): seconds until the error
  stays below `converge_tol_m` for `converge_hold_s`, or None if it never does before the next event.
- Latency from published_s when given (same clock as timestamp_s).
- Pass/fail checks against docs/roadmap.md §3.3.

CSV contracts (timestamps strictly increasing, seconds on one clock):
  ground truth: timestamp_s,x_m,y_m,theta_rad[,event]
  estimates:    timestamp_s,status,x_m,y_m,theta_rad[,side_confidence][,published_s]
Estimate status is one of localized, uncertain, initializing, lost. Only localized and uncertain are usable.

CLI:
  python -m td_localization.eval evaluate --gt gt.csv --est run.csv [--est other.csv] --out reports/x
  python -m td_localization.eval suite --out reports/suite      (simulated scenario suite; see suite.py)
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

USABLE = ("localized", "uncertain")
STATUSES = ("localized", "uncertain", "initializing", "lost")
EVENTS = ("initial", "fall", "kidnap", "penalty_reentry")


# ---------------------------------------------------------------- data


@dataclass
class GroundTruth:
    t: np.ndarray  # (N,)
    pose: np.ndarray  # (N, 3) x, y, θ in the field frame (own goal at -x)
    events: list[tuple[float, str]] = field(default_factory=list)


@dataclass
class EstimateTrack:
    t: np.ndarray  # (M,)
    status: np.ndarray  # (M,) str
    pose: np.ndarray  # (M, 3); rows for unusable statuses may be nan
    side_confidence: np.ndarray | None = None  # (M,) or None
    published: np.ndarray | None = None  # (M,) or None; nan where unknown


@dataclass(frozen=True)
class EvalConfig:
    tolerance_s: float = 0.05
    pos_threshold_m: float = 0.5  # "accurate" = within this AND heading_threshold_deg
    heading_threshold_deg: float = 5.0
    heading_scale: float = 2.0  # m per rad, for the wrong-half test
    converge_tol_m: float = 0.3
    converge_hold_s: float = 2.0
    confident_side: float = 0.9
    # docs/roadmap.md §3.3
    target_pos_p50_m: float = 0.2
    target_pos_p95_m: float = 0.5
    target_heading_p95_deg: float = 5.0
    target_converge_s: float = 10.0


@dataclass
class EvalResult:
    name: str
    metrics: dict
    events: list[dict]
    checks: dict  # name → True / False / None
    samples: dict  # per ground-truth sample arrays, for the samples CSV


# ---------------------------------------------------------------- CSV


def _num(row: dict, key: str, required: bool) -> float | None:
    v = (row.get(key) or "").strip()
    if v == "":
        if required:
            raise ValueError(f"missing {key}")
        return None
    x = float(v)
    if not math.isfinite(x):
        raise ValueError(f"nonfinite {key}")
    return x


def _check_increasing(t: list[float], path) -> None:
    if any(b <= a for a, b in zip(t, t[1:], strict=False)):
        raise ValueError(f"{path}: timestamps must be strictly increasing")


def read_ground_truth(path) -> GroundTruth:
    t, pose, events = [], [], []
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        need = {"timestamp_s", "x_m", "y_m", "theta_rad"}
        if not need.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: required columns {sorted(need)}")
        for line, row in enumerate(reader, 2):
            try:
                ts = _num(row, "timestamp_s", True)
                t.append(ts)
                pose.append([_num(row, k, True) for k in ("x_m", "y_m", "theta_rad")])
                ev = (row.get("event") or "").strip()
                if ev:
                    if ev not in EVENTS:
                        raise ValueError(f"unknown event {ev!r}")
                    events.append((ts, ev))
            except (ValueError, TypeError) as e:
                raise ValueError(f"{path}:{line}: {e}") from e
    if not t:
        raise ValueError(f"{path}: ground truth is empty")
    _check_increasing(t, path)
    return GroundTruth(np.array(t), np.array(pose), events)


def read_estimates(path) -> EstimateTrack:
    t, st, pose, conf, pub = [], [], [], [], []
    with Path(path).open(newline="") as f:
        reader = csv.DictReader(f)
        need = {"timestamp_s", "status", "x_m", "y_m", "theta_rad"}
        if not need.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: required columns {sorted(need)}")
        for line, row in enumerate(reader, 2):
            try:
                ts = _num(row, "timestamp_s", True)
                s = (row["status"] or "").strip().lower()
                if s not in STATUSES:
                    raise ValueError(f"unknown status {s!r}")
                usable = s in USABLE
                p = [_num(row, k, usable) for k in ("x_m", "y_m", "theta_rad")]
                c = _num(row, "side_confidence", False)
                if c is not None and not 0.0 <= c <= 1.0:
                    raise ValueError("side_confidence must be in [0, 1]")
                q = _num(row, "published_s", False)
                if q is not None and q < ts:
                    raise ValueError("published_s precedes timestamp_s")
                t.append(ts)
                st.append(s)
                pose.append([np.nan if v is None else v for v in p])
                conf.append(np.nan if c is None else c)
                pub.append(np.nan if q is None else q)
            except (ValueError, TypeError) as e:
                raise ValueError(f"{path}:{line}: {e}") from e
    _check_increasing(t, path)
    conf_a, pub_a = np.array(conf), np.array(pub)
    return EstimateTrack(
        np.array(t),
        np.array(st, dtype=object),
        np.array(pose).reshape(-1, 3),
        None if np.all(np.isnan(conf_a)) else conf_a,
        None if np.all(np.isnan(pub_a)) else pub_a,
    )


def write_ground_truth(path, gt: GroundTruth) -> None:
    ev = {round(t, 9): k for t, k in gt.events}
    with Path(path).open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_s", "x_m", "y_m", "theta_rad", "event"])
        for t, p in zip(gt.t, gt.pose, strict=True):
            w.writerow(
                [f"{t:.6f}", f"{p[0]:.6f}", f"{p[1]:.6f}", f"{p[2]:.6f}", ev.get(round(float(t), 9), "")]
            )


def write_estimates(path, est: EstimateTrack) -> None:
    def fmt(v):
        return "" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.6f}"

    with Path(path).open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["timestamp_s", "status", "x_m", "y_m", "theta_rad", "side_confidence", "published_s"])
        for i in range(len(est.t)):
            conf = None if est.side_confidence is None else float(est.side_confidence[i])
            pub = None if est.published is None else float(est.published[i])
            w.writerow(
                [
                    fmt(float(est.t[i])),
                    est.status[i],
                    *(fmt(float(v)) for v in est.pose[i]),
                    fmt(conf),
                    fmt(pub),
                ]
            )


# ---------------------------------------------------------------- scoring


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def _pct(x: np.ndarray, q: float) -> float | None:
    return float(np.percentile(x, q)) if len(x) else None


def align(gt: GroundTruth, est: EstimateTrack, tolerance_s: float) -> np.ndarray:
    """Index of the nearest estimate for each ground-truth sample within tolerance, else -1.

    Ties go to the earlier estimate.
    """
    j = np.searchsorted(est.t, gt.t)
    lo, hi = np.clip(j - 1, 0, len(est.t) - 1), np.clip(j, 0, len(est.t) - 1)
    pick = np.where(np.abs(est.t[lo] - gt.t) <= np.abs(est.t[hi] - gt.t), lo, hi)
    ok = np.abs(est.t[pick] - gt.t) <= tolerance_s + 1e-9
    return np.where(ok, pick, -1)


def _episodes(flags: np.ndarray, t: np.ndarray) -> tuple[int, float]:
    """Number of runs of True and the longest run's duration (s)."""
    count, longest, start = 0, 0.0, None
    for i, f in enumerate(flags):
        if f and start is None:
            start, count = i, count + 1
        if (not f or i == len(flags) - 1) and start is not None:
            end = i if f else i - 1
            longest = max(longest, float(t[end] - t[start]))
            start = None
    return count, longest


def evaluate(
    gt: GroundTruth, est: EstimateTrack, name: str = "run", cfg: EvalConfig | None = None
) -> EvalResult:
    cfg = cfg or EvalConfig()
    idx = align(gt, est, cfg.tolerance_s)
    matched = idx >= 0
    usable = matched & np.isin(np.where(matched, est.status[np.maximum(idx, 0)], ""), USABLE)
    n = len(gt.t)
    p = np.full((n, 3), np.nan)
    p[usable] = est.pose[idx[usable]]
    pos_err = np.hypot(p[:, 0] - gt.pose[:, 0], p[:, 1] - gt.pose[:, 1])
    head_err = np.abs(_wrap(p[:, 2] - gt.pose[:, 2]))
    mirror = np.column_stack([-gt.pose[:, 0], -gt.pose[:, 1], _wrap(gt.pose[:, 2] + math.pi)])
    d_true = np.hypot(pos_err, cfg.heading_scale * head_err)
    d_mir = np.hypot(
        np.hypot(p[:, 0] - mirror[:, 0], p[:, 1] - mirror[:, 1]),
        cfg.heading_scale * np.abs(_wrap(p[:, 2] - mirror[:, 2])),
    )
    wrong = usable & (d_mir < d_true)
    accurate = usable & (pos_err <= cfg.pos_threshold_m) & (np.degrees(head_err) <= cfg.heading_threshold_deg)

    conf = np.full(n, np.nan)
    if est.side_confidence is not None:
        conf[matched] = est.side_confidence[idx[matched]]
    has_conf = usable & ~np.isnan(conf)
    confidently_wrong = wrong & has_conf & (conf >= cfg.confident_side)

    pe, he = pos_err[usable], np.degrees(head_err[usable])
    ep_count, ep_longest = _episodes(wrong, gt.t)
    lat = None
    if est.published is not None:
        pub = est.published[idx[usable]]
        lat_ms = 1000.0 * (pub - est.t[idx[usable]])
        lat = lat_ms[~np.isnan(lat_ms)]
    offsets = np.abs(est.t[idx[matched]] - gt.t[matched]) if matched.any() else np.zeros(0)

    m = {
        "samples": n,
        "accuracy_within_threshold": float(accurate.sum() / n),
        "availability": float(usable.sum() / n),
        "pos_p50_m": _pct(pe, 50),
        "pos_p95_m": _pct(pe, 95),
        "pos_max_m": float(pe.max()) if len(pe) else None,
        "pos_rmse_m": float(np.sqrt(np.mean(pe**2))) if len(pe) else None,
        "heading_p50_deg": _pct(he, 50),
        "heading_p95_deg": _pct(he, 95),
        "wrong_half_fraction": float(wrong.sum() / usable.sum()) if usable.any() else None,
        "wrong_half_episodes": ep_count,
        "wrong_half_longest_s": ep_longest,
        "confidently_wrong_samples": int(confidently_wrong.sum()) if has_conf.any() else None,
        "side_confidence_mean_when_right": float(conf[has_conf & ~wrong].mean())
        if (has_conf & ~wrong).any()
        else None,
        "side_confidence_mean_when_wrong": float(conf[has_conf & wrong].mean())
        if (has_conf & wrong).any()
        else None,
        "latency_p50_ms": _pct(lat, 50) if lat is not None else None,
        "latency_p95_ms": _pct(lat, 95) if lat is not None else None,
        "alignment_offset_max_s": float(offsets.max()) if len(offsets) else None,
        "tolerance_s": cfg.tolerance_s,
        "pos_threshold_m": cfg.pos_threshold_m,
        "heading_threshold_deg": cfg.heading_threshold_deg,
    }

    events = []
    ev_times = [t for t, _ in gt.events] + [float(gt.t[-1]) + 1.0]
    for k, (t0, kind) in enumerate(gt.events):
        t1 = ev_times[k + 1]
        win = (gt.t >= t0) & (gt.t < t1)
        ok = usable & (pos_err < cfg.converge_tol_m)
        conv = None
        for i in np.flatnonzero(win):
            hold = win & (gt.t >= gt.t[i]) & (gt.t < gt.t[i] + cfg.converge_hold_s)
            if gt.t[i] + cfg.converge_hold_s > t1 + 1e-9:
                break
            if ok[hold].all():
                conv = float(gt.t[i] - t0)
                break
        events.append({"t": t0, "event": kind, "converge_s": conv})

    def chk(value, limit):
        return None if value is None else bool(value <= limit)

    conv_times = [e["converge_s"] for e in events]
    checks = {
        f"pos p50 ≤ {cfg.target_pos_p50_m} m": chk(m["pos_p50_m"], cfg.target_pos_p50_m),
        f"pos p95 ≤ {cfg.target_pos_p95_m} m": chk(m["pos_p95_m"], cfg.target_pos_p95_m),
        f"heading p95 ≤ {cfg.target_heading_p95_deg}°": chk(m["heading_p95_deg"], cfg.target_heading_p95_deg),
        "no wrong-half samples": None if m["wrong_half_fraction"] is None else ep_count == 0,
        f"every event converges ≤ {cfg.target_converge_s} s": None
        if not events
        else all(c is not None and c <= cfg.target_converge_s for c in conv_times),
    }
    samples = {
        "timestamp_s": gt.t,
        "matched_timestamp_s": np.where(matched, est.t[np.maximum(idx, 0)], np.nan),
        "usable": usable.astype(int),
        "gt_x_m": gt.pose[:, 0],
        "gt_y_m": gt.pose[:, 1],
        "gt_theta_rad": gt.pose[:, 2],
        "est_x_m": p[:, 0],
        "est_y_m": p[:, 1],
        "est_theta_rad": p[:, 2],
        "pos_error_m": pos_err,
        "heading_error_deg": np.degrees(head_err),
        "wrong_half": wrong.astype(int),
        "side_confidence": conf,
    }
    return EvalResult(name, m, events, checks, samples)


# ---------------------------------------------------------------- reports


def _fmt(v, digits=3) -> str:
    if v is None:
        return "N/A"
    if isinstance(v, bool):
        return "PASS" if v else "FAIL"
    if isinstance(v, float):
        return "N/A" if math.isnan(v) else f"{v:.{digits}f}"
    return str(v)


ROWS = [
    ("accuracy_within_threshold", "Accuracy within threshold"),
    ("availability", "Availability"),
    ("pos_p50_m", "Position p50 (m)"),
    ("pos_p95_m", "Position p95 (m)"),
    ("pos_max_m", "Position max (m)"),
    ("heading_p95_deg", "Heading p95 (°)"),
    ("wrong_half_fraction", "Wrong-half fraction"),
    ("wrong_half_episodes", "Wrong-half episodes"),
    ("wrong_half_longest_s", "Longest wrong-half (s)"),
    ("confidently_wrong_samples", "Confidently wrong samples"),
    ("side_confidence_mean_when_right", "Side confidence when right"),
    ("side_confidence_mean_when_wrong", "Side confidence when wrong"),
    ("latency_p50_ms", "Latency p50 (ms)"),
    ("latency_p95_ms", "Latency p95 (ms)"),
]


def report_markdown(
    results: list[EvalResult], title: str = "Localization evaluation", notes: str = ""
) -> str:
    names = [r.name for r in results]
    lines = [f"# {title}", ""]
    if notes:
        lines += [notes, ""]
    cfg = results[0].metrics if results else {}
    lines += [
        "Each ground-truth sample is matched to the nearest estimate within "
        f"{_fmt(cfg.get('tolerance_s'))} s (no interpolation). Accuracy = position ≤ "
        f"{_fmt(cfg.get('pos_threshold_m'), 2)} m and heading ≤ "
        f"{_fmt(cfg.get('heading_threshold_deg'), 1)}°, "
        "over all samples; missing estimates count as failures. Percentiles are over usable samples only. "
        "Wrong half = estimate nearer the mirrored truth.",
        "",
        "## Metrics",
        "",
        "| Metric | " + " | ".join(names) + " |",
        "|---|" + "---|" * len(names),
    ]
    for key, label in ROWS:
        lines.append(f"| {label} | " + " | ".join(_fmt(r.metrics.get(key)) for r in results) + " |")
    lines += ["", "## Target checks (docs/roadmap.md §3.3)", ""]
    check_names = list(results[0].checks) if results else []
    lines += ["| Check | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for c in check_names:
        lines.append(f"| {c} | " + " | ".join(_fmt(r.checks.get(c)) for r in results) + " |")
    lines += [
        "",
        "## Convergence after events",
        "",
        "| Run | t (s) | Event | Converged after (s) |",
        "|---|---|---|---|",
    ]
    for r in results:
        for e in r.events:
            lines.append(f"| {r.name} | {e['t']:.2f} | {e['event']} | {_fmt(e['converge_s'], 2)} |")
    return "\n".join(lines) + "\n"


def write_report(
    results: list[EvalResult], out_dir, title: str = "Localization evaluation", notes: str = ""
) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / "report.md").write_text(report_markdown(results, title, notes), encoding="utf-8")
    payload = {r.name: {"metrics": r.metrics, "events": r.events, "checks": r.checks} for r in results}
    (out / "results.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    for r in results:
        keys = list(r.samples)
        with (out / f"samples_{r.name}.csv").open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(keys)
            for i in range(len(r.samples["timestamp_s"])):
                w.writerow(_fmt(float(r.samples[k][i]), 6) for k in keys)
    return out / "report.md"


# ---------------------------------------------------------------- CLI


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m td_localization.eval", description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    ev = sub.add_parser("evaluate", help="score estimate CSVs against one ground-truth CSV")
    ev.add_argument("--gt", required=True)
    ev.add_argument("--est", required=True, action="append", help="estimate CSV (repeat to compare runs)")
    ev.add_argument("--name", action="append", help="run names, same order as --est")
    ev.add_argument("--out", required=True)
    ev.add_argument("--tolerance", type=float, default=EvalConfig.tolerance_s)
    su = sub.add_parser("suite", help="run the simulated scenario suite and report it")
    su.add_argument("--out", required=True)
    su.add_argument("--field", default="hsl_m_2026")
    su.add_argument("--noise", choices=("default", "harsh"), default="default")
    su.add_argument("--seed", type=int, default=1)
    su.add_argument("--only", action="append", help="scenario name(s) to run (default: all)")
    args = ap.parse_args(argv)

    if args.cmd == "evaluate":
        names = args.name or [Path(p).stem for p in args.est]
        if len(names) != len(args.est):
            ap.error("--name count must match --est count")
        gt = read_ground_truth(args.gt)
        cfg = EvalConfig(tolerance_s=args.tolerance)
        results = [evaluate(gt, read_estimates(p), n, cfg) for p, n in zip(args.est, names, strict=True)]
        print(f"wrote {write_report(results, args.out)}")
        return 0

    from td_localization.suite import run_suite  # needs td_field_model's simulator

    path = run_suite(args.out, field_name=args.field, noise=args.noise, seed=args.seed, only=args.only)
    print(f"wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
