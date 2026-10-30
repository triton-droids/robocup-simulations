# INTERIM handoff: 2026-10-27 02:15, drift fix in progress (shell checks failing)

Previous: [2026-10-27_0139_loc07-evaluation.md](2026-10-27_0139_loc07-evaluation.md).
Rules: no GitHub writes; new files only in `localization_work/`; team repos untouched.

## State
- **`td_localization/pf.py`:** added the local drift test (`_drift_test`, `drift` property, `UpdateStats.drift`).
  - Each informative frame, score 200 local candidates (σ 0.6 m, 0.25 rad) on 40 points.
  - Fire when the per-point gap is > 0.5 on 3 of the last 4 frames; inject the top 5% local candidates.
- **`suite.py`:** status is "uncertain" while `pf.drift`.
- **Verified with the first version** (single-frame margin 15 = 0.375 per point): fixed the seed-1 drift (1.3 m → 0.16
  m within 0.8 s), BUT it caused an accuracy **regression** from false alarms. M harsh p95 went 0.26 → 0.38; L harsh
  0.60 → 0.90.
- **NOT yet verified:** the current per-point + persistence version (chosen from measured gap distributions: clean
  p99 0.4–0.7, drifted median 0.7–1.6).
- **Tests edited, NOT yet run:**
  - `test_pf.py`: same-half kidnap is now parametrized over seeds 1/4/5 with `kidnap_same_half`; new
    `test_drift_test_fires_on_offset_cloud_not_on_good_one`.
  - `test_side.py`: cross-field kidnaps now use `kidnap_to_mirror` and assert exactly 1 flip.

## Next commands (scratchpad scripts; recreate if gone)
```bash
cd <workspace>/localization_work
S=<scratchpad>
../.venv-loc/Scripts/python -I $S/drift_check.py   # same-half kidnap convergence across seeds + false-alarm rate
../.venv-loc/Scripts/python -I $S/sweep_pf.py      # must match pre-fix: M def p95 ≤0.05, M harsh ≤0.26, L def ≤0.13, L harsh ≤0.60
PYTHONPATH="td_field_model;td_localization;td_localization/test" ../.venv-loc/Scripts/python -m pytest -q td_localization/test
```

If the sweep still regresses:
- raise `drift_margin_per_point`, or require 4/4 votes;
- or inject fewer particles.

Then lint, re-run the suite reports, and write the final handoff.
