# test_report

Generated test pages for the localization packages, with each test shown beside its result. Both pages are
self-contained (images and plots are inline) and work offline in any browser. GitHub shows them as source, so
download them or open them from a clone.

| File | What |
|---|---|
| [index.html](index.html) | Every test, from all four packages (237). See below. |
| [visual_only.html](visual_only.html) | The visual-only test and four variants, each on 3 seeds × 30 s. See below. |
| [make_results.py](make_results.py) | Regenerates `index.html` (about 4 minutes; the td_localization suite is the slow part). |
| [visual_only.py](visual_only.py) | Regenerates `visual_only.html` (about 8 minutes). |
| `raw/` | JUnit XML, logs and `visual_only.json` from the last run (git-ignored). |

**`index.html`** contains:
- **Every test:** its docstring and assertions (expand for the full code) beside its outcome, time and any
  measured values.
- **A card per bench run:** which inputs are real, simulated or recorded; measured values against the pass
  conditions; the estimated path against ground truth on the field; and error over time.
- **The vision check:** a camera frame beside the line points detected from it, plus the same frame with a head
  pitch 6° wrong.

**`visual_only.html`** covers camera images + the cuVSLAM model + encoder/IMU head angles, and four variants
that isolate head-scan speed, head-angle error, no odometry at all, and legs instead of VO. The plots show each
variant's worst seed.

```bash
cd localization
python test_report/make_results.py
python test_report/visual_only.py
```

Where the measured values come from:
- Bench and vision tests call `show(record_property, check, measured, expected)` (from
  [`td_bench/test/bench_report.py`](../td_bench/test/bench_report.py)), and pytest writes these into the JUnit
  XML.
- `make_results.py` runs the td_bench tests in-process and keeps every bench run, so the plots come from the same
  runs the tests checked.

Everything here is simulation or loopback UDP. cuVSLAM and the encoder/IMU head angles are models
([`SimVisualOdometry`](../td_bench/td_bench/sources.py), `ImuHead`); no robot hardware was involved.
