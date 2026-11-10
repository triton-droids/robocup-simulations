# Handoff: getting-started documentation series, 2026-11-10 12:51

## Request

"Create a getting started documentation series for agents and humans, with the human documentation being extremely
in depth and assuming the reader has never worked on the type of project before (explain codebase and math,
breaking it down into simple, understandable steps). The agent documentation should be precise and straight to the
point (concise). Both docs should introduce how to contribute and explain current progress. These docs should be
linked to the readme"

## What changed

Work was done in `<workspace>/push_staging/robocup-simulations` on branch `localization`. **Not committed, not
pushed.** The remote branch was still at `4d353ef` when checked (2026-11-10).

New `localization/docs/getting_started/` (about 3,000 lines):

- `README.md`: the series index, with both tracks.
- `humans/`, twelve chapters:
  1. `01_welcome`
  2. `02_setup` (including a numpy primer)
  3. `03_math_geometry` (frames, radians, rotation, compose, mirror, pose distance)
  4. `04_math_probability` (Gaussian, quadrature, covariance, Bayes, Cauchy kernel, log-odds, EMA, percentiles)
  5. `05_field_model`
  6. `06_camera` (pinhole, back-projection, the (h²+d²)/h error, covariances, simulator, detector)
  7. `07_particle_filter` (predict, update, tempering, ESS and low-variance resampling, recovery, drift test, lost
     mode, folding, clustering)
  8. `08_side_belief` (LLR derivation, gating, consensus, limits)
  9. `09_game_and_team` (GameController, events, 22 B message, budget math)
  10. `10_testing_and_evaluation`
  11. `11_progress`
  12. `12_contributing`

  Each chapter has worked examples, code excerpts, a "Try it" section, and a summary.
- `agents/`, five pages:
  - `README` (read order + 10 hard rules);
  - `01_orientation` (layout, environment, commands, conventions, invariants);
  - `02_architecture` (class index, data flow, per-tick loop, API tables, contracts, parameters, extension points);
  - `03_status`;
  - `04_contributing` (workflow, done criteria, stop-and-ask, handoff and PR templates, gotchas).

Linked from:
- `localization/README.md`: a new "Getting started" section, and the contributors section;
- `localization/docs/README.md`: the first row of its table.

Also fixed stale `results/` paths in `docs/implementation_plan.md` §6 (the LOC-19 and LOC-25 rows) → `test_report/`.

## Verified

- Every number in the docs was computed with the real code (scratch scripts), not written from memory.
- **Every runnable Python block** (the 10 blocks with imports, in chapters 2–10) runs without error from
  `localization/`. Outputs match the comments in the docs; the particle-filter kidnap and side-flip exercises match
  their quoted output exactly.
- CLI commands checked:
  - `python -m td_bench --duration 10 --start known`;
  - `--start search`;
  - record → replay (identical);
  - `python -m td_localization.eval suite` (≈80 s);
  - `evaluate` on the first-run CSVs.
- **All relative links resolve:** 26 markdown files checked.
- **Repo markdown rule** (link the first mention of an in-repo class to its defining file): checked with a script.
  The only remaining hits are English words ("Bench harness", "Step by step", "Teammate's ball", the "Estimate:"
  heading). `agents/02_architecture.md` uses a linked class index at the top, because many class names appear
  inside call signatures.

## Not verified / notes

- The docs render on GitHub with ```` ```math ```` and `mermaid` blocks; this wasn't previewed on github.com.
- Python 3.11 is still unverified (only 3.10 is installed here).
- Branch naming (`loc-XX-short-desc`, PRs into `localization`) is documented as following `td_perception`'s existing
  convention. It's a suggestion, not a recorded team decision.

## Finding (small)

`BudgetPacer` (`td_game/team_msg.py`) caps its credit at 1, so the overshoot on each sending tick is lost. At a 30 Hz
loop and 1.54 Hz it sends 90 messages a minute instead of 92 (about 2% under target).
- This is the safe direction, but the code comment claims it "doesn't round intervals up".
- The docs describe the real behavior.
- A fix would keep the remainder, e.g. cap the credit at 1 + (rate × dt) instead of 1; if changed, update
  `test_team_comm`/`test_team_msg` expectations.

## Next

- Commit and push the docs (needs the user's OK).
- Optionally add `localization/CLAUDE.md` pointing agents at `docs/getting_started/agents/README.md`, so it's picked
  up automatically.
- Keep `humans/11_progress.md` and `agents/03_status.md` in sync with plan §6 as tickets move.
