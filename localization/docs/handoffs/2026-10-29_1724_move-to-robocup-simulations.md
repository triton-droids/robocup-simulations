# Handoff: localization prepared for robocup-simulations (branch `localization`), 2026-10-29 17:24

## Request

"Add docs, a readme, and gitignore, prepare this folder to be pushed to tritondroid/robocup-simulations under a
new branch: 'localization'". The repo is `triton-droids/robocup-simulations`.

## What was done

- **Staging clone:** the branch was built in a separate clone, `<workspace>/push_staging/robocup-simulations`.
  The team clone `<workspace>/robocup-simulations` was not touched.
  - Origin is set to `https://github.com/triton-droids/robocup-simulations.git`.
  - Branch `localization` was cut from `origin/main` @ `4bfbefa`, with no upstream set.
  - **One local commit. Not pushed.**
- **Folder:** `localization/` at the repo root. The only change outside it is one row in the root `README.md`
  layout table.
- **Renamed for this repo:**
  - `results/` → `test_report/`. The repo's `CLAUDE.md` forbids `results/` outside experiment directories.
  - `docs/localization_roadmap.md` → `docs/roadmap.md`.
  - `docs/localization_implementation_plan.md` → `docs/implementation_plan.md`.
  - `handoffs/` → `docs/handoffs/`.
  - References in code comments and docs were updated to match.
- **New files:**
  - `README.md`: what it is, status, layout, setup, results, bench, evaluation, known limits, agent rules.
  - `docs/README.md`: docs index and the rules that constrain the design.
  - `docs/handoffs/README.md`: maps the old workspace paths to this folder.
  - `test_report/README.md`.
  - `.gitignore`: caches, venvs, `test_report/raw/`, `runs/`, `*.npz`, and `reports/*` except the two
    reference summaries.
  - `ruff.toml`: the td_perception settings (110, py310), which override the repo root's 100.
  - `pytest.ini`: stops pytest from using the root pyproject.
  - `requirements.txt`.
  - `run_tests.py`: runs every package separately, then ruff.
- **Plan updates:**
  - §0.1a: the local-only rules are replaced by "where this code lives now".
  - §9: a 2026-10-29 decision for the move; the old no-GitHub rule is marked superseded.
  - §6: the "not in any repo" status cells now name this branch.
- **Left out:**
  - Suite CSVs (3.6 MB, regenerable).
  - Figure PNGs; the docs now embed the SVGs, which GitHub renders.
  - `test_report/raw/`.
  - Caches.
- **Handoffs:** older handoffs are kept as history, with absolute paths replaced by `<workspace>` /
  `<scratchpad>`.

## Verified in the staging copy

- `python run_tests.py`: td_field_model 78, td_game 42, td_localization 86 and td_bench 31 tests pass, and ruff
  is clean.
- Both HTML pages were regenerated from the staging copy: 237/237, and the visual-only numbers match the earlier
  run.
- Not verified: Python 3.11 / numpy 1.26. Only Python 3.10 with numpy 1.21.5 is installed on this machine.

## To push (a human runs this)

```bash
cd <workspace>/push_staging/robocup-simulations
git push -u origin localization
```

## Open points

- The `package.xml` maintainer is `parthtttrivedi@gmail.com` ("Triton Droids Perception"), copied from
  td_perception's convention. Confirm or change it before review.
- The repo's CI (`ci.yml`) neither lints nor tests `localization/`. Adding a job (Python 3.10 + numpy 1.21.5 +
  `python run_tests.py`) would be a separate change to `.github/`.
- The final home is still open: td_perception (ROS packages) or here.
- From now on, `<workspace>/localization_work/` and this branch can diverge. Pick one as the source of truth;
  after the push, the branch should be it.
