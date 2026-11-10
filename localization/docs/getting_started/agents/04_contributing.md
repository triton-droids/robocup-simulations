# 04 Contributing

## Workflow

1. **Read:**
   - the newest handoff in `docs/handoffs/`;
   - the ticket in `implementation_plan.md`;
   - roadmap §3.5;
   - the repo `CLAUDE.md`;
   - the files the ticket names.
2. **Check:**
   - the dependencies are done (plan §6);
   - no 🧑 decision blocks the ticket (if one does, write a decision memo instead);
   - nothing on the stop-and-ask list is involved.
3. **Branch** from `localization`: `loc-XX-short-desc`. Commit messages: `LOC-XX: what changed`. Never commit to
   `main` or `localization` directly.
4. **Test first:** add failing tests using synthetic data ([`ScenarioBuilder`](../../../td_field_model/td_field_model/scenarios.py) + [`Simulator`](../../../td_field_model/td_field_model/sim.py)), seeded.
5. **Implement the pure library** (numpy only, no ROS). Body-specific values go in config dataclasses with commented
   defaults.
6. **Verify:**
   - `python run_tests.py` is green (all four packages + ruff);
   - if PF, side belief, simulator or noise behavior changed: `python -m td_localization.eval suite` before and
     after, **≥ 2 seeds**, both noise presets; report the deltas;
   - if test-report content changed: `python test_report/make_results.py`.
7. **Document:**
   - update the plan §6 row;
   - update the roadmap if the design changed;
   - update `getting_started/` if behavior that's described there changed;
   - record anything a future agent needs in the ticket section of the plan.
8. **Hand off:** write a new `docs/handoffs/YYYY-MM-DD_HHMM_<topic>.md` (template below).
9. **PR (only if the user asks):** base `localization`, body from the template below.

## Done criteria

- [ ] Tests cover the new behavior; all suites pass; ruff clean.
- [ ] No regression in the suite (or the regression is explained and accepted by a human).
- [ ] No invariant from `01_orientation.md` is weakened.
- [ ] Plan §6 updated; handoff written.
- [ ] Claims about verification are exact (which seeds, which Python, what was not run).

## Stop and ask a human

Ask before:
- changing a §9 team decision (`min_flip_sources`, `flip_sources_window`, the IR projector policy);
- `td_msgs` changes;
- motor, servo, CAN or safety config;
- enabling the D435 emitter;
- ROS / JetPack / Isaac ROS / Docker upgrades;
- rules interpretation, or anything sent to the TC;
- edits under `third-party/`;
- merging or rebasing `p-33-ball-tracker`;
- deleting packages;
- `test_frozen` data;
- unclear hardware state (wiring, encoder choice, offsets);
- any GitHub write the user didn't explicitly request.

## Handoff template

```markdown
# Handoff: <topic>, YYYY-MM-DD HH:MM

## Request
<the user's words, quoted>

## What changed
<files and what each change does; ticket IDs>

## Verified
<exact commands and results: test counts per package, suite numbers with seeds, Python version>

## Not verified / limits
<hardware, other Python versions, seeds not run, assumptions>

## Next
<concrete next steps; open decisions for humans>
```

## PR template

```markdown
## What changed
<summary; LOC-XX>

## How it was tested
- `python run_tests.py`: <counts per package>, ruff clean
- Suite before/after (seeds …, default/harsh): <key rows>

## Docs
- Plan §6 row, roadmap / getting_started updates

## Follow-ups / decisions
<human decisions, hardware steps>
```

## Gotchas

| Gotcha | Detail |
|---|---|
| Duplicate `test_import.py` | Every package has one, so pytest can't collect two packages in one run. Use `run_tests.py`, or one package per pytest call. |
| `PYTHONPATH` | The packages aren't installed. The separator is `:` (POSIX) or `;` (Windows). `td_game` and `td_localization` tests also need their `test/` folder on the path (helper modules `gc_fixtures`, `helpers`, `team_harness`). |
| `pytest.ini` | It exists in `localization/` to stop pytest from using the repo-root `pyproject.toml`. Don't remove it. |
| [`ParticleFilter`](../../../td_localization/td_localization/pf.py) args | `ParticleFilter(fm, likelihood=None, config=None, seed=0)`: pass `config=` by keyword. |
| Ruff isort | `td_field_model` may sort as first-party in a separate block; accept `ruff check --fix` output. |
| `ruff.toml` | Local config (110, py310) overrides the repo root (100). The repo CI only lints `robocup_rl` and `tests`. |
| Real-time bench test | `test_everything_but_camera_live_odometry_and_udp` is wall-clock sensitive. Rerun once before debugging a failure. |
| Windows firewall | UDP loopback tests may trigger a prompt; they pass either way. |
| `results/` name | Forbidden outside experiment dirs (repo `CLAUDE.md`). The report folder is `test_report/`. |
| Reference reports | `reports/suite_m_*_seed1/{report.md,results.json}` are committed. Regenerate them with `run_suite`; never hand-edit. Other `reports/*` content is ignored. |
| Test report numbers | Bench and vision tests log values via `show(record_property, …)` (xunit1 JUnit properties). `make_results.py` runs td_bench in-process to capture runs for plots. |
| Line endings | Files are LF in git; Windows `autocrlf` warnings are harmless. |
| Old handoffs | Notes before 2026-10-29 use workspace paths (`<workspace>/localization_work/…`, `results/`, `docs/localization_*.md`); see `docs/handoffs/README.md` for the mapping. |
| Markdown links | Link the first mention of an in-repo class to its defining file (relative path, no `#L` anchors). Names in code blocks stay plain. |
