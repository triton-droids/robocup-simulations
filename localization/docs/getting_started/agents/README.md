# Agent track

Concise reference for AI agents (and developers who want the facts fast). The in-depth explanations are in the
[human track](../humans/01_welcome.md).

## Read order

1. This file: the hard rules.
2. [01_orientation.md](01_orientation.md): layout, environment, commands, conventions, invariants.
3. [02_architecture.md](02_architecture.md): modules, APIs, contracts, loop order, parameters.
4. [03_status.md](03_status.md): what's done, measured results, limits, ready work.
5. [04_contributing.md](04_contributing.md): workflow, done criteria, stop-and-ask, templates, gotchas.
6. The newest file in [`docs/handoffs/`](../../handoffs/).
7. For a ticket: its section in [`implementation_plan.md`](../../implementation_plan.md), and roadmap §3.5.

## Hard rules

1. **No GitHub writes** (push, PR, comments, branches) unless the user explicitly asks for that specific action.
2. **Libraries stay pure:** numpy only (`td_game`: stdlib only), no ROS imports. ROS nodes are thin wrappers (LOC-20).
3. **The particle filter never chooses the half.** Side is decided only by [`SideBelief`](../../../td_localization/td_localization/side.py) (game events, teammates).
   Don't add any code path that mirrors particles or moves `side_reference` outside the existing mechanisms.
4. **Team decisions are fixed** (plan §9): side consensus `min_flip_sources = 2` within 15 s or the goalkeeper; D435
   projector off. Ask before changing either.
5. **Stop and ask** before:
   - `td_msgs` changes, motor/servo commands, or safety config;
   - version upgrades;
   - rules interpretation;
   - editing `third-party/`;
   - changing a decision in plan §9.
6. **No `results/` or `output/` directories** outside `robocup_rl/experiments/<name>/` (repo `CLAUDE.md`).
7. **Every change:**
   - `python run_tests.py` is green, ruff included;
   - behavior changes are judged with the suite across ≥ 2 seeds;
   - plan §6 is updated.
8. **End every run with a new timestamped handoff** in `docs/handoffs/YYYY-MM-DD_HHMM_<topic>.md`. Never edit old
   ones. Near a usage limit, write a short one immediately.
9. **Markdown:** link the first mention of an in-repo class to its defining file (relative path, no line anchors).
10. **Report faithfully:** state what was verified and what wasn't (hardware, Python 3.11, seeds).
