# Handoff: 2026-10-27 04:58, side consensus rule recorded as a team decision

Previous: [2026-10-27_0355_loc21-team-communication.md](2026-10-27_0355_loc21-team-communication.md).
Rules in force: no GitHub writes; new files only in `localization_work/`; team repos untouched.

## Decision (from the user)
**Keep the side consensus rule as is.** A robot flips halves only on evidence from the goalkeeper or from ≥ 2
different teammates within 15 s. Accepted cost: with no goalkeeper and only one teammate, teammates can't flip a
robot. It stays unsure (no shots) until a game event re-seeds it.

## Where it's recorded
- `docs/localization_implementation_plan.md` **§9 "Team decisions (log)"** (new section). It also back-fills the two
  earlier user decisions (local-only work rules, IR policy). Agents must not change logged decisions without
  asking.
- `docs/localization_roadmap.md` §3.5, under "Teammates, with the goalkeeper as anchor".
- `td_localization/side.py`: a comment on `SideConfig.min_flip_sources` points to the log. No behavior change;
  lint is clean.

## Open human decisions (remaining)
- Substitute-entry assumption (LOC-08).
- Team-message rate: 2 Hz cap vs using more of the budget (LOC-21).
- LOC-01 IR policy TC confirmation; LOC-02 `td_msgs`; leg-model source; head encoders; which `td_perception` code
  base; when to move `localization_work` into a repo.

## Next steps (unchanged from the LOC-21 handoff)
Head control toward the team ball when unsure of the side; real GameController and two-robot network test; the
hardware/decision tickets.
