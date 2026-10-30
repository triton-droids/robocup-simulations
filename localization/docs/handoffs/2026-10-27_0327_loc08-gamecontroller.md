# Handoff: 2026-10-27 03:27, LOC-08 GameController client

Previous: [2026-10-27_0302_drift-fix.md](2026-10-27_0302_drift-fix.md).

## Rules in force
- No GitHub writes.
- New files only in `localization_work/`.
- Team repos untouched (all clean).
- End each run with a handoff here.

**Power loss during this run:** the final test step was interrupted. On resume, all `.py` files compiled with no
truncation or NUL bytes, and every suite was re-run green (numbers below).

## Reference source
Cloned the official **HSL GameController** (public, read-only) into `CustomHumanoid/GameController/` at
`a6cc98b` (2026-10-12, v7 line).
- Wire format: `game_controller_msgs/headers/RoboCupGameControlData.h`.
- Byte order from `src/control_message.rs` / `status_message.rs`, cross-checked with
  `dissectors/robocup_gc.lua`.
- Penalty behavior from `game_controller_core/src/actions/penalize.rs` and `config/large_*/params.yaml`.

## What was added

### `localization_work/td_game/` (new package, standard library only)

- **`gc_data.py`:** control message (`RGme`, struct v20, **158 B**, packed little-endian) and status message (`RGrt`,
  v4, **32 B**) parse/build.
  - Validates like the GameController and rejects monitor `RGTD` data by default.
  - `status_from_localization()` converts m → mm. **Our `map` frame (+x toward the goal we attack) is exactly the
    GameController's pose frame.**
- **`game_state.py`:** `GameStateTracker(team, player)`.
  - Dedupes and drops stale packets by u8 `packetNumber` (handles wraparound), and ignores other fields'
    GameControllers.
  - Events:
    - `initial` (including timeout), `state`, `half`;
    - `penalized` (with `in_place`), `reentry`, `unpenalized_in_place`, `replaced`;
    - `gc_lost` / `gc_back` (3 s timeout).
  - Views: `penalized`, `is_goalkeeper`, `our_kick`, `message_budget`, `budget_counts`
    (Ready/Set/Playing only), `active_teammates`.
- **`team_msg.py`:** `TeamMessage` packs into **22 B**: pose (cm, 1e-4 rad), side confidence, ball in the field frame
  plus sigma and age, and goalkeeper/fallen/localized flags.
  - `BudgetPacer` is a token bucket driven by the GameController's authoritative `messageBudget`. It applies a
    per-robot share, a 200-message reserve, a 0.9 safety factor and a 2 Hz cap.
  - `remaining_game_seconds()` adds a 15% Ready/Set allowance.
- **`udp.py`:** non-blocking `GameControllerLink` (receive on 3838; reply with status unicast to the sender on
  3939) and `TeamLink` (broadcast on 10000 + team).

### `td_localization/game_bridge.py`

`GameBridge(side, field, start_poses)`:
- `initial` / `replaced` → `side.on_initial` (known spots or own-half search);
- `penalized` (not in place) → `paused = True` (the node must stop feeding the filter);
- `reentry` → `side.on_penalty_reentry` with both own-half touchline candidates;
- `team_report()` turns a teammate's `TeamMessage` into a side-belief `TeamReport` (fresh, seen balls only).

## Penalty mapping (important; please review)

| GameController penalty | Treated as |
|---|---|
| Motion in Set | in place (15 s, auto-removed): no re-seed |
| Any penalty given during Initial/Timeout (pick-up for service) | `replaced` → re-seed like Initial |
| Sent off | permanent, no re-entry |
| Substitute → none | **ASSUMPTION:** enters like a re-entry (placed on the own-half touchline). Confirm against the 2027 rules. |
| All others (illegal positioning, pushing, pick-up in play, …) | standard removal → `reentry` |

Re-entry fires when the penalty *ends* (the robot was placed on the touchline before the timer started).

## Bugs and findings this run
1. **Side-belief bug (fixed):** after a re-seed from several candidates (both touchlines), the estimate switching
   to the correct one triggered the jump detector and capped confidence at 0.73. Re-seeds now get the same 2 s
   grace period as leaving lost mode (`side._set`). There's a new test,
   `test_no_jump_cap_while_settling_after_a_reseed`, and `test_jump_caps_confidence` now jumps after the grace
   period.
2. **Pacer usage:** with 3 robots the **2 Hz per-robot cap binds** (≤ 7,200 of 12,000 messages). With 5 robots the
   budget share binds (about 78% of spendable). If LOC-21 wants more messages per robot, raise `max_rate_hz`.
   It's conservative on purpose, because overspending nullifies goals.

## Tests (all green after the power-loss re-run; lint and format clean across 3 packages)

| Package | Tests | What they cover |
|---|---|---|
| `td_game` | 36 | sizes; byte offsets from the dissector on raw bytes; round trips; validation; packet dedupe and wrap; every event path; link loss; budget sim for 3 and 5 robots; UDP loopback |
| `td_localization` | 77 | includes 5 bridge tests; end-to-end: Pushing via GameController packets → paused → touchline → re-entry converges on the correct touchline within 3 s, confidence > 0.9, 0 flips |
| `td_field_model` | 78 | – |

## How to run
```bash
cd <workspace>/localization_work
PYTHONPATH="td_game;td_game/test" ../.venv-loc/Scripts/python -m pytest -q td_game/test
PYTHONPATH="td_field_model;td_game;td_localization;td_localization/test" ../.venv-loc/Scripts/python -m pytest -q td_localization/test
PYTHONPATH=td_field_model ../.venv-loc/Scripts/python -m pytest -q td_field_model/test
../.venv-loc/Scripts/ruff check --config ../td_perception/ruff.toml td_game td_localization td_field_model
```
(Bash's safety check was flaky earlier this session; PowerShell runs the same commands.)

## Not done or not verified
- **Never run against a real GameController.** Next: build the GameController (`cargo run`, needs Rust plus a
  frontend build) or use its C API, and point `GameControllerLink` at it. On a Jetson, check firewall rules,
  interface choice and broadcast vs the limited-broadcast option.
- No ROS node yet (LOC-20 wires tracker + bridge + filter; LOC-21 wires `TeamLink` + `BudgetPacer`).
- Not handled: the "delayed state" after Playing (the GameController holds the old state up to 10 s; the robot must
  hear the whistle itself), penalty shoot-out behavior, opponents-at-Set cue wiring (`side.on_opponents_at_set`
  exists).

## Next steps
1. **LOC-21 team communication:** the loop `TeamLink` + `BudgetPacer` + `pack/unpack` + `GameBridge.team_report` →
   `side.on_team_report`, plus a multi-robot simulation via the suite.
2. Test against a real GameController build.
3. Optional: the remaining 1/8 drift cases (drift-fix handoff).

## Open human decisions
- **New:** confirm the substitute-entry assumption, and whether the team wants more than 2 Hz team messages.
- Unchanged: LOC-01 IR policy, LOC-02 `td_msgs`, leg-model source, head encoders, which `td_perception` code base to
  keep, when to move `localization_work` packages into a repo.
