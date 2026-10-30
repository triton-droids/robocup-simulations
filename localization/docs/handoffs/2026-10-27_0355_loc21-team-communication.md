# Handoff: 2026-10-27 03:55, LOC-21 team communication

Previous: [2026-10-27_0327_loc08-gamecontroller.md](2026-10-27_0327_loc08-gamecontroller.md).

## Rules in force
- No GitHub writes.
- New files only in `localization_work/`.
- Team repos untouched (all five clones clean).
- End each run with a handoff here.

## What was added

### `td_game/team_comm.py`: `TeamComm(player)`, transport-agnostic
- `outgoing(t, msg, budget, counts, remaining_s, robots)` → packed bytes if the `BudgetPacer` allows, else None.
- `incoming(t, payload)` → drops our own echo and foreign or malformed payloads, and stores the latest message per
  teammate.
- `teammates(t)` returns fresh teammates (≤ 3 s).
- `team_ball(t)` picks the best shared ball by uncertainty after aging (1 m/s growth). It skips fallen robots and
  robots unsure of their side (< 0.6).

### `td_game/udp.py`
`TeamLink(..., send_port=None)`, so two endpoints can talk on one machine (the plan's "two laptops" test). On the
field it's still one port, 10000 + team.

### `td_localization/team_sim.py`
`run_team(fm, lf, [RobotSpec...])`:
- Every robot runs its own filter, side belief, `GameBridge` and `TeamComm`.
- Real packed messages go over `LossyNetwork` (default 5% loss, 50 ms delay).
- The **shared team budget** is decremented per send.
- The ball is visible within 6 m and a 70° field of view around the head yaw.
- This **replaces the scripted teammate oracle** from LOC-06.

### Suite
New `team_kidnap_cross` scenario (goalkeeper standing at our goal, a field player, player 3 kidnapped to its mirror
at t = 10 s). It's reported as `team_kidnap_cross_kidnapped_p3` and `team_kidnap_cross_teammate_p2`.

## Side-belief changes (`side.py`), found by the multi-robot simulation

1. **Time matching was too strict.**
   - Problem: a teammate's ball report had to be within 0.3 s of my latest sighting. With real visibility, 2 Hz
     reports and network delay, that almost never happened: 0 usable reports after the kidnap.
   - Fix: keep a 4 s history of my sightings and match each report with the sighting **nearest in time**,
     **in both directions**. A report that arrives before I see the ball stays pending and is matched later; each
     report is used once.
   - The gap may be up to 3 s. My σ is widened by 1 m/s × gap, so the evidence fades by itself for long gaps or
     near the center (direct and mirror positions are 2|ball| apart).
   - `GameBridge.max_ball_age`: 0.5 → 3.0 s.
2. **One confidently-wrong teammate could flip a healthy robot.**
   - Found in seed 3: kidnapped player 3, mirrored and confident, flipped healthy player 2.
   - Fix, a **consensus rule:** a flip needs evidence against my side from the **goalkeeper**, or from **≥ 2
     different teammates**, within 15 s.
   - Opponents-at-Set evidence counts as one source. The evidence is cleared after a flip or a re-seed.

## Results (simulation, M-Field; 30 s, 4 seeds, 3 robots)

| | Result |
|---|---|
| Budget | 177 messages in 30 s (≈ 2 Hz × 3, the per-robot cap); about 4% of deliveries lost |
| Healthy robots | **0 wrong flips and 0 wrong-half samples in all seeds** |
| Kidnapped robot | recovered in seeds 1 and 4 (9.4 s and 18.9 s after the kidnap). Seed 3: knows it's wrong (confidence 0.00 → no shots) but no flip (only one corroborating source in time). Seed 2: stays **confidently wrong** (center-circle kidnap, no evidence ever lined up in time). |
| Reference suite (`reports/suite_m_{default,harsh}_seed1/`) | `team_kidnap_cross_kidnapped_p3` converges in **9.37 s** (PASS ≤ 10 s); teammate p2 passes every check. **All earlier scenarios unchanged** (no regression). |
| Lag | p3 has 250 confidently-wrong samples (about 8 s) between the kidnap and its flip |

## Tests: 206 passing, lint and format clean

- **`td_game` (42):** new `test_team_comm.py` (6).
  - paced outgoing; incoming filtering and the teammate table; team-ball choice;
  - a 5-robot, 20-minute game **never touches the 200-message reserve** and uses > 60% of the budget;
  - **two real-socket endpoints exchange messages over loopback.**
- **`td_localization` (86):** new `test_team_sim.py` (9).
  - lossy network;
  - a pending report is matched by a later sighting; old sightings compared with widened σ;
  - **one teammate can't flip me; two can; the goalkeeper alone can;**
  - team kidnap recovers within 15 s with healthy robots never misled;
  - regression for the wrong-flip bug.
- **Existing tests updated** for the deliberate changes: stale-ball gap 1 → 4 s; the flip-timing test uses
  `min_flip_sources=0`; bridge ball-age 2 → 4 s; the suite test skips team entries.
- **`td_field_model` (78):** unchanged.

## How to run
```bash
cd <workspace>/localization_work
PYTHONPATH="td_game;td_game/test" ../.venv-loc/Scripts/python -m pytest -q td_game/test
PYTHONPATH="td_field_model;td_game;td_localization;td_localization/test" ../.venv-loc/Scripts/python -m pytest -q td_localization/test
PYTHONPATH=td_field_model ../.venv-loc/Scripts/python -m pytest -q td_field_model/test
PYTHONPATH="td_field_model;td_game;td_localization" ../.venv-loc/Scripts/python -m td_localization.eval suite --out reports/x
```
(PowerShell works if Bash's safety check is flaky.)

## Limits and open items
- **Recovery is slow and incomplete in simulation:** 2 of 4 seeds. It's limited by how often two robots see the
  ball at nearly the same time. Ideas:
  - tie the head-scan schedule to teammates' ball reports (look where the team ball is);
  - add teammate-pose evidence once robot detection exists;
  - raise the 2 Hz cap (the budget allows about 3 Hz with 3 robots).
- **The consensus rule trades speed for safety.** With no goalkeeper and only 2 robots on the field, nobody can
  ever be flipped by teammates (only 1 other source). Revisit if the team plays Foundation with 3 field players.
- Simulation simplifications: no occlusion by robots, no opponents, the game is always Playing, and every robot
  sees the same ideal ball track.
- **Field test (🔧) not done:** real Wi-Fi access point, broadcast on the field network, two Jetsons.

## Next steps (suggested)
1. Head control that looks toward the team ball when the robot is unsure of its side. That's behavior/LOC-23
   territory; it speeds up side recovery.
2. A real GameController and two-robot network test (LOC-08 and LOC-21 field checks).
3. Remaining plan tickets that need hardware or decisions: LOC-01/02/10/13/14/15.

## Open human decisions
- Confirm the substitute-entry assumption (LOC-08).
- Team-message rate: 2 Hz cap vs using more of the budget.
- Is the consensus rule (goalkeeper or ≥ 2 teammates) acceptable for the team's lineups?
- Unchanged: LOC-01 IR policy, LOC-02 `td_msgs`, leg-model source, head encoders, which `td_perception` code base,
  when to move `localization_work` into a repo.
