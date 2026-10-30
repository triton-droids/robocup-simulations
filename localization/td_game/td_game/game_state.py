"""GameController state tracker for one robot (LOC-08). Standard library only.

Feed every received control message to `GameStateTracker.update(t, data)`. It returns the events that
matter to localization and behavior, and keeps the latest view of the game for this robot.

Events (`GameEvent.kind`):
- "initial": the game entered Initial (start, each half, after a timeout). Handlers place robots in their own
  half → localization re-seeds (SideBelief.on_initial).
- "penalized": this robot got a penalty. `detail` is the penalty name; `in_place` says whether it stays
  where it is. Localization should stop trusting motion and vision until the penalty ends.
- "reentry": a standard-removal penalty ended. By rule the robot stands on its own-half touchline level
  with the penalty mark, facing the field → SideBelief.on_penalty_reentry with both touchline candidates.
- "unpenalized_in_place": Motion in Set ended (the robot was never moved) → no re-seed.
- "replaced": a penalty given during Initial/Timeout ended (picked up for service, put back by a handler) →
  treat as "initial".
- "state": game state changed (`detail` = new state name). "half": first/second half (or the phase) changed.
- "gc_lost" / "gc_back": no valid control message for `timeout_s`, or messages resumed.

Penalty interpretation (HSL GameController v7, `actions/penalize.rs` + competition params; see the LOC-08
handoff):
- Motion in Set: served in place, removed automatically after 15 s.
- Penalty given in Initial/Timeout (pick-up for service): zero duration, robot is put back as in Initial.
- Sent off: permanent; no re-entry.
- Substitute → none: a substitute entering. ASSUMPTION: it enters like a re-entry (placed by a handler on the
  own-half touchline). Confirm against the 2027 rules.
- Everything else: standard removal → re-entry.

Packets are deduplicated and reordered packets dropped by `packetNumber` (u8, wraps). Monitor ("RGTD")
data must never reach this tracker; gc_data.parse_control rejects it by default.
"""

from __future__ import annotations

from dataclasses import dataclass

from td_game.gc_data import (
    PENALTY_MOTION_IN_SET,
    PENALTY_NAMES,
    PENALTY_NONE,
    PENALTY_SENT_OFF,
    PENALTY_SUBSTITUTE,
    PHASE_TIMEOUT,
    STATE_INITIAL,
    STATE_NAMES,
    STATE_PLAYING,
    STATE_READY,
    STATE_SET,
    ControlData,
    TeamInfo,
)

IN_PLACE_PENALTIES = {PENALTY_MOTION_IN_SET}
COUNTED_STATES = {STATE_READY, STATE_SET, STATE_PLAYING}  # team messages count against the budget here


@dataclass(frozen=True)
class GameEvent:
    t: float
    kind: str
    detail: str = ""
    in_place: bool = False


class GameStateTracker:
    def __init__(self, team_number: int, player_number: int, timeout_s: float = 3.0):
        if not 1 <= player_number <= 20:
            raise ValueError("player_number is 1-based (1..20)")
        self.team_number = team_number
        self.player_number = player_number
        self.timeout_s = timeout_s
        self.data: ControlData | None = None
        self.last_rx: float | None = None
        self.connected = False
        self._penalty_given_in_initial = False

    # ---------------------------------------------------------------- views

    @property
    def our_team(self) -> TeamInfo | None:
        return None if self.data is None else self.data.team(self.team_number)

    @property
    def state(self) -> int | None:
        return None if self.data is None else self.data.state

    @property
    def state_name(self) -> str:
        return "unknown" if self.data is None else STATE_NAMES.get(self.data.state, "?")

    @property
    def penalty(self) -> int:
        team = self.our_team
        return PENALTY_NONE if team is None else team.players[self.player_number - 1].penalty

    @property
    def penalized(self) -> bool:
        return self.penalty != PENALTY_NONE

    @property
    def is_goalkeeper(self) -> bool:
        team = self.our_team
        return team is not None and team.goalkeeper == self.player_number

    @property
    def our_kick(self) -> bool | None:
        """True if we take the next kick-off / set play, False if they do, None if nobody."""
        if self.data is None or self.data.kicking_team == 255:
            return None
        return self.data.kicking_team == self.team_number

    @property
    def message_budget(self) -> int | None:
        team = self.our_team
        return None if team is None else team.message_budget

    @property
    def budget_counts(self) -> bool:
        """Whether team messages sent now count against the budget (Ready, Set, Playing)."""
        return (
            self.data is not None
            and self.data.state in COUNTED_STATES
            and self.data.game_phase != PHASE_TIMEOUT
        )

    @property
    def active_teammates(self) -> int:
        """Players of our team that are not sent off or substitutes, capped at playersPerTeam."""
        team = self.our_team
        if team is None:
            return 1
        n = sum(1 for p in team.players if p.penalty not in (PENALTY_SENT_OFF, PENALTY_SUBSTITUTE))
        return max(1, min(n, self.data.players_per_team))

    # ---------------------------------------------------------------- input

    def check_timeout(self, t: float) -> list[GameEvent]:
        """Call periodically. Emits "gc_lost" once when no valid message arrived for timeout_s."""
        if self.connected and self.last_rx is not None and t - self.last_rx > self.timeout_s:
            self.connected = False
            return [GameEvent(t, "gc_lost")]
        return []

    def _is_new(self, data: ControlData) -> bool:
        if self.data is None:
            return True
        diff = (data.packet_number - self.data.packet_number) & 0xFF
        # Same number → duplicate. Small negative (≥ 128 forward) → stale or reordered. Allow a restart of the
        # GameController (numbers far off) only after the link was lost.
        return diff != 0 and (diff < 128 or not self.connected)

    def update(self, t: float, data: ControlData) -> list[GameEvent]:
        if data.true_data:
            raise ValueError("monitor (true data) messages must not be used by robots")
        events: list[GameEvent] = []
        if not self._is_new(data):
            return events
        if data.team(self.team_number) is None:
            # Not our game (another field's GameController on the same network).
            return events
        if not self.connected:
            events.append(GameEvent(t, "gc_back"))
            self.connected = True
        self.last_rx = t
        prev = self.data
        self.data = data

        if prev is None or prev.state != data.state:
            events.append(GameEvent(t, "state", STATE_NAMES.get(data.state, str(data.state))))
            if data.state == STATE_INITIAL:
                events.append(GameEvent(t, "initial", "timeout" if data.game_phase == PHASE_TIMEOUT else ""))
        if prev is not None and (prev.first_half != data.first_half or prev.game_phase != data.game_phase):
            events.append(GameEvent(t, "half", f"first_half={data.first_half} phase={data.game_phase}"))

        before = (
            PENALTY_NONE
            if prev is None
            else prev.team(self.team_number).players[self.player_number - 1].penalty
        )
        now = self.penalty
        if before == PENALTY_NONE and now != PENALTY_NONE:
            self._penalty_given_in_initial = data.state == STATE_INITIAL or data.game_phase == PHASE_TIMEOUT
            events.append(
                GameEvent(t, "penalized", PENALTY_NAMES.get(now, str(now)), now in IN_PLACE_PENALTIES)
            )
        elif before != PENALTY_NONE and now == PENALTY_NONE:
            name = PENALTY_NAMES.get(before, str(before))
            if before in IN_PLACE_PENALTIES:
                events.append(GameEvent(t, "unpenalized_in_place", name, True))
            elif self._penalty_given_in_initial or data.state == STATE_INITIAL:
                events.append(GameEvent(t, "replaced", name))
            else:
                events.append(GameEvent(t, "reentry", name))
            self._penalty_given_in_initial = False
        return events
