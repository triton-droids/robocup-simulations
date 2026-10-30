"""Team messages: compact packing and a budget pacer (LOC-08, groundwork for LOC-21). Standard library only.

Rules (HSL 2026, GameController v7):
- UDP broadcast on port 10000 + team number through the field access point, payload ≤ 512 bytes.
- 12,000 messages per team per game (+600 per added minute, +6,000 for extra time). Only messages sent in
  Ready, Set and Playing count. Exceeding the budget nullifies the team's goals.
- The GameController publishes the remaining budget per team in every control message
  (`TeamInfo.messageBudget`), so the pacer uses that number as the truth.

`TeamMessage` packs what localization (LOC-06) and behavior need into 22 bytes:
pose, side confidence, ball in the field frame with its age and uncertainty, and role/fall/localized flags.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass

MAX_PAYLOAD = 512
MAGIC = b"TDLc"
VERSION = 1
_FMT = struct.Struct("<4sBBBB3hBB2hBB")  # 4+1+1+1+1+6+1+1+4+1+1 = 22
FLAG_GOALKEEPER, FLAG_FALLEN, FLAG_LOCALIZED, FLAG_BALL_SEEN = 1, 2, 4, 8


@dataclass(frozen=True)
class TeamMessage:
    player_number: int
    pose: tuple[float, float, float]  # field frame (own goal at -x), m, m, rad
    pose_sigma: float  # m
    side_confidence: float  # 0..1
    ball_xy: tuple[float, float] | None  # field frame, m; None if not seen
    ball_sigma: float = 0.3  # m
    ball_age: float = 0.0  # s since last seen
    is_goalkeeper: bool = False
    fallen: bool = False
    localized: bool = True


def _clip(v, lo, hi):
    return max(lo, min(hi, v))


def pack(m: TeamMessage) -> bytes:
    """Encode to 22 bytes.

    Positions in cm (int16), angle in 1e-4 rad, sigmas in cm (≤ 2.55 m), ages in 0.1 s.
    """
    flags = (
        (FLAG_GOALKEEPER if m.is_goalkeeper else 0)
        | (FLAG_FALLEN if m.fallen else 0)
        | (FLAG_LOCALIZED if m.localized else 0)
        | (FLAG_BALL_SEEN if m.ball_xy is not None else 0)
    )
    x, y, th = m.pose
    th = math.atan2(math.sin(th), math.cos(th))
    bx, by = m.ball_xy if m.ball_xy is not None else (0.0, 0.0)
    data = _FMT.pack(
        MAGIC,
        VERSION,
        m.player_number,
        flags,
        int(round(_clip(m.side_confidence, 0.0, 1.0) * 255)),
        int(round(_clip(x * 100, -32767, 32767))),
        int(round(_clip(y * 100, -32767, 32767))),
        int(round(th * 1e4)),
        int(round(_clip(m.pose_sigma * 100, 0, 255))),
        int(round(_clip(m.ball_age * 10, 0, 255))),
        int(round(_clip(bx * 100, -32767, 32767))),
        int(round(_clip(by * 100, -32767, 32767))),
        int(round(_clip(m.ball_sigma * 100, 0, 255))),
        0,  # reserved
    )
    assert len(data) <= MAX_PAYLOAD
    return data


def unpack(data: bytes) -> TeamMessage:
    """Decode. Raises ValueError for anything that isn't one of our messages.

    Other teams share the network.
    """
    if len(data) != _FMT.size:
        raise ValueError("not a TDLc team message (length)")
    magic, ver, player, flags, conf, x, y, th, ps, age, bx, by, bs, _ = _FMT.unpack(data)
    if magic != MAGIC or ver != VERSION:
        raise ValueError("not a TDLc team message (magic/version)")
    seen = bool(flags & FLAG_BALL_SEEN)
    return TeamMessage(
        player_number=player,
        pose=(x / 100, y / 100, th / 1e4),
        pose_sigma=ps / 100,
        side_confidence=conf / 255,
        ball_xy=(bx / 100, by / 100) if seen else None,
        ball_sigma=bs / 100,
        ball_age=age / 10,
        is_goalkeeper=bool(flags & FLAG_GOALKEEPER),
        fallen=bool(flags & FLAG_FALLEN),
        localized=bool(flags & FLAG_LOCALIZED),
    )


class BudgetPacer:
    """Spreads the team's remaining message budget over the remaining counted game time.

    Each robot gets an equal share. Call `allow(t, budget, counts, remaining_s, robots)` before each send;
    it returns True if this robot may send now (then the caller sends).
    - budget: our team's messageBudget from the latest control message (authoritative; includes
      teammates' sends).
    - counts: whether sending now counts (GameStateTracker.budget_counts). Outside Ready/Set/Playing,
      messages are free and limited only by `max_rate_hz`.
    - remaining_s: estimate of counted time left in the game, see `remaining_game_seconds`.
    `reserve` messages are never spent, so a late extra-time or estimate error can't push the team over.
    """

    def __init__(self, max_rate_hz: float = 2.0, reserve: int = 200, safety: float = 0.9):
        self.max_rate_hz = max_rate_hz
        self.reserve = reserve
        self.safety = safety
        # Token bucket: credit accrues at the current rate, capped at one message, so the caller's tick rate
        # doesn't round intervals up and waste budget.
        self._credit = 1.0
        self._t_last: float | None = None

    def rate_hz(self, budget: int, counts: bool, remaining_s: float, robots: int) -> float:
        if not counts:
            return self.max_rate_hz
        spendable = max(0, budget - self.reserve)
        if spendable == 0:
            return 0.0
        rate = self.safety * spendable / max(remaining_s, 1.0) / max(robots, 1)
        return min(self.max_rate_hz, rate)

    def allow(self, t: float, budget: int, counts: bool, remaining_s: float, robots: int) -> bool:
        rate = self.rate_hz(budget, counts, remaining_s, robots)
        if self._t_last is not None:
            self._credit = min(1.0, self._credit + rate * max(0.0, t - self._t_last))
        self._t_last = t
        if rate <= 0.0 or self._credit < 1.0:
            return False
        self._credit -= 1.0
        return True


def remaining_game_seconds(
    secs_remaining: int, first_half: bool, half_duration_s: float = 600.0, ready_overhead: float = 1.15
) -> float:
    """Counted time left in regular time: this half (+ the next if in the first half).

    Ready/Set phases also count but stop the clock only at kick-offs, so `ready_overhead` adds a margin.
    Extra time adds budget (+6,000) when it starts, so it is not included here.
    """
    t = max(0, secs_remaining) + (half_duration_s if first_half else 0.0)
    return t * ready_overhead
