"""Team communication coordinator (LOC-21). Standard library only; transport-agnostic.

One `TeamComm` per robot:
- `outgoing(...)`: asks the BudgetPacer whether this robot may send now. If so, returns the packed TeamMessage
  bytes for the caller to broadcast (TeamLink.send, or a simulated network).
- `incoming(t, payload)`: decodes a received payload. Drops our own echo, other teams' formats and malformed
  data. Stores the latest message per teammate.
- `teammates(t)`: fresh teammates only (received within `max_age`).
- `team_ball(t)`: the most trustworthy shared ball (lowest uncertainty after aging), for when our own camera
  doesn't see it.

The GameController's `messageBudget` (in every control message) is the authority on how many messages the
whole team has left; the pacer only spreads it (team_msg.BudgetPacer). Counters here are for logging and
tests.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from td_game.team_msg import BudgetPacer, TeamMessage, pack, unpack


@dataclass(frozen=True)
class TeammateInfo:
    msg: TeamMessage
    t_rx: float


@dataclass(frozen=True)
class SharedBall:
    xy: tuple[float, float]
    sigma: float  # m, grown with age
    age: float  # s, at the query time
    source: int  # player number


class TeamComm:
    def __init__(
        self,
        player_number: int,
        pacer: BudgetPacer | None = None,
        max_age: float = 3.0,
        ball_speed_sigma: float = 1.0,
    ):
        self.player_number = player_number
        self.pacer = pacer or BudgetPacer()
        self.max_age = max_age
        self.ball_speed_sigma = ball_speed_sigma  # m/s: how fast an old ball sighting loses value
        self._mates: dict[int, TeammateInfo] = {}
        self.sent = 0
        self.received = 0
        self.rejected = 0

    def outgoing(
        self, t: float, msg: TeamMessage, budget: int, counts: bool, remaining_s: float, robots: int
    ) -> bytes | None:
        """Packed message if the pacer allows sending now, else None."""
        if msg.player_number != self.player_number:
            raise ValueError("message player number must be ours")
        if not self.pacer.allow(t, budget, counts, remaining_s, robots):
            return None
        self.sent += 1
        return pack(msg)

    def incoming(self, t: float, payload: bytes) -> TeamMessage | None:
        """Decode and store a teammate's message.

        None for our own echo or anything that isn't ours to read.
        """
        try:
            msg = unpack(payload)
        except ValueError:
            self.rejected += 1
            return None
        if msg.player_number == self.player_number:
            return None  # our own broadcast coming back
        self.received += 1
        self._mates[msg.player_number] = TeammateInfo(msg, t)
        return msg

    def teammates(self, t: float) -> dict[int, TeammateInfo]:
        return {p: i for p, i in self._mates.items() if t - i.t_rx <= self.max_age}

    def team_ball(self, t: float) -> SharedBall | None:
        """Best fresh ball sighting among teammates, by its uncertainty after aging.

        Fallen robots and robots unsure of their side are skipped. A mirrored teammate would report the
        mirrored ball.
        """
        best: SharedBall | None = None
        for p, info in self.teammates(t).items():
            m = info.msg
            if m.ball_xy is None or m.fallen or m.side_confidence < 0.6:
                continue
            age = m.ball_age + (t - info.t_rx)
            sigma = math.hypot(m.ball_sigma, m.pose_sigma, self.ball_speed_sigma * age)
            if best is None or sigma < best.sigma:
                best = SharedBall(m.ball_xy, sigma, age, p)
        return best
