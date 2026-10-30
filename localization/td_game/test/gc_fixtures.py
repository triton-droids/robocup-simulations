"""Helpers to build GameController control messages for tests."""

from __future__ import annotations

from dataclasses import replace

from td_game.gc_data import (
    COMPETITION_LARGE,
    MAX_NUM_PLAYERS,
    PHASE_NORMAL,
    SET_PLAY_NONE,
    STATE_INITIAL,
    ControlData,
    RobotInfo,
    TeamInfo,
)

US, THEM = 7, 13


def team(number: int, budget: int = 12000, goalkeeper: int = 1, penalties: dict | None = None) -> TeamInfo:
    players = [RobotInfo() for _ in range(MAX_NUM_PLAYERS)]
    for p, pen in (penalties or {}).items():
        players[p - 1] = RobotInfo(pen, 30 if pen else 0, 0)
    return TeamInfo(number, 0, 1, goalkeeper, 0, 0, 0, budget, tuple(players))


def control(
    packet: int = 0, state: int = STATE_INITIAL, our_penalties: dict | None = None, **kw
) -> ControlData:
    base = ControlData(
        packet_number=packet,
        players_per_team=3,
        competition_type=COMPETITION_LARGE,
        stopped=False,
        game_phase=PHASE_NORMAL,
        state=state,
        set_play=SET_PLAY_NONE,
        first_half=True,
        kicking_team=US,
        secs_remaining=600,
        secondary_time=0,
        teams=(team(THEM), team(US, penalties=our_penalties)),
    )
    return replace(base, **kw)
