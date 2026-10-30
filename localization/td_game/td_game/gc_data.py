"""GameController wire formats (LOC-08). Standard library only.

Mirrors the official HSL GameController v7 header `game_controller_msgs/headers/RoboCupGameControlData.h`
(https://github.com/RoboCup-HumanoidSoccerLeague/GameController, struct version 20, return version 4).
Byte layout taken from `game_controller_msgs/src/control_message.rs` and cross-checked with the Wireshark
dissector `dissectors/robocup_gc.lua`: packed, little-endian, no padding.

  Control message (GameController → robots, UDP broadcast, port 3838, 2 Hz; 5 Hz while stopped): 158 bytes.
    0  header "RGme" ("RGTD" = true data for monitors; robots must ignore it)
    4  version (20), packetNumber, playersPerTeam, competitionType, stopped, gamePhase, state, setPlay,
       firstHalf, kickingTeam (u8 each, offsets 4–13)
    14 secsRemaining (i16), 16 secondaryTime (i16)
    18 teams[2], 70 bytes each: teamNumber, fieldPlayerColour, goalkeeperColour, goalkeeper, score,
       penaltyShot (u8), singleShots (u16), messageBudget (u16), players[20] × (penalty, secsTillUnpenalised,
       cautions)
  Status message (robot → GameController, UDP unicast to the sender, port 3939, 0.5–2 Hz): 32 bytes.
    "RGrt", version (4), playerNum (1-based), teamNum, fallen (0/1), pose x, y [mm], θ [rad] (field frame with
    +x toward the goal WE attack: the same as td_localization's map frame), ballAge [s] (-1 = never seen),
    ball x, y [mm] relative to the robot (f32 each).

Note from the GameController README: after a transition to Playing, robots receive the previous state for
up to 10 s (the whistle must be detected on the robot). Team order in `teams` follows the GameController's
sides, not home/away, so always look a team up by number.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

CONTROL_PORT = 3838
STATUS_PORT = 3939
TEAM_PORT_BASE = 10000  # team messages: 10000 + team number

CONTROL_HEADER = b"RGme"
MONITOR_HEADER = b"RGTD"
CONTROL_VERSION = 20
STATUS_HEADER = b"RGrt"
STATUS_VERSION = 4
MAX_NUM_PLAYERS = 20

COMPETITION_SMALL, COMPETITION_MIDDLE, COMPETITION_LARGE = 0, 1, 2
PHASE_NORMAL, PHASE_PENALTY_SHOOT_OUT, PHASE_EXTRA_TIME, PHASE_TIMEOUT = 0, 1, 2, 3
STATE_INITIAL, STATE_READY, STATE_SET, STATE_PLAYING, STATE_FINISHED = 0, 1, 2, 3, 4
STATE_NAMES = {0: "initial", 1: "ready", 2: "set", 3: "playing", 4: "finished"}
SET_PLAY_NONE, SET_PLAY_DIRECT_FREE_KICK, SET_PLAY_INDIRECT_FREE_KICK = 0, 1, 2
SET_PLAY_PENALTY_KICK, SET_PLAY_THROW_IN, SET_PLAY_GOAL_KICK, SET_PLAY_CORNER_KICK = 3, 4, 5, 6
KICKING_TEAM_NONE = 255

PENALTY_NONE = 0
PENALTY_ILLEGAL_POSITIONING = 1
PENALTY_MOTION_IN_SET = 2
PENALTY_MOTION_IN_STOP = 3
PENALTY_LOCAL_GAME_STUCK = 4
PENALTY_INCAPABLE_ROBOT = 5
PENALTY_PICK_UP = 6
PENALTY_BALL_HOLDING = 7
PENALTY_LEAVING_THE_FIELD = 8
PENALTY_PLAYING_WITH_ARMS_HANDS = 9
PENALTY_PUSHING = 10
PENALTY_CAUTIONED = 11
PENALTY_SENT_OFF = 12
PENALTY_SUBSTITUTE = 13
PENALTY_NAMES = {
    0: "none",
    1: "illegal_positioning",
    2: "motion_in_set",
    3: "motion_in_stop",
    4: "local_game_stuck",
    5: "incapable_robot",
    6: "pick_up",
    7: "ball_holding",
    8: "leaving_the_field",
    9: "playing_with_arms_hands",
    10: "pushing",
    11: "cautioned",
    12: "sent_off",
    13: "substitute",
}

_HEAD = struct.Struct("<4s10Bhh")  # 18 bytes
_TEAM = struct.Struct("<6BHH")  # 10 bytes, then 20 × 3 player bytes
_PLAYER = struct.Struct("<3B")
TEAM_SIZE = _TEAM.size + MAX_NUM_PLAYERS * _PLAYER.size  # 70
CONTROL_SIZE = _HEAD.size + 2 * TEAM_SIZE  # 158
_STATUS = struct.Struct("<4s4B3ff2f")
STATUS_SIZE = _STATUS.size  # 32


class GCFormatError(ValueError):
    """A packet that is not a valid GameController message for this protocol version."""


@dataclass(frozen=True)
class RobotInfo:
    penalty: int = PENALTY_NONE
    secs_till_unpenalised: int = 0
    cautions: int = 0


@dataclass(frozen=True)
class TeamInfo:
    team_number: int
    field_player_colour: int = 0
    goalkeeper_colour: int = 0
    goalkeeper: int = 0  # player number, 0 = none
    score: int = 0
    penalty_shot: int = 0
    single_shots: int = 0
    message_budget: int = 0
    players: tuple[RobotInfo, ...] = field(default_factory=lambda: (RobotInfo(),) * MAX_NUM_PLAYERS)


@dataclass(frozen=True)
class ControlData:
    packet_number: int
    players_per_team: int
    competition_type: int
    stopped: bool
    game_phase: int
    state: int
    set_play: int
    first_half: bool
    kicking_team: int
    secs_remaining: int
    secondary_time: int
    teams: tuple[TeamInfo, TeamInfo]
    true_data: bool = False  # "RGTD" monitor message

    def team(self, team_number: int) -> TeamInfo | None:
        return next((t for t in self.teams if t.team_number == team_number), None)

    def opponent(self, team_number: int) -> TeamInfo | None:
        return next((t for t in self.teams if t.team_number != team_number), None)


def parse_control(data: bytes, accept_monitor: bool = False) -> ControlData:
    """Decode a control message. Raises GCFormatError for wrong size, header or version."""
    if len(data) != CONTROL_SIZE:
        raise GCFormatError(f"wrong length {len(data)} (expected {CONTROL_SIZE})")
    head = _HEAD.unpack_from(data, 0)
    magic = head[0]
    if magic == MONITOR_HEADER and not accept_monitor:
        raise GCFormatError("monitor (true data) message; robots must not use it")
    if magic not in (CONTROL_HEADER, MONITOR_HEADER):
        raise GCFormatError(f"wrong header {magic!r}")
    version = head[1]
    if version != CONTROL_VERSION:
        raise GCFormatError(f"wrong version {version} (expected {CONTROL_VERSION})")
    teams = []
    for k in range(2):
        off = _HEAD.size + k * TEAM_SIZE
        t = _TEAM.unpack_from(data, off)
        players = tuple(
            RobotInfo(*_PLAYER.unpack_from(data, off + _TEAM.size + p * _PLAYER.size))
            for p in range(MAX_NUM_PLAYERS)
        )
        teams.append(TeamInfo(*t, players=players))
    return ControlData(
        packet_number=head[2],
        players_per_team=head[3],
        competition_type=head[4],
        stopped=bool(head[5]),
        game_phase=head[6],
        state=head[7],
        set_play=head[8],
        first_half=bool(head[9]),
        kicking_team=head[10],
        secs_remaining=head[11],
        secondary_time=head[12],
        teams=(teams[0], teams[1]),
        true_data=magic == MONITOR_HEADER,
    )


def build_control(c: ControlData) -> bytes:
    """Encode a control message (for tests and simulation; robots never send these)."""
    out = bytearray(
        _HEAD.pack(
            MONITOR_HEADER if c.true_data else CONTROL_HEADER,
            CONTROL_VERSION,
            c.packet_number & 0xFF,
            c.players_per_team,
            c.competition_type,
            int(c.stopped),
            c.game_phase,
            c.state,
            c.set_play,
            int(c.first_half),
            c.kicking_team,
            c.secs_remaining,
            c.secondary_time,
        )
    )
    for t in c.teams:
        out += _TEAM.pack(
            t.team_number,
            t.field_player_colour,
            t.goalkeeper_colour,
            t.goalkeeper,
            t.score,
            t.penalty_shot,
            t.single_shots,
            t.message_budget,
        )
        players = list(t.players) + [RobotInfo()] * (MAX_NUM_PLAYERS - len(t.players))
        for p in players[:MAX_NUM_PLAYERS]:
            out += _PLAYER.pack(p.penalty, p.secs_till_unpenalised, p.cautions)
    assert len(out) == CONTROL_SIZE
    return bytes(out)


@dataclass(frozen=True)
class StatusData:
    """What a robot reports to the GameController (RoboCupGameControlReturnData)."""

    player_number: int  # 1-based
    team_number: int
    fallen: bool
    pose_mm_rad: tuple[float, float, float]  # field frame, +x toward the goal we attack
    ball_age_s: float  # -1 if never seen
    ball_mm: tuple[float, float]  # relative to the robot


def build_status(s: StatusData) -> bytes:
    if not 1 <= s.player_number <= MAX_NUM_PLAYERS:
        raise GCFormatError("player number must be 1..20")
    values = (*s.pose_mm_rad, s.ball_age_s, *s.ball_mm)
    if any(math.isnan(v) for v in values):
        raise GCFormatError("NaN in status message (the GameController rejects it)")
    return _STATUS.pack(STATUS_HEADER, STATUS_VERSION, s.player_number, s.team_number, int(s.fallen), *values)


def parse_status(data: bytes) -> StatusData:
    """Decode a status message with the same checks as the GameController (status_message.rs)."""
    if len(data) != STATUS_SIZE:
        raise GCFormatError("wrong length")
    magic, version, player, team, fallen, x, y, th, age, bx, by = _STATUS.unpack(data)
    if magic != STATUS_HEADER:
        raise GCFormatError("wrong header")
    if version != STATUS_VERSION:
        raise GCFormatError("wrong version")
    if not 1 <= player <= MAX_NUM_PLAYERS:
        raise GCFormatError("invalid player number")
    if fallen > 1:
        raise GCFormatError("invalid fallen")
    if any(math.isnan(v) for v in (x, y, th, age, bx, by)):
        raise GCFormatError("NaN field")
    return StatusData(player, team, fallen == 1, (x, y, th), age, (bx, by))


def status_from_localization(
    player_number: int,
    team_number: int,
    fallen: bool,
    pose_m_rad,
    ball_rel_m=None,
    ball_age_s: float | None = None,
) -> StatusData:
    """Convenience: build the status message from td_localization outputs (meters → millimeters).

    Our map frame already has +x toward the goal we attack, which is what the GameController expects.
    """
    x, y, th = pose_m_rad
    th = math.atan2(math.sin(th), math.cos(th))
    if ball_rel_m is None or ball_age_s is None:
        return StatusData(player_number, team_number, fallen, (1000 * x, 1000 * y, th), -1.0, (0.0, 0.0))
    return StatusData(
        player_number,
        team_number,
        fallen,
        (1000 * x, 1000 * y, th),
        float(ball_age_s),
        (1000 * ball_rel_m[0], 1000 * ball_rel_m[1]),
    )
