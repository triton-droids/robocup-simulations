import math
import struct

import pytest
from gc_fixtures import THEM, US, control, team
from td_game.gc_data import (
    CONTROL_SIZE,
    PENALTY_PUSHING,
    STATE_PLAYING,
    STATUS_SIZE,
    GCFormatError,
    StatusData,
    build_control,
    build_status,
    parse_control,
    parse_status,
    status_from_localization,
)


def test_sizes_match_the_official_struct():
    # sizeof(RoboCupGameControlData) = 4 + 10 + 2·2 + 2·(6 + 2·2 + 20·3) = 158, sizeof(ReturnData) = 32
    assert CONTROL_SIZE == 158
    assert STATUS_SIZE == 32


def test_byte_offsets_match_the_wireshark_dissector():
    """Offsets from game_controller_msgs/dissectors/robocup_gc.lua.

    Checked on raw bytes, independent of parse_control.
    """
    c = control(packet=42, state=STATE_PLAYING, secs_remaining=-5, secondary_time=300, kicking_team=US,
                our_penalties={3: PENALTY_PUSHING}, first_half=False)  # fmt: skip
    b = build_control(c)
    assert b[0:4] == b"RGme" and b[4] == 20
    assert b[5] == 42  # packetNumber
    assert b[10] == STATE_PLAYING and b[12] == 0 and b[13] == US  # state, firstHalf, kickingTeam
    assert struct.unpack_from("<h", b, 14)[0] == -5 and struct.unpack_from("<h", b, 16)[0] == 300
    team2 = 18 + 70  # teams[1] is US in the fixture
    assert b[team2] == US
    assert struct.unpack_from("<H", b, team2 + 8)[0] == 12000  # messageBudget
    assert b[team2 + 10 + 3 * (3 - 1)] == PENALTY_PUSHING  # player 3 penalty


def test_control_round_trip():
    c = control(packet=200, our_penalties={2: PENALTY_PUSHING})
    assert parse_control(build_control(c)) == c


def test_control_validation():
    good = build_control(control())
    with pytest.raises(GCFormatError, match="length"):
        parse_control(good[:-1])
    with pytest.raises(GCFormatError, match="header"):
        parse_control(b"XXXX" + good[4:])
    with pytest.raises(GCFormatError, match="version"):
        parse_control(good[:4] + bytes([19]) + good[5:])
    monitor = b"RGTD" + good[4:]
    with pytest.raises(GCFormatError, match="monitor"):
        parse_control(monitor)
    assert parse_control(monitor, accept_monitor=True).true_data


def test_team_lookup_by_number_not_position():
    c = control()
    assert c.team(US).team_number == US and c.opponent(US).team_number == THEM
    assert c.team(99) is None
    swapped = control(teams=(team(US), team(THEM)))
    assert swapped.team(US).team_number == US


def test_status_round_trip_and_units():
    s = status_from_localization(2, US, False, (1.5, -0.25, 3.5), ball_rel_m=(0.8, 0.1), ball_age_s=0.2)
    assert s.pose_mm_rad[0] == pytest.approx(1500) and s.pose_mm_rad[2] == pytest.approx(3.5 - 2 * math.pi)
    back = parse_status(build_status(s))
    assert back.player_number == 2 and back.team_number == US and not back.fallen
    assert back.ball_mm == pytest.approx((800, 100), abs=1e-3)
    unseen = status_from_localization(2, US, True, (0, 0, 0))
    assert parse_status(build_status(unseen)).ball_age_s == -1.0


def test_status_validation_matches_gamecontroller_checks():
    with pytest.raises(GCFormatError):
        build_status(StatusData(0, US, False, (0, 0, 0), -1, (0, 0)))  # player numbers start at 1
    with pytest.raises(GCFormatError):
        build_status(StatusData(1, US, False, (math.nan, 0, 0), -1, (0, 0)))
    raw = bytearray(build_status(StatusData(1, US, False, (0, 0, 0), -1, (0, 0))))
    raw[7] = 2  # fallen must be 0/1
    with pytest.raises(GCFormatError, match="fallen"):
        parse_status(bytes(raw))
