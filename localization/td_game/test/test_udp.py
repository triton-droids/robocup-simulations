"""Loopback tests of the UDP endpoints (real sockets on 127.0.0.1, free ports)."""

import socket
import time

import pytest
from gc_fixtures import US, control
from td_game.gc_data import STATE_READY, StatusData, build_control, parse_status
from td_game.team_msg import TeamMessage, pack, unpack
from td_game.udp import GameControllerLink, TeamLink


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_for(fn, timeout=1.0):
    end = time.time() + timeout
    while time.time() < end:
        out = fn()
        if out:
            return out
        time.sleep(0.01)
    return fn()


def test_control_receive_and_status_reply():
    cport, sport = free_port(), free_port()
    link = GameControllerLink(control_port=cport, status_port=sport, bind_host="127.0.0.1")
    gc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    gc.bind(("127.0.0.1", sport))  # the "GameController" listens for status messages here
    gc.settimeout(1.0)
    try:
        assert not link.send_status(
            StatusData(2, US, False, (0, 0, 0), -1, (0, 0))
        )  # no GameController known yet
        gc.sendto(b"garbage", ("127.0.0.1", cport))
        gc.sendto(build_control(control(5, STATE_READY)), ("127.0.0.1", cport))
        msgs = wait_for(link.poll)
        assert len(msgs) == 1 and msgs[0].state == STATE_READY
        assert link.rejected == 1 and link.gc_address == "127.0.0.1"
        assert link.send_status(StatusData(2, US, True, (1000.0, -500.0, 1.0), 0.4, (300.0, 0.0)))
        data, _ = gc.recvfrom(64)
        s = parse_status(data)
        assert s.player_number == 2 and s.fallen and s.pose_mm_rad[0] == pytest.approx(1000.0)
    finally:
        link.close()
        gc.close()


def test_team_link_loopback():
    port = free_port()
    a = TeamLink(US, broadcast="127.0.0.1", port=port, bind_host="127.0.0.1")
    try:
        a.send(pack(TeamMessage(1, (0.0, 0.0, 0.0), 0.1, 0.9, (1.0, 2.0))))
        got = wait_for(a.poll)
        assert unpack(got[0][0]).ball_xy == pytest.approx((1.0, 2.0))
        with pytest.raises(ValueError):
            a.send(b"x" * 513)
    finally:
        a.close()
