import socket
import time

import pytest
from td_game.team_comm import TeamComm
from td_game.team_msg import BudgetPacer, TeamMessage, pack, remaining_game_seconds
from td_game.udp import TeamLink


def msg(player, ball=(1.0, 2.0), **kw):
    base = dict(pose=(0.0, 0.0, 0.0), pose_sigma=0.1, side_confidence=0.98, ball_xy=ball, ball_sigma=0.2)
    base.update(kw)
    return TeamMessage(player, **base)


def test_outgoing_is_paced_and_checks_player():
    c = TeamComm(2)
    assert c.outgoing(0.0, msg(2), 12000, True, 1000, 3) is not None
    assert c.outgoing(0.1, msg(2), 12000, True, 1000, 3) is None  # 2 Hz cap
    assert c.sent == 1
    with pytest.raises(ValueError):
        c.outgoing(1.0, msg(3), 12000, True, 1000, 3)


def test_incoming_filters_and_stores():
    c = TeamComm(2)
    assert c.incoming(0.0, pack(msg(2))) is None  # our own echo
    assert c.incoming(0.0, b"other team's format") is None and c.rejected == 1
    assert c.incoming(0.0, pack(msg(3))).player_number == 3
    c.incoming(1.0, pack(msg(1, is_goalkeeper=True)))
    assert set(c.teammates(2.0)) == {1, 3}
    assert set(c.teammates(3.5)) == {1}  # player 3 is older than max_age (3 s)


def test_team_ball_prefers_fresh_precise_and_trustworthy():
    c = TeamComm(2)
    c.incoming(0.0, pack(msg(1, ball=(1.0, 1.0), ball_sigma=0.1)))
    c.incoming(1.9, pack(msg(3, ball=(1.5, 1.0), ball_sigma=0.3)))
    c.incoming(2.0, pack(msg(4, ball=(9.0, 9.0), ball_sigma=0.05, fallen=True)))  # fallen: skipped
    c.incoming(2.0, pack(msg(5, ball=(8.0, 8.0), ball_sigma=0.05, side_confidence=0.4)))  # unsure: skipped
    b = c.team_ball(2.0)
    assert b.source == 3  # player 1's sighting is 2 s old, which outweighs its better sigma
    assert b.age == pytest.approx(0.1)
    assert TeamComm(2).team_ball(0.0) is None


def test_five_robot_game_stays_under_budget():
    """20 counted minutes, 5 robots each pacing from the shared budget the GameController reports."""
    comms = [TeamComm(p) for p in range(1, 6)]
    budget, dt, t = 12000, 0.05, 0.0
    for half in (True, False):
        for k in range(int(600 / dt)):
            rem = remaining_game_seconds(600 - k * dt, half)
            for c in comms:
                if c.outgoing(t, msg(c.player_number), budget, True, rem, 5) is not None:
                    budget -= 1
            t += dt
    assert budget >= 200  # reserve never touched
    assert sum(c.sent for c in comms) > 0.6 * 12000  # and most of it is used


def test_custom_pacer_is_used():
    c = TeamComm(2, pacer=BudgetPacer(max_rate_hz=10.0))
    assert c.outgoing(0.0, msg(2), 12000, True, 1000, 1) is not None
    assert c.outgoing(0.11, msg(2), 12000, True, 1000, 1) is not None


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_two_endpoints_exchange_over_real_sockets():
    """The plan's "two laptops" check on one machine.

    Two TeamLinks over loopback, each with its own TeamComm.
    """
    pa, pb = _free_port(), _free_port()
    a = TeamLink(7, broadcast="127.0.0.1", port=pa, bind_host="127.0.0.1", send_port=pb)
    b = TeamLink(7, broadcast="127.0.0.1", port=pb, bind_host="127.0.0.1", send_port=pa)
    ca, cb = TeamComm(1), TeamComm(2)
    try:
        a.send(ca.outgoing(0.0, msg(1, ball=(3.0, -1.0)), 12000, True, 1000, 2))
        b.send(cb.outgoing(0.0, msg(2, ball=(3.1, -1.1)), 12000, True, 1000, 2))
        end = time.time() + 1.0
        got_a, got_b = [], []
        while time.time() < end and not (got_a and got_b):
            got_a += [cb.incoming(0.1, p) for p, _ in b.poll()]
            got_b += [ca.incoming(0.1, p) for p, _ in a.poll()]
            time.sleep(0.01)
        assert got_a and got_a[0].player_number == 1 and got_a[0].ball_xy == pytest.approx((3.0, -1.0))
        assert got_b and got_b[0].player_number == 2
        assert set(ca.teammates(0.2)) == {2} and set(cb.teammates(0.2)) == {1}
    finally:
        a.close()
        b.close()
