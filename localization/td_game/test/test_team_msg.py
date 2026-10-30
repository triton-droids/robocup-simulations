import math

import pytest
from td_game.team_msg import MAX_PAYLOAD, BudgetPacer, TeamMessage, pack, remaining_game_seconds, unpack


def msg(**kw):
    base = dict(player_number=3, pose=(-2.5, 1.25, 2.0), pose_sigma=0.15, side_confidence=0.97,
                ball_xy=(1.0, -3.2), ball_sigma=0.25, ball_age=0.3, is_goalkeeper=False)  # fmt: skip
    base.update(kw)
    return TeamMessage(**base)


def test_pack_size_and_round_trip():
    data = pack(msg())
    assert len(data) == 22 <= MAX_PAYLOAD
    back = unpack(data)
    assert back.player_number == 3
    assert back.pose == pytest.approx((-2.5, 1.25, 2.0), abs=1e-3)
    assert back.ball_xy == pytest.approx((1.0, -3.2), abs=0.01)
    assert back.side_confidence == pytest.approx(0.97, abs=1 / 255)
    assert back.ball_age == pytest.approx(0.3) and back.localized and not back.fallen


def test_no_ball_and_flags():
    back = unpack(pack(msg(ball_xy=None, is_goalkeeper=True, fallen=True, localized=False)))
    assert back.ball_xy is None and back.is_goalkeeper and back.fallen and not back.localized


def test_angles_wrap_and_values_clip():
    back = unpack(pack(msg(pose=(500.0, -500.0, 3 * math.pi), pose_sigma=10.0, side_confidence=1.7)))
    assert back.pose[0] == pytest.approx(327.67) and back.pose[1] == pytest.approx(-327.67)
    assert abs(back.pose[2]) == pytest.approx(math.pi, abs=1e-3)
    assert back.pose_sigma == pytest.approx(2.55) and back.side_confidence == pytest.approx(1.0)


def test_foreign_messages_rejected():
    with pytest.raises(ValueError):
        unpack(b"x" * 22)
    with pytest.raises(ValueError):
        unpack(pack(msg())[:-1])


def test_remaining_game_seconds():
    assert remaining_game_seconds(300, first_half=True, ready_overhead=1.0) == 900
    assert remaining_game_seconds(-10, first_half=False, ready_overhead=1.0) == 0


def _simulate_game(robots: int, budget: int = 12000, dt: float = 0.05) -> tuple[int, int]:
    """Each robot paces from the shared budget the GameController reports, over 2 × 600 s of counted time."""
    pacers = [BudgetPacer() for _ in range(robots)]
    sent, t = 0, 0.0
    for half in (True, False):
        for k in range(int(600 / dt)):
            rem = remaining_game_seconds(600 - k * dt, half)
            for p in pacers:
                if p.allow(t, budget, True, rem, robots):
                    budget -= 1
                    sent += 1
            t += dt
    return sent, budget


@pytest.mark.parametrize("robots", [3, 5])
def test_pacer_never_exceeds_budget_over_a_simulated_game(robots):
    sent, left = _simulate_game(robots)
    assert left >= 200  # never overspends, reserve kept
    # Whichever limit binds: the per-robot 2 Hz cap (3 robots: 7,200 max) or the deliberately conservative
    # budget share (safety 0.9 and a 15% Ready/Set allowance ≈ 78% of 11,800 spendable).
    expected = min(robots * 2.0 * 1200, 0.9 / 1.15 * (12000 - 200))
    assert sent > 0.9 * expected


def test_pacer_free_outside_counted_states_and_stops_at_reserve():
    p = BudgetPacer(max_rate_hz=2.0, reserve=200)
    assert p.rate_hz(budget=0, counts=False, remaining_s=100, robots=3) == 2.0
    assert p.rate_hz(budget=200, counts=True, remaining_s=100, robots=3) == 0.0
    assert not BudgetPacer().allow(0.0, 150, True, 100, 3)


def test_pacer_rate_limit():
    p = BudgetPacer(max_rate_hz=2.0)
    assert p.allow(0.0, 12000, True, 10, 1)
    assert not p.allow(0.2, 12000, True, 10, 1)
    assert p.allow(0.51, 12000, True, 10, 1)
