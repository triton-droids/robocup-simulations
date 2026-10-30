import pytest
from gc_fixtures import US, control, team
from td_game.game_state import GameStateTracker
from td_game.gc_data import (
    PENALTY_ILLEGAL_POSITIONING,
    PENALTY_MOTION_IN_SET,
    PENALTY_PICK_UP,
    PENALTY_PUSHING,
    PENALTY_SENT_OFF,
    PENALTY_SUBSTITUTE,
    PHASE_TIMEOUT,
    STATE_FINISHED,
    STATE_INITIAL,
    STATE_PLAYING,
    STATE_READY,
    STATE_SET,
)

ME = 2


def kinds(events):
    return [e.kind for e in events]


@pytest.fixture
def tr():
    return GameStateTracker(US, ME)


def feed(tr, seq):
    """seq: list of (t, ControlData). Returns all events."""
    out = []
    for t, c in seq:
        out += tr.update(t, c)
    return out


def test_first_packet_in_initial(tr):
    ev = tr.update(0.0, control(packet=0))
    assert kinds(ev) == ["gc_back", "state", "initial"]
    assert tr.state_name == "initial" and not tr.penalized and tr.message_budget == 12000


def test_full_half_cycle_and_second_half(tr):
    seq = [
        (0.0, control(0)),
        (1.0, control(1, STATE_READY)),
        (2.0, control(2, STATE_SET)),
        (3.0, control(3, STATE_PLAYING)),
        (4.0, control(4, STATE_FINISHED)),
        (5.0, control(5, STATE_INITIAL, first_half=False)),
    ]
    ev = feed(tr, seq)
    assert [e.detail for e in ev if e.kind == "state"] == [
        "initial",
        "ready",
        "set",
        "playing",
        "finished",
        "initial",
    ]
    assert kinds(ev).count("initial") == 2
    assert "half" in kinds(ev)


def test_duplicates_and_stale_packets_are_ignored(tr):
    tr.update(0.0, control(10))
    assert tr.update(0.5, control(10)) == []  # duplicate
    assert tr.update(0.6, control(9, STATE_READY)) == []  # stale
    assert tr.state_name == "initial"
    assert kinds(tr.update(1.0, control(11, STATE_READY))) == ["state"]


def test_packet_number_wraps(tr):
    tr.update(0.0, control(255))
    assert kinds(tr.update(0.5, control(0, STATE_READY))) == ["state"]


def test_standard_penalty_gives_pause_then_reentry(tr):
    seq = [
        (0.0, control(0, STATE_PLAYING)),
        (1.0, control(1, STATE_PLAYING, our_penalties={ME: PENALTY_PUSHING})),
        (40.0, control(2, STATE_PLAYING)),
    ]
    ev = feed(tr, seq)
    pen = next(e for e in ev if e.kind == "penalized")
    assert pen.detail == "pushing" and not pen.in_place
    assert [e for e in ev if e.kind == "reentry"][0].detail == "pushing"


def test_teammate_penalty_is_not_ours(tr):
    ev = feed(
        tr,
        [
            (0.0, control(0, STATE_PLAYING)),
            (1.0, control(1, STATE_PLAYING, our_penalties={3: PENALTY_PUSHING})),
        ],
    )
    assert "penalized" not in kinds(ev)


def test_motion_in_set_is_in_place(tr):
    seq = [
        (0.0, control(0, STATE_SET)),
        (1.0, control(1, STATE_SET, our_penalties={ME: PENALTY_MOTION_IN_SET})),
        (16.0, control(2, STATE_PLAYING)),
    ]
    ev = feed(tr, seq)
    assert next(e for e in ev if e.kind == "penalized").in_place
    assert "unpenalized_in_place" in kinds(ev) and "reentry" not in kinds(ev)


def test_pickup_during_initial_is_a_replacement(tr):
    seq = [(0.0, control(0)), (1.0, control(1, our_penalties={ME: PENALTY_PICK_UP})), (5.0, control(2))]
    ev = feed(tr, seq)
    assert "replaced" in kinds(ev) and "reentry" not in kinds(ev)


def test_illegal_positioning_in_set_then_reentry(tr):
    seq = [
        (0.0, control(0, STATE_SET)),
        (1.0, control(1, STATE_SET, our_penalties={ME: PENALTY_ILLEGAL_POSITIONING})),
        (46.0, control(2, STATE_PLAYING)),
    ]
    assert "reentry" in kinds(feed(tr, seq))


def test_timeout_phase_counts_as_initial(tr):
    seq = [(0.0, control(0, STATE_PLAYING)), (1.0, control(1, STATE_INITIAL, game_phase=PHASE_TIMEOUT))]
    ev = feed(tr, seq)
    init = next(e for e in ev if e.kind == "initial")
    assert init.detail == "timeout"
    assert not tr.budget_counts


def test_budget_counts_only_in_ready_set_playing(tr):
    for s, counts in [
        (STATE_INITIAL, False),
        (STATE_READY, True),
        (STATE_SET, True),
        (STATE_PLAYING, True),
        (STATE_FINISHED, False),
    ]:
        t = GameStateTracker(US, ME)
        t.update(0.0, control(0, s))
        assert t.budget_counts is counts, s


def test_other_fields_game_is_ignored(tr):
    other = control(0, teams=(team(40), team(41)))
    assert tr.update(0.0, other) == [] and tr.data is None


def test_link_loss_and_return(tr):
    tr.update(0.0, control(0))
    assert tr.check_timeout(2.0) == []
    assert kinds(tr.check_timeout(3.5)) == ["gc_lost"]
    assert tr.check_timeout(4.0) == []  # reported once
    # After a loss even a far-off packet number is accepted (GameController restarted).
    assert "gc_back" in kinds(tr.update(5.0, control(3)))


def test_views(tr):
    tr.update(
        0.0,
        control(
            0, kicking_team=US, teams=(team(13), team(US, goalkeeper=ME, penalties={3: PENALTY_SENT_OFF}))
        ),
    )
    assert tr.is_goalkeeper and tr.our_kick is True
    assert tr.active_teammates == 3  # 19 others unpenalised, capped by playersPerTeam = 3
    t2 = GameStateTracker(US, ME)
    t2.update(0.0, control(0, kicking_team=255))
    assert t2.our_kick is None


def test_substitute_entering_counts_as_reentry(tr):
    """ASSUMPTION (see module doc): a substitute entering is placed like a re-entry."""
    seq = [
        (0.0, control(0, STATE_PLAYING, our_penalties={ME: PENALTY_SUBSTITUTE})),
        (10.0, control(1, STATE_PLAYING)),
    ]
    ev = feed(tr, seq)
    assert "reentry" in kinds(ev)


def test_rejects_monitor_data(tr):
    with pytest.raises(ValueError):
        tr.update(0.0, control(0, true_data=True))


def test_player_number_validation():
    with pytest.raises(ValueError):
        GameStateTracker(US, 0)
