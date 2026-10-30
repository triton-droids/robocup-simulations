"""End-to-end bench runs for different hardware mixes, record/replay and the CLI."""

import math
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest
from bench_report import show
from td_bench.__main__ import main
from td_bench.record import Recorder, Recording, ReplayCamera, ReplayGame, ReplayHead, ReplayOdometry
from td_bench.sources import (
    FixedHead,
    ImuHead,
    LiveOdometry,
    ScriptedGame,
    SimBall,
    SimCamera,
    SimHead,
    SimOdometry,
    SimTeam,
    SimVisualOdometry,
    StationaryOdometry,
    UdpGame,
    UdpTeam,
    default_world,
)
from td_bench.vision import D435_HALF, ImageCamera, SyntheticFrames
from td_field_model.sim import NoiseModel, Simulator
from td_game.gc_data import STATE_INITIAL, STATE_PLAYING, STATE_READY, build_control
from td_game.team_msg import pack
from td_game.udp import GameControllerLink, TeamLink
from td_localization.suite import BallTrack

from td_bench import Bench
from td_field_model import FieldModel
from td_game import TeamComm, TeamMessage

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "td_game" / "test"))
from gc_fixtures import US, control  # noqa: E402

START = (-3.0, 1.0, 0.3)


@pytest.fixture(scope="module")
def fm():
    return FieldModel.builtin("hsl_m_2026")


def tail_error(result, secs: float = 3.0) -> float:
    """Median position error over the last `secs` of the run."""
    t = result.est.t
    sel = t >= t[-1] - secs
    return float(np.median(np.hypot(*(result.est.pose[sel, :2] - result.gt.pose[sel, :2]).T)))


def sim_camera(fm, world, seed=1):
    return SimCamera(world, Simulator(fm, noise=NoiseModel(false_positives=2.0), seed=seed))


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


# ---------------------------------------------------------------- hardware mixes


def test_all_simulated_search_converges(fm, record_property):
    w = default_world(fm, "walk", seed=1, duration=20.0, start=START)
    r = Bench(fm, sim_camera(fm, w), SimOdometry(w), SimHead(w), truth=w, seed=1).run(20.0)
    show(record_property, "Ticks run", str(r.ticks), "601")
    show(record_property, "Position error, median of last 3 s", f"{tail_error(r):.3f} m", "< 0.3 m")
    show(record_property, "Side confidence at end", f"{r.side.confidence:.2f}", "> 0.7")
    show(record_property, "Final x (own half is x < 0)", f"{r.est.pose[-1, 0]:.2f} m", "< 0")
    show(record_property, "Game events", ", ".join(e.kind for e in r.events), "initial")
    assert r.ticks == 601 and r.gt is not None
    assert tail_error(r) < 0.3
    assert r.side.confidence > 0.7 and r.est.pose[-1, 0] < 0  # own half, our side
    assert [e.kind for e in r.events] == ["initial"]  # automatic start without a game source


def test_camera_only_tripod(fm, record_property):
    """Only a camera: synthetic images (stand-in for a D435 on a tripod), no motion, no head encoders."""
    w = default_world(fm, "still", seed=2, duration=8.0, start=START, head_pitch=0.35)
    cam = ImageCamera(SyntheticFrames(fm, D435_HALF, w, seed=2), D435_HALF, FixedHead(0.0, 0.35))
    rough = (START[0] + 0.3, START[1] - 0.25, START[2] + 0.1)  # tape-measure start, 40 cm / 6° off
    r = Bench(
        fm, cam, StationaryOdometry(), FixedHead(0.0, 0.35), truth=START, start_poses=[rough], seed=2
    ).run(8.0)
    head_err = abs(math.remainder(r.est.pose[-1, 2] - START[2], 2 * math.pi))
    show(
        record_property,
        "Ticks with camera observations",
        f"{r.ticks - r.frames_without_obs} of {r.ticks}",
        "> 0",
    )
    show(record_property, "Start-pose error given to the filter", "0.39 m, 5.7°", "(input)")
    show(record_property, "Position error, median of last 3 s", f"{tail_error(r):.3f} m", "< 0.2 m")
    show(record_property, "Heading error at end", f"{math.degrees(head_err):.2f}°", "< 5°")
    assert r.frames_without_obs < r.ticks  # images arrived
    assert tail_error(r) < 0.2
    assert abs(math.remainder(r.est.pose[-1, 2] - START[2], 2 * math.pi)) < math.radians(5)


def test_everything_but_camera_live_odometry_and_udp(fm, record_property):
    """Real-time run: odometry pushed by a producer thread (like an encoder callback), a GameController and a
    teammate over real UDP sockets on loopback; only the camera is simulated."""
    dur = 4.0
    w = default_world(fm, "walk", seed=4, duration=dur + 1, start=START)
    odo = LiveOdometry()
    cport, sport, pa, pb = free_port(), free_port(), free_port(), free_port()
    gc_link = GameControllerLink(control_port=cport, status_port=sport, bind_host="127.0.0.1")
    us = TeamLink(US, broadcast="127.0.0.1", port=pa, bind_host="127.0.0.1", send_port=pb)
    mate = TeamLink(US, broadcast="127.0.0.1", port=pb, bind_host="127.0.0.1", send_port=pa)
    stop = threading.Event()

    def odometry_producer(t0):
        for s in w.steps[1:]:
            lag = s.t - (time.perf_counter() - t0)
            if lag > 0:
                time.sleep(lag)
            if stop.is_set():
                return
            odo.push(*s.odom, s.odom_cov)

    def game_controller():
        gc = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        k = 0
        while not stop.is_set():
            state = STATE_INITIAL if k < 5 else STATE_READY if k < 10 else STATE_PLAYING
            gc.sendto(build_control(control(k, state)), ("127.0.0.1", cport))
            mate.send(pack(TeamMessage(3, (-1.0, -1.0, 0.0), 0.1, 0.95, (0.5, 0.5), 0.2)))
            k += 1
            time.sleep(0.1)
        gc.close()

    game = UdpGame(gc_link, US, 2)
    bench = Bench(
        fm, sim_camera(fm, w, seed=4), odo, SimHead(w), game=game, truth=w, start_poses=[START], seed=4
    )
    bench.team = UdpTeam(us, TeamComm(2), bench.bridge, tracker=game.tracker)
    t0 = time.perf_counter()
    threads = [
        threading.Thread(target=odometry_producer, args=(t0,)),
        threading.Thread(target=game_controller),
    ]
    for th in threads:
        th.start()
    try:
        r = bench.run(dur, realtime=True)
        wall = time.perf_counter() - t0
        got = []
        for _ in range(20):
            got += mate.poll()
            if got:
                break
            time.sleep(0.05)
    finally:
        stop.set()
        for th in threads:
            th.join()
        for link in (gc_link, us, mate):
            link.close()
    kinds = [e.kind for e in r.events]
    show(record_property, "Wall-clock time for a 4 s run", f"{wall:.2f} s", ">= 3.8 s")
    show(record_property, "GameController events", ", ".join(dict.fromkeys(kinds)), "includes initial, state")
    show(record_property, "GameController state at end", game.tracker.state_name, "PLAYING")
    show(
        record_property, "Teammates heard over UDP", str(sorted(bench.team.comm.teammates(dur))), "includes 3"
    )
    show(
        record_property,
        "Our messages sent / received by teammate",
        f"{bench.team.comm.sent} / {len(got)}",
        "> 0 / > 0",
    )
    show(record_property, "Position error, median of last 1 s", f"{tail_error(r, 1.0):.3f} m", "< 0.3 m")
    assert wall >= dur * 0.95  # paced to wall time
    assert "initial" in kinds and "state" in kinds
    assert game.tracker.state == STATE_PLAYING
    assert 3 in bench.team.comm.teammates(dur)  # teammate heard
    assert got and bench.team.comm.sent > 0  # and we were heard
    assert tail_error(r, 1.0) < 0.3


def test_scripted_penalty_pauses_and_reseeds(fm, record_property):
    w = default_world(fm, "walk", seed=5, duration=6.0, start=START)
    game = ScriptedGame([(0.0, "initial"), (2.0, "penalized", "pushing"), (4.0, "reentry", "pushing")])
    r = Bench(fm, sim_camera(fm, w), SimOdometry(w), SimHead(w), game=game, truth=w, start_poses=[START]).run(
        6.0
    )
    lost = r.est.status == "lost"
    show(record_property, "Status at 1.5 s (before penalty)", r.est.status[45], "not lost")
    show(record_property, "Status at 2.5 s (penalized)", r.est.status[75], "lost (paused)")
    show(record_property, "Status at 6 s (after re-entry)", r.est.status[-1], "(info)")
    show(record_property, "Scored events", ", ".join(k for _, k in r.gt.events), "initial, penalty_reentry")
    assert lost[int(2.5 * 30)] and not lost[int(1.5 * 30)]  # paused while penalized
    assert [k for _, k in r.gt.events] == ["initial", "penalty_reentry"]


def test_ball_and_simulated_team(fm, record_property):
    w = default_world(fm, "walk", seed=6, duration=8.0, start=START)
    ball = SimBall(w, BallTrack(fm, seed=6, start=(-1.5, 0.5)), seed=6)
    r = Bench(
        fm,
        sim_camera(fm, w),
        SimOdometry(w),
        SimHead(w),
        team=SimTeam(ball, seed=6),
        ball=ball,
        truth=w,
        start_poses=[START],
    ).run(8.0)
    show(record_property, "Side confidence at end", f"{r.side.confidence:.3f}", "> 0.9")
    assert r.side.confidence > 0.9


def test_visual_only_cuvslam_camera_imu_head(fm, record_property):
    """Visual only: camera images, visual odometry (cuVSLAM model) as the only motion input, head angles from
    encoders + head IMU (bias + noise). No legs, GameController, teammates or ball; start pose unknown."""
    w = default_world(fm, "walk", seed=1, duration=25.0, start=START)
    head = ImuHead(w, seed=1)
    cam = ImageCamera(SyntheticFrames(fm, D435_HALF, w, seed=1), D435_HALF, head)
    vo = SimVisualOdometry(w, head, seed=1)
    r = Bench(fm, cam, vo, head, truth=w, seed=1).run(25.0)
    show(record_property, "VO tracking lost (fast pans + random)", f"{100 * vo.lost_fraction:.0f}%", "(info)")
    show(record_property, "Head pitch bias", f"{math.degrees(head.pitch_bias):+.2f}°", "(info)")
    show(record_property, "Position error, median of last 5 s", f"{tail_error(r, 5.0):.3f} m", "< 0.3 m")
    show(record_property, "Side confidence at end", f"{r.side.confidence:.2f}", "> 0.7")
    assert vo.lost_fraction > 0  # the scanning head does drop tracking
    assert tail_error(r, 5.0) < 0.3  # also rules out the mirrored pose
    assert r.side.confidence > 0.7


# ---------------------------------------------------------------- record / replay


def test_record_then_replay_is_identical(fm, tmp_path, record_property):
    w = default_world(fm, "walk", seed=7, duration=5.0, start=START)
    rec = Recorder()
    game = ScriptedGame([(0.0, "initial")])
    a = Bench(
        fm, sim_camera(fm, w), SimOdometry(w), SimHead(w), game=game, truth=w, seed=3, recorder=rec
    ).run(5.0)
    path = rec.save(tmp_path / "run.npz")
    rp = Recording(path)
    b = Bench(
        fm, ReplayCamera(rp), ReplayOdometry(rp), ReplayHead(rp), game=ReplayGame(rp), truth=rp, seed=3
    ).run(5.0)
    show(record_property, "Recording size", f"{path.stat().st_size / 1024:.0f} KiB", "(info)")
    show(
        record_property,
        "Largest estimate difference, replay vs original",
        f"{np.abs(a.est.pose - b.est.pose).max():.1e}",
        "0 (bit-exact)",
    )
    assert np.array_equal(a.est.pose, b.est.pose)
    assert np.allclose(a.gt.pose, b.gt.pose)
    assert [e.kind for e in a.events] == [e.kind for e in b.events]


# ---------------------------------------------------------------- CLI


def test_cli_sim_record_and_replay(tmp_path):
    rec = tmp_path / "s.npz"
    main(["--duration", "3", "--start", "known", "--record", str(rec), "--out", str(tmp_path / "a")])
    assert (tmp_path / "a" / "est_bench.csv").exists() and (tmp_path / "a" / "gt_bench.csv").exists()
    assert rec.exists()
    spec = f"replay:{rec}"
    r = main(
        [
            "--duration",
            "3",
            "--start",
            "known",
            "--camera",
            spec,
            "--odom",
            spec,
            "--head",
            spec,
            "--game",
            spec,
            "--truth",
            spec,
            "--out",
            str(tmp_path / "b"),
        ]
    )
    assert r == 0 and (tmp_path / "b" / "est_bench.csv").exists()


def test_cli_camera_only_without_truth(tmp_path):
    main(
        [
            "--duration",
            "1",
            "--camera",
            "synthetic-images",
            "--world",
            "still",
            "--odom",
            "stationary",
            "--head",
            "fixed:0,0.35",
            "--start",
            "known",
            "--truth",
            "none",
            "--out",
            str(tmp_path),
        ]
    )
    assert (tmp_path / "est_bench.csv").exists() and not (tmp_path / "gt_bench.csv").exists()


def test_cli_rejects_bad_source(tmp_path):
    with pytest.raises(SystemExit):
        main(["--camera", "webcam", "--out", str(tmp_path)])
