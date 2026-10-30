"""Bench CLI: run localization with any mix of real, recorded and simulated inputs.

Examples (from localization/, PYTHONPATH="td_field_model;td_game;td_localization;td_bench"):

  # Everything simulated (sanity check)
  python -m td_bench --camera sim --odom sim --head sim --out reports/bench_sim

  # Only the camera is real: D435 on a tripod at a measured pose, nothing else connected
  python -m td_bench --camera realsense --odom stationary --head fixed:0,0.35 --truth known:-3,1,0.3 \
      --start known --world still --mount-height 1.2 --realtime --duration 60 --record runs/tripod.npz --out
      reports/tripod

  # Same, without a camera: rendered images through the same detector path
  python -m td_bench --camera synthetic-images --world tripod --odom stationary --head sim --start known --out
  reports/img

  # Everything but the camera: real GameController + teammates over UDP, simulated observations
  python -m td_bench --camera sim --odom sim --head sim --game udp:7,2 --team udp:7,2 --realtime --out
  reports/net

  # Replay a recorded session against changed code
  R=replay:runs/tripod.npz
  python -m td_bench --camera $R --odom $R --head $R --game $R --truth $R \
      --start known --world still --out
      reports/replay

Real encoders / IMU without ROS: use the Python API with td_bench.sources.LiveOdometry / LiveHead and push().
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from td_field_model.sim import CameraModel, NoiseModel, Simulator

from td_field_model import FieldModel


def _floats(s: str) -> list[float]:
    return [float(v) for v in s.split(",")]


def build_and_run(argv=None):
    from td_bench.bench import Bench
    from td_bench.record import Recorder, Recording, ReplayCamera, ReplayGame, ReplayHead, ReplayOdometry
    from td_bench.sources import (
        FixedHead,
        ImuHead,
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
    from td_bench.vision import D435_HALF, FolderFrames, ImageCamera, RealSenseFrames, SyntheticFrames

    ap = argparse.ArgumentParser(prog="python -m td_bench", description=__doc__.split("\n")[0])
    ap.add_argument("--field", default="hsl_m_2026")
    ap.add_argument(
        "--world", default="walk", choices=["walk", "tripod", "still"], help="for simulated parts"
    )
    ap.add_argument("--start", default="search", help="'known' (start pose given to the filter) or 'search'")
    ap.add_argument(
        "--start-pose", default="-3,1,0.3", help="x,y,theta of the robot/tripod (field frame, m, rad)"
    )
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument(
        "--camera", default="sim", help="sim | synthetic-images | folder:DIR[@fps] | realsense | replay:FILE"
    )
    ap.add_argument("--intrinsics", default=None, help="fx,fy,cx,cy,width,height for folder frames")
    ap.add_argument("--mount-height", type=float, default=1.2, help="camera height above the floor (m)")
    ap.add_argument(
        "--odom",
        default="sim",
        help="sim (legs) | vo (simulated cuVSLAM) | none (motion unknown) | stationary | replay:FILE",
    )
    ap.add_argument(
        "--head",
        default="sim",
        help="sim (exact) | imu (encoders + head IMU) | fixed:YAW,PITCH | replay:FILE",
    )
    ap.add_argument("--game", default="none", help="none | script:FILE.json | udp:TEAM,PLAYER | replay:FILE")
    ap.add_argument("--team", default="none", help="none | sim | udp:TEAM,PLAYER")
    ap.add_argument(
        "--ball", default="auto", help="auto | none | sim (auto = sim if the team source needs it)"
    )
    ap.add_argument("--truth", default="auto", help="auto | none | known:X,Y,THETA | replay:FILE")
    ap.add_argument(
        "--realtime", action="store_true", help="pace the loop to wall time (needed for live sources)"
    )
    ap.add_argument("--record", default=None, help="save the session to this .npz for replay")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)

    fm = FieldModel.builtin(a.field)
    start = tuple(_floats(a.start_pose))
    world = default_world(fm, a.world, seed=a.seed, duration=a.duration, start=start)
    recs: dict[str, Recording] = {}

    def rec(spec: str) -> Recording:
        path = spec.split(":", 1)[1]
        return recs.setdefault(path, Recording(path))

    used_sim = False
    # head first: image cameras need it
    if a.head == "sim":
        head, used_sim = SimHead(world), True
    elif a.head == "imu":
        head, used_sim = ImuHead(world, seed=a.seed), True
    elif a.head.startswith("fixed:"):
        head = FixedHead(*_floats(a.head.split(":", 1)[1]))
    elif a.head.startswith("replay:"):
        head = ReplayHead(rec(a.head))
    else:
        raise SystemExit(f"bad --head {a.head}")

    closers = []
    if a.camera == "sim":
        camera, used_sim = (
            SimCamera(world, Simulator(fm, noise=NoiseModel(false_positives=2.0), seed=a.seed)),
            True,
        )
    elif a.camera == "synthetic-images":
        camera = ImageCamera(SyntheticFrames(fm, D435_HALF, world, seed=a.seed), D435_HALF, head)
        used_sim = True
    elif a.camera.startswith("folder:"):
        spec = a.camera.split(":", 1)[1]
        folder, _, fps = spec.partition("@")
        if not a.intrinsics:
            raise SystemExit("--camera folder:... needs --intrinsics fx,fy,cx,cy,width,height")
        fx, fy, cx, cy, w, h = _floats(a.intrinsics)
        cam = CameraModel(fx, fy, cx, cy, int(w), int(h), a.mount_height)
        camera = ImageCamera(FolderFrames(folder, float(fps or 15)), cam, head)
    elif a.camera == "realsense":
        frames = RealSenseFrames()
        closers.append(frames.close)
        camera = ImageCamera(frames, frames.camera_model(a.mount_height), head)
    elif a.camera.startswith("replay:"):
        camera = ReplayCamera(rec(a.camera))
    else:
        raise SystemExit(f"bad --camera {a.camera}")

    if a.odom == "sim":
        odom, used_sim = SimOdometry(world), True
    elif a.odom == "stationary":
        odom = StationaryOdometry()
    elif a.odom == "vo":
        odom, used_sim = SimVisualOdometry(world, head, seed=a.seed), True
    elif a.odom == "none":
        odom = StationaryOdometry(sigma=(0.012, 0.012, 0.02))  # walking, motion unknown
    elif a.odom.startswith("replay:"):
        odom = ReplayOdometry(rec(a.odom))
    else:
        raise SystemExit(f"bad --odom {a.odom}")

    game = None
    if a.game.startswith("script:"):
        game = ScriptedGame(json.loads(Path(a.game.split(":", 1)[1]).read_text()))
    elif a.game.startswith("udp:"):
        from td_game.udp import GameControllerLink

        team_no, player = (int(v) for v in a.game.split(":", 1)[1].split(","))
        link = GameControllerLink()
        closers.append(link.close)
        game = UdpGame(link, team_no, player)
    elif a.game.startswith("replay:"):
        game = ReplayGame(rec(a.game))
    elif a.game != "none":
        raise SystemExit(f"bad --game {a.game}")

    ball = None
    want_ball = a.ball == "sim" or (a.ball == "auto" and a.team != "none")
    if want_ball:
        from td_localization.suite import BallTrack

        ball, used_sim = SimBall(world, BallTrack(fm, seed=a.seed, start=(-1.5, 0.5)), seed=a.seed), True

    team = None
    bench_kw = {}
    if a.team == "sim":
        team = SimTeam(ball, seed=a.seed)
    elif a.team.startswith("udp:"):
        from td_game.udp import TeamLink

        from td_game import TeamComm

        team_no, player = (int(v) for v in a.team.split(":", 1)[1].split(","))
        link = TeamLink(team_no)
        closers.append(link.close)
        bench_kw["player"] = player
        team = ("udp", link, TeamComm(player), game)
    elif a.team != "none":
        raise SystemExit(f"bad --team {a.team}")

    if a.truth == "auto":
        truth = world if used_sim else None
    elif a.truth == "none":
        truth = None
    elif a.truth.startswith("known:"):
        truth = tuple(_floats(a.truth.split(":", 1)[1]))
    elif a.truth.startswith("replay:"):
        truth = rec(a.truth)
    else:
        raise SystemExit(f"bad --truth {a.truth}")

    start_poses = [start] if a.start == "known" else None
    recorder = Recorder() if a.record else None
    bench = Bench(
        fm,
        camera,
        odom,
        head,
        game,
        None,
        ball,
        truth,
        start_poses,
        seed=a.seed,
        recorder=recorder,
        **bench_kw,
    )
    if isinstance(team, tuple):  # UdpTeam needs the bench's bridge
        _, link, comm, g = team
        team = UdpTeam(link, comm, bench.bridge, tracker=g.tracker if isinstance(g, UdpGame) else None)
    bench.team = team
    realtime = a.realtime or a.camera == "realsense"
    try:
        result = bench.run(a.duration, realtime=realtime)
    finally:
        for c in closers:
            c()
    notes = (
        f"camera={a.camera} odom={a.odom} head={a.head} game={a.game} team={a.team} ball={a.ball} "
        f"truth={a.truth} "
        f"start={a.start} world={a.world} field={a.field} seed={a.seed}"
    )
    out = result.report(a.out, "bench", notes=notes)
    if recorder is not None:
        print(f"recorded {recorder.save(a.record)}")
    print(f"wrote {out}")
    e = result.est.pose[-1]
    print(
        f"final estimate x={e[0]:.2f} y={e[1]:.2f} theta={math.degrees(e[2]):.1f}deg  "
        f"side confidence={result.side.confidence:.2f}  ticks w/o observations={result.frames_without_obs}"
    )
    return result


def main(argv=None) -> int:
    build_and_run(argv)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
