"""Record a bench session once, replay it any number of times (LOC bench). numpy only, no pickle.

A recording holds, per bench tick:
- odometry increment + covariance, head angles;
- the camera's Observations (line points with covariance and range flags, landmarks if any);
- game events;
- ground truth if known.

So a session on real hardware (camera on a tripod, a robot walking, a GameController) can be re-run offline
against
changed localization code, and the results compared with td_localization.eval.

Replay sources (ReplayCamera, ReplayOdometry, ReplayHead, ReplayGame) plug into the same bench slots as live
ones.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from td_field_model.sim import Observations, ObservedLandmark

from td_game import GameEvent


class Recorder:
    def __init__(self):
        self.t, self.odom, self.odom_cov, self.head, self.truth = [], [], [], [], []
        self.obs_present, self.pt_off, self.pts, self.pcov, self.prange = [], [0], [], [], []
        self.lm_off, self.lm_kind, self.lm_xy, self.lm_cov, self.lm_range = [0], [], [], [], []
        self.ev_t, self.ev_kind, self.ev_detail, self.ev_in_place = [], [], [], []

    def add(self, t, odom, odom_cov, head, obs: Observations | None, events=(), truth=None) -> None:
        self.t.append(t)
        self.odom.append(np.asarray(odom, float))
        self.odom_cov.append(np.asarray(odom_cov, float))
        self.head.append(tuple(head))
        self.truth.append((np.nan, np.nan, np.nan) if truth is None else tuple(truth))
        self.obs_present.append(obs is not None)
        if obs is not None:
            self.pts.append(np.asarray(obs.line_points, float).reshape(-1, 2))
            self.pcov.append(np.asarray(obs.line_cov, float).reshape(-1, 2, 2))
            self.prange.append(np.asarray(obs.line_range_ok, bool).reshape(-1))
            for lm in obs.landmarks or []:
                self.lm_kind.append(lm.kind)
                self.lm_xy.append(np.asarray(lm.xy, float))
                self.lm_cov.append(np.asarray(lm.cov, float))
                self.lm_range.append(bool(lm.range_ok))
        self.pt_off.append(self.pt_off[-1] + (len(self.pts[-1]) if obs is not None else 0))
        self.lm_off.append(len(self.lm_kind))
        for e in events:
            self.ev_t.append(e.t)
            self.ev_kind.append(e.kind)
            self.ev_detail.append(e.detail)
            self.ev_in_place.append(e.in_place)

    def save(self, path) -> Path:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        cat2 = np.concatenate(self.pts) if self.pts else np.zeros((0, 2))
        np.savez_compressed(
            path,
            t=np.array(self.t),
            odom=np.array(self.odom).reshape(-1, 3),
            odom_cov=np.array(self.odom_cov).reshape(-1, 3, 3),
            head=np.array(self.head).reshape(-1, 2),
            truth=np.array(self.truth).reshape(-1, 3),
            obs_present=np.array(self.obs_present, bool),
            pt_off=np.array(self.pt_off),
            pts=cat2,
            pcov=np.concatenate(self.pcov) if self.pcov else np.zeros((0, 2, 2)),
            prange=np.concatenate(self.prange) if self.prange else np.zeros(0, bool),
            lm_off=np.array(self.lm_off),
            lm_kind=np.array(self.lm_kind, dtype="U16"),
            lm_xy=np.array(self.lm_xy).reshape(-1, 2),
            lm_cov=np.array(self.lm_cov).reshape(-1, 2, 2),
            lm_range=np.array(self.lm_range, bool),
            ev_t=np.array(self.ev_t, float),
            ev_kind=np.array(self.ev_kind, dtype="U32"),
            ev_detail=np.array(self.ev_detail, dtype="U64"),
            ev_in_place=np.array(self.ev_in_place, bool),
        )
        return path


class Recording:
    """A loaded recording with tick lookup by time."""

    def __init__(self, path):
        with np.load(path, allow_pickle=False) as z:
            self.z = {k: z[k] for k in z.files}
        self.t = self.z["t"]

    def index(self, t: float) -> int | None:
        """Last recorded tick at or before t (None before the first)."""
        i = int(np.searchsorted(self.t, t + 1e-9, side="right")) - 1
        return None if i < 0 else i

    def observations(self, i: int) -> Observations | None:
        z = self.z
        if not z["obs_present"][i]:
            return None
        a, b = z["pt_off"][i], z["pt_off"][i + 1]
        la, lb = z["lm_off"][i], z["lm_off"][i + 1]
        lms = [
            ObservedLandmark(
                str(z["lm_kind"][k]), z["lm_xy"][k], z["lm_cov"][k], bool(z["lm_range"][k]), None
            )
            for k in range(la, lb)
        ]
        return Observations(float(self.t[i]), z["pts"][a:b], z["pcov"][a:b], z["prange"][a:b], landmarks=lms)

    @property
    def truth(self) -> np.ndarray | None:
        tr = self.z["truth"]
        return None if np.all(np.isnan(tr)) else tr


class _ReplayBase:
    def __init__(self, rec: Recording):
        self.rec = rec
        self._last = -1

    def _new_ticks(self, t: float) -> range:
        i = self.rec.index(t)
        if i is None or i <= self._last:
            return range(0)
        r = range(self._last + 1, i + 1)
        self._last = i
        return r


class ReplayCamera(_ReplayBase):
    def get(self, t: float) -> Observations | None:
        obs = None
        for i in self._new_ticks(t):
            obs = self.rec.observations(i) or obs
        return obs


class ReplayOdometry(_ReplayBase):
    def get(self, t: float):
        from td_bench.sources import _compose_rel

        d, cov = None, np.zeros((3, 3))
        for i in self._new_ticks(t):
            # the first increment is taken as stored (bit-exact replay when bench ticks match recorded ticks)
            d = self.rec.z["odom"][i].copy() if d is None else _compose_rel(d, self.rec.z["odom"][i])
            cov = cov + self.rec.z["odom_cov"][i]
        return (np.zeros(3) if d is None else d), cov


class ReplayHead:
    def __init__(self, rec: Recording):
        self.rec = rec

    def get(self, t: float):
        i = self.rec.index(t)
        return (0.0, 0.35) if i is None else tuple(float(v) for v in self.rec.z["head"][i])


class ReplayGame:
    def __init__(self, rec: Recording):
        z = rec.z
        self._events = sorted(
            zip(
                z["ev_t"].tolist(),
                z["ev_kind"].tolist(),
                z["ev_detail"].tolist(),
                z["ev_in_place"].tolist(),
                strict=True,
            )
        )
        self._i = 0

    def get(self, t: float) -> list[GameEvent]:
        out = []
        while self._i < len(self._events) and self._events[self._i][0] <= t + 1e-9:
            te, kind, detail, in_place = self._events[self._i]
            out.append(GameEvent(te, kind, detail, in_place))
            self._i += 1
        return out
