"""Connects GameController events (td_game) to localization (LOC-08 → LOC-05/06). Pure Python, no ROS.

- "initial" and "replaced": robots placed by handlers in our own half → SideBelief.on_initial. Uses the team's
  configured starting spots if given, else an own-half search.
- "penalized" (standard removal): the robot is carried off. `paused` becomes True; the node must stop
  feeding odometry and vision to the filter until re-entry. Being carried isn't motion the legs measure,
  and the off-field view is meaningless.
- "reentry": SideBelief.on_penalty_reentry with both own-half touchline candidates (rules: level with the
  penalty mark, outside the field, facing the opposite touchline). Vision picks the touchline.
- "unpenalized_in_place" (Motion in Set): nothing to re-seed. The robot never moved.
- Teammates' TeamMessages become SideBelief TeamReports (only fresh, seen balls).

Our `map` frame (own goal at -x) equals the GameController's status-message frame, so no conversion is needed.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from td_field_model.scenarios import penalty_reentry_poses

from td_field_model import FieldModel
from td_game import GameEvent, TeamMessage
from td_localization.side import SideBelief, TeamReport


@dataclass
class GameBridge:
    side: SideBelief
    field_model: FieldModel
    start_poses: list | None = None  # known Initial placements for this robot, or None for own-half search
    max_ball_age: float = 3.0  # s; older teammate ball sightings are ignored (SideBelief matches the time)
    paused: bool = False
    log: list[str] = field(default_factory=list)

    def on_events(self, events: list[GameEvent]) -> None:
        for e in events:
            if e.kind in ("initial", "replaced"):
                self.paused = False
                self.side.on_initial(e.t, self.start_poses)
                self.log.append(
                    f"{e.t:.2f} {e.kind}: re-seeded ({'known poses' if self.start_poses else 'search'})"
                )
            elif e.kind == "penalized" and not e.in_place:
                self.paused = True
                self.log.append(f"{e.t:.2f} penalized ({e.detail}): localization paused")
            elif e.kind == "reentry":
                self.paused = False
                self.side.on_penalty_reentry(e.t, penalty_reentry_poses(self.field_model))
                self.log.append(f"{e.t:.2f} reentry ({e.detail}): two touchline candidates")

    def team_report(self, t: float, msg: TeamMessage) -> TeamReport | None:
        """Convert a teammate's message into side-belief evidence, or None if it carries none."""
        if msg.ball_xy is None or msg.ball_age > self.max_ball_age or msg.fallen:
            return None
        return TeamReport(
            t - msg.ball_age,
            msg.player_number,
            msg.ball_xy,
            msg.ball_sigma,
            msg.side_confidence,
            msg.is_goalkeeper,
        )
