# Getting started with localization

There are two tracks. Both cover the same project; pick the one that fits you.

| Track | For | Style |
|---|---|---|
| **[Humans](humans/01_welcome.md)** | New team members, including people who have never worked on robotics, localization or probability. | In depth. Every idea is built up step by step, with worked examples, the math explained in plain words, code tours and exercises you can run. |
| **[Agents](agents/README.md)** | AI coding agents, and experienced developers who want the facts fast. | Short and precise: layout, contracts, invariants, commands, status, workflow. |

## Human track, in reading order

| # | Chapter | You will learn |
|---|---|---|
| 1 | [Welcome](humans/01_welcome.md) | What the robot is, what RoboCup asks of it, what "localization" means and why this field makes it hard. |
| 2 | [Setup and first run](humans/02_setup.md) | How to install everything, run the tests, open the results and run your first simulated robot. |
| 3 | [Math toolkit, part 1: positions and angles](humans/03_math_geometry.md) | Coordinate frames, poses, radians, rotating and moving points, and composing motions. |
| 4 | [Math toolkit, part 2: uncertainty](humans/04_math_probability.md) | Noise, the normal distribution, standard deviation and covariance, Bayes' rule and log-odds. |
| 5 | [The field model](humans/05_field_model.md) | How the code describes the soccer field, measures distance to a line, and why the field is symmetric. |
| 6 | [Seeing the field: the camera](humans/06_camera.md) | How a pixel becomes a point on the floor, why head angles matter so much, and how the simulator fakes a camera. |
| 7 | [The particle filter](humans/07_particle_filter.md) | The core algorithm, one step at a time: particles, predict, update, resample, recover and estimate. |
| 8 | [Which half am I on?](humans/08_side_belief.md) | How the robot tells its own half from the opponent's, using game events and teammates. |
| 9 | [The GameController and teammates](humans/09_game_and_team.md) | The referee's network messages, the team message format and the message budget. |
| 10 | [Testing and measuring](humans/10_testing_and_evaluation.md) | Unit tests, simulated scenarios, the evaluation metrics, the bench and the test report. |
| 11 | [Where the project stands](humans/11_progress.md) | What works, what's measured, what's missing, and what comes next. |
| 12 | [How to contribute](humans/12_contributing.md) | Picking a task, the git workflow, writing tests, code style, reviews, and when to ask. |

Chapters 3 and 4 are the math foundation. If you're comfortable with 2-D rotations, Gaussians and Bayes' rule,
skim them.

## Agent track

| Doc | Contents |
|---|---|
| [README](agents/README.md) | Read order and the hard rules. |
| [Orientation](agents/01_orientation.md) | Layout, environment, commands, conventions, invariants. |
| [Architecture](agents/02_architecture.md) | Modules, APIs, data contracts, the per-tick loop, key parameters. |
| [Status](agents/03_status.md) | What's done, measured results, known limits, ready tickets. |
| [Contributing](agents/04_contributing.md) | Workflow, done criteria, stop-and-ask list, handoff and PR templates, gotchas. |

The reference docs both tracks build on are the [roadmap](../roadmap.md) (rules, design) and the
[implementation plan](../implementation_plan.md) (tickets, status, team decisions).
