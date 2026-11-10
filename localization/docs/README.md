# Localization docs

| Doc | Read it for |
|---|---|
| [getting_started/](getting_started/README.md) | **Start here if you're new.** A series in two tracks: [humans](getting_started/humans/01_welcome.md) (in depth: the problem, the math, a tour of the code, testing, progress, contributing) and [agents](getting_started/agents/README.md) (concise: layout, invariants, APIs, status, workflow). |
| [roadmap.md](roadmap.md) | What existed when this started (no localization anywhere in the team repos), the hardware, the HSL 2026 rules that constrain localization (§2), the design (§3.5 fusion and side handling, §3.6 the cuVSLAM assessment) and the phased plan (§4). **Read it first.** |
| [implementation_plan.md](implementation_plan.md) | Tickets LOC-01 to LOC-25, with dependencies, labels and done criteria; rules for agents (§0); the status tracker (§6); the team decision log (§9). |
| [figures/](figures/) | Pipeline diagrams: detailed and simple localization pipelines, and the implementation pipeline. Each `.svg` is rendered from the `.dot` beside it (`dot -Tsvg x.dot -o x.svg`). |
| [handoffs/](handoffs/) | One note per agent work session, newest last. Read the newest one before continuing. |

## Rules and decisions that constrain the design

These are summaries; the docs above give the full reasoning.

- **Symmetric field:** the lines don't tell the halves apart. Side comes from game events (initial positions,
  penalty re-entry) and teammates' ball reports. It never comes from a single robot's vision alone.
- **No compass:** magnetometers must stay unused (the BNO085 game rotation vector is fine).
- **D435 projector off** (team policy): passive IR stereo is allowed, active-IR RGB-D is not (roadmap §2.2). The
  TC's written confirmation is still open (LOC-01).
- **Team messages:** 12,000 per game, at most 512 B each, UDP port 10000 + team number. `td_game` paces sends
  with the budget the GameController reports.
- **Side consensus rule** (team decision, 2026-10-27): a robot flips halves only on evidence from the goalkeeper
  or from ≥ 2 different teammates within 15 s.
