# robocup-simulations

Read `README.md` for setup and `docs/protomotions.md` before touching `third-party/`.

## Keep the repo root clean

- `results/` and `output/` directories belong **directly under the experiment that produced
  them**: `robocup_rl/experiments/<name>/results/<run>/` and
  `robocup_rl/experiments/<name>/output/`. They must never appear at the repo root or anywhere
  else in the tree.
- ProtoMotions writes both relative to the current working directory (`results/<experiment>`
  from the trainer, `output/renderings/` from the recorder as soon as any simulator starts), so
  always launch training and inference from inside `robocup_rl/experiments/<name>/`.
- `pytest` runs from the repo root; `tests/conftest.py` moves each test into a temporary
  directory so simulator start-up cannot drop `output/` here.
- Job-scheduler logs (SLURM `-o`) go under `~/.cache/`, never into the repo.
- If a stray `results/` or `output/` shows up at the root, delete it and fix whatever launched
  from the wrong directory; do not add it to `.gitignore`.

## Environments

One named env per simulator at the root (`.venv-mujoco`, `.venv-newton`, `.venv-isaaclab`).
MuJoCo is for inspecting and evaluating on CPU only; training runs on Newton or IsaacLab on
the GPU node.

## Markdown

- When prose names a class whose definition is in this tree (ours or a vendored subtree), link
  the first mention in the file to the defining file with a relative path:
  [`MdpComponent`](third-party/protomotions/protomotions/envs/mdp_component.py). Link the file
  only, never a `#L<n>` line anchor; anchors go stale on the next edit. Later mentions in the
  same file and names inside code blocks stay plain, as do classes from pip packages (Newton's
  `SolverMuJoCo`), which have no file here.
