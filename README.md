# robocup-simulations

Simulation and policy training for the Triton Droids RoboCup humanoid, built on
[NVIDIA ProtoMotions](https://github.com/NVlabs/ProtoMotions), which is vendored
as a git subtree under `third-party/protomotions`.

Before touching anything under `third-party/`, read [docs/protomotions.md](docs/protomotions.md).

## Environments

ProtoMotions supports several physics backends, and their dependencies conflict
(different torch, wandb, and tensordict pins), so **each simulator gets its own
environment**. You never need more than one at a time, but you can keep several
on disk side by side.

| Simulator | Managed by                        | Python | Needs                       | Use it for                                  |
| --------- | --------------------------------- | ------ | --------------------------- | ------------------------------------------- |
| MuJoCo    | this repo's `uv sync`             | 3.11   | nothing (CPU, any OS*)      | local debugging, single-env inference       |
| Newton    | this repo's `uv sync`             | 3.11   | NVIDIA GPU                  | GPU training without Isaac Sim              |
| IsaacLab  | the pinned IsaacLab checkout's uv | 3.12   | NVIDIA GPU, Linux x86_64    | large-scale training, Isaac Sim rendering   |
| IsaacGym  | conda (legacy)                    | 3.8    | NVIDIA GPU, Linux, download | only if you specifically need IsaacGym      |

\* The MuJoCo and Newton extras pin `tensordict==0.9.0`, which ships macOS wheels
only for macOS 15+. On older macOS use a Linux box.

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) once. It
downloads the right Python for you.

### MuJoCo (default, uv-managed)

```sh
uv sync --extra mujoco              # creates .venv with Python 3.11 and installs third-party/protomotions in place
uv run protomotions info --json     # smoke test
uv run protomotions-train-agent --help
```

`uv run <cmd>` runs inside `.venv` without activating it. If you prefer a shell,
`source .venv/bin/activate`.

Day to day:

- **After `git pull`:** run `uv sync --extra mujoco` again. uv re-resolves if the
  vendored `pyproject.toml` changed and re-installs anything that moved.
- **Add a package:** `uv add <name>` (it lands in the root `pyproject.toml` and
  `uv.lock`; commit both). Use `uv add --optional mujoco <name>` for something only
  the MuJoCo environment needs.
- **Dev tools** (pytest, ONNX export): `uv sync --extra mujoco --extra dev`.
- **Start over:** `rm -rf .venv && uv sync --extra mujoco`.

### Newton (GPU, uv-managed, optional)

Same project, different extra. Give it its own directory so it doesn't replace the
MuJoCo environment:

```sh
UV_PROJECT_ENVIRONMENT=.venv-newton uv sync --extra newton
UV_PROJECT_ENVIRONMENT=.venv-newton uv run protomotions-train-agent --simulator newton ...
```

Running `uv sync --extra newton` without that variable would turn `.venv` into a
Newton environment (uv removes the MuJoCo-only packages). That is fine if you only
want one.

### IsaacLab (GPU, Linux x86_64)

IsaacLab is not resolvable from PyPI, so this environment is **owned by a pinned
IsaacLab checkout**, not by this repo's `pyproject.toml`. ProtoMotions is then
installed into it as an editable package. Upstream's reference is
`third-party/protomotions/docs/source/getting_started/installation.rst`.

One-time setup:

```sh
# 1. Pinned IsaacLab checkout (the commit ProtoMotions is tested against)
git clone https://github.com/isaac-sim/IsaacLab.git ~/IsaacLab
cd ~/IsaacLab
git checkout 4ecd0b036da19ff6ad2bb4d621f886b63e9f6db8

# 2. IsaacLab's own uv environment with Isaac Sim 6.0 (Python 3.12)
uv sync --extra isaacsim
source .venv/bin/activate

# 3. ProtoMotions from this repo, editable, into that environment
PM=/path/to/robocup-simulations/third-party/protomotions
uv pip install -e "$PM[isaaclab]" --extra-index-url https://pypi.nvidia.com
uv pip install -r "$PM/requirements_isaaclab.txt"
```

Isaac Sim asks you to accept the NVIDIA EULA the first time it starts. Do that
interactively before launching headless jobs.

Using it:

```sh
source ~/IsaacLab/.venv/bin/activate
cd /path/to/robocup-simulations
protomotions info --json
protomotions-train-agent --simulator isaaclab ...
```

Call `python` / `protomotions-*` directly with the IsaacLab venv active. Do not use
`uv run` here: that always targets this repo's `.venv` (the MuJoCo environment).

Day to day:

- **After a ProtoMotions sync merge:** re-run step 3 inside the IsaacLab venv so new
  upstream dependencies land. The editable install picks up code changes by itself.
- **Bump IsaacLab:** only when upstream ProtoMotions moves its pinned commit. Check
  the commit hash in the installation guide above, `git checkout` it, and re-run
  steps 2 and 3.
- **Start over:** `rm -rf ~/IsaacLab/.venv` and repeat steps 2 and 3.

### IsaacGym (legacy)

Only if you need IsaacGym specifically. Python 3.8 and a manual download, so conda:

```sh
conda create -n isaacgym python=3.8 && conda activate isaacgym
# download Isaac Gym Preview 4 from https://developer.nvidia.com/isaac-gym and extract it
pip install -e isaacgym/python
pip install -e third-party/protomotions
pip install -r third-party/protomotions/requirements_isaacgym.txt
```

### Which one am I in?

```sh
python -c "import protomotions, sys; print(sys.prefix, protomotions.__file__)"
```

The prefix tells you the environment; the module path should always be inside this
repo's `third-party/protomotions`.
