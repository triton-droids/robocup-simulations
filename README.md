# robocup-simulations

Simulation and policy training for the Triton Droids RoboCup humanoid, built on
[NVIDIA ProtoMotions](https://github.com/NVlabs/ProtoMotions). ProtoMotions and the
IsaacLab revision it is tested against are vendored as git subtrees under `third-party/`,
so a clone of this repo is everything you need.

Before touching anything under `third-party/`, read [docs/protomotions.md](docs/protomotions.md).

## Environments

ProtoMotions supports several physics backends, and their dependencies conflict
(different torch, wandb, and tensordict pins), so **each simulator gets its own named
environment at the repo root**. Create the ones you need; activate one at a time.

| Environment      | Simulator | Python | Needs                      | Use it for                                |
| ---------------- | --------- | ------ | -------------------------- | ----------------------------------------- |
| `.venv-mujoco`   | MuJoCo    | 3.11   | nothing (CPU, any OS*)     | local debugging, single-env inference     |
| `.venv-newton`   | Newton    | 3.11   | NVIDIA GPU                 | GPU training without Isaac Sim            |
| `.venv-isaaclab` | IsaacLab  | 3.12   | NVIDIA GPU, Linux x86_64   | large-scale training, Isaac Sim rendering |

\* The MuJoCo and Newton extras pin `tensordict==0.9.0`, which ships macOS wheels only
for macOS 15+. Isaac Sim has no macOS build at all. On older macOS, use a Linux box.

Install [uv](https://docs.astral.sh/uv/getting-started/installation/) once; it downloads
the right Python for each environment.

The pattern is the same for every environment:

```sh
uv venv .venv-<sim> --python <version>   # create, once
source .venv-<sim>/bin/activate          # select
uv sync --active ...                     # install or update into the active env
```

Once an environment is active, run `python`, `protomotions`, `protomotions-train-agent`,
and so on directly. Do **not** use `uv run` without `--active`: it targets `.venv`, which
does not exist here, and would quietly create a fourth environment.

### MuJoCo

```sh
uv venv .venv-mujoco --python 3.11
source .venv-mujoco/bin/activate
uv sync --active --extra mujoco          # installs third-party/protomotions in place
protomotions info --json                 # smoke test: prints the asset root and which simulators import
```

- **After `git pull`:** `uv sync --active --extra mujoco` again. uv re-resolves if the
  vendored `pyproject.toml` changed.
- **Add a package:** `uv add <name>` (lands in the root `pyproject.toml` and `uv.lock`;
  commit both). `uv add --optional mujoco <name>` for something only this env needs.
- **Dev tools** (pytest, ONNX export): `uv sync --active --extra mujoco --extra dev`.
- **Start over:** `rm -rf .venv-mujoco` and repeat the three lines above.

### Newton

Same project, different extra and name:

```sh
uv venv .venv-newton --python 3.11
source .venv-newton/bin/activate
uv sync --active --extra newton
protomotions info --json
```

Maintenance is identical to MuJoCo with `--extra newton`.

### IsaacLab

IsaacLab is not installable from PyPI, so `third-party/IsaacLab` holds the exact commit
ProtoMotions is pinned to. Its own installer (`isaaclab.sh -i`) populates the active
environment with Isaac Sim 6.0, CUDA torch, and the IsaacLab packages in editable mode,
using `uv pip` under the hood. ProtoMotions goes in afterwards. (IsaacLab's `uv sync`
path is broken at this commit; its lockfile cannot resolve, so use the installer.)

Requires Linux x86_64, an NVIDIA driver, and `cmake` (the installer runs
`sudo apt-get install cmake build-essential` if it is missing).

```sh
uv venv .venv-isaaclab --python 3.12 --seed
source .venv-isaaclab/bin/activate

# 1. Isaac Sim 6.0 + torch (cu128) + IsaacLab packages, editable from third-party/IsaacLab (several GB)
third-party/IsaacLab/isaaclab.sh -i isaacsim

# 2. ProtoMotions with its IsaacLab extra, editable, from this repo
UV_EXTRA_INDEX_URL=https://pypi.nvidia.com UV_INDEX_STRATEGY=unsafe-best-match UV_PRERELEASE=allow \
  uv pip install -e "third-party/protomotions[isaaclab]"
uv pip install -r third-party/protomotions/requirements_isaaclab.txt

# 3. Isaac Sim asks you to accept the NVIDIA EULA on first launch; this answers it for headless runs
export OMNI_KIT_ACCEPT_EULA=yes
protomotions info --json
```

Then, with the env active, from the repo root:

```sh
protomotions-train-agent --simulator isaaclab --headless ...
```

- **After a ProtoMotions sync merge:** re-run step 2 with the env active. The editable
  install picks up code changes by itself; this only matters when upstream dependencies move.
- **Do not move or delete `third-party/IsaacLab`** while the env exists: it is installed
  editable and resolves its `apps/` directory relative to the source tree at import time.
- **When ProtoMotions bumps its IsaacLab pin**, an owner updates the subtree (see
  `docs/protomotions.md`); then `rm -rf .venv-isaaclab` and repeat the steps above.
- **Start over:** `rm -rf .venv-isaaclab` and repeat the steps above.

### Which one am I in?

```sh
python -c "import protomotions, sys; print(sys.prefix, protomotions.__file__)"
```

The prefix names the environment; the module path should always be inside this repo's
`third-party/protomotions`.
