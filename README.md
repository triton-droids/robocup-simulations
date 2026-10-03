# robocup-simulations

Simulation and policy training for the Triton Droids RoboCup humanoid, built on
[NVIDIA ProtoMotions](https://github.com/NVlabs/ProtoMotions). ProtoMotions and the
IsaacLab revision it is tested against are vendored as git subtrees under `third-party/`,
so a clone of this repo is everything you need.

Before touching anything under `third-party/`, read [docs/protomotions.md](docs/protomotions.md).

## Layout

| Path                      | What                                                                                              |
| ------------------------- | ------------------------------------------------------------------------------------------------- |
| `robots/<robot>/`         | Robot descriptions (MJCF, meshes). See `robots/triton_humanoid/README.md`.                        |
| `robocup_rl/`             | Our package: robot configs and one directory per experiment, each holding its own `results/`. Installed editable by `uv sync`; registers robots with ProtoMotions via entry points. |
| `tests/`                  | `pytest` smoke tests (needs the `dev` extra).                                                     |
| `third-party/`            | Vendored ProtoMotions and IsaacLab subtrees. Owner approval needed for edits.                      |

## Usage

With an environment active (see below).

### Tests

```sh
pytest                                                   # whole suite; Newton cases skip without a GPU
pytest tests/test_triton_humanoid.py::test_kinematics    # one test
pytest -k newton -v                                      # by keyword, here the Newton cases (GPU)
pytest tests/test_triton_humanoid_isaaclab.py            # IsaacLab (GPU); runs alone, in its own process
```

Tests need the `dev` extra. The IsaacLab file stays separate because Isaac Sim must start
before torch is imported.

### Training

Every experiment is one directory, `robocup_rl/experiments/<experiment>/`, holding a
`config.py`; [docs/tutorials/adding-an-experiment.md](docs/tutorials/adding-an-experiment.md)
walks through adding one. The command is the same for all of them; only `<experiment>` and
`<run>` change:

```sh
cd robocup_rl/experiments/<experiment>              # outputs go to this directory's results/<run>/
protomotions-train-agent --robot-name triton_humanoid --simulator newton \
  --num-envs 1024 --batch-size 4096 --motion-file none \
  --experiment-path config.py --experiment-name <run> --headless \
  --training-max-iterations 1000 --use-wandb --wandb-project robocup-triton-humanoid
```

- Train on Newton or IsaacLab (GPU). The MuJoCo env is for inspecting and evaluating only.
- Re-running with the same `--experiment-name` resumes from `results/<run>/last.ckpt`.
- `--batch-size` must divide `--num-envs` × 32. `--motion-file none` is required by the CLI for
  motion-free experiments. Drop `--use-wandb ...` for TensorBoard only; W&B reads its key from
  `~/.netrc` (`wandb login` writes it, `wandb login --verify` checks it).

## Environments

ProtoMotions supports several physics backends, and their dependencies conflict
(different torch, wandb, and tensordict pins), so **each simulator gets its own named
environment at the repo root**. Create the ones you need; activate one at a time.

| Environment      | Simulator | Python | Needs                      | Use it for                                |
| ---------------- | --------- | ------ | -------------------------- | ----------------------------------------- |
| `.venv-mujoco`   | MuJoCo    | 3.11   | nothing (CPU, any OS*)     | local debugging, single-env inference     |
| `.venv-newton`   | Newton    | 3.11   | NVIDIA GPU                 | GPU training without Isaac Sim            |
| `.venv-isaaclab` | IsaacLab  | 3.12   | NVIDIA GPU, Linux x86_64   | large-scale training, Isaac Sim rendering |

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
uv sync --active --extra mujoco          # installs third-party/protomotions and robocup_rl in place
protomotions info --json                 # smoke test: prints the asset root and which simulators import
```

If `uv sync` cannot resolve on your platform (older macOS, see above) but you have a working
env anyway, `uv pip install --no-deps -e .` still installs `robocup_rl` and its entry points.

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

IsaacLab is not on PyPI, so `third-party/IsaacLab` is vendored at the commit ProtoMotions is
pinned to, with its own `uv.lock`. Sync it first, then add ProtoMotions on top (`uv sync`
removes anything not in that lock, so ProtoMotions goes last). Needs Linux x86_64, glibc 2.35+,
and an NVIDIA driver.

```sh
uv venv .venv-isaaclab --python 3.12 --seed
source .venv-isaaclab/bin/activate

# 1. IsaacLab (editable) + Isaac Sim 6.0 + torch cu128, from the committed lock (several GB)
uv sync --active --project third-party/IsaacLab --extra isaacsim

# 2. ProtoMotions (editable) and robocup_rl. The pillow override settles an Isaac Sim / moviepy pin conflict.
UV_EXTRA_INDEX_URL=https://pypi.nvidia.com UV_INDEX_STRATEGY=unsafe-best-match UV_PRERELEASE=allow \
  uv pip install --override <(echo pillow==12.1.1) -e "third-party/protomotions[isaaclab,dev]"
uv pip install --no-deps -e .

# 3. Accept the Isaac Sim EULA for headless runs
export OMNI_KIT_ACCEPT_EULA=yes
protomotions info --json
```

Do not add upstream's `requirements_isaaclab.txt` on top; its floors override the pins.

- **After a ProtoMotions sync merge:** re-run step 2.
- **Re-sync IsaacLab's own packages** (rare): `uv sync --active --inexact --project third-party/IsaacLab --extra isaacsim`.
- **Do not move or delete `third-party/IsaacLab`** while the env exists; it is installed editable.

### Which one am I in?

```sh
python -c "import protomotions, sys; print(sys.prefix, protomotions.__file__)"
```

The prefix names the environment; the module path should always be inside this repo's
`third-party/protomotions`.
