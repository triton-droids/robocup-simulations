# 2. Setup and first run

**Series:** [index](../README.md) · [1 Welcome](01_welcome.md) · **2 Setup** · [3 Geometry](03_math_geometry.md) ·
[4 Uncertainty](04_math_probability.md) · [5 Field](05_field_model.md) · [6 Camera](06_camera.md) ·
[7 Particle filter](07_particle_filter.md) · [8 Which half](08_side_belief.md) · [9 Game and team](09_game_and_team.md) ·
[10 Testing](10_testing_and_evaluation.md) · [11 Progress](11_progress.md) · [12 Contributing](12_contributing.md)

By the end of this chapter you will have:

- the code on your computer;
- a Python environment with everything installed;
- all 237 tests passing;
- the test report open in your browser;
- run a simulated robot and read its results;
- enough numpy to read the rest of the code.

Budget about 30 minutes, most of it waiting for downloads and tests.

---

## 2.1 What you need

| Tool | Why | Check it's there |
|---|---|---|
| **Python 3.10 or newer** | All the code is Python. | `python3 --version` (Windows: `python --version`) |
| **git** | To download the code and, later, contribute. | `git --version` |
| **A terminal** | To type the commands below. macOS: Terminal. Windows: PowerShell. Linux: any. | |
| **A code editor** | To read the code. VS Code with the Python extension works well. | |

You do **not** need ROS, Docker, a GPU or the robot.

If Python is missing, install it from [python.org](https://www.python.org/downloads/) (on Windows, tick "Add
python.exe to PATH" in the installer). The code is tested with Python 3.10 and numpy 1.21.5, the versions on the
robot's Ubuntu 22.04.

## 2.2 Get the code

The code lives in the team repository `triton-droids/robocup-simulations`, on the branch `localization`, in the
folder `localization/`. A **branch** is a parallel version of the repository; this work isn't merged into `main`
yet.

```bash
git clone --branch localization https://github.com/triton-droids/robocup-simulations.git
cd robocup-simulations/localization
```

The repository also holds large simulation libraries you don't need here. If the download is slow, fetch only the
`localization/` folder instead:

```bash
git clone --filter=blob:none --sparse --branch localization https://github.com/triton-droids/robocup-simulations.git
cd robocup-simulations
git sparse-checkout set localization
cd localization
```

From now on, every command runs **inside the `localization/` folder**.

## 2.3 Create a virtual environment

A **virtual environment** ("venv") is a private Python installation for one project. Packages you install into it
don't affect anything else on your computer, and you can delete it at any time.

macOS / Linux:

```bash
python3 -m venv .venv-localization
source .venv-localization/bin/activate
pip install -r requirements.txt
```

Windows (PowerShell):

```powershell
python -m venv .venv-localization
.venv-localization\Scripts\Activate.ps1
pip install -r requirements.txt
```

If PowerShell refuses to run `Activate.ps1`, run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once, then
try again.

When the venv is active, your prompt starts with `(.venv-localization)`. **Activate it every time you open a new
terminal.** [`requirements.txt`](../../../requirements.txt) installs three things:

- **numpy**, the only library the code itself uses;
- **pytest**, which runs the tests;
- **ruff**, which checks code style.

The folder `.venv-localization/` is in [`.gitignore`](../../../.gitignore), so git never tries to commit it.

## 2.4 Run the tests

```bash
python run_tests.py
```

This runs every package's tests, then the style checker. It takes about 5 minutes; most of that is
`td_localization`, which simulates whole robot runs. The end of the output should look like this:

```text
== td_field_model
78 passed in 7.65s
== td_game
42 passed in 1.32s
== td_localization
86 passed in 227.07s (0:03:47)
== td_bench
31 passed in 80.99s (0:01:20)
== ruff
All checks passed!

all passed
```

Each dot pytest prints while running is one passing test; an `F` is a failure. To run just one package, name it:

```bash
python run_tests.py td_game
```

On Windows, the first run of `td_game` or `td_bench` may pop up a firewall prompt. Some tests send network messages
to your own computer (address `127.0.0.1`) to test the network code, and nothing leaves your machine. You can allow
it or cancel; the tests pass either way.

### Why `run_tests.py` and not plain `pytest`?

Two reasons, both worth knowing because you'll hit them when running things by hand:

1. **The packages aren't installed.** Python finds modules by searching a list of folders. The packages here
   aren't installed into the venv, so Python has to be told where they are, with the environment variable
   **`PYTHONPATH`**. For example, `td_bench` needs all four package folders on the path. `run_tests.py` sets this
   for you.
2. **Every package has a file called `test_import.py`.** pytest refuses to load two test files with the same name
   in one run, so each package runs separately. (This mirrors `colcon`, the build tool the robot uses.)

To run tests by hand, set `PYTHONPATH` first. macOS / Linux use `:` between folders:

```bash
PYTHONPATH=td_field_model:td_game:td_localization:td_bench python -m pytest -q td_bench/test
```

Windows PowerShell uses `;`:

```powershell
$env:PYTHONPATH = "td_field_model;td_game;td_localization;td_bench"
python -m pytest -q td_bench/test
```

Useful pytest options:

- `-k name` runs only tests whose name contains `name`;
- `-x` stops at the first failure;
- `-s` shows `print()` output.

## 2.5 Open the test report

[`test_report/index.html`](../../../test_report/index.html) is a web page that shows every test beside its result:

- on the left, what the test checks (its name, description and the conditions it asserts);
- on the right, whether it passed, how long it took and, for the bigger tests, the measured numbers and plots.

Open the file from your clone in any browser (double-click it, or drag it into a browser window). GitHub shows
HTML files as source code, so viewing it on github.com won't work.

Look at the "Hardware-mix runs" section. Each card is one simulated robot run, and the field plot shows the
**true path** (white, dashed) and the **estimated path** (blue). When they overlap, localization works. You'll
understand every number on that page by the end of chapter 10.

## 2.6 Your first simulated robot

The **bench** (package `td_bench`, [chapter 10](10_testing_and_evaluation.md)) runs the real localization code
with fake or real inputs. Run a 10-second simulated walk:

macOS / Linux:

```bash
export PYTHONPATH=td_field_model:td_game:td_localization:td_bench
python -m td_bench --duration 10 --start known --out reports/first_run
```

Windows PowerShell:

```powershell
$env:PYTHONPATH = "td_field_model;td_game;td_localization;td_bench"
python -m td_bench --duration 10 --start known --out reports/first_run
```

What the flags mean:

- `--duration 10`: simulate 10 seconds of robot time;
- `--start known`: tell the filter where the robot starts (the alternative, `search`, makes it find itself);
- `--out reports/first_run`: where to write results.

Every input defaults to "simulated", so this needs no hardware. It finishes in a few seconds and prints:

```text
wrote reports/first_run/report.md
final estimate x=-1.05 y=-0.10 theta=-113.9deg  side confidence=1.00  ticks w/o observations=150
```

Line by line:

- **final estimate**: where the filter thinks the robot ended up, in meters and degrees.
- **side confidence = 1.00**: it's sure it's on its own half (rounded; really 0.998).
- **ticks w/o observations = 150**: the loop runs 30 times per second (301 ticks in 10 s), and the simulated camera
  delivers a picture every second tick, so 150 ticks had no new image. That's expected.

Now open `reports/first_run/`:

| File | Contents |
|---|---|
| `report.md` | A summary table. Position p50 ≈ 0.014 m means half the time the estimate was within 1.4 cm of the truth. |
| `est_bench.csv` | The estimate at every tick: time, status, x, y, θ, side confidence. Open it in a spreadsheet. |
| `gt_bench.csv` | The **ground truth**: where the simulated robot really was. |
| `samples_bench.csv` | Truth and estimate side by side, with the error, for every tick. |
| `bench_log.txt` | Game events, here just `0.00 initial: re-seeded (known poses)`. |

Try `--start search` and compare. The filter now has to find the robot somewhere on its own half. In
`est_bench.csv`, the status is `initializing` for the first 2.5 s or so while it searches, then `localized`, with
the error back to centimeters.

The `reports/` folder is git-ignored, so experiment freely.

## 2.7 A short numpy primer

The code processes hundreds of guesses and hundreds of points at once with numpy, and it rarely uses `for` loops.
If numpy is new to you, this section saves a lot of confusion later. Run it in `python` (with the venv active):

```python
import numpy as np

# An array is a grid of numbers. .shape says how many rows and columns.
poses = np.array([[0.0, 0.0, 0.0],
                  [1.0, 2.0, 0.5],
                  [3.0, -1.0, 1.0]])      # 3 poses: (x, y, θ) per row
print(poses.shape)                        # (3, 3)

# Slicing: [rows, columns]. ':' means "all".
print(poses[:, 0])                        # every x: [0. 1. 3.]
print(poses[1])                           # second pose: [1.  2.  0.5]

# Math works element by element, on whole arrays at once.
print(np.cos(poses[:, 2]))                # cos of every θ

# Broadcasting: arrays of different shapes combine by stretching size-1 dimensions.
xs = poses[:, 0:1]                        # shape (3, 1): one column
pts = np.array([10.0, 20.0, 30.0, 40.0])  # shape (4,): one row of 4
table = xs + pts                          # shape (3, 4): every pose with every point
print(table.shape)                        # (3, 4)
print(table[2])                           # pose 3's x plus each point: [13. 23. 33. 43.]

# Reductions collapse a dimension.
print(table.sum(axis=1))                  # one total per pose: [100. 104. 112.]

# Boolean masks select elements.
print(poses[poses[:, 0] > 0.5])           # poses with x > 0.5 (the last two)
```

The pattern `xs + pts` with shapes `(3, 1)` and `(4,)` giving `(3, 4)` is exactly how the particle filter scores
every particle against every observed point in one line ([chapter 7](07_particle_filter.md)). With 500 particles
and 120 points, that's 60,000 checks per camera frame, done in a few milliseconds.

Two more things you'll see:

- `rng = np.random.default_rng(seed)` makes a random number generator. The same seed always gives the same random
  numbers, which is why every test is repeatable.
- `@` is matrix multiplication: `a @ b`.

## 2.8 Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named 'td_field_model'` | `PYTHONPATH` isn't set, or you're not in `localization/`. | `cd` into `localization/` and set `PYTHONPATH` as in 2.4, or use `run_tests.py`. |
| `ModuleNotFoundError: No module named 'numpy'` | The venv isn't active. | Activate it (2.3). |
| `import file mismatch` / duplicate `test_import` errors | Several packages' tests in one pytest run. | Run one package at a time (2.4). |
| `ruff not found (pip install ruff); skipped` | ruff isn't installed in the active environment. | `pip install -r requirements.txt` with the venv active. |
| `td_localization` seems stuck | It's slow: about 4 minutes. | Wait, or run another package with `run_tests.py <name>`. |
| A `td_bench` real-time test fails once on a busy laptop | One test checks timing against the wall clock. | Run it again with less load; open an issue if it keeps failing. |

---

**Next:** [3. Math toolkit, part 1: positions and angles](03_math_geometry.md)
