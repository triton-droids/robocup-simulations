"""Run every package's tests (each in its own pytest process, as colcon does) and ruff.

    cd localization
    python run_tests.py              # all packages + lint
    python run_tests.py td_game      # some packages
    python run_tests.py --no-lint

Each package runs separately because every package has a `test_import.py`, and pytest refuses duplicate test
module names in one run. For a report with each test beside its result, use test_report/make_results.py.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PATHS = {
    "td_field_model": ["td_field_model"],
    "td_game": ["td_game", "td_game/test"],
    "td_localization": ["td_field_model", "td_game", "td_localization", "td_localization/test"],
    "td_bench": ["td_field_model", "td_game", "td_localization", "td_bench"],
}


def main(argv: list[str]) -> int:
    lint = "--no-lint" not in argv
    pkgs = [a for a in argv if not a.startswith("--")] or list(PATHS)
    failed = []
    for pkg in pkgs:
        env = dict(os.environ, PYTHONPATH=os.pathsep.join(str(HERE / p) for p in PATHS[pkg]))
        print(f"== {pkg}", flush=True)
        # one rootdir per package, as colcon runs them (pytest.ini keeps the repo root config out)
        cmd = [
            sys.executable,
            "-m",
            "pytest",
            "-q",
            "-p",
            "no:cacheprovider",
            f"--rootdir={HERE / pkg}",
            f"{pkg}/test",
        ]
        if subprocess.call(cmd, cwd=HERE, env=env):
            failed.append(pkg)
    if lint:
        ruff = shutil.which("ruff")
        print("== ruff", flush=True)
        if ruff is None:
            print("ruff not found (pip install ruff); skipped")
        elif subprocess.call([ruff, "check", *PATHS, "test_report", "run_tests.py"], cwd=HERE):
            failed.append("ruff")
    print("\nall passed" if not failed else f"\nFAILED: {', '.join(failed)}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
