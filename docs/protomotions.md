# Working with the vendored ProtoMotions

[NVIDIA ProtoMotions](https://github.com/NVlabs/ProtoMotions) (Apache-2.0) lives in
`third-party/protomotions` as a **squashed git subtree**. The repo pins one exact
upstream commit. To see which one:

```sh
git log -1 --grep='^git-subtree-dir: third-party/protomotions/*$' \
  --format='%(trailers:key=git-subtree-split,valueonly)'
```

## Rules

- **Edits under `third-party/protomotions` need an owner.** Owners are listed in
  `.github/CODEOWNERS` and may commit there directly. Everyone else opens a PR, which
  needs an owner's approval to merge. Ask an owner before starting non-trivial work in
  the subtree so it doesn't collide with the weekly sync. If a change is a genuine
  ProtoMotions fix rather than RoboCup-specific, send it upstream to NVlabs and let the
  sync bring it in.
- **Keep subtree edits small and local.** Our robot, its config, experiments, and tests
  live outside the subtree (`robots/`, `robocup_rl/`, `tests/`). The local changes inside it
  are listed under "Local changes" below: the robot-registration hook in
  `robot_configs/factory.py` and the Newton effort-limit pass-through. Fewer touched upstream
  files means fewer conflicts at sync time.
- **Merge sync PRs with a merge commit only.** Never squash-merge or rebase-merge them,
  and never rebase `main` across a sync merge. Both destroy the `git-subtree-split`
  trailer that the next sync depends on. The `main` ruleset enforces merge-commit-only.

## Local changes

Both are marked with `robocup-simulations` comments and are candidate upstream PRs.

| File | Change | Why |
| --- | --- | --- |
| `protomotions/robot_configs/factory.py` | `else` branch resolves unknown robot names through the `protomotions.robots` entry-point group | lets our robots live outside the subtree |
| `protomotions/simulator/newton/simulator.py` | `_setup_sim` copies the `ControlInfo` effort limits into MuJoCo Warp's `actuator_forcerange` / `actuator_forcelimited` for `BUILT_IN_PD` | Newton 1.0's `SolverMuJoCo` creates the PD actuators without a force range, so on Newton the effort limits were silently ignored while the MuJoCo backend enforces them; unbounded PD torque drove triton_humanoid's joints thousands of radians past their limits and into a non-finite state |

If a sync conflicts on either file, take upstream's version and re-apply the few lines
(`git log -p` on the file shows them). `protomotions-train-agent` itself needs a GPU
(`FabricConfig` hardcodes `accelerator="gpu"`); the MuJoCo env is for inspecting and
evaluating, not training.

## Binary assets

Upstream keeps its checkpoints, meshes, and media in Git LFS. We do not use LFS: those
files are committed here as the ~130-byte pointer stubs they are in git, and upstream's
`.gitattributes` was deliberately removed from the subtree so git never tries to resolve
them. Our custom robot needs none of them (MuJoCo generates its own ground texture; the
G1/H1_2 checkpoints and SMPL/SOMA assets are for other robots). If an upstream example
ever needs one, download it from the upstream repo and keep it outside the subtree.

## Install

See the **Environments** section of the top-level [README](../README.md). MuJoCo and
Newton are `uv sync` extras of the root `pyproject.toml`, which depends on
`third-party/protomotions` as an editable path source. IsaacLab syncs from its own
vendored uv project (below) with ProtoMotions installed on top.

## IsaacLab subtree

`third-party/IsaacLab` is a squashed subtree of
[isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) pinned to the commit
ProtoMotions is tested against. ProtoMotions records that pin in two places:
`_ISAACLAB_PIN` in `third-party/protomotions/protomotions/utils/simulator_imports.py`
and the header of `third-party/protomotions/requirements_isaaclab.txt`. Its
`.gitattributes` was removed like ProtoMotions' (the only LFS files are test fixtures).

**Local changes to the vendored tree**, all in `third-party/IsaacLab/pyproject.toml` and
marked with `robocup-simulations:` comments:

- Four platform markers rewritten from substring form (`platform_machine in 'x86_64 AMD64'`)
  to equality form. uv cannot prove substring markers disjoint, so upstream's form makes
  `uv lock` fail with a phantom x86/ARM conflict on `pytetwild`.
- The `ov` extra emptied. It pinned an `ovphysx` build that exists on no public index,
  and `uv lock` resolves every extra. We do not use the Omniverse PhysX backend.
- `uv.lock` committed, generated from the patched project. This is what pins everyone's
  `.venv-isaaclab`.

There is no weekly sync for IsaacLab, and no bump is planned. If one is ever needed
(owners only), expect the pyproject edits above to conflict and reapply them:

```sh
git fetch https://github.com/isaac-sim/IsaacLab.git <new-sha>
git subtree pull --prefix=third-party/IsaacLab https://github.com/isaac-sim/IsaacLab.git <new-sha> --squash
git rm third-party/IsaacLab/.gitattributes      # if the pull brought it back
uv lock --project third-party/IsaacLab           # after re-applying the marker/ov patch
```

Then everyone recreates `.venv-isaaclab` per the README.

## Adding our robot

Upstream's `docs/source/tutorials/workflows/custom_robot.rst` puts a new robot inside the
package and adds an `elif` to `protomotions/robot_configs/factory.py`. We keep everything
outside the subtree instead and register the robot through a Python entry point:

| Where                                                          | What                                                                                                   |
| -------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ |
| `robots/<robot>/`                                              | MJCF and meshes. ProtoMotions needs one actuator per joint and a `<worldbody>` in the file it loads.    |
| `robocup_rl/robots/<robot>.py`                                 | `RobotConfig` subclass: absolute `asset_root`, body-name mapping, PD gains, default pose, sim params.   |
| `pyproject.toml` (root)                                        | `[project.entry-points."protomotions.robots"] <robot> = "robocup_rl.robots.<robot>:<Config>"`.         |
| `third-party/protomotions/protomotions/robot_configs/factory.py` | **Local subtree change (owners only):** the `else` branch calls `_robot_config_from_entry_point`, which looks the name up in that entry-point group. Nothing RoboCup-specific is in the file. |

`uv sync` (or `uv pip install -e .`) writes the entry point; `--robot-name <robot>` then
works in `protomotions-train-agent`, inference, and every upstream helper script, because
they all go through `robot_config()`. Adding another robot is a config file plus one
pyproject line; the subtree is not touched again.

If a sync conflicts on `factory.py`, take upstream's version and re-apply the hook: the
`else` branch becomes `config = _robot_config_from_entry_point(robot_name)` and the helper
is appended at the end of the file (it is ~14 generic lines; `git log -p` on the file shows
them). The hook is a candidate upstream PR; if NVlabs adopts it, the local diff disappears.

Useful upstream helpers: `scripts/identify_robot_facing_axis.py --robots <robot>` for
`semantic_forward_axis_xy`, and `examples/random_pose_visualizer.py` to check the model
loads (its `--robot` choices and `ROBOT_SPECS` dict are hard-coded; copy it if needed).

Not done yet for `triton_humanoid`: a URDF and a legs-only retargeting script for the
PyRoki retargeter (`pyroki/batch_retarget_to_g1_from_keypoints.py` is G1-specific).

## Syncing with upstream

`.github/workflows/sync-protomotions.yml` runs every Monday (or on demand from the
Actions tab) and opens or updates a PR on branch `chore/sync-protomotions`. An owner
reviews and merges it with a merge commit. PRs opened by the workflow do not trigger
other workflows; if CI ever becomes a required check, switch the job to a GitHub App token.

To sync by hand, or when the weekly PR hits a conflict (owners only):

```sh
git switch main && git pull --ff-only
git remote add protomotions-upstream https://github.com/NVlabs/ProtoMotions.git 2>/dev/null
git fetch protomotions-upstream main
git subtree pull --prefix=third-party/protomotions protomotions-upstream main --squash \
  -m "chore(protomotions): sync to NVlabs/ProtoMotions@$(git rev-parse --short protomotions-upstream/main)"

# only if the pull stopped on conflicts:
git status --short | grep -E '^(UU|AA|DU|UD|AU|UA)'
git checkout --theirs -- <path>        # take upstream's version
git rm -- <path>                       # upstream deleted it, or it is .gitattributes (keep it deleted)
git add third-party/protomotions && git commit --no-edit

git push origin main                   # or push a branch and open a PR
```

Never `git merge --abort` and retry without `--squash`; always pull with `--squash`.
