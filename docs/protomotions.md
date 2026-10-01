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
- **Keep subtree edits small and local.** Our robot lives inside the subtree (see below),
  but training scripts, experiments, and anything RoboCup-specific that does not have
  to be in ProtoMotions should live outside it. Fewer touched upstream files means fewer
  conflicts at sync time.
- **Merge sync PRs with a merge commit only.** Never squash-merge or rebase-merge them,
  and never rebase `main` across a sync merge. Both destroy the `git-subtree-split`
  trailer that the next sync depends on. The `main` ruleset enforces merge-commit-only.

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
`third-party/protomotions` as an editable path source. IsaacLab is installed by its
own installer from the vendored checkout (below) with ProtoMotions on top.

## IsaacLab subtree

`third-party/IsaacLab` is a squashed subtree of
[isaac-sim/IsaacLab](https://github.com/isaac-sim/IsaacLab) pinned to the commit
ProtoMotions is tested against. ProtoMotions records that pin in two places:
`_ISAACLAB_PIN` in `third-party/protomotions/protomotions/utils/simulator_imports.py`
and the header of `third-party/protomotions/requirements_isaaclab.txt`. Its
`.gitattributes` was removed like ProtoMotions' (the only LFS files are test fixtures).

There is no weekly sync for IsaacLab. Bump it only when a ProtoMotions sync changes
that pin (owners only):

```sh
git fetch https://github.com/isaac-sim/IsaacLab.git <new-sha>
git subtree pull --prefix=third-party/IsaacLab https://github.com/isaac-sim/IsaacLab.git <new-sha> --squash
git rm third-party/IsaacLab/.gitattributes      # if the pull brought it back
```

Then everyone recreates `.venv-isaaclab` per the README.

## Adding our robot

The robot is added inside the subtree, following upstream's
`docs/source/tutorials/workflows/custom_robot.rst`. The files involved, all under
`third-party/protomotions/`:

| File                                             | What goes there                                                                                                   |
| ------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------- |
| `protomotions/data/assets/mjcf/<robot>.xml`      | The MJCF (root `<freejoint/>`, joint limits, collision geoms). URDF must be converted to MJCF first.               |
| `protomotions/data/assets/mesh/<Robot>/*.stl`    | Meshes referenced by the MJCF's `meshdir`.                                                                        |
| `protomotions/robot_configs/<robot>.py`          | `RobotConfig` subclass: body-name mapping, `trackable_bodies_subset`, `default_root_height`, control overrides.    |
| `protomotions/robot_configs/factory.py`          | One `elif robot_name == "<robot>":` branch.                                                                       |
| `protomotions/data/assets/urdf/for_retargeting/` | The URDF used by the PyRoki retargeter.                                                                           |
| `pyroki/batch_retarget_to_<robot>_from_keypoints.py` | Copy of the G1 script with our keypoint-to-link map, scale factors, and URDF path.                            |

Useful upstream helpers: `scripts/identify_robot_facing_axis.py --robots <robot> --view`
for `semantic_forward_axis_xy`, and `examples/random_pose_visualizer.py --robot <robot>`
to check the model loads (it has its own `ROBOT_SPECS` dict to extend).

These are exactly the files most likely to conflict at sync time when upstream touches
them (`factory.py` in particular). Resolve by keeping both sides: upstream's new branches
plus ours.

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
