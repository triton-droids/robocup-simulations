import pytest


def pytest_addoption(parser):
    parser.addoption(
        "--robot",
        default="triton_humanoid",
        help="robot name for tests/test_isaaclab.py (Isaac Sim starts once per process, so "
        "that file runs one robot per invocation)",
    )


@pytest.fixture(autouse=True)
def _run_in_tmp_dir(tmp_path, monkeypatch):
    """ProtoMotions' recorder mkdirs output/renderings under the cwd when a simulator starts.
    Keep that out of the repo root (see CLAUDE.md): every test runs in its own temp dir. The
    tests only use absolute paths (robocup_rl.paths), so nothing else changes."""
    monkeypatch.chdir(tmp_path)


# --- shared by the per-robot simulator-backed tests ---------------------------------------
# torch and protomotions are imported inside the functions on purpose: this module is loaded
# before tests/test_isaaclab.py, which must launch Isaac Sim before torch is imported.


class Backend:
    def __init__(self, name: str, device, num_envs: int):
        self.name = name
        self.device = device
        self.num_envs = num_envs

    def __repr__(self):
        return f"{self.name}:{self.device}:{self.num_envs}"


@pytest.fixture(params=["mujoco", "newton"])
def backend(request) -> Backend:
    import torch

    name = request.param
    if name == "mujoco":
        # Upstream's MuJoCo backend is single-env.
        return Backend(name, torch.device("cpu"), num_envs=1)
    pytest.importorskip("newton")
    if not torch.cuda.is_available():
        pytest.skip("newton (mujoco-warp) needs a CUDA device")
    # More than one env so that ModelBuilder.replicate() and per-env indexing are exercised.
    return Backend(name, torch.device("cuda:0"), num_envs=4)


def build_simulator(backend: Backend, cfg, terrain=None, scene_lib=None):
    from protomotions.components.scene_lib import SceneLib
    from protomotions.components.terrains.config import TerrainConfig
    from protomotions.components.terrains.terrain import Terrain
    from protomotions.simulator.factory import simulator_config
    from protomotions.utils.hydra_replacement import get_class

    n, device = backend.num_envs, backend.device
    sim_cfg = simulator_config(backend.name, cfg, headless=True, num_envs=n, experiment_name="smoke")
    terrain = terrain or Terrain(config=TerrainConfig(), num_envs=n, device=device)
    scene_lib = scene_lib or SceneLib.empty(num_envs=n, device=device)
    sim = get_class(sim_cfg._target_)(
        config=sim_cfg, robot_config=cfg, terrain=terrain, scene_lib=scene_lib, device=device
    )
    return sim, terrain, scene_lib
