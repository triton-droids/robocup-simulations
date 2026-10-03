"""IsaacLab smoke test for one registered robot. GPU only; run alone, in .venv-isaaclab:

    pytest -v tests/test_isaaclab.py                      # triton_humanoid
    pytest -v tests/test_isaaclab.py --robot booster_k1

It is a separate file, not a parameter of the per-robot test modules, because Isaac Sim's
AppLauncher has to start before torch is imported anywhere in the process and stays up for the
rest of it, and closing the simulator shuts Kit down. So: one process, one robot, chosen with
the --robot option (tests/conftest.py), resolved through the entry point like the CLI does.
Isaac Sim converts the MJCF to USD on first use (cached under ~/.cache/protomotions/).

Read the per-test PASSED/FAILED lines in the log: Kit shuts the process down at exit before
pytest prints its summary, and the exit code is 0 either way (seen with Isaac Sim 6.0.0.1).
"""

import os

import pytest

pytest.importorskip("isaaclab.app")
if not os.path.exists("/dev/nvidia0") and not os.environ.get("CUDA_VISIBLE_DEVICES"):
    pytest.skip("Isaac Sim needs an NVIDIA GPU", allow_module_level=True)

os.environ.setdefault("OMNI_KIT_ACCEPT_EULA", "yes")

from isaaclab.app import AppLauncher  # noqa: E402

# Kit must be launched before torch (see protomotions/utils/simulator_imports.py).
_app_launcher = AppLauncher({"headless": True, "device": "cuda:0"})

import torch  # noqa: E402

from protomotions.robot_configs.factory import robot_config  # noqa: E402

NUM_ENVS = 4


@pytest.fixture(scope="module")
def simulator(request):
    from protomotions.components.scene_lib import SceneLib
    from protomotions.components.terrains.config import TerrainConfig
    from protomotions.components.terrains.terrain import Terrain
    from protomotions.simulator.factory import simulator_config
    from protomotions.utils.hydra_replacement import get_class

    device = torch.device("cuda:0")
    cfg = robot_config(request.config.getoption("--robot"))
    sim_cfg = simulator_config(
        "isaaclab", cfg, headless=True, num_envs=NUM_ENVS, experiment_name="smoke"
    )
    sim = get_class(sim_cfg._target_)(
        config=sim_cfg,
        robot_config=cfg,
        terrain=Terrain(config=TerrainConfig(), num_envs=NUM_ENVS, device=device),
        device=device,
        scene_lib=SceneLib.empty(num_envs=NUM_ENVS, device=device),
        simulation_app=_app_launcher.app,
    )
    sim._initialize_with_markers({})
    yield sim, cfg
    sim.close()


def test_isaaclab_simulator_steps_without_falling_through_the_floor(simulator):
    sim, cfg = simulator
    # PhysX tensor views want index tensors on the simulation device.
    sim.reset_envs(sim.get_default_robot_reset_state(), env_ids=torch.arange(NUM_ENVS, device=sim.device))
    for _ in range(20):
        sim.step(torch.zeros(NUM_ENVS, cfg.number_of_actions, device=sim.device))
    root_z = sim.get_root_state().root_pos[:, 2].cpu()
    assert torch.isfinite(root_z).all()
    assert (root_z > 0.3).all(), f"root heights {root_z.tolist()}: did the USD conversion drop the floor?"


def test_isaaclab_dof_order_matches_config(simulator):
    sim, cfg = simulator
    dof_pos = sim.get_dof_state().dof_pos
    assert dof_pos.shape == (NUM_ENVS, cfg.number_of_actions)
    assert torch.isfinite(dof_pos).all()
