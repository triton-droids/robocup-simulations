import pytest


@pytest.fixture(autouse=True)
def _run_in_tmp_dir(tmp_path, monkeypatch):
    """ProtoMotions' recorder mkdirs output/renderings under the cwd when a simulator starts.
    Keep that out of the repo root (see CLAUDE.md): every test runs in its own temp dir. The
    tests only use absolute paths (robocup_rl.paths), so nothing else changes."""
    monkeypatch.chdir(tmp_path)
