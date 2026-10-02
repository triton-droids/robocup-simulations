from pathlib import Path

# parents[1] is correct only while this file sits directly in robocup_rl/.
REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOTS_DIR = REPO_ROOT / "robots"
