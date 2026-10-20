"""Regenerates worlds/sample_navigation.wbt (standalone: no backend needed, prints to the Webots console).

Usage (from the project root, venv active):  python scripts/make_sample_world.py
"""
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from app.config import Settings  # noqa: E402
from app.schemas import RobotSpec, WorldSpec  # noqa: E402
from app.simulation.world_generator import generate_project  # noqa: E402

spec = RobotSpec(world=WorldSpec(width=2.0, height=2.0, obstacles=0,
                                 obstacle_positions=[(-0.2, -0.1), (0.15, 0.25), (0.45, 0.0), (-0.45, 0.45)]),
                 start_position=(-0.7, -0.7), target_position=(0.7, 0.7), max_speed=4.0)
settings = Settings(_env_file=None)
with tempfile.TemporaryDirectory() as tmp:
    out = generate_project(spec, "sample000000", settings, "standalone", out_dir=Path(tmp) / "s",
                           standalone_arg="../../worlds/sample_config.json")
    shutil.copy(out["world"], ROOT / "worlds" / "sample_navigation.wbt")
    shutil.copy(out["config"], ROOT / "worlds" / "sample_config.json")
print("wrote worlds/sample_navigation.wbt and worlds/sample_config.json")
