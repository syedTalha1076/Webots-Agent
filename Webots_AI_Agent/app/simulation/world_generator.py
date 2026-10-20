"""Builds Webots projects (world + controller + config) from a validated RobotSpec.

No arbitrary code is generated: only parameters are substituted into fixed templates, and every number
has been validated by `schemas.RobotSpec` first.
"""
from __future__ import annotations

import json
import math
import random
import shutil
from pathlib import Path

from app.config import Settings
from app.schemas import WALL_MARGIN, RobotSpec
from app.simulation.registry import ROBOT_MODELS
from app.simulation.navigation import NavParams, plan_path

OBSTACLE_SIZE = (0.08, 0.14)   # box edge length range in metres (e-puck is ~7.4 cm wide)


def layout_obstacles(spec: RobotSpec, seed: str | int) -> list[dict]:
    """Explicit obstacle positions first, remaining ones random but never on top of start/target/each other."""
    rng = random.Random(str(seed))
    w, h = spec.world.width, spec.world.height
    out = [{"x": p[0], "y": p[1], "size": 0.1} for p in spec.world.obstacle_positions]
    keep_out = [(spec.start_position, 0.3), (spec.target_position, 0.3)]
    attempts = 0
    while len(out) < spec.world.obstacles:
        attempts += 1
        if attempts > 4000:
            raise ValueError("The room is too small for that many obstacles; reduce the obstacle count.")
        x = rng.uniform(-w / 2 + 0.25, w / 2 - 0.25)
        y = rng.uniform(-h / 2 + 0.25, h / 2 - 0.25)
        if any(math.dist((x, y), c) < r for c, r in keep_out):
            continue
        if any(math.dist((x, y), (o["x"], o["y"])) < 0.22 for o in out):
            continue
        out.append({"x": round(x, 3), "y": round(y, 3), "size": round(rng.uniform(*OBSTACLE_SIZE), 3)})
    return out


def build_plan_waypoints(spec: RobotSpec, obstacles: list[dict]) -> tuple[list[tuple[float, float]], str]:
    goal = tuple(spec.target_position)
    if spec.planning_mode == "waypoints":
        path = plan_path(tuple(spec.start_position), goal, obstacles, spec.world.width, spec.world.height)
        if path:
            return [tuple(p) for p in path], f"A* waypoint path ({len(path)} waypoints) + reactive IR avoidance"
        return [goal], "no collision-free path on the known map; falling back to reactive navigation"
    return [goal], "reactive go-to-goal with IR obstacle avoidance"


def resolve_assets(settings: Settings) -> tuple[str, str]:
    """Return (EXTERNPROTO block, #VRML_SIM version). Every PROTO file is verified on disk when WEBOTS_HOME is set;
    a missing file raises AssetError (we never emit a reference Webots would have to skip)."""
    res = settings.asset_resolver()
    keys = ["TexturedBackground", "TexturedBackgroundLight", "RectangleArena", "WoodenBox", "E-puck"]
    assets = res.require(keys) if settings.webots_home else {k: res.resolve(k) for k in keys}
    version = res.version if settings.webots_home else settings.webots_version
    return "\n".join(a.externproto_line for a in assets.values()), version


from app.pipeline.worldgen import look_at  # noqa: E402


def _world_text(spec: RobotSpec, obstacles: list[dict], controller_arg: str, version: str,
                externproto: str | None = None) -> str:
    if externproto is None:   # legacy callers/tests: unverified default references
        from app.webots_assets import AssetResolver
        r = AssetResolver(None, version)
        externproto = "\n".join(r.resolve(k).externproto_line for k in
                                ["TexturedBackground", "TexturedBackgroundLight", "RectangleArena", "WoodenBox", "E-puck"])
    m = ROBOT_MODELS[spec.robot_type.value]
    w, h = spec.world.width, spec.world.height
    sx, sy = spec.start_position
    tx, ty = spec.target_position
    cam = max(w, h) * 1.1 + 0.8
    boxes = "\n".join(
        f'WoodenBox {{ translation {o["x"]} {o["y"]} {o["size"] / 2:.4f} name "obstacle_{i}" '
        f'size {o["size"]} {o["size"]} {o["size"]} mass 5 }}'
        for i, o in enumerate(obstacles)
    )
    return f'''#VRML_SIM {version} utf8

{externproto}

WorldInfo {{
  title "Text-to-Robot session"
  basicTimeStep 32
  coordinateSystem "ENU"
}}
Viewpoint {{
  orientation {look_at((0.0, -cam * 0.64, cam * 0.77), (0.0, 0.0, 0.0))}
  position 0 {-cam * 0.64:.3f} {cam * 0.77:.3f}
}}
TexturedBackground {{
}}
TexturedBackgroundLight {{
}}
RectangleArena {{
  floorSize {w} {h}
  wallHeight 0.1
}}
DEF TARGET Solid {{
  translation {tx} {ty} 0.001
  name "target"
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor 1 0 0 roughness 1 metalness 0 }}
      geometry Cylinder {{ height 0.002 radius 0.05 }}
    }}
  ]
}}
{boxes}
DEF ROBOT {m.webots_proto} {{
  translation {sx} {sy} 0
  rotation 0 0 1 {spec.start_heading:.5f}
  name "{m.key}"
  controller "{m.controller}"
  controllerArgs [ "{controller_arg}" ]
  supervisor TRUE
}}
'''


def generate_project(spec: RobotSpec, session_id: str, settings: Settings, token: str,
                     out_dir: Path | None = None, standalone_arg: str | None = None,
                     timeout_s: int | None = None) -> dict:
    """Create <simulations>/<session_id>/ with world, controller copy and config. Never overwrites other dirs."""
    root = settings.simulations_dir
    out_dir = out_dir or (root / session_id)
    if out_dir.exists() and any(out_dir.iterdir()):
        raise FileExistsError(f"Session directory already exists: {out_dir}")
    model = ROBOT_MODELS[spec.robot_type.value]
    obstacles = layout_obstacles(spec, session_id)
    waypoints, nav_desc = build_plan_waypoints(spec, obstacles)

    ctrl_dir = out_dir / "controllers" / model.controller
    world_dir = out_dir / "worlds"
    ctrl_dir.mkdir(parents=True, exist_ok=True)
    world_dir.mkdir(parents=True, exist_ok=True)
    src = settings.controller_src_dir
    shutil.copy(src / f"{model.controller}.py", ctrl_dir / f"{model.controller}.py")
    shutil.copy(Path(__file__).with_name("navigation.py"), ctrl_dir / "navigation.py")
    if settings.webots_python_command:
        (ctrl_dir / "runtime.ini").write_text(f"[python]\nCOMMAND = {settings.webots_python_command}\n", encoding="utf-8")

    host = "127.0.0.1" if settings.host in ("0.0.0.0", "::") else settings.host
    config = {
        "session_id": session_id,
        "token": token,
        "backend_url": "" if standalone_arg else f"http://{host}:{settings.port}",
        "spec": json.loads(spec.model_dump_json()),
        "robot": {"wheel_radius": model.wheel_radius, "axle_length": model.axle_length,
                  "max_wheel_speed": model.max_wheel_speed, "robot_radius": model.robot_radius},
        "obstacles": obstacles,
        "waypoints": waypoints,
        "nav": NavParams(max_speed=spec.max_speed).__dict__,
        "timeout_s": timeout_s or settings.simulation_timeout,
    }
    config_path = out_dir / "config.json"
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    world_path = world_dir / "world.wbt"
    arg = standalone_arg or config_path.resolve().as_posix()
    externproto, version = resolve_assets(settings)
    world_path.write_text(_world_text(spec, obstacles, arg, version, externproto), encoding="utf-8")
    return {"dir": out_dir, "world": world_path, "config": config_path, "obstacles": obstacles,
            "waypoints": waypoints, "navigation": nav_desc}


def cleanup_session_dir(settings: Settings, session_id: str) -> bool:
    """Delete a session directory, but only if it lives inside the simulations directory."""
    target = (settings.simulations_dir / session_id).resolve()
    if target.parent != settings.simulations_dir or not target.exists():
        return False
    shutil.rmtree(target, ignore_errors=True)
    return True
