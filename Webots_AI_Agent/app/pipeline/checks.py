"""Pre-launch validation of a generated project (nothing is launched if this fails)."""
from __future__ import annotations

import json
import py_compile
import re
from pathlib import Path

from app.webots_assets import AssetResolver

BUILTIN_NODES = {"WorldInfo", "Viewpoint", "Solid", "Robot", "Shape", "Transform", "HingeJoint", "BallJoint",
                 "PBRAppearance", "Box", "Cylinder", "Sphere", "DistanceSensor", "Camera", "RotationalMotor",
                 "PositionSensor", "Physics", "HingeJointParameters", "BallJointParameters", "DEF"}


def check_project(project: Path, world: Path, controller: str, resolver: AssetResolver, expected_defs: list[str],
                  robot_names: list[str]) -> list[str]:
    """Return a list of problems (empty = OK)."""
    problems: list[str] = []
    text = world.read_text(encoding="utf-8")
    if not text.startswith("#VRML_SIM "):
        problems.append("World file does not start with a '#VRML_SIM <version> utf8' header.")
    else:
        hdr = text.split()[1]
        if resolver.home and resolver.home.is_dir() and hdr != resolver.version:
            problems.append(f"World header {hdr} differs from installed Webots {resolver.version}.")
    # balanced braces / brackets outside strings
    stripped = re.sub(r'"[^"]*"', '""', text)
    for o, c in (("{", "}"), ("[", "]")):
        if stripped.count(o) != stripped.count(c):
            problems.append(f"Unbalanced '{o}{c}' in world file ({stripped.count(o)} vs {stripped.count(c)}).")
    # every EXTERNPROTO exists (verified path) and every PROTO-looking node is declared
    declared = {}
    for m in re.finditer(r'^EXTERNPROTO\s+"([^"]+)"', text, re.M):
        url = m.group(1)
        node = Path(url).stem
        declared[node] = url
        if re.match(r"^[A-Za-z]:[\\/]|\\", url):
            problems.append(f"EXTERNPROTO uses an absolute Windows path: {url}")
        local = None
        if url.startswith("webots://projects/") and resolver.projects_dir:
            local = resolver.projects_dir / url[len("webots://projects/"):]
        elif url.startswith("https://raw.githubusercontent.com/cyberbotics/webots/"):
            local = None          # online reference: Webots downloads it; existence was verified at generation time
        elif url.startswith("file:") or re.match(r"^[A-Za-z]:/", url):
            local = Path(url.replace("file:///", ""))
        if local is not None and not local.is_file():
            problems.append(f"PROTO file not found on disk: {local}")
    body = text.split("\n\n", 2)[-1]
    for node in set(re.findall(r"^\s*(?:DEF\s+\w+\s+)?([A-Z][A-Za-z0-9\-]+)\s*\{", body, re.M)):
        if node not in BUILTIN_NODES and node not in declared and node not in {"TexturedBackground", "TexturedBackgroundLight"}:
            problems.append(f"Node '{node}' is used but has no EXTERNPROTO declaration.")
    for node in ("TexturedBackground", "TexturedBackgroundLight"):
        if re.search(rf"^{node}\s*\{{", body, re.M) and node not in declared:
            problems.append(f"Node '{node}' is used but has no EXTERNPROTO declaration.")
    for d in expected_defs:
        if not re.search(rf"\bDEF\s+{d}\b", text):
            problems.append(f"Controller expects DEF {d} but the world does not define it.")
    for nm in robot_names:
        if f'name "{nm}"' not in text:
            problems.append(f"Robot name '{nm}' (used in controller config) missing from the world.")
    if text.count(f'controller "{controller}"') < len(robot_names):
        problems.append(f"Not every robot references controller \"{controller}\".")
    # controller folder / files
    cdir = project / "controllers" / controller
    py = cdir / f"{controller}.py"
    if not py.is_file():
        problems.append(f"Controller file missing: {py}")
    else:
        for f in cdir.glob("*.py"):
            try:
                py_compile.compile(str(f), doraise=True, cfile=str(project / ".pyc_check"))
            except py_compile.PyCompileError as exc:
                problems.append(f"Python syntax error in {f.name}: {exc.msg}")
        (project / ".pyc_check").unlink(missing_ok=True)
    cfgp = cdir / "config.json"
    if not cfgp.is_file():
        problems.append("config.json missing next to the controller.")
    else:
        try:
            json.loads(cfgp.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            problems.append(f"config.json is not valid JSON: {exc}")
    if not world.parent.name == "worlds" or world.parent.parent != project:
        problems.append("World must live in <project>/worlds/ so Webots finds <project>/controllers/.")
    return problems
