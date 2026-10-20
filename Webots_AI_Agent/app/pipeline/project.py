"""Create one self-contained Webots project folder per simulation (never overwrites an earlier one)."""
from __future__ import annotations

import json
import re
import shutil
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path

from app.config import Settings
from app.pipeline import worldgen
from app.pipeline.checks import check_project
from app.pipeline.spec import SimSpec, ValidationReport, derived, validate_spec
from app.webots_assets import AssetError

TEMPLATES = Path(__file__).parent / "controller_templates"


class GenerationError(Exception):
    pass


@dataclass
class Generated:
    project: Path
    world: Path
    controller: str
    robot_names: list[str]
    spec: SimSpec
    report: ValidationReport
    spec_summary: dict
    notes: list[str] = field(default_factory=list)


def slug(text: str) -> str:
    s = re.sub(r"[^a-z0-9]+", "-", text.lower()).strip("-")[:32]
    return s or "simulation"


def generate(spec: SimSpec, settings: Settings) -> Generated:
    report = validate_spec(spec)
    if not report.ok:
        raise GenerationError("Specification rejected:\n  - " + "\n  - ".join(report.errors))
    resolver = settings.asset_resolver()
    problems = resolver.installation_problems()
    if settings.webots_home and any("does not exist" in p or "No 'projects'" in p for p in problems):
        raise GenerationError("; ".join(problems))
    try:
        assets = resolver.require(worldgen.needed_assets(spec)) if settings.webots_home else \
            {k: resolver.resolve(k) for k in worldgen.needed_assets(spec)}
    except AssetError as exc:
        raise GenerationError(str(exc)) from exc
    version = resolver.version if settings.webots_home else settings.webots_version

    stamp = time.strftime("%Y%m%d-%H%M%S")
    project = settings.simulations_dir / f"{stamp}_{slug(spec.title)}_{uuid.uuid4().hex[:4]}"
    if project.exists():
        raise GenerationError(f"Project folder already exists (refusing to overwrite): {project}")
    try:
        layout = worldgen.build_layout(spec, project.name)
        world_text, cfg = worldgen.build_world(spec, layout, assets, version)
    except ValueError as exc:
        raise GenerationError(str(exc)) from exc

    ctrl = worldgen.controller_name(spec)
    cdir = project / "controllers" / ctrl
    wdir = project / "worlds"
    cdir.mkdir(parents=True)
    (project / "protos").mkdir()          # reserved for custom PROTO files (none needed: models use core nodes)
    wdir.mkdir()
    for f in (f"{ctrl}.py", "agent_common.py"):
        shutil.copy(TEMPLATES / f, cdir / f)
    cfg["keep_running"] = not settings.keep_webots_open
    (cdir / "config.json").write_text(json.dumps(cfg, indent=2), encoding="utf-8")
    if settings.webots_python_command:
        (cdir / "runtime.ini").write_text(f"[python]\nCOMMAND = {settings.webots_python_command}\n", encoding="utf-8")
    world = wdir / "world.wbt"
    world.write_text(world_text, encoding="utf-8")
    summary = {"spec": json.loads(spec.model_dump_json()), "derived": derived(spec), "layout_notes": layout.notes,
               "warnings": report.warnings, "webots_version": version,
               "assets": {k: a.url for k, a in assets.items()}, "controller": ctrl, "robots": cfg["robot_names"]}
    (project / "spec.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    bad = check_project(project, world, ctrl, resolver, ["OBJECT", "ARM_TIP"] if spec.robot.kind == "arm" else [],
                        cfg["robot_names"])
    if bad:
        raise GenerationError("Generated project failed validation (not launched):\n  - " + "\n  - ".join(bad)
                              + f"\nProject kept for inspection: {project}")
    return Generated(project, world, ctrl, cfg["robot_names"], spec, report, summary, layout.notes)
