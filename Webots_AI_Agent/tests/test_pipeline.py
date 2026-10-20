"""Tests for the prompt-to-simulation pipeline. Webots itself is NOT required (see tests/mock_webots.py)."""
import json
import math
import re
import time
from pathlib import Path

import pytest

from tests import mock_webots as mw
from app.config import Settings
from app.pipeline import worldgen
from app.pipeline.parser import parse_rules
from app.pipeline.project import GenerationError, generate
from app.pipeline.runner import run_world, scan_log
from app.pipeline.service import run_pipeline
from app.pipeline.spec import ARM, SimSpec, validate_spec
from app.webots_assets import CATALOG, AssetError, AssetResolver

EXAMPLES = {
    "epuck_nav": "Wheeled robot in a 3 x 3 m room with 5 obstacles, target at (1.0, 1.0)",
    "four_wheel": "A four wheel robot 40 cm long and 25 cm wide goes to (2.0, 1.0) in a 5 x 5 m room with 6 obstacles",
    "line": "A custom robot follows a black line track for 1 lap",
    "wander": "two e-puck robots wander and avoid obstacles in a 4 m room for 40 seconds",
    "warehouse": "Warehouse 10 x 8 m with 4 shelves and 3 robots delivering boxes from (-3,-2) to (3, 2)",
    "arm": "Robotic arm picks a cube at (0.3, 0.1) and places it at (0.2, -0.3)",
    "drone": "Drone flies to altitude of 1.5 meters and flies waypoints (2,0) (2,2) (0,2)",
}


def fake_install(root: Path, version="R2025a", skip=()):
    for k, (_, rel) in CATALOG.items():
        if k in skip:
            continue
        f = root / "projects" / rel
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text(f"#VRML_SIM {version} utf8\n")
    (root / "resources").mkdir(parents=True, exist_ok=True)
    (root / "resources" / "version.txt").write_text(version)
    return root


@pytest.fixture
def settings(tmp_path):
    home = fake_install(tmp_path / "Webots")
    return Settings(_env_file=None, webots_home=str(home), webots_project_dir=str(tmp_path / "projects"),
                    llm_provider="rules")


# ------------------------------------------------------------------------------------------ assets
def test_asset_urls_are_not_absolute_windows_paths(settings):
    res = settings.asset_resolver()
    assert res.version == "R2025a"
    for k in CATALOG:
        a = res.resolve(k)
        assert a.verified and a.url.startswith("webots://projects/")
        assert ":" not in a.url.split("//", 1)[1] and "\\" not in a.url


def test_missing_asset_raises_with_exact_path(tmp_path, monkeypatch):
    monkeypatch.setattr(AssetResolver, "online_exists", staticmethod(lambda u: False))
    home = fake_install(tmp_path / "W", skip=("E-puck",))
    with pytest.raises(AssetError) as e:
        AssetResolver(home).require(["E-puck", "RectangleArena"])
    assert "E-puck" in str(e.value) and "E-puck.proto" in str(e.value)


def test_github_mode_pins_detected_version(tmp_path):
    home = fake_install(tmp_path / "W", version="R2023b")
    a = AssetResolver(home, "R2025a", "github").resolve("WoodenBox")
    assert "/R2023b/projects/objects/factory/containers/protos/WoodenBox.proto" in a.url


# ------------------------------------------------------------------------------------------ parsing / validation
def test_parser_extracts_fields():
    s = parse_rules(EXAMPLES["four_wheel"]).spec
    assert (s.robot.kind, s.robot.drive, s.robot.body_length, s.robot.body_width) == ("custom_wheeled", "four_wheel", 0.4, 0.25)
    assert (s.environment.width, s.environment.obstacles) == (5.0, 6) and s.behavior.targets == [(2.0, 1.0)]
    w = parse_rules(EXAMPLES["warehouse"]).spec
    assert w.environment.kind == "warehouse" and w.robot.count == 3 and w.environment.shelves == 4


@pytest.mark.parametrize("text", ["build a humanoid that walks", "make a snake robot", "underwater submarine"])
def test_unsupported_is_explained_not_silently_generated(text):
    out = parse_rules(text)
    assert out.spec is None and "not supported" in out.message


def test_validation_rejects_bad_specs():
    far = parse_rules("e-puck in a 2 x 2 m room, target at (5, 5)").spec
    assert any("outside the room" in e for e in validate_spec(far).errors)
    arm = parse_rules("robot arm picks cube at (1.5, 1.5) places at (0.2, 0.2)").spec
    assert any("reaches" in e for e in validate_spec(arm).errors)
    s = SimSpec(robot={"kind": "arm"}, environment={"kind": "arena"}, behavior={"task": "pick_and_place"})
    assert validate_spec(s).errors
    s = SimSpec(robot={"kind": "drone", "count": 1}, behavior={"task": "line_follow"}, environment={"kind": "open_field"})
    assert validate_spec(s).errors


# ------------------------------------------------------------------------------------------ generation
@pytest.mark.parametrize("key", list(EXAMPLES))
def test_generate_valid_project(settings, key):
    spec = parse_rules(EXAMPLES[key]).spec
    g = generate(spec, settings)
    text = g.world.read_text()
    assert text.startswith("#VRML_SIM R2025a utf8")
    assert not re.search(r'EXTERNPROTO\s+"[A-Za-z]:|\\\\', text)
    assert g.project.parent == settings.simulations_dir
    cdir = g.project / "controllers" / g.controller
    assert (cdir / f"{g.controller}.py").is_file() and (cdir / "config.json").is_file()
    assert text.count(f'controller "{g.controller}"') == len(g.robot_names)
    # robots in the world == robots in controller config
    cfg = json.loads((cdir / "config.json").read_text())
    assert set(cfg["robots"]) == set(g.robot_names)


def test_every_simulation_gets_its_own_folder(settings):
    spec = parse_rules(EXAMPLES["epuck_nav"]).spec
    a, b = generate(spec, settings), generate(spec, settings)
    assert a.project != b.project and a.world.exists() and b.world.exists()


def test_generation_fails_cleanly_when_proto_missing(tmp_path, monkeypatch):
    monkeypatch.setattr(AssetResolver, "online_exists", staticmethod(lambda u: False))
    home = fake_install(tmp_path / "W", skip=("Mavic2Pro",))
    s = Settings(_env_file=None, webots_home=str(home), webots_project_dir=str(tmp_path / "p"))
    with pytest.raises(GenerationError) as e:
        generate(parse_rules(EXAMPLES["drone"]).spec, s)
    assert "Mavic2Pro" in str(e.value)
    assert not (tmp_path / "p" / "simulations").exists() or not list((tmp_path / "p" / "simulations").iterdir())


def test_validation_error_stops_generation(settings):
    spec = parse_rules("e-puck in a 2 x 2 m room, target at (5, 5)").spec
    with pytest.raises(GenerationError) as e:
        generate(spec, settings)
    assert "rejected" in str(e.value)


def test_warehouse_layout_keeps_clear_of_starts_and_targets(settings):
    spec = parse_rules(EXAMPLES["warehouse"]).spec
    L = worldgen.build_layout(spec, "t")
    for o in L.obstacles:
        for sx, sy, _ in L.starts:
            assert not (abs(o["x"] - sx) < o["sx"] / 2 + 0.2 and abs(o["y"] - sy) < o["sy"] / 2 + 0.2)
        for ts in L.targets:
            for tx, ty in ts:
                assert not (abs(o["x"] - tx) < o["sx"] / 2 + 0.2 and abs(o["y"] - ty) < o["sy"] / 2 + 0.2)
    assert len(L.targets) == 3 and all(len(t) == 2 for t in L.targets)


# ------------------------------------------------------------------------------------------ controllers (mock)
def _wheeled(g, robot_idx=0):
    cdir = g.project / "controllers" / g.controller
    cfg = json.loads((cdir / "config.json").read_text())
    nm = cfg["robot_names"][robot_idx]
    if g.spec.robot.kind == "epuck":
        defs = {f"ps{i}": (a, 0.07) for i, a in enumerate([-0.3, -0.8, -1.57, 0, 0, 1.57, 0.8, 0.3])}
        wr, ax = 0.0205, 0.052
    else:
        defs = {"ds_left_side": (1.2, 0.5), "ds_left": (0.6, 0.5), "ds_front": (0, 0.5), "ds_right": (-0.6, 0.5), "ds_right_side": (-1.2, 0.5)}
        wr, ax = g.spec.robot.wheel_radius, g.spec.robot.body_width + 0.025
    line = worldgen.make_track(g.spec.environment.track_shape, *cfg["room"]) if cfg["task"] == "line_follow" else None
    sim = mw.WheeledSim(cfg, nm, cfg["robots"][nm]["start"], wr, ax, defs, cfg["obstacles"], cfg["room"], line=line,
                        t_limit=cfg["timeout_s"] + 5)
    mw.run_controller_file(cdir, sim)
    return json.loads((g.project / f"result_{nm}.json").read_text()), sim


def _fixed_layout(monkeypatch):
    """Deterministic obstacle layout so the mock tests are reproducible (random layouts are covered in my manual runs)."""
    orig = worldgen.build_layout
    monkeypatch.setattr(worldgen, "build_layout", lambda spec, seed: orig(spec, "fixed-seed-1"))


@pytest.mark.parametrize("key", ["epuck_nav", "four_wheel", "line", "wander"])
def test_wheeled_controllers_complete_task_in_mock(settings, key, monkeypatch):
    _fixed_layout(monkeypatch)
    g = generate(parse_rules(EXAMPLES[key]).spec, settings)
    res, sim = _wheeled(g)
    assert res["success"], res
    if key != "line":
        assert res["metrics"]["collisions"] <= 2     # random layouts: occasional graze in the 2D mock


def test_warehouse_every_robot_reaches_its_targets_in_mock(settings, monkeypatch):
    _fixed_layout(monkeypatch)
    g = generate(parse_rules(EXAMPLES["warehouse"]).spec, settings)
    for i in range(len(g.robot_names)):
        res, _ = _wheeled(g, i)
        assert res["success"], (i, res)


def test_ik_matches_independent_forward_kinematics():
    """IK result is checked with explicit rotation matrices (hinge axis +y: positive angle swings the link DOWN)."""
    import sys
    sys.path.insert(0, str(Path(worldgen.__file__).parent / "controller_templates"))
    import importlib
    cfg = {"arm": {"l1": ARM["l1"], "l2": ARM["l2"], "h0": ARM["h0"], "limits": {"shoulder": [-1.6, 1.0], "elbow": [0, 2.8]},
                   "base": [0, 0, ARM["table_h"]], "pick": [0.3, 0.1], "place": [0.2, -0.3], "cube_z": 0.425}, "timeout_s": 60}
    tmp = Path(__file__).parent / "_tmp_cfg"
    # load module with a fake config.json
    src = (Path(worldgen.__file__).parent / "controller_templates")
    import types
    mw.install_fake_controller_module()
    ac = types.ModuleType("agent_common")
    ac.clamp = lambda v, lo, hi: max(lo, min(hi, v))
    ac.load_config = lambda: cfg
    ac.write_result = lambda *a, **k: None
    sys.modules["agent_common"] = ac
    spec = importlib.util.spec_from_file_location("arm_ctrl", src / "arm_controller.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    def tip(yaw, t1, t2):
        def ry(t, v): return (v[0] * math.cos(t) + v[2] * math.sin(t), v[1], -v[0] * math.sin(t) + v[2] * math.cos(t))
        v = (ARM["l1"], 0, 0)
        v2 = ry(t2, (ARM["l2"], 0, 0))
        v = ry(t1, (v[0] + v2[0], v2[1], v2[2]))
        v = (v[0], v[1], v[2] + ARM["h0"])
        return (v[0] * math.cos(yaw) - v[1] * math.sin(yaw), v[0] * math.sin(yaw) + v[1] * math.cos(yaw), v[2])

    n = 0
    for r in (0.18, 0.25, 0.35, 0.45, 0.5):
        for ang in (-2.5, -1.0, 0.0, 0.9, 2.8):
            for z in (-0.09, 0.0, 0.03):
                x, y = r * math.cos(ang), r * math.sin(ang)
                sol = mod.ik(x, y, ARM["h0"] + z)
                assert sol is not None, (r, ang, z)
                tx, ty, tz = tip(*sol)
                assert math.dist((tx, ty, tz), (x, y, ARM["h0"] + z)) < 1e-6
                n += 1
    assert n == 75
    assert mod.ik(0.9, 0, 0.1) is None          # unreachable
    sys.modules.pop("agent_common", None)


def test_arm_controller_pick_and_place_in_mock(settings):
    g = generate(parse_rules(EXAMPLES["arm"]).spec, settings)
    cdir = g.project / "controllers" / g.controller
    cfg = json.loads((cdir / "config.json").read_text())
    L1, L2, H0, TH = ARM["l1"], ARM["l2"], ARM["h0"], ARM["table_h"]

    class ArmSim:
        name, t, dt, paused, t_limit = "arm", 0.0, 0.016, False, 120.0

        def __init__(self):
            self.cube = mw.Node((cfg["arm"]["pick"][0], cfg["arm"]["pick"][1], cfg["arm"]["cube_z"] + 0.002))
            self.tipn = mw.Node()
            self.node = mw.Node((0, 0, TH))
            self.defs = {"OBJECT": self.cube, "ARM_TIP": self.tipn}
            self.m = {n: mw.Motor() for n in ("yaw motor", "shoulder motor", "elbow motor")}
            self.cur = {n: 0.0 for n in self.m}
            self.tgt = {n: 0.0 for n in self.m}

        def device(self, n):
            if n in self.m:
                mo = self.m[n]
                orig = mo.setPosition
                mo.setPosition = lambda p, n=n: self.tgt.__setitem__(n, p)
                return mo
            base = n.replace(" sensor", " motor")
            sim = self

            class S:
                def enable(s, ms): pass
                def getValue(s): return sim.cur[base]
            return S()

        def advance(self):
            for n in self.cur:
                d = self.tgt[n] - self.cur[n]
                self.cur[n] += max(-1.2 * self.dt, min(1.2 * self.dt, d))
            y, a, b = self.cur["yaw motor"], self.cur["shoulder motor"], self.cur["elbow motor"]
            ph1, ph2 = -a, -a - b
            r = L1 * math.cos(ph1) + L2 * math.cos(ph2)
            z = TH + H0 + L1 * math.sin(ph1) + L2 * math.sin(ph2)
            self.tipn.pos = [r * math.cos(y), r * math.sin(y), z]
            self.t += self.dt

    sim = ArmSim()
    mw.run_controller_file(cdir, sim)
    res = json.loads((g.project / "result_arm.json").read_text())
    assert res["success"], res
    assert res["metrics"]["final_error_m"] < 0.02


def test_drone_controller_smoke_and_timeout_failure(settings):
    spec = parse_rules(EXAMPLES["drone"]).spec
    spec.sim.timeout_s = 20.0
    g = generate(spec, settings)
    cdir = g.project / "controllers" / g.controller

    class DroneSim:
        name, t, dt, paused, t_limit = "drone", 0.0, 0.016, False, 40.0

        def __init__(self):
            self.z, self.vz, self.motors, self.node = 0.1, 0.0, {}, mw.Node()

        def device(self, n):
            sim = self
            if n.endswith("propeller"):
                return self.motors.setdefault(n, mw.Motor())
            class D:
                def enable(s, ms): pass
                def getRollPitchYaw(s): return (0.0, 0.0, 0.0)
                def getValues(s): return [0.0, 0.0, sim.z] if n == "gps" else [0.0, 0.0, 0.0]
            return D()

        def advance(self):
            if len(self.motors) == 4:
                thrust = sum(abs(m.velocity) for m in self.motors.values()) / 4
                assert math.isfinite(thrust)
                self.vz += (thrust - 68.5) * 0.05 * self.dt
                self.z = max(0.1, self.z + self.vz * self.dt)
            self.t += self.dt

    sim = DroneSim()
    mw.run_controller_file(cdir, sim)
    res = json.loads((g.project / "result_drone.json").read_text())
    assert res["robot"] == "drone" and res["success"] is False and "time limit" in res["reason"]   # fake never moves in x/y
    assert sim.z > 0.5            # altitude loop pushed thrust up


# ------------------------------------------------------------------------------------------ runner
class FakeProc:
    def __init__(self, code=None): self.code, self.pid = code, 4242
    def poll(self): return self.code
    def terminate(self): self.code = -15


def _launcher(log_text="", results=None, exit_code=None):
    def launch(cmd, cwd, log_path):
        Path(log_path).write_text(log_text, encoding="utf-8")
        for nm, r in (results or {}).items():
            (Path(cwd) / f"result_{nm}.json").write_text(json.dumps(r))
        return FakeProc(exit_code)
    return launch


def _fake_exe(settings, tmp_path):
    exe = tmp_path / "Webots" / "msys64" / "mingw64" / "bin" / "webots.exe"
    exe.parent.mkdir(parents=True, exist_ok=True)
    exe.write_text("")
    return settings.model_copy(update={"webots_executable": str(exe)})


def test_runner_success_needs_controller_result(settings, tmp_path):
    s = _fake_exe(settings, tmp_path)
    g = generate(parse_rules(EXAMPLES["epuck_nav"]).spec, s)
    ok = run_world(g.world, g.project, g.robot_names, 30, s, launcher=_launcher("INFO: started", {"robot_1": {"success": True, "reason": "ok"}}), poll_s=0.01)
    assert ok.status == "success" and ok.ok
    fail = run_world(g.world, g.project, g.robot_names, 30, s, launcher=_launcher("", {"robot_1": {"success": False, "reason": "time limit"}}), poll_s=0.01)
    assert fail.status == "task_failed" and not fail.ok


def test_runner_reports_webots_errors_instead_of_success(settings, tmp_path, monkeypatch):
    s = _fake_exe(settings, tmp_path)
    g = generate(parse_rules(EXAMPLES["epuck_nav"]).spec, s)
    log = "WARNING: Skipped unavailable PROTO 'E-puck'\nERROR: 'x.proto' not found\n"
    rep = run_world(g.world, g.project, g.robot_names, 30, s, launcher=_launcher(log, {"robot_1": {"success": True, "reason": "ok"}}), poll_s=0.01, startup_s=0.1)
    assert rep.status == "webots_errors" and not rep.ok and any("not found" in e for e in rep.errors)


def test_runner_reports_crash_and_missing_webots(settings, tmp_path):
    s = _fake_exe(settings, tmp_path)
    g = generate(parse_rules(EXAMPLES["epuck_nav"]).spec, s)
    rep = run_world(g.world, g.project, g.robot_names, 30, s, launcher=_launcher("", None, exit_code=3), poll_s=0.01)
    assert rep.status == "exited" and "code 3" in rep.message
    none = run_world(g.world, g.project, g.robot_names, 30, settings.model_copy(update={"webots_home": ""}), poll_s=0.01)
    assert none.status == "webots_not_found"


def test_scan_log_flags_controller_traceback():
    errs, warns = scan_log("Traceback (most recent call last):\n  File x\nNameError: name 'q' is not defined\nWARNING: soft thing\n[RESULT] {}")
    assert any("Traceback" in e for e in errs) and any("NameError" in e for e in errs) and warns == ["WARNING: soft thing"]


def test_pipeline_end_to_end_without_launch(settings):
    r = run_pipeline(EXAMPLES["line"], settings, launch=False)
    assert r.ok and r.stage == "done" and Path(r.world).is_file()
    bad = run_pipeline("build a humanoid", settings, launch=False)
    assert not bad.ok and bad.stage == "parse" and "not supported" in bad.message


def test_assets_not_shipped_locally_fall_back_to_pinned_github_url(tmp_path, monkeypatch):
    home = fake_install(tmp_path / "W", skip=tuple(CATALOG))          # install without the PROTO library
    (home / "projects").mkdir(exist_ok=True)
    monkeypatch.setattr(AssetResolver, "online_exists", staticmethod(lambda u: True))
    s = Settings(_env_file=None, webots_home=str(home), webots_project_dir=str(tmp_path / "p"))
    g = generate(parse_rules(EXAMPLES["epuck_nav"]).spec, s)
    text = g.world.read_text()
    assert 'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"' in text
