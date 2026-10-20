#  uv run uvicorn app.main:app --host 127.0.0.1 --port 8000 --> Run the project
import math
import sys
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from app.config import Settings
from app.main import create_app
from app.simulation.navigation import NavParams, Navigator, Odometry, summarize_sensors
from tests.fake_robot import AXLE, R, FakeEpuck


class FakeProc:
    _next = 4000

    def __init__(self, cmd):
        FakeProc._next += 1
        self.pid, self.cmd, self.code = FakeProc._next, cmd, None
        self.terminated = False

    def poll(self):
        return self.code

    def terminate(self):
        self.terminated, self.code = True, -15

    def kill(self):
        self.terminate()

    def wait(self, timeout=None):
        return self.code


@pytest.fixture
def procs():
    return []


@pytest.fixture
def settings(tmp_path):
    exe = tmp_path / "webots"
    exe.write_text("fake")
    return Settings(_env_file=None, webots_project_dir=str(tmp_path), webots_executable=str(exe),
                    simulation_timeout=120, max_robot_count=2)


@pytest.fixture
def client(settings, procs, monkeypatch):
    monkeypatch.setattr("app.agent.nodes.REPLAN_COOLDOWN_S", 0.15)

    def launcher(cmd, cwd, log):
        p = FakeProc(cmd)
        procs.append(p)
        return p

    app = create_app(settings, launcher=launcher, poll_s=0.05)
    app.state.manager.stop_grace_s = 0.3
    with TestClient(app) as c:
        c.app_ = app
        yield c


PROMPT = "Wheeled robot in a 2 x 2 m room with 3 obstacles and a red target on the far side"


def make_session(client, prompt=PROMPT):
    r = client.post("/api/simulations", json={"prompt": prompt})
    assert r.status_code == 201, r.text
    return r.json()


def drive_controller(client, session, stop_after_steps=40000, stuck=False):
    """Plays the role of the Webots controller using the real Navigator + kinematic robot, speaking the real
    HTTP telemetry protocol (token header, command replies)."""
    sid = session["id"]
    token = client.app_.state.manager.get(sid).token
    spec, world = session["spec"], session["world"]
    robot = FakeEpuck(spec["start_position"], spec["start_heading"], world["obstacles"], world["width"], world["height"])
    odo = Odometry(*spec["start_position"], spec["start_heading"], R, AXLE)
    nav = Navigator([tuple(w) for w in world["waypoints"]], NavParams(max_speed=spec["max_speed"]))
    target = tuple(spec["target_position"])
    collected, travelled, last = [], 0.0, (robot.x, robot.y)
    for step in range(stop_after_steps):
        pose = odo.update(robot.wl, robot.wr)
        summ = summarize_sensors(robot.sensors())
        cmd = nav.update(pose, summ, robot.t)
        robot.step(*cmd)
        travelled += math.dist(last, (robot.x, robot.y)); last = (robot.x, robot.y)
        done = nav.state == "arrived"
        if step % 8 == 0 or done:
            d = math.dist((robot.x, robot.y), target)
            payload = {"t": robot.t, "state": "finished" if done else nav.state, "position": [robot.x, robot.y],
                       "heading": robot.th, "odometry": list(pose), "target_distance": d,
                       "sensors": {}, "sensor_summary": summ, "motors": list(cmd),
                       "metrics": {"distance_travelled": travelled, "collisions": robot.collisions,
                                   "obstacle_encounters": nav.encounters},
                       "finished": done, "events": nav.pop_events(),
                       "result": {"success": d < 0.15, "reason": "target reached"} if done else None}
            r = client.post(f"/internal/sessions/{sid}/telemetry", json=payload, headers={"X-Session-Token": token})
            assert r.status_code == 200, r.text
            for c in r.json()["commands"]:
                collected.append(c["name"])
                if c["name"] == "stop":
                    return collected
        if done:
            return collected
    return collected
