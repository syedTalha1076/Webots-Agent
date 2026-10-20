import time

import pytest
from fastapi.testclient import TestClient

from app.config import Settings
from app.main import create_app
from tests.conftest import PROMPT, drive_controller, make_session


def wait_for(client, sid, pred, timeout=8.0):
    end = time.time() + timeout
    while time.time() < end:
        s = client.get(f"/api/simulations/{sid}").json()
        if pred(s):
            return s
        time.sleep(0.05)
    raise AssertionError(f"timeout; last state: {s['status']} / {s['stage']}")


# ------------------------------------------------------------------ API + agent planning phase
def test_health_reports_webots_and_fallback_mode(client):
    h = client.get("/api/health").json()
    assert h["webots"]["available"] is True and h["llm"]["enabled"] is False and h["llm"]["parser"] == "rules"


def test_create_runs_agent_and_generates_world(client, settings):
    s = make_session(client)
    assert s["status"] == "ready" and s["plan"]["controller"] == "navigation_controller"
    assert s["world"]["obstacles"] and s["world"]["target"]
    world = (settings.simulations_dir / s["id"] / "worlds" / "world.wbt").read_text()
    assert "E-puck" in world and "DEF TARGET" in world and world.count("WoodenBox {") == 3
    assert (settings.simulations_dir / s["id"] / "controllers" / "navigation_controller" / "navigation.py").exists()
    stages = [e["message"] for e in s["events"]]
    assert any("Interpreting" in m for m in stages) and any("World ready" in m for m in stages)


def test_unsupported_request_returns_clarification_not_world(client, settings):
    r = client.post("/api/simulations", json={"prompt": "Build a drone that flies to a target"}).json()
    assert r["status"] == "needs_clarification" and "not supported" in r["clarification"]
    assert r["world"] is None and not (settings.simulations_dir / r["id"]).exists()


def test_invalid_config_returns_clarification(client):
    r = client.post("/api/simulations", json={"prompt": "robot in a 2 m room, target at (8, 8)"}).json()
    assert r["status"] == "needs_clarification" and "outside the room" in r["clarification"]


def test_overcrowded_room_is_rejected(client):
    r = client.post("/api/simulations", json={"prompt": "robot in a 1 x 1 m room with 20 obstacles"}).json()
    assert r["status"] == "needs_clarification"


def test_request_validation_and_unknown_session(client):
    assert client.post("/api/simulations", json={"prompt": "x"}).status_code == 422
    assert client.get("/api/simulations/deadbeef0000").status_code == 404
    assert client.get("/api/simulations/not-a-valid-id").status_code == 422


def test_start_requires_ready_state(client):
    s = client.post("/api/simulations", json={"prompt": "Build a drone"}).json()
    assert client.post(f"/api/simulations/{s['id']}/start").status_code == 409


# ------------------------------------------------------------------ manager / lifecycle
def test_start_without_webots_gives_503_and_session_stays_ready(tmp_path, procs):
    st = Settings(_env_file=None, webots_project_dir=str(tmp_path), webots_executable="", webots_home="")
    st_missing = st.model_copy(update={"webots_executable": str(tmp_path / "nope.exe")})
    import shutil
    if shutil.which("webots"):
        pytest.skip("a real Webots is on PATH")
    with TestClient(create_app(st_missing, poll_s=0.05)) as c:
        s = make_session(c)
        r = c.post(f"/api/simulations/{s['id']}/start")
        assert r.status_code == 503 and "Webots not found" in r.json()["detail"]
        assert c.get(f"/api/simulations/{s['id']}").json()["status"] == "ready"
        assert c.get("/api/health").json()["webots"]["available"] is False


def test_start_launches_only_approved_exe_with_generated_world(client, procs, settings):
    s = make_session(client)
    r = client.post(f"/api/simulations/{s['id']}/start")
    assert r.status_code == 200 and r.json()["status"] == "starting"
    cmd = procs[-1].cmd
    assert cmd[0] == str(settings.find_webots()) and cmd[-1].endswith("world.wbt")
    assert str(settings.simulations_dir / s["id"]) in cmd[-1] and "--batch" in cmd
    assert client.post(f"/api/simulations/{s['id']}/start").status_code == 409   # no double launch


def test_concurrency_limit(client, procs):
    ids = [make_session(client)["id"] for _ in range(3)]
    assert client.post(f"/api/simulations/{ids[0]}/start").status_code == 200
    assert client.post(f"/api/simulations/{ids[1]}/start").status_code == 200
    assert client.post(f"/api/simulations/{ids[2]}/start").status_code == 429


def test_telemetry_requires_correct_session_token(client):
    a, b = make_session(client), make_session(client)
    tok_b = client.app_.state.manager.get(b["id"]).token
    body = {"t": 1.0, "state": "go_to_goal", "position": [0, 0]}
    assert client.post(f"/internal/sessions/{a['id']}/telemetry", json=body).status_code == 403
    assert client.post(f"/internal/sessions/{a['id']}/telemetry", json=body, headers={"X-Session-Token": tok_b}).status_code == 403
    assert client.post(f"/internal/sessions/deadbeef0000/telemetry", json=body, headers={"X-Session-Token": tok_b}).status_code == 404


def test_telemetry_flows_to_rest_and_websocket_and_sessions_are_isolated(client):
    a, b = make_session(client), make_session(client)
    for s in (a, b):
        client.post(f"/api/simulations/{s['id']}/start")
    tok = client.app_.state.manager.get(a["id"]).token
    with client.websocket_connect(f"/ws/simulations/{a['id']}") as wa, client.websocket_connect(f"/ws/simulations/{b['id']}") as wb:
        assert wa.receive_json()["type"] == "snapshot" and wb.receive_json()["type"] == "snapshot"
        r = client.post(f"/internal/sessions/{a['id']}/telemetry", headers={"X-Session-Token": tok},
                        json={"t": 2.0, "state": "go_to_goal", "position": [0.1, 0.2], "target_distance": 1.0,
                              "sensor_summary": {"left": 1, "front": 2, "right": 3}})
        assert r.json() == {"commands": []}
        seen = []
        while True:
            m = wa.receive_json()
            seen.append(m["type"])
            if m["type"] == "telemetry":
                assert m["telemetry"]["position"] == [0.1, 0.2]
                break
        wa.send_text("ping")
    assert client.get(f"/api/simulations/{a['id']}/telemetry").json()["telemetry"]["t"] == 2.0
    assert client.get(f"/api/simulations/{b['id']}/telemetry").json()["telemetry"] is None
    assert client.get(f"/api/simulations/{a['id']}").json()["status"] == "running"
    assert client.get(f"/api/simulations/{b['id']}").json()["status"] == "starting"


def test_websocket_unknown_session_closed(client):
    with pytest.raises(Exception):
        with client.websocket_connect("/ws/simulations/deadbeef0000"):
            pass


def test_pause_resume_commands_are_delivered_in_order(client):
    s = make_session(client)
    client.post(f"/api/simulations/{s['id']}/start")
    tok = client.app_.state.manager.get(s["id"]).token
    post = lambda state: client.post(f"/internal/sessions/{s['id']}/telemetry", headers={"X-Session-Token": tok},
                                     json={"t": 1, "state": state, "position": [0, 0]}).json()["commands"]
    post("go_to_goal")
    assert client.post(f"/api/simulations/{s['id']}/pause").status_code == 200
    assert post("go_to_goal") == [{"name": "pause"}]
    post("paused")
    assert client.get(f"/api/simulations/{s['id']}").json()["status"] == "paused"
    assert client.post(f"/api/simulations/{s['id']}/pause").status_code == 409
    assert client.post(f"/api/simulations/{s['id']}/resume").status_code == 200
    assert post("paused") == [{"name": "resume"}]


def test_commands_rejected_when_not_running(client):
    s = make_session(client)
    for cmd in ("pause", "resume"):
        assert client.post(f"/api/simulations/{s['id']}/{cmd}").status_code == 409
    assert client.post(f"/api/simulations/{s['id']}/reset").status_code == 409


def test_emergency_stop_has_priority_and_kills_only_own_process(client, procs):
    a, b = make_session(client), make_session(client)
    client.post(f"/api/simulations/{a['id']}/start")
    client.post(f"/api/simulations/{b['id']}/start")
    pa, pb = procs[-2], procs[-1]
    tok = client.app_.state.manager.get(a["id"]).token
    client.post(f"/api/simulations/{a['id']}/pause")
    out = client.post(f"/api/simulations/{a['id']}/stop")          # blocks <=4s waiting for graceful quit, then kills
    assert out.status_code == 200 and out.json()["status"] == "stopped"
    assert pa.terminated and not pb.terminated
    assert out.json()["result"]["reason"] == "stopped by user"
    # queued pause was discarded; only the stop is delivered to the controller
    q = client.app_.state.manager.get(a["id"]).commands
    assert q.estop
    assert client.get(f"/api/simulations/{b['id']}").json()["status"] == "starting"
    assert client.post(f"/api/simulations/{a['id']}/pause").status_code == 409


def test_stop_command_is_first_and_only_command_after_estop(client):
    s = make_session(client)
    client.post(f"/api/simulations/{s['id']}/start")
    m = client.app_.state.manager.get(s["id"])
    m.commands.push("pause")
    m.commands.push("stop")
    m.commands.push("resume")
    assert m.commands.drain() == [{"name": "stop"}]


# ------------------------------------------------------------------ watchdog
def test_watchdog_fails_session_when_webots_crashes_before_connecting(client, procs):
    s = make_session(client)
    client.post(f"/api/simulations/{s['id']}/start")
    procs[-1].code = 1
    import asyncio
    sess = client.app_.state.manager.get(s["id"])
    asyncio.run(client.app_.state.manager.check(sess))
    assert sess.status.value == "failed" and "exited with code 1" in sess.result["reason"]


def test_watchdog_lost_telemetry_and_timeout(client):
    import asyncio
    s = make_session(client)
    client.post(f"/api/simulations/{s['id']}/start")
    sess = client.app_.state.manager.get(s["id"])
    tok = sess.token
    client.post(f"/internal/sessions/{s['id']}/telemetry", headers={"X-Session-Token": tok},
                json={"t": 1, "state": "go_to_goal", "position": [0, 0]})
    asyncio.run(client.app_.state.manager.check(sess, now=time.time() + 60))
    assert sess.status.value == "failed" and "Lost telemetry" in sess.result["reason"]


def test_watchdog_controller_never_connected(client):
    import asyncio
    s = make_session(client)
    client.post(f"/api/simulations/{s['id']}/start")
    sess = client.app_.state.manager.get(s["id"])
    asyncio.run(client.app_.state.manager.check(sess, now=time.time() + 500))
    assert sess.status.value in ("failed", "timeout")


def test_delete_cleans_directory_only_inside_simulations(client, settings):
    s = make_session(client)
    d = settings.simulations_dir / s["id"]
    assert d.exists() and client.delete(f"/api/simulations/{s['id']}").status_code == 200 and not d.exists()
    client.post("/api/simulations", json={"prompt": PROMPT})
    assert settings.simulations_dir.exists()


# ------------------------------------------------------------------ end-to-end protocol with a fake controller
def test_full_run_with_fake_controller_reports_measured_result(client):
    s = make_session(client, "robot in a 2 x 2 m room with 3 obstacles, target far side")
    assert client.post(f"/api/simulations/{s['id']}/start").status_code == 200
    drive_controller(client, s)
    done = wait_for(client, s["id"], lambda x: x["status"] == "finished")
    res = done["result"]
    assert res["success"] is True and res["measured"]["final_distance_to_target_m"] < 0.15
    assert res["measured"]["distance_travelled_m"] > 1.0
    assert res["estimated"]["straight_line_distance_m"] == pytest.approx(1.98, abs=0.01)
    assert "estimated" in res and "measured" in res
    end = wait_for(client, s["id"], lambda x: any("Final result" in e["message"] for e in x["events"]))
    assert end["status"] == "finished"


def test_agent_replans_when_stuck_then_gives_up(client):
    s = make_session(client)
    client.post(f"/api/simulations/{s['id']}/start")
    tok = client.app_.state.manager.get(s["id"]).token
    names, end = [], time.time() + 10
    while time.time() < end:
        r = client.post(f"/internal/sessions/{s['id']}/telemetry", headers={"X-Session-Token": tok},
                        json={"t": 50, "state": "stuck", "position": [0, 0], "target_distance": 1.5})
        names += [c["name"] for c in r.json()["commands"]]
        if client.get(f"/api/simulations/{s['id']}").json()["status"] == "finished":
            break
        time.sleep(0.05)
    final = client.get(f"/api/simulations/{s['id']}").json()
    assert names.count("replan") == 3
    assert final["result"]["success"] is False and "stuck after 3" in final["result"]["reason"]
    assert final["result"]["measured"]["replanning_events"] == 3


def test_reset_after_finish_restarts_run(client):
    s = make_session(client)
    client.post(f"/api/simulations/{s['id']}/start")
    drive_controller(client, s)
    wait_for(client, s["id"], lambda x: x["status"] == "finished")
    assert client.post(f"/api/simulations/{s['id']}/reset").status_code == 200
    got = wait_for(client, s["id"], lambda x: x["status"] == "running" and x["result"] is None)
    tok = client.app_.state.manager.get(s["id"]).token
    cmds = client.post(f"/internal/sessions/{s['id']}/telemetry", headers={"X-Session-Token": tok},
                       json={"t": 0, "state": "go_to_goal", "position": [0, 0]}).json()["commands"]
    assert cmds == [{"name": "reset"}]
