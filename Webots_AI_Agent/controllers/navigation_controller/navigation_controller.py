"""Webots controller (Supervisor-enabled e-puck): sensor-based navigation + telemetry bridge.

Runs inside Webots' Python. Standard library only.
 - Control loop and safety run locally at every simulation step (never waits for an LLM).
 - Navigation uses wheel odometry + IR distance sensors. Supervisor ground truth is used ONLY for
   telemetry and evaluation (final distance, collisions, distance travelled).
 - Telemetry is POSTed to the backend; the HTTP reply carries pending commands (stop/pause/resume/reset/replan).
 - If the backend cannot be reached the robot stops safely and keeps trying to reconnect.
"""
import json
import math
import os
import sys
import time
import urllib.request

try:
    from controller import Supervisor
except ImportError:  # allows `python navigation_controller.py` syntax checks outside Webots
    Supervisor = None

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    from navigation import NavParams, Navigator, Odometry, summarize_sensors, wrap
except ImportError:  # sample world inside the repo: import from the app package
    sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
    from app.simulation.navigation import NavParams, Navigator, Odometry, summarize_sensors, wrap

POST_EVERY_STEPS = 4
COMM_LOSS_SECONDS = 4.0
SUCCESS_RADIUS = 0.15   # ground-truth distance (m) that counts as "reached"


def load_config():
    if len(sys.argv) < 2:
        raise SystemExit("navigation_controller: missing config path argument")
    path = sys.argv[1]
    if not os.path.isabs(path):
        path = os.path.abspath(os.path.join(HERE, path))
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


class Bridge:
    def __init__(self, cfg):
        self.base = cfg.get("backend_url", "")
        self.sid, self.token = cfg["session_id"], cfg["token"]
        self.last_ok = time.time()
        self.ok = True

    def exchange(self, payload):
        """POST telemetry; returns list of commands (possibly empty). Sets self.ok."""
        if not self.base:
            return []
        try:
            req = urllib.request.Request(
                "%s/internal/sessions/%s/telemetry" % (self.base, self.sid),
                data=json.dumps(payload).encode("utf-8"),
                headers={"Content-Type": "application/json", "X-Session-Token": self.token},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=0.4) as resp:
                body = json.loads(resp.read().decode("utf-8"))
            self.last_ok, self.ok = time.time(), True
            return body.get("commands", [])
        except Exception:
            self.ok = False
            return []

    @property
    def lost(self):
        return bool(self.base) and (time.time() - self.last_ok) > COMM_LOSS_SECONDS


def main():
    cfg = load_config()
    spec, rb = cfg["spec"], cfg["robot"]
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    me = robot.getSelf()
    translation, rotation = me.getField("translation"), me.getField("rotation")
    start_xy, start_h = spec["start_position"], spec["start_heading"]

    left_m, right_m = robot.getDevice("left wheel motor"), robot.getDevice("right wheel motor")
    for m in (left_m, right_m):
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    left_s, right_s = robot.getDevice("left wheel sensor"), robot.getDevice("right wheel sensor")
    left_s.enable(step_ms)
    right_s.enable(step_ms)
    ps = []
    for i in range(8):
        d = robot.getDevice("ps%d" % i)
        d.enable(step_ms)
        ps.append(d)

    odo = Odometry(start_xy[0], start_xy[1], start_h, rb["wheel_radius"], rb["axle_length"])
    nav_params = NavParams(**cfg["nav"])

    def new_navigator():
        return Navigator([tuple(w) for w in cfg["waypoints"]], NavParams(**cfg["nav"]))

    nav = new_navigator()
    bridge = Bridge(cfg)
    target = tuple(spec["target_position"])
    obstacles = cfg["obstacles"]
    limit = rb["max_wheel_speed"]

    state_override = None       # None | "paused" | "stopped" | "comm_lost"
    finished, result = False, None
    travelled, collisions, in_contact = 0.0, 0, False
    last_pos = None
    pending_events = ["controller started (session %s)" % cfg["session_id"]]
    step = 0
    cmd = (0.0, 0.0)
    pending_quit = False
    t0_wall = time.time()

    def truth():
        p = me.getPosition()
        o = me.getOrientation()
        return (p[0], p[1]), math.atan2(o[3], o[0])

    def reset_robot():
        nonlocal nav, finished, result, travelled, collisions, in_contact, last_pos, state_override
        translation.setSFVec3f([start_xy[0], start_xy[1], 0.0])
        rotation.setSFRotation([0, 0, 1, start_h])
        me.resetPhysics()
        odo.reset(start_xy[0], start_xy[1], start_h)
        nav = new_navigator()
        finished, result, travelled, collisions, in_contact, last_pos = False, None, 0.0, 0, False, None
        state_override = None
        pending_events.append("robot reset to start pose")

    while robot.step(step_ms) != -1:
        step += 1
        t = robot.getTime()
        pos, heading = truth()
        if last_pos is not None:
            travelled += math.hypot(pos[0] - last_pos[0], pos[1] - last_pos[1])
        last_pos = pos

        # collision accounting against known obstacle boxes (evaluation only, approximate geometry)
        touching = any(
            max(abs(pos[0] - o["x"]), abs(pos[1] - o["y"])) < o["size"] / 2 + rb["robot_radius"] * 0.9
            for o in obstacles
        )
        if touching and not in_contact:
            collisions += 1
            pending_events.append("collision with obstacle (ground truth)")
        in_contact = touching

        raw = [d.getValue() for d in ps]
        pose = odo.update(left_s.getValue(), right_s.getValue())
        summary = summarize_sensors(raw)

        # ---- decide motor command (safety first)
        if bridge.lost and state_override is None and not finished:
            state_override = "comm_lost"
            pending_events.append("backend connection lost: robot stopped safely")
        if state_override in ("paused", "stopped", "comm_lost") or finished:
            cmd = (0.0, 0.0)
        else:
            cmd = nav.update(pose, summary, t)
            pending_events.extend(nav.pop_events())
            if nav.state == "arrived":
                d = math.dist(pos, target)
                finished = True
                result = {"success": d <= SUCCESS_RADIUS, "reason": "target reached" if d <= SUCCESS_RADIUS
                          else "odometry drifted: stopped %.2f m from target" % d}
            elif nav.state == "stuck":
                pass  # backend agent decides: replan or fail
        cmd = (max(-limit, min(limit, cmd[0])), max(-limit, min(limit, cmd[1])))
        left_m.setVelocity(cmd[0])
        right_m.setVelocity(cmd[1])

        if t > cfg.get("timeout_s", 300) and not finished:
            finished = True
            result = {"success": False, "reason": "simulation time limit exceeded"}
            pending_events.append("time limit exceeded")

        # ---- telemetry + commands
        if step % POST_EVERY_STEPS == 0 or finished and step % 2 == 0:
            tdist = math.dist(pos, target)
            state = state_override or ("finished" if finished else nav.state)
            payload = {
                "t": round(t, 3), "state": state,
                "position": [round(pos[0], 4), round(pos[1], 4)], "heading": round(heading, 4),
                "odometry": [round(pose[0], 4), round(pose[1], 4), round(pose[2], 4)],
                "target_distance": round(tdist, 4),
                "sensors": {"ps%d" % i: round(v, 1) for i, v in enumerate(raw)},
                "sensor_summary": {k: round(v, 1) for k, v in summary.items()},
                "motors": [round(cmd[0], 3), round(cmd[1], 3)],
                "waypoint_index": nav.wp_index,
                "metrics": {"distance_travelled": round(travelled, 4), "collisions": collisions,
                            "obstacle_encounters": nav.encounters, "controller_replans": nav.replans,
                            "wall_time": round(time.time() - t0_wall, 2)},
                "finished": finished,
                "result": dict(result, final_distance=round(tdist, 4)) if result else None,
                "events": pending_events[-20:],
            }
            if not bridge.base:   # standalone sample world: print to the Webots console
                for e in pending_events:
                    print("[nav t=%.1fs] %s" % (t, e))
                if finished and result and step % 200 == 0:
                    print("[nav] RESULT:", payload["result"])
            pending_events.clear()
            for c in bridge.exchange(payload):
                name = c.get("name")
                if name == "stop":
                    state_override = "stopped"
                    left_m.setVelocity(0.0)
                    right_m.setVelocity(0.0)
                    pending_quit = True
                elif name == "pause" and not pending_quit:
                    state_override = "paused"
                elif name == "resume" and state_override in ("paused", "comm_lost"):
                    state_override = None
                elif name == "reset":
                    reset_robot()
                elif name == "replan":
                    nav.replan(float(c.get("speed_scale", 1.0)))
            if pending_quit:
                bridge.exchange(dict(payload, state="stopped", events=["emergency stop executed"]))
                robot.simulationQuit(0)   # closes exactly this Webots instance
                return


if __name__ == "__main__":
    main()
