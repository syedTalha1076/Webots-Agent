"""Generated controller for wheeled robots (E-puck or generated 2/4-wheel robot).

Tasks (set in config.json): wander_avoid | go_to_targets | line_follow.
 - Pose comes from the Supervisor API (simulator ground truth) - state this when reporting results.
 - Obstacle sensing uses the robot's own distance sensors; the control loop never calls an LLM.
 - Multi-robot: every robot runs this same file and looks itself up in config["robots"] by its Webots name.
"""
import math
import random
import sys
import time

from controller import Supervisor

from agent_common import all_results_present, clamp, load_config, pose2d, result_path, wrap, write_result

CFG = load_config()


class Sensors:
    """Maps raw device values to normalised obstacle levels (0 = free, ~1 = touching)."""

    def __init__(self, robot, step_ms):
        self.layout = CFG["distance"]["layout"]
        self.devs = []
        for n in CFG["distance"]["names"]:
            d = robot.getDevice(n)
            d.enable(step_ms)
            self.devs.append(d)
        self.scale = CFG["distance"]["scale"]

    def read(self):
        v = [max(0.0, d.getValue()) / self.scale for d in self.devs]
        if self.layout == "epuck":          # ps0 FR, ps1 R45, ps2 R, ps5 L, ps6 L45, ps7 FL
            return {"front": max(v[0], v[7]), "front_right": v[1], "right": v[2], "front_left": v[6], "left": v[5]}
        # custom: [left_side, left45, front, right45, right_side]
        return {"front": v[2], "front_left": v[1], "front_right": v[3], "left": v[0], "right": v[4]}


class LineCamera:
    def __init__(self, robot, step_ms):
        self.cam = robot.getDevice(CFG["line_camera"]["name"])
        self.cam.enable(step_ms)
        self.w, self.h = self.cam.getWidth(), self.cam.getHeight()

    def line_position(self):
        """Returns (-1..1, seen). +1 = line at the right edge of the image (robot's right)."""
        img = self.cam.getImage()
        if not img:
            return 0.0, False
        w, h = self.w, self.h
        weights, total = [], 0.0
        for x in range(w):
            lum = 0.0
            for y in range(h):
                i = 4 * (y * w + x)
                lum += (img[i] + img[i + 1] + img[i + 2]) / 3.0
            lum /= h
            dark = max(0.0, 110.0 - lum)
            weights.append(dark)
            total += dark
        if total < 40.0:
            return 0.0, False
        c = (w - 1) / 2.0
        return sum(wt * (x - c) / c for x, wt in enumerate(weights)) / total, True


def main():
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    me = robot.getSelf()
    name = robot.getName()
    mine = CFG["robots"][name]
    names = list(CFG["robots"].keys())
    first = name == names[0]
    vmax = float(CFG["max_speed"])
    task = CFG["task"]
    rnd = random.Random(sum(ord(c) for c in name))

    left_motors = [robot.getDevice(n) for n in CFG["motors"]["left"]]
    right_motors = [robot.getDevice(n) for n in CFG["motors"]["right"]]
    for m in left_motors + right_motors:
        m.setPosition(float("inf"))
        m.setVelocity(0.0)
    sensors = Sensors(robot, step_ms)
    line = LineCamera(robot, step_ms) if task == "line_follow" else None

    def drive(l, r):
        l, r = clamp(l, -vmax, vmax), clamp(r, -vmax, vmax)
        for m in left_motors:
            m.setVelocity(l)
        for m in right_motors:
            m.setVelocity(r)

    on, off = CFG["distance"]["avoid_on"], CFG["distance"]["avoid_off"]
    side_on = CFG["distance"]["side_on"]
    rr = CFG["contact_radius"]
    targets = [tuple(t) for t in mine["targets"]]
    tol = CFG["goal_tol"]
    obstacles, hw, hh = CFG["obstacles"], CFG["room"][0] / 2.0, CFG["room"][1] / 2.0

    state, state_until, turn_dir, last_avoid = "go", 0.0, 1.0, -99.0
    idx = 0
    travelled, collisions, in_contact, last_xy = 0.0, 0, False, None
    prog_t, prog_xy = 0.0, None
    result = None
    paused = False
    ang_acc, last_ang = 0.0, None
    lost_dir, lost_since = 1.0, None
    wander_turn_until, wander_dir = 0.0, 1.0
    cx, cy = CFG.get("track_center", [0.0, 0.0])
    print("[%s] controller started: task=%s robots=%d" % (name, task, len(names)))

    while robot.step(step_ms) != -1:
        t = robot.getTime()
        x, y, th = pose2d(me)
        if last_xy is not None:
            travelled += math.hypot(x - last_xy[0], y - last_xy[1])
        last_xy = (x, y)

        touching = any(abs(x - o["x"]) < o["sx"] / 2 + rr * 0.95 and abs(y - o["y"]) < o["sy"] / 2 + rr * 0.95
                       for o in obstacles) or abs(x) > hw - rr * 0.6 or abs(y) > hh - rr * 0.6
        if touching and not in_contact:
            collisions += 1
        in_contact = touching

        if result is not None:
            drive(0.0, 0.0)
            if first and not paused and all_results_present(names):
                paused = True
                if not CFG.get("keep_running"):
                    robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
            continue

        metrics = {"time_s": round(t, 2), "distance_travelled_m": round(travelled, 3), "collisions": collisions,
                   "pose_source": "supervisor ground truth"}
        if t > CFG["timeout_s"] and task != "wander_avoid":
            result = write_result(name, False, "simulation time limit exceeded (%.0f s)" % CFG["timeout_s"], metrics)
            continue

        s = sensors.read()
        front_blocked = s["front"] > on or max(s["front_left"], s["front_right"]) > on * 1.3

        # ------------------------------------------------------------------ line following
        if task == "line_follow":
            pos, seen = line.line_position()
            ang = math.atan2(y - cy, x - cx)
            if last_ang is not None:
                ang_acc += wrap(ang - last_ang)
            last_ang = ang
            if abs(ang_acc) >= 2 * math.pi * CFG["laps"] - 0.35:
                result = write_result(name, True, "completed %d lap(s)" % CFG["laps"], metrics)
                continue
            if seen:
                lost_since = None
                lost_dir = 1.0 if pos > 0 else -1.0
                v = vmax * 0.5 * (1.0 - 0.65 * min(1.0, abs(pos)))
                k = vmax * 0.9
                drive(v + k * pos, v - k * pos)
            else:
                lost_since = lost_since or t
                if t - lost_since > 12.0:
                    result = write_result(name, False, "line lost for more than 12 s", metrics)
                    continue
                drive(vmax * 0.25 + lost_dir * vmax * 0.3, vmax * 0.25 - lost_dir * vmax * 0.3)
            continue

        # ------------------------------------------------------------------ wander + avoid
        if task == "wander_avoid":
            if t > CFG["duration_s"]:
                ok = collisions == 0
                result = write_result(name, ok, "ran %.0f s with %d collision(s)" % (CFG["duration_s"], collisions), metrics)
                continue
            if t > wander_turn_until and state == "go":
                if rnd.random() < 0.02:
                    wander_dir = rnd.choice((-1.0, 1.0))
                    wander_turn_until = t + rnd.uniform(0.4, 1.2)
            v = 0.7 * vmax
            w = wander_dir * 0.5 * vmax if t < wander_turn_until else 0.0
        # ------------------------------------------------------------------ go to targets
        else:
            if not targets:
                result = write_result(name, True, "no targets", metrics)
                continue
            gx, gy = targets[idx]
            dist = math.hypot(gx - x, gy - y)
            if dist < tol:
                idx += 1
                print("[%s] reached target %d/%d at t=%.1fs" % (name, idx, len(targets), t))
                if idx >= len(targets):
                    if CFG.get("loop_targets"):
                        idx = 0
                    else:
                        result = write_result(name, True, "all %d target(s) reached" % len(targets), dict(metrics, final_distance_m=round(dist, 3)))
                        continue
                gx, gy = targets[idx]
                dist = math.hypot(gx - x, gy - y)
            err = wrap(math.atan2(gy - y, gx - x) - th)
            if abs(err) > 0.7:
                v, w = 0.0, clamp(2.0 * err, -0.7, 0.7) * vmax / 0.7 * 0.6
            else:
                v = vmax * clamp(dist / 0.4, 0.25, 0.85) * (1.0 - abs(err) / 1.4)
                w = clamp(2.5 * err, -1.0, 1.0) * vmax * 0.5

        # ------------------------------------------------------------------ shared avoidance + recovery
        if state in ("go", "bypass") and front_blocked:
            if state == "go" or t - last_avoid > 4.0:     # keep the same turning side while bypassing one obstacle
                lvl_l, lvl_r = max(s["front_left"], s["left"]), max(s["front_right"], s["right"])
                turn_dir = 1.0 if lvl_r >= lvl_l else -1.0      # +1 = turn left (CCW)
            state, state_until = "avoid", t + 0.5
        if state == "avoid":
            last_avoid = t
            if t > state_until and s["front"] < off and max(s["front_left"], s["front_right"]) < off * 1.4:
                state, state_until = "bypass", t + CFG["bypass_s"]
            else:
                drive(-turn_dir * vmax * 0.5, turn_dir * vmax * 0.5)
                continue
        if state == "bypass":
            if t > state_until:
                state = "go"
            else:                                           # drive past the obstacle before re-aiming at the goal
                v, w = 0.55 * vmax, 0.0
        if state == "recover":
            if t > state_until:
                state = "go"
            elif t < state_until - 1.0:
                drive(-vmax * 0.5, -vmax * 0.5)
                continue
            else:
                drive(-turn_dir * vmax * 0.5, turn_dir * vmax * 0.5)
                continue
        # progress check (stuck against something the sensors cannot see)
        if prog_xy is None or t - prog_t > 6.0:
            if prog_xy is not None and math.hypot(x - prog_xy[0], y - prog_xy[1]) < 0.04 and task != "line_follow":
                turn_dir = rnd.choice((-1.0, 1.0))
                state, state_until = "recover", t + 2.0
                print("[%s] stuck at t=%.1fs: backing up" % (name, t))
            prog_t, prog_xy = t, (x, y)
        w += clamp(max(s["front_right"], s["right"]) - max(s["front_left"], s["left"]), -1, 1) * vmax * 0.4
        # never steer INTO an obstacle that is beside us, and slow down near anything
        lvl_l, lvl_r = max(s["front_left"], s["left"]), max(s["front_right"], s["right"])
        if w > 0 and lvl_l > side_on:
            w = 0.0
        if w < 0 and lvl_r > side_on:
            w = 0.0
        v *= 1.0 - 0.6 * clamp(max(lvl_l, lvl_r, s["front"]) / on, 0.0, 1.0)
        drive(v - w, v + w)


if __name__ == "__main__":
    main()
