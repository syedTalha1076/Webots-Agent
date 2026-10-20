"""Minimal fake `controller` module + 2D kinematic simulator used to exercise generated controllers WITHOUT Webots.

This checks the control logic and the controller<->world contract (device names, config keys, DEF names).
It is NOT a physics simulation: passing here does not prove the robot behaves identically in Webots.
"""
from __future__ import annotations

import math
import runpy
import sys
import types
from pathlib import Path

ACTIVE: dict = {}


class Motor:
    def __init__(self): self.velocity, self.position = 0.0, 0.0
    def setPosition(self, p): self.position = p
    def setVelocity(self, v): self.velocity = v
    def getValue(self): return self.position


class PosSensor:
    def __init__(self, motor): self.motor = motor
    def enable(self, ms): pass
    def getValue(self): return self.motor.position


class RaySensor:
    def __init__(self, sim, angle, rng): self.sim, self.angle, self.rng = sim, angle, rng
    def enable(self, ms): pass
    def getValue(self): return self.sim.ray(self.angle, self.rng)


class Camera:
    def __init__(self, sim): self.sim, self.w, self.h = sim, 32, 8
    def enable(self, ms): pass
    def getWidth(self): return self.w
    def getHeight(self): return self.h
    def getImage(self): return self.sim.camera_image(self.w, self.h)


class Field:
    def __init__(self, node): self.node = node
    def setSFVec3f(self, v): self.node.pos = list(v)


class Node:
    def __init__(self, pos=(0, 0, 0), yaw=0.0): self.pos, self.yaw = list(pos), yaw
    def getPosition(self): return list(self.pos)
    def getOrientation(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [c, -s, 0, s, c, 0, 0, 0, 1]
    def getField(self, name): return Field(self)
    def resetPhysics(self): pass


class WheeledSim:
    """Differential/skid drive, ray distance sensors, downward line camera."""

    def __init__(self, cfg, name, start, wheel_r, axle, sensor_defs, boxes, room, line=None, cam_ahead=0.15, t_limit=400.0):
        self.cfg, self.name, self.r, self.axle = cfg, name, wheel_r, axle
        self.node = Node((start[0], start[1], 0), start[2])
        self.boxes, self.room, self.line, self.t, self.dt = boxes, room, line or [], 0.0, 0.016
        self.t_limit, self.paused, self.motors, self.devs = t_limit, False, {}, {}
        self.sensor_defs, self.cam_ahead, self.radius = sensor_defs, cam_ahead, cfg["contact_radius"]
        self.collision_steps = 0

    # -- geometry
    def _aabbs(self):
        hw, hh = self.room[0] / 2, self.room[1] / 2
        out = [(o["x"], o["y"], o["sx"], o["sy"]) for o in self.boxes]
        out += [(0, hh + 0.5, self.room[0] + 2, 1.0), (0, -hh - 0.5, self.room[0] + 2, 1.0),
                (hw + 0.5, 0, 1.0, self.room[1] + 2), (-hw - 0.5, 0, 1.0, self.room[1] + 2)]
        return out

    def ray(self, rel_angle, rng):
        x, y, th = self.node.pos[0], self.node.pos[1], self.node.yaw
        a = th + rel_angle
        ox, oy = x + math.cos(th) * self.radius * 0.8, y + math.sin(th) * self.radius * 0.8
        dx, dy = math.cos(a), math.sin(a)
        best = rng
        for cx, cy, sx, sy in self._aabbs():
            tmin, tmax = 0.0, rng
            for o, d, lo, hi in ((ox, dx, cx - sx / 2, cx + sx / 2), (oy, dy, cy - sy / 2, cy + sy / 2)):
                if abs(d) < 1e-9:
                    if o < lo or o > hi:
                        tmin, tmax = 1, 0
                        break
                else:
                    t1, t2 = (lo - o) / d, (hi - o) / d
                    tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
            if tmin <= tmax:
                best = min(best, tmin)
        return 1000.0 * max(0.0, 1.0 - best / rng)

    def camera_image(self, w, h):
        x, y, th = self.node.pos[0], self.node.pos[1], self.node.yaw
        cx, cy = x + math.cos(th) * self.cam_ahead, y + math.sin(th) * self.cam_ahead
        rx, ry = math.sin(th), -math.cos(th)                       # robot right vector
        row = bytearray()
        for i in range(w):
            off = (i - (w - 1) / 2) / ((w - 1) / 2) * 0.082
            px, py = cx + rx * off, cy + ry * off
            dark = self.line and min(self._seg_dist(px, py, a, b) for a, b in zip(self.line, self.line[1:] + self.line[:1])) < 0.0225
            v = 8 if dark else 225
            row += bytes((v, v, v, 255))
        return bytes(row) * h

    @staticmethod
    def _seg_dist(px, py, a, b):
        ax, ay = a; bx, by = b
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
        return math.hypot(px - (ax + t * dx), py - (ay + t * dy))

    # -- devices
    def device(self, name):
        if name in self.devs:
            return self.devs[name]
        if name.endswith("motor"):
            d = self.motors[name] = Motor()
        elif name.endswith("sensor") and name[:-6] + "motor" in self.motors:
            d = PosSensor(self.motors[name[:-6] + "motor"])
        elif name in self.sensor_defs:
            ang, rng = self.sensor_defs[name]
            d = RaySensor(self, ang, rng)
        elif name == "line_camera":
            d = Camera(self)
        else:
            raise KeyError("no such device: " + name)
        self.devs[name] = d
        return d

    # -- stepping
    def advance(self):
        names = self.cfg["motors"]
        lv = [self.motors[n].velocity for n in names["left"]]
        rv = [self.motors[n].velocity for n in names["right"]]
        wl, wr = sum(lv) / len(lv), sum(rv) / len(rv)
        v, w = self.r * (wl + wr) / 2, self.r * (wr - wl) / self.axle
        th = self.node.yaw + w * self.dt
        nx, ny = self.node.pos[0] + v * math.cos(th) * self.dt, self.node.pos[1] + v * math.sin(th) * self.dt
        hit = any(abs(nx - cx) < sx / 2 + self.radius * 0.8 and abs(ny - cy) < sy / 2 + self.radius * 0.8
                  for cx, cy, sx, sy in [(o["x"], o["y"], o["sx"], o["sy"]) for o in self.boxes])
        hit = hit or abs(nx) > self.room[0] / 2 - self.radius * 0.5 or abs(ny) > self.room[1] / 2 - self.radius * 0.5
        if hit:
            self.collision_steps += 1
        else:
            self.node.pos[0], self.node.pos[1] = nx, ny
        self.node.yaw = th
        for m in self.motors.values():
            pass
        for n, m in self.motors.items():       # integrate wheel angles for position sensors
            m.position += m.velocity * self.dt
        self.t += self.dt


class Supervisor:
    SIMULATION_MODE_PAUSE = 0

    def __init__(self): self.sim = ACTIVE["sim"]
    def getBasicTimeStep(self): return 16
    def getName(self): return self.sim.name
    def getSelf(self): return self.sim.node
    def getTime(self): return self.sim.t
    def getDevice(self, n): return self.sim.device(n)
    def simulationSetMode(self, m): self.sim.paused = True
    def getFromDef(self, d): return getattr(self.sim, "defs", {}).get(d)
    def step(self, ms):
        self.sim.advance()
        return -1 if (self.sim.paused or self.sim.t > self.sim.t_limit) else 0


def install_fake_controller_module():
    mod = types.ModuleType("controller")
    mod.Supervisor, mod.Robot = Supervisor, Supervisor
    sys.modules["controller"] = mod


def run_controller_file(cdir: Path, sim) -> None:
    """Execute <cdir>/<name>.py as __main__ against the fake simulator (fresh module state each call)."""
    install_fake_controller_module()
    for m in ("agent_common",):
        sys.modules.pop(m, None)
    ACTIVE["sim"] = sim
    sys.path.insert(0, str(cdir))
    try:
        file = next(p for p in cdir.glob("*_controller.py"))
        runpy.run_path(str(file), run_name="__main__")
    finally:
        sys.path.remove(str(cdir))
        sys.modules.pop("agent_common", None)
