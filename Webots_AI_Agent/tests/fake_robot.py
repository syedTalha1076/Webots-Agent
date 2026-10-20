"""Kinematic stand-in for the e-puck used to unit-test the navigation logic WITHOUT Webots.

It is a simplification (point IR rays, no wheel slip, blocked-move collisions) - it checks the algorithm,
it does not replace running the real Webots simulation.
"""
from __future__ import annotations

import math

from app.simulation.navigation import NavParams, Navigator, Odometry, summarize_sensors

R, AXLE, RADIUS, DT = 0.0205, 0.052, 0.037, 0.032
SENSOR_ANGLES = [-0.30, -0.80, -1.57, -2.64, 2.64, 1.57, 0.80, 0.30]   # ps0..ps7 (right = negative)
LOOKUP = [(0.0, 4095), (0.005, 2133), (0.01, 1466), (0.015, 601), (0.02, 384), (0.03, 235),
          (0.04, 158), (0.05, 120), (0.06, 104), (0.07, 67)]


def ir_value(d: float) -> float:
    if d > 0.07:
        return 0.0
    for (d0, v0), (d1, v1) in zip(LOOKUP, LOOKUP[1:]):
        if d0 <= d <= d1:
            return v0 + (v1 - v0) * (d - d0) / (d1 - d0)
    return 0.0


class FakeEpuck:
    def __init__(self, start, heading, obstacles, width, height):
        self.x, self.y, self.th = start[0], start[1], heading
        self.obstacles, self.w, self.h = obstacles, width, height
        self.wl = self.wr = 0.0            # accumulated wheel angles
        self.t = 0.0
        self.travelled = 0.0
        self.collisions = 0

    def solid(self, x, y, margin=0.0) -> bool:
        if abs(x) > self.w / 2 - 0.005 - margin or abs(y) > self.h / 2 - 0.005 - margin:
            return True
        if margin == 0.0:
            return any(abs(x - o["x"]) <= o["size"] / 2 and abs(y - o["y"]) <= o["size"] / 2 for o in self.obstacles)
        # circle (robot body) vs axis-aligned box
        return any(math.hypot(max(abs(x - o["x"]) - o["size"] / 2, 0.0), max(abs(y - o["y"]) - o["size"] / 2, 0.0)) < margin
                   for o in self.obstacles)

    def sensors(self) -> list[float]:
        out = []
        for a in SENSOR_ANGLES:
            ang = self.th + a
            v, d = 0.0, 0.0
            while d <= 0.07:
                px = self.x + (RADIUS + d) * math.cos(ang)
                py = self.y + (RADIUS + d) * math.sin(ang)
                if self.solid(px, py):
                    v = ir_value(d)
                    break
                d += 0.002
            out.append(v)
        return out

    def step(self, cl: float, cr: float) -> None:
        self.wl += cl * DT
        self.wr += cr * DT
        v, w = R * (cl + cr) / 2, R * (cr - cl) / AXLE
        nx = self.x + v * DT * math.cos(self.th + w * DT / 2)
        ny = self.y + v * DT * math.sin(self.th + w * DT / 2)
        if self.solid(nx, ny, RADIUS):
            self.collisions += 1 if v * DT > 0 else 0
        else:
            self.travelled += math.hypot(nx - self.x, ny - self.y)
            self.x, self.y = nx, ny
        self.th = (self.th + w * DT + math.pi) % (2 * math.pi) - math.pi
        self.t += DT


def run_navigation(start, target, obstacles, width, height, waypoints=None, max_speed=4.0,
                   max_time=240.0, heading=None):
    heading = math.atan2(target[1] - start[1], target[0] - start[0]) if heading is None else heading
    robot = FakeEpuck(start, heading, obstacles, width, height)
    odo = Odometry(start[0], start[1], heading, R, AXLE)
    nav = Navigator(waypoints or [tuple(target)], NavParams(max_speed=max_speed))
    log: list[str] = []
    while robot.t < max_time and nav.state not in ("arrived", "stuck"):
        pose = odo.update(robot.wl, robot.wr)
        cmd = nav.update(pose, summarize_sensors(robot.sensors()), robot.t)
        log += nav.pop_events()
        robot.step(*cmd)
    return robot, nav, log
