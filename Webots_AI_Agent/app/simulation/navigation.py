"""Navigation logic: odometry, grid path planning and a reactive go-to-goal/obstacle-avoidance navigator.

IMPORTANT: this module uses ONLY the Python standard library. It is imported both by the backend
(planning, tests) and by the Webots controller (which runs in Webots' own Python process and is
copied next to the controller). It never calls an LLM and never touches the simulator itself, so it is
safe to run at every control step.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

Vec = tuple[float, float]


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


# ----------------------------------------------------------------------------- odometry
class Odometry:
    """Differential-drive dead reckoning from wheel encoder angles (a real on-board sensor)."""

    def __init__(self, x: float, y: float, theta: float, wheel_radius: float, axle: float):
        self.x, self.y, self.theta = x, y, theta
        self.r, self.axle = wheel_radius, axle
        self._last: tuple[float, float] | None = None

    def update(self, left_angle: float, right_angle: float) -> tuple[float, float, float]:
        if self._last is not None and all(map(math.isfinite, (left_angle, right_angle))):
            dl = (left_angle - self._last[0]) * self.r
            dr = (right_angle - self._last[1]) * self.r
            ds, dth = (dl + dr) / 2, (dr - dl) / self.axle
            self.x += ds * math.cos(self.theta + dth / 2)
            self.y += ds * math.sin(self.theta + dth / 2)
            self.theta = wrap(self.theta + dth)
        if all(map(math.isfinite, (left_angle, right_angle))):
            self._last = (left_angle, right_angle)
        return self.x, self.y, self.theta

    def reset(self, x: float, y: float, theta: float) -> None:
        self.x, self.y, self.theta, self._last = x, y, theta, None


# ----------------------------------------------------------------------------- sensors
def summarize_sensors(ps: list[float]) -> dict[str, float]:
    """E-puck IR layout: ps0 front-right, ps1 right-45, ps2 right, ps5 left, ps6 left-45, ps7 front-left.
    Raw values grow as obstacles get closer (~4095 touching, ~100 at 5 cm, ~0 beyond 7 cm)."""
    ps = list(ps) + [0.0] * (8 - len(ps))
    return {
        "front": max(ps[0], ps[7]),
        "front_right": ps[1],
        "front_left": ps[6],
        "right": ps[2],
        "left": ps[5],
    }


# ----------------------------------------------------------------------------- planning
def plan_path(start: Vec, goal: Vec, obstacles: list[dict], width: float, height: float,
              inflate: float = 0.08, res: float = 0.05) -> list[Vec] | None:
    """A* on an occupancy grid built from the known (a-priori) layout. Returns waypoints excluding start,
    or None when no collision-free path exists. Obstacles: dicts with x, y, size (axis-aligned squares)."""
    nx, ny = int(width / res) + 1, int(height / res) + 1
    ox, oy = -width / 2, -height / 2

    def cell(p: Vec) -> tuple[int, int]:
        return clamp(int(round((p[0] - ox) / res)), 0, nx - 1), clamp(int(round((p[1] - oy) / res)), 0, ny - 1)

    def centre(c: tuple[int, int]) -> Vec:
        return ox + c[0] * res, oy + c[1] * res

    def blocked_point(p: Vec) -> bool:
        if abs(p[0]) > width / 2 - inflate or abs(p[1]) > height / 2 - inflate:
            return True
        return any(abs(p[0] - o["x"]) <= o["size"] / 2 + inflate and abs(p[1] - o["y"]) <= o["size"] / 2 + inflate
                   for o in obstacles)

    s, g = cell(start), cell(goal)
    blocked = {(i, j) for i in range(nx) for j in range(ny) if blocked_point(centre((i, j)))}
    blocked.discard(s)
    blocked.discard(g)
    openq: list[tuple[float, tuple[int, int]]] = [(0.0, s)]
    came: dict[tuple[int, int], tuple[int, int]] = {}
    cost = {s: 0.0}
    while openq:
        _, cur = heapq.heappop(openq)
        if cur == g:
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if not (di or dj):
                    continue
                nxt = (cur[0] + di, cur[1] + dj)
                if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny) or nxt in blocked:
                    continue
                if di and dj and ((cur[0] + di, cur[1]) in blocked or (cur[0], cur[1] + dj) in blocked):
                    continue  # no corner cutting
                c = cost[cur] + math.hypot(di, dj)
                if c < cost.get(nxt, 1e18):
                    cost[nxt], came[nxt] = c, cur
                    heapq.heappush(openq, (c + math.hypot(g[0] - nxt[0], g[1] - nxt[1]), nxt))
    if g not in came and s != g:
        return None
    path = [g]
    while path[-1] != s:
        path.append(came[path[-1]])
    pts = [start] + [centre(c) for c in reversed(path[1:-1])] + [goal]

    def line_free(a: Vec, b: Vec) -> bool:
        n = max(2, int(math.dist(a, b) / (res / 2)))
        return not any(blocked_point((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)) for k in range(1, n))

    smooth, i = [], 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not line_free(pts[i], pts[j]):
            j -= 1
        smooth.append(pts[j])
        i = j
    return smooth


# ----------------------------------------------------------------------------- navigator
@dataclass
class NavParams:
    max_speed: float = 4.0          # wheel rad/s
    goal_tol: float = 0.05          # m (odometry based)
    wp_tol: float = 0.08
    obstacle_on: float = 180.0      # raw IR value that triggers avoidance (~3 cm)
    obstacle_off: float = 90.0
    wall_target: float = 150.0
    stuck_window: float = 40.0      # sim seconds without progress -> stuck
    stuck_progress: float = 0.05    # m of progress required per window
    avoid_timeout: float = 25.0


@dataclass
class Navigator:
    waypoints: list[Vec]
    params: NavParams = field(default_factory=NavParams)
    state: str = "go_to_goal"       # go_to_goal | avoid | arrived | stuck
    wp_index: int = 0
    encounters: int = 0
    replans: int = 0
    stuck_reason: str = ""

    def __post_init__(self) -> None:
        self.wall_side = "auto"
        self.forced_side: str | None = None
        self.last_side = "right"
        self.hit_dist = math.inf
        self.avoid_since = 0.0
        self.best = math.inf
        self.best_t = 0.0
        self.maneuver_until = 0.0
        self._t = 0.0
        self.events: list[str] = []

    # ---- helpers
    @property
    def goal(self) -> Vec:
        return self.waypoints[self.wp_index]

    def _remaining(self, pos: Vec) -> float:
        d = math.dist(pos, self.goal)
        for a, b in zip(self.waypoints[self.wp_index:], self.waypoints[self.wp_index + 1:]):
            d += math.dist(a, b)
        return d

    def pop_events(self) -> list[str]:
        ev, self.events = self.events, []
        return ev

    def replan(self, speed_scale: float = 1.0) -> None:
        """Called after the backend agent decides to replan: back off, then try the other side."""
        self.replans += 1
        self.forced_side = "left" if self.last_side == "right" else "right"
        self.state = "go_to_goal"
        self.maneuver_until = self._t + 1.5
        self.best, self.best_t = math.inf, self._t
        self.params.max_speed = clamp(self.params.max_speed * speed_scale, 0.5, 6.28)
        self.events.append(f"replan #{self.replans}: backing off, passing obstacles on the {self.forced_side} side")

    # ---- main step
    def update(self, pose: tuple[float, float, float], s: dict[str, float], t: float) -> tuple[float, float]:
        p = self.params
        self._t = t
        x, y, th = pose
        if self.state in ("arrived", "stuck"):
            return 0.0, 0.0
        gx, gy = self.goal
        dist = math.hypot(gx - x, gy - y)
        last = self.wp_index == len(self.waypoints) - 1
        if dist <= (p.goal_tol if last else p.wp_tol):
            if last:
                self.state = "arrived"
                self.events.append("target reached (odometry)")
                return 0.0, 0.0
            self.wp_index += 1
            self.best, self.best_t, self.state = math.inf, t, "go_to_goal"
            self.events.append(f"waypoint {self.wp_index}/{len(self.waypoints)} reached")
            return self.update(pose, s, t)

        rem = self._remaining((x, y))
        if rem < self.best - p.stuck_progress:
            self.best, self.best_t = rem, t
        elif t - self.best_t > p.stuck_window:
            self.state, self.stuck_reason = "stuck", f"no progress for {p.stuck_window:.0f}s"
            self.events.append("stuck: " + self.stuck_reason)
            return 0.0, 0.0

        if t < self.maneuver_until:      # back away from whatever we were stuck against
            return -0.5 * p.max_speed, -0.25 * p.max_speed

        err = wrap(math.atan2(gy - y, gx - x) - th)
        front_obs = max(s["front"], s["front_left"], s["front_right"])
        any_obs = max(front_obs, s["left"], s["right"])

        if self.state == "go_to_goal" and front_obs > p.obstacle_on:
            self._enter_avoid(err, dist, s, t)

        if self.state == "avoid":
            timed_out = t - self.avoid_since > p.avoid_timeout and abs(err) < 1.2
            if any_obs < p.obstacle_off and ((dist < self.hit_dist - 0.03 and abs(err) < 0.7) or timed_out):
                self.state = "go_to_goal"
                self.events.append("path clear, resuming go-to-goal")
            else:
                return self._avoid(s, front_obs)
        return self._go(err, dist, s, any_obs, last)

    def _enter_avoid(self, err: float, dist: float, s: dict[str, float], t: float) -> None:
        self.state, self.hit_dist, self.avoid_since = "avoid", dist, t
        self.encounters += 1
        if self.forced_side:
            side = self.forced_side
        else:
            lw, rw = s["left"] + s["front_left"], s["right"] + s["front_right"]
            side = "left" if lw > rw * 1.5 else "right" if rw > lw * 1.5 else ("right" if err > 0 else "left")
        self.wall_side = self.last_side = side
        self.events.append(f"obstacle detected, following it with wall on the {side}")

    def _go(self, err: float, dist: float, s: dict[str, float], any_obs: float, last: bool) -> tuple[float, float]:
        p = self.params
        vmax = p.max_speed
        forward = vmax * max(0.0, 1 - abs(err) / 0.9)
        turn = clamp(3.5 * err, -0.8 * vmax, 0.8 * vmax) - 0.003 * (s["left"] - s["right"])
        if last and dist < 0.2:
            forward *= max(0.25, dist / 0.2)
        if any_obs > p.obstacle_off:
            forward *= 0.6
        return clamp(forward - turn, -vmax, vmax), clamp(forward + turn, -vmax, vmax)

    def _avoid(self, s: dict[str, float], front_obs: float) -> tuple[float, float]:
        p = self.params
        vmax = p.max_speed
        left_wall = self.wall_side == "left"
        away = -1.0 if left_wall else 1.0            # +1 = turn left
        if front_obs > p.obstacle_on * 0.8:          # blocked ahead: rotate away from the wall
            w = 0.5 * vmax
            return -away * w, away * w
        v = 0.5 * vmax
        side_val = s["left"] if left_wall else s["right"]
        if side_val < 40:                            # lost the wall (corner): curve back around it
            turn = -away * 0.35 * vmax
        else:
            turn = away * clamp((side_val - p.wall_target) / p.wall_target, -1, 1) * 0.3 * vmax
        return clamp(v - turn, -vmax, vmax), clamp(v + turn, -vmax, vmax)
