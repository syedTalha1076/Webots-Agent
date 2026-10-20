import math

import pytest

from app.schemas import RobotSpec, WorldSpec
from app.simulation.navigation import Navigator, NavParams, Odometry, plan_path, summarize_sensors, wrap
from app.simulation.world_generator import build_plan_waypoints, layout_obstacles
from tests.fake_robot import run_navigation


def test_odometry_straight_line():
    o = Odometry(0, 0, 0, 0.02, 0.05)
    o.update(0, 0)
    x, y, th = o.update(5.0, 5.0)   # both wheels 5 rad -> 0.1 m
    assert x == pytest.approx(0.1) and y == pytest.approx(0, abs=1e-9) and th == pytest.approx(0)


def test_odometry_turn_in_place():
    o = Odometry(0, 0, 0, 0.02, 0.05)
    o.update(0, 0)
    _, _, th = o.update(-1.0, 1.0)  # dth = (0.02+0.02)/0.05 = 0.8 rad
    assert th == pytest.approx(0.8)


def test_wrap():
    assert wrap(math.pi * 3) == pytest.approx(-math.pi) or wrap(math.pi * 3) == pytest.approx(math.pi)


def test_plan_path_goes_around_wall_of_boxes():
    obs = [{"x": 0, "y": y / 10, "size": 0.1} for y in range(-6, 7)]   # wall from y=-0.6..0.6, gap above/below
    path = plan_path((-0.7, 0), (0.7, 0), obs, 2.0, 2.0)
    assert path is not None and len(path) >= 2
    assert max(abs(p[1]) for p in path) > 0.7       # detours around the end of the wall


def test_plan_path_returns_none_when_blocked():
    obs = [{"x": 0, "y": y / 10, "size": 0.12} for y in range(-10, 11)]
    assert plan_path((-0.7, 0), (0.7, 0), obs, 2.0, 2.0) is None


def test_plan_path_direct_when_free():
    assert plan_path((-0.7, -0.7), (0.7, 0.7), [], 2, 2) == [(0.7, 0.7)]


def test_sensor_summary_mapping():
    s = summarize_sensors([5, 4, 3, 2, 1, 7, 6, 8])
    assert s == {"front": 8, "front_right": 4, "front_left": 6, "right": 3, "left": 7}


def test_navigator_turns_toward_goal_and_stops_on_arrival():
    n = Navigator([(1.0, 1.0)], NavParams(max_speed=4))
    clear = {"front": 0, "front_left": 0, "front_right": 0, "left": 0, "right": 0}
    l, r = n.update((0, 0, 0), clear, 0.0)          # goal is to the left -> right wheel faster
    assert r > l
    l, r = n.update((0.99, 0.99, 0), clear, 1.0)
    assert n.state == "arrived" and (l, r) == (0, 0)


def test_navigator_avoids_when_blocked_and_reports_event():
    n = Navigator([(1.0, 0.0)], NavParams(max_speed=4))
    blocked = {"front": 900, "front_left": 400, "front_right": 100, "left": 0, "right": 0}
    l, r = n.update((0, 0, 0), blocked, 0.0)
    assert n.state == "avoid" and n.encounters == 1
    assert l > r or r > l                            # rotating, not driving straight into the obstacle
    assert any("obstacle detected" in e for e in n.pop_events())


def test_navigator_declares_stuck_then_replan_recovers():
    n = Navigator([(1.0, 0.0)], NavParams(max_speed=4, stuck_window=5))
    clear = {"front": 0, "front_left": 0, "front_right": 0, "left": 0, "right": 0}
    t = 0.0
    while n.state != "stuck" and t < 20:
        n.update((0, 0, 0), clear, t)               # robot never moves
        t += 0.1
    assert n.state == "stuck" and n.stuck_reason
    n.replan()
    assert n.state == "go_to_goal" and n.replans == 1


def test_open_room_reaches_target():
    r, nav, _ = run_navigation((-0.7, -0.7), (0.7, 0.7), [], 2, 2)
    assert nav.state == "arrived" and math.dist((r.x, r.y), (0.7, 0.7)) < 0.15 and r.collisions == 0


@pytest.mark.parametrize("mode", ["reactive", "waypoints"])
@pytest.mark.parametrize("seed", range(6))
def test_obstacle_room_reaches_target(mode, seed):
    spec = RobotSpec(world=WorldSpec(width=3, height=3, obstacles=6), planning_mode=mode)
    obs = layout_obstacles(spec, f"t{seed}")
    wps, _ = build_plan_waypoints(spec, obs)
    r, nav, _ = run_navigation(spec.start_position, spec.target_position, obs, 3, 3, wps, max_time=300)
    assert nav.state == "arrived", f"{mode}/{seed} ended {nav.state}"
    assert math.dist((r.x, r.y), spec.target_position) < 0.15


def test_reactive_mode_really_uses_sensors_to_avoid_obstacle():
    obs = [{"x": 0.0, "y": 0.0, "size": 0.14}]      # directly on the straight line
    r, nav, log = run_navigation((-0.7, 0.0), (0.7, 0.0), obs, 2, 2)
    assert nav.state == "arrived" and nav.encounters >= 1 and r.collisions == 0
    assert any("obstacle detected" in e for e in log)


def test_layout_is_deterministic_and_clear_of_start_and_target():
    spec = RobotSpec(world=WorldSpec(width=3, height=3, obstacles=8))
    a, b = layout_obstacles(spec, "same"), layout_obstacles(spec, "same")
    assert a == b and layout_obstacles(spec, "other") != a
    for o in a:
        assert math.dist((o["x"], o["y"]), spec.start_position) >= 0.3
        assert math.dist((o["x"], o["y"]), spec.target_position) >= 0.3


def test_layout_rejects_overcrowded_room():
    spec = RobotSpec(world=WorldSpec(width=1, height=1, obstacles=20))
    with pytest.raises(ValueError):
        layout_obstacles(spec, "x")
