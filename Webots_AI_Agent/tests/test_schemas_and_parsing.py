import asyncio

import pytest
from pydantic import ValidationError

from app.agent.llm import RuleBasedParser
from app.schemas import RobotSpec, Telemetry, WorldSpec


def parse(text):
    return asyncio.run(RuleBasedParser().parse(text))


def test_defaults_fill_start_and_target():
    s = RobotSpec()
    assert s.start_position == (-0.7, -0.7) and s.target_position == (0.7, 0.7)
    assert 0 < s.max_speed <= 6.28


def test_example_spec_from_brief_is_valid():
    s = RobotSpec.model_validate({"robot_type": "differential_drive", "sensors": ["distance"],
                                  "task": "navigate_to_target",
                                  "world": {"width": 10, "height": 10, "obstacles": 5},
                                  "start_position": [0, 0], "target_position": [4, 4], "max_speed": 3.0})
    assert s.world.obstacles == 5


@pytest.mark.parametrize("bad", [
    {"max_speed": 50},
    {"max_speed": -1},
    {"sensors": ["lidar"]},
    {"robot_type": "drone"},
    {"task": "pick_and_place"},
    {"world": {"width": 100, "height": 2}},
    {"world": {"width": 2, "height": 2, "obstacles": 500}},
    {"target_position": [9, 9]},
    {"start_position": [0, 0], "target_position": [0.05, 0.05]},
    {"world": {"obstacle_positions": [[0.7, 0.7]]}},
    {"start_position": [float("nan"), 0]},
])
def test_invalid_specs_rejected(bad):
    with pytest.raises(ValidationError):
        RobotSpec.model_validate(bad)


def test_rules_parser_extracts_values():
    r = parse("Wheeled robot in a 4 by 4 meter room with 6 obstacles, start at (-1.5, -1.5), target at (1.5, 1.2), speed 3")
    s = r.spec
    assert (s.world.width, s.world.height, s.world.obstacles) == (4, 4, 6)
    assert s.start_position == (-1.5, -1.5) and s.target_position == (1.5, 1.2) and s.max_speed == 3


def test_rules_parser_clamps_speed_with_note():
    r = parse("robot in a 3m room, speed 99")
    assert r.spec.max_speed == 6.28 and r.notes


def test_rules_parser_waypoint_mode_and_no_obstacles():
    r = parse("robot in an empty room, use path planning")
    assert r.spec.planning_mode == "waypoints" and r.spec.world.obstacles == 0


@pytest.mark.parametrize("text", ["Build me a drone that flies through hoops", "make a humanoid robot walk",
                                  "robot arm that stacks cubes"])
def test_unsupported_robots_explained(text):
    r = parse(text)
    assert r.spec is None and "not supported" in r.clarification


def test_ambiguous_request_asks_for_clarification():
    r = parse("make something cool")
    assert r.spec is None and "robot" in r.clarification.lower()


def test_out_of_room_target_becomes_clarification():
    r = parse("robot in a 2 m room with target at (5, 5)")
    assert r.spec is None and "outside the room" in r.clarification


def test_telemetry_roundtrip_serialization():
    t = Telemetry(t=1.5, state="avoid", position=(0.1, -0.2), sensors={"ps0": 12.0},
                  sensor_summary={"left": 1, "front": 2, "right": 3}, motors=(1, 2))
    again = Telemetry.model_validate_json(t.model_dump_json())
    assert again == t and again.position == (0.1, -0.2)


def test_worldspec_expands_obstacle_count_for_explicit_positions():
    w = WorldSpec(obstacles=1, obstacle_positions=[(0, 0), (0.5, 0.5)])
    assert w.obstacles == 2
