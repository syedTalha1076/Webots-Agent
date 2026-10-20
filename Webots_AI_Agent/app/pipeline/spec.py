"""Structured simulation specification + validation (the ONLY thing a parser/LLM is allowed to produce).

Everything generated later (world, PROTO, controller config) is derived from a validated SimSpec.
`validate_spec` returns hard errors (nothing is generated) and warnings (generated, but user is told).
"""
from __future__ import annotations

import math
from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator, model_validator

RobotKind = Literal["epuck", "custom_wheeled", "arm", "drone"]
Task = Literal["wander_avoid", "go_to_targets", "line_follow", "pick_and_place", "fly_waypoints"]
EnvKind = Literal["arena", "warehouse", "line_track", "arm_table", "open_field"]

# What each robot kind can actually do in this generator (anything else is rejected with an explanation).
CAPABILITIES: dict[str, dict] = {
    "epuck": {"tasks": {"wander_avoid", "go_to_targets"}, "envs": {"arena", "warehouse"},
              "motors": "2 wheel motors (built-in E-puck PROTO)", "sensors": ["distance (8 IR: ps0-ps7)"],
              "max_count": 8},
    "custom_wheeled": {"tasks": {"wander_avoid", "go_to_targets", "line_follow"},
                       "envs": {"arena", "warehouse", "line_track"},
                       "motors": "2 or 4 wheel motors (generated Robot)", "sensors": ["distance (3 front)", "camera (line)"],
                       "max_count": 8},
    "arm": {"tasks": {"pick_and_place"}, "envs": {"arm_table"},
            "motors": "3 rotational motors (yaw, shoulder, elbow) + magnetic grasp", "sensors": ["joint position sensors"],
            "max_count": 1},
    "drone": {"tasks": {"fly_waypoints"}, "envs": {"open_field"},
              "motors": "4 propeller motors (built-in Mavic2Pro PROTO)", "sensors": ["gps", "inertial unit", "gyro"],
              "max_count": 1},
}
UNSUPPORTED_HINT = ("Supported: e-puck / custom 2- or 4-wheel robots (wander+avoid, go to targets, warehouse transport, "
                    "multi-robot, line following), a 3-joint robotic arm (pick and place a cube), and a Mavic 2 Pro "
                    "drone flying waypoints. Not supported: humanoids, legged robots, underwater/ground-steering cars, "
                    "multi-arm cells, SLAM/vision tasks.")

XY = tuple[float, float]


class RobotCfg(BaseModel):
    kind: RobotKind = "epuck"
    count: int = Field(1, ge=1, le=8)
    drive: Literal["differential", "four_wheel"] = "differential"        # custom_wheeled only
    body_length: float = Field(0.25, ge=0.08, le=1.2)                  # m (custom_wheeled)
    body_width: float = Field(0.16, ge=0.06, le=1.0)
    body_height: float = Field(0.06, ge=0.03, le=0.5)
    wheel_radius: float = Field(0.04, ge=0.015, le=0.25)
    max_speed: float = Field(6.0, gt=0.2, le=20.0, description="wheel speed rad/s (e-puck hardware limit 6.28)")
    start_positions: list[XY] = Field(default_factory=list, max_length=8)

    @field_validator("start_positions")
    @classmethod
    def _finite(cls, v):
        for p in v:
            if not all(map(math.isfinite, p)):
                raise ValueError("start positions must be finite")
        return v


class EnvCfg(BaseModel):
    kind: EnvKind = "arena"
    width: float = Field(3.0, ge=1.0, le=30.0)
    height: float = Field(3.0, ge=1.0, le=30.0)
    obstacles: int = Field(0, ge=0, le=40)
    obstacle_positions: list[XY] = Field(default_factory=list, max_length=40)
    shelves: int = Field(0, ge=0, le=12)                                  # warehouse
    track_shape: Literal["oval", "square"] = "oval"                       # line_track


class BehaviorCfg(BaseModel):
    task: Task = "go_to_targets"
    targets: list[XY] = Field(default_factory=list, max_length=12)       # ordered visit list / waypoints (x, y)
    altitude: float = Field(1.0, ge=0.3, le=5.0)                          # drone
    laps: int = Field(1, ge=1, le=5)                                      # line follow
    duration_s: float = Field(60.0, ge=10.0, le=1800.0)                   # wander
    pick_position: Optional[XY] = None                                    # arm: offsets from arm base (m)
    place_position: Optional[XY] = None
    loop_targets: bool = False


class SimCfg(BaseModel):
    time_step_ms: int = Field(16, ge=4, le=64)
    timeout_s: float = Field(180.0, ge=10.0, le=3600.0)
    mode: Literal["realtime", "fast"] = "realtime"


class SimSpec(BaseModel):
    title: str = Field("Generated simulation", max_length=80)
    prompt: str = ""
    robot: RobotCfg = Field(default_factory=RobotCfg)
    environment: EnvCfg = Field(default_factory=EnvCfg)
    behavior: BehaviorCfg = Field(default_factory=BehaviorCfg)
    sim: SimCfg = Field(default_factory=SimCfg)

    @model_validator(mode="after")
    def _timeouts(self) -> "SimSpec":
        # the wall-clock watcher uses sim.timeout_s: a long wander run must not be declared a timeout
        if self.behavior.task == "wander_avoid":
            self.sim.timeout_s = min(3600.0, max(self.sim.timeout_s, self.behavior.duration_s + 30))
        return self


class ValidationReport(BaseModel):
    errors: list[str] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


WALL_MARGIN = 0.2
ARM = {"l1": 0.30, "l2": 0.28, "h0": 0.12, "table_h": 0.40, "cube": 0.05, "min_r": 0.18, "max_r": 0.50}


def derived(spec: SimSpec) -> dict:
    """Facts derived from the spec (shown to the user as the 'robot specification')."""
    r = spec.robot
    cap = CAPABILITIES[r.kind]
    d = {"robot_type": r.kind, "count": r.count, "motors": cap["motors"], "sensors": cap["sensors"]}
    if r.kind == "custom_wheeled":
        d["dimensions_m"] = {"length": r.body_length, "width": r.body_width, "height": r.body_height,
                             "wheel_radius": r.wheel_radius}
        d["motor_count"] = 4 if r.drive == "four_wheel" else 2
    elif r.kind == "epuck":
        d["dimensions_m"] = {"diameter": 0.074, "note": "fixed by the built-in E-puck model"}
        d["motor_count"] = 2
    elif r.kind == "arm":
        d["dimensions_m"] = {"link1": ARM["l1"], "link2": ARM["l2"], "reach": round(ARM["l1"] + ARM["l2"], 2)}
        d["motor_count"] = 3
    else:
        d["dimensions_m"] = {"note": "fixed by the built-in Mavic 2 Pro model"}
        d["motor_count"] = 4
    return d


def validate_spec(spec: SimSpec) -> ValidationReport:
    rep = ValidationReport()
    r, e, b = spec.robot, spec.environment, spec.behavior
    cap = CAPABILITIES[r.kind]
    if b.task not in cap["tasks"]:
        rep.errors.append(f"A {r.kind} robot cannot do '{b.task}' here (it supports: {', '.join(sorted(cap['tasks']))}).")
    if e.kind not in cap["envs"]:
        rep.errors.append(f"A {r.kind} robot cannot be placed in a '{e.kind}' environment "
                          f"(supported: {', '.join(sorted(cap['envs']))}).")
    if r.count > cap["max_count"]:
        rep.errors.append(f"At most {cap['max_count']} {r.kind} robot(s) are supported (requested {r.count}).")
    if rep.errors:
        return rep

    hw, hh = e.width / 2 - WALL_MARGIN, e.height / 2 - WALL_MARGIN

    def inside(p: XY) -> bool:
        return abs(p[0]) <= hw and abs(p[1]) <= hh

    if r.kind in ("epuck", "custom_wheeled"):
        if r.kind == "epuck" and r.max_speed > 6.28:
            rep.warnings.append("E-puck wheel speed limited to 6.28 rad/s.")
        if r.kind == "custom_wheeled" and r.wheel_radius * 2 > r.body_height * 4:
            rep.errors.append("Wheel diameter is unrealistically large for this body height.")
        if r.kind == "custom_wheeled" and r.body_length > 0.45:
            rep.warnings.append("Bodies longer than 0.45 m turn wide and clip obstacles more often; use a larger room / fewer obstacles.")
        if r.kind == "custom_wheeled" and r.body_width < 2 * 0.012 + 0.02:
            rep.errors.append("Body too narrow for wheels.")
        if r.kind == "epuck" and any(v != d for v, d in ((r.body_length, 0.25), (r.body_width, 0.16))):
            rep.warnings.append("Dimensions are fixed for the built-in E-puck; use a 'custom' robot for custom sizes.")
        for p in r.start_positions:
            if not inside(p):
                rep.errors.append(f"Start position {list(p)} is outside the room (|x|<={hw:.2f}, |y|<={hh:.2f}).")
        if len(r.start_positions) not in (0, r.count) and len(r.start_positions) != 1:
            rep.warnings.append("Number of start positions differs from robot count; the rest are laid out automatically.")
    if b.task in ("go_to_targets",):
        if not b.targets:
            rep.warnings.append("No target given: a default target on the far side of the room is used.")
        for t in b.targets:
            if not inside(t):
                rep.errors.append(f"Target {list(t)} is outside the room (|x|<={hw:.2f}, |y|<={hh:.2f}).")
    if b.task == "go_to_targets" and r.count > 1 and len(b.targets) > r.count:
        rep.warnings.append("More targets than robots: robots share the targets in order, each visiting its own list.")
    for p in e.obstacle_positions:
        if not inside(p):
            rep.errors.append(f"Obstacle position {list(p)} is outside the room.")
        for q in list(r.start_positions) + list(b.targets):
            if math.dist(p, q) < 0.2:
                rep.errors.append(f"Obstacle at {list(p)} blocks a start/target at {list(q)}.")
    area = max(e.obstacles, len(e.obstacle_positions)) * 0.12 ** 2 + e.shelves * 0.8 * 0.3
    if area > 0.25 * e.width * e.height:
        rep.errors.append("Obstacles/shelves would cover more than 25% of the floor; reduce their number or enlarge the room.")
    if e.kind == "warehouse" and e.shelves == 0 and b.task == "go_to_targets":
        rep.warnings.append("Warehouse without shelves requested; default shelf rows are added.")
    if e.kind == "warehouse" and min(e.width, e.height) < 4:
        rep.errors.append("A warehouse needs at least 4 m x 4 m.")
    if b.task == "line_follow":
        if min(e.width, e.height) < 2:
            rep.errors.append("Line track needs at least a 2 m x 2 m floor.")
        if r.count != 1:
            rep.errors.append("Line following supports exactly one robot.")
        if e.obstacles or e.obstacle_positions:
            rep.warnings.append("Obstacles are ignored on a line track.")
    if b.task == "pick_and_place":
        for name, p in (("pick", b.pick_position), ("place", b.place_position)):
            if p is None:
                continue
            rr = math.hypot(*p)
            if not (ARM["min_r"] <= rr <= ARM["max_r"]):
                rep.errors.append(f"The {name} position {list(p)} is {rr:.2f} m from the arm base; the arm reaches "
                                  f"{ARM['min_r']}-{ARM['max_r']} m. Move it closer/further.")
        if b.pick_position and b.place_position and math.dist(b.pick_position, b.place_position) < 0.12:
            rep.errors.append("Pick and place positions are too close together (min 0.12 m).")
    if b.task == "fly_waypoints":
        if not b.targets:
            rep.warnings.append("No waypoints given: a square pattern is flown.")
        if any(not (abs(t[0]) <= e.width / 2 - 0.5 and abs(t[1]) <= e.height / 2 - 0.5) for t in b.targets):
            rep.errors.append("Waypoint outside the flight area.")
        if e.width < 4 or e.height < 4:
            rep.errors.append("Drone flights need at least a 4 m x 4 m area.")
        if b.altitude > 3.0:
            rep.warnings.append("High altitude: the default camera view may lose the drone.")
    if spec.sim.timeout_s < 20:
        rep.warnings.append("Very short timeout.")
    return rep
