"""Pydantic schemas: robot specification, API payloads and telemetry."""
from __future__ import annotations

import math
from enum import Enum
from typing import Literal, Optional

from pydantic import BaseModel, Field, field_validator, model_validator

WALL_MARGIN = 0.15          # m, keep start/target/obstacles this far from walls
MIN_START_TARGET_DIST = 0.3  # m
MAX_ROOM = 10.0
MAX_OBSTACLES = 20
MAX_WHEEL_SPEED = 6.28


class RobotType(str, Enum):
    differential_drive = "differential_drive"
    epuck = "epuck"


class WorldSpec(BaseModel):
    width: float = Field(2.0, ge=1.0, le=MAX_ROOM, description="Room size along X in metres")
    height: float = Field(2.0, ge=1.0, le=MAX_ROOM, description="Room size along Y in metres")
    obstacles: int = Field(3, ge=0, le=MAX_OBSTACLES)
    obstacle_positions: list[tuple[float, float]] = Field(default_factory=list, max_length=MAX_OBSTACLES)

    @model_validator(mode="after")
    def _enough_count(self) -> "WorldSpec":
        if len(self.obstacle_positions) > self.obstacles:
            self.obstacles = len(self.obstacle_positions)
        return self


class RobotSpec(BaseModel):
    """Validated specification. The LLM may only produce this structure - never code."""

    robot_type: RobotType = RobotType.differential_drive
    sensors: list[Literal["distance"]] = Field(default_factory=lambda: ["distance"])
    task: Literal["navigate_to_target"] = "navigate_to_target"
    world: WorldSpec = Field(default_factory=WorldSpec)
    start_position: Optional[tuple[float, float]] = None
    start_heading: Optional[float] = Field(None, description="radians, None = face the target")
    target_position: Optional[tuple[float, float]] = None
    max_speed: float = Field(4.0, gt=0.2, le=MAX_WHEEL_SPEED, description="wheel speed in rad/s")
    planning_mode: Literal["reactive", "waypoints"] = "reactive"

    @field_validator("sensors")
    @classmethod
    def _dedupe(cls, v: list[str]) -> list[str]:
        return sorted(set(v)) or ["distance"]

    @model_validator(mode="after")
    def _defaults_and_bounds(self) -> "RobotSpec":
        w, h = self.world.width, self.world.height
        if self.start_position is None:
            self.start_position = (-w / 2 + 0.3, -h / 2 + 0.3)
        if self.target_position is None:
            self.target_position = (w / 2 - 0.3, h / 2 - 0.3)
        for name, p in (("start_position", self.start_position), ("target_position", self.target_position)):
            if not all(map(math.isfinite, p)):
                raise ValueError(f"{name} must be finite numbers")
            if abs(p[0]) > w / 2 - WALL_MARGIN or abs(p[1]) > h / 2 - WALL_MARGIN:
                raise ValueError(
                    f"{name} {list(p)} is outside the room "
                    f"(x in ±{w / 2 - WALL_MARGIN:.2f}, y in ±{h / 2 - WALL_MARGIN:.2f})"
                )
        if math.dist(self.start_position, self.target_position) < MIN_START_TARGET_DIST:
            raise ValueError(f"start and target must be at least {MIN_START_TARGET_DIST} m apart")
        for p in self.world.obstacle_positions:
            if abs(p[0]) > w / 2 - WALL_MARGIN or abs(p[1]) > h / 2 - WALL_MARGIN:
                raise ValueError(f"obstacle position {list(p)} is outside the room")
            if min(math.dist(p, self.start_position), math.dist(p, self.target_position)) < 0.2:
                raise ValueError(f"obstacle position {list(p)} blocks the start or target")
        if self.start_heading is None:
            dx = self.target_position[0] - self.start_position[0]
            dy = self.target_position[1] - self.start_position[1]
            self.start_heading = math.atan2(dy, dx)
        return self


class SimulationPlan(BaseModel):
    robot_model: str
    controller: str
    objective: str
    navigation: str
    max_wheel_speed: float
    waypoints: list[tuple[float, float]] = Field(default_factory=list)
    timeout_s: int
    steps: list[str] = Field(default_factory=list)


class CreateSimulationRequest(BaseModel):
    prompt: str = Field(..., min_length=3, max_length=2000)
    auto_start: bool = False


class SimStatus(str, Enum):
    planning = "planning"
    needs_clarification = "needs_clarification"
    ready = "ready"
    starting = "starting"
    running = "running"
    paused = "paused"
    finished = "finished"
    stopped = "stopped"
    failed = "failed"
    timeout = "timeout"


TERMINAL_STATES = {SimStatus.finished, SimStatus.stopped, SimStatus.failed, SimStatus.timeout}


class Telemetry(BaseModel):
    """Posted by the Webots controller. Every field is measured in the simulator."""

    t: float = 0.0                                  # simulation time (s)
    state: str = "idle"                             # navigator state
    position: tuple[float, float] = (0.0, 0.0)      # ground truth (Supervisor)
    heading: float = 0.0                            # ground truth, rad
    odometry: Optional[tuple[float, float, float]] = None  # wheel-encoder estimate x, y, theta
    target_distance: Optional[float] = None         # ground truth
    sensors: dict[str, float] = Field(default_factory=dict)  # raw ps0..ps7
    sensor_summary: dict[str, float] = Field(default_factory=dict)  # left/front/right
    motors: tuple[float, float] = (0.0, 0.0)
    waypoint_index: int = 0
    metrics: dict[str, float | int] = Field(default_factory=dict)
    finished: bool = False
    result: Optional[dict] = None
    comm: str = "ok"
    events: list[str] = Field(default_factory=list)  # controller-side events since last post
