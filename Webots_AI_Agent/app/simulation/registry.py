"""Registry of supported robot models and tasks.

EXTENSION POINT: add a RobotModel entry here (plus a PROTO/controller capable of using it)
and add its key to `schemas.RobotType`. See README -> "Adding a new robot type".
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class RobotModel:
    key: str
    webots_proto: str            # PROTO name used in the world file
    proto_url: str               # EXTERNPROTO url (webots:// resolves to the local install)
    controller: str              # controller folder name
    wheel_radius: float          # m
    axle_length: float           # m
    max_wheel_speed: float       # rad/s
    robot_radius: float          # m (for collision accounting / path inflation)
    sensors: tuple[str, ...] = field(default=("distance",))
    description: str = ""


EPUCK = RobotModel(
    key="epuck",
    webots_proto="E-puck",
    proto_url="webots://projects/robots/gctronic/e-puck/protos/E-puck.proto",
    controller="navigation_controller",
    wheel_radius=0.0205,
    axle_length=0.052,
    max_wheel_speed=6.28,
    robot_radius=0.037,
    description="GCTronic e-puck: 2 motors, 8 infra-red distance sensors (~7 cm range).",
)

ROBOT_MODELS: dict[str, RobotModel] = {
    "differential_drive": EPUCK,   # generic request -> e-puck is the built-in differential-drive robot
    "epuck": EPUCK,
}

SUPPORTED_TASKS = {"navigate_to_target": "Drive to a target marker while avoiding obstacles."}
