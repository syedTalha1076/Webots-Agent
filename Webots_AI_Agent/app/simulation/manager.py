"""Simulation session manager: lifecycle, subprocess control, watchdog and result evaluation."""
from __future__ import annotations

import asyncio
import math
import subprocess
import sys
import time
import uuid
import secrets
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

from app.config import APPROVED_EXECUTABLE_NAMES, Settings
from app.schemas import TERMINAL_STATES, RobotSpec, SimStatus, Telemetry
from app.simulation.controller_bridge import CommandQueue, Hub
from app.simulation.world_generator import cleanup_session_dir

STARTUP_GRACE_S = 90        # wall seconds for Webots to open and the controller to connect
TELEMETRY_LOSS_S = 15       # wall seconds without telemetry while running -> failure
TRACE_LIMIT = 3000


class SimulationError(Exception):
    """Raised for user-facing lifecycle problems (mapped to HTTP errors by the API layer)."""

    def __init__(self, message: str, code: int = 400):
        super().__init__(message)
        self.code = code


@dataclass
class Session:
    id: str
    prompt: str
    token: str = field(default_factory=lambda: secrets.token_urlsafe(16))
    status: SimStatus = SimStatus.planning
    stage: str = "created"
    spec: Optional[RobotSpec] = None
    plan: Optional[dict] = None
    clarification: Optional[str] = None
    directory: Optional[Path] = None
    world: Optional[Path] = None
    obstacles: list[dict] = field(default_factory=list)
    waypoints: list = field(default_factory=list)
    process: Any = None
    log_path: Optional[Path] = None
    telemetry: Optional[dict] = None
    telemetry_count: int = 0
    trace: list = field(default_factory=list)
    events: deque = field(default_factory=lambda: deque(maxlen=300))
    result: Optional[dict] = None
    agent_replans: int = 0
    commands: CommandQueue = field(default_factory=CommandQueue)
    task: Any = None
    created_wall: float = field(default_factory=time.time)
    started_wall: Optional[float] = None
    last_telemetry_wall: Optional[float] = None
    auto_start: bool = False
    launch_error: Optional[str] = None
    launch_error_code: int = 503

    def public(self) -> dict:
        return {
            "id": self.id, "prompt": self.prompt, "status": self.status.value, "stage": self.stage,
            "spec": self.spec.model_dump(mode="json") if self.spec else None,
            "plan": self.plan, "clarification": self.clarification,
            "world": {
                "width": self.spec.world.width, "height": self.spec.world.height,
                "obstacles": self.obstacles,
                "start": self.spec.start_position, "target": self.spec.target_position,
                "waypoints": self.waypoints,
            } if self.spec else None,
            "result": self.result, "agent_replans": self.agent_replans,
            "telemetry": self.telemetry, "events": list(self.events)[-60:],
            "world_path": str(self.world) if self.world else None,
        }


Launcher = Callable[[list[str], Path, Path], Any]


def default_launcher(cmd: list[str], cwd: Path, log_path: Path) -> subprocess.Popen:
    log = open(log_path, "ab")
    flags = subprocess.CREATE_NEW_PROCESS_GROUP if sys.platform == "win32" else 0
    return subprocess.Popen(cmd, cwd=str(cwd), stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            creationflags=flags)


class SimulationManager:
    def __init__(self, settings: Settings, hub: Optional[Hub] = None, launcher: Launcher = default_launcher):
        self.settings = settings
        self.hub = hub or Hub()
        self.launcher = launcher
        self.sessions: dict[str, Session] = {}
        self.stop_grace_s = 4.0     # how long to wait for the controller to quit Webots itself
        self._watch: Optional[asyncio.Task] = None

    # ------------------------------------------------------------------ sessions
    def new_session(self, prompt: str, auto_start: bool = False) -> Session:
        s = Session(id=uuid.uuid4().hex[:12], prompt=prompt, auto_start=auto_start)
        self.sessions[s.id] = s
        self.add_event(s, "info", "Session created")
        return s

    def get(self, sid: str) -> Session:
        s = self.sessions.get(sid)
        if not s:
            raise SimulationError("Unknown simulation session", 404)
        return s

    def active_count(self) -> int:
        return sum(1 for s in self.sessions.values() if s.status in
                   (SimStatus.starting, SimStatus.running, SimStatus.paused))

    def add_event(self, s: Session, kind: str, message: str) -> None:
        ev = {"time": round(time.time(), 2), "kind": kind, "message": message}
        s.events.append(ev)
        self.hub.publish(s.id, {"type": "event", "event": ev})

    def set_status(self, s: Session, status: SimStatus, stage: Optional[str] = None) -> None:
        s.status = status
        if stage:
            s.stage = stage
        self.hub.publish(s.id, {"type": "status", "status": status.value, "stage": s.stage})

    # ------------------------------------------------------------------ lifecycle
    def validate_launch_paths(self, s: Session, exe: Path) -> None:
        """Only approved executables and generated world files inside the simulations directory may run."""
        sim_root = self.settings.simulations_dir
        if not s.world or sim_root not in s.world.resolve().parents or s.world.suffix != ".wbt":
            raise SimulationError("World file is outside the simulations directory", 400)
        if exe.name.lower() not in APPROVED_EXECUTABLE_NAMES:
            raise SimulationError("Executable is not an approved Webots binary", 400)

    def launch(self, s: Session) -> None:
        if s.status in (SimStatus.starting, SimStatus.running, SimStatus.paused):
            raise SimulationError("Simulation is already running", 409)
        if s.status != SimStatus.ready:
            raise SimulationError(f"Cannot start a simulation in state '{s.status.value}'. "
                                  "Create a new simulation instead.", 409)
        if not s.world:
            raise SimulationError("Simulation world has not been generated", 409)
        if self.active_count() >= self.settings.max_robot_count:
            raise SimulationError(f"Too many simulations running (limit {self.settings.max_robot_count})", 429)
        exe = self.settings.find_webots()
        if exe is None:
            msg = self.settings.webots_status()["message"]
            self.add_event(s, "error", msg)
            raise SimulationError(msg, 503)
        self.validate_launch_paths(s, exe)
        cmd = [str(exe), f"--mode={self.settings.webots_mode}", "--batch", "--stdout", "--stderr", str(s.world)]
        s.log_path = s.directory / "webots.log"
        try:
            s.process = self.launcher(cmd, s.directory, s.log_path)
        except OSError as exc:
            self.add_event(s, "error", f"Could not start Webots: {exc}")
            raise SimulationError(f"Could not start Webots: {exc}", 500) from exc
        s.started_wall = time.time()
        s.last_telemetry_wall = None
        self.set_status(s, SimStatus.starting, "launching_webots")
        self.add_event(s, "info", f"Webots launched (pid {getattr(s.process, 'pid', '?')})")

    def command(self, s: Session, name: str, **kw: Any) -> None:
        if s.status in TERMINAL_STATES:
            raise SimulationError(f"Simulation already {s.status.value}", 409)
        if s.status not in (SimStatus.starting, SimStatus.running, SimStatus.paused):
            raise SimulationError("Simulation is not running", 409)
        if name == "pause" and s.status == SimStatus.paused:
            raise SimulationError("Already paused", 409)
        if name == "resume" and s.status != SimStatus.paused:
            raise SimulationError("Simulation is not paused", 409)
        s.commands.push(name, **kw)
        self.add_event(s, "command", f"Command queued: {name}")

    async def reset(self, s: Session) -> None:
        if s.status in (SimStatus.stopped, SimStatus.failed, SimStatus.timeout):
            raise SimulationError("Simulation process is gone; create a new simulation", 409)
        if s.status == SimStatus.ready:
            raise SimulationError("Nothing to reset yet", 409)
        s.commands.push("reset")
        s.result, s.trace, s.agent_replans = None, [], 0
        if s.status == SimStatus.finished:
            self.set_status(s, SimStatus.running, "reset")
        self.add_event(s, "command", "Reset requested: robot returns to its start pose")

    async def stop(self, s: Session, grace: Optional[float] = None) -> None:
        """Emergency stop: ask the controller to halt and quit its own Webots instance, then force-kill it."""
        if s.status in TERMINAL_STATES:
            return
        s.commands.push("stop")
        self.add_event(s, "command", "Emergency stop requested")
        if s.task and not s.task.done() and s.task is not asyncio.current_task():
            s.task.cancel()
        proc = s.process
        deadline = time.time() + (self.stop_grace_s if grace is None else grace)
        while proc is not None and time.time() < deadline and proc.poll() is None:
            await asyncio.sleep(0.2)
        self.kill_process(s)
        if s.result is None:
            s.result = self.build_result(s, False, "stopped by user")
        self.set_status(s, SimStatus.stopped, "stopped")
        self.add_event(s, "info", "Simulation stopped")

    def kill_process(self, s: Session) -> None:
        """Terminate ONLY the process tree this session started."""
        proc = s.process
        if proc is None:
            return
        try:
            if proc.poll() is None:
                if sys.platform == "win32":
                    subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"],
                                   capture_output=True, check=False, timeout=10)
                else:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except Exception:
                        proc.kill()
        except Exception as exc:  # pragma: no cover - best effort
            self.add_event(s, "error", f"Error while stopping process: {exc}")

    def delete(self, s: Session) -> None:
        if s.status in (SimStatus.starting, SimStatus.running, SimStatus.paused):
            raise SimulationError("Stop the simulation before deleting it", 409)
        self.sessions.pop(s.id, None)
        cleanup_session_dir(self.settings, s.id)

    # ------------------------------------------------------------------ telemetry
    def ingest(self, s: Session, tel: Telemetry) -> list[dict]:
        """Store controller telemetry; returns commands for the controller (e-stop first)."""
        data = tel.model_dump(mode="json")
        s.telemetry = data
        s.telemetry_count += 1
        s.last_telemetry_wall = time.time()
        if not s.trace or math.dist(s.trace[-1], tel.position) > 0.01:
            s.trace.append(list(tel.position))
            del s.trace[:-TRACE_LIMIT]
        for msg in tel.events:
            self.add_event(s, "controller", msg)
        if s.status not in TERMINAL_STATES:
            if tel.finished:
                ok = bool((tel.result or {}).get("success"))
                s.result = self.build_result(s, ok, (tel.result or {}).get("reason", ""))
                self.set_status(s, SimStatus.finished, "finished")
                self.add_event(s, "success" if ok else "error",
                               f"Task {'completed' if ok else 'failed'}: {s.result['reason']}")
            elif tel.state == "paused":
                if s.status != SimStatus.paused:
                    self.set_status(s, SimStatus.paused, "paused")
            elif s.status in (SimStatus.starting, SimStatus.paused) and tel.state not in ("stopped",):
                self.set_status(s, SimStatus.running, "navigating")
                if s.status == SimStatus.running:
                    self.add_event(s, "info", "Controller connected - receiving live telemetry")
        self.hub.publish(s.id, {"type": "telemetry", "telemetry": data, "trace_tail": s.trace[-1:]})
        return s.commands.drain()

    def build_result(self, s: Session, success: bool, reason: str) -> dict:
        tel = s.telemetry or {}
        m = tel.get("metrics", {})
        travelled = m.get("distance_travelled")
        straight = math.dist(s.spec.start_position, s.spec.target_position) if s.spec else None
        return {
            "success": success, "reason": reason or ("target reached" if success else "unknown"),
            "measured": {
                "simulation_time_s": tel.get("t"),
                "final_distance_to_target_m": tel.get("target_distance"),
                "distance_travelled_m": travelled,
                "collisions": m.get("collisions"),
                "obstacle_encounters": m.get("obstacle_encounters"),
                "replanning_events": s.agent_replans,
                "wall_clock_s": round(time.time() - s.started_wall, 1) if s.started_wall else None,
            },
            "estimated": {
                "straight_line_distance_m": round(straight, 3) if straight else None,
                "path_efficiency": round(straight / travelled, 3) if straight and travelled else None,
            },
            "notes": "Position/distance values are Supervisor ground truth inside the simulator; "
                     "collisions use approximate obstacle geometry. Estimated values are derived, not measured.",
        }

    # ------------------------------------------------------------------ watchdog
    def start_watchdog(self) -> None:
        if self._watch is None or self._watch.done():
            self._watch = asyncio.create_task(self._watchdog())

    async def shutdown(self) -> None:
        if self._watch:
            self._watch.cancel()
        for s in list(self.sessions.values()):
            if s.status in (SimStatus.starting, SimStatus.running, SimStatus.paused):
                await self.stop(s, grace=0.5)

    async def _watchdog(self) -> None:
        while True:
            await asyncio.sleep(1.0)
            for s in list(self.sessions.values()):
                try:
                    await self.check(s)
                except Exception as exc:  # pragma: no cover
                    self.add_event(s, "error", f"watchdog error: {exc}")

    async def check(self, s: Session, now: Optional[float] = None) -> None:
        if s.status not in (SimStatus.starting, SimStatus.running, SimStatus.paused) or s.started_wall is None:
            return
        now = now or time.time()
        if now - s.started_wall > self.settings.simulation_timeout * 3 + STARTUP_GRACE_S:
            await self._fail(s, SimStatus.timeout, "wall-clock timeout")
        elif s.last_telemetry_wall is None:
            exited = s.process is not None and s.process.poll() not in (None,)
            if exited and s.process.poll() != 0:
                await self._fail(s, SimStatus.failed, f"Webots exited with code {s.process.poll()} before the "
                                 f"controller connected. See {s.log_path}")
            elif now - s.started_wall > STARTUP_GRACE_S:
                await self._fail(s, SimStatus.failed, "Controller never connected to the backend. "
                                 f"Check Python/controller errors in {s.log_path}")
        elif now - s.last_telemetry_wall > TELEMETRY_LOSS_S:
            await self._fail(s, SimStatus.failed, "Lost telemetry from the controller (Webots closed or crashed)")

    async def _fail(self, s: Session, status: SimStatus, reason: str) -> None:
        self.add_event(s, "error", reason)
        self.kill_process(s)
        s.result = self.build_result(s, False, reason)
        self.set_status(s, status, status.value)
