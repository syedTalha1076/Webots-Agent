"""LangGraph nodes. Every node only calls whitelisted manager/generator functions - no shell, no generated code."""
from __future__ import annotations

import asyncio
import math
import secrets
import time

from app.agent.llm import SpecParser
from app.agent.state import MAX_REPLANS, AgentState
from app.config import Settings
from app.schemas import TERMINAL_STATES, RobotSpec, SimStatus, SimulationPlan
from app.simulation.manager import SimulationError, SimulationManager
from app.simulation.registry import ROBOT_MODELS
from app.simulation.world_generator import build_plan_waypoints, generate_project, layout_obstacles


REPLAN_COOLDOWN_S = 2.5   # give the controller time to act on a replan before judging it again


class AgentNodes:
    def __init__(self, manager: SimulationManager, parser: SpecParser, settings: Settings, poll_s: float = 1.0):
        self.m, self.parser, self.settings, self.poll_s = manager, parser, settings, poll_s

    def _enter(self, state: AgentState, stage: str, message: str, kind: str = "agent") -> "object":
        s = self.m.get(state["session_id"])
        s.stage = stage
        self.m.add_event(s, kind, message)
        self.m.hub.publish(s.id, {"type": "status", "status": s.status.value, "stage": stage})
        return s

    # 1. parse
    async def parse_request(self, state: AgentState) -> AgentState:
        s = self._enter(state, "parse_request", f"Interpreting request ({self.parser.name} parser)")
        res = await self.parser.parse(s.prompt)
        for n in res.notes:
            self.m.add_event(s, "warning", n)
        return {"spec": res.spec.model_dump(mode="json") if res.spec else None,
                "clarification": res.clarification, "notes": res.notes, "parse_source": res.source}

    # 2. validate (semantic checks beyond schema validation)
    async def validate_spec(self, state: AgentState) -> AgentState:
        s = self._enter(state, "validate_spec", "Validating robot configuration")
        if state.get("clarification") or not state.get("spec"):
            return {}
        spec = RobotSpec.model_validate(state["spec"])
        try:
            obstacles = layout_obstacles(spec, "validation")
            area = sum(o["size"] ** 2 for o in obstacles)
            if area > 0.2 * spec.world.width * spec.world.height:
                return {"clarification": "Too many obstacles for that room size (they would fill over 20% of the floor)."}
        except ValueError as exc:
            return {"clarification": str(exc)}
        s.spec = spec
        self.m.add_event(s, "agent", f"Spec valid: {spec.world.width}x{spec.world.height} m room, "
                                     f"{spec.world.obstacles} obstacles, speed {spec.max_speed} rad/s")
        return {}

    def route_after_validate(self, state: AgentState) -> str:
        return "clarify" if state.get("clarification") or not state.get("spec") else "plan"

    async def clarify(self, state: AgentState) -> AgentState:
        s = self._enter(state, "clarification", state["clarification"] or "Need more information", "warning")
        s.clarification = state["clarification"]
        self.m.set_status(s, SimStatus.needs_clarification, "clarification")
        return {}

    # 3. plan
    async def plan(self, state: AgentState) -> AgentState:
        s = self._enter(state, "plan", "Generating simulation plan")
        spec, model = s.spec, ROBOT_MODELS[s.spec.robot_type.value]
        obstacles = layout_obstacles(spec, s.id)
        waypoints, nav = build_plan_waypoints(spec, obstacles)
        straight = math.dist(spec.start_position, spec.target_position)
        timeout = int(min(self.settings.simulation_timeout, max(60, straight / 0.05 + 60)))
        plan = SimulationPlan(
            robot_model=f"{model.webots_proto} ({model.description})", controller=model.controller,
            objective=f"Drive from {tuple(round(v, 2) for v in spec.start_position)} to "
                      f"{tuple(round(v, 2) for v in spec.target_position)} ({straight:.2f} m)",
            navigation=nav, max_wheel_speed=spec.max_speed, waypoints=waypoints, timeout_s=timeout,
            steps=["Build Webots world", "Launch Webots + controller", "Stream telemetry",
                   "Evaluate progress / replan if stuck", "Report result"])
        s.plan = plan.model_dump(mode="json")
        return {}

    # 4. build world
    async def build_world(self, state: AgentState) -> AgentState:
        s = self._enter(state, "build_world", "Creating Webots world")
        try:
            out = generate_project(s.spec, s.id, self.settings, s.token, timeout_s=s.plan["timeout_s"])
        except Exception as exc:
            self.m.add_event(s, "error", f"World generation failed: {exc}")
            s.result = {"success": False, "reason": f"world generation failed: {exc}"}
            self.m.set_status(s, SimStatus.failed, "build_world")
            return {}
        s.directory, s.world, s.obstacles, s.waypoints = out["dir"], out["world"], out["obstacles"], out["waypoints"]
        self.m.set_status(s, SimStatus.ready, "ready")
        self.m.add_event(s, "agent", f"World ready with {len(out['obstacles'])} obstacles - press Start")
        return {}

    # 5. launch
    async def launch(self, state: AgentState) -> AgentState:
        s = self._enter(state, "launch", "Starting Webots simulation")
        try:
            self.m.launch(s)
            s.launch_error = None
        except SimulationError as exc:
            s.launch_error, s.launch_error_code = str(exc), exc.code
        return {}

    def route_after_launch(self, state: AgentState) -> str:
        return "report" if getattr(self.m.get(state["session_id"]), "launch_error", None) else "monitor"

    # 6. monitor
    async def monitor(self, state: AgentState) -> AgentState:
        s = self.m.get(state["session_id"])
        if s.status not in TERMINAL_STATES:
            s.stage = "monitor" if s.status != SimStatus.paused else "paused"
        await asyncio.sleep(self.poll_s)
        return {}

    # 7/8. evaluate + decide
    async def evaluate(self, state: AgentState) -> AgentState:
        s = self.m.get(state["session_id"])
        if s.status in TERMINAL_STATES:
            return {"decision": "done"}
        tel = s.telemetry or {}
        if tel.get("state") == "stuck" and time.time() - state.get("last_replan_wall", 0) > REPLAN_COOLDOWN_S:
            if s.agent_replans < MAX_REPLANS:
                return {"decision": "replan"}
            s.result = self.m.build_result(s, False, f"robot stuck after {MAX_REPLANS} replanning attempts")
            self.m.set_status(s, SimStatus.finished, "finished")
            self.m.add_event(s, "error", s.result["reason"])
            return {"decision": "done"}
        return {"decision": "continue"}

    def route_after_evaluate(self, state: AgentState) -> str:
        return state.get("decision", "continue")

    async def replan(self, state: AgentState) -> AgentState:
        s = self._enter(state, "replan", "Robot is stuck - replanning (other side, slower)", "warning")
        s.agent_replans += 1
        try:
            self.m.command(s, "replan", speed_scale=0.9)
        except SimulationError as exc:
            self.m.add_event(s, "error", f"Replan command failed: {exc}")
        return {"last_replan_wall": time.time()}

    # 9. report
    async def report(self, state: AgentState) -> AgentState:
        s = self.m.get(state["session_id"])
        if getattr(s, "launch_error", None):
            self.m.add_event(s, "error", s.launch_error)
            return {"result": None}
        if s.result is None:
            s.result = self.m.build_result(s, False, f"ended in state {s.status.value}")
        self.m.add_event(s, "success" if s.result["success"] else "error",
                         f"Final result: {'SUCCESS' if s.result['success'] else 'FAILURE'} - {s.result['reason']}")
        self.m.hub.publish(s.id, {"type": "result", "result": s.result})
        return {"result": s.result}
