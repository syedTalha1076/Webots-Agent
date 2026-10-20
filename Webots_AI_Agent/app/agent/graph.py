"""LangGraph workflow.

   plan phase : parse_request -> validate_spec -> (clarify | plan -> build_world) -> END
   run phase  : launch -> (monitor <-> evaluate -> [replan]) -> report -> END
"""
from __future__ import annotations

import asyncio

from langgraph.graph import END, START, StateGraph

from app.agent.nodes import AgentNodes
from app.agent.state import AgentState
from app.config import Settings
from app.simulation.manager import Session, SimulationError, SimulationManager


def build_graph(nodes: AgentNodes):
    g = StateGraph(AgentState)
    for name in ("parse_request", "validate_spec", "clarify", "plan", "build_world", "launch",
                 "monitor", "evaluate", "replan", "report"):
        g.add_node(name, getattr(nodes, name))
    g.add_conditional_edges(
        START, lambda st: {"run": "launch", "monitor": "monitor"}.get(st.get("phase"), "parse_request"),
        {"launch": "launch", "monitor": "monitor", "parse_request": "parse_request"})
    g.add_edge("parse_request", "validate_spec")
    g.add_conditional_edges("validate_spec", nodes.route_after_validate, {"clarify": "clarify", "plan": "plan"})
    g.add_edge("clarify", END)
    g.add_edge("plan", "build_world")
    g.add_edge("build_world", END)
    g.add_conditional_edges("launch", nodes.route_after_launch, {"monitor": "monitor", "report": "report"})
    g.add_edge("monitor", "evaluate")
    g.add_conditional_edges("evaluate", nodes.route_after_evaluate,
                            {"continue": "monitor", "replan": "replan", "done": "report"})
    g.add_edge("replan", "monitor")
    g.add_edge("report", END)
    return g.compile()


class AgentRunner:
    def __init__(self, manager: SimulationManager, parser, settings: Settings, poll_s: float = 1.0):
        self.manager = manager
        self.graph = build_graph(AgentNodes(manager, parser, settings, poll_s))
        self.cfg = {"recursion_limit": 100_000}

    async def plan(self, s: Session) -> dict:
        return await self.graph.ainvoke({"session_id": s.id, "phase": "plan"}, self.cfg)

    async def start(self, s: Session) -> None:
        """Launch Webots through the graph and keep monitoring in the background."""
        s.launch_error = None
        s.task = asyncio.create_task(self._run(s))
        for _ in range(100):
            await asyncio.sleep(0.05)
            if s.launch_error or s.status.value != "ready" or s.task.done():
                break
        if s.launch_error:
            err, s.launch_error = s.launch_error, None
            raise SimulationError(err, s.launch_error_code)

    async def _run_monitor(self, s: Session) -> None:
        """Resume the monitor/evaluate loop (e.g. after a reset of a finished run)."""
        await self._run(s, "monitor")

    async def _run(self, s: Session, phase: str = "run") -> None:
        try:
            await self.graph.ainvoke({"session_id": s.id, "phase": phase}, self.cfg)
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # keep the session consistent even if the workflow crashes
            self.manager.add_event(s, "error", f"Agent workflow error: {exc}")
