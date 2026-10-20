"""Explicit LangGraph state and structured actions."""
from __future__ import annotations

from typing import Literal, Optional, TypedDict

MAX_REPLANS = 3


class AgentState(TypedDict, total=False):
    session_id: str
    phase: Literal["plan", "run", "monitor"]
    spec: Optional[dict]
    clarification: Optional[str]
    notes: list[str]
    parse_source: str
    decision: Literal["continue", "replan", "done"]
    last_replan_wall: float
    stage: str
    result: Optional[dict]
