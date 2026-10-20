"""Request parsers. Provider is replaceable: implement `SpecParser.parse` and register it in `make_parser`.

 - RuleBasedParser: no API key, deterministic (default fallback).
 - GroqParser: LangChain ChatGroq with structured output; falls back to rules on any failure.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from typing import Optional, Protocol

from pydantic import BaseModel, ValidationError

from app.agent.prompts import SPEC_SYSTEM_PROMPT
from app.config import Settings
from app.schemas import MAX_WHEEL_SPEED, RobotSpec

UNSUPPORTED = ("drone", "quadcopter", "flying", "humanoid", "biped", "robot arm", "robotic arm", "manipulator",
               "legged", "quadruped", "underwater", "submarine", "spider", "walking robot", "gripper")
ROBOT_WORDS = ("robot", "rover", "bot", "wheel", "e-puck", "epuck", "vehicle", "car ")
NUMBER_WORDS = {"no": 0, "zero": 0, "one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
                "seven": 7, "eight": 8, "nine": 9, "ten": 10, "a few": 3, "several": 5, "many": 8}
NUM = r"(-?\d+(?:\.\d+)?)"
SUPPORTED_HELP = ("I can simulate a small differential-drive wheeled robot (e-puck) that drives to a target in a "
                  "rectangular room with box obstacles. Describe the room size, number of obstacles, and optionally "
                  "start/target positions and speed.")


@dataclass
class ParseResult:
    spec: Optional[RobotSpec] = None
    clarification: Optional[str] = None
    notes: list[str] = field(default_factory=list)
    source: str = "rules"


class SpecParser(Protocol):
    name: str

    async def parse(self, text: str) -> ParseResult: ...


def build_spec(d: dict, notes: list[str], source: str) -> ParseResult:
    """Single authority for validation: every parser output goes through RobotSpec."""
    try:
        return ParseResult(spec=RobotSpec.model_validate(d), notes=notes, source=source)
    except ValidationError as exc:
        msgs = "; ".join(e["msg"].replace("Value error, ", "") + (f" ({'.'.join(map(str, e['loc']))})" if e["loc"] else "")
                         for e in exc.errors())
        return ParseResult(clarification=f"That configuration is not valid: {msgs}. Please adjust your request.",
                           notes=notes, source=source)


class RuleBasedParser:
    name = "rules"

    async def parse(self, text: str) -> ParseResult:
        t = " ".join(text.lower().split())
        notes: list[str] = []
        hit = next((w for w in UNSUPPORTED if w in t), None)
        if hit:
            return ParseResult(clarification=f"'{hit}' robots are not supported yet. {SUPPORTED_HELP}", source=self.name)
        if not any(w in t + " " for w in ROBOT_WORDS):
            return ParseResult(clarification="I could not tell what robot you want. " + SUPPORTED_HELP, source=self.name)

        d: dict = {"world": {}}
        m = re.search(NUM + r"\s*(?:m|meters?|metres?)?\s*(?:x|by|×)\s*" + NUM + r"\s*(?:m\b|meters?|metres?)?", t)
        if m:
            d["world"]["width"], d["world"]["height"] = float(m[1]), float(m[2])
        else:
            m = re.search(NUM + r"\s*(?:m|meter|metre)s?\s*(?:square\s*)?(?:room|arena|area|floor)", t) or \
                re.search(r"(?:room|arena|area)\s*(?:of|is|size)?\s*" + NUM + r"\s*(?:m|meters?|metres?)", t)
            if m:
                d["world"]["width"] = d["world"]["height"] = float(m[1])

        m = re.search(r"(\d+|" + "|".join(NUMBER_WORDS) + r")\s+(?:\w+\s+)?(?:obstacles?|boxes|blocks|barriers)", t)
        if m:
            d["world"]["obstacles"] = int(m[1]) if m[1].isdigit() else NUMBER_WORDS[m[1]]
        elif re.search(r"\bno obstacles?\b|\bwithout obstacles?\b|\bempty room\b", t):
            d["world"]["obstacles"] = 0

        pair = NUM + r"\s*,\s*" + NUM
        m = re.search(r"start(?:ing)?(?:\s+(?:position|point|pose))?\s*(?:at|from|is|:)?\s*[\(\[]\s*" + pair, t)
        if m:
            d["start_position"] = (float(m[1]), float(m[2]))
        m = re.search(r"(?:target|goal|destination)(?:\s+(?:position|point))?\s*(?:at|is|:)?\s*[\(\[]\s*" + pair, t)
        if m:
            d["target_position"] = (float(m[1]), float(m[2]))

        m = re.search(r"speed\s*(?:of|=|:|to)?\s*" + NUM, t)
        if m:
            speed = float(m[1])
            if speed > MAX_WHEEL_SPEED:
                notes.append(f"speed {speed} exceeds the robot limit; clamped to {MAX_WHEEL_SPEED} rad/s")
            d["max_speed"] = min(max(speed, 0.5), MAX_WHEEL_SPEED)
        elif re.search(r"\b(slow|slowly)\b", t):
            d["max_speed"] = 2.0
        elif re.search(r"\b(fast|quickly|full speed)\b", t):
            d["max_speed"] = MAX_WHEEL_SPEED

        if re.search(r"waypoint|path planning|plan a path|a\*|known map|\bmap\b", t):
            d["planning_mode"] = "waypoints"
        if not d["world"]:
            del d["world"]
        return build_spec(d, notes, self.name)


class SpecDraft(BaseModel):
    """Flat, tolerant schema the LLM fills in. Converted + strictly validated into RobotSpec afterwards."""
    room_width: Optional[float] = None
    room_height: Optional[float] = None
    obstacles: Optional[int] = None
    start_x: Optional[float] = None
    start_y: Optional[float] = None
    target_x: Optional[float] = None
    target_y: Optional[float] = None
    max_speed: Optional[float] = None
    planning_mode: Optional[str] = None
    unsupported_reason: Optional[str] = None
    clarification_question: Optional[str] = None


def draft_to_result(draft: SpecDraft, source: str) -> ParseResult:
    if draft.unsupported_reason:
        return ParseResult(clarification=f"{draft.unsupported_reason} {SUPPORTED_HELP}", source=source)
    if draft.clarification_question:
        return ParseResult(clarification=draft.clarification_question, source=source)
    notes: list[str] = []
    d: dict = {"world": {}}
    for src, key in (("room_width", "width"), ("room_height", "height"), ("obstacles", "obstacles")):
        if getattr(draft, src) is not None:
            d["world"][key] = getattr(draft, src)
    if draft.start_x is not None and draft.start_y is not None:
        d["start_position"] = (draft.start_x, draft.start_y)
    if draft.target_x is not None and draft.target_y is not None:
        d["target_position"] = (draft.target_x, draft.target_y)
    if draft.max_speed is not None:
        if draft.max_speed > MAX_WHEEL_SPEED:
            notes.append(f"speed clamped to {MAX_WHEEL_SPEED} rad/s")
        d["max_speed"] = min(max(draft.max_speed, 0.5), MAX_WHEEL_SPEED)
    if draft.planning_mode in ("reactive", "waypoints"):
        d["planning_mode"] = draft.planning_mode
    return build_spec(d, notes, source)


class GroqParser:
    name = "groq"

    def __init__(self, settings: Settings, fallback: SpecParser):
        self.settings, self.fallback = settings, fallback

    async def parse(self, text: str) -> ParseResult:
        try:
            from langchain_core.messages import HumanMessage, SystemMessage
            from langchain_groq import ChatGroq

            llm = ChatGroq(model=self.settings.groq_model or "llama-3.3-70b-versatile",
                           api_key=self.settings.groq_api_key, temperature=0, timeout=30, max_retries=1)
            draft = await llm.with_structured_output(SpecDraft).ainvoke(
                [SystemMessage(content=SPEC_SYSTEM_PROMPT), HumanMessage(content=text)])
            return draft_to_result(draft, self.name)
        except Exception as exc:  # network, quota, schema problems -> deterministic fallback
            res = await self.fallback.parse(text)
            res.notes.append(f"LLM unavailable ({type(exc).__name__}); used rule-based parser")
            res.source = "rules (llm fallback)"
            return res


def make_parser(settings: Settings) -> SpecParser:
    rules = RuleBasedParser()
    return GroqParser(settings, rules) if settings.llm_enabled else rules
