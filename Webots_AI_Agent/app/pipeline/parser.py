"""Prompt -> SimSpec.

Providers (LLM_PROVIDER in .env):  auto | ollama | groq | rules
  auto   : Ollama if the server is reachable and the model is installed, else Groq if GROQ_API_KEY is set,
           else the rule-based parser.
  Any LLM failure (network, bad JSON, invalid spec) falls back to the rule-based parser and says so.
The LLM only fills the JSON spec - it never writes code, commands or file paths.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from typing import Optional

import httpx
from pydantic import ValidationError

from app.config import Settings
from app.pipeline.spec import CAPABILITIES, UNSUPPORTED_HINT, SimSpec

NUMW = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "seven": 7, "eight": 8, "nine": 9,
        "ten": 10, "a few": 3, "several": 5, "many": 8, "no": 0, "zero": 0}
NUM = r"(-?\d+(?:\.\d+)?)"
UNSUPPORTED = ("humanoid", "biped", "legged", "quadruped", "hexapod", "spider", "snake", "underwater", "submarine",
               "boat", "ackermann", "steering car", "slam", "reinforcement learning", "two arms", "dual-arm",
               "dual arm", "walking")


@dataclass
class ParseOutcome:
    spec: Optional[SimSpec] = None
    message: Optional[str] = None          # unsupported / clarification / error explanation
    provider: str = "rules"
    notes: list[str] = field(default_factory=list)


def _n(tok: str) -> int:
    return int(tok) if tok.isdigit() else NUMW[tok]


def _pairs(text: str) -> list[tuple[float, float, str]]:
    out = []
    for m in re.finditer(r"[\(\[]\s*" + NUM + r"\s*[,;]\s*" + NUM + r"\s*[\)\]]", text):
        out.append((float(m[1]), float(m[2]), text[max(0, m.start() - 28):m.start()]))
    return out


# --------------------------------------------------------------------------------------------- rules
def parse_rules(prompt: str) -> ParseOutcome:
    t = " ".join(prompt.lower().split())
    out = ParseOutcome(provider="rules")
    hit = next((w for w in UNSUPPORTED if w in t), None)
    if hit:
        out.message = f"'{hit}' is not supported. {UNSUPPORTED_HINT}"
        return out

    has = lambda *ws: any(re.search(w, t) for w in ws)       # noqa: E731
    is_arm = has(r"\barm\b", r"manipulator", r"gripper", r"pick[- ]?and[- ]?place", r"robotic arm")
    is_drone = has(r"drone", r"quadcopter", r"quadrotor", r"mavic", r"\bfly\b", r"flying", r"hover")
    is_line = has(r"line[- ]?follow", r"follow(?:s|ing)? (?:a |the )?(?:black )?line", r"line track", r"\bline\b.*\btrack\b")
    is_wh = has(r"warehouse", r"shelf", r"shelves", r"pallet", r"deliver", r"logistics")
    four = has(r"four[- ]?wheel", r"4[- ]?wheel", r"skid[- ]?steer")
    custom_words = four or has(r"\bcustom\b", r"\d\s*(?:cm|m)\s*(?:long|wide|length|width)", r"wheel radius")
    d: dict = {"robot": {}, "environment": {}, "behavior": {}, "sim": {}, "prompt": prompt, "title": prompt[:60]}
    R, E, B = d["robot"], d["environment"], d["behavior"]

    # ---- robot kind + task
    if is_arm:
        R["kind"], B["task"], E["kind"] = "arm", "pick_and_place", "arm_table"
    elif is_drone:
        R["kind"], B["task"], E["kind"] = "drone", "fly_waypoints", "open_field"
        E["width"] = E["height"] = 10.0
    else:
        if is_line:
            R["kind"], B["task"], E["kind"] = "custom_wheeled", "line_follow", "line_track"
            E["width"] = E["height"] = 3.0
        else:
            R["kind"] = "custom_wheeled" if custom_words else "epuck"
            has_goal = has(r"target", r"goal", r"navigate", r"go to", r"reach", r"deliver", r"destination", r"waypoint",
                           r"visit", r"drive to", r"move to", r"pick ?up")
            B["task"] = "go_to_targets" if (has_goal or is_wh) else (
                "wander_avoid" if has(r"avoid", r"wander", r"explore", r"patrol") else "go_to_targets")
            E["kind"] = "warehouse" if is_wh else "arena"
            if is_wh:
                E["width"], E["height"] = 8.0, 6.0
        if four:
            R["drive"] = "four_wheel"

    # ---- sizes
    m = re.search(NUM + r"\s*(?:m|meters?|metres?)?\s*(?:x|by|×)\s*" + NUM + r"\s*(?:m\b|meters?|metres?)?", t)
    if m and not is_drone and not re.search(r"\bx\s*\(", t):
        E["width"], E["height"] = float(m[1]), float(m[2])
    else:
        m = re.search(NUM + r"\s*(?:m|meter|metre)s?\s*(?:square\s*)?(?:room|arena|area|floor|field|warehouse)", t)
        if m:
            E["width"] = E["height"] = float(m[1])
    m = re.search(r"(\d+|" + "|".join(NUMW) + r")\s+(?:\w+\s+){0,2}?(?:obstacles?|boxes|blocks|barriers)", t)
    if m and not re.search(r"(?:robots?|rovers?)", m[0]):
        E["obstacles"] = _n(m[1])
    elif has(r"no obstacles?", r"without obstacles?", r"empty (?:room|arena)"):
        E["obstacles"] = 0
    elif not is_arm and not is_drone and not is_line and B.get("task") in ("go_to_targets", "wander_avoid") \
            and has(r"obstacle", r"avoid"):
        E["obstacles"] = 5 if E["kind"] == "arena" else 0
    m = re.search(r"(\d+|" + "|".join(NUMW) + r")\s+shel(?:f|ves)", t)
    if m:
        E["shelves"] = _n(m[1])
    elif E["kind"] == "warehouse":
        E["shelves"] = 4
    m = re.search(r"(\d+|" + "|".join(NUMW) + r")\s+(?:\w+[- ]?){0,2}?(?:robots|e-?pucks|rovers|vehicles|agvs|bots)\b", t)
    if m and not is_arm and not is_drone:
        R["count"] = _n(m[1])
    elif has(r"multi[- ]?robot", r"swarm", r"fleet", r"several robots", r"multiple robots"):
        R["count"] = 3
    # custom dimensions
    for key, pat in (("body_length", r"(?:length|long)"), ("body_width", r"(?:width|wide)"), ("body_height", r"height|tall")):
        mm = re.search(NUM + r"\s*(cm|m)\s*" + pat, t) or re.search(pat + r"\s*(?:of|is|:)?\s*" + NUM + r"\s*(cm|m)", t)
        if mm:
            vals = [g for g in mm.groups() if g]
            num = float(next(v for v in vals if re.fullmatch(NUM, v)))
            unit = next(v for v in vals if v in ("cm", "m"))
            R[key] = num / 100 if unit == "cm" else num
    mm = re.search(r"wheel radius\s*(?:of|is|:)?\s*" + NUM + r"\s*(cm|m)", t)
    if mm:
        R["wheel_radius"] = float(mm[1]) / 100 if mm[2] == "cm" else float(mm[1])
    mm = re.search(r"speed\s*(?:of|=|:|to)?\s*" + NUM, t)
    if mm:
        R["max_speed"] = float(mm[1])
    elif has(r"\bslow"):
        R["max_speed"] = 3.0
    # ---- coordinates
    starts, targets = [], []
    for x, y, ctx in _pairs(t):
        if "start" in ctx or "begin" in ctx or "from" in ctx.split()[-1:] and "start" in ctx:
            starts.append((x, y))
        elif is_arm and re.search(r"pick|from|cube|object", ctx):
            B["pick_position"] = (x, y)
        elif is_arm and re.search(r"place|drop|to\b|put", ctx):
            B["place_position"] = (x, y)
        else:
            targets.append((x, y))
    if starts:
        R["start_positions"] = starts
    if targets and not is_arm:
        B["targets"] = targets
    if is_drone:
        mm = re.search(r"(?:altitude|height|at)\s*(?:of)?\s*" + NUM + r"\s*(?:m|meters?|metres?)", t)
        if mm:
            B["altitude"] = float(mm[1])
    mm = re.search(r"(\d+|" + "|".join(NUMW) + r")\s+laps?", t)
    if mm:
        B["laps"] = _n(mm[1])
    if has(r"square (?:track|path|line)", r"square-shaped"):
        E["track_shape"] = "square"
    mm = re.search(r"(?:for|during)\s+" + NUM + r"\s*(?:s|sec|seconds|min|minutes)", t)
    if mm:
        secs = float(mm[1]) * (60 if "min" in mm[0] else 1)
        B["duration_s"] = max(10.0, secs)
    if has(r"\bfast\b", r"real[- ]?time off"):
        d["sim"]["mode"] = "fast"
    # ---- default title
    d["title"] = prompt.strip()[:60]
    try:
        out.spec = SimSpec.model_validate(d)
    except ValidationError as exc:
        out.message = "That configuration is not valid: " + "; ".join(
            f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in exc.errors()) + ". " + UNSUPPORTED_HINT
    if out.spec and not any([is_arm, is_drone, is_line, has(r"robot", r"rover", r"e-?puck", r"vehicle", r"wheel", r"agv", r"bot\b")]):
        out.spec, out.message = None, "I could not tell what robot you want. " + UNSUPPORTED_HINT
    return out


# --------------------------------------------------------------------------------------------- LLM
def system_prompt() -> str:
    caps = "\n".join(f"- {k}: tasks={sorted(v['tasks'])}, environments={sorted(v['envs'])}, max_count={v['max_count']}"
                     for k, v in CAPABILITIES.items())
    return f"""You convert a robotics simulation request into ONE JSON object (no prose, no markdown).
Units: metres, radians, seconds. The room is centred on (0,0); x = width axis, y = height axis.
Robot kinds and what they can do:
{caps}
- "epuck" is the small built-in differential-drive robot (default for generic 'wheeled robot').
- "custom_wheeled" is a generated robot (drive: "differential" = 2 wheels, "four_wheel" = 4-wheel skid steer) with
  body_length/body_width/body_height/wheel_radius in metres; required for line following or custom sizes.
- "arm": pick_position/place_position are (x,y) offsets in metres from the arm base, 0.18-0.50 m away.
- "drone": targets are (x,y) waypoints, altitude in metres.
Task names: wander_avoid, go_to_targets, line_follow, pick_and_place, fly_waypoints.
Environment kinds: arena, warehouse, line_track, arm_table, open_field.
JSON shape (omit fields the user did not state or imply; defaults are applied later):
{{"title": str, "robot": {{"kind":..., "count": int, "drive":..., "body_length":..., "body_width":..., "body_height":...,
 "wheel_radius":..., "max_speed":..., "start_positions": [[x,y],...]}},
 "environment": {{"kind":..., "width":..., "height":..., "obstacles": int, "obstacle_positions": [[x,y],...],
 "shelves": int, "track_shape": "oval"|"square"}},
 "behavior": {{"task":..., "targets": [[x,y],...], "altitude":..., "laps": int, "duration_s":...,
 "pick_position": [x,y], "place_position": [x,y]}},
 "sim": {{"timeout_s":..., "mode": "realtime"|"fast"}}}}
If the request needs something unsupported (humanoid, legged, underwater, steering cars...), output ONLY
{{"unsupported_reason": "<short explanation>"}}. If it is too vague to build anything, output ONLY
{{"clarification_question": "<one question>"}}. Never output code, commands or file paths."""


def _spec_from_llm(data: dict, prompt: str, provider: str) -> ParseOutcome:
    out = ParseOutcome(provider=provider)
    if data.get("unsupported_reason"):
        out.message = f"{data['unsupported_reason']} {UNSUPPORTED_HINT}"
        return out
    if data.get("clarification_question"):
        out.message = str(data["clarification_question"])
        return out
    data = {k: v for k, v in data.items() if k in ("title", "robot", "environment", "behavior", "sim")}
    data["prompt"] = prompt
    try:
        out.spec = SimSpec.model_validate(data)
    except ValidationError as exc:
        raise ValueError("; ".join(f"{'.'.join(map(str, e['loc']))}: {e['msg']}" for e in exc.errors())) from exc
    return out


def _json_from_text(text: str) -> dict:
    text = re.sub(r"^```(?:json)?|```$", "", text.strip(), flags=re.M).strip()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        m = re.search(r"\{.*\}", text, re.S)
        if not m:
            raise
        return json.loads(m.group(0))


def ollama_status(settings: Settings) -> tuple[bool, str]:
    try:
        r = httpx.get(settings.ollama_base_url.rstrip("/") + "/api/tags", timeout=1.5)
        r.raise_for_status()
        names = [m.get("name", "") for m in r.json().get("models", [])]
    except Exception as exc:
        return False, f"Ollama not reachable at {settings.ollama_base_url} ({type(exc).__name__})"
    want = settings.ollama_model
    if any(n == want or n.split(":")[0] == want.split(":")[0] and ":" not in want for n in names):
        return True, f"Ollama ready, model {want}"
    return False, f"Ollama is running but model '{want}' is not installed (run: ollama pull {want}). Installed: {names or 'none'}"


def parse_ollama(prompt: str, settings: Settings) -> ParseOutcome:
    r = httpx.post(settings.ollama_base_url.rstrip("/") + "/api/chat", timeout=settings.ollama_timeout, json={
        "model": settings.ollama_model, "stream": False, "format": "json", "options": {"temperature": 0},
        "messages": [{"role": "system", "content": system_prompt()}, {"role": "user", "content": prompt}]})
    r.raise_for_status()
    return _spec_from_llm(_json_from_text(r.json()["message"]["content"]), prompt, f"ollama:{settings.ollama_model}")


def parse_groq(prompt: str, settings: Settings) -> ParseOutcome:
    model = settings.groq_model or "llama-3.3-70b-versatile"
    r = httpx.post("https://api.groq.com/openai/v1/chat/completions", timeout=45,
                   headers={"Authorization": f"Bearer {settings.groq_api_key}"},
                   json={"model": model, "temperature": 0, "response_format": {"type": "json_object"},
                         "messages": [{"role": "system", "content": system_prompt()},
                                      {"role": "user", "content": prompt}]})
    r.raise_for_status()
    return _spec_from_llm(_json_from_text(r.json()["choices"][0]["message"]["content"]), prompt, f"groq:{model}")


def parse_prompt(prompt: str, settings: Settings) -> ParseOutcome:
    """Pick a provider per LLM_PROVIDER, falling back to rules (with an explanation) if an LLM fails."""
    choice = (settings.llm_provider or "auto").lower()
    notes: list[str] = []
    order: list[str] = []
    if choice == "auto":
        ok, why = ollama_status(settings)
        if ok:
            order.append("ollama")
        else:
            notes.append(why)
        if settings.groq_api_key.strip():
            order.append("groq")
    elif choice in ("ollama", "groq"):
        order.append(choice)
    for prov in order:
        try:
            if prov == "groq" and not settings.groq_api_key.strip():
                raise RuntimeError("GROQ_API_KEY is empty")
            res = parse_ollama(prompt, settings) if prov == "ollama" else parse_groq(prompt, settings)
            res.notes = notes + res.notes
            return res
        except Exception as exc:
            notes.append(f"{prov} failed ({type(exc).__name__}: {str(exc)[:160]}); trying the next option")
    res = parse_rules(prompt)
    res.notes = notes + ["Used the rule-based parser (supports a fixed set of tasks)."] + res.notes
    return res
