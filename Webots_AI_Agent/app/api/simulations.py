"""REST API: configuration, lifecycle commands and telemetry reads (kept separate from the live channel)."""
from __future__ import annotations

from fastapi import APIRouter, HTTPException, Path, Request

from app.schemas import CreateSimulationRequest, SimStatus
from app.simulation.manager import SimulationError

router = APIRouter(prefix="/api")
SID = Path(pattern=r"^[a-f0-9]{12}$", description="simulation session id")

EXAMPLES = [
    {"title": "Room with obstacles",
     "prompt": "Create a small autonomous wheeled robot with two motors and distance sensors. Place it in a 3 x 3 m room "
               "with 5 obstacles. Put a red target at the other side of the room and make it navigate there while avoiding obstacles."},
    {"title": "Precise start and goal",
     "prompt": "Wheeled robot in a 4 by 4 meter room with 6 obstacles, start at (-1.5, -1.5), target at (1.5, 1.2), speed 3."},
    {"title": "Planned path",
     "prompt": "E-puck robot in a 3 x 3 m room with 8 boxes. Use path planning with waypoints to reach the red target."},
    {"title": "Open floor",
     "prompt": "Slow wheeled robot in a 2 m room with no obstacles that drives to the target."},
]


def _mgr(request: Request):
    return request.app.state.manager


def _call(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except SimulationError as exc:
        raise HTTPException(exc.code, str(exc)) from exc


@router.get("/health")
def health(request: Request):
    st = request.app.state
    return {"app": st.settings.app_name, "webots": st.settings.webots_status(),
            "llm": {"enabled": st.settings.llm_enabled, "parser": st.runner_parser_name},
            "active_simulations": st.manager.active_count()}


@router.get("/examples")
def examples():
    return EXAMPLES


@router.post("/simulations", status_code=201)
async def create_simulation(body: CreateSimulationRequest, request: Request):
    st = request.app.state
    s = st.manager.new_session(body.prompt.strip(), body.auto_start)
    await st.runner.plan(s)
    if body.auto_start and s.status == SimStatus.ready:
        try:
            await st.runner.start(s)
        except SimulationError as exc:
            st.manager.add_event(s, "error", f"Auto-start failed: {exc}")
    return s.public()


@router.get("/simulations")
def list_simulations(request: Request):
    return [{"id": s.id, "status": s.status.value, "prompt": s.prompt[:80]} for s in _mgr(request).sessions.values()]


@router.get("/simulations/{sid}")
def get_simulation(request: Request, sid: str = SID):
    return _call(_mgr(request).get, sid).public()


@router.delete("/simulations/{sid}")
def delete_simulation(request: Request, sid: str = SID):
    m = _mgr(request)
    _call(m.delete, _call(m.get, sid))
    return {"deleted": sid}


@router.post("/simulations/{sid}/start")
async def start_simulation(request: Request, sid: str = SID):
    s = _call(_mgr(request).get, sid)
    if s.status != SimStatus.ready:
        raise HTTPException(409, f"Cannot start a simulation in state '{s.status.value}'")
    try:
        await request.app.state.runner.start(s)
    except SimulationError as exc:
        raise HTTPException(exc.code, str(exc)) from exc
    return s.public()


@router.post("/simulations/{sid}/pause")
def pause(request: Request, sid: str = SID):
    m = _mgr(request)
    s = _call(m.get, sid)
    _call(m.command, s, "pause")
    return {"id": sid, "queued": "pause"}


@router.post("/simulations/{sid}/resume")
def resume(request: Request, sid: str = SID):
    m = _mgr(request)
    s = _call(m.get, sid)
    _call(m.command, s, "resume")
    return {"id": sid, "queued": "resume"}


@router.post("/simulations/{sid}/reset")
async def reset(request: Request, sid: str = SID):
    m = _mgr(request)
    s = _call(m.get, sid)
    try:
        await m.reset(s)
    except SimulationError as exc:
        raise HTTPException(exc.code, str(exc)) from exc
    # a finished run had its monitor loop end -> restart monitoring
    if s.task is None or s.task.done():
        import asyncio
        s.task = asyncio.create_task(request.app.state.runner._run_monitor(s))
    return {"id": sid, "queued": "reset"}


@router.post("/simulations/{sid}/stop")
async def stop(request: Request, sid: str = SID):
    m = _mgr(request)
    s = _call(m.get, sid)
    await m.stop(s)
    return s.public()


@router.get("/simulations/{sid}/telemetry")
def telemetry(request: Request, sid: str = SID):
    s = _call(_mgr(request).get, sid)
    return {"id": sid, "status": s.status.value, "count": s.telemetry_count,
            "telemetry": s.telemetry, "trace": s.trace[-500:]}
