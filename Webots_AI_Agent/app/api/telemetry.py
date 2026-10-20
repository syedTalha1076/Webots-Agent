"""Live channel: controller telemetry ingest (internal) and browser WebSocket."""
from __future__ import annotations

import secrets

from fastapi import APIRouter, Header, HTTPException, Path, Request, WebSocket, WebSocketDisconnect

from app.schemas import Telemetry
from app.simulation.manager import SimulationError

router = APIRouter()
SID = Path(pattern=r"^[a-f0-9]{12}$")


@router.post("/internal/sessions/{sid}/telemetry")
def ingest(tel: Telemetry, request: Request, sid: str = SID, x_session_token: str = Header(default="")):
    """Called by the Webots controller. The per-session token keeps sessions isolated from each other."""
    m = request.app.state.manager
    try:
        s = m.get(sid)
    except SimulationError:
        raise HTTPException(404, "unknown session")
    if not secrets.compare_digest(x_session_token, s.token):
        raise HTTPException(403, "bad session token")
    return {"commands": m.ingest(s, tel)}


@router.websocket("/ws/simulations/{sid}")
async def live(ws: WebSocket, sid: str):
    m = ws.app.state.manager
    s = m.sessions.get(sid)
    if s is None:
        await ws.close(code=4404)
        return
    await ws.accept()
    m.hub.add(sid, ws)
    try:
        await ws.send_json({"type": "snapshot", "session": s.public(), "trace": s.trace[-1000:]})
        while True:
            msg = await ws.receive_text()      # only keep-alives are accepted; clients cannot send commands here
            if msg == "ping":
                await ws.send_json({"type": "pong"})
    except WebSocketDisconnect:
        pass
    finally:
        m.hub.remove(sid, ws)
