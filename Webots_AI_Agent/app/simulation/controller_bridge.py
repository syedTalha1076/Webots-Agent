"""Backend side of the controller link: per-session command queue (e-stop priority) and WebSocket hub."""
from __future__ import annotations

import asyncio
from collections import deque
from typing import Any

ALLOWED_COMMANDS = {"stop", "pause", "resume", "reset", "replan"}


class CommandQueue:
    """Commands are delivered in the HTTP reply to the controller's next telemetry post.
    `stop` (emergency stop) clears everything else and is never queued behind other commands."""

    def __init__(self) -> None:
        self._q: deque[dict[str, Any]] = deque()
        self.estop = False

    def push(self, name: str, **kwargs: Any) -> None:
        if name not in ALLOWED_COMMANDS:
            raise ValueError(f"unsupported controller command: {name}")
        if self.estop:
            return                      # after an e-stop nothing else may be sent
        if name == "stop":
            self.estop = True
            self._q.clear()
        self._q.append({"name": name, **kwargs})

    def drain(self) -> list[dict[str, Any]]:
        items = list(self._q)
        self._q.clear()
        return items

    def clear(self) -> None:
        self._q.clear()


class Hub:
    """Fan-out of live messages to the browser WebSockets of ONE session (sessions never share channels)."""

    def __init__(self) -> None:
        self._conns: dict[str, set[Any]] = {}
        self._loop: asyncio.AbstractEventLoop | None = None

    def add(self, sid: str, ws: Any) -> None:
        self._loop = asyncio.get_running_loop()      # remember the server loop for publishes from worker threads
        self._conns.setdefault(sid, set()).add(ws)

    def remove(self, sid: str, ws: Any) -> None:
        self._conns.get(sid, set()).discard(ws)

    def count(self, sid: str) -> int:
        return len(self._conns.get(sid, ()))

    async def broadcast(self, sid: str, message: dict) -> None:
        for ws in list(self._conns.get(sid, ())):
            try:
                await ws.send_json(message)
            except Exception:           # client went away; drop it
                self.remove(sid, ws)

    def publish(self, sid: str, message: dict) -> None:
        """Fire-and-forget from sync code, including FastAPI worker threads (thread-safe)."""
        if not self._conns.get(sid):
            return
        try:
            asyncio.get_running_loop().create_task(self.broadcast(sid, message))
        except RuntimeError:
            if self._loop is not None and self._loop.is_running():
                asyncio.run_coroutine_threadsafe(self.broadcast(sid, message), self._loop)
