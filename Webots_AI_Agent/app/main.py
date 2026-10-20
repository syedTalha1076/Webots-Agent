"""FastAPI application factory."""
from __future__ import annotations

import logging
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Optional

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from app.agent.graph import AgentRunner
from app.agent.llm import SpecParser, make_parser
from app.api import generate, simulations, telemetry
from app.config import Settings, get_settings
from app.simulation.manager import Launcher, SimulationManager, default_launcher

log = logging.getLogger("uvicorn.error")
STATIC = Path(__file__).parent / "static"


def create_app(settings: Optional[Settings] = None, launcher: Launcher = default_launcher,
               parser: Optional[SpecParser] = None, poll_s: float = 1.0) -> FastAPI:
    settings = settings or get_settings()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        settings.simulations_dir.mkdir(parents=True, exist_ok=True)
        st = settings.webots_status()
        (log.info if st["available"] else log.warning)("Webots: %s %s", st["message"], st["executable"] or "")
        log.info("LLM mode: %s", "Groq" if settings.llm_enabled else "rule-based fallback (no GROQ_API_KEY)")
        app.state.manager.start_watchdog()
        yield
        await app.state.manager.shutdown()

    app = FastAPI(title=settings.app_name, lifespan=lifespan)
    manager = SimulationManager(settings, launcher=launcher)
    p = parser or make_parser(settings)
    app.state.settings, app.state.manager = settings, manager
    app.state.runner = AgentRunner(manager, p, settings, poll_s)
    app.state.runner_parser_name = p.name
    app.include_router(generate.router)
    app.include_router(simulations.router)
    app.include_router(telemetry.router)
    app.mount("/", StaticFiles(directory=STATIC, html=True), name="static")
    return app


app = create_app()
