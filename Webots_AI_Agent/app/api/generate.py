"""REST entry for the general prompt-to-simulation pipeline."""
from __future__ import annotations

from fastapi import APIRouter, Request
from pydantic import BaseModel, Field

from app.pipeline.parser import ollama_status
from app.pipeline.service import run_pipeline

router = APIRouter(prefix="/api")

EXAMPLES = [
    "Wheeled robot in a 3 x 3 m room with 5 obstacles, target at (1.0, 1.0)",
    "A four wheel robot 40 cm long and 25 cm wide goes to (2.0, 1.0) in a 5 x 5 m room with 6 obstacles",
    "A custom robot follows a black line track for 2 laps",
    "Two e-puck robots wander and avoid obstacles in a 4 m room for 60 seconds",
    "Warehouse 10 x 8 m with 4 shelves and 3 robots delivering boxes from (-3,-2) to (3, 2)",
    "Robotic arm picks a cube at (0.3, 0.1) and places it at (0.2, -0.3)",
    "Drone flies at an altitude of 1.5 meters through waypoints (2,0) (2,2) (0,2)",
]


class GenerateRequest(BaseModel):
    prompt: str = Field(..., min_length=3, max_length=2000)
    launch: bool = True


@router.post("/generate")
def generate(body: GenerateRequest, request: Request):
    """Sync handler -> runs in a worker thread (launching waits for the simulation)."""
    return run_pipeline(body.prompt.strip(), request.app.state.settings, launch=body.launch).to_dict()


@router.get("/generate/examples")
def examples():
    return EXAMPLES


@router.get("/generate/doctor")
def doctor(request: Request):
    s = request.app.state.settings
    res = s.asset_resolver()
    ver, how = res.detect_version()
    return {"webots": s.webots_status(), "version": ver, "version_source": how,
            "assets": {k: {"url": res.resolve(k).url, "exists": res.resolve(k).verified} for k in __import__("app.webots_assets", fromlist=["CATALOG"]).CATALOG},
            "ollama": dict(zip(("ok", "detail"), ollama_status(s))), "groq_key_set": bool(s.groq_api_key.strip()),
            "provider": s.llm_provider}
