"""Application settings loaded from environment / .env (no hardcoded install paths)."""
from __future__ import annotations

import shutil
import sys
from functools import lru_cache
from pathlib import Path

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

ROOT_DIR = Path(__file__).resolve().parent.parent
APPROVED_EXECUTABLE_NAMES = {"webots", "webots.exe", "webotsw.exe", "webots-bin", "webots-bin.exe"}


class Settings(BaseSettings):
    model_config = SettingsConfigDict(env_file=ROOT_DIR / ".env", extra="ignore")

    app_name: str = "Webots_AI_Robotics_Agent"
    app_env: str = "development"
    host: str = "127.0.0.1"
    port: int = 8000

    groq_api_key: str = ""
    groq_model: str = ""

    # --- prompt-to-simulation pipeline (app/pipeline) ---
    llm_provider: str = "auto"                  # auto | ollama | groq | rules
    ollama_base_url: str = "http://127.0.0.1:11434"
    ollama_model: str = "llama3.1:8b"
    ollama_timeout: int = Field(120, ge=5, le=900)
    asset_url_mode: str = "webots"              # webots | github | local   (see app/webots_assets.py)
    pipeline_run_mode: str = "realtime"         # realtime | fast
    keep_webots_open: bool = True               # leave Webots open after the controller reports a result

    webots_home: str = ""
    webots_executable: str = ""
    webots_project_dir: str = ""

    simulation_timeout: int = Field(300, ge=10, le=3600)
    max_robot_count: int = Field(5, ge=1, le=50)

    webots_version: str = "R2025a"
    webots_mode: str = "realtime"
    webots_python_command: str = ""

    @property
    def simulations_dir(self) -> Path:
        base = Path(self.webots_project_dir) if self.webots_project_dir else ROOT_DIR
        return (base / "simulations").resolve()

    @property
    def controller_src_dir(self) -> Path:
        return ROOT_DIR / "controllers" / "navigation_controller"

    @property
    def llm_enabled(self) -> bool:
        return bool(self.groq_api_key.strip())

    def find_webots(self) -> Path | None:
        """Resolve the Webots executable from settings, WEBOTS_HOME or PATH."""
        candidates: list[Path] = []
        if self.webots_executable:
            candidates.append(Path(self.webots_executable))
        if self.webots_home:
            home = Path(self.webots_home)
            candidates += [
                home / "msys64" / "mingw64" / "bin" / "webots.exe",
                home / "webots.exe",
                home / "webots",
                home / "bin" / "webots",
                home / "Contents" / "MacOS" / "webots",
            ]
        found = shutil.which("webots")
        if found:
            candidates.append(Path(found))
        if sys.platform == "win32":
            candidates.append(Path(r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"))
        for c in candidates:
            if c.is_file() and c.name.lower() in APPROVED_EXECUTABLE_NAMES:
                c = c.resolve()
                # webotsw.exe is the windowless launcher: its console output cannot be captured,
                # so use the sibling webots.exe when it exists (same program, keeps webots.log useful).
                console = c.with_name("webots.exe")
                if c.name.lower() == "webotsw.exe" and console.is_file():
                    return console
                return c
        return None

    def asset_resolver(self):
        from app.webots_assets import AssetResolver
        return AssetResolver(self.webots_home or None, self.webots_version, self.asset_url_mode)

    def webots_status(self) -> dict:
        exe = self.find_webots()
        return {
            "available": exe is not None,
            "executable": str(exe) if exe else None,
            "message": "Webots found." if exe else
            "Webots not found. Set WEBOTS_EXECUTABLE or WEBOTS_HOME in .env (see README).",
        }


@lru_cache
def get_settings() -> Settings:
    return Settings()
