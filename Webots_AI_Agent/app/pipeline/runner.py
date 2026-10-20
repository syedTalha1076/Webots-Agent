"""Launch a generated world in Webots, capture its log, and report what REALLY happened.

Success is only reported when every robot's controller wrote a result file saying so. Webots log lines that
indicate a skipped/unavailable PROTO, a controller crash or an ERROR make the run fail (and are shown to the user).
"""
from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Optional

from app.config import Settings
from app.simulation.manager import default_launcher

ERROR_RE = re.compile(
    r"(\bERROR\b|Traceback \(most recent call last\)|Skipped|PROTO .*(unavailable|not found|cannot|failed)|"
    r"unavailable|could not (open|load|find|start)|cannot (open|load|find)|No module named|"
    r"(Syntax|Name|Import|Attribute|Type|Key|Index|Value)Error|world file .* (invalid|cannot))", re.I)
WARN_RE = re.compile(r"\bWARNING\b", re.I)
IGNORE_RE = re.compile(r"(libpng|Qt:|QStandardPaths|OpenGL.*warning|software rendering)", re.I)


@dataclass
class RunReport:
    status: str = "not_started"      # success | task_failed | webots_errors | timeout | exited | webots_not_found
    ok: bool = False
    message: str = ""
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    results: dict[str, dict] = field(default_factory=dict)
    log_path: Optional[str] = None
    command: list[str] = field(default_factory=list)
    pid: Optional[int] = None


def scan_log(text: str) -> tuple[list[str], list[str]]:
    errors, warnings = [], []
    for line in text.splitlines():
        line = line.strip()
        if not line or IGNORE_RE.search(line) or line.startswith("[RESULT]"):
            continue
        if ERROR_RE.search(line):
            errors.append(line[:300])
        elif WARN_RE.search(line):
            warnings.append(line[:300])
    return list(dict.fromkeys(errors)), list(dict.fromkeys(warnings))


def run_world(world: Path, project: Path, robot_names: list[str], timeout_s: float, settings: Settings,
              launcher: Callable[..., Any] = default_launcher, poll_s: float = 0.5,
              startup_s: float = 90.0, keep_open: Optional[bool] = None,
              on_progress: Optional[Callable[[str], None]] = None) -> RunReport:
    rep = RunReport()
    say = on_progress or (lambda _m: None)
    exe = settings.find_webots()
    if exe is None:
        rep.status = "webots_not_found"
        rep.message = ("Webots executable not found. Set WEBOTS_EXECUTABLE (e.g. "
                       r"G:\InstallSoftWare\Webots\msys64\mingw64\bin\webots.exe) or WEBOTS_HOME in .env.")
        rep.errors.append(rep.message)
        return rep
    keep_open = settings.keep_webots_open if keep_open is None else keep_open
    for old in project.glob("result_*.json"):
        old.unlink()
    log_path = project / "webots.log"
    cmd = [str(exe), f"--mode={settings.pipeline_run_mode}", "--batch", "--stdout", "--stderr", str(world)]
    rep.command, rep.log_path = cmd, str(log_path)
    try:
        proc = launcher(cmd, project, log_path)
    except OSError as exc:
        rep.status, rep.message = "webots_errors", f"Could not start Webots: {exc}"
        rep.errors.append(rep.message)
        return rep
    rep.pid = getattr(proc, "pid", None)
    say(f"Webots started (pid {rep.pid}); waiting for the controllers to finish...")
    start = time.time()
    deadline = start + startup_s + timeout_s * 1.6
    seen_err: list[str] = []

    def kill() -> None:
        import subprocess
        import sys
        try:
            if proc.poll() is None:
                if sys.platform == "win32":
                    subprocess.run(["taskkill", "/PID", str(proc.pid), "/T", "/F"], capture_output=True, timeout=10)
                else:
                    proc.terminate()
        except Exception:
            pass

    def read_log() -> str:
        try:
            return log_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return ""

    while True:
        time.sleep(poll_s)
        results = {}
        for nm in robot_names:
            f = project / f"result_{nm}.json"
            if f.exists():
                try:
                    results[nm] = json.loads(f.read_text(encoding="utf-8"))
                except json.JSONDecodeError:
                    pass
        errors, warnings = scan_log(read_log())
        rep.warnings = warnings
        new = [e for e in errors if e not in seen_err]
        if new:
            seen_err += new
            rep.errors = list(seen_err)
            for e in new:
                say(f"Webots reported: {e}")
        if results and len(results) == len(robot_names) and not rep.errors:
            rep.results = results
            rep.ok = all(r.get("success") for r in results.values())
            rep.status = "success" if rep.ok else "task_failed"
            rep.message = "; ".join(f"{k}: {v.get('reason')}" for k, v in results.items())
            break
        exited = proc.poll() is not None
        if rep.errors and (time.time() - start > 6 or exited):
            rep.status = "webots_errors"
            rep.message = "Webots reported errors; the simulation did not run correctly (see below / webots.log)."
            rep.results = results
            kill()
            break
        if exited:
            rep.status = "exited"
            rep.message = (f"Webots exited (code {proc.poll()}) before the controllers reported a result. "
                           f"Check {log_path}.")
            rep.errors = rep.errors or [rep.message]
            break
        if time.time() > deadline:
            rep.status = "timeout"
            rep.message = f"No result within {deadline - start:.0f} s wall-clock. Check {log_path}."
            rep.errors = rep.errors or [rep.message]
            kill()
            break
    rep.results = rep.results or {}
    if rep.status in ("success", "task_failed") and not keep_open:
        kill()
    return rep
