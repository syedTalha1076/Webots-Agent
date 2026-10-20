"""One entry point used by the CLI and the REST API: prompt -> parse -> validate -> generate -> (launch)."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Callable, Optional

from app.config import Settings
from app.pipeline.parser import ParseOutcome, parse_prompt
from app.pipeline.project import GenerationError, generate
from app.pipeline.runner import RunReport, run_world


@dataclass
class PipelineResult:
    stage: str = "parse"            # parse | validate | generate | launch | done
    ok: bool = False
    message: str = ""
    provider: str = ""
    notes: list[str] = field(default_factory=list)
    spec: Optional[dict] = None
    summary: Optional[dict] = None
    project: Optional[str] = None
    world: Optional[str] = None
    run: Optional[dict] = None

    def to_dict(self) -> dict:
        return asdict(self)


def run_pipeline(prompt: str, settings: Settings, launch: bool = True,
                 on_progress: Optional[Callable[[str], None]] = None) -> PipelineResult:
    say = on_progress or (lambda _m: None)
    res = PipelineResult()
    say("Interpreting the request...")
    out: ParseOutcome = parse_prompt(prompt, settings)
    res.provider, res.notes = out.provider, out.notes
    if out.spec is None:
        res.message = out.message or "Could not understand the request."
        return res
    res.spec = out.spec.model_dump(mode="json")
    res.stage = "generate"
    say(f"Spec ready ({out.provider}): {out.spec.robot.kind} / {out.spec.behavior.task} / {out.spec.environment.kind}. Validating and generating files...")
    try:
        gen = generate(out.spec, settings)
    except GenerationError as exc:
        res.stage = "validate" if str(exc).startswith("Specification rejected") else "generate"
        res.message = str(exc)
        return res
    res.project, res.world, res.summary = str(gen.project), str(gen.world), gen.spec_summary
    res.notes += gen.report.warnings + gen.notes
    if not launch:
        res.stage, res.ok = "done", True
        res.message = f"Project generated and validated (not launched): {gen.project}"
        return res
    res.stage = "launch"
    rep: RunReport = run_world(gen.world, gen.project, gen.robot_names, gen.spec.sim.timeout_s, settings,
                               on_progress=say)
    res.run = asdict(rep)
    res.ok = rep.ok
    res.stage = "done"
    res.message = rep.message
    return res
