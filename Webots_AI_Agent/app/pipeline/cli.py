"""Command line:  python -m app.pipeline "prompt" [--no-launch] [--json] [--check]"""
from __future__ import annotations

import argparse
import json
import sys

from app.config import get_settings
from app.pipeline.parser import ollama_status
from app.pipeline.service import run_pipeline


def doctor(settings) -> int:
    print("== Environment check ==")
    res = settings.asset_resolver()
    ver, how = res.detect_version()
    print(f"Webots home      : {settings.webots_home or '(not set)'}")
    print(f"Webots version   : {ver}  ({how})")
    exe = settings.find_webots()
    print(f"Webots executable: {exe or 'NOT FOUND'}")
    print(f"Simulations dir  : {settings.simulations_dir}")
    for p in res.installation_problems():
        print("  !", p)
    from app.webots_assets import CATALOG
    bad = 0
    for k in CATALOG:
        a = res.resolve(k)
        if a.verified:
            tag = "local "
        elif a.online and res.online_exists(a.url):
            tag = "online"          # not on disk, but exists at the pinned GitHub URL Webots will download
        else:
            tag = "MISSING"
            bad += 1
        print(f"  [{tag}] {a.node:24s} {a.url}")
    ok, why = ollama_status(settings)
    print(f"Ollama           : {why}")
    print(f"Groq key present : {bool(settings.groq_api_key.strip())}   provider setting: {settings.llm_provider}")
    return 1 if (bad or exe is None) else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="python -m app.pipeline", description="Natural language -> Webots simulation")
    ap.add_argument("prompt", nargs="?", help="what to simulate (quote it)")
    ap.add_argument("--no-launch", action="store_true", help="generate + validate only")
    ap.add_argument("--json", action="store_true", help="print the full result as JSON")
    ap.add_argument("--check", action="store_true", help="verify the Webots install, assets and LLM options, then exit")
    a = ap.parse_args(argv)
    settings = get_settings()
    if a.check:
        return doctor(settings)
    if not a.prompt:
        ap.error("a prompt is required (or use --check)")
    res = run_pipeline(a.prompt, settings, launch=not a.no_launch, on_progress=lambda m: print("..", m, flush=True))
    if a.json:
        print(json.dumps(res.to_dict(), indent=2))
    else:
        print(f"\nParser: {res.provider}")
        for n in res.notes:
            print("  note:", n)
        if res.summary:
            d = res.summary["derived"]
            print(f"Robot: {d['robot_type']} x{d['count']}, {d['motor_count']} motors, sensors {d['sensors']}")
            print(f"Project: {res.project}\nWorld  : {res.world}")
        if res.run:
            for e in res.run["errors"]:
                print("  ERROR:", e)
            print("Log    :", res.run["log_path"])
        print(("\nSUCCESS: " if res.ok else f"\nFAILED at stage '{res.stage}': ") + res.message)
    return 0 if res.ok else 1


if __name__ == "__main__":
    sys.exit(main())
