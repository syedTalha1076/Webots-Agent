# Prompt -> Webots simulation pipeline

```
prompt --> parser (Ollama | Groq | rules) --> SimSpec (JSON, validated) --> project folder --> pre-launch checks --> Webots --> report
```
Code: `app/pipeline/` (spec, parser, worldgen, project, checks, runner, service, cli) and `app/webots_assets.py`.
Controllers copied into every project come from `app/pipeline/controller_templates/`.
Your original app (FastAPI + LangGraph e-puck navigation + telemetry UI) is unchanged and still works.

## What was wrong with the assets, and the fix
* The archive only contained `worlds/sample_navigation.wbt` (it already used `webots://projects/...` URLs, which are the
  form Webots' own R2025a PROTOs use). I could not see your generated world with `G:\...` paths nor your installation, so:
* `app/webots_assets.py` now **reads `WEBOTS_HOME\resources\version.txt`**, **checks every PROTO file exists** under
  `WEBOTS_HOME\projects\...` and writes `EXTERNPROTO "webots://projects/<path>"` (never `G:\...`). If a file is missing,
  nothing is generated and the exact missing path is printed. `ASSET_URL_MODE=github` pins the detected version URL instead.
* The `#VRML_SIM` header uses the detected installed version.
* Also fixed: your `.env` pointed to `webotsw.exe`, which the code did not accept, so Webots was "not found". It is accepted now
  (and `webots.exe` next to it is preferred because its console output can be logged).
* Nothing inside the Webots folder is modified. Projects go to `WEBOTS_PROJECT_DIR\simulations\<timestamp>_<slug>_<id>\`.

Run `python -m app.pipeline --check` first: it prints version, executable, each asset (ok/MISSING), Ollama and Groq status.

## Supported (anything else is refused with an explanation)
| Robot | Tasks | Environment |
|---|---|---|
| `epuck` (built-in E-puck, 2 motors, 8 IR sensors) | wander + avoid, go to targets | arena, warehouse |
| `custom_wheeled` (generated; 2 wheels + casters, or 4-wheel skid steer; custom length/width/height/wheel radius; 5 distance sensors; line camera) | wander + avoid, go to targets, line following | arena, warehouse, line track |
| `arm` (generated 3-joint arm, cube, table) | pick and place (magnetic attach grasp) | arm table |
| `drone` (built-in Mavic 2 Pro) | fly waypoints | open field |
Multi-robot: up to 8 wheeled robots. Not supported: humanoids, legged, underwater, steering cars, multi-arm cells, SLAM.

## What is verified vs. not (please read)
Verified here (no Webots in my environment): PROTO files exist in the R2025a source tree; spec validation; generated `.wbt`
structure (brace balance, EXTERNPROTO coverage, DEF names, controller name/folder/config); controller Python compiles; controller
logic runs against a 2D kinematic mock (e-puck/custom nav, line following, wander, warehouse, arm IK checked against independent
forward kinematics, pick-and-place state machine); runner/error reporting with a fake Webots. 105 tests pass.
**Not verified:** actual physics inside Webots. Possible tuning needed: custom robot friction/caster behaviour, line-camera
brightness threshold, arm joint tuning, and especially the **drone (experimental)** controller. Position/pose used by controllers is
Supervisor ground truth (stated in each result). If Webots shows a problem, the runner prints the log lines and fails instead of
claiming success; send me `webots.log` from the project folder.

## Windows commands (PowerShell or cmd)
```
cd /d G:\path\to\Webots_AI_Agent
py -3.13 -m venv .venv            (or your installed 3.11-3.13)
.venv\Scripts\activate
pip install -r requirements.txt
python -m app.pipeline --check
python -m pytest -q
python -m app.pipeline "Wheeled robot in a 3 x 3 m room with 5 obstacles, target at (1.0, 1.0)"
python -m app.pipeline "your prompt" --no-launch        (generate + validate only)
uvicorn app.main:app --host 127.0.0.1 --port 8000       (then open http://127.0.0.1:8000/generate.html)
```
Free local LLM (optional): `winget install Ollama.Ollama`, then `ollama pull llama3.1:8b`. `LLM_PROVIDER=auto` uses Ollama when
running, else Groq if `GROQ_API_KEY` is set (yours was in a commented line of the old `.env`; I activated it), else rules.
Webots uses Python from PATH for controllers (stdlib only); set `WEBOTS_PYTHON_COMMAND` or Webots > Preferences > Python command if needed.
**Security:** the old `.env` contained real-looking Groq and Google keys. If the zip was ever shared, rotate them.

## Example prompts
1. Wheeled robot in a 3 x 3 m room with 5 obstacles, target at (1.0, 1.0)
2. A four wheel robot 40 cm long and 25 cm wide goes to (2.0, 1.0) in a 5 x 5 m room with 6 obstacles
3. A custom robot follows a black line track for 2 laps
4. Two e-puck robots wander and avoid obstacles in a 4 m room for 60 seconds
5. Warehouse 10 x 8 m with 4 shelves and 3 robots delivering boxes from (-3,-2) to (3, 2)
6. Robotic arm picks a cube at (0.3, 0.1) and places it at (0.2, -0.3)
7. Drone flies at an altitude of 1.5 meters through waypoints (2,0) (2,2) (0,2)
8. build a humanoid that walks   (must be refused with an explanation)
