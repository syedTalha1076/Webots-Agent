# Text-to-Robot AI Simulation Agent

Describe a robot task in plain English. A LangGraph agent turns it into a **validated specification**, builds a
**Webots** world (e-puck robot, walls, box obstacles, red target), launches Webots with a Python controller, and
streams **real telemetry** into a browser dashboard.

* Works **without any API key** (rule-based parser). Add a free Groq key to let an LLM do the parsing.
* The LLM only fills a structured schema. It never writes code or shell commands.
* Motor control and safety run inside the Webots controller at every simulation step - never waiting on an LLM.
* Standalone: independent of your STAS GPT project.

---

## What has and has not been verified

| Verified here (automated, 73 tests passing) | **Not verified - needs your Webots install** |
|---|---|
| Spec validation, parsers, A* planner, navigator logic (on a kinematic stand-in robot with IR ray-casting) | Launching the real `webots.exe` and opening the generated world |
| FastAPI endpoints, WebSocket, session isolation, e-stop priority, watchdog, cleanup | `webots://` PROTO resolution on your Webots version |
| LangGraph workflow incl. clarification, replanning and give-up paths | E-puck device names / IR geometry matching my assumptions |
| Controller <-> backend HTTP protocol (using the real Navigator) | Real physics behaviour (wheel slip, box pushing, IR noise) |
| Real uvicorn server serves the dashboard and API | Visual check of the Three.js dashboard in a browser |

The controller file compiles, but it has **never run inside Webots**. Follow *Section 8* to check it. If something differs
(for example a PROTO URL), the Troubleshooting section lists the likely fixes.

---

## 1. Install Webots (free, open source)

1. Download the Windows installer from <https://cyberbotics.com> (Download -> Webots, latest release, e.g. R2025a).
2. Run it with the default options (installs to `C:\Program Files\Webots`; the program itself may live on C: - only
   your project files go to another drive).
3. Start Webots once, so it finishes first-time setup. Close it.

## 2. Check Python (Command Prompt)

```bat
python --version
```
Use Python 3.10 - 3.12. The backend runs in a venv. The **Webots controller** uses whichever Python Webots finds
(Webots > Tools > Preferences > Python command). The controller only needs the standard library, so any Python that
Webots supports works. If Webots reports "Python not found" or a version mismatch, set `WEBOTS_PYTHON_COMMAND` in `.env`
(see Section 5).

## 3. Put the project on another drive (example: D:)

Unzip `Webots_AI_Agent.zip` so you get `D:\Webots_AI_Agent\app`, `D:\Webots_AI_Agent\controllers`, ...

```bat
D:
cd \Webots_AI_Agent
```

## 4. Virtual environment and dependencies (Command Prompt)

```bat
python -m venv .venv
.venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt
```
PowerShell instead? Activate with `.venv\Scripts\Activate.ps1`. If PowerShell blocks scripts, run once:
`Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`.

## 5. Configure `.env`

```bat
copy .env.example .env
notepad .env
```
Set at least **one** of these:

```
WEBOTS_EXECUTABLE=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe
# or
WEBOTS_HOME=C:\Program Files\Webots
```
Optional:

| Variable | Meaning |
|---|---|
| `GROQ_API_KEY`, `GROQ_MODEL` | Enables the LLM parser (blank model = `llama-3.3-70b-versatile`). Empty key = rule-based mode |
| `WEBOTS_PROJECT_DIR` | Folder whose `simulations\` subfolder receives generated worlds (default: project folder) |
| `WEBOTS_VERSION` | World-file header, default `R2025a`. Set it to your installed release (e.g. `R2023b`) |
| `WEBOTS_MODE` | `realtime` (watch it) or `fast` |
| `WEBOTS_PYTHON_COMMAND` | Full path to python.exe for controllers (written to each session's `runtime.ini`) |
| `SIMULATION_TIMEOUT` | Max simulated seconds per run (default 300) |
| `MAX_ROBOT_COUNT` | Max simultaneously running simulations (default 5) |

Never commit `.env` (already in `.gitignore`).

## 6. Run the backend and open the dashboard

```bat
python -m uvicorn app.main:app --host 127.0.0.1 --port 8000
```
Open <http://127.0.0.1:8000>. The header badges show whether Webots was found and which parser is active.

## 7. Run the automated tests

```bat
python -m pytest -q
```

## 8. First run: step-by-step verification

**8a. Controller + world on their own (Milestone 1, no backend):**
1. Start Webots, **File > Open World...** -> `D:\Webots_AI_Agent\worlds\sample_navigation.wbt`.
2. Expected: a 2x2 m arena, four boxes, a red disc in one corner, an e-puck in the opposite corner.
3. Press Play. The e-puck drives to the red disc, steers around boxes, and stops on it. The Webots console (bottom)
   prints lines like `[nav t=12.3s] obstacle detected ...` and finally `RESULT: {'success': True ...}`.
4. If it fails, see Troubleshooting. Do this before using the dashboard.

**8b. Whole platform:**
1. Dashboard -> pick an example prompt -> **Create simulation**. The agent log shows parse -> validate -> plan ->
   build world, and the 3D view shows the generated room.
2. Click **Start**. Webots opens the generated world (`simulations\<id>\worlds\world.wbt`).
3. Within seconds the badge says "Live link: connected", the telemetry panel fills with numbers, and the robot in the
   dashboard moves exactly like the one in Webots.
4. When the robot arrives, the **Result** panel shows measured and estimated values separately.

**Verify controller communication:** the event log must contain "Controller connected - receiving live telemetry".
Also `http://127.0.0.1:8000/api/simulations/<id>/telemetry` returns a rising `count`.

**Test obstacle avoidance:** use *"robot in a 2 m room with 1 obstacle"* or put a box on the straight line with
*"start at (-0.7, 0), target at (0.7, 0)"* and watch for `obstacle detected` events and a non-zero
"Obstacle encounters" in the result.

**Stop / reset:**
* **Emergency stop** - highest priority. The controller halts the motors and closes *its own* Webots instance; if it does
  not exit within 4 s the backend kills only that session's process tree (`taskkill /PID <pid> /T /F`). Other
  Webots windows are untouched.
* **Reset** - teleports the robot back to the start pose, clears metrics, restarts navigation (Webots stays open).
* **Pause** - the robot halts; Webots' simulation clock keeps running (the world is not paused - see limitations).

## 9. Architecture

```
Browser  <-- WebSocket / REST -->  FastAPI (app/)  <-- HTTP telemetry (token) -->  Webots controller
Dashboard                          LangGraph agent                                 (Supervisor + e-puck)
Three.js                           SimulationManager (subprocess)
```
* `app/agent/` - LangGraph: `parse_request -> validate_spec -> (clarify | plan -> build_world)` and, on Start,
  `launch -> monitor <-> evaluate -> (replan) -> report`. Parsers in `llm.py` (rule-based, Groq).
* `app/simulation/` - `world_generator.py` (templates only), `manager.py` (lifecycle, watchdog, results),
  `controller_bridge.py` (command queue with e-stop priority + WebSocket hub), `navigation.py` (odometry, A*, navigator),
  `registry.py` (robot models).
* `controllers/navigation_controller/` - the Webots controller. At world-generation time it is **copied** into the
  session folder together with `navigation.py`, so each session is a self-contained Webots project.
* Navigation uses wheel odometry + IR sensors. Supervisor ground truth is used only for telemetry/evaluation.
* If the backend is unreachable for 4 s, the robot **stops** and waits (safe stop on communication loss).
* Stuck detection happens in the controller; the agent then sends a `replan` (back off, pass obstacles on the other
  side, slightly slower), up to 3 times, then reports failure.

### API
```
POST   /api/simulations                  create + plan (body: prompt, auto_start)
GET    /api/simulations[/{id}]           list / details
POST   /api/simulations/{id}/start|pause|resume|reset|stop
GET    /api/simulations/{id}/telemetry   latest telemetry + trace
DELETE /api/simulations/{id}             delete files (must be stopped)
GET    /api/health, /api/examples
WS     /ws/simulations/{id}              snapshot, telemetry, event, status, result
POST   /internal/sessions/{id}/telemetry controller -> backend (needs X-Session-Token)
```

## 10. Webots modes

The controller needs only the regular controller API plus Supervisor API (`getSelf`, `getPosition`, field setters,
`simulationQuit`). These work in realtime and fast modes, and with the standard GUI. `WEBOTS_MODE=fast` speeds up the
run; pure `--no-rendering`/headless use is possible but untested here. Pausing the Webots world itself from the
backend is intentionally not used, because a paused world would also block the controller from receiving commands.

## 11. Adding things later

**New robot type**
1. Add a `RobotModel` in `app/simulation/registry.py` (PROTO name/URL, wheel radius, axle length, speed, sensors).
2. Add its key to `RobotType` in `app/schemas.py`.
3. If the device names differ from the e-puck, write a new controller folder (copy `navigation_controller`), set
   `controller=` in the model, and keep the same telemetry JSON. `world_generator._world_text` is the single place
   that writes the robot node.

**New task** (e.g. `follow_line`)
1. Add the task name to `RobotSpec.task` (schemas) and `SUPPORTED_TASKS` (registry).
2. Add fields it needs to the spec and validate them.
3. Add the behaviour to the navigator/controller; extend `plan` and `build_result`.
4. Teach the parsers (`prompts.py`, `llm.py`) and add tests.

**New LLM provider:** implement `SpecParser.parse` (returns `ParseResult`) and return it from `make_parser`.
Every parser's output still goes through `RobotSpec`, which is the only authority.

**Custom robot geometry** (later): add a world template that emits a custom PROTO from `protos/` and register it
as a new `RobotModel`.

## 12. Limitations (read these)

* Simulated physics and IR sensors are approximations: e-puck IR range is only about 7 cm, readings are noisy,
  and real hardware will differ. Odometry drifts, so success is judged on Supervisor ground truth (<= 0.15 m).
* "Collisions" are counted from ground-truth positions against approximate box geometry, so they are measured in
  the simulator but not physically exact. "Obstacle encounters" come from IR triggers.
* The browser view is a **synchronized visualization** of real telemetry (about 8 updates/s), not a Webots video
  stream. Webots' own window is the true 3D view. Telemetry shows ground-truth position, not what the robot "knows".
* Room limits: 1-10 m, up to 20 obstacles, speed <= 6.28 rad/s (about 0.13 m/s). A 10 m room takes minutes.
* Only the e-puck / differential-drive robot and the `navigate_to_target` task exist so far.
* Local development design: the session token protects the controller endpoint, but there is no user login yet.
  Add authentication, HTTPS and per-user session ownership before exposing it beyond localhost.

## 13. Troubleshooting

| Symptom | Fix |
|---|---|
| Badge "Webots: not found" | Set `WEBOTS_EXECUTABLE` to the full path of `webots.exe` (search `C:\Program Files\Webots` for it), restart uvicorn |
| Webots opens but errors about `EXTERNPROTO` / PROTO not found | Set `WEBOTS_VERSION` to your installed release. If `webots://` is not resolved, replace URLs in `app/simulation/registry.py` / `world_generator.py` with `https://raw.githubusercontent.com/cyberbotics/webots/<release>/projects/...` |
| World loads but the robot does not move, console shows Python errors | Set Webots > Tools > Preferences > Python command, or `WEBOTS_PYTHON_COMMAND` in `.env`. Read `simulations\<id>\webots.log` |
| "Controller never connected" after 90 s | Controller cannot reach `http://127.0.0.1:8000`. Check the port in `.env`, firewall, and `webots.log` |
| Robot spins or drives the wrong way | Check the Webots version is ENU (R2022b+). Older versions need a rotated world |
| Robot stops immediately, state `comm_lost` | Backend stopped or unreachable; restart it, create a new simulation |
| Robot gets stuck, `replan` events | Expected in dense rooms; reduce obstacles or enlarge the room |
| `Address already in use` | Another program uses the port: change `PORT` or `--port` |
| Dashboard 3D area blank | CDN scripts for Three.js blocked; use the **Top view** tab (works offline) |

## 14. Free tools / optional paid items

Free and open source: Webots, Python, FastAPI, Uvicorn, Pydantic, LangGraph, LangChain, Three.js, pytest.
Optional external service: Groq API (has a free tier; the project works fully without it).

## 15. Milestone map

M1 sample world + controller (`worlds/`, `controllers/`) · M2 backend + dashboard (`app/main.py`, `app/static/`) ·
M3 telemetry bridge (`controller_bridge.py`, `/internal/...`) · M4 navigation (`navigation.py`) ·
M5 agent (`app/agent/`) · M6 world generation (`world_generator.py`) · M7 live view + metrics + errors ·
M8 tests + this README.
