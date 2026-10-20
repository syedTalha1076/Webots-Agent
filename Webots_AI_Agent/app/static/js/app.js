/* Dashboard controller: REST for commands, WebSocket for live updates (auto-reconnect). */
(function () {
  const $ = id => document.getElementById(id);
  const STAGES = ['parse_request', 'validate_spec', 'plan', 'build_world', 'ready', 'launch', 'monitor', 'replan', 'report'];
  const TERMINAL = ['finished', 'stopped', 'failed', 'timeout'];
  const S = { id: null, status: null, ws: null, retry: 0, timer: null };

  async function api(path, opts = {}) {
    const r = await fetch(path, { headers: { 'Content-Type': 'application/json' }, ...opts });
    const body = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(body.detail ? (typeof body.detail === 'string' ? body.detail : JSON.stringify(body.detail)) : r.statusText);
    return body;
  }
  const hint = (msg, cls = '') => { const h = $('hint'); h.textContent = msg; h.className = 'hint ' + cls; };
  const badge = (id, text, cls) => { const b = $(id); b.textContent = text; b.className = 'badge ' + (cls || ''); };

  function setButtons() {
    const st = S.status;
    $('btn-start').disabled = st !== 'ready';
    $('btn-pause').disabled = !['running'].includes(st);
    $('btn-resume').disabled = st !== 'paused';
    $('btn-reset').disabled = !['running', 'paused', 'finished'].includes(st);
    $('btn-stop').disabled = !['starting', 'running', 'paused'].includes(st);
  }

  function setStage(stage) {
    const idx = STAGES.indexOf(stage);
    $('stages').innerHTML = STAGES.map((s, i) =>
      `<li class="${i === idx ? 'now' : (idx > i || (S.status && S.status !== 'planning' && i < 5) ? 'done' : '')}">${s.replace('_', ' ')}</li>`).join('');
  }

  function showPlan(session) {
    if (!session.plan) { $('plan').innerHTML = ''; return; }
    const p = session.plan, sp = session.spec;
    $('plan').innerHTML = `<div><b>Robot:</b> ${p.robot_model}</div><div><b>Objective:</b> ${p.objective}</div>` +
      `<div><b>Navigation:</b> ${p.navigation}</div><div><b>Room:</b> ${sp.world.width}×${sp.world.height} m, ${session.world ? session.world.obstacles.length : sp.world.obstacles} obstacles, speed ${sp.max_speed} rad/s</div>`;
  }

  function addEvent(ev) {
    const li = document.createElement('li');
    const d = new Date(ev.time * 1000);
    li.innerHTML = `<time>${d.toLocaleTimeString()}</time><span class="${ev.kind}"></span>`;
    li.lastChild.textContent = ev.message;
    const log = $('log'); log.prepend(li);
    while (log.children.length > 150) log.lastChild.remove();
  }

  function showResult(r) {
    if (!r) { $('result-card').hidden = true; return; }
    const m = r.measured || {}, e = r.estimated || {};
    const row = (k, v, u = '') => `<tr><td>${k}</td><td>${v === null || v === undefined ? '–' : (typeof v === 'number' ? +v.toFixed(3) : v) + u}</td></tr>`;
    $('result-card').hidden = false;
    $('result').innerHTML = `<p class="verdict ${r.success ? 'good' : 'bad'}">${r.success ? 'Task completed' : 'Task failed'}</p><p>${r.reason}</p>
      <table><caption class="muted" style="text-align:left">Measured in the simulator</caption>
      ${row('Simulation time', m.simulation_time_s, ' s')}${row('Final distance to target', m.final_distance_to_target_m, ' m')}
      ${row('Distance travelled', m.distance_travelled_m, ' m')}${row('Collisions', m.collisions)}
      ${row('Obstacle encounters (IR)', m.obstacle_encounters)}${row('Replanning events', m.replanning_events)}${row('Wall-clock time', m.wall_clock_s, ' s')}</table>
      <table><caption class="muted" style="text-align:left">Estimated (derived, not measured)</caption>
      ${row('Straight-line distance', e.straight_line_distance_m, ' m')}${row('Path efficiency', e.path_efficiency)}</table>
      <p class="note">${r.notes || ''}</p>`;
  }

  function applyStatus(status, stage) {
    S.status = status; setButtons();
    if (stage) setStage(stage === 'navigating' ? 'monitor' : stage === 'paused' ? 'monitor' : stage);
    if (status === 'needs_clarification') hint('The agent needs more detail — see the activity log.', 'ask');
  }

  function loadSession(session, trace) {
    S.id = session.id; S.status = session.status;
    $('log').innerHTML = ''; session.events.forEach(addEvent);
    showPlan(session); showResult(session.result);
    if (session.world) Viewer.setWorld(session.world);
    if (trace) Viewer.trace = trace;
    if (session.telemetry) { Telemetry.render(session.telemetry); Viewer.update(session.telemetry, trace); }
    applyStatus(session.status, session.stage);
    if (session.clarification) hint(session.clarification, 'ask');
  }

  // ---------------------------------------------------------------- live channel
  function connect() {
    if (!S.id) return;
    clearTimeout(S.timer);
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    const ws = new WebSocket(`${proto}://${location.host}/ws/simulations/${S.id}`);
    S.ws = ws;
    ws.onopen = () => { S.retry = 0; badge('b-conn', 'Live link: connected', 'ok'); ws._ping = setInterval(() => ws.readyState === 1 && ws.send('ping'), 15000); };
    ws.onmessage = e => {
      const m = JSON.parse(e.data);
      if (m.type === 'snapshot') loadSession(m.session, m.trace);
      else if (m.type === 'telemetry') { Telemetry.render(m.telemetry); Viewer.update(m.telemetry); }
      else if (m.type === 'event') addEvent(m.event);
      else if (m.type === 'status') applyStatus(m.status, m.stage);
      else if (m.type === 'result') showResult(m.result);
    };
    ws.onclose = () => {
      clearInterval(ws._ping);
      if (S.ws !== ws) return;
      if (S.status && TERMINAL.includes(S.status)) { badge('b-conn', 'Live link: closed', ''); return; }
      badge('b-conn', 'Live link: reconnecting…', 'warn');
      S.timer = setTimeout(connect, Math.min(5000, 500 * 2 ** S.retry++));
    };
  }

  // ---------------------------------------------------------------- actions
  async function create() {
    const prompt = $('prompt').value.trim();
    if (prompt.length < 3) { hint('Describe the robot and task first.', 'err'); return; }
    $('btn-create').disabled = true; hint('Planning…');
    try {
      if (S.ws) { const w = S.ws; S.ws = null; w.close(); }
      const session = await api('/api/simulations', { method: 'POST', body: JSON.stringify({ prompt, auto_start: $('auto').checked }) });
      Viewer.clearTrace();
      loadSession(session);
      connect();
      if (session.status === 'ready') hint('World ready. Press Start to launch Webots.');
      else if (session.status !== 'needs_clarification') hint('');
    } catch (e) { hint(e.message, 'err'); }
    $('btn-create').disabled = false;
  }

  function act(name, label) {
    return async () => {
      if (!S.id) return;
      try { hint(label + '…'); await api(`/api/simulations/${S.id}/${name}`, { method: 'POST' }); hint(''); if (name === 'reset') { Viewer.clearTrace(); $('result-card').hidden = true; } }
      catch (e) { hint(e.message, 'err'); }
    };
  }

  async function init() {
    Viewer.init(); Telemetry.init(); setStage(''); setButtons();
    $('btn-create').onclick = create;
    $('btn-start').onclick = act('start', 'Launching Webots');
    $('btn-pause').onclick = act('pause', 'Pausing');
    $('btn-resume').onclick = act('resume', 'Resuming');
    $('btn-reset').onclick = act('reset', 'Resetting');
    $('btn-stop').onclick = act('stop', 'Stopping');
    try {
      const h = await api('/api/health');
      badge('b-webots', h.webots.available ? 'Webots: found' : 'Webots: not found', h.webots.available ? 'ok' : 'bad');
      $('b-webots').title = h.webots.message;
      badge('b-llm', h.llm.enabled ? 'Parser: Groq LLM' : 'Parser: rule-based (no API key)', h.llm.enabled ? 'ok' : 'warn');
      if (!h.webots.available) hint(h.webots.message, 'err');
    } catch { badge('b-webots', 'Backend unreachable', 'bad'); }
    try {
      const ex = await api('/api/examples');
      $('examples').innerHTML = ex.map((e, i) => `<button data-i="${i}">${e.title}</button>`).join('');
      $('examples').onclick = ev => { const i = ev.target.dataset.i; if (i !== undefined) $('prompt').value = ex[i].prompt; };
      $('prompt').value = ex[0].prompt;
    } catch { /* optional */ }
  }
  init();
})();
