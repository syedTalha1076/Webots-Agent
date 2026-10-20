/* Telemetry panel: renders only values received from the Webots controller. */
(function () {
  const f = (v, d = 2, unit = '') => (v === null || v === undefined || Number.isNaN(v)) ? '–' : Number(v).toFixed(d) + unit;
  const deg = r => (r * 180 / Math.PI).toFixed(0) + '°';
  const ROWS = [
    ['Position (x, y)', t => `${f(t.position[0])}, ${f(t.position[1])} m`],
    ['Orientation', t => deg(t.heading)],
    ['Distance to target', t => f(t.target_distance, 2, ' m')],
    ['Odometry estimate', t => t.odometry ? `${f(t.odometry[0])}, ${f(t.odometry[1])}` : '–'],
    ['Motors (L / R)', t => `${f(t.motors[0], 2)} / ${f(t.motors[1], 2)} rad/s`],
    ['Navigator state', t => t.state],
    ['Waypoint', t => t.waypoint_index + 1],
    ['Simulation time', t => f(t.t, 1, ' s')],
    ['Distance travelled', t => f(t.metrics.distance_travelled, 2, ' m')],
    ['Collisions', t => t.metrics.collisions ?? '–'],
  ];
  const BARS = [['Left', 'left'], ['Front', 'front'], ['Right', 'right']];

  const Telemetry = {
    init() {
      this.kv = document.getElementById('kv');
      this.bars = document.getElementById('bars');
      this.pill = document.getElementById('state-pill');
      this.kv.innerHTML = ROWS.map(r => `<dt>${r[0]}</dt><dd>–</dd>`).join('');
      this.bars.innerHTML = BARS.map(b => `<div class="bar"><span>${b[0]}</span><i><b></b></i><span>–</span></div>`).join('');
    },
    render(t) {
      if (!t) return;
      const dds = this.kv.querySelectorAll('dd');
      ROWS.forEach((r, i) => { dds[i].textContent = r[1](t); });
      const rows = this.bars.querySelectorAll('.bar');
      BARS.forEach((b, i) => {
        const v = t.sensor_summary[b[1]] ?? 0;
        const fill = rows[i].querySelector('b');
        fill.style.width = Math.min(100, v / 6) + '%';
        fill.classList.toggle('hot', v > 180);
        rows[i].lastElementChild.textContent = Math.round(v);
      });
      const cls = { finished: 'good', stuck: 'warn', paused: 'warn', stopped: 'bad', comm_lost: 'bad' }[t.state] || 'run';
      this.pill.className = 'pill ' + cls;
      this.pill.textContent = t.state.replace('_', ' ');
    },
  };
  window.Telemetry = Telemetry;
})();
