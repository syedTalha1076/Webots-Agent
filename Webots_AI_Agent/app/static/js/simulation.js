/* Synchronized visualization: Three.js 3D view + 2D top view, both driven only by real telemetry. */
(function () {
  const ROBOT_R = 0.037;
  const Viewer = {
    world: null, pose: null, trace: [], mode: '3d', three: null,

    init() {
      this.host = document.getElementById('three');
      this.canvas = document.getElementById('top');
      this.hasThree = typeof THREE !== 'undefined';
      document.getElementById('tab-3d').onclick = () => this.setMode('3d');
      document.getElementById('tab-2d').onclick = () => this.setMode('2d');
      if (!this.hasThree) { document.getElementById('tab-3d').hidden = true; this.setMode('2d'); }
      window.addEventListener('resize', () => this.resize());
      const loop = () => { this.render(); requestAnimationFrame(loop); };
      loop();
    },

    setMode(m) {
      this.mode = m;
      document.getElementById('tab-3d').setAttribute('aria-selected', m === '3d');
      document.getElementById('tab-2d').setAttribute('aria-selected', m === '2d');
      this.host.hidden = m !== '3d';
      this.canvas.hidden = m !== '2d';
      this.resize();
    },

    setWorld(world) {
      this.world = world; this.trace = []; this.pose = null;
      document.getElementById('view-empty').hidden = !!world;
      if (this.hasThree) this.buildScene();
      this.resize();
    },

    update(tel, trace) {
      if (!tel) return;
      this.pose = { x: tel.position[0], y: tel.position[1], h: tel.heading };
      if (trace) this.trace = trace;
      else if (!this.trace.length || Math.hypot(this.trace.at(-1)[0] - this.pose.x, this.trace.at(-1)[1] - this.pose.y) > 0.01)
        this.trace.push([this.pose.x, this.pose.y]);
    },

    clearTrace() { this.trace = []; },

    resize() {
      const box = document.getElementById('stage').getBoundingClientRect();
      if (this.canvas) { this.canvas.width = box.width * devicePixelRatio; this.canvas.height = box.height * devicePixelRatio; }
      if (this.three) { this.three.renderer.setSize(box.width, box.height); this.three.camera.aspect = box.width / box.height; this.three.camera.updateProjectionMatrix(); }
    },

    buildScene() {
      const w = this.world; if (!w) return;
      if (!this.three) {
        const renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setPixelRatio(devicePixelRatio);
        this.host.appendChild(renderer.domElement);
        const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
        const controls = THREE.OrbitControls ? new THREE.OrbitControls(camera, renderer.domElement) : null;
        this.three = { renderer, camera, controls, scene: null };
      }
      const scene = new THREE.Scene();
      scene.background = new THREE.Color(0xd9e3e7);
      scene.add(new THREE.HemisphereLight(0xffffff, 0x8899a0, 0.9));
      const sun = new THREE.DirectionalLight(0xffffff, 0.6); sun.position.set(1, -2, 3); scene.add(sun);
      // Webots ENU: x,y on the floor, z up -> keep the same axes (camera up = +z)
      const W = w.width, H = w.height;
      const floor = new THREE.Mesh(new THREE.PlaneGeometry(W, H), new THREE.MeshLambertMaterial({ color: 0xf1f4f5 }));
      scene.add(floor);
      scene.add(new THREE.GridHelper(Math.max(W, H), Math.round(Math.max(W, H) * 4), 0xb7c4ca, 0xd0dade).rotateX(Math.PI / 2));
      const wallMat = new THREE.MeshLambertMaterial({ color: 0x7c8d95 });
      [[0, H / 2, W, 0.02], [0, -H / 2, W, 0.02], [W / 2, 0, 0.02, H], [-W / 2, 0, 0.02, H]].forEach(([x, y, sx, sy]) => {
        const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, 0.1), wallMat); m.position.set(x, y, 0.05); scene.add(m);
      });
      w.obstacles.forEach(o => {
        const m = new THREE.Mesh(new THREE.BoxGeometry(o.size, o.size, o.size), new THREE.MeshLambertMaterial({ color: 0xb8864b }));
        m.position.set(o.x, o.y, o.size / 2); scene.add(m);
      });
      const t = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.004, 32).rotateX(Math.PI / 2), new THREE.MeshBasicMaterial({ color: 0xd32f2f }));
      t.position.set(w.target[0], w.target[1], 0.003); scene.add(t);
      const robot = new THREE.Group();
      const body = new THREE.Mesh(new THREE.CylinderGeometry(ROBOT_R, ROBOT_R, 0.055, 24).rotateX(Math.PI / 2), new THREE.MeshLambertMaterial({ color: 0x0e7c86 }));
      const nose = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.012, 0.012), new THREE.MeshBasicMaterial({ color: 0xffffff }));
      nose.position.set(ROBOT_R - 0.005, 0, 0.03);
      robot.add(body, nose); robot.position.set(w.start[0], w.start[1], 0.03); robot.rotation.z = 0;
      scene.add(robot);
      const line = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: 0x0e7c86 }));
      scene.add(line);
      const d = Math.max(W, H);
      this.three.camera.up.set(0, 0, 1);
      this.three.camera.position.set(0, -d * 0.9, d * 1.1);
      this.three.camera.lookAt(0, 0, 0);
      if (this.three.controls) { this.three.controls.target.set(0, 0, 0); this.three.controls.update(); }
      Object.assign(this.three, { scene, robot, line });
    },

    render() {
      if (!this.world) return;
      if (this.mode === '3d' && this.three && this.three.scene) {
        const { robot, line, renderer, scene, camera, controls } = this.three;
        if (this.pose) { robot.position.set(this.pose.x, this.pose.y, 0.03); robot.rotation.z = this.pose.h; }
        if (this.trace.length > 1) line.geometry.setFromPoints(this.trace.map(p => new THREE.Vector3(p[0], p[1], 0.006)));
        if (controls) controls.update();
        renderer.render(scene, camera);
      } else if (this.mode === '2d') this.draw2d();
    },

    draw2d() {
      const c = this.canvas, g = c.getContext('2d'), w = this.world;
      const css = getComputedStyle(document.documentElement);
      g.clearRect(0, 0, c.width, c.height);
      const pad = 24 * devicePixelRatio;
      const s = Math.min((c.width - 2 * pad) / w.width, (c.height - 2 * pad) / w.height);
      const X = x => c.width / 2 + x * s, Y = y => c.height / 2 - y * s;
      g.fillStyle = '#f1f4f5'; g.fillRect(X(-w.width / 2), Y(w.height / 2), w.width * s, w.height * s);
      g.strokeStyle = '#7c8d95'; g.lineWidth = 3 * devicePixelRatio; g.strokeRect(X(-w.width / 2), Y(w.height / 2), w.width * s, w.height * s);
      g.fillStyle = '#b8864b';
      w.obstacles.forEach(o => g.fillRect(X(o.x - o.size / 2), Y(o.y + o.size / 2), o.size * s, o.size * s));
      g.fillStyle = '#d32f2f'; g.beginPath(); g.arc(X(w.target[0]), Y(w.target[1]), 0.05 * s, 0, 7); g.fill();
      if (this.trace.length > 1) {
        g.strokeStyle = '#0e7c86'; g.lineWidth = 2 * devicePixelRatio; g.beginPath();
        this.trace.forEach((p, i) => i ? g.lineTo(X(p[0]), Y(p[1])) : g.moveTo(X(p[0]), Y(p[1]))); g.stroke();
      }
      const p = this.pose || { x: w.start[0], y: w.start[1], h: 0 };
      g.fillStyle = '#0e7c86'; g.beginPath(); g.arc(X(p.x), Y(p.y), ROBOT_R * s, 0, 7); g.fill();
      g.strokeStyle = '#fff'; g.lineWidth = 2 * devicePixelRatio; g.beginPath();
      g.moveTo(X(p.x), Y(p.y)); g.lineTo(X(p.x + Math.cos(p.h) * ROBOT_R), Y(p.y + Math.sin(p.h) * ROBOT_R)); g.stroke();
    },
  };
  window.Viewer = Viewer;
})();
