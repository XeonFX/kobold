// Kobold control app — plain ES module, no bundler.
//
// Alpine handles declarative binding (chat, status, tabs). Everything realtime
// — telemetry, audio levels, the radar, the joystick — is imperative canvas and
// WebSocket work, which is exactly where a component framework adds ceremony
// without buying anything.

const CMD_HZ = 20;            // comfortably inside the firmware's 300 ms watchdog
const RADAR_MAX_MM = 2000;    // beyond 2 m is not actionable indoors

const TABS = [
  { id: 'talk',   glyph: '◍' },
  { id: 'drive',  glyph: '⌖' },
  { id: 'find',   glyph: '⌕' },
  { id: 'system', glyph: '⚙' },
];

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const dpr = () => window.devicePixelRatio || 1;

/** Read a CSS custom property so canvas drawing follows the theme. */
function token(name, fallback) {
  const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
  return v || fallback;
}

/** Green when far, amber when close, red when very close. */
function proximityColor(mm) {
  if (!isFinite(mm)) return token('--line', '#232a3a');
  if (mm < 200) return token('--bad', '#fb5b6b');
  if (mm < 500) return token('--warn', '#fbbf24');
  return token('--ok', '#34d399');
}

function kobold() {
  return {
    tabs: TABS,
    tab: 'talk',

    robot: {
      connected: false, stale: true, battery_pct: null, battery_v: null,
      faults: 'unknown', estopped: false, ranges: {}, mode: 'floor',
    },
    deploy: { sha: '…', previous: null, deployed_at: null },
    update: null,
    socketUp: false,

    messages: [],
    draft: '',
    thinking: false,
    recording: false,
    levels: new Array(9).fill(3),

    targets: [],
    targetLabel: '',
    photoPicked: false,

    videoSrc: '/api/camera/stream',
    // Live view sits behind the joystick. On by default — seeing where you are
    // driving is the point — but switchable for a slow link.
    liveBehind: true,
    videoErr: false,

    _cmdSock: null,
    _cmd: { x: 0, z: 0 },
    _stick: { active: false, c: 0, r: 0, dx: 0, dy: 0 },

    // ---------------------------------------------------------- lifecycle --

    start() {
      // Fetch once over HTTP before the socket opens. Without this the UI shows
      // placeholder dashes for the first second or two on every load, which
      // reads as "the robot is broken" rather than "still connecting".
      fetch('/api/status')
        .then((r) => r.json())
        .then((d) => {
          Object.assign(this.robot, d.robot ?? {});
          Object.assign(this.deploy, d.deploy ?? {});
          this.drawRadar();
        })
        .catch(() => { /* socket will fill it in */ });

      this.connectTelemetry();
      this.connectCmd();
      this.loadTargets();
      setInterval(() => this.pushCmd(), 1000 / CMD_HZ);

      this.$nextTick(() => { this.initStick(); this.initRadar(); });
      window.addEventListener('resize', () => { this.initStick(); this.initRadar(); });
      // Re-theme the canvases if the OS switches between light and dark.
      matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => {
        this.drawRadar(); this.drawStick();
      });
    },

    onTab(id) {
      // Canvases in a hidden element have zero size; size them on reveal.
      if (id === 'drive') this.$nextTick(() => { this.initStick(); this.initRadar(); });
    },

    // Two independent links, and conflating them makes debugging miserable:
    //   socketUp        — this browser can reach the app on the robot
    //   robot.connected — the bridge can reach the microcontrollers
    // "Offline" meaning either one tells you nothing about which to go fix.
    get linkLive() { return this.socketUp && this.robot.connected && !this.robot.stale; },

    // Reconnecting sockets throughout. The robot moves, WiFi drops, and a UI
    // that needs a manual refresh after every dropout is useless in practice.
    connectTelemetry() {
      const ws = new WebSocket(`ws://${location.host}/ws/telemetry`);
      ws.onopen = () => { this.socketUp = true; };
      ws.onmessage = (e) => {
        const d = JSON.parse(e.data);
        Object.assign(this.robot, d.robot);
        Object.assign(this.deploy, d.deploy);
        this.update = d.update ?? null;
        this.drawRadar();
      };
      ws.onclose = () => {
        this.socketUp = false;
        this.drawRadar();
        setTimeout(() => this.connectTelemetry(), 1000);
      };
    },

    connectCmd() {
      this._cmdSock = new WebSocket(`ws://${location.host}/ws/cmd`);
      this._cmdSock.onclose = () => setTimeout(() => this.connectCmd(), 1000);
    },

    // Send continuously, not on change: the firmware treats silence as "stop",
    // so an event-driven UI makes the robot stutter whenever a packet is late.
    pushCmd() {
      if (this._cmdSock?.readyState !== WebSocket.OPEN) return;
      if (!this._stick.active && !this._cmd.x && !this._cmd.z) return;
      this._cmdSock.send(JSON.stringify({ type: 'cmd_vel', ...this._cmd }));
    },

    // -------------------------------------------------------------- radar --

    initRadar() {
      const c = this.$refs.radar;
      if (!c || !c.offsetParent) return;
      const size = Math.min(300, c.parentElement.clientWidth - 8);
      c.width = size * dpr(); c.height = size * dpr();
      c.style.width = c.style.height = `${size}px`;
      c.getContext('2d').setTransform(dpr(), 0, 0, dpr(), 0, 0);
      this.drawRadar();
    },

    drawRadar() {
      const c = this.$refs.radar;
      if (!c || !c.width) return;
      const g = c.getContext('2d');
      const size = c.width / dpr();
      const cx = size / 2, cy = size / 2;
      const rMax = size / 2 - 16;

      g.clearRect(0, 0, size, size);

      // Distance rings at 0.5 m intervals.
      g.strokeStyle = token('--line-soft', '#1a2030');
      g.lineWidth = 1;
      for (let i = 1; i <= 4; i++) {
        g.beginPath();
        g.arc(cx, cy, (rMax * i) / 4, 0, Math.PI * 2);
        g.stroke();
      }
      g.beginPath();
      g.moveTo(cx - rMax, cy); g.lineTo(cx + rMax, cy);
      g.moveTo(cx, cy - rMax); g.lineTo(cx, cy + rMax);
      g.stroke();

      // Sensor arcs. Angles are screen-space: front is up.
      const sensors = [
        ['front', -Math.PI / 2],
        ['right',  0],
        ['back',   Math.PI / 2],
        ['left',   Math.PI],
      ];
      const spread = 0.42;   // ~24 degrees, roughly the HC-SR04 cone

      for (const [name, angle] of sensors) {
        const mm = this.robot.ranges?.[name];
        const color = proximityColor(mm ?? Infinity);
        const known = mm !== undefined && mm !== null && isFinite(mm);
        const r = known ? clamp(mm / RADAR_MAX_MM, 0.06, 1) * rMax : rMax;

        // Filled wedge, faint — shows the covered region.
        g.beginPath();
        g.moveTo(cx, cy);
        g.arc(cx, cy, r, angle - spread, angle + spread);
        g.closePath();
        g.fillStyle = color + (known ? '1f' : '0c');
        g.fill();

        // The arc itself: the actual measurement.
        g.beginPath();
        g.arc(cx, cy, r, angle - spread, angle + spread);
        g.strokeStyle = known ? color : token('--line', '#232a3a');
        g.lineWidth = known ? 3 : 1.5;
        g.setLineDash(known ? [] : [3, 4]);
        g.stroke();
        g.setLineDash([]);
      }

      // Robot body, oriented so "up" is forward.
      g.fillStyle = token('--surface-3', '#1b2030');
      g.strokeStyle = this.linkLive ? token('--accent', '#5b9dff') : token('--line', '#232a3a');
      g.lineWidth = 1.5;
      const w = 22, h = 28;
      g.beginPath();
      if (g.roundRect) g.roundRect(cx - w / 2, cy - h / 2, w, h, 6);
      else g.rect(cx - w / 2, cy - h / 2, w, h);
      g.fill(); g.stroke();

      // Heading notch.
      g.beginPath();
      g.moveTo(cx, cy - h / 2 - 5);
      g.lineTo(cx - 4, cy - h / 2 + 1);
      g.lineTo(cx + 4, cy - h / 2 + 1);
      g.closePath();
      g.fillStyle = this.linkLive ? token('--accent', '#5b9dff') : token('--fg-dim', '#616b82');
      g.fill();

      // Scale label.
      g.fillStyle = token('--fg-dim', '#616b82');
      g.font = `10px ${token('--mono', 'monospace')}`;
      g.textAlign = 'right';
      g.fillText('2 m', size - 4, cy - 4);
    },

    closest() {
      const vals = Object.values(this.robot.ranges || {}).filter((v) => isFinite(v));
      return vals.length ? Math.min(...vals) : null;
    },
    closestLabel() {
      const m = this.closest();
      return m === null ? 'no data' : `${(m / 10).toFixed(0)} cm`;
    },
    closestChip() {
      const m = this.closest();
      if (m === null) return '';
      return m < 200 ? 'bad' : m < 500 ? 'warn' : 'ok';
    },

    // ----------------------------------------------------------- joystick --

    initStick() {
      const c = this.$refs.stick;
      if (!c || !c.offsetParent) return;
      const size = Math.min(260, c.parentElement.clientWidth - 8);
      c.width = size * dpr(); c.height = size * dpr();
      c.style.width = c.style.height = `${size}px`;
      c.getContext('2d').setTransform(dpr(), 0, 0, dpr(), 0, 0);
      this._stick.c = size / 2;
      this._stick.r = size / 2 - 34;
      this.drawStick();
    },

    drawStick() {
      const c = this.$refs.stick;
      if (!c || !c.width) return;
      const g = c.getContext('2d');
      const size = c.width / dpr();
      const { c: cc, r, dx, dy, active } = this._stick;

      g.clearRect(0, 0, size, size);

      g.strokeStyle = token('--line', '#232a3a');
      g.lineWidth = 1.5;
      g.beginPath(); g.arc(cc, cc, r, 0, Math.PI * 2); g.stroke();

      g.strokeStyle = token('--line-soft', '#1a2030');
      g.lineWidth = 1;
      g.beginPath();
      g.moveTo(cc - r, cc); g.lineTo(cc + r, cc);
      g.moveTo(cc, cc - r); g.lineTo(cc, cc + r);
      g.stroke();

      // Vector from centre to knob — shows magnitude and direction at a glance.
      if (active && (dx || dy)) {
        g.strokeStyle = token('--accent', '#5b9dff') + '55';
        g.lineWidth = 3;
        g.beginPath(); g.moveTo(cc, cc); g.lineTo(cc + dx, cc + dy); g.stroke();
      }

      const kx = cc + dx, ky = cc + dy;
      if (active) {
        g.fillStyle = token('--accent', '#5b9dff') + '22';
        g.beginPath(); g.arc(kx, ky, 40, 0, Math.PI * 2); g.fill();
      }
      g.fillStyle = active ? token('--accent', '#5b9dff') : token('--surface-3', '#1b2030');
      g.strokeStyle = active ? token('--accent', '#5b9dff') : token('--line', '#232a3a');
      g.lineWidth = 1.5;
      g.beginPath(); g.arc(kx, ky, 27, 0, Math.PI * 2); g.fill(); g.stroke();
    },

    stickDown(e) {
      this._stick.active = true;
      e.target.setPointerCapture?.(e.pointerId);
      navigator.vibrate?.(8);
      this.stickMove(e);
    },

    stickMove(e) {
      if (!this._stick.active) return;
      const rect = e.target.getBoundingClientRect();
      let dx = e.clientX - rect.left - this._stick.c;
      let dy = e.clientY - rect.top - this._stick.c;
      const d = Math.hypot(dx, dy), r = this._stick.r;
      if (d > r) { dx = (dx / d) * r; dy = (dy / d) * r; }
      this._stick.dx = dx; this._stick.dy = dy;
      this.drawStick();

      // Forward is up. Table mode is ALSO enforced on the MCU — never trust a
      // UI for a limit that keeps the robot on a table.
      const cap = this.robot.mode === 'table' ? 0.15 : 0.4;
      this._cmd = { x: +(-dy / r * cap).toFixed(3), z: +(-dx / r * 1.5).toFixed(3) };
    },

    stickUp() {
      if (!this._stick.active) return;
      this._stick.active = false;
      this._stick.dx = this._stick.dy = 0;
      this._cmd = { x: 0, z: 0 };
      this.drawStick();
      // Send an explicit zero rather than waiting for the watchdog.
      if (this._cmdSock?.readyState === WebSocket.OPEN) {
        this._cmdSock.send(JSON.stringify({ type: 'cmd_vel', x: 0, z: 0 }));
      }
    },

    setMode(e) {
      const mode = e.target.checked ? 'table' : 'floor';
      this.robot.mode = mode;
      navigator.vibrate?.(12);
      fetch('/api/mode', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ mode }),
      });
    },

    // --------------------------------------------------------------- talk --

    async startTalking() {
      if (this.recording) return;
      try {
        const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
        this.recording = true;
        navigator.vibrate?.(10);

        // Live input level. A static "recording" label tells you nothing about
        // whether the mic is actually picking you up.
        const ctx = new AudioContext();
        const analyser = ctx.createAnalyser();
        analyser.fftSize = 64;
        ctx.createMediaStreamSource(stream).connect(analyser);
        const bins = new Uint8Array(analyser.frequencyBinCount);
        this._meter = setInterval(() => {
          analyser.getByteFrequencyData(bins);
          this.levels = Array.from({ length: 9 }, (_, i) =>
            Math.max(3, Math.round((bins[i * 2] / 255) * 18)));
        }, 60);
        this._audioCtx = ctx;

        this._sock = new WebSocket(`ws://${location.host}/ws/audio`);
        this._sock.binaryType = 'arraybuffer';
        this._sock.onmessage = (e) => this.onAudioReply(e);

        this._rec = new MediaRecorder(stream, { mimeType: 'audio/webm;codecs=opus' });
        this._rec.ondataavailable = (e) => {
          if (e.data.size && this._sock?.readyState === WebSocket.OPEN) this._sock.send(e.data);
        };
        this._rec.start(250);
        this._stream = stream;
      } catch (err) {
        this.recording = false;
        this.say('system', `microphone unavailable: ${err.message}`);
      }
    },

    stopTalking() {
      if (!this.recording) return;
      this.recording = false;
      clearInterval(this._meter);
      this._audioCtx?.close();
      this._rec?.stop();
      this._stream?.getTracks().forEach((t) => t.stop());
      this.thinking = true;
    },

    levelFor(n) { return this.levels[n - 1] ?? 3; },

    onAudioReply(e) {
      if (typeof e.data === 'string') {
        const m = JSON.parse(e.data);
        if (m.type === 'transcript') this.say('you', m.text);
        if (m.type === 'reply') { this.thinking = false; this.say('robot', m.text); }
        return;
      }
      this.thinking = false;
      new Audio(URL.createObjectURL(new Blob([e.data], { type: 'audio/ogg' }))).play();
    },

    sendText() {
      const text = this.draft.trim();
      if (!text) return;
      this.say('you', text);
      this.draft = '';
      this.thinking = true;
      fetch('/api/chat', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ text }),
      })
        .then((r) => r.json())
        .then((d) => this.say('robot', d.reply ?? '…'))
        .catch(() => this.say('system', 'agent unreachable'))
        .finally(() => { this.thinking = false; });
    },

    say(role, text) {
      this.messages.push({ role, text });
      this.$nextTick(() => {
        const c = this.$refs.chat;
        if (c) c.scrollTop = c.scrollHeight;
      });
    },

    // --------------------------------------------------------------- find --

    async loadTargets() {
      try { this.targets = await (await fetch('/api/targets')).json(); } catch { /* offline */ }
    },

    async uploadTarget() {
      const f = this.$refs.photo.files[0];
      if (!f) return;
      const fd = new FormData();
      fd.append('file', f);
      fd.append('label', this.targetLabel);
      await fetch('/api/targets', { method: 'POST', body: fd });
      this.targetLabel = '';
      this.photoPicked = false;
      this.$refs.photo.value = '';
      this.loadTargets();
      navigator.vibrate?.(12);
    },

    findObject(t) {
      this.tab = 'talk';
      this.say('you', `find the ${this.prettyName(t.id)}`);
      this.thinking = true;
      fetch('/api/find', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ target: t.id }),
      }).catch(() => { this.thinking = false; this.say('system', 'agent unreachable'); });
    },

    prettyName(id) {
      return id.split('_').slice(1).join(' ').replace(/([a-z])([A-Z])/g, '$1 $2') || 'target';
    },

    // ------------------------------------------------------------- system --

    toggleEstop() {
      navigator.vibrate?.(this.robot.estopped ? 10 : [20, 40, 20]);
      const fd = new FormData();
      fd.append('engage', String(!this.robot.estopped));
      fetch('/api/estop', { method: 'POST', body: fd });
    },

    hasFaults() {
      const f = this.robot.faults;
      return f && f !== 'none' && f !== 'unknown';
    },

    batteryClass() {
      const p = this.robot.battery_pct;
      if (p === null) return '';
      return p < 15 ? 'crit' : p < 30 ? 'low' : '';
    },

    linkLabel() {
      if (!this.socketUp) return 'no signal';        // can't reach the app
      if (!this.robot.connected) return 'no robot';  // app is up, MCUs are not
      return this.robot.stale ? 'stale' : 'connected';
    },
    linkChip() {
      if (!this.socketUp || !this.robot.connected) return 'bad';
      return this.robot.stale ? 'warn' : 'ok';
    },

    shortTime(iso) {
      if (!iso) return '—';
      try {
        return new Date(iso).toLocaleString(undefined,
          { month: 'short', day: 'numeric', hour: '2-digit', minute: '2-digit' });
      } catch { return iso; }
    },
  };
}

// Register through Alpine's own lifecycle rather than assigning to `window`.
//
// A `type="module"` script is always deferred to the end of parsing, so it runs
// AFTER Alpine has already initialised and looked for `x-data="kobold()"` —
// which produced a page of "kobold is not defined" and empty bindings. This
// file is now a classic deferred script loaded before Alpine, and it registers
// the component on `alpine:init`, which is the supported hook.
document.addEventListener('alpine:init', () => {
  window.Alpine.data('kobold', kobold);
});
