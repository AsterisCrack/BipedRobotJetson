// Control tab — RL policy arm/drive/monitor.
//
// Follows the module conventions used by servos.js / imu.js:
//   initControl()          wire DOM listeners once, at load
//   onControlTelemetry()   called for every telemetry frame
// WebSocket for streaming + continuous commands, fetch() for one-shot actions.

import { send } from './app.js';
import { initGamepad, gamepadSupported } from './gamepad.js';

const $ = id => document.getElementById(id);

let lastState = null;
let cmdTimer = null;
let keysEnabled = false;
let padEnabled = false;
let autoEnabled = false;
const keyHeld = new Set();
const jointRows = {};

// ---------------------------------------------------------------------------
// REST helpers
// ---------------------------------------------------------------------------

async function api(path, body) {
  const opts = body === undefined
    ? { method: 'POST' }
    : { method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body) };
  const r = await fetch(`/api/policy/${path}`, opts);
  const data = await r.json().catch(() => ({}));
  if (!r.ok) {
    flashFault(data.detail || `${path} failed`);
    throw new Error(data.detail || path);
  }
  return data;
}

function flashFault(msg) {
  const el = $('ctl-fault');
  el.textContent = msg;
  el.classList.add('visible');
  setTimeout(() => el.classList.remove('visible'), 6000);
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

function currentCommand() {
  return {
    vx: parseFloat($('ctl-vx').value),
    vy: parseFloat($('ctl-vy').value),
    wz: parseFloat($('ctl-wz').value),
  };
}

// Debounced to 60 ms, matching the servo slider pattern — the policy consumes
// commands at 50 Hz and the server clamps them, so there is nothing to gain from
// flooding the socket on every pixel of slider travel.
function pushCommand() {
  clearTimeout(cmdTimer);
  cmdTimer = setTimeout(() => {
    send({ type: 'set_velocity_command', ...currentCommand() });
  }, 60);
}

function showCommand(vx, vy, wz) {
  $('ctl-vx').value = vx; $('ctl-vy').value = vy; $('ctl-wz').value = wz;
  syncCommandLabels();
}

function setSliders(vx, vy, wz) {
  showCommand(vx, vy, wz);
  pushCommand();
}

function syncCommandLabels() {
  $('ctl-vx-val').textContent = parseFloat($('ctl-vx').value).toFixed(2);
  $('ctl-vy-val').textContent = parseFloat($('ctl-vy').value).toFixed(2);
  $('ctl-wz-val').textContent = parseFloat($('ctl-wz').value).toFixed(2);
}

function applyKeys() {
  if (!keysEnabled) return;
  const m = maxSpeeds();
  const vx = (keyHeld.has('w') ? m.fwd : 0) - (keyHeld.has('s') ? m.back : 0);
  const vy = (keyHeld.has('a') ? m.lat : 0) - (keyHeld.has('d') ? m.lat : 0);
  const wz = (keyHeld.has('q') ? m.turn : 0) - (keyHeld.has('e') ? m.turn : 0);
  setSliders(vx, vy, wz);
}

// ---------------------------------------------------------------------------
// Command ranges + max speeds (what full stick / a held key means)
// ---------------------------------------------------------------------------

const MAX_IDS = { fwd: 'ctl-max-fwd', back: 'ctl-max-back', lat: 'ctl-max-lat', turn: 'ctl-max-turn' };
const MAX_STORE = 'ctl-max-speeds';
let rangesKey = '';

function maxSpeeds() {
  const m = {};
  for (const [k, id] of Object.entries(MAX_IDS)) m[k] = parseFloat($(id).value) || 0;
  return m;
}

function syncMaxLabels() {
  for (const id of Object.values(MAX_IDS)) $(`${id}-val`).textContent = parseFloat($(id).value).toFixed(2);
}

function saveMaxSpeeds() {
  try { localStorage.setItem(MAX_STORE, JSON.stringify(maxSpeeds())); } catch (_) {}
}

// Bound the command sliders and the max-speed sliders by the bundle's trained command box
// ({lin_vel_x: [lo, hi], ...}). The server clamps anyway; this keeps the UI honest about it.
function applyCommandRanges(r) {
  if (!r || !r.lin_vel_x) return;
  const key = JSON.stringify(r);
  if (key === rangesKey) return;
  rangesKey = key;

  const setRange = (id, lo, hi) => {
    const el = $(id);
    el.min = lo; el.max = hi;
    el.step = Math.max(0.005, (hi - lo) / 200).toFixed(3);
  };
  const [xlo, xhi] = r.lin_vel_x, [ylo, yhi] = r.lin_vel_y, [wlo, whi] = r.ang_vel_z;
  setRange('ctl-vx', xlo, xhi);
  setRange('ctl-vy', ylo, yhi);
  setRange('ctl-wz', wlo, whi);

  const bounds = {
    fwd: Math.max(0, xhi), back: Math.max(0, -xlo),
    lat: Math.max(0, -ylo, yhi), turn: Math.max(0, -wlo, whi),
  };
  let stored = {};
  try { stored = JSON.parse(localStorage.getItem(MAX_STORE) || '{}'); } catch (_) {}
  for (const [k, id] of Object.entries(MAX_IDS)) {
    const el = $(id);
    el.min = 0; el.max = bounds[k];
    el.step = Math.max(0.005, bounds[k] / 100).toFixed(3);
    // A stored value from another bundle is clamped into this one's box; no stored value = full range.
    el.value = stored[k] != null ? Math.min(stored[k], bounds[k]) : bounds[k];
  }
  syncMaxLabels();
}

// ---------------------------------------------------------------------------
// Gamepad
// ---------------------------------------------------------------------------

let padConnected = null;

function padStatus() {
  const el = $('ctl-pad-status');
  if (!gamepadSupported()) {
    el.textContent = 'This browser has no Gamepad API.';
  } else if (padConnected) {
    el.textContent = `${padConnected.id.slice(0, 60)}`
      + (padEnabled ? ' (driving)' : ' (connected, driving off; X still e-stops)');
  } else if (!window.isSecureContext) {
    el.textContent = 'No gamepad. Browsers may hide gamepads on plain http: open the UI through '
      + 'an SSH tunnel (ssh -L 8080:localhost:8080 <jetson>) at http://localhost:8080.';
  } else {
    el.textContent = 'No gamepad. Press a button on it to connect.';
  }
}

// E-STOP: fire over BOTH transports. The socket is lower latency, the REST call
// is the one that still works if the socket has silently dropped. An e-stop
// should not depend on a single channel being healthy.
function estop() {
  send({ type: 'policy_estop' });
  fetch('/api/policy/estop', { method: 'POST' }).catch(() => {});
  setSliders(0, 0, 0);
}

function onPadTick({ lx, ly, rx }) {
  if (!padEnabled) return;
  const m = maxSpeeds();
  const vx = ly >= 0 ? ly * m.fwd : ly * m.back;
  const vy = -lx * m.lat;          // stick right = move right = -y
  const wz = -rx * m.turn;         // stick right = turn clockwise = -z
  showCommand(vx, vy, wz);
  // Sent every tick, even at zero: the server zeroes a streamed command that stops arriving.
  send({ type: 'set_velocity_command', vx, vy, wz, stream: true });
}

function onPadButton(name) {
  if (name === 'X') { estop(); return; }   // always, even with gamepad driving off
  if (!padEnabled) return;
  const quiet = p => p.catch(() => {});
  if (name === 'A') {
    if (lastState === 'idle') quiet(api('arm'));
    else if (lastState === 'armed') quiet(api('start'));
  } else if (name === 'B') {
    quiet(api('stop'));
  } else if (name === 'Y') {
    quiet(setAuto({ enabled: !autoEnabled }));
  } else if (name === 'View') {
    quiet(api('disarm'));
  }
}

function setPadEnabled(on) {
  padEnabled = on;
  $('ctl-pad').checked = on;
  if (on && keysEnabled) {
    keysEnabled = false; $('ctl-keys').checked = false; keyHeld.clear();
  }
  // A plain (non-stream) zero both stops the robot and clears the server's deadman.
  if (!on) setSliders(0, 0, 0);
  padStatus();
}

// ---------------------------------------------------------------------------
// Auto-recover
// ---------------------------------------------------------------------------

const AUTO_SLIDERS = [
  ['upright_deg', 'ctl-auto-upright', v => v.toFixed(0)],
  ['settle_s', 'ctl-auto-settle', v => v.toFixed(1)],
  ['countdown_s', 'ctl-auto-countdown', v => v.toFixed(0)],
];
const AUTO_LABELS = {
  off: 'off', watching: 'watching for falls', wait_upright: 'stand it up',
  arming: 'arming', countdown: 'starting in',
};

function applyAutoSettings(st) {
  if (!st || !st.settings) return;
  for (const [key, id, fmt] of AUTO_SLIDERS) {
    if (st.settings[key] == null) continue;
    $(id).value = st.settings[key];
    $(`${id}-val`).textContent = fmt(st.settings[key]);
  }
}

async function setAuto(body) {
  const r = await api('auto', body);
  applyAutoSettings(r);
  renderAuto(r);
  return r;
}

async function refreshAuto() {
  const r = await fetch('/api/policy/auto').then(x => (x.ok ? x.json() : null)).catch(() => null);
  if (r) { applyAutoSettings(r); renderAuto(r); }
}

function renderAuto(a) {
  if (!a) return;
  autoEnabled = !!a.enabled;
  $('ctl-auto-en').checked = autoEnabled;
  const ph = $('ctl-auto-phase');
  ph.textContent = AUTO_LABELS[a.phase] || a.phase;
  ph.className = `ctl-auto-phase ctl-auto-${a.phase}`;
  let big = '';
  if (a.phase === 'countdown' && a.countdown_left != null) big = `${Math.ceil(a.countdown_left)}`;
  else if (a.phase === 'wait_upright' && a.upright_s > 0 && a.settings)
    big = `${a.upright_s.toFixed(1)} / ${a.settings.settle_s.toFixed(1)} s`;
  $('ctl-auto-big').textContent = big;
  $('ctl-auto-count').textContent = a.recoveries ?? 0;
  $('ctl-auto-err').textContent = a.last_error || '';
}

// ---------------------------------------------------------------------------
// Models
// ---------------------------------------------------------------------------

async function refreshModels() {
  const r = await fetch('/api/policy/models').then(x => x.json()).catch(() => null);
  const sel = $('ctl-model-select');
  sel.innerHTML = '';
  if (!r || !r.models || !r.models.length) {
    sel.innerHTML = '<option value="">— no models found —</option>';
    $('ctl-model-meta').textContent = `No bundles in ${r ? r.models_root : 'models/'}`;
    return;
  }
  r.models.forEach(m => {
    const o = document.createElement('option');
    o.value = m.name;
    // Surface broken bundles rather than hiding them — a bad export should be
    // visible here, not discovered at arm time.
    o.textContent = m.valid ? m.name : `${m.name} (invalid)`;
    o.disabled = !m.valid;
    o.title = m.valid ? '' : m.error;
    sel.appendChild(o);
  });
}

// ---------------------------------------------------------------------------
// Safety limit sliders
// ---------------------------------------------------------------------------

const RAD2DEG = 180 / Math.PI;

// Show the runner's real limits ({tilt_fault_deg, max_action_rate, velocity_scale}) in the sliders.
// The tilt slider snaps to whole degrees; the label keeps the exact value.
function applyLimits(l) {
  if (!l) return;
  if (l.tilt_fault_deg != null) {
    $('ctl-lim-tilt').value = l.tilt_fault_deg;
    $('ctl-lim-tilt-val').textContent = Number(l.tilt_fault_deg.toFixed(1));
  }
  if (l.max_action_rate != null) {
    $('ctl-lim-rate').value = l.max_action_rate;
    $('ctl-lim-rate-val').textContent = l.max_action_rate.toFixed(2);
  }
  if (l.velocity_scale != null) {
    $('ctl-lim-scale').value = l.velocity_scale;
    $('ctl-lim-scale-val').textContent = l.velocity_scale.toFixed(2);
  }
}

// The sliders must start from what the runner is really using (a bundle can change it on load),
// never from the markup defaults.
async function refreshLimits() {
  const r = await fetch('/api/policy/safety').then(x => x.json()).catch(() => null);
  if (r && r.limits) applyLimits(r.limits);
}

// ---------------------------------------------------------------------------
// Telemetry rendering
// ---------------------------------------------------------------------------

// "v4 residual | clock fixed_period 0.70 s | home hip-10 knee-26 ankle-16"
function modelSummary(p) {
  if (!p.model || p.format_version == null) return '';
  const parts = [`v${p.format_version} ${p.action_map}`];
  const gc = p.gait_clock;
  if (gc) {
    if (gc.mode === 'cadence_grid') {
      const f = gc.freq.flat(2);
      parts.push(`clock cadence ${(1 / Math.max(...f)).toFixed(2)}-${(1 / Math.min(...f)).toFixed(2)} s`);
    } else {
      parts.push(gc.mode === 'fixed_period'
        ? `clock fixed_period ${Number(gc.period).toFixed(2)} s`
        : `clock ${gc.mode} ${Number(gc.freq).toFixed(2)} Hz/(m/s)`);
    }
  }
  const h = p.home_deg || {};
  const sgn = v => `${v > 0 ? '+' : ''}${Math.round(v)}`;
  const named = [['hip', 'l_hip_pitch_joint'], ['knee', 'l_knee_joint'], ['ankle', 'l_ankle_pitch_joint']]
    .filter(([, k]) => h[k] !== undefined).map(([n, k]) => `${n}${sgn(h[k])}`);
  const allZero = Object.values(h).every(v => Math.abs(v) < 0.05);
  parts.push(allZero ? 'home 0' : `home ${named.length ? named.join(' ') : 'custom'}`);
  return parts.join(' | ');
}

function renderButtons(state) {
  const running = state === 'running';
  const armed = state === 'armed';
  const fault = state === 'fault';
  const idle = state === 'idle';

  $('ctl-arm').disabled = !idle;
  $('ctl-start').disabled = !armed;
  $('ctl-stop').disabled = !running;
  $('ctl-disarm').disabled = idle || fault;
  $('ctl-load').disabled = !idle;
  $('ctl-model-select').disabled = !idle;
  $('ctl-clear').classList.toggle('hidden', !fault);
}

function renderJoints(policy, servos) {
  const targets = policy.targets_deg || {};
  const names = Object.keys(targets);
  if (!names.length) return;

  const measured = {};
  (servos || []).forEach(s => { measured[s.joint] = s; });

  const tbody = $('ctl-joint-tbody');
  names.forEach((name, i) => {
    if (!jointRows[name]) {
      const tr = document.createElement('tr');
      tr.innerHTML = `<td>${name}</td><td></td><td></td><td></td><td></td><td></td><td></td>`;
      tbody.appendChild(tr);
      jointRows[name] = tr;
    }
    const tr = jointRows[name];
    const cmd = targets[name];
    const s = measured[name];
    const meas = s ? s.logical_deg : null;
    const err = (meas === null || meas === undefined) ? null : cmd - meas;
    const act = policy.action && policy.action[i] !== undefined ? policy.action[i] : null;

    tr.children[1].textContent = cmd.toFixed(2);
    tr.children[2].textContent = meas === null ? '—' : meas.toFixed(2);
    tr.children[3].textContent = err === null ? '—' : err.toFixed(2);
    if (err !== null) tr.children[3].className = Math.abs(err) > 10 ? 'ctl-warn' : '';
    tr.children[4].textContent = act === null ? '—' : act.toFixed(3);
    tr.children[5].textContent = s ? s.load : '—';
    tr.children[6].textContent = s ? s.temperature_c : '—';
    if (s) tr.children[6].className = s.temperature_c > 55 ? 'ctl-warn' : '';
  });
}

// ---------------------------------------------------------------------------
// Run traces
// ---------------------------------------------------------------------------

function fmtBytes(n) {
  return n > 1e6 ? `${(n / 1e6).toFixed(1)} MB` : `${Math.max(1, Math.round(n / 1e3))} kB`;
}

async function refreshTraces() {
  const r = await fetch('/api/policy/traces').then(x => x.json()).catch(() => null);
  const body = $('ctl-trace-tbody');
  if (!r || !body) return;
  $('ctl-trace-rec').textContent = r.recording ? '● recording' : '';
  body.replaceChildren();
  if (!r.traces || !r.traces.length) {
    const tr = body.insertRow(); const td = tr.insertCell();
    td.colSpan = 6; td.textContent = 'No traces yet.';
    return;
  }
  for (const t of r.traces) {
    const tr = body.insertRow();
    tr.insertCell().textContent = (t.modified || '').replace('T', ' ').slice(0, 19);
    tr.insertCell().textContent = t.model || '—';
    tr.insertCell().textContent = t.seconds != null ? `${t.seconds} s (${t.steps})` : '—';
    tr.insertCell().textContent = t.end_reason || t.error || '—';
    tr.insertCell().textContent = fmtBytes(t.bytes || 0);
    const a = document.createElement('a');
    a.href = `/api/policy/traces/${encodeURIComponent(t.name)}`;
    a.download = t.name;
    a.textContent = 'Download';
    tr.insertCell().appendChild(a);
  }
}

export function onControlTelemetry(frame) {
  const p = frame.policy;
  if (!p) return;

  if (p.state !== lastState) {
    // A run just ended: its trace is being saved in the background, so list it shortly.
    if (lastState === 'running') setTimeout(refreshTraces, 1000);
    if (p.state === 'running') $('ctl-trace-rec').textContent = '● recording';
    lastState = p.state;
    const el = $('ctl-state');
    el.textContent = p.state;
    el.className = `ctl-state ctl-${p.state}`;
    renderButtons(p.state);
    if (p.state === 'fault' && p.fault) {
      flashFault(`${p.fault}: ${p.fault_detail || ''}`);
    }
  }

  $('ctl-h-state').textContent = p.state;
  $('ctl-h-steps').textContent = p.steps ?? 0;
  $('ctl-h-infer').textContent = p.inference_ms != null ? `${p.inference_ms} ms` : '— ms';
  $('ctl-h-infermax').textContent = p.inference_ms_max != null ? `${p.inference_ms_max} ms` : '— ms';
  $('ctl-h-cycle').textContent = p.cycle_ms != null ? `${p.cycle_ms} ms` : '— ms';
  $('ctl-h-budget').textContent = p.budget_ms != null ? `${p.budget_ms} ms` : '— ms';
  $('ctl-h-bus').textContent = frame.bus_hz != null ? `${frame.bus_hz} Hz` : '— Hz';
  $('ctl-h-phase').textContent = p.gait_phase != null ? p.gait_phase.toFixed(2) : '—';

  // Colour the cycle time against the control budget — this is the number that
  // tells you whether inference is fitting in the loop.
  if (p.cycle_ms != null && p.budget_ms != null) {
    $('ctl-h-cycle').className = p.cycle_ms > p.budget_ms ? 'ctl-warn' : '';
  }

  const tilt = p.tilt_deg ?? 0;
  const tiltMax = p.limits ? p.limits.tilt_fault_rad * RAD2DEG : (parseFloat($('ctl-lim-tilt').value) || 45);
  $('ctl-tilt').textContent = tilt.toFixed(1);
  $('ctl-tilt-max').textContent = Number(tiltMax.toFixed(1));
  const pct = Math.min(100, (tilt / tiltMax) * 100);
  const fill = $('ctl-tilt-fill');
  fill.style.width = `${pct}%`;
  fill.className = pct > 80 ? 'ctl-danger' : (pct > 50 ? 'ctl-caution' : '');

  if (p.model) {
    const info = p.model_info || {};
    $('ctl-model-meta').textContent =
      `${p.model}${info.step ? ` — step ${info.step}` : ''}${info.git_sha ? ` (${info.git_sha})` : ''}`;
  }
  $('ctl-model-summary').textContent = modelSummary(p);
  applyCommandRanges(p.command_ranges);
  renderAuto(p.auto);

  renderJoints(p, frame.servos);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

export function initControl() {
  refreshModels();
  refreshTraces();
  $('ctl-trace-refresh').addEventListener('click', refreshTraces);

  $('ctl-estop').addEventListener('click', estop);

  $('ctl-load').addEventListener('click', async () => {
    const name = $('ctl-model-select').value;
    if (!name) return;
    await api('load', { name });
    refreshLimits();      // the bundle may have changed the action-rate / tilt limits
  });

  $('ctl-arm').addEventListener('click', () => api('arm'));
  $('ctl-start').addEventListener('click', () => { setSliders(0, 0, 0); api('start'); });
  $('ctl-stop').addEventListener('click', () => api('stop'));
  $('ctl-disarm').addEventListener('click', () => api('disarm'));
  $('ctl-clear').addEventListener('click', () => api('clear_fault'));

  ['ctl-vx', 'ctl-vy', 'ctl-wz'].forEach(id => {
    $(id).addEventListener('input', () => { syncCommandLabels(); pushCommand(); });
  });
  $('ctl-zero-cmd').addEventListener('click', () => setSliders(0, 0, 0));

  $('ctl-keys').addEventListener('change', e => {
    keysEnabled = e.target.checked;
    if (keysEnabled && padEnabled) setPadEnabled(false);
    if (!keysEnabled) { keyHeld.clear(); setSliders(0, 0, 0); }
  });

  $('ctl-pad').addEventListener('change', e => setPadEnabled(e.target.checked));
  initGamepad({
    onTick: onPadTick,
    onButton: onPadButton,
    onConnection: pad => {
      const lost = padConnected && !pad;
      padConnected = pad;
      if (lost && padEnabled) setSliders(0, 0, 0);   // pad unplugged mid-drive: stop now
      padStatus();
    },
  });
  padStatus();

  for (const id of Object.values(MAX_IDS)) {
    $(id).addEventListener('input', () => { syncMaxLabels(); applyKeys(); });
    $(id).addEventListener('change', saveMaxSpeeds);
  }
  syncMaxLabels();

  $('ctl-auto-en').addEventListener('change', e => {
    setAuto({ enabled: e.target.checked }).catch(refreshAuto);
  });
  for (const [key, id, fmt] of AUTO_SLIDERS) {
    $(id).addEventListener('input', e => { $(`${id}-val`).textContent = fmt(parseFloat(e.target.value)); });
    $(id).addEventListener('change', e => {
      setAuto({ [key]: parseFloat(e.target.value) }).catch(refreshAuto);
    });
  }
  refreshAuto();
  window.addEventListener('keydown', e => {
    if (!keysEnabled) return;
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    const k = e.key.toLowerCase();
    if ('wasdqe'.includes(k)) { keyHeld.add(k); applyKeys(); e.preventDefault(); }
  });
  window.addEventListener('keyup', e => {
    const k = e.key.toLowerCase();
    if (keyHeld.delete(k)) applyKeys();
  });
  // Releasing focus must not leave the robot driving — a held key with the window
  // backgrounded would otherwise never see its keyup.
  window.addEventListener('blur', () => {
    if (keyHeld.size) { keyHeld.clear(); setSliders(0, 0, 0); }
  });

  // Send ONLY the slider that changed: pushing all three would also write back whatever the
  // other two sliders happen to show (e.g. a tilt rounded to a whole degree).
  const limitPush = (field, id) => {
    api('safety', { [field]: parseFloat($(id).value) })
      .then(r => applyLimits(r.limits))
      .catch(refreshLimits);
  };
  $('ctl-lim-tilt').addEventListener('input', e => {
    $('ctl-lim-tilt-val').textContent = e.target.value;
  });
  $('ctl-lim-rate').addEventListener('input', e => {
    $('ctl-lim-rate-val').textContent = parseFloat(e.target.value).toFixed(2);
  });
  $('ctl-lim-scale').addEventListener('input', e => {
    $('ctl-lim-scale-val').textContent = parseFloat(e.target.value).toFixed(2);
  });
  [['tilt_fault_deg', 'ctl-lim-tilt'], ['max_action_rate', 'ctl-lim-rate'],
   ['velocity_scale', 'ctl-lim-scale']].forEach(([field, id]) => {
    $(id).addEventListener('change', () => limitPush(field, id));
  });
  refreshLimits();

  fetch('/api/policy/status').then(r => r.json()).then(s => {
    if (s.state) { lastState = s.state; $('ctl-state').textContent = s.state; renderButtons(s.state); }
    applyCommandRanges(s.command_ranges);
  }).catch(() => {});
}
