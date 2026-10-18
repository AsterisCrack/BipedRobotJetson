// Control tab — RL policy arm/drive/monitor.
//
// Follows the module conventions used by servos.js / imu.js:
//   initControl()          wire DOM listeners once, at load
//   onControlTelemetry()   called for every telemetry frame
// WebSocket for streaming + continuous commands, fetch() for one-shot actions.

import { send } from './app.js';

const $ = id => document.getElementById(id);

let lastState = null;
let cmdTimer = null;
let keysEnabled = false;
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

function setSliders(vx, vy, wz) {
  $('ctl-vx').value = vx; $('ctl-vy').value = vy; $('ctl-wz').value = wz;
  syncCommandLabels();
  pushCommand();
}

function syncCommandLabels() {
  $('ctl-vx-val').textContent = parseFloat($('ctl-vx').value).toFixed(2);
  $('ctl-vy-val').textContent = parseFloat($('ctl-vy').value).toFixed(2);
  $('ctl-wz-val').textContent = parseFloat($('ctl-wz').value).toFixed(2);
}

function applyKeys() {
  if (!keysEnabled) return;
  const vx = (keyHeld.has('w') ? 0.5 : 0) + (keyHeld.has('s') ? -0.3 : 0);
  const vy = (keyHeld.has('a') ? 0.3 : 0) + (keyHeld.has('d') ? -0.3 : 0);
  const wz = (keyHeld.has('q') ? 0.3 : 0) + (keyHeld.has('e') ? -0.3 : 0);
  setSliders(vx, vy, wz);
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
    parts.push(gc.mode === 'fixed_period'
      ? `clock fixed_period ${Number(gc.period).toFixed(2)} s`
      : `clock ${gc.mode} ${Number(gc.freq).toFixed(2)} Hz/(m/s)`);
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

  renderJoints(p, frame.servos);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------

export function initControl() {
  refreshModels();
  refreshTraces();
  $('ctl-trace-refresh').addEventListener('click', refreshTraces);

  // E-STOP: fire over BOTH transports. The socket is lower latency, the REST call
  // is the one that still works if the socket has silently dropped. An e-stop
  // should not depend on a single channel being healthy.
  $('ctl-estop').addEventListener('click', () => {
    send({ type: 'policy_estop' });
    fetch('/api/policy/estop', { method: 'POST' }).catch(() => {});
    setSliders(0, 0, 0);
  });

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
    if (!keysEnabled) { keyHeld.clear(); setSliders(0, 0, 0); }
  });
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
  }).catch(() => {});
}
