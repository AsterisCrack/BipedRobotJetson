/**
 * debug.js — Single-servo debug controls and telemetry graph.
 */

const $ = id => document.getElementById(id);

let activeId = null;
const history = [];
const maxPoints = 300;
let lastTelemetryAt = 0;

const metrics = [
  { key: 'raw_deg',      label: 'Raw Deg',   color: '#4f8ef7', min: 0, max: 360 },
  { key: 'speed',        label: 'Speed',     color: '#3ecf8e', min: 0,   max: 3400 },
  { key: 'load_pct',     label: 'Load %',    color: '#f0a030', min: 0,   max: 100 },
  { key: 'voltage_v',    label: 'Volt (V)',  color: '#c8cc30', min: 0,   max: 12 },
  { key: 'temperature_c',label: 'Temp C',    color: '#e05555', min: 0,   max: 100 },
];

export function initDebug() {
  const select = $('debug-servo-select');
  const manual = $('debug-servo-id');
  const useBtn = $('debug-use-id');
  const scanBtn = $('debug-scan');
  const torqueToggle = $('debug-torque-toggle');
  const zeroBtn = $('debug-zero');

  if (!select || !manual || !useBtn || !scanBtn || !torqueToggle || !zeroBtn) return;

  scanBtn.addEventListener('click', () => {
    const refresh = $('id-refresh');
    if (refresh) refresh.click();
    scanIds();
  });

  useBtn.addEventListener('click', () => {
    const id = parseInt(manual.value, 10);
    if (!Number.isNaN(id)) setActiveId(id);
  });

  select.addEventListener('change', () => {
    const id = parseInt(select.value, 10);
    if (!Number.isNaN(id)) setActiveId(id);
  });

  torqueToggle.addEventListener('change', () => {
    if (!activeId && activeId !== 0) return;
    fetch(`/api/servos/${activeId}/torque`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ enable: torqueToggle.checked }),
    }).catch(() => null);
  });

  zeroBtn.addEventListener('click', () => {
    if (!activeId && activeId !== 0) return;
    fetch(`/api/servos/${activeId}/zero_offset`, { method: 'POST' }).catch(() => null);
  });

  bindSlider('debug-position', 'debug-position-val');
  bindSlider('debug-speed', 'debug-speed-val');
  bindSlider('debug-accel', 'debug-accel-val');
  bindSlider('debug-torque-limit', 'debug-torque-limit-val');

  const moveBtn = $('debug-move');
  const speedApply = $('debug-speed-apply');
  const accelApply = $('debug-accel-apply');
  const torqueLimitApply = $('debug-torque-limit-apply');
  const pidApply = $('debug-pid-apply');

  if (!moveBtn || !speedApply || !accelApply || !torqueLimitApply || !pidApply) return;

  moveBtn.addEventListener('click', () => {
    if (!activeId && activeId !== 0) return;
    const steps = parseInt($('debug-position-val').value, 10);
    const speed = parseInt($('debug-speed-val').value, 10);
    fetch(`/api/servos/${activeId}/raw_position`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ steps, speed: Number.isNaN(speed) ? 0 : speed }),
    }).catch(() => null);
  });

  speedApply.addEventListener('click', () => {
    if (!activeId && activeId !== 0) return;
    const speed = parseInt($('debug-speed-val').value, 10);
    fetch(`/api/servos/${activeId}/speed`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ speed }),
    }).catch(() => null);
  });

  accelApply.addEventListener('click', () => {
    if (!activeId && activeId !== 0) return;
    const accel = parseInt($('debug-accel-val').value, 10);
    fetch(`/api/servos/${activeId}/accel`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ accel }),
    }).catch(() => null);
  });

  torqueLimitApply.addEventListener('click', () => {
    if (!activeId && activeId !== 0) return;
    const limit = parseInt($('debug-torque-limit-val').value, 10);
    fetch(`/api/servos/${activeId}/torque_limit`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ limit }),
    }).catch(() => null);
  });

  pidApply.addEventListener('click', () => {
    if (!activeId && activeId !== 0) return;
    const p = parseInt($('debug-p').value, 10);
    const d = parseInt($('debug-d').value, 10);
    const i = parseInt($('debug-i').value, 10);
    fetch(`/api/servos/${activeId}/pid`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ p, d, i }),
    }).catch(() => null);
  });

  scanIds();
  setInterval(pollStatus, 250);
}

export function onDebugTelemetry(servos) {
  if (!activeId && activeId !== 0) return;
  const s = servos.find(item => item.id === activeId);
  if (s && s.raw_deg !== undefined) {
    lastTelemetryAt = performance.now();
    pushSample(s);
  }
}

function pollStatus() {
  if (!activeId && activeId !== 0) return;
  const now = performance.now();
  if (now - lastTelemetryAt < 400) return;
  fetch(`/api/servos/${activeId}`)
    .then(r => r.json())
    .then(status => {
      lastTelemetryAt = performance.now();
      pushSample(status);
    })
    .catch(() => null);
}

function pushSample(s) {
  if (!s) return;

  history.push({
    t: performance.now(),
    raw_deg: s.raw_deg,
    speed: s.speed || 0,
    load_pct: s.load ? s.load / 10 : 0,
    voltage_v: s.voltage_v || 0,
    temperature_c: s.temperature_c || 0,
  });
  if (history.length > maxPoints) history.shift();

  drawGraph();
  updateLegend(s);
  if (s.torque_enabled !== undefined) syncTorqueToggle(s.torque_enabled);
}

function bindSlider(rangeId, inputId) {
  const range = $(rangeId);
  const input = $(inputId);
  if (!range || !input) return;

  const sync = val => {
    range.value = val;
    input.value = val;
  };

  range.addEventListener('input', () => sync(range.value));
  input.addEventListener('input', () => sync(input.value));
}

function scanIds() {
  disableAllTorques();
  const select = $('debug-servo-select');
  const status = $('debug-status');
  if (!select) return;

  select.innerHTML = '';
  setStatus(status, 'Scanning...');

  fetch('/api/servos/scan')
    .then(r => r.json())
    .then(list => {
      if (!Array.isArray(list) || list.length === 0) {
        setStatus(status, 'No servos found');
        return;
      }
      list.forEach(item => {
        const opt = document.createElement('option');
        opt.value = item.id;
        opt.textContent = `${item.id}${item.joint ? ' — ' + item.joint : ''}`;
        select.appendChild(opt);
      });
      setStatus(status, `Found ${list.length} servo${list.length === 1 ? '' : 's'}.`);
      if (activeId === null) {
        setActiveId(parseInt(select.value, 10));
      }
    })
    .catch(() => setStatus(status, 'Scan failed'));
}

function setActiveId(id) {
  activeId = id;
  const select = $('debug-servo-select');
  const manual = $('debug-servo-id');
  if (select) select.value = String(id);
  if (manual) manual.value = String(id);
  setStatus($('debug-status'), `Active ID: ${id}`);
  history.length = 0;
  disableTorqueFor(id);
  fetch(`/api/servos/${id}`)
    .then(r => r.json())
    .then(status => {
      if (status.raw_steps !== undefined) {
        $('debug-position').value = status.raw_steps;
        $('debug-position-val').value = status.raw_steps;
      }
    })
    .catch(() => null);
}

function disableAllTorques() {
  fetch('/api/servos/scan')
    .then(r => r.json())
    .then(list => list.forEach(s => disableTorqueFor(s.id)));
}

function disableTorqueFor(id) {
  fetch(`/api/servos/${id}/torque`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enable: false }),
  }).catch(() => null);
}

function setStatus(el, text) {
  if (el) el.textContent = text;
}

function syncTorqueToggle(enabled) {
  const toggle = $('debug-torque-toggle');
  if (toggle && document.activeElement !== toggle) toggle.checked = enabled;
}

function drawGraph() {
  const canvas = $('debug-graph');
  if (!canvas) return;

  const ctx = canvas.getContext('2d');
  const w = canvas.width;
  const h = canvas.height;
  const band = h / metrics.length;

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0d0d1a';
  ctx.fillRect(0, 0, w, h);

  ctx.strokeStyle = '#2a2a4a';
  ctx.lineWidth = 1;
  for (let i = 1; i < metrics.length; i += 1) {
    const y = i * band;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(w, y);
    ctx.stroke();
  }

  metrics.forEach((m, idx) => {
    const y0 = idx * band;
    const y1 = y0 + band;
    ctx.strokeStyle = m.color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    history.forEach((pt, i) => {
      const x = (i / Math.max(1, maxPoints - 1)) * w;
      const val = pt[m.key];
      const t = (val - m.min) / (m.max - m.min);
      const y = y1 - Math.max(0, Math.min(1, t)) * (band - 10) - 5;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    ctx.fillStyle = '#7070a0';
    ctx.font = '11px sans-serif';
    ctx.fillText(m.label, 8, y0 + 14);
  });
}

function updateLegend(s) {
  const el = $('debug-legend');
  if (!el) return;
  el.innerHTML = '';

  const items = [
    { label: 'Raw', color: '#4f8ef7', value: s.raw_deg !== undefined ? `${s.raw_deg.toFixed(1)} deg` : '-' },
    { label: 'Speed', color: '#3ecf8e', value: `${s.speed}` },
    { label: 'Load', color: '#f0a030', value: s.load !== undefined ? `${(s.load / 10).toFixed(0)} %` : '-' },
    { label: 'Volt', color: '#c8cc30', value: s.voltage_v !== undefined ? `${s.voltage_v.toFixed(2)} V` : '-' },
    { label: 'Temp', color: '#e05555', value: s.temperature_c !== undefined ? `${s.temperature_c} C` : '-' },
  ];

  items.forEach(item => {
    const span = document.createElement('span');
    const dot = document.createElement('i');
    dot.className = 'legend-dot';
    dot.style.background = item.color;
    span.appendChild(dot);
    span.appendChild(document.createTextNode(`${item.label}: ${item.value}`));
    el.appendChild(span);
  });
}
