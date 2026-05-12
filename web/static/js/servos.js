/**
 * servos.js — Servo table, position sliders, PID tuning, ID change, zero calibration.
 */
import { send } from './app.js';

// State
const rows = {};  // servo_id → { el refs }

// Modal state
let modalServoid = null;
let modalMode = null;  // 'pid' | 'id'

// ── Init ─────────────────────────────────────────────────────────────────────
export function initServos() {
  document.getElementById('btn-enable-all').addEventListener('click', () =>
    fetch('/api/servos/').then(r => r.json()).then(list =>
      list.forEach(s => send({ type: 'set_torque', servo_id: s.id, enable: true }))
    )
  );
  document.getElementById('btn-disable-all').addEventListener('click', () =>
    fetch('/api/servos/').then(r => r.json()).then(list =>
      list.forEach(s => send({ type: 'set_torque', servo_id: s.id, enable: false }))
    )
  );

  // PID modal
  document.getElementById('pid-apply').addEventListener('click', applyPID);
  document.getElementById('pid-cancel').addEventListener('click', closeModal);
  // ID modal
  document.getElementById('id-apply').addEventListener('click', applyID);
  document.getElementById('id-cancel').addEventListener('click', closeModal);
  document.getElementById('modal-overlay').addEventListener('click', e => {
    if (e.target === e.currentTarget) closeModal();
  });
}

// ── Telemetry update ─────────────────────────────────────────────────────────
export function onServoTelemetry(servos) {
  servos.forEach(s => {
    if (!rows[s.id]) _createRow(s);
    _updateRow(s);
  });
}

function _createRow(s) {
  const tbody = document.getElementById('servo-tbody');
  const tr = document.createElement('tr');
  tr.dataset.id = s.id;
  tr.innerHTML = `
    <td class="s-id">${s.id}</td>
    <td class="s-joint" title="${s.joint}">${_shortName(s.joint)}</td>
    <td><input type="range" class="pos-slider" min="-180" max="180" step="0.5" value="${s.position_deg}">
        <span class="pos-value">${s.position_deg.toFixed(1)}°</span></td>
    <td class="s-speed">0</td>
    <td class="s-load">0</td>
    <td class="s-voltage">0.0</td>
    <td class="s-temp">0</td>
    <td><input type="checkbox" class="torque-toggle" ${s.torque_enabled ? 'checked' : ''}></td>
    <td><button class="cfg-btn btn-zero">Zero</button></td>
    <td>
      <button class="cfg-btn btn-pid">PID</button>
      <button class="cfg-btn btn-id">ID</button>
    </td>`;
  tbody.appendChild(tr);

  const slider = tr.querySelector('.pos-slider');
  const valSpan = tr.querySelector('.pos-value');

  // Send position on slider input (throttled)
  let tid;
  slider.addEventListener('input', () => {
    valSpan.textContent = `${parseFloat(slider.value).toFixed(1)}°`;
    clearTimeout(tid);
    tid = setTimeout(() => {
      send({ type: 'set_position', servo_id: s.id, position_deg: parseFloat(slider.value) });
    }, 60);
  });

  tr.querySelector('.torque-toggle').addEventListener('change', e => {
    send({ type: 'set_torque', servo_id: s.id, enable: e.target.checked });
  });

  tr.querySelector('.btn-zero').addEventListener('click', () => {
    if (confirm(`Zero servo ${s.id} (${s.joint}) at current position?`))
      fetch(`/api/servos/${s.id}/zero`, { method: 'POST' });
  });

  tr.querySelector('.btn-pid').addEventListener('click', () => openPIDModal(s.id, s.joint));
  tr.querySelector('.btn-id').addEventListener('click', () => openIDModal(s.id));

  rows[s.id] = {
    tr, slider, valSpan,
    speed: tr.querySelector('.s-speed'),
    load:  tr.querySelector('.s-load'),
    volt:  tr.querySelector('.s-voltage'),
    temp:  tr.querySelector('.s-temp'),
    torque: tr.querySelector('.torque-toggle'),
  };
}

function _updateRow(s) {
  const r = rows[s.id];
  if (!r) return;
  // Only update slider if user is not actively dragging
  if (document.activeElement !== r.slider) {
    r.slider.value = s.position_deg;
    r.valSpan.textContent = `${s.position_deg.toFixed(1)}°`;
  }
  _setText(r.speed, s.speed);
  _setText(r.load,  `${(s.load / 10).toFixed(0)}%`);
  _setText(r.volt,  s.voltage_v.toFixed(1));
  _setText(r.temp,  s.temperature_c);
  if (document.activeElement !== r.torque) r.torque.checked = s.torque_enabled;

  // Temperature colour
  r.temp.style.color = s.temperature_c > 60 ? 'var(--danger)'
                      : s.temperature_c > 45 ? 'var(--warning)'
                      : '';
  r.volt.style.color = s.voltage_v < 6.0 ? 'var(--danger)'
                     : s.voltage_v < 6.8 ? 'var(--warning)'
                     : '';
}

// ── PID modal ────────────────────────────────────────────────────────────────
function openPIDModal(id, joint) {
  modalServoid = id;
  modalMode = 'pid';
  document.getElementById('modal-pid-title').textContent = `${id} – ${joint}`;
  document.getElementById('modal-pid').classList.remove('hidden');
  document.getElementById('modal-id').classList.add('hidden');
  document.getElementById('modal-overlay').classList.remove('hidden');
  // Pre-fill current PID
  fetch(`/api/servos/${id}`)
    .then(r => r.json())
    .catch(() => null)
    .then(s => {
      if (!s) return;
      fetch(`/api/servos/${id}/pid`).catch(() => null);  // no-op for now
    });
}

function applyPID() {
  const p = parseInt(document.getElementById('pid-p').value);
  const d = parseInt(document.getElementById('pid-d').value);
  const i = parseInt(document.getElementById('pid-i').value);
  if ([p,d,i].some(isNaN)) return;
  send({ type: 'set_pid', servo_id: modalServoid, p, d, i });
  closeModal();
}

// ── ID modal ─────────────────────────────────────────────────────────────────
function openIDModal(id) {
  modalServoid = id;
  modalMode = 'id';
  document.getElementById('modal-id-title').textContent = id;
  document.getElementById('new-servo-id').value = id;
  document.getElementById('modal-id').classList.remove('hidden');
  document.getElementById('modal-pid').classList.add('hidden');
  document.getElementById('modal-overlay').classList.remove('hidden');
}

function applyID() {
  const newId = parseInt(document.getElementById('new-servo-id').value);
  if (isNaN(newId) || newId < 0 || newId > 253) {
    alert('ID must be 0-253');
    return;
  }
  fetch(`/api/servos/${modalServoid}/id`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ new_id: newId }),
  }).then(r => r.json()).then(r => {
    if (r.ok) alert(`ID changed: ${r.old_id} → ${r.new_id}. Reconnect to verify.`);
  });
  closeModal();
}

// ── Helpers ──────────────────────────────────────────────────────────────────
function closeModal() {
  document.getElementById('modal-overlay').classList.add('hidden');
  modalServoid = null; modalMode = null;
}

function _setText(el, val) {
  const s = String(val);
  if (el.textContent !== s) el.textContent = s;
}

function _shortName(joint) {
  return joint
    .replace('_joint', '')
    .replace(/_/g, ' ')
    .replace(/l hip/,  'L Hip')
    .replace(/r hip/,  'R Hip')
    .replace(/l knee/, 'L Knee')
    .replace(/r knee/, 'R Knee')
    .replace(/l ankle/,'L Ankle')
    .replace(/r ankle/,'R Ankle');
}
