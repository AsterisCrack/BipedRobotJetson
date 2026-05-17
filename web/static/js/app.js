/**
 * app.js — Central WebSocket manager and tab router.
 * Dispatches incoming telemetry frames to module listeners.
 */

import { initServos, onServoTelemetry } from './servos.js';
import { onIMUTelemetry } from './imu.js';
import { initRobot3D, onRobotTelemetry, onFKResult } from './robot3d.js';
import { initIdManager } from './id_manager.js';
import { initDebug, onDebugTelemetry } from './debug.js';

// ── Tab routing ──────────────────────────────────────────────────────────────
document.querySelectorAll('.tab-btn').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    document.querySelectorAll('.tab-panel').forEach(p => p.classList.remove('active'));
    btn.classList.add('active');
    document.getElementById(`tab-${btn.dataset.tab}`).classList.add('active');
  });
});

// ── WebSocket ────────────────────────────────────────────────────────────────
const wsStatus = document.getElementById('ws-status');
let ws = null;
const listeners = {};   // type → [fn]

export function on(type, fn) {
  (listeners[type] ||= []).push(fn);
}

export function send(msg) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(msg));
  }
}

function connect() {
  const url = `ws://${location.host}/ws`;
  ws = new WebSocket(url);

  ws.onopen = () => {
    wsStatus.textContent = 'Connected';
    wsStatus.className = 'ws-status connected';
  };
  ws.onclose = () => {
    wsStatus.textContent = 'Disconnected';
    wsStatus.className = 'ws-status disconnected';
    setTimeout(connect, 2000);
  };
  ws.onerror = () => {
    wsStatus.textContent = 'Error';
    wsStatus.className = 'ws-status disconnected';
  };
  ws.onmessage = ({ data }) => {
    let msg;
    try { msg = JSON.parse(data); } catch { return; }
    const fns = listeners[msg.type] || [];
    fns.forEach(fn => fn(msg));
  };
}

// ── Collapsible sections ─────────────────────────────────────────────────────
document.querySelectorAll('.collapse-header').forEach(h => {
  h.addEventListener('click', () => {
    h.classList.toggle('open');
    const body = h.closest('.collapsible').querySelector('.collapse-body');
    body.classList.toggle('hidden');
  });
});

// ── Init modules ─────────────────────────────────────────────────────────────
on('telemetry', msg => {
  onServoTelemetry(msg.servos);
  onIMUTelemetry(msg.imu);
  onRobotTelemetry(msg);
  onDebugTelemetry(msg.servos);
});
on('ik_result', msg => {
  if (!msg.success) {
    console.warn('IK failed:', msg.message);
  }
});
on('fk_result', onFKResult);
on('error', msg => console.error('Server error:', msg.message));

// ── Simulation mode toggle ───────────────────────────────────────────────────
const simBtn = document.getElementById('btn-sim-mode');
function _applySimState(enabled) {
  simBtn.textContent = enabled ? 'Sim Mode: ON' : 'Sim Mode: OFF';
  simBtn.classList.toggle('active', enabled);
}
simBtn.addEventListener('click', () => {
  const next = !simBtn.classList.contains('active');
  fetch('/api/servos/simulation', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enabled: next }),
  })
    .then(r => r.json())
    .then(d => _applySimState(d.enabled));
});
fetch('/api/servos/simulation').then(r => r.json()).then(d => _applySimState(d.enabled));

initServos();
initRobot3D();
initIdManager();
initDebug();
connect();
