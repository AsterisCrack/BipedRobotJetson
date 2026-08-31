/**
 * policy.js — Policy tab: load model, WASD/QE velocity commands, fall state display.
 */

import { on, send } from './app.js';

const HELD_KEYS = new Set();

// Default magnitudes for each axis (m/s and rad/s)
const DEFAULT_VX_MAG = 0.3;
const DEFAULT_VY_MAG = 0.2;
const DEFAULT_WZ_MAG = 0.5;

// Map: key → [axis, sign]
const KEY_MAP = {
    'w': ['vx',  1],
    's': ['vx', -1],
    'a': ['vy',  1],
    'd': ['vy', -1],
    'q': ['wz',  1],
    'e': ['wz', -1],
};

export function initPolicy() {
    document.addEventListener('keydown', _onKeyDown);
    document.addEventListener('keyup',   _onKeyUp);

    document.getElementById('btn-policy-load').addEventListener('click', _loadPolicy);
    document.getElementById('btn-policy-enable').addEventListener('click', () => _setEnabled(true));
    document.getElementById('btn-policy-disable').addEventListener('click', () => _setEnabled(false));

    // Fetch initial status
    fetch('/api/policy/status')
        .then(r => r.json())
        .then(_applyStatus)
        .catch(() => {});

    on('telemetry', msg => {
        if (msg.policy) _applyStatus(msg.policy);
    });
}

// ── Keyboard ─────────────────────────────────────────────────────────────────

function _onKeyDown(e) {
    const key = e.key.toLowerCase();
    if (!(key in KEY_MAP)) return;
    // Only act if policy tab is visible and policy panel exists
    if (!document.getElementById('tab-policy').classList.contains('active')) return;
    if (HELD_KEYS.has(key)) return; // already held
    e.preventDefault();
    HELD_KEYS.add(key);
    _updateKeyDisplay();
    _sendVelocity();
}

function _onKeyUp(e) {
    const key = e.key.toLowerCase();
    if (!(key in KEY_MAP)) return;
    HELD_KEYS.delete(key);
    _updateKeyDisplay();
    _sendVelocity();
}

function _sendVelocity() {
    const vxMag = parseFloat(document.getElementById('mag-vx').value) || 0;
    const vyMag = parseFloat(document.getElementById('mag-vy').value) || 0;
    const wzMag = parseFloat(document.getElementById('mag-wz').value) || 0;

    let vx = 0, vy = 0, wz = 0;
    for (const key of HELD_KEYS) {
        const [axis, sign] = KEY_MAP[key];
        if (axis === 'vx') vx += sign * vxMag;
        if (axis === 'vy') vy += sign * vyMag;
        if (axis === 'wz') wz += sign * wzMag;
    }

    send({ type: 'set_velocity', vx, vy, wz });
    document.getElementById('cmd-vx').textContent = vx.toFixed(2);
    document.getElementById('cmd-vy').textContent = vy.toFixed(2);
    document.getElementById('cmd-wz').textContent = wz.toFixed(2);
}

function _updateKeyDisplay() {
    for (const [key] of Object.entries(KEY_MAP)) {
        const el = document.getElementById(`key-${key}`);
        if (el) el.classList.toggle('active', HELD_KEYS.has(key));
    }
}

// ── Load / enable ─────────────────────────────────────────────────────────────

function _loadPolicy() {
    const weights = document.getElementById('policy-weights-path').value.trim();
    const submodule = document.getElementById('policy-submodule-path').value.trim();
    const config = document.getElementById('policy-config-path').value.trim() || null;
    const scale = parseFloat(document.getElementById('policy-action-scale').value) || 40.0;

    if (!weights || !submodule) {
        _setLoadStatus('Weights and submodule paths are required', 'error');
        return;
    }

    _setLoadStatus('Loading…', '');
    fetch('/api/policy/load', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            weights_path: weights,
            submodule_path: submodule,
            config_path: config,
            action_scale_deg: scale,
        }),
    })
        .then(async r => {
            const data = await r.json();
            if (!r.ok) throw new Error(data.detail || 'Load failed');
            _setLoadStatus('Loaded', 'ok');
            document.getElementById('btn-policy-enable').disabled = false;
        })
        .catch(err => _setLoadStatus(err.message, 'error'));
}

function _setEnabled(enabled) {
    send({ type: 'set_policy_enabled', enabled });
}

function _setLoadStatus(msg, cls) {
    const el = document.getElementById('policy-load-status');
    el.textContent = msg;
    el.className = 'policy-load-status ' + cls;
}

// ── Telemetry ─────────────────────────────────────────────────────────────────

function _applyStatus(status) {
    const stateEl = document.getElementById('policy-state');
    const state = status.state ?? 'idle';
    stateEl.textContent = state.charAt(0).toUpperCase() + state.slice(1);
    stateEl.className = 'policy-state-badge policy-state-' + state;

    if (status.loaded !== undefined) {
        document.getElementById('btn-policy-enable').disabled = !status.loaded;
    }

    // Update live command display from telemetry when running
    if (state === 'running') {
        document.getElementById('cmd-vx').textContent = (status.vx ?? 0).toFixed(2);
        document.getElementById('cmd-vy').textContent = (status.vy ?? 0).toFixed(2);
        document.getElementById('cmd-wz').textContent = (status.wz ?? 0).toFixed(2);
    }
}
