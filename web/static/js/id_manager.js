/**
 * id_manager.js — Servo ID reassignment page.
 */

const $ = id => document.getElementById(id);

export function initIdManager() {
  const refreshBtn = $('id-refresh');
  const disableBtn = $('id-disable-all');
  const applyManualBtn = $('id-apply-manual');

  if (!refreshBtn || !disableBtn || !applyManualBtn) return;

  refreshBtn.addEventListener('click', refreshList);
  disableBtn.addEventListener('click', disableAllTorques);
  applyManualBtn.addEventListener('click', applyManualChange);

  refreshList();
}

function refreshList() {
  const tbody = $('id-tbody');
  const status = $('id-status');
  if (!tbody) return;
  tbody.innerHTML = '';
  _setStatus(status, 'Scanning IDs 0-253...');

  disableAllTorques();

  fetch('/api/servos/scan')
    .then(r => r.json())
    .then(list => {
      if (!Array.isArray(list) || list.length === 0) {
        _setStatus(status, 'No servos reported. Check power and bus.');
        return;
      }
      _setStatus(status, `Found ${list.length} servo${list.length === 1 ? '' : 's'}.`);
      list.forEach(s => tbody.appendChild(_rowForServo(s)));
    })
    .catch(() => _setStatus(status, 'Failed to load servos.'));
}

function _rowForServo(s) {
  const tr = document.createElement('tr');
  const jointLabel = s.joint ? _shortName(s.joint) : 'Unknown';
  tr.innerHTML = `
    <td class="id-current">${s.id}</td>
    <td class="id-joint" title="${s.joint || 'Unknown'}">${jointLabel}</td>
    <td><input type="number" class="id-new" min="0" max="253" value="${s.id}"></td>
    <td><button class="primary">Apply</button></td>
    <td class="id-msg"></td>
  `;

  const btn = tr.querySelector('button');
  btn.addEventListener('click', () => {
    const input = tr.querySelector('.id-new');
    const newId = parseInt(input.value, 10);
    if (Number.isNaN(newId) || newId < 0 || newId > 253) {
      _setRowMsg(tr, 'ID must be 0-253');
      return;
    }
    _changeId(s.id, newId, tr);
  });

  return tr;
}

function applyManualChange() {
  const oldEl = $('id-old');
  const newEl = $('id-new');
  if (!oldEl || !newEl) return;

  const oldId = parseInt(oldEl.value, 10);
  const newId = parseInt(newEl.value, 10);
  if (Number.isNaN(oldId) || oldId < 0 || oldId > 253) {
    alert('Current ID must be 0-253');
    return;
  }
  if (Number.isNaN(newId) || newId < 0 || newId > 253) {
    alert('New ID must be 0-253');
    return;
  }
  _changeId(oldId, newId);
}

function _changeId(oldId, newId, row) {
  if (!confirm(`Change servo ID ${oldId} → ${newId}?`)) return;
  const status = row ? row.querySelector('.id-msg') : $('id-status');
  _setStatus(status, 'Writing ID register...');

  fetch(`/api/servos/${oldId}/torque`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ enable: false }),
  }).catch(() => null);

  fetch(`/api/servos/${oldId}/id`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ new_id: newId }),
  })
    .then(r => r.json())
    .then(r => {
      if (!r.ok) throw new Error('Failed');
      _setStatus(status, `Updated. Reconnect to verify (0x05 = ${newId}).`);
      setTimeout(refreshList, 300);
    })
    .catch(() => _setStatus(status, 'Failed to change ID.'));
}

function disableAllTorques() {
  fetch('/api/servos/scan')
    .then(r => r.json())
    .then(list => list.forEach(s =>
      fetch(`/api/servos/${s.id}/torque`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enable: false }),
      })
    ));
}

function _setStatus(el, text) {
  if (!el) return;
  el.textContent = text;
}

function _setRowMsg(tr, text) {
  const el = tr.querySelector('.id-msg');
  if (el) el.textContent = text;
}

function _shortName(joint) {
  return String(joint || '')
    .replace('_joint', '')
    .replace(/_/g, ' ')
    .replace(/l hip/,  'L Hip')
    .replace(/r hip/,  'R Hip')
    .replace(/l knee/, 'L Knee')
    .replace(/r knee/, 'R Knee')
    .replace(/l ankle/,'L Ankle')
    .replace(/r ankle/,'R Ankle');
}
