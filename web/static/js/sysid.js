/**
 * sysid.js — SysID tab: record bench trajectories for BAM, and screen all joints in situ.
 *
 * Everything goes through /api/sysid (web/routers/sysid.py). Recordings run on the bus thread
 * server-side; this page only starts them, polls /live for samples while one is running
 * (10 Hz), and polls /status for files and results (1 Hz).
 *
 * The coverage grid is the protocol checklist: every trajectory x kp, for each weight hole.
 * BAM's validation holds one kp out entirely, so a full grid is what makes the fit testable.
 */

const $ = id => document.getElementById(id);

let lastStatus = null;
let liveNext = 0;
let liveSamples = [];
let livePoll = null;
let autoRunning = false;

async function api(path, body) {
  const res = await fetch(`/api/sysid/${path}`, body === undefined ? {} : {
    method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) {
    $('sid-error').textContent = data.detail || `HTTP ${res.status}`;
    throw new Error(data.detail || res.status);
  }
  $('sid-error').textContent = '';
  return data;
}

// ── status rendering ──────────────────────────────────────────────────────────

function renderRig(rig) {
  const meta = $('sid-rig-meta');
  const tbody = $('sid-rig-tbody');
  tbody.innerHTML = '';
  if (!rig || !rig.available) {
    meta.textContent = rig ? `No loads.json yet. ${rig.hint} (${rig.path})` : '';
    return;
  }
  meta.textContent = `Payload ${(rig.payload_mass_kg * 1000).toFixed(0)} g, arm ${(rig.arm_mass_kg * 1000).toFixed(0)} g, ` +
                     `vin ${rig.vin} V. Mass/length are the exact point-pendulum equivalent (rig.py).`;
  rig.loads.forEach((l, i) => {
    tbody.insertAdjacentHTML('beforeend',
      `<tr><td>h${i}</td><td>${(l.hole_m * 1000).toFixed(0)}</td><td>${l.mass.toFixed(3)}</td>` +
      `<td>${(l.length * 1000).toFixed(1)}</td><td>${l.static_torque_nm.toFixed(3)}</td></tr>`);
  });
}

function fillSelect(sel, items, label) {
  const keep = sel.value;
  sel.innerHTML = items.map(([v, t]) => `<option value="${v}">${label(t)}</option>`).join('');
  if (items.some(([v]) => String(v) === keep)) sel.value = keep;
}

function renderControls(st) {
  fillSelect($('sid-traj'), st.trajectories.map(t => [t, t]), t => t);
  const kps = (st.rig && st.rig.kp_sweep) || [4, 8, 16, 32];
  fillSelect($('sid-kp'), kps.map(k => [k, k]), k => `kp ${k}`);
  const holes = (st.rig && st.rig.loads) || [];
  fillSelect($('sid-hole'), holes.map((l, i) => [i, l]), l => `${(l.hole_m * 1000).toFixed(0)} mm hole`);

  // autoRunning is included even though its own pauses leave st.state === 'idle': otherwise
  // a manual click during the "move the weight" pause would race the sequence's next /run.
  const busy = st.state !== 'idle' || autoRunning;
  $('sid-state').textContent = st.state;
  $('sid-zero-meta').textContent = st.zero_steps == null
    ? 'Torque goes off; let the arm hang still.'
    : `zero = ${st.zero_steps} steps (servo ${st.servo_id}, session ${st.session})`;
  $('sid-start-session').disabled = busy;
  $('sid-zero').disabled = busy || !st.session;
  $('sid-run').disabled = busy || st.zero_steps == null || !(st.rig && st.rig.available);
  $('sid-screen').disabled = busy;
  $('sid-auto-start').disabled = busy || st.zero_steps == null || !(st.rig && st.rig.available);
  if (st.last_error) $('sid-error').textContent = st.last_error;

  const last = st.last;
  if (last && last.n !== undefined) {
    $('sid-run-meta').textContent =
      `${last.saved ? 'saved ' + last.saved : 'NOT saved (' + last.reason + ')'} · ${last.n} samples · ` +
      `${last.achieved_hz} Hz · ${last.gaps} gaps · ${last.mean_volts} V · ${last.temp_start}→${last.temp_end} °C`;
  }
}

// Parses recording filenames into "trajectory|kp|hole" keys. Shared by the coverage
// grid and by buildCombos(), which needs the same "already recorded" set to skip cells.
function doneSet(st) {
  const files = st.files || [];
  return new Set(files.map(f => {
    const m = f.match(/_([a-z_]+)_kp(\d+)_h(\d+)\.json$/);
    return m ? `${m[1]}|${m[2]}|${m[3]}` : null;
  }).filter(Boolean));
}

function holeLabel(st, idx) {
  const loads = (st.rig && st.rig.loads) || [];
  return loads[idx] ? `${(loads[idx].hole_m * 1000).toFixed(0)} mm` : `hole ${idx}`;
}

// Every (hole, kp, trajectory) cell not yet recorded, ordered hole-major so "Record all
// remaining" finishes one weight position before asking to move to the next.
function buildCombos(st) {
  const kps = (st.rig && st.rig.kp_sweep) || [4, 8, 16, 32];
  const holes = ((st.rig && st.rig.loads) || []).map((_, i) => i);
  const trajectories = st.trajectories || [];
  const done = doneSet(st);
  const combos = [];
  for (const h of holes) {
    for (const k of kps) {
      for (const t of trajectories) {
        if (!done.has(`${t}|${k}|${h}`)) combos.push({ hole: h, kp: k, trajectory: t });
      }
    }
  }
  return combos;
}

function renderFiles(st) {
  const files = st.files || [];
  $('sid-files').innerHTML = files.map(f => `<li>${f}</li>`).join('');
  $('sid-download').style.display = st.session ? '' : 'none';

  // Coverage: trajectory rows x (hole, kp) columns.
  const kps = (st.rig && st.rig.kp_sweep) || [4, 8, 16, 32];
  const holes = ((st.rig && st.rig.loads) || []).map((_, i) => i);
  const done = doneSet(st);
  const cols = holes.flatMap(h => kps.map(k => [h, k]));
  const grid = $('sid-grid');
  grid.style.gridTemplateColumns = `8rem repeat(${cols.length}, 1fr)`;
  let html = '<span class="head"></span>' + cols.map(([h, k]) => `<span class="head">h${h}·${k}</span>`).join('');
  let n = 0;
  for (const t of st.trajectories) {
    html += `<span class="head">${t}</span>`;
    for (const [h, k] of cols) {
      const ok = done.has(`${t}|${k}|${h}`);
      n += ok ? 1 : 0;
      html += `<span class="${ok ? 'done' : ''}">${ok ? '✓' : ''}</span>`;
    }
  }
  grid.innerHTML = html;
  const total = st.trajectories.length * cols.length;
  $('sid-coverage').textContent = total
    ? `${n}/${total} cells recorded (~${Math.round(total * 6 / 60)} min of data for the full grid).`
    : 'Load the rig to see the protocol grid.';
}

function renderScreen(st) {
  const tbody = $('sid-screen-tbody');
  const joints = (st.screen && st.screen.joints) || {};
  tbody.innerHTML = Object.entries(joints).map(([name, r]) =>
    `<tr><td>${name}</td><td>${r.hysteresis_deg ?? '—'}</td><td>${r.lag_up_deg ?? '—'}</td>` +
    `<td>${r.lag_down_deg ?? '—'}</td><td>${r.achieved_hz ?? '—'}</td></tr>`).join('');
}

function renderAll(st) {
  renderRig(st.rig);
  renderControls(st);
  renderFiles(st);
  renderScreen(st);
}

async function refresh() {
  try {
    const st = await (await fetch('/api/sysid/status')).json();
    const wasRunning = lastStatus && lastStatus.state !== 'idle';
    lastStatus = st;
    renderAll(st);
    if (st.state !== 'idle' && !livePoll) startLive();
    if (st.state === 'idle' && wasRunning) stopLive();
  } catch { /* server restarting; the next tick retries */ }
}

// ── auto-record: run every un-recorded (hole, kp, trajectory) cell in sequence ─────────────

async function fetchStatus() {
  const st = await (await fetch('/api/sysid/status')).json();
  lastStatus = st;
  renderAll(st);
  return st;
}

async function waitIdle() {
  let st = await fetchStatus();
  while (st.state !== 'idle') {
    await new Promise(r => setTimeout(r, 300));
    st = await fetchStatus();
  }
  return st;
}

function setAutoUI(running, meta) {
  autoRunning = running;
  $('sid-auto-start').style.display = running ? 'none' : '';
  $('sid-auto-stop').style.display = running ? '' : 'none';
  if (meta !== undefined) $('sid-auto-meta').textContent = meta;
}

// Blocks until the user confirms the weight has been moved. Used once per hole, including
// the first, so there is never ambiguity about which hole is currently mounted.
function pauseForHole(label) {
  return new Promise(resolve => {
    $('sid-auto-modal-text').textContent = `Move the weight to the ${label} hole, then click Continue.`;
    $('sid-auto-modal').classList.remove('hidden');
    const onClick = () => {
      $('sid-auto-modal').classList.add('hidden');
      $('sid-auto-continue').removeEventListener('click', onClick);
      resolve();
    };
    $('sid-auto-continue').addEventListener('click', onClick);
  });
}

function stopAuto() {
  autoRunning = false;
  $('sid-auto-modal').classList.add('hidden');
  fetch('/api/sysid/abort', { method: 'POST' }).catch(() => {});
}

async function runAutoSequence() {
  let st = await waitIdle();
  const combos = buildCombos(st);
  if (!combos.length) {
    $('sid-auto-meta').textContent = 'Nothing to record — the grid is already complete.';
    return;
  }
  setAutoUI(true);
  let lastHole = null;
  let stoppedEarly = false;
  for (let i = 0; i < combos.length; i++) {
    if (!autoRunning) { stoppedEarly = true; break; }
    const c = combos[i];
    if (c.hole !== lastHole) {
      lastHole = c.hole;
      $('sid-auto-meta').textContent = `Waiting — move the weight to ${holeLabel(st, c.hole)}…`;
      await pauseForHole(holeLabel(st, c.hole));
      if (!autoRunning) { stoppedEarly = true; break; }
    }
    $('sid-auto-meta').textContent =
      `Recording ${i + 1}/${combos.length}: ${c.trajectory} · kp${c.kp} · ${holeLabel(st, c.hole)}`;
    try {
      await api('run', { trajectory: c.trajectory, kp: c.kp, hole: c.hole });
    } catch {
      stoppedEarly = true;   // api() already surfaced the error in #sid-error
      break;
    }
    startLive();
    st = await waitIdle();
  }
  setAutoUI(false, stoppedEarly ? 'Auto-record stopped.' : 'Auto-record finished — grid complete.');
  refresh();
}

// ── live plot ─────────────────────────────────────────────────────────────────

function startLive() {
  liveNext = 0;
  liveSamples = [];
  livePoll = setInterval(async () => {
    try {
      const d = await (await fetch(`/api/sysid/live?since=${liveNext}`)).json();
      liveSamples.push(...d.samples);
      liveNext = d.next;
      drawPlot();
      if (d.state === 'idle') stopLive();
    } catch { /* ignore one missed poll */ }
  }, 100);
}

function stopLive() {
  if (livePoll) clearInterval(livePoll);
  livePoll = null;
  drawPlot();
}

function drawPlot() {
  const cv = $('sid-plot');
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.fillStyle = '#0d0d1a';
  ctx.fillRect(0, 0, W, H);
  if (liveSamples.length < 2) return;

  const t0 = liveSamples[0].t;
  const tSpan = Math.max(6, liveSamples[liveSamples.length - 1].t - t0);
  let lo = Infinity, hi = -Infinity;
  for (const s of liveSamples) { lo = Math.min(lo, s.goal, s.pos); hi = Math.max(hi, s.goal, s.pos); }
  const pad = (hi - lo) * 0.1 || 0.1;
  lo -= pad; hi += pad;
  const x = t => ((t - t0) / tSpan) * W;
  const y = v => H - ((v - lo) / (hi - lo)) * H;

  // torque-off shading
  ctx.fillStyle = 'rgba(255, 90, 90, .18)';
  for (let i = 1; i < liveSamples.length; i++) {
    if (!liveSamples[i].torque) ctx.fillRect(x(liveSamples[i - 1].t), 0, x(liveSamples[i].t) - x(liveSamples[i - 1].t) + 1, H);
  }
  // grid + zero line
  ctx.strokeStyle = '#2a2a4a';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, y(0)); ctx.lineTo(W, y(0)); ctx.stroke();

  const line = (key, color) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    liveSamples.forEach((s, i) => (i ? ctx.lineTo : ctx.moveTo).call(ctx, x(s.t), y(s[key])));
    ctx.stroke();
  };
  const css = getComputedStyle(document.documentElement);
  line('goal', css.getPropertyValue('--accent').trim() || '#4f8ef7');
  line('pos', css.getPropertyValue('--success').trim() || '#3ecf8e');

  ctx.fillStyle = '#8888aa';
  ctx.font = '11px sans-serif';
  ctx.fillText(`${(hi * 180 / Math.PI).toFixed(0)}°`, 4, 12);
  ctx.fillText(`${(lo * 180 / Math.PI).toFixed(0)}°`, 4, H - 4);
}

// ── wiring ────────────────────────────────────────────────────────────────────

export function initSysid() {
  if (!$('tab-sysid')) return;

  // ABORT goes over REST and never throws: like the e-stop, an abort must not fail.
  // Also stops any auto-record sequence in progress -- an abort mid-sequence must not
  // silently continue on to the next cell.
  $('sid-abort').addEventListener('click', stopAuto);
  $('sid-auto-stop').addEventListener('click', stopAuto);
  $('sid-auto-start').addEventListener('click', () => {
    if (!autoRunning) runAutoSequence();
  });
  $('sid-start-session').addEventListener('click', () => api('session', {
    name: $('sid-session-name').value.trim(), servo_id: parseInt($('sid-servo-id').value, 10),
  }).then(refresh).catch(() => {}));
  $('sid-zero').addEventListener('click', () => {
    $('sid-zero-meta').textContent = 'Capturing… (torque off, ~2 s)';
    api('zero', {}).then(refresh).catch(() => {});
  });
  $('sid-run').addEventListener('click', () => api('run', {
    trajectory: $('sid-traj').value, kp: parseInt($('sid-kp').value, 10),
    hole: parseInt($('sid-hole').value, 10), force_robot_servo: $('sid-force').checked,
  }).then(() => { startLive(); refresh(); }).catch(() => {}));
  $('sid-screen').addEventListener('click', () => {
    if (!confirm('Robot suspended with every leg free to move? Each joint will sweep in turn.')) return;
    api('screen', {
      amplitude_deg: parseFloat($('sid-scr-amp').value), period_s: parseFloat($('sid-scr-period').value),
      cycles: parseInt($('sid-scr-cycles').value, 10),
    }).then(() => { startLive(); refresh(); }).catch(() => {});
  });

  refresh();
  setInterval(refresh, 1000);
}
