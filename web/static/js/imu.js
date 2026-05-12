/**
 * imu.js — IMU readouts and artificial horizon.
 */

const $ = id => document.getElementById(id);

// ── Calibrate button ─────────────────────────────────────────────────────────
document.getElementById('btn-calibrate').addEventListener('click', () => {
  fetch('/api/imu/calibrate', { method: 'POST' })
    .then(r => r.json())
    .then(r => console.log('Calibrate:', r.message));
});

// ── Telemetry update ─────────────────────────────────────────────────────────
export function onIMUTelemetry(imu) {
  _set('imu-roll',  imu.euler_deg.roll.toFixed(2));
  _set('imu-pitch', imu.euler_deg.pitch.toFixed(2));
  _set('imu-yaw',   imu.euler_deg.yaw.toFixed(2));

  _set('imu-qw', imu.quaternion.w.toFixed(4));
  _set('imu-qx', imu.quaternion.x.toFixed(4));
  _set('imu-qy', imu.quaternion.y.toFixed(4));
  _set('imu-qz', imu.quaternion.z.toFixed(4));

  _set('imu-ax', imu.accel.x.toFixed(3));
  _set('imu-ay', imu.accel.y.toFixed(3));
  _set('imu-az', imu.accel.z.toFixed(3));

  _set('imu-gx', imu.gyro.x.toFixed(4));
  _set('imu-gy', imu.gyro.y.toFixed(4));
  _set('imu-gz', imu.gyro.z.toFixed(4));

  _updateHorizon(imu.euler_deg.roll, imu.euler_deg.pitch);
  _updateCalDots(imu.calibration);
}

// ── Artificial horizon ───────────────────────────────────────────────────────
const horizon = $('horizon');
const horizonSky = $('horizon-sky');
const horizonGnd = $('horizon-gnd');

function _updateHorizon(roll, pitch) {
  if (!horizon) return;
  // Rotate the SVG group by roll; shift sky/ground by pitch
  const pitchPx = pitch * (58 / 90);  // map ±90° to ±58px
  horizon.style.transform = `rotate(${roll}deg)`;
  const cy = -pitchPx;
  horizonSky.setAttribute('y', String(-60 + cy));
  horizonGnd.setAttribute('y', String(cy));
}

// ── Calibration dots ─────────────────────────────────────────────────────────
function _updateCalDots(level) {
  const dots = document.querySelectorAll('#cal-dots .dot');
  dots.forEach(d => {
    d.className = `dot${level >= 1 ? ' cal-1' : ''}${level >= 2 ? ' cal-2' : ''}${level >= 3 ? ' cal-3' : ''}`;
  });
}

// ── Util ─────────────────────────────────────────────────────────────────────
function _set(id, val) {
  const el = $(id);
  if (el && el.textContent !== val) el.textContent = val;
}
