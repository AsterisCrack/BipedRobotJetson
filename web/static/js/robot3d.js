/**
 * robot3d.js — Three.js URDF viewer, IK sliders, merged FK sliders,
 * servo config panel (direction sign + set-default), and pose buttons.
 *
 * The viewer doubles as the sign-convention oracle for the RL policy: the model
 * is the real URDF parsed by urdf-loader (so every joint <axis> is applied as
 * declared, not hand-tuned to look right) and it is driven by `logical_deg`,
 * which is the same quantity Isaac trains on. If the model and the physical
 * robot disagree about which way a joint moved, the servo's direction_sign is
 * wrong relative to what the policy expects.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'https://cdn.jsdelivr.net/npm/urdf-loader@0.12.0/src/URDFLoader.js';

import { send } from './app.js';

const LEFT_JOINTS  = ['l_hip_yaw','l_hip_roll_joint','l_hip_pitch_joint','l_knee_joint','l_ankle_roll_joint','l_ankle_pitch_joint'];
const RIGHT_JOINTS = ['r_hip_yaw','r_hip_roll_joint','r_hip_pitch_joint','r_knee_joint','r_ankle_roll_joint','r_ankle_pitch_joint'];

const IK_RANGE    = [-0.3, 0.3];   // all position axes, metres
const IK_DEFAULTS = { left: { x: 0, y: 0.021, z: -0.28 }, right: { x: 0, y: -0.021, z: -0.28 } };
// Slider ranges, in LOGICAL degrees. These mirror _JOINT_LIMITS_DEG in the
// training repo (envs/assets/robotV2/biped_robot.py) — i.e. the exact range the
// policy's action maps onto — so dragging a slider covers the same span the
// policy can command and nothing more. The URDF's own <limit> tags are symmetric
// placeholders (knee reads ±120°) and do NOT describe the real mechanism, so
// they are deliberately not used here.
const JOINT_LIMITS = {
  l_hip_yaw:          [-45,  45],  r_hip_yaw:          [-45,  45],
  l_hip_roll_joint:   [-25,  45],  r_hip_roll_joint:   [-25,  45],
  l_hip_pitch_joint:  [-90,  30],  r_hip_pitch_joint:  [-90,  30],
  l_knee_joint:       [-120,  5],  r_knee_joint:       [-120,  5],
  l_ankle_roll_joint: [-60,  60],  r_ankle_roll_joint: [-60,  60],
  l_ankle_pitch_joint:[-35,  50],  r_ankle_pitch_joint:[-35,  50],
};

// ── IMU visualisation ─────────────────────────────────────────────────────────

// Arrow length per unit of sensor reading, and the cap so a hard knock doesn't
// draw an arrow off-screen. The robot is ~0.3 m tall, so 10 m/s² -> 0.15 m puts
// a decent shove at about half body height.
const ACCEL_SCALE    = 0.015;   // m per m/s²
const GYRO_SCALE     = 0.030;   // m per rad/s
const ARROW_MAX      = 0.30;    // m
const ACCEL_DEADZONE = 0.15;    // m/s² — below this the arrow hides instead of jittering
const GYRO_DEADZONE  = 0.08;    // rad/s

const AXIS_COLORS  = [0xff4444, 0x44cc44, 0x4488ff];   // X red, Y green, Z blue (matches legend)
const AXIS_VECTORS = [
  new THREE.Vector3(1, 0, 0),
  new THREE.Vector3(0, 1, 0),
  new THREE.Vector3(0, 0, 1),
];

// Arrow triad origins, in URDF body frame (Z up). Stacked above base_link so
// they read as "on top of the robot" and don't tangle with the AxesHelper.
const ACCEL_ORIGIN_Z = 0.08;
const GYRO_ORIGIN_Z  = 0.15;

let robot3d    = null;   // the URDF model
let bodyFrame  = null;   // Group carrying the IMU orientation; children are body-frame
let hudEl      = null;
let accelArrows = [];
let gyroArrows  = [];

const _opts = { follow: true, accel: true, gyro: false };

const _ikValues = {
  left:  { ...IK_DEFAULTS.left,  roll: 0, pitch: 0, yaw: 0 },
  right: { ...IK_DEFAULTS.right, roll: 0, pitch: 0, yaw: 0 },
};

// joint_name → { servo_id, direction_sign, default_position_deg, ... }
const _servoConfig = new Map();

// joint_name → current position_deg (URDF) from latest telemetry frame
const _livePositions = new Map();

// ── Public API ────────────────────────────────────────────────────────────────

export function initRobot3D() {
  _buildPoseButtons();

  const ikLeft  = document.getElementById('ik-left');
  const ikRight = document.getElementById('ik-right');
  const fkAll   = document.getElementById('fk-all');
  const cfgPanel = document.getElementById('servo-config-panel');
  const printBtn = document.getElementById('btn-print-config');

  if (ikLeft)  _buildIKSliders('left',  ikLeft);
  if (ikRight) _buildIKSliders('right', ikRight);
  if (fkAll)   _buildFKAll(fkAll);
  _syncIKSlidersToFK();
  if (cfgPanel || fkAll) _loadServoConfigs();
  _wireViewerOptions();
  _initScene();

  if (printBtn) {
    printBtn.addEventListener('click', () => {
      const statusEl = document.getElementById('config-export-status');
      if (statusEl) statusEl.textContent = 'Exporting…';
      fetch('/api/config/export', { method: 'POST' })
        .then(r => r.json())
        .then(d => { if (statusEl) statusEl.textContent = `✓ ${d.path}`; })
        .catch(() => { if (statusEl) statusEl.textContent = 'Export failed'; });
    });
  }
}

export function onRobotTelemetry(msg) {
  msg.servos.forEach(s => {
    // Update 3D model
    if (robot3d) {
      const joint = robot3d.joints?.[s.joint];
      if (joint) joint.setJointValue(s.logical_deg * Math.PI / 180);
    }
    // Update live config panel display
    _livePositions.set(s.joint, s.position_deg);
    const liveEl = document.querySelector(`[data-live-for="${s.joint}"]`);
    if (liveEl) liveEl.textContent = s.position_deg.toFixed(1) + '°';
  });

  if (msg.imu) _updateIMUVisuals(msg.imu);
}

// ── IMU orientation + acceleration/gyro arrows ───────────────────────────────

/**
 * Convert the RAW BNO055 quaternion into a body-frame orientation.
 *
 * The board is mounted rotated 180° about the body X axis (see
 * hardware/imu/bno055.py), so body->sensor is R_mount = diag(1,-1,-1) and
 *   R_body->world = R_sensor->world · R_mount
 * i.e. q_body = q_raw ⊗ q_mount with q_mount = (w=0, x=1, y=0, z=0).
 * Expanding that product collapses to a relabelling:
 *   (w', x', y', z') = (-x, w, z, -y)
 *
 * This is the composition hardware/imu/bno055.py declines to do at the source
 * because it was never verified. It is verified now, algebraically: feeding q'
 * through the same g_body formula ServoBusManager.get_rl_state() uses reproduces
 * that function's output exactly, including the explicit "negate Y,Z" it applies
 * afterwards. So this rotation and the gravity vector the policy consumes agree
 * by construction — the HUD's `grav` row is a live check of that.
 *
 * THREE.Quaternion's constructor takes (x, y, z, w).
 */
function _mountCorrectedQuat(q) {
  return new THREE.Quaternion(q.w, q.z, -q.y, -q.x);
}

function _makeArrowTriad(parent, originZ) {
  return AXIS_VECTORS.map((axis, i) => {
    const arrow = new THREE.ArrowHelper(
      axis, new THREE.Vector3(0, 0, originZ), 0.05, AXIS_COLORS[i],
    );
    arrow.visible = false;
    parent.add(arrow);
    return arrow;
  });
}

// Each arrow points along +axis for a positive reading and flips for a negative
// one, so the sign is readable at a glance rather than only the magnitude.
function _updateTriad(arrows, vec, scale, deadzone, enabled) {
  arrows.forEach((arrow, i) => {
    const v = vec[i];
    if (!enabled || !Number.isFinite(v) || Math.abs(v) < deadzone) {
      arrow.visible = false;
      return;
    }
    arrow.visible = true;
    arrow.setDirection(AXIS_VECTORS[i].clone().multiplyScalar(Math.sign(v)));
    const len = Math.min(Math.abs(v) * scale, ARROW_MAX);
    arrow.setLength(len, len * 0.28, len * 0.16);
  });
}

function _updateIMUVisuals(imu) {
  if (!bodyFrame) return;

  const q = _mountCorrectedQuat(imu.quaternion);
  if (_opts.follow) bodyFrame.quaternion.copy(q);
  else              bodyFrame.quaternion.identity();

  const a = [imu.accel.x, imu.accel.y, imu.accel.z];
  const g = [imu.gyro.x,  imu.gyro.y,  imu.gyro.z];
  _updateTriad(accelArrows, a, ACCEL_SCALE, ACCEL_DEADZONE, _opts.accel);
  _updateTriad(gyroArrows,  g, GYRO_SCALE,  GYRO_DEADZONE,  _opts.gyro);

  if (!hudEl) return;
  // World-down expressed in the body frame — the same `projected_gravity` the
  // policy sees. Upright reads [0, 0, -1].
  const pg = new THREE.Vector3(0, 0, -1).applyQuaternion(q.clone().invert());
  const tilt = Math.acos(Math.max(-1, Math.min(1, -pg.z))) * 180 / Math.PI;
  hudEl.innerHTML =
    _hudRow('grav', [pg.x, pg.y, pg.z], '') +
    `<div class="hud-tilt${tilt > 45 ? ' hud-warn' : ''}">tilt ${tilt.toFixed(1)}°</div>` +
    _hudRow('acc', a, 'm/s²') +
    _hudRow('gyr', g, 'rad/s');
}

function _hudRow(label, v, unit) {
  const c = ['ax-x', 'ax-y', 'ax-z'];
  const cells = v.map((n, i) =>
    `<span class="${c[i]}">${(n >= 0 ? '+' : '') + n.toFixed(2)}</span>`).join(' ');
  return `<div><span class="hud-dim">${label}</span> ${cells} <span class="hud-dim">${unit}</span></div>`;
}

function _wireViewerOptions() {
  [['opt-imu-follow', 'follow'],
   ['opt-accel-arrows', 'accel'],
   ['opt-gyro-arrows', 'gyro']].forEach(([id, key]) => {
    const el = document.getElementById(id);
    if (!el) return;
    el.checked = _opts[key];
    el.addEventListener('change', () => { _opts[key] = el.checked; });
  });
}

// ── Three.js scene ────────────────────────────────────────────────────────────

function _initScene() {
  const container = document.getElementById('viewer-container');
  if (!container) return;

  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0d0d1a);
  scene.add(new THREE.AmbientLight(0xffffff, 0.6));
  const dirLight = new THREE.DirectionalLight(0xffffff, 1.0);
  dirLight.position.set(0.5, 1, 0.8);
  scene.add(dirLight);

  const grid = new THREE.GridHelper(0.8, 8, 0x2a2a4a, 0x1c1c3a);
  grid.position.y = -0.29;
  scene.add(grid);

  const w = container.clientWidth || 600;
  const h = container.clientHeight || 500;

  const camera = new THREE.PerspectiveCamera(50, w / h, 0.001, 10);
  camera.position.set(0.4, 0.2, 0.5);
  camera.lookAt(0, 0, 0);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.setSize(w, h);
  container.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, -0.1, 0);
  controls.update();

  // Frame chain:
  //   scene                          Three.js world, Y up
  //   └─ urdfFrame  (rot.x = -90°)   URDF world, Z up
  //      └─ bodyFrame (IMU quat)     robot body frame
  //         └─ robot3d, axes, arrows
  //
  // The IMU quaternion is expressed in the Z-up URDF convention, so it has to be
  // applied *inside* urdfFrame — hanging it off the scene root would mix the two
  // conventions and tilt the robot about the wrong axis.
  const urdfFrame = new THREE.Group();
  urdfFrame.rotation.x = -Math.PI / 2;
  scene.add(urdfFrame);

  bodyFrame = new THREE.Group();
  urdfFrame.add(bodyFrame);

  // Axes in base_link (URDF) frame: X=red, Y=green, Z=blue.
  bodyFrame.add(new THREE.AxesHelper(0.12));
  accelArrows = _makeArrowTriad(bodyFrame, ACCEL_ORIGIN_Z);
  gyroArrows  = _makeArrowTriad(bodyFrame, GYRO_ORIGIN_Z);

  const loader = new URDFLoader();
  loader.packages = { RobotDescription: '/robot_description' };
  loader.load('/robot_description/urdf/robot_flat.urdf', obj => {
    robot3d = obj;
    bodyFrame.add(robot3d);
  });

  // Legend overlay — explains axis colours in the viewer corner.
  const legend = document.createElement('div');
  legend.className = 'axes-legend';
  legend.innerHTML =
    '<span class="ax-x">X</span> forward &nbsp;' +
    '<span class="ax-y">Y</span> lateral &nbsp;' +
    '<span class="ax-z">Z</span> up<br>' +
    '<span class="hud-dim">lower triad = accel · upper = gyro</span>';
  container.appendChild(legend);

  hudEl = document.createElement('div');
  hudEl.className = 'viewer-hud';
  container.appendChild(hudEl);

  new ResizeObserver(() => {
    const nw = container.clientWidth, nh = container.clientHeight;
    camera.aspect = nw / nh;
    camera.updateProjectionMatrix();
    renderer.setSize(nw, nh);
  }).observe(container);

  (function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
  })();
}

// ── Pose buttons ──────────────────────────────────────────────────────────────

function _buildPoseButtons() {
  const container = document.getElementById('pose-buttons');
  fetch('/api/kinematics/poses')
    .then(r => r.json())
    .then(poses => {
      poses.forEach(name => {
        const btn = document.createElement('button');
        btn.textContent = name.charAt(0).toUpperCase() + name.slice(1);
        btn.addEventListener('click', () =>
          fetch(`/api/kinematics/poses/${name}`, { method: 'POST' })
            .then(() => _syncSlidersFromPose(name))
        );
        container.appendChild(btn);
      });
    })
    .catch(() => {});

  const homeBtn = document.createElement('button');
  homeBtn.textContent = 'Home';
  homeBtn.className = 'primary';
  homeBtn.addEventListener('click', () =>
    fetch('/api/kinematics/home', { method: 'POST' })
      .then(r => r.json())
      .then(d => _syncSlidersFromPose(d.pose))
  );
  container.appendChild(homeBtn);
}

// Fetch pose joint angles and sync both FK and IK sliders.
function _syncSlidersFromPose(poseName) {
  fetch(`/api/kinematics/poses/${poseName}`)
    .then(r => r.json())
    .then(poseAngles => {
      ['left', 'right'].forEach(leg => {
        const joints = leg === 'left' ? LEFT_JOINTS : RIGHT_JOINTS;
        const angles  = joints.map(j => poseAngles[j] ?? 0);
        _setFKSliders(leg, angles);
        // set_joints_fk writes servos (idempotent — same position) and
        // returns fk_result, which onFKResult uses to update IK sliders.
        send({ type: 'set_joints_fk', leg, angles_deg: angles });
      });
    })
    .catch(() => {});
}

// Set FK slider values for one leg without triggering a servo command.
function _setFKSliders(leg, logicalAngles) {
  const fkContainer = document.getElementById('fk-all');
  if (!fkContainer) return;
  const joints = leg === 'left' ? LEFT_JOINTS : RIGHT_JOINTS;
  joints.forEach((name, i) => {
    const row = fkContainer.querySelector(`[data-joint="${name}"]`);
    if (!row) return;
    const v = logicalAngles[i] ?? 0;
    const input = row.querySelector('input');
    const sv    = row.querySelector('.sv');
    if (input) input.value = v;
    if (sv)    sv.textContent = parseFloat(v).toFixed(1);
  });
}

// ── IK sliders ────────────────────────────────────────────────────────────────

function _buildIKSliders(leg, container) {
  const def = IK_DEFAULTS[leg];

  const posLbl = document.createElement('div');
  posLbl.className = 'fk-leg-label';
  posLbl.textContent = 'Position (m)';
  container.appendChild(posLbl);

  ['x', 'y', 'z'].forEach(axis => {
    const row = _sliderRow(axis.toUpperCase(), IK_RANGE[0], IK_RANGE[1], def[axis], 0.001, () => _sendIK(leg));
    row.dataset.axis = axis;
    container.appendChild(row);
  });

  const rotLbl = document.createElement('div');
  rotLbl.className = 'fk-leg-label';
  rotLbl.textContent = 'Orientation (°)';
  container.appendChild(rotLbl);

  ['roll', 'pitch', 'yaw'].forEach(rot => {
    const label = rot.charAt(0).toUpperCase() + rot.slice(1);
    const row = _sliderRow(label, -180, 180, 0, 1, () => _sendIK(leg));
    row.dataset.rot = rot;
    container.appendChild(row);
  });
}

// ── RPY ↔ rotation-matrix helpers ────────────────────────────────────────────

function _matrixToRPY(R) {
  // R is a 3×3 list-of-lists; extrinsic XYZ convention (= Rz @ Ry @ Rx).
  const sp    = -R[2][0];
  const pitch = Math.asin(Math.max(-1, Math.min(1, sp)));
  const cp    = Math.cos(pitch);
  let roll, yaw;
  if (cp > 1e-6) {
    roll = Math.atan2(R[2][1], R[2][2]);
    yaw  = Math.atan2(R[1][0], R[0][0]);
  } else {
    // Gimbal lock (pitch ≈ ±90°)
    roll = Math.atan2(-R[0][1], R[1][1]);
    yaw  = 0;
  }
  const d = 180 / Math.PI;
  return { roll: roll * d, pitch: pitch * d, yaw: yaw * d };
}

function _setIKSliders(leg, x, y, z, roll, pitch, yaw) {
  _ikValues[leg] = { x, y, z, roll, pitch, yaw };
  const container = document.getElementById(`ik-${leg}`);
  if (!container) return;
  [['x', x, 3], ['y', y, 3], ['z', z, 3]].forEach(([axis, val, dp]) => {
    const row = container.querySelector(`[data-axis="${axis}"]`);
    if (!row) return;
    const input = row.querySelector('input');
    const sv    = row.querySelector('.sv');
    if (input) input.value = val;
    if (sv)    sv.textContent = val.toFixed(dp);
  });
  [['roll', roll], ['pitch', pitch], ['yaw', yaw]].forEach(([rot, val]) => {
    const row = container.querySelector(`[data-rot="${rot}"]`);
    if (!row) return;
    const input = row.querySelector('input');
    const sv    = row.querySelector('.sv');
    if (input) input.value = val;
    if (sv)    sv.textContent = val.toFixed(1);
  });
}

function _syncIKSlidersToFK() {
  ['left', 'right'].forEach(leg => {
    fetch(`/api/kinematics/fk/${leg}`)
      .then(r => r.json())
      .then(data => {
        const [x, y, z] = data.position;
        const { roll, pitch, yaw } = _matrixToRPY(data.rotation_matrix);
        _setIKSliders(leg, x, y, z, roll, pitch, yaw);
      })
      .catch(() => {});
  });
}

// Called by app.js whenever a set_joints_fk response arrives.
export function onFKResult(msg) {
  const [x, y, z] = msg.position;
  const { roll, pitch, yaw } = _matrixToRPY(msg.rotation_matrix);
  _setIKSliders(msg.leg, x, y, z, roll, pitch, yaw);
}

function _sendIK(leg) {
  const container = document.getElementById(`ik-${leg}`);
  ['x', 'y', 'z'].forEach(axis => {
    const row = container.querySelector(`[data-axis="${axis}"]`);
    if (row) _ikValues[leg][axis] = parseFloat(row.querySelector('input').value);
  });
  ['roll', 'pitch', 'yaw'].forEach(rot => {
    const row = container.querySelector(`[data-rot="${rot}"]`);
    if (row) _ikValues[leg][rot] = parseFloat(row.querySelector('input').value);
  });
  const { x, y, z, roll, pitch, yaw } = _ikValues[leg];
  send({ type: 'set_foot_ik', leg, x, y, z, roll, pitch, yaw });
}

// ── FK sliders (both legs merged) ─────────────────────────────────────────────

function _buildFKAll(container) {
  ['left', 'right'].forEach(leg => {
    const divider = document.createElement('div');
    divider.className = 'fk-leg-label';
    divider.textContent = leg === 'left' ? 'Left Leg' : 'Right Leg';
    container.appendChild(divider);

    const joints = leg === 'left' ? LEFT_JOINTS : RIGHT_JOINTS;
    joints.forEach(name => {
      const [min, max] = JOINT_LIMITS[name] || [-180, 180];
      const label = (leg === 'left' ? 'L ' : 'R ') +
        name.replace(/^[lr]_/, '').replace(/_joint$/, '').replace(/_/g, ' ');
      const row = _sliderRow(label, min, max, 0, 0.5, () => _sendFKLeg(leg));
      row.dataset.joint = name;
      row.dataset.leg = leg;
      container.appendChild(row);
    });
  });
}

function _sendFKLeg(leg) {
  const container = document.getElementById('fk-all');
  const joints = leg === 'left' ? LEFT_JOINTS : RIGHT_JOINTS;
  const angles = joints.map(name => {
    const row = container.querySelector(`[data-joint="${name}"]`);
    return row ? parseFloat(row.querySelector('input').value) : 0;
  });
  send({ type: 'set_joints_fk', leg, angles_deg: angles });
}

// ── Servo config panel ────────────────────────────────────────────────────────

function _loadServoConfigs() {
  fetch('/api/config/servos')
    .then(r => r.json())
    .then(configs => {
      configs.forEach(c => _servoConfig.set(c.joint_name, { ...c }));
      _buildConfigPanel(document.getElementById('servo-config-panel'));
    })
    .catch(() => {});
}

function _buildConfigPanel(container) {
  if (!container) return;
  container.innerHTML = '';

  // Ordered: kinematic joints first (L then R), then any extras
  const allKnown = [...LEFT_JOINTS, ...RIGHT_JOINTS];
  const ordered = [
    ...allKnown.filter(j => _servoConfig.has(j)),
    ...[..._servoConfig.keys()].filter(j => !allKnown.includes(j)),
  ];

  ordered.forEach(jointName => {
    const cfg = _servoConfig.get(jointName);
    if (!cfg) return;

    const row = document.createElement('div');
    row.className = 'cfg-row';
    row.dataset.joint = jointName;

    const label = document.createElement('span');
    label.className = 'cfg-joint';
    label.textContent = _cfgLabel(jointName);

    const dirBtn = document.createElement('button');
    dirBtn.className = 'dir-btn ' + (cfg.direction_sign > 0 ? 'positive' : 'negative');
    dirBtn.textContent = cfg.direction_sign > 0 ? '+1' : '−1';
    dirBtn.title = 'Toggle direction sign (±1)';
    dirBtn.addEventListener('click', () => _toggleDir(jointName, dirBtn));

    const liveSpan = document.createElement('span');
    liveSpan.className = 'cfg-default';
    liveSpan.dataset.liveFor = jointName;
    liveSpan.title = 'Current live position_deg (URDF°) — click Set Here to make this the new default';
    liveSpan.textContent = (_livePositions.get(jointName) ?? cfg.default_position_deg).toFixed(1) + '°';

    const setBtn = document.createElement('button');
    setBtn.className = 'set-default-btn';
    setBtn.textContent = 'Set Here';
    setBtn.title = 'Use current physical position as the new default (logical zero)';
    setBtn.addEventListener('click', () => _setDefaultPos(jointName));

    row.append(label, dirBtn, liveSpan, setBtn);
    container.appendChild(row);
  });
}

function _toggleDir(jointName, btn) {
  const cfg = _servoConfig.get(jointName);
  if (!cfg) return;
  const newDir = -cfg.direction_sign;
  fetch(`/api/servos/${cfg.servo_id}/direction_sign`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ direction_sign: newDir }),
  })
    .then(r => r.json())
    .then(() => {
      cfg.direction_sign = newDir;
      btn.textContent = newDir > 0 ? '+1' : '−1';
      btn.className = 'dir-btn ' + (newDir > 0 ? 'positive' : 'negative');
    });
}

function _setDefaultPos(jointName) {
  const cfg = _servoConfig.get(jointName);
  if (!cfg) return;

  // Use the live physical position_deg (URDF) as the new default.
  // Fall back to the configured default if telemetry hasn't arrived yet.
  const newDefault = _livePositions.has(jointName)
    ? _livePositions.get(jointName)
    : cfg.default_position_deg;

  fetch(`/api/servos/${cfg.servo_id}/default_position`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ default_position_deg: newDefault }),
  })
    .then(r => r.json())
    .then(() => {
      cfg.default_position_deg = newDefault;
      // Reset FK slider to 0 — logical angle is now 0 at this position
      const fkRow = document.getElementById('fk-all')
        ?.querySelector(`[data-joint="${jointName}"]`);
      if (fkRow) {
        fkRow.querySelector('input').value = 0;
        fkRow.querySelector('.sv').textContent = '0.0';
      }
    });
}

function _cfgLabel(jointName) {
  const prefix = jointName.startsWith('l_') ? 'L ' : (jointName.startsWith('r_') ? 'R ' : '');
  return prefix + jointName.replace(/^[lr]_/, '').replace(/_joint$/, '').replace(/_/g, ' ');
}

// ── Utility ───────────────────────────────────────────────────────────────────

function _sliderRow(label, min, max, value, step, onChange) {
  const row = document.createElement('div');
  row.className = 'slider-row';

  const lbl = document.createElement('label');
  lbl.textContent = label;

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = min; slider.max = max; slider.step = step; slider.value = value;

  const val = document.createElement('span');
  val.className = 'sv';
  val.textContent = parseFloat(value).toFixed(step < 1 ? 3 : 1);

  let tid;
  slider.addEventListener('input', () => {
    val.textContent = parseFloat(slider.value).toFixed(step < 1 ? 3 : 1);
    clearTimeout(tid);
    tid = setTimeout(() => onChange(parseFloat(slider.value)), 80);
  });

  row.appendChild(lbl);
  row.appendChild(slider);
  row.appendChild(val);
  return row;
}
