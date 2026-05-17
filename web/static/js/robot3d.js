/**
 * robot3d.js — Three.js URDF viewer, IK sliders, merged FK sliders,
 * servo config panel (direction sign + set-default), and pose buttons.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'https://cdn.jsdelivr.net/npm/urdf-loader@0.12.0/src/URDFLoader.js';

import { send } from './app.js';

const LEFT_JOINTS  = ['l_hip_yaw','l_hip_roll_joint','l_hip_pitch_joint','l_knee_joint','l_ankle_roll_joint','l_ankle_pitch_joint'];
const RIGHT_JOINTS = ['r_hip_yaw','r_hip_roll_joint','r_hip_pitch_joint','r_knee_joint','r_ankle_roll_joint','r_ankle_pitch_joint'];

const IK_RANGE    = [-0.3, 0.3];   // all position axes, metres
const IK_DEFAULTS = { left: { x: 0, y: 0.021, z: -0.28 }, right: { x: 0, y: -0.021, z: -0.28 } };
const JOINT_LIMITS = {
  l_hip_yaw:          [-45,  45],  r_hip_yaw:          [-45,  45],
  l_hip_roll_joint:   [-25,  90],  r_hip_roll_joint:   [-25,  90],
  l_hip_pitch_joint:  [-90,  90],  r_hip_pitch_joint:  [-90,  90],
  l_knee_joint:       [-120,120],  r_knee_joint:       [-120,120],
  l_ankle_roll_joint: [-80,  80],  r_ankle_roll_joint: [-80,  80],
  l_ankle_pitch_joint:[-35,  85],  r_ankle_pitch_joint:[-35,  85],
};

let robot3d = null;

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

  const loader = new URDFLoader();
  loader.packages = { RobotDescription: '/robot_description' };
  loader.load('/robot_description/urdf/robot_flat.urdf', obj => {
    robot3d = obj;
    robot3d.rotation.x = -Math.PI / 2;
    scene.add(robot3d);
    // Axes in base_link (URDF) frame: X=red, Y=green, Z=blue.
    // Added as a child so the -π/2 rotation carries the axes into world space
    // with the correct URDF orientation.
    robot3d.add(new THREE.AxesHelper(0.12));
  });

  // Legend overlay — explains axis colours in the viewer corner.
  const legend = document.createElement('div');
  legend.className = 'axes-legend';
  legend.innerHTML =
    '<span class="ax-x">X</span> forward &nbsp;' +
    '<span class="ax-y">Y</span> lateral &nbsp;' +
    '<span class="ax-z">Z</span> up';
  container.appendChild(legend);

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
