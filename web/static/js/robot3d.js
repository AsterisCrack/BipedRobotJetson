/**
 * robot3d.js — Three.js URDF viewer, IK sliders, FK sliders, and pose buttons.
 *
 * Three.js and OrbitControls are resolved via the importmap in index.html.
 * urdf-loader is fetched directly from jsDelivr; it imports 'three' as a bare
 * specifier which the importmap resolves correctly.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import URDFLoader from 'https://cdn.jsdelivr.net/npm/urdf-loader@0.12.0/src/URDFLoader.js';

import { send } from './app.js';

const LEFT_JOINTS  = ['l_hip_yaw','l_hip_roll_joint','l_hip_pitch_joint','l_knee_joint','l_ankle_roll_joint','l_ankle_pitch_joint'];
const RIGHT_JOINTS = ['r_hip_yaw','r_hip_roll_joint','r_hip_pitch_joint','r_knee_joint','r_ankle_roll_joint','r_ankle_pitch_joint'];

const IK_RANGES   = { x: [-0.06, 0.06], y: [-0.07, 0.07], z: [-0.32, -0.18] };
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
  left:  { ...IK_DEFAULTS.left },
  right: { ...IK_DEFAULTS.right },
};

// ── Public API ────────────────────────────────────────────────────────────────

export function initRobot3D() {
  _buildPoseButtons();
  _buildIKSliders('left',  document.getElementById('ik-left'));
  _buildIKSliders('right', document.getElementById('ik-right'));
  _buildFKSliders('left',  document.getElementById('fk-left'));
  _buildFKSliders('right', document.getElementById('fk-right'));
  _initScene();
}

export function onRobotTelemetry(msg) {
  if (!robot3d) return;
  msg.servos.forEach(s => {
    const joint = robot3d.joints?.[s.joint];
    if (joint) joint.setJointValue(s.position_deg * Math.PI / 180);
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

  // Load URDF
  const loader = new URDFLoader();
  loader.packages = { RobotDescription: '/robot_description' };
  loader.load('/robot_description/urdf/robot_flat.urdf', obj => {
    robot3d = obj;
    // URDF uses Z-up; Three.js is Y-up
    robot3d.rotation.x = -Math.PI / 2;
    scene.add(robot3d);
  });

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
  );
  container.appendChild(homeBtn);
}

// ── IK sliders ────────────────────────────────────────────────────────────────

function _buildIKSliders(leg, container) {
  const def = IK_DEFAULTS[leg];
  ['x', 'y', 'z'].forEach(axis => {
    const [min, max] = IK_RANGES[axis];
    const row = _sliderRow(axis.toUpperCase(), min, max, def[axis], 0.001, () => _sendIK(leg));
    row.dataset.axis = axis;
    container.appendChild(row);
  });
}

function _sendIK(leg) {
  const container = document.getElementById(`ik-${leg}`);
  ['x', 'y', 'z'].forEach(axis => {
    const row = container.querySelector(`[data-axis="${axis}"]`);
    if (row) _ikValues[leg][axis] = parseFloat(row.querySelector('input').value);
  });
  const { x, y, z } = _ikValues[leg];
  send({ type: 'set_foot_ik', leg, x, y, z });
}

// ── FK sliders ────────────────────────────────────────────────────────────────

function _buildFKSliders(leg, container) {
  const joints = leg === 'left' ? LEFT_JOINTS : RIGHT_JOINTS;
  joints.forEach(name => {
    const [min, max] = JOINT_LIMITS[name] || [-180, 180];
    const label = name.replace(/^[lr]_/, '').replace(/_joint$/, '').replace(/_/g, ' ');
    const row = _sliderRow(label, min, max, 0, 0.5, () => _sendFK(leg));
    row.dataset.joint = name;
    container.appendChild(row);
  });
}

function _sendFK(leg) {
  const container = document.getElementById(`fk-${leg}`);
  const joints = leg === 'left' ? LEFT_JOINTS : RIGHT_JOINTS;
  const angles = joints.map(name => {
    const row = container.querySelector(`[data-joint="${name}"]`);
    return row ? parseFloat(row.querySelector('input').value) : 0;
  });
  send({ type: 'set_joints_fk', leg, angles_deg: angles });
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
