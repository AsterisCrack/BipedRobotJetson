# BipedRobotJetson

Standalone Python control stack for a 12-DOF biped robot running on a **Jetson Orin Nano**. Features a single web-based control interface accessible from any browser on the local network — no ROS, no separate scripts per operation.

---

## Hardware

| Component | Details |
|-----------|---------|
| Computer | NVIDIA Jetson Orin Nano |
| Servos | 12× ST3215 smart servos (SCS/Feetech protocol, half-duplex UART) |
| IMU | BNO055 (I2C, register-mapped, addr 0x28) |
| Legs | 2× 6-DOF (hip yaw/roll/pitch · knee · ankle roll/pitch) |

---

## Features

- **Servo control** — real-time position, speed, load, voltage, temperature monitoring; per-servo position sliders; torque enable/disable; PID tuning; ID reassignment; zero-point calibration
- **IMU readouts** — quaternion, Euler angles (roll/pitch/yaw), accelerometer, gyroscope, calibration status, artificial horizon display
- **Forward kinematics** — computes foot position from current joint angles using transforms extracted directly from the URDF
- **Inverse kinematics** — numerical IK (DLS — Damped Least Squares) from a target foot position to joint angles, with warm-start from current pose
- **3D URDF viewer** — live Three.js visualisation of the robot model, updated from servo telemetry at 20 Hz
- **Pose system** — named poses defined in YAML; one-click execution
- **WebSocket telemetry** — unified 20 Hz frame to all connected browsers (servos + IMU + foot positions)
- **REST API** — every operation is also available as a plain HTTP endpoint
- **RL policy control** — load an exported ONNX policy and drive the robot from it at 50 Hz, with arm/start/stop lifecycle, tilt-fault detection, and operator-confirmed recovery

---

## Repository Structure

```
BipedRobotJetson/
├── .env                          # Hardware paths (overrides YAML)
├── config/
│   ├── hardware.yaml             # UART port, baud rate, I2C bus/address, timeouts
│   └── robot.yaml                # Servo IDs, zero offsets, PIDs, default poses
│
├── hardware/                     # Standalone servo bus + IMU library (no web/robot deps)
│   ├── README.md                 # Library documentation + usage examples
│   ├── config.py                 # PIDConfig, ServoConfig, HardwareConfig (Pydantic)
│   ├── serial_bus.py             # Thread-safe half-duplex UART (echo drain)
│   ├── servo_bus_manager.py      # 50 Hz SYNC_READ/WRITE bus thread
│   ├── st3215/
│   │   ├── registers.py          # Full ST3215 register + instruction map
│   │   ├── protocol.py           # SCS packet encode/decode, checksum, SYNC_WRITE
│   │   └── servo.py              # ST3215 driver: position, torque, PID, status
│   └── imu/
│       └── bno055.py             # BNO055 driver (quat → Euler, calibration)
│
├── kinematics/                   # Standalone FK/IK library (no web/robot/hardware deps)
│   ├── README.md                 # Library documentation + usage examples
│   ├── chain.py                  # URDF parser → kinematic chain, FK (Rodrigues)
│   └── solver.py                 # FK + numerical IK (DLS — Damped Least Squares)
│
├── policy/                       # ONNX policy inference (uses hardware/ and robot/)
│   ├── deploy_config.py          # Loads/validates an exported bundle
│   ├── observation.py            # Joint permutation, 50-dim frame, history ring, gait clock
│   ├── policy_runner.py          # ONNX session, EMA filter, arming ramp, per-cycle step()
│   ├── safety.py                 # Tilt fault, action-rate limit, observation sanity
│   ├── state_machine.py          # IDLE → ARMING → ARMED → RUNNING → FAULT
│   └── selftest.py               # Dry run with torque off — run before first arm
│
├── robot/                        # Robot orchestrator (uses hardware/ and kinematics/)
│   ├── config.py                 # RobotConfig + Settings — loads .env + both YAMLs
│   └── robot.py                  # Robot class: motion, IK, telemetry loop, policy lifecycle
│
├── web/                          # Debug web UI (uses robot/ only)
│   ├── app.py                    # FastAPI factory, lifespan, static mounts
│   ├── routers/
│   │   ├── servos.py             # REST /api/servos/*
│   │   ├── imu.py                # REST /api/imu/*
│   │   ├── kinematics.py         # REST /api/kinematics/*
│   │   ├── config.py             # REST /api/config/*
│   │   └── policy.py             # REST /api/policy/*
│   ├── websocket.py              # WS broadcaster + command dispatcher
│   └── static/
│       ├── index.html            # Single-page app (5 tabs)
│       ├── css/style.css
│       └── js/
│           ├── app.js            # Tab router, central WS manager
│           ├── servos.js         # Servo tab
│           ├── imu.js            # IMU tab
│           ├── robot3d.js        # Three.js viewer, IK/FK sliders, pose buttons
│           ├── debug.js          # Debug tab — single-servo raw control + telemetry graph
│           ├── id_manager.js     # Servo ID reassignment page
│           └── control.js        # Control tab — RL policy arm/drive/monitor
│
├── models/                       # Exported ONNX bundles: <name>/policy.onnx + deploy_config.json
│
├── tools/
│   ├── bus_profiler.py           # Bus timing profiler (real ServoBusManager harness)
│   └── return_delay.py           # Read/zero RETURN_DELAY register on all servos
│
├── RobotDescription/
│   ├── urdf/Robot.urdf           # Robot URDF (source)
│   └── meshes/*.stl              # Visual/collision meshes
├── main.py                       # Entry point
└── requirements.txt
```

### Dependency Layers

```
hardware/    ←  no project imports (standalone)
kinematics/  ←  no project imports (standalone)
robot/       ←  imports from hardware/ and kinematics/
policy/      ←  imports from hardware/ (via robot/) — same layer as kinematics/
web/         ←  imports from robot/ only (never hardware/, kinematics/, or policy/ directly)
```

`hardware/` and `kinematics/` can be copied into any future project and used independently.
`policy/` is optional — the server starts without `onnxruntime` installed, with the Control
tab reporting the feature as unavailable.

---

## Installation

### 1. Clone and install dependencies

```bash
git clone <repo-url>
cd BipedRobotJetson
pip3 install -r requirements.txt
```

### 2. Install hardware libraries (Jetson only)

The Adafruit CircuitPython libraries require the Blinka hardware abstraction layer:

```bash
pip3 install -r requirements-hardware.txt
```

> On a development machine without hardware, these can be skipped — the robot initialises in a degraded mode and the web UI still works.

### 3. Configure hardware paths

Copy the example env file and edit it to match your Jetson's device nodes:

```bash
cp .env.example .env
```

All variables are optional — uncomment only what you need to change from the defaults:

```env
# Hardware paths
UART_PORT=/dev/ttyTHS1   # or /dev/ttyUSB0 for USB-to-SCS adapter
I2C_BUS=7                # 40-pin header I2C on Jetson Orin Nano

# Bus tuning (see "Bus Performance" section below)
BIPED_RT_SCHEDULING=1    # SCHED_FIFO for the servo bus thread (reduces jitter)
BIPED_FAST_MODE=2        # 0=full status, 1=position only, 2=position+speed (RL policy)

# Debug
BIPED_DEBUG=1            # DEBUG-level logs for kinematics/ and robot/
```

Verify hardware is visible:

```bash
ls /dev/ttyTHS*          # hardware UART
sudo i2cdetect -y 7      # BNO055 should appear at 0x28
```

### 4. Run

```bash
python3 main.py
```

Open `http://<jetson-ip>:8080` in any browser on the same network.

---

## Configuration

### `config/hardware.yaml`

```yaml
uart_port: /dev/ttyTHS1
baud_rate: 1000000        # ST3215 default: 1 Mbps
i2c_bus: 7
i2c_address: 0x28         # BNO055 ADDR pin low → 0x28, high → 0x29
uart_timeout_s: 0.05
max_retries: 3
```

### `config/robot.yaml`

Defines all 12 servos and named poses. Each servo entry:

```yaml
servos:
  - servo_id: 1                   # physical bus ID (1–253)
    joint_name: l_hip_yaw         # must match URDF joint name
    direction_sign: 1             # +1 or -1 — maps servo rotation to URDF convention
    zero_offset_steps: 2048       # encoder count (0–4095) when joint is at 0°
    default_position_deg: 0.0
    pid: {p: 32, d: 16, i: 0}
```

Default servo ID assignment (note ID 13, not 1, for `l_hip_yaw` — historical, from an early
ID reassignment):

| ID | Joint | Side | `default_position_deg` |
|----|-------|------|------------------------|
| 13 | l_hip_yaw | Left | -33.66 |
| 2 | l_hip_roll_joint | Left | -77.26 |
| 3 | l_hip_pitch_joint | Left | 37.44 |
| 4 | l_knee_joint | Left | 134.38 |
| 5 | l_ankle_roll_joint | Left | -8.70 |
| 6 | l_ankle_pitch_joint | Left | -21.88 |
| 7 | r_hip_yaw | Right | -4.75 |
| 8 | r_hip_roll_joint | Right | -7.56 |
| 9 | r_hip_pitch_joint | Right | 29.62 |
| 10 | r_knee_joint | Right | 3.60 |
| 11 | r_ankle_roll_joint | Right | 5.71 |
| 12 | r_ankle_pitch_joint | Right | 89.47 |

`default_position_deg` is non-zero here because URDF zero on this build is not the standing
pose (a physical horn-mounting offset). It defines *logical* zero (`logical = urdf − default_position_deg`),
which is the space the policy works in — see [RL Policy Deployment](#rl-policy-deployment).

Named poses are defined under `poses:` as a map of joint name → degrees:

```yaml
poses:
  stand:
    l_hip_yaw: 0.0
    l_knee_joint: 0.0
    # ... all 12 joints
  crouch:
    l_hip_pitch_joint: -30.0
    l_knee_joint: 60.0
    l_ankle_pitch_joint: -30.0
    # ...
```

---

## Web Interface

### Servos Tab

A live table of all 12 servos updated at 20 Hz showing:

- **Position °** — current angle with an interactive drag slider; dragging sends a `set_position` command in real time
- **Speed** — current velocity in steps/s, signed (1 step/s = 0.088 °/s)
- **Load %** — motor output load (0–100%)
- **Voltage V** — supply voltage (turns orange/red near limits)
- **Temp °C** — internal temperature (turns orange/red near limits)
- **Torque toggle** — enable or disable torque output per servo
- **Zero** — records the current encoder position as the zero point for that joint
- **PID** — opens a modal to set P, D, I gains and apply them live
- **ID** — opens a modal to reassign the servo's bus ID (requires torque off; reconnect after)

The **Enable All / Disable All** buttons control all 12 torques at once.

### IMU Tab

- **Euler angles** — Roll, Pitch, Yaw in degrees
- **Artificial horizon** — SVG indicator that rotates/tilts with the robot
- **Quaternion** — W, X, Y, Z components
- **Accelerometer** — X, Y, Z in m/s²
- **Gyroscope** — X, Y, Z in rad/s
- **Calibration dots** — 4 dots (red → green) showing sensor calibration quality (0–3)
- **Calibrate button** — triggers dynamic calibration mode on the BNO055

### Robot Tab

Split view: 3D viewer on the left, controls on the right.

**3D Viewer**
- Loads the robot URDF model with STL meshes via Three.js + urdf-loaders (no install needed, loaded from CDN)
- Joints update live from servo telemetry at 20 Hz
- Mouse: left-drag to orbit, right-drag to pan, scroll to zoom

**Pose Buttons**
- One button per named pose defined in `robot.yaml`
- **Home** button returns to the configured home pose

**IK Sliders (per foot)**
- X / Y / Z sliders set the target foot position in the base_link frame (metres)
- On release, IK is solved and the result is sent to the servos atomically via SYNC_WRITE
- Default ranges: X ±60 mm, Y ±70 mm, Z −180 to −320 mm

**FK Sliders (per leg, collapsible)**
- One slider per joint (degrees, within URDF joint limits)
- On release, joint angles are sent directly and the resulting foot position is shown in the console

### Debug Tab

- **Servo picker** — select by ID (scan or manual entry) to make it the "active" servo
- **Raw controls** — torque toggle and zero-point set for the active servo, in URDF space
- **Live telemetry graph** — rolling 300-point chart of raw degrees, speed, load %, voltage, and temperature for the active servo

### Control Tab

RL policy arm/drive/monitor — see [RL Policy Deployment](#rl-policy-deployment) below.

---

## REST API

All endpoints return JSON. The base URL is `http://<host>:8080`.

### Servos

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/servos/` | Status of all 12 servos |
| `GET` | `/api/servos/{id}` | Status of one servo |
| `POST` | `/api/servos/{id}/position` | `{"deg": 15.0}` — move to position |
| `POST` | `/api/servos/{id}/torque` | `{"enable": true}` — enable/disable torque |
| `POST` | `/api/servos/{id}/pid` | `{"p": 32, "d": 16, "i": 0}` — set PID gains |
| `POST` | `/api/servos/{id}/zero` | Record current position as zero offset |
| `POST` | `/api/servos/{id}/id` | `{"new_id": 5}` — reassign servo ID |
| `POST` | `/api/servos/sync_positions` | `{"joints": {"l_knee_joint": 30.0, ...}}` — move multiple joints atomically |

### IMU

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/imu/` | Latest IMU reading |
| `POST` | `/api/imu/calibrate` | Start dynamic calibration |

### Kinematics

| Method | Path | Description |
|--------|------|-------------|
| `GET` | `/api/kinematics/fk/{leg}` | FK of current joint angles (`left` or `right`) |
| `POST` | `/api/kinematics/fk/{leg}` | `{"angles_deg": [0,0,0,0,0,0]}` — FK of given angles |
| `POST` | `/api/kinematics/ik/{leg}` | `{"x": 0.0, "y": 0.021, "z": -0.25, "execute": true}` — solve IK and optionally move |
| `GET` | `/api/kinematics/poses` | List of named poses |
| `POST` | `/api/kinematics/poses/{name}` | Execute a named pose |
| `POST` | `/api/kinematics/home` | Go to home pose |

### Policy

| Method | Path | Description |
|--------|------|--------------|
| `GET` | `/api/policy/models` | List bundles found under `models/` (invalid ones included, with their error) |
| `GET` | `/api/policy/status` | Current state, model, fault, step/timing stats |
| `POST` | `/api/policy/load` | `{"name": "walk_v1"}` — load a bundle (only while `idle`) |
| `POST` | `/api/policy/arm` | Goals set to the present pose, torque on, ramp to the bundle's home pose over ~2 s |
| `POST` | `/api/policy/start` | Hand control to the policy (only from `armed`) |
| `POST` | `/api/policy/stop` | Stop stepping the policy, keep torque and hold pose |
| `POST` | `/api/policy/disarm` | Torque off, back to `idle` |
| `POST` | `/api/policy/estop` | Immediate torque cut from any state — never fails |
| `POST` | `/api/policy/clear_fault` | Clear a latched fault, back to `idle` |
| `POST` | `/api/policy/command` | `{"vx": 0.3, "vy": 0.0, "wz": 0.0}` — velocity command, limited server-side (per-axis box, or the bundle's elliptical `command_limit`); the reply has `command` (after velocity scale), `limited` and `limit_shape` |
| `GET` / `POST` | `/api/policy/safety` | Read/set tilt limit (10–45°), action-rate limit (0.05–2.0), velocity scale (0–1); out of range → 422 |
| `GET` | `/api/policy/transitions` | Recent state-machine transitions — first place to look after a fault |

---

## WebSocket

Connect to `ws://<host>:8080/ws`.

### Server → Client: Telemetry (~20 Hz)

```json
{
  "type": "telemetry",
  "timestamp": 1715000000.123,
  "servos": [
    {"id": 1, "joint": "l_hip_yaw", "position_deg": 0.0, "speed": 0,
     "load": 0, "voltage_v": 7.4, "temperature_c": 32, "torque_enabled": true}
  ],
  "imu": {
    "quaternion": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
    "euler_deg":  {"roll": 0.0, "pitch": 0.0, "yaw": 0.0},
    "accel":      {"x": 0.0, "y": 0.0, "z": 9.81},
    "gyro":       {"x": 0.0, "y": 0.0, "z": 0.0},
    "calibration": 3
  },
  "kinematics": {
    "left_foot":  {"x": -0.001, "y":  0.032, "z": -0.253},
    "right_foot": {"x": -0.001, "y": -0.032, "z": -0.253}
  },
  "policy": {
    "state": "running", "model": "walk_v1", "fault": null,
    "steps": 4210, "inference_ms": 0.4, "cycle_ms": 0.6,
    "tilt_deg": 2.1, "budget_ms": 20.0,
    "action": [0.12, -0.08, "..."], "targets_deg": {"l_hip_yaw": 3.1, "...": "..."},
    "gait_phase": 1.57
  }
}
```

`policy` is present once `onnxruntime` is installed, absent otherwise — the Control tab uses
that to hide itself on a machine without the dependency.

### Client → Server: Commands

```json
{"type": "set_position",  "servo_id": 1, "position_deg": 15.0}
{"type": "set_torque",    "servo_id": 1, "enable": true}
{"type": "set_pid",       "servo_id": 1, "p": 32, "d": 16, "i": 0}
{"type": "set_pose",      "pose": "stand"}
{"type": "set_foot_ik",   "leg": "left", "x": 0.0, "y": 0.021, "z": -0.25}
{"type": "set_joints_fk", "leg": "left", "angles_deg": [0, 0, 0, 0, 0, 0]}
{"type": "set_velocity_command", "vx": 0.3, "vy": 0.0, "wz": 0.0}
{"type": "policy_estop"}
```

`set_velocity_command` and `policy_estop` are also exposed as REST endpoints (`/api/policy/command`,
`/api/policy/estop`) — the UI fires the e-stop over both transports so it does not depend on
the socket being healthy.

**Command limit.** The server never trusts the client's command. It limits it to the bundle's trained
command region: a per-axis box from `command_ranges`, or, when `deploy_config.json` has the optional
`command_limit` key with `"shape": "ellipse_xy"`, an ellipse in (vx, vy) with `wz` on its own box.
An over-limit (vx, vy) is scaled back along its direction, so e.g. the corner (0.2, 0.15) with
vx ±0.2 / vy ±0.15 becomes (0.141, 0.106) instead of the hard corner. Bundles without the key
behave as before. A non-finite command is treated as zero. After a slider/keyboard
`set_velocity_command` the server replies `{"type": "command_applied", "command": [vx, vy, wz],
"shape": ...}` (the limited command, before the velocity scale) and the Control tab shows it;
`status().command_limit_shape` reports the shape. The gamepad stream is not echoed (its stick mapping
is already elliptical).

---

## ST3215 Servo Driver

The driver implements the **SCS/Feetech serial protocol** from scratch using `pyserial`.

**Packet format:**
```
0xFF  0xFF  ID  LEN  INSTR  [PARAMS...]  CHECKSUM
CHECKSUM = ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF
LEN = len(PARAMS) + 2
```

**Half-duplex handling:** The ST3215 bus is half-duplex — the Jetson's TX bytes loop back on its own RX line. After every write, the driver drains exactly `len(packet)` echo bytes before reading the servo's response.

**SYNC_WRITE:** All motion commands use a single broadcast `SYNC_WRITE` (instruction `0x83`) packet. This sends one packet to all servos simultaneously, ensuring synchronised joint motion without per-servo round-trip latency.

**Unit conversion:**
```
steps = zero_offset_steps + direction_sign × angle_deg × (4096 / 360)
```
`direction_sign` (+1 or −1) corrects for physical mounting orientation. `zero_offset_steps` is the raw encoder value (0–4095) at the joint's zero angle, configurable via the **Zero** button in the UI.

---

## Kinematics

### URDF Parsing

At first run, `robot_flat.urdf` is auto-generated from `Robot.urdf` by stripping the three non-kinematic xacro include lines. The kinematic chain is then parsed using Python's stdlib `xml.etree.ElementTree` — no URDF library dependency.

Each joint's origin transform and rotation axis are extracted directly. The result is two 6-joint chains (left leg and right leg) from `base_link` to the foot end-effector.

### Forward Kinematics

FK is a product of 4×4 homogeneous transforms:

```
T_foot = T_joint1(θ₁) · T_joint2(θ₂) · ... · T_joint6(θ₆)
```

Each `T_joint` = `T_origin × R(axis, θ)` using the Rodrigues rotation formula.

**FK at zero angles (from URDF):**
- Left foot:  `x=−0.001 m, y=+0.032 m, z=−0.253 m`
- Right foot: `x=−0.001 m, y=−0.032 m, z=−0.253 m`

### Inverse Kinematics

IK uses the **Damped Least Squares (DLS)** method. Analytical IK is not used because the hip joint offsets in this URDF prevent the hip pitch/roll axes from intersecting, which breaks standard closed-form humanoid IK.

Key implementation details:
- **Warm start** — always initialises from the current joint angles; avoids large jumps and converges to the physically correct branch (knee bending direction)
- **Multi-start** — retries from the calibrated standing pose if the warm-start solution exceeds 5 mm error
- **DLS step** — `dq = Jᵀ(JJᵀ + λ²I)⁻¹e`, λ²=0.01; robust near singular configurations
- **Posture regularisation** — pulls solution toward warm-start to resolve redundancy
- **Geometric Jacobian** — `J[:, i] = zᵢ × (p_e − pᵢ)`, computed analytically via `fk_all()`
- **Typical performance** — under 200 iterations, < 5 ms on Jetson

---

## RL Policy Deployment

Runs a policy trained in [`BipedRobot`](https://github.com/AsterisCrack/BipedRobot) (Isaac Lab)
against the real robot, driven from `policy/` inside the 50 Hz bus thread's spare-time slot
(~16.9 ms/cycle measured, inference is ~0.5 ms).

### 1. Export and copy a bundle

In the `BipedRobot` repo:

```bash
python -m src.isaaclab.export_onnx --checkpoint checkpoints/my_run/step_5000000.pt
python -m src.isaaclab.verify_onnx_bundle --bundle exported/my_run_step_5000000
```

Copy the resulting directory here as `models/<name>/` (must contain `policy.onnx` +
`deploy_config.json`):

```bash
cp -r ../BipedRobot/exported/my_run_step_5000000 models/walk_v1
```

### 2. Verify the sensor mapping before arming

```bash
python3 -m policy.selftest --model walk_v1              # live sensors, torque stays off
python3 -m policy.selftest --model walk_v1 --synthetic  # no hardware needed
```

Prints IMU specific-force reconstruction, projected gravity, tilt, the per-joint
robot→policy mapping, and the joint targets that *would* be commanded — nothing is written to
the bus. Run this after any recalibration.

It also prints the bundle's format version, action map, gait clock and home pose, and (live mode)
the measured pose against that home with a WARN for any joint more than 10° away — the arm ramp
will move it there.

**Calibration prerequisite:** the policy works in *logical* space,
`radians(urdf_deg − default_position_deg)` from `config/robot.yaml`, and its standing/home pose is
the bundle's `default_joint_pos` (a crouch for residual v4 bundles, 0 for centered/absolute ones) —
not URDF zero. If `default_position_deg` is wrong, every observation and target is shifted and part
of the commanded range may be clamped. `selftest.py` flags any joint already outside the policy's limits.

### 3. Arm and drive

From the Control tab, or equivalently:

```bash
curl -X POST localhost:8080/api/policy/load  -d '{"name":"walk_v1"}' -H 'Content-Type: application/json'
curl -X POST localhost:8080/api/policy/arm     # torque on, ramps to the bundle home pose over ~2 s
curl -X POST localhost:8080/api/policy/start   # hands control to the policy
curl -X POST localhost:8080/api/policy/command -d '{"vx":0.3,"vy":0,"wz":0}' -H 'Content-Type: application/json'
```

Lifecycle: `IDLE → ARMING → ARMED → RUNNING`, with `FAULT` reachable from any state. `start`
is only valid from `ARMED` — the policy never begins stepping straight from `IDLE`.

Arming first writes every servo's goal = its present position (at the slow arm speed, torque
still off), so a stale goal left in the registers cannot make a joint jump when torque comes on;
if that write fails for any servo the arm is refused. `start` is what resets the policy's runtime
state: history zero-filled (how every training episode starts), gait phase 0, previous action 0,
filtered targets at the measured pose.

**Motion lock:** while the policy is `arming`, `armed` or `running`, the endpoints that move or
reconfigure servos (poses, home, IK execute, servo position/raw position/torque *enable*/PID/zero/
default position/direction/ID/speed/accel/torque limit, sync positions, and the matching WebSocket
commands) answer HTTP 409 (WebSocket: an `error` message). Torque *disable*, e-stop, and the policy's
own endpoints are never locked. `idle` and `fault` are not locked.

### 4. Falls and recovery

Tilt past 45° (`biped_env`'s own termination threshold — the policy has never learned to
recover past it) cuts torque immediately and latches `FAULT`. So does an e-stop, a stale
sensor read, a non-finite action, or a sustained loop overrun.

Recovery is always operator-confirmed: `POST /api/policy/clear_fault` → `arm` → `start`. Arming
only ramps the joints to the home pose and does not touch the observation history; `start` zero-fills
the history and resets the gait phase, previous action and EMA filter (to the measured pose), the
same state every training episode starts from. Nothing moves autonomously.

### 5. Bundle format, action map, gait clock, traces

`deploy_config.json` carries `format_version` (v6 is the newest; the runtime refuses anything newer)
and the **action map**, which must match training exactly:

| `action_map` | Bundle | `target` for action `a ∈ [−1, 1]` |
|---|---|---|
| `absolute` | v1 | `min + (a+1)/2 · (max−min)` (`a = 0` = limit midpoint) |
| `centered` | v2+ | `a = 0` is `default_joint_pos`, `a = ±1` reach each limit |
| `residual` | v4 | `clamp(home + action_scale_rad · a, min, max)`, `home = default_joint_pos` |
| `residual`, asymmetric scales | v6 | the same with `action_scale_pos_rad` for `a ≥ 0` and `action_scale_neg_rad` for `a < 0` (no `action_scale_rad`) |

The observation's `joint_pos_rel` is `q − default_joint_pos`. **Gait clock** (`gait_clock`): v1/v2
`speed_proportional` (advances with commanded speed), v3+ `fixed_period` (constant rate `2π/period`,
held while the command is below `stand_threshold`). The runtime action-rate clip is the bundle's
`max_action_rate` only if it is > 0, otherwise 0.5; the tilt fault is `min(bundle, 0.784 rad)`.
Both are re-derived from the defaults on every load. Current training uses no in-env limiter.

Every `RUNNING` episode is saved to `data/traces/*.npz` (Control tab → *Run traces*). The `meta`
entry holds what is needed to interpret it offline (re-running the policy still needs the bundle's ONNX
and normalisation): format version, action map,
home pose and per-joint scale, joint limits, history size, control rate, EMA alpha and applications,
gait clock, per-joint `default_position_deg` from `robot.yaml`, actuator block, and the limits in
force. Format: `policy/trace.py`.

### Sensor/hardware differences the runtime accounts for

| | Isaac Lab (trained on) | This robot |
|---|---|---|
| Joint order | interleaved L/R | per-leg (all left, then all right) — permuted **by name**, never by index |
| IMU accel | specific force, gravity included (~+9.81 z upright) | BNO055 `linear_accel` is gravity-*removed* — reconstructed as `linear_accel − 9.81 × projected_gravity` |
| Action smoothing | EMA applied 4× per control step (decimation) | replicated as 4 applications, not 1 — applying once leaves the robot measurably laggier |
| Joint velocity | rad/s | servo `CURRENT_SPEED` register, in steps/s — **sign-magnitude, bit 15 = direction** (decode with `unpack_sign_magnitude16`, not as unsigned), then `direction_sign` applied. Needs `BIPED_FAST_MODE` 0 or 2; the policy refuses to arm under 1 |

---

## Bus Performance

The servo bus runs at 50 Hz (20 ms/cycle). Each cycle: SYNC_READ all 12 servos → optionally SYNC_WRITE new targets → get IMU from parallel reader thread → sleep.

**UART timing at 1 Mbps** (10 µs/byte):

| Segment | Bytes | Time |
|---------|-------|------|
| SYNC_READ request | 20 | 200 µs |
| 12 × servo response (8-byte status) | 168 | 1,680 µs |
| SYNC_WRITE (12 servos) | 68 | 680 µs |
| **Wire-only minimum** | | **~2.6 ms** |

Measured on Jetson Orin Nano: UART ~2.9 ms, state fetch ~20 µs, spare ~16.9 ms mean (P5: ~16.6 ms). Zero true overruns with SCHED_FIFO.

### Profiler

`tools/bus_profiler.py` is a `ServoBusManager` harness — it runs the same code path as production (SCHED_FIFO, IMU reader thread, SYNC_READ/WRITE) and collects per-phase timing via built-in profiling hooks.

```bash
sudo venv/bin/python tools/bus_profiler.py              # 500 cycles, read-only
sudo venv/bin/python tools/bus_profiler.py --cycles 1000 --write
```

Reports mean/std/min/max/P5/P95/P99 for: `t_uart_total`, `t_write_w` (if `--write`), `t_state_fetch`, `t_spare`, `t_cycle`. Also shows IMU reader thread I2C timing and two ASCII histograms (wide + jitter-zoomed). Run with `sudo` to apply SCHED_FIFO.

### Return delay

`tools/return_delay.py` reads and optionally zeros the `RETURN_DELAY` EEPROM register on all servos. Non-zero values add inter-servo gaps to each SYNC_READ cycle.

```bash
sudo venv/bin/python tools/return_delay.py              # read-only
sudo venv/bin/python tools/return_delay.py --set-zero   # write 0 to all
```

### Optimizations

Always active:

| Optimization | Effect |
|---|---|
| **Parallel IMU** — dedicated `_IMUReaderThread` reads BNO055 over I2C continuously | removes 4 ms I2C latency from servo cycle |
| **Batch RX** — one `read(168)` instead of 12 × `read(14)` | −550–1,100 µs/cycle |
| **No flush()** — `tcdrain` removed from hot path (Jetson L4T adds ~10 ms/call) | −10 ms/cycle |
| **Single BNO055 bulk read** — registers 0x14–0x2D in one 26-byte I2C transaction | −3 separate I2C calls |

Opt-in via `.env`:

**`BIPED_RT_SCHEDULING=1`** — applies `SCHED_FIFO` (Linux real-time scheduling, priority 10) to the servo bus thread. Eliminates 1–3 ms P99 spikes from OS preemption. Requires:

```bash
sudo setcap cap_sys_nice+eip $(readlink -f venv/bin/python3)
# or: sudo venv/bin/python main.py
```

Note: `setcap` is silently ignored on `nosuid` filesystems (getcap will still show it; use `findmnt -T venv/bin/python3` to check). Running as root bypasses this.

**`BIPED_FAST_MODE`** — how many bytes SYNC_READ pulls per servo from the status block at `0x38`. The registers are contiguous, so a shorter read is just a prefix; whatever is skipped reads as 0. UART time is at 1 Mbps over 12 servos, against a 20 ms cycle.

| Mode | Bytes/servo | Fields | UART | vs mode 0 |
|---|---|---|---|---|
| `0` (default) | 8 | pos, speed, load, voltage, temp | 1680 µs | — |
| `1` | 2 | pos | 960 µs | −720 µs |
| `2` | 4 | pos, speed | 1200 µs | −480 µs |

**Mode 2 is the one to use when running the policy**: it keeps joint velocity and still drops two thirds of what mode 1 saves. Mode 1 is incompatible with the policy — `PolicyRunner.arm()` and `policy.selftest` both refuse to start under it, because a zero joint_vel channel is in-range and plausible rather than obviously broken. Modes 1 and 2 both blank the load/voltage/temperature columns in the web UI.

---

## Calibrating Servos

Before the first run, each servo's **zero offset** must be set so the software knows where 0° is:

1. **Physically move the robot to its zero pose** (all joints at their mechanical zero, typically the standing position with legs straight)
2. **Disable all torques** using the "Disable All Torques" button
3. For each servo, click **Zero** in the servo table — this reads the current encoder position and saves it as `zero_offset_steps` in memory
4. The offsets are applied immediately for the current session

To persist offsets across restarts, manually update the `zero_offset_steps` values in `config/robot.yaml` to match what the Zero button read.

---

## Changing Servo IDs

New ST3215 servos ship with ID 1. To assign unique IDs:

1. Connect **one servo at a time** to the bus
2. In the Servos tab, find the servo (it will appear with ID 1)
3. Click **ID**, enter the desired new ID (1–253), and confirm
4. **Power cycle the servo** — the ID change is written to EEPROM but takes effect after restart
5. Update `config/robot.yaml` to reflect the new ID

---

## Jetson Orin Nano Setup Notes

**UART:**
- Hardware UART on 40-pin header: `/dev/ttyTHS1` (pins 8/10)
- Default baud rate: 1 Mbps — supported by Jetson hardware UART
- Half-duplex requires an external transceiver (e.g. Waveshare Bus Servo Adapter) or a USB-to-SCS adapter (`/dev/ttyUSB0`)
- Add your user to the `dialout` group: `sudo usermod -aG dialout $USER`

**I2C:**
- BNO055 on 40-pin header I2C: bus 7 (pin 3 = SDA, pin 5 = SCL)
- Verify: `sudo i2cdetect -y 7` — BNO055 should appear at address `0x28` (ADDR pin low)
- Driver uses `Adafruit_PureIO.smbus` directly — no GPIO or Blinka HAL calls needed

**Autostart (optional):**

```bash
# /etc/systemd/system/bipedrobot.service
[Unit]
Description=Biped Robot Control
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/<user>/BipedRobotJetson/main.py
WorkingDirectory=/home/<user>/BipedRobotJetson
Restart=on-failure
User=<user>

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable bipedrobot
sudo systemctl start bipedrobot
```

---

## Development Without Hardware

The server starts in a degraded mode if the serial port or IMU is unavailable — it logs a warning and continues. The web UI, kinematics, and REST API all work normally; servo status reads return zeros and IMU returns a static identity quaternion.

To run locally:

```bash
pip3 install -r requirements.txt   # no adafruit-blinka needed
python3 main.py
# → http://localhost:8080
```

---

## Dependencies

| Package | Purpose |
|---------|---------|
| `pyserial` | UART communication with ST3215 servos |
| `numpy` | Kinematics matrix math |
| `scipy` | Numerical IK optimisation |
| `fastapi` | Web framework and REST API |
| `uvicorn` | ASGI server |
| `pydantic` + `pydantic-settings` | Config models and `.env` loading |
| `pyyaml` | YAML config file parsing |
| `onnxruntime` | RL policy inference (CPU execution provider — plenty for a 250→1024→512→256→12 MLP against a ~16.9 ms/cycle budget) *(optional)* |
| `Jetson.GPIO` | Jetson GPIO HAL (required by adafruit-blinka for platform detection) *(hardware only)* |
| `adafruit-blinka` | Provides `Adafruit_PureIO.smbus` for direct I2C register access *(hardware only)* |

Three.js and urdf-loader are loaded from CDN at runtime — no npm or build step required.