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
├── robot/                        # Robot orchestrator (uses hardware/ and kinematics/)
│   ├── config.py                 # RobotConfig + Settings — loads .env + both YAMLs
│   └── robot.py                  # Robot class: motion, IK, telemetry loop
│
├── web/                          # Debug web UI (uses robot/ only)
│   ├── app.py                    # FastAPI factory, lifespan, static mounts
│   ├── routers/
│   │   ├── servos.py             # REST /api/servos/*
│   │   ├── imu.py                # REST /api/imu/*
│   │   └── kinematics.py         # REST /api/kinematics/*
│   ├── websocket.py              # WS broadcaster + command dispatcher
│   └── static/
│       ├── index.html            # Single-page app (3 tabs)
│       ├── css/style.css
│       └── js/
│           ├── app.js            # Tab router, central WS manager
│           ├── servos.js         # Servo tab
│           ├── imu.js            # IMU tab
│           └── robot3d.js        # Three.js viewer, IK/FK sliders, pose buttons
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
web/         ←  imports from robot/ only
```

`hardware/` and `kinematics/` can be copied into any future project and used independently.

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
BIPED_FAST_MODE=1        # Position-only reads — saves 720 µs per cycle

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

Default servo ID assignment:

| ID | Joint | Side |
|----|-------|------|
| 1 | l_hip_yaw | Left |
| 2 | l_hip_roll_joint | Left |
| 3 | l_hip_pitch_joint | Left |
| 4 | l_knee_joint | Left |
| 5 | l_ankle_roll_joint | Left |
| 6 | l_ankle_pitch_joint | Left |
| 7 | r_hip_yaw | Right |
| 8 | r_hip_roll_joint | Right |
| 9 | r_hip_pitch_joint | Right |
| 10 | r_knee_joint | Right |
| 11 | r_ankle_roll_joint | Right |
| 12 | r_ankle_pitch_joint | Right |

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
- **Speed** — current velocity in steps/s
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
  }
}
```

### Client → Server: Commands

```json
{"type": "set_position",  "servo_id": 1, "position_deg": 15.0}
{"type": "set_torque",    "servo_id": 1, "enable": true}
{"type": "set_pid",       "servo_id": 1, "p": 32, "d": 16, "i": 0}
{"type": "set_pose",      "pose": "stand"}
{"type": "set_foot_ik",   "leg": "left", "x": 0.0, "y": 0.021, "z": -0.25}
{"type": "set_joints_fk", "leg": "left", "angles_deg": [0, 0, 0, 0, 0, 0]}
```

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

**`BIPED_FAST_MODE=1`** — SYNC_READ fetches only position (2 bytes/servo) instead of the full 8-byte status block. Saves 720 µs of pure UART time per cycle. Side effect: speed, load, voltage, and temperature read as 0 in telemetry.

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
| `Jetson.GPIO` | Jetson GPIO HAL (required by adafruit-blinka for platform detection) *(hardware only)* |
| `adafruit-blinka` | Provides `Adafruit_PureIO.smbus` for direct I2C register access *(hardware only)* |

Three.js and urdf-loader are loaded from CDN at runtime — no npm or build step required.