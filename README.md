# BipedRobotJetson

Standalone Python control stack for a 12-DOF biped robot running on a **Jetson Orin Nano**. Features a single web-based control interface accessible from any browser on the local network — no ROS, no separate scripts per operation.

---

## Hardware

| Component | Details |
|-----------|---------|
| Computer | NVIDIA Jetson Orin Nano |
| Servos | 12× ST3215 smart servos (SCS/Feetech protocol, half-duplex UART) |
| IMU | BNO085 (I2C, Adafruit library) |
| Legs | 2× 6-DOF (hip yaw/roll/pitch · knee · ankle roll/pitch) |

---

## Features

- **Servo control** — real-time position, speed, load, voltage, temperature monitoring; per-servo position sliders; torque enable/disable; PID tuning; ID reassignment; zero-point calibration
- **IMU readouts** — quaternion, Euler angles (roll/pitch/yaw), accelerometer, gyroscope, calibration status, artificial horizon display
- **Forward kinematics** — computes foot position from current joint angles using transforms extracted directly from the URDF
- **Inverse kinematics** — numerical IK (scipy SLSQP) from a target foot position to joint angles, with warm-start from current pose
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
│       └── bno085.py             # BNO085 wrapper (quat → Euler, calibration)
│
├── kinematics/                   # Standalone FK/IK library (no web/robot/hardware deps)
│   ├── README.md                 # Library documentation + usage examples
│   ├── chain.py                  # URDF parser → kinematic chain, FK (Rodrigues)
│   └── solver.py                 # FK + numerical IK (scipy SLSQP + L-BFGS-B fallback)
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

Edit `.env` to match your Jetson's device nodes:

```env
UART_PORT=/dev/ttyTHS1   # or /dev/ttyUSB0 for USB adapter
I2C_BUS=7                # 40-pin header I2C on Jetson Orin Nano
```

Verify with:

```bash
ls /dev/ttyTHS*          # hardware UART
sudo i2cdetect -y 7      # BNO085 should appear at 0x4a
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
i2c_address: 0x4A         # BNO085 ADDR pin low → 0x4A, high → 0x4B
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
- **Calibrate button** — triggers dynamic calibration mode on the BNO085

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

IK uses `scipy.optimize.minimize` with the SLSQP method (bounded, gradient-based). Analytical IK is not used because the hip joint offsets in this URDF prevent the hip pitch/roll axes from intersecting, which breaks standard closed-form humanoid IK.

Key implementation details:
- **Warm start** — always initialises from the current joint angles; avoids large jumps and converges to the physically correct branch (knee bending direction)
- **Joint limits** — bounds extracted from the URDF, applied as scipy bounds constraints
- **Cost function** — weighted sum of position error² + orientation error² (orientation weight 0.1, or disabled for position-only targets)
- **Fallback** — if SLSQP fails to converge, retries with L-BFGS-B
- **Typical performance** — 50–150 iterations, < 2 ms on Jetson

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
- BNO085 on 40-pin header I2C: bus 7 (pin 3 = SDA, pin 5 = SCL)
- Verify: `sudo i2cdetect -y 7` — BNO085 should appear at address `0x4a`
- Adafruit Blinka auto-detects Jetson; no extra env vars needed for recent versions

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
| `adafruit-blinka` | CircuitPython HAL for Jetson *(hardware only)* |
| `adafruit-circuitpython-bno08x` | BNO085 IMU driver *(hardware only)* |

Three.js and urdf-loader are loaded from CDN at runtime — no npm or build step required.



## ST3215 registers

> Bytes are little-endian


| Memory First Address   | Function                                       |   Number of Bytes |   Initial Value | Storage Area   | Permission   | Minimum Value   | Maximum Value   | Unit         | Value Parsing                                                                                                                                                                                                                                                                                                                                                                                   |
|:-----------------------|:-----------------------------------------------|------------------:|----------------:|:---------------|:-------------|:----------------|:----------------|:-------------|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| 0x0                    | Firmware major version number                  |                 1 |               3 | EPROM          | read only    | -1              | -1              | nan          | nan                                                                                                                                                                                                                                                                                                                                                                                             |
| 0x1                    | Firmware minor version number                  |                 1 |               7 | EPROM          | read only    | -1              | -1              | nan          | nan                                                                                                                                                                                                                                                                                                                                                                                             |
| 0x3                    | Servo major version number                     |                 1 |               9 | EPROM          | read only    | -1              | -1              | nan          | nan                                                                                                                                                                                                                                                                                                                                                                                             |
| 0x4                    | Servo minor version number                     |                 1 |               3 | EPROM          | read only    | -1              | -1              | nan          | nan                                                                                                                                                                                                                                                                                                                                                                                             |
| 0x5                    | ID                                             |                 1 |               1 | EPROM          | read/write   | 0               | 253             | Baud         | A unique identification code on the bus, with no duplicate ID numbers allowed on the same bus. ID number 254 (0xFE) is the broadcast ID, and broadcasts do not receive response packets.                                                                                                                                                                                                        |
| 0x6                    | Baudrate                                       |                 1 |               0 | EPROM          | read/write   | 0               | 7               | None         | 0-7 respectively represent baud rates as follows:                                                                                                                                                                                                                                                                                                                                               |
|                        |                                                |                   |                 |                |              |                 |                 |              | 1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400                                                                                                                                                                                                                                                                                                                                    |
| 0x7                    | Return delay                                   |                 1 |               0 | EPROM          | read/write   | 0               | 254             | 2us          | The minimum unit is 2us, and the maximum allowable setting for response delay is 254*2=508us                                                                                                                                                                                                                                                                                                    |
| 0x8                    | Response status level                          |                 1 |               1 | EPROM          | read/write   | 0               | 1               | None         | 0: Except for read and PING instructions, other instructions do not return response packets.                                                                                                                                                                                                                                                                                                    |
|                        |                                                |                   |                 |                |              |                 |                 |              | 1: Return response packets for all instructions                                                                                                                                                                                                                                                                                                                                                 |
| 0x9                    | Minimum angle                                  |                 2 |               0 | EPROM          | read/write   | -32766          | --              | Step         | Set the minimum value limit for the motion range, which should be smaller than the maximum angle limit. When performing multi-turn absolute position control, this value is set to 0.                                                                                                                                                                                                           |
| 0xB                    | Maximum angle                                  |                 2 |            4095 | EPROM          | read/write   | --              | 32767           | Step         | Set the maximum value limit for the motion range, which should be greater than the minimum angle limit. When performing multi-turn absolute position control, this value is set to 0.                                                                                                                                                                                                           |
| 0xD                    | Maximum temperature                            |                 1 |              70 | EPROM          | read/write   | 0               | 100             | °C           | The maximum operating temperature limit, when set to 70, means the maximum temperature is 70 degrees Celsius, with a precision setting of 1 degree Celsius.                                                                                                                                                                                                                                     |
| 0xE                    | Maximum input voltage                          |                 1 |              80 | EPROM          | read/write   | 0               | 254             | 0.1V         | If the maximum input voltage is set to 80, then the maximum operating voltage limit is 8.0V, with a precision setting of 0.1V.                                                                                                                                                                                                                                                                  |
| 0xF                    | Minimum input voltage                          |                 1 |              40 | EPROM          | read/write   | 0               | 254             | 0.1V         | If the minimum input voltage is set to 40, then the minimum operating voltage limit is 4.0V, with a precision setting of 0.1V.                                                                                                                                                                                                                                                                  |
| 0x10                   | Maximum torque                                 |                 2 |            1000 | EPROM          | read/write   | 0               | 1000            | nan          | Set the maximum output torque limit for the servo motor, where 1000 corresponds to 100% of the locked-rotor torque. Assign this value to address 48 upon power-up as the torque limit.                                                                                                                                                                                                          |
| 0x12                   | Phase                                          |                 1 |              12 | EPROM          | read/write   | 0               | 254             | None         | Special function byte, do not modify unless there are specific requirements. Please refer to the special byte bit analysis for further details.                                                                                                                                                                                                                                                 |
| 0x13                   | Unloading conditions                           |                 1 |              44 | EPROM          | read/write   | 0               | 254             | None         | Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 set corresponding bit to 1 to enable the corresponding protection.                                                                                                                                                                                                                                                                                              |
|                        |                                                |                   |                 |                |              |                 |                 |              | Voltage Sensor Temperature Current Angle Overload set corresponding bit to 0 to disable the corresponding protection                                                                                                                                                                                                                                                                            |
| 0x14                   | LED alarm conditions                           |                 1 |              47 | EPROM          | read/write   | 0               | 254             | None         | Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 set the corresponding bit to 1 to enable flashing  LED.                                                                                                                                                                                                                                                                                                         |
|                        |                                                |                   |                 |                |              |                 |                 |              | Voltage Sensor Temperature Current Angle Overload set corresponding bit to 0 to disable the corresponding protection                                                                                                                                                                                                                                                                            |
| 0x15                   | Position loop P (Proportional) coefficient     |                 1 |              32 | EPROM          | read/write   | 0               | 254             | None         | Proportional coefficient of control motor                                                                                                                                                                                                                                                                                                                                                       |
| 0x16                   | Position loop D (Differential) coefficient     |                 1 |              32 | EPROM          | read/write   | 0               | 254             | None         | Differential coefficient of control motor                                                                                                                                                                                                                                                                                                                                                       |
| 0x17                   | Position loop I (Integral) coefficient         |                 1 |               0 | EPROM          | read/write   | 0               | 254             | None         | Integral coefficient of the control motor                                                                                                                                                                                                                                                                                                                                                       |
| 0x18                   | Minimum starting force                         |                 2 |              16 | EPROM          | read/write   | 0               | 1000            | nan          | Set the minimum output startup torque for the servo, where 1000 corresponds to 100% of the locked-rotor torque.                                                                                                                                                                                                                                                                                 |
| 0x1A                   | Clockwise insensitive zone                     |                 1 |               1 | EPROM          | read/write   | 0               | 32              | Step         | The minimum unit is one minimum resolution angle.                                                                                                                                                                                                                                                                                                                                               |
| 0x1B                   | Anti-clockwise insensitive zone                |                 1 |               1 | EPROM          | read/write   | 0               | 32              | Step         | The minimum unit is a minimum resolution angle.                                                                                                                                                                                                                                                                                                                                                 |
| 0x1C                   | Protection current                             |                 2 |             500 | EPROM          | read/write   | 0               | 511             | 6.5mA        | The maximum settable current is 500 * 6.5mA= 3250mA.                                                                                                                                                                                                                                                                                                                                            |
| 0x1E                   | Angle resolution                               |                 1 |               1 | EPROM          | read/write   | 1               | 3               | None         | For the amplification factor of the minimum resolution angle (degree/step) of the sensor, modifying this value can expand the number of control range. When performing the multi-turn control, you need to modify the parameter at address 0x12 by setting BIT4 to 1. This modification will result in the current position feedback value being adjusted to reflect the larger angle feedback. |
| 0x1F                   | Position correction                            |                 2 |               0 | EPROM          | read/write   | -2047           | 2047            | Step         | BIT11 is the direction bit, indicating the positive and negative direction, and other bits can indicate the range of 0-2047 steps.                                                                                                                                                                                                                                                              |
| 0x21                   | Operation mode                                 |                 1 |               0 | EPROM          | read/write   | 0               | 3               | None         | 0: position servo mode                                                                                                                                                                                                                                                                                                                                                                          |
|                        |                                                |                   |                 |                |              |                 |                 |              | 1: motor constant speed mode, controlled by parameter 0x2E running speed parameter, the highest bit BIT15 is direction bit.                                                                                                                                                                                                                                                                     |
|                        |                                                |                   |                 |                |              |                 |                 |              | 2: PWM open-loop speed regulation mode, controlled by parameter 0x2Cthe  running time parameter, BIT10 is direction bit                                                                                                                                                                                                                                                                         |
|                        |                                                |                   |                 |                |              |                 |                 |              | 3: step servo mode, and the target position of parameter 0x2A is used to indicate the number of steps, and the highest bit BIT15 is the direction bit. When working                                                                                                                                                                                                                             |
|                        |                                                |                   |                 |                |              |                 |                 |              | In mode 3, the minimum and maximum angle limits of 0x9 and 0xB must be set to 0. Otherwise, it is impossible to step indefinitely.                                                                                                                                                                                                                                                              |
| 0x22                   | Protection torque                              |                 1 |              20 | EPROM          | read/write   | 0               | 100             | nan          | Output torque after entering overload protection. If 20 is set, it means 20% of the maximum torque.                                                                                                                                                                                                                                                                                             |
| 0x23                   | Protection time                                |                 1 |             200 | EPROM          | read/write   | 0               | 254             | 10ms         | The duration for which the current load output exceeds the overload torque and remains is represented by a value, such as 200, which indicates 2 seconds. The maximum value that can be set is 2.5 seconds.                                                                                                                                                                                     |
| 0x24                   | Overload torque                                |                 1 |              80 | EPROM          | read/write   | 0               | 100             | nan          | The maximum torque threshold for starting the overload protection time countdown can be represented by a value, such as 80, indicating 80% of the maximum torque.                                                                                                                                                                                                                               |
| 0x25                   | Speed closed-loop proportional (P) coefficient |                 1 |              10 | EPROM          | read/write   | 0               | 100             | None         | Proportional coefficient of speed loop in motor constant speed mode (mode 1)                                                                                                                                                                                                                                                                                                                    |
| 0x26                   | Overcurrent protection time                    |                 1 |             200 | EPROM          | read/write   | 0               | 254             | 10ms         | The maximum setting is 254 * 10ms = 2540ms.                                                                                                                                                                                                                                                                                                                                                     |
| 0x27                   | Velocity closed-loop integral (I) coefficient  |                 1 |              10 | EPROM          | read/write   | 0               | 254             | 1/10         | In the motor constant speed mode (mode 1), the speed loop integral coefficient (change note: the speed closed loop I integral coefficient is reduced by 10 times compared with version 3.6).                                                                                                                                                                                                    |
| 0x28                   | Torque switch                                  |                 1 |               0 | SRAM           | read/write   | 0               | 128             | None         | Write 0: disable the torque output; Write 1: enable the torque output; Write 128: Arbitrary current position correction to 2048.                                                                                                                                                                                                                                                                |
| 0x29                   | Acceleration                                   |                 1 |               0 | SRAM           | read/write   | 0               | 254             | 100 step/s^2 | If set to 10, it corresponds to an acceleration and deceleration rate of 1000 steps per second squared.                                                                                                                                                                                                                                                                                         |
| 0x2A                   | Target location                                |                 2 |               0 | SRAM           | read/write   | -30719          | 30719           | Step         | Each step corresponds to the minimum resolution angle, and it is used in absolute position control mode. The maximum number of steps corresponds to the maximum effective angle.                                                                                                                                                                                                                |
| 0x2C                   | Operation time                                 |                 2 |               0 | SRAM           | read/write   | 0               | 1000            | nan          | In the PWM open-loop speed control mode, the value range is from 50 to 1000, and BIT10 serves as the direction bit.                                                                                                                                                                                                                                                                             |
| 0x2E                   | Operation speed                                |                 2 |               0 | SRAM           | read/write   | 0               | 3400            | step/s       | Number of steps per unit time (per second), 50 steps per second = 0.732 RPM (revolutions per minute)                                                                                                                                                                                                                                                                                            |
| 0x30                   | Torque limit                                   |                 2 |            1000 | SRAM           | read/write   | 0               | 1000            | nan          | The initial value of power-on will be assigned by the maximum torque (0x10), which can be modified by the user to control the output of the maximum torque.                                                                                                                                                                                                                                     |
| 0x37                   | Lock flag                                      |                 1 |               0 | SRAM           | read/write   | 0               | 1               | None         | Writing 0: Disables the write lock, allowing values written to the EPROM address to be saved even after power loss.                                                                                                                                                                                                                                                                             |
|                        |                                                |                   |                 |                |              |                 |                 |              | Writing 1: Enables the write lock, preventing values written to the EPROM address from being saved after power loss.                                                                                                                                                                                                                                                                            |
| 0x38                   | Current location                               |                 2 |               0 | SRAM           | read only    | -1              | -1              | Step         | Feedback the number of steps in the current position, each step is a minimum resolution angle; Absolute position control mode, the maximum value corresponds to the maximum effective angle.                                                                                                                                                                                                    |
| 0x3A                   | Current speed                                  |                 2 |               0 | SRAM           | read only    | -1              | -1              | step/s       | Feedback the current speed of motor rotation and the number of steps in unit time (per second).                                                                                                                                                                                                                                                                                                 |
| 0x3C                   | Current load                                   |                 2 |               0 | SRAM           | read only    | -1              | -1              | nan          | The voltage duty cycle of the current control output driving motor.                                                                                                                                                                                                                                                                                                                             |
| 0x3E                   | Current voltage                                |                 1 |               0 | SRAM           | read only    | -1              | -1              | 0.1V         | Current servo operation voltage                                                                                                                                                                                                                                                                                                                                                                 |
| 0x3F                   | Current temperature                            |                 1 |               0 | SRAM           | read only    | -1              | -1              | °C           | Current servo internal operating temperature                                                                                                                                                                                                                                                                                                                                                    |
| 0x40                   | Asynchronous write flag                        |                 1 |               0 | SRAM           | read only    | -1              | -1              | None         | The flag bit for using asynchronous write instructions                                                                                                                                                                                                                                                                                                                                          |
| 0x41                   | Servo status                                   |                 1 |               0 | SRAM           | read only    | -1              | -1              | None         | Bit0  Bit1  Bit2 Bit3 Bit4 Bit5 the corresponding bit is set to 1 to indicate that the corresponding error occurs,                                                                                                                                                                                                                                                                              |
|                        |                                                |                   |                 |                |              |                 |                 |              | Voltage Sensor Temperature Current Angle Overload the corresponding bit is set to 0 to indicate that there is no corresponding error.                                                                                                                                                                                                                                                           |
| 0x42                   | Move flag                                      |                 1 |               0 | SRAM           | read only    | -1              | -1              | None         | The sign of the servo is 1 when it is moving, and 0 when it is stopped.                                                                                                                                                                                                                                                                                                                         |
| 0x45                   | Current current                                |                 2 |               0 | SRAM           | read only    | -1              | -1              | 6.5mA        | The maximum measurable current is 500 * 6.5mA= 3250mA.                                                                                                                                                                                                                                                                                                                                          |