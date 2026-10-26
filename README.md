# BipedRobotJetson

**The on-robot half of [BipedRobot](https://github.com/AsterisCrack/BipedRobot): a standalone Python
stack that runs a 12-servo biped from a Jetson Orin Nano.** It talks to the servos and the IMU
directly, serves a web control panel to any browser on your network, and runs an exported ONNX
walking policy at 50 Hz with the safety checks a real robot needs. No ROS, no torch, no Isaac Lab.

## Features

- **One web app for everything.** Servo table, IMU readout, 3D robot view with forward/inverse
  kinematics, single-servo debug, policy control and servo identification, plus a REST + WebSocket API.
- **From-scratch drivers.** ST3215 (Feetech SCS protocol) over a half-duplex UART with batched
  SYNC_READ / SYNC_WRITE, and a BNO055 IMU driver, in the standalone [`hardware/`](hardware/README.md) library.
- **Policy runner and safety.** Loads an ONNX bundle, builds the training observation and runs read, infer,
  write in one 50 Hz bus cycle, behind an arm/start/stop/e-stop lifecycle, a register audit, tilt,
  stale-sensor and action-rate limits, a command limiter and a motion lock.
- **Run traces** of every policy run at 50 Hz, and **no hardware needed** to start the server, so the UI,
  kinematics and tests work on a laptop.

---

## Hardware

A Jetson Orin Nano, 12 Feetech STS3215 servos (2 legs x 6 DOF: hip yaw, roll, pitch, knee, ankle roll,
ankle pitch) on one shared bus with a 12 V supply, a
[Waveshare Bus Servo Adapter (A)](https://www.waveshare.com/wiki/Bus_Servo_Adapter_(A)) between the Jetson
UART and the bus, and a BNO055 IMU on I2C (address `0x28`). Servo IDs and joint names are in
[`config/robot.yaml`](config/robot.yaml); wiring detail is in [`hardware/README.md`](hardware/README.md).

---

## Install and Jetson setup

### 1. Python dependencies

```bash
git clone https://github.com/AsterisCrack/BipedRobotJetson
cd BipedRobotJetson
python3 -m venv venv && source venv/bin/activate
pip install -r requirements.txt            # pyserial, numpy, scipy, fastapi, uvicorn, pydantic, pyyaml, onnxruntime
pip install -r requirements-hardware.txt   # Jetson only: Jetson.GPIO + adafruit-blinka (I2C access)
```

`onnxruntime` is optional: without it the server starts and the Control tab reports the policy feature
as unavailable. The CPU execution provider is deliberate, because the actor is a small MLP. The web UI
loads Three.js from a CDN, so the browser needs internet access.

### 2. Serial port and IMU

The Waveshare adapter uses **straight-through** wiring, not a TX/RX crossover: Jetson pin 8 (TX) to the
adapter's TX, pin 10 (RX) to RX, pin 6 (GND) to GND. The BNO055 sits on the header's I2C bus 7 (pin 3 = SDA,
pin 5 = SCL). The header UART is off on a fresh Jetson image. Enable it once, then reboot:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py     # Configure 40pin header, set pins 8 + 10 to "uarta"
cat /sys/class/tty/ttyTHS1/uartclk          # after reboot: must be non-zero
sudo usermod -aG dialout $USER              # then log out and back in
sudo i2cdetect -y 7                         # the BNO055 should appear at 0x28
```

A USB-to-SCS adapter works too (`UART_PORT=/dev/ttyUSB0`). The default baud rate is 1 Mbaud.

### 3. `.env`

Copy the example and uncomment only what you change. Do not put inline comments after a value.

```bash
cp .env.example .env
```

| Variable | Default | Meaning |
|---|---|---|
| `UART_PORT` | `/dev/ttyTHS1` | Servo bus serial device |
| `I2C_BUS` | `7` | IMU I2C bus |
| `BIPED_FAST_MODE` | `0` | How much each servo sends per cycle (below) |
| `BIPED_RT_SCHEDULING` | off | `1` runs the bus thread with `SCHED_FIFO` priority 10 |
| `BIPED_DEBUG` | off | Any non-empty value turns on DEBUG logs for `kinematics/`, `robot/` and the bus manager |

**`BIPED_FAST_MODE`** sets how many bytes SYNC_READ pulls per servo. Fields a mode skips read as 0.

| Mode | Fields | Use |
|---|---|---|
| `0` | position, speed, load, voltage, temperature | Full UI telemetry, and load/voltage/temperature in traces |
| `1` | position | Fastest bus, but **the policy refuses to arm**: it has no joint velocity |
| `2` | position, speed | The slimmest mode the policy accepts, and a shorter bus cycle than mode 0 |

**`BIPED_RT_SCHEDULING`** reduces timing jitter. It needs `CAP_SYS_NICE` or root
(`sudo setcap cap_sys_nice+eip $(readlink -f venv/bin/python3)`, silently ignored on `nosuid` filesystems).
The startup log says whether `SCHED_FIFO` was applied.

### 4. Run

```bash
python3 main.py        # web UI + API on http://<jetson-ip>:8080
```

Servo torque is always **off** at startup. The server runs a single worker, because only one process may
own the serial bus. To start on boot, wrap the same command in a systemd service
(`WorkingDirectory` = the repo, `ExecStart` = `venv/bin/python main.py`, `Restart=on-failure`,
`WantedBy=multi-user.target`) and run `sudo systemctl enable --now bipedrobot`.

---

## Configuration

Settings are merged at startup from [`config/hardware.yaml`](config/hardware.yaml) (bus and IMU),
[`config/robot.yaml`](config/robot.yaml) (servos and poses) and `.env` (port, bus and tuning).
Restart the server after editing any of them.

### `config/hardware.yaml`

```yaml
baud_rate: 1000000
i2c_address: 0x28          # BNO055 ADDR pin low = 0x28, high = 0x29
uart_timeout_s: 0.005
max_retries: 3
imu_mode: ndof             # ndof | imuplus
imu_raw_accel: false
imu_accel_average: true
expect_echo: false         # false for the Waveshare adapter (no TX loopback); not in the shipped file
```

`uart_port` and `i2c_bus` also appear in the file, but `.env` (or its defaults) always overrides them, so
set the port and bus in `.env`.

- **`imu_mode`**: `ndof` fuses the magnetometer, `imuplus` does not. The servos' motor fields can disturb the
  magnetometer and the policy never uses heading, so try `imuplus` if the orientation drifts.
- **`imu_raw_accel`**: `true` feeds the policy the measured accelerometer (specific force, as in training);
  `false` rebuilds it from the fused linear acceleration (see [observation](#observation-frame)).
- **`imu_accel_average`**: `true` gives the policy the **mean acceleration over each 20 ms tick**, as training
  does, which removes the foot-impact spikes a single read would hand it.

### `config/robot.yaml`

```yaml
urdf_path: RobotDescription/urdf/robot_flat.urdf
home_pose: stand            # the pose the Home button runs

servos:
  - servo_id: 13
    joint_name: l_hip_yaw          # must match the URDF joint name
    direction_sign: 1              # +1 / -1: servo rotation to URDF convention
    zero_offset_steps: 2048        # encoder count (0-4095) at the joint's zero
    default_position_deg: -36.0    # standing offset, robot-specific, see below
    pid: {p: 32, d: 0, i: 0}

poses:                             # named poses, in logical degrees (0 = standing)
  crouch: {l_hip_pitch_joint: -30.0, l_knee_joint: 60.0}     # ...all 12 joints
```

- **Two angle spaces.** *URDF space* has 0° at the servo's calibrated centre. *Logical space* has 0° at
  the robot's standing pose: `logical = urdf - default_position_deg`. Poses, the UI sliders and the policy
  all use logical space. `default_position_deg` is non-zero because the horn mounting offsets the standing
  pose from URDF zero. **It must be right for every joint**, or every policy observation and target is shifted.
- **PID is P-only (`d: 0`, `i: 0`) on purpose.** The training servo model is a P-only firmware loop. A
  bundle states the PID it needs, and `arm()` refuses on a mismatch.
- `direction_sign` must be exactly `-1` or `1`.

New poses go under `poses:` and appear in the Robot tab automatically.

---

## The web app

Open `http://<jetson-ip>:8080`. Telemetry updates at 20 Hz.

The tabs are **Servos** (live table with position slider, torque toggle, per-servo Zero, PID and ID
dialogs), **IMU** (orientation, horizon, accelerometer, gyro, calibration status), **Robot** (3D URDF viewer
driven by the live joints, pose buttons and Home, foot IK and joint FK sliders per leg, a Config panel and
**Print Config**), **Debug** (ID reassignment, raw single-servo control, rolling telemetry plot), **Control**
(policy load, arm, start, stop, e-stop, velocity command, auto-recover, safety limits, run traces) and
**SysID** (servo identification recordings, below).

The header's **Sim Mode** switch suppresses all servo writes: commands then move only the 3D model.

The Robot tab's IK is numerical (damped least squares), because the hip offsets rule out a closed form. The
leg chains are parsed from the URDF by [`kinematics/`](kinematics/README.md), which can be used on its own.

### Calibration

Do this once per build, and again after reseating a horn.

1. Disable all torques and pose the robot by hand at its standing pose.
2. **Direction.** In the Robot tab, Config panel, toggle each joint's sign until a slider moves the real joint the same way as the 3D model.
3. **Zero.** In the Debug tab, *Set Current As Zero (config)* writes `zero_offset_steps` to `config/robot.yaml` and resets that joint's `default_position_deg` to 0. The Servos tab's **Zero** button only changes the running session.
4. **Standing offset.** With the robot in the pose you want as logical zero, click **Set Here** per joint, then **Print Config**. That logs the YAML and writes `/tmp/robot_config.yaml`; copy the servo entries into `config/robot.yaml`.
5. Restart, then check with the [self-test](#3-self-test-torque-off).

New servos ship as ID 1: connect **one at a time** and use Debug tab, *Reassign Servo IDs* (torque off), then
update `config/robot.yaml`.

### SysID

The SysID tab records single-servo runs on a test rig, plus a hysteresis screen of all 12 joints in place,
in the raw format of Rhoban's [BAM](https://github.com/Rhoban/bam), under `data/sysid/<session>/`. Fitting
happens in BipedRobot's [`actuator/`](https://github.com/AsterisCrack/BipedRobot/tree/main/actuator). Bench
trajectories swing ±90°, so they are refused for the robot's own servo IDs unless forced, and a running job
owns the bus, so the policy cannot arm.

---

## Deploying a policy

### 1. Export and copy the bundle

In [BipedRobot](https://github.com/AsterisCrack/BipedRobot):

```bash
python -m src.isaaclab.export_onnx --checkpoint checkpoints/<run>/step_<N>.pt
python -m src.isaaclab.verify_onnx_bundle --bundle exported/<bundle>
python -m src.mujoco.policy_gate --bundle exported/<bundle>
```

Copy the directory here as `models/<name>/`:

```
models/<name>/
├── policy.onnx             obs [1, 250] -> action [1, 12]
├── deploy_config.json      normaliser, joint limits, action map, gait clock, limits, required PID
└── checkpoint_info.json    provenance (optional)
```

`models/walk_v1/` is an early example bundle. The Control tab lists every bundle, including invalid
ones with their error. Validation is strict and happens at load, not mid-gait.

### 2. Bundle format

`deploy_config.json` carries a `format_version`. The runtime accepts v1 to v6 and refuses anything newer.

| `action_map` | Version | Joint target for action `a` in [-1, 1] |
|---|---|---|
| `absolute` | v1 | `min + (a+1)/2 * (max-min)` |
| `centered` | v2 | `a = 0` is the default pose, `±1` reach the limits |
| `residual` | v4 | `clamp(home + scale * a, min, max)`, with `home = default_joint_pos` |
| `residual`, asymmetric | v6 | the same with `scale_pos` for `a >= 0` and `scale_neg` for `a < 0` |

The **gait clock** (`gait_clock`) is the phase input the policy reads: `speed_proportional` (v1/v2),
`fixed_period` (v3) or `cadence_grid` (v5, where the step frequency follows the command). A bundle may also
carry a `command_limit` (box or ellipse) and an `actuator` block with the PID it requires. The action map
must match training exactly, or every action is executed wrongly.

### 3. Self-test (torque off)

```bash
python3 -m policy.selftest --model <name>              # live sensors, torque stays off
python3 -m policy.selftest --model <name> --synthetic  # no hardware
```

It prints the bundle's format, action map, gait clock, home pose and command ranges. It then checks that
`projected_gravity` is a unit vector, that the specific force is about 9.81 m/s², that every joint is inside
the policy's limits, and how far the measured pose is from the bundle's home (WARN beyond 10° per joint).
Last, it runs inference and prints the targets that *would* be sent. Nothing is written to the bus. Run it
**before the first arm and after any recalibration**. It always rebuilds acceleration from the fused linear
acceleration, so it does not exercise `imu_raw_accel`.

### 4. Arm, start, drive

In the Control tab (or through the [API](#api)):

| Step | What happens |
|---|---|
| **Load** | Pick a bundle (only while idle). The ONNX graph is test-run once. |
| **Arm** | Audits the servo registers, sets every goal to the present position, turns torque on, then ramps to the bundle's home pose over about 2 s |
| **Start** | Hands control to the policy. Only valid from `armed`. |
| **Command** | Sliders, keyboard (WASD/QE) or a gamepad set `vx`, `vy`, `wz` |
| **Stop** | Back to `armed`: the policy stops, torque and pose are held |
| **Disarm** | Torque off, back to `idle` |
| **E-STOP** | Torque off from any state. It cannot fail, and the UI sends it over both REST and WebSocket. |

The states are `idle`, `arming`, `armed` and `running`. `fault` is reachable from any of them, is latched,
and is cleared only by the operator.

Arming is refused unless all of these hold:

- every servo replies, and the cached readings are fresh (under 0.15 s old);
- `BIPED_FAST_MODE` is 0 or 2 (the joint-velocity channel needs the speed register);
- the servos' own registers match training: acceleration cap cleared (the STS3215 ships with an
  undocumented cap that makes it too sluggish for a 50 Hz policy; it is also cleared at startup), P/D/I equal
  to the bundle's required PID, and torque limit 1000;
- every goal register can first be set to the present pose, so a stale goal cannot make a joint jump when
  torque comes on;
- no SysID job is using the bus.

`start` resets the runtime state the way every training episode begins: zero-filled history, gait phase 0,
previous action 0, filtered targets at the measured pose.

**Motion lock.** While the policy is arming, armed or running, every endpoint that moves or reconfigures
servos (poses, Home, IK, servo position, torque enable, PID, zero, ID, ...) answers HTTP 409 (WebSocket: an
`error` message). Torque disable, e-stop and the policy endpoints are never locked.

**Commands.** The server never trusts the client. Each command is clamped to the region the policy was
trained on (a per-axis box, or an ellipse in vx/vy from the bundle's `command_limit`), then multiplied by the
velocity scale. A non-finite command becomes zero. A streamed command (gamepad) is zeroed if it is not
refreshed for 0.5 s, so a closed tab or dropped Wi-Fi cannot leave the robot walking.

**Gamepad.** Runs in the browser (standard mapping): left stick = vx/vy, right stick X = turn,
**A** arm/start, **B** stop, **X** e-stop, **Y** toggle auto-recover, **View** disarm. Browsers may expose
gamepads only on secure origins, so over plain `http` use an SSH tunnel (`ssh -L 8080:localhost:8080 <user>@<jetson>`).

**Auto-recover** (off at boot, Control tab). After a *tilt* fault only: it waits until the robot is upright
(default within 8°) and still for 1 s, then arms, counts down 5 s and starts. Any other fault, disarm or
e-stop switches it off, and three failed arms in a row disable it. Without it, nothing ever moves on its own.

---

## How the policy loop works

[`ServoBusManager`](hardware/servo_bus_manager.py) owns the serial port in one daemon thread. Each 20 ms
cycle does a SYNC_READ of all 12 servos (one broadcast), takes the latest cached IMU sample, runs
`policy.step()` (observation, ONNX, targets) and SYNC_WRITEs the targets in the same cycle. The action
therefore takes effect with no extra cycle of delay. The IMU is read continuously over I2C by its own thread,
so the bus thread never waits on it. Most of the 20 ms budget is left for inference; the Control tab's *Loop
Health* card shows the live numbers.

### Observation frame

One 50-dimensional frame per tick. The actor sees the last 5 frames, oldest first (250 inputs), after a
two-stage normalisation (stage 1 uses `obs_mean`/`obs_var` from the bundle, stage 2 is baked into the ONNX).

| Slots | Content | Source on this robot |
|---|---|---|
| 0:3 | Accelerometer, **specific force** (gravity included, about +9.81 on z upright) | see below |
| 3:6 | Gyro, rad/s | BNO055 |
| 6:9 | Projected gravity (unit vector, about `[0, 0, -1]` upright) | BNO055 quaternion |
| 9:12 | Command `[vx, vy, wz]` | After the command limit and velocity scale |
| 12:24 | Joint position minus home pose, rad | Servo position, logical space |
| 24:36 | Joint velocity, rad/s | Servo speed register |
| 36:48 | Previous action | Last action sent, clipped |
| 48:50 | `[sin, cos]` of the gait phase | Gait clock |

Conventions the policy depends on, all handled by the runtime:

- **Joint order.** Isaac interleaves left and right; this robot is per leg, so joints are mapped **by name**.
- **Acceleration.** The policy trained on specific force, but the BNO055's linear-acceleration output has
  gravity removed, so by default it is rebuilt as `linear_accel - 9.81 * projected_gravity`. With
  `imu_raw_accel: true` the raw accelerometer is used directly. With `imu_accel_average: true` either one is
  the mean over all reads in the tick.
- **Joint velocity** comes from the servo's speed register, far finer than differencing 50 Hz positions. It is
  sign-magnitude (bit 15 is the direction), and `direction_sign` is applied after decoding.
- **IMU mounting.** This board is rotated 180° about the forward axis, so the driver negates Y and Z of
  accel, gyro and projected gravity. The IMU tab's quaternion and Euler angles still show the raw sensor
  frame. If you remount the IMU, re-check `projected_gravity` with the self-test.
- **Gait clock.** Advances once per tick, before the frame is built. For `fixed_period` and `cadence_grid`
  it runs while the command is at or above the bundle's `stand_threshold` (default 0.05) and holds
  otherwise, at a constant rate or at `freq(command)`. Phase 0 is right-foot touchdown.

### From network output to servo target

1. Clip the action to [-1, 1].
2. **Rate limiter.** Bound the change per tick: 0.5 by default, the bundle's value if it sets one, operator range 0.05 to 2.0.
3. Map to joint targets with the bundle's action map, then clamp to the bundle's joint limits.
4. **Action filter.** An exponential moving average, applied **four times per tick**. Isaac applies it
   inside its 4-step decimation loop, so the effective gain is `1 - (1-α)^4` (0.87 for α = 0.4), not α.
5. Write the targets in logical degrees. `Robot` adds `default_position_deg` and converts to servo steps.
   During policy runs the servo speed is 0 (full speed), since the filter is the rate limiter.

---

## Safety faults

Any fault **cuts torque at once**, latches `FAULT`, closes the run trace and records the reason (the first
fault wins). Clear it with `clear_fault`, then `arm`, then `start`. Nothing restarts by itself unless
auto-recover is on.

| Fault | Trigger | What to check |
|---|---|---|
| `tilt_exceeded` | Body tilt above the limit (default 45°, the training termination angle; adjustable 10 to 45°, and a bundle can only tighten it). Also checked at the end of the arm ramp. | A real fall, or a wrong IMU mount or calibration. Run the self-test with the robot standing. |
| `stale_sensor` | Oldest servo or IMU sample older than 0.15 s, or a servo that never replied | Bus power, cabling, a loose servo connector, I2C errors in the log |
| `bad_observation` | A non-finite value, or any observation value above 100 in magnitude | A unit or sign error, a corrupt read |
| `inference_error` | The ONNX call raised, or the action was non-finite | The log (the exception is printed) |
| `loop_overrun` | More than 10 consecutive cycles over the 20 ms budget | CPU load, `BIPED_RT_SCHEDULING`, other processes |
| `estop` | E-stop pressed | |

`GET /api/policy/transitions` lists recent state changes and is the first place to look after a fault. The
trace saved with the fault holds the last steps before it.

---

## Run traces

Every `RUNNING` episode, from Start until stop, fault, disarm or e-stop, is recorded at 50 Hz and saved in
the background to `data/traces/<UTC>_<model>_<reason>.npz` (git-ignored, up to 5 minutes per file). The
Control tab's *Run traces* card lists and downloads them (`GET /api/policy/traces`).

| Field | Shape | Content |
|---|---|---|
| `t` | [T] | Seconds since start |
| `frame` | [T, 50] | The observation frame the policy saw, in policy joint order |
| `action_raw`, `action` | [T, 12] | Network output, and after clip and rate limiter |
| `targets_rad` | [T, 12] | Filtered joint targets sent to the servos (logical rad, policy order) |
| `rate_limited`, `tilt_rad`, `cycle_ms`, `infer_ms` | [T] | 1 when the limiter changed the action; body tilt; step and inference time |
| `command` | [T, 3] | Velocity command |
| `servo_load`, `servo_voltage`, `servo_temp` | [T, 12] | Load (signed, 0.1 % of max, so ±1000 is full torque), supply volts and °C, in `robot.yaml` order |
| `meta` | JSON string | Bundle info, action map, home pose, scales, joint limits, gait clock, actuator block, `default_position_deg`, limits in force, end reason |

Load, voltage and temperature are only read from the bus with `BIPED_FAST_MODE=0`. In mode 2 they are
recorded as zeros, so use mode 0 when you want them. A field that is missing on a step (the fault step has
no action, load or temperature) is NaN, so a trace that ends in a fault has a NaN last row.

```python
import json, numpy as np

d = np.load("data/traces/<file>.npz")
meta = json.loads(str(d["meta"]))
home = np.array(meta["default_joint_pos"])           # rad, policy order
q = d["frame"][:, 12:24] + home                      # measured joint angles
err = d["targets_rad"][:-1] - q[1:]                  # tracking error one tick later
chatter = np.nanmean(d["rate_limited"])              # fraction of ticks the limiter fired
```

**Debugging the sim-to-real gap.** For chatter, look at the `rate_limited` fraction and whether `action_raw`
jumps between ticks, and compare `frame[:, 0:3]` (acceleration) and `frame[:, 24:36]` (joint velocity) with
the simulation's noise model. For sluggish tracking, look for a large, growing `err`, then check `servo_load`
near ±1000, `servo_voltage` sag and `servo_temp`, and re-check the register audit and the PID. For falls, read
`tilt_rad`, `command` and the last ~100 steps before the fault. To replay a trace in simulation, use
BipedRobot's `src/mujoco/policy_gate.py --trace <file>.npz`, which reads the `frame` array.

---

## Tools

Run from the repository root. **Stop the web app first**: only one process may own the serial bus.

| Tool | Purpose |
|---|---|
| [`servo_registers.py`](tools/servo_registers.py) | Audit and clear the STS3215 acceleration cap (`0x55`/`0x29`): `--clear`, `--step-test 10` |
| [`stand_record.py`](tools/stand_record.py) | Arm to the bundle's home pose **without starting the policy** and record while you tap the torso, to measure passive compliance: `--bundle models/<name> [--seconds 20]`. Aborts above 35° of tilt. |
| [`speed_register_check.py`](tools/speed_register_check.py) | Check the speed register as `joint_vel` (sign, scale, lag, dropouts): `--joints <names> [--mode backdrive\|sweep]`. Sweep drives the joint, so suspend the robot. |
| [`bus_profiler.py`](tools/bus_profiler.py) | Time the real bus loop per phase: `--cycles N --write`. Run with `sudo` so `SCHED_FIFO` applies. |
| [`return_delay.py`](tools/return_delay.py) | Read or zero the servos' `RETURN_DELAY` register, which adds dead time to every SYNC_READ: `--set-zero` |

---

## API

`http://<host>:8080`, JSON in and out. Every UI action is also an endpoint; the interactive docs are at
`/docs`.

| Prefix | Endpoints |
|---|---|
| `/api/servos` | `GET /`, `/{id}`, `/scan`; `POST /{id}/position` (`{"deg": 15.0}`), `/torque`, `/pid`, `/zero`, `/id`, `/speed`, `/accel`, `/torque_limit`; `POST /sync_positions` (`{"joints": {"l_knee_joint": 30.0}}`) |
| `/api/imu`, `/api/kinematics` | `GET /api/imu/`, `POST /api/imu/calibrate`; `GET`/`POST /fk/{leg}`, `POST /ik/{leg}` (`{"x": 0.0, "y": 0.021, "z": -0.25, "execute": true}`), `GET /poses`, `POST /poses/{name}`, `POST /home` |
| `/api/policy` | `GET /models`, `/status`, `/transitions`, `/traces[/{name}]`, `/safety`, `/auto`; `POST /load` (`{"name": "<bundle>"}`), `/arm`, `/start`, `/stop`, `/disarm`, `/estop`, `/clear_fault`, `/command` (`{"vx": 0.1, "vy": 0, "wz": 0}`), `/safety`, `/auto` |
| `/api/sysid`, `/api/config` | SysID sessions and runs; servo config and `POST /export` |

`POST /api/policy/safety` takes a tilt limit of 10 to 45°, an action rate of 0.05 to 2.0 and a velocity scale
of 0 to 1; anything outside is a 422. Example: `curl -X POST localhost:8080/api/policy/arm`.

`ws://<host>:8080/ws` pushes a `telemetry` frame at about 20 Hz (`servos`, `imu`, `kinematics` and, with
`onnxruntime`, `policy`) and accepts `set_position`, `set_torque`, `set_pid`, `set_pose`, `set_foot_ik`,
`set_joints_fk`, `set_velocity_command` and `policy_estop`.

---

## Tests

```bash
pip install pytest
python -m pytest tests -q          # no hardware needed
```

The suite covers the policy runtime (arming, action maps, command limit, gait clock, rate limits, stale
sensors, motion lock, auto-recover, traces) with a stubbed robot. The command-limit and cadence tests compare
against golden files in `tests/data/` that are shared with BipedRobot, so regenerate both copies together.

---

## Project structure

`main.py` starts the robot and the web app. `config/` holds the YAML files, `hardware/` the servo bus and IMU
library, `kinematics/` the URDF-based FK/IK, `robot/` the orchestrator (angle spaces, motion, telemetry,
SysID), `policy/` the ONNX runtime, `web/` the FastAPI app and static UI, `tools/` the diagnostics, `models/`
the policy bundles and `RobotDescription/` the URDF and meshes. Layers import only downward, so `hardware/`
and `kinematics/` can be copied into other projects.

---

## License

[MIT](LICENSE)
