# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Running the server

```bash
python3 main.py          # starts FastAPI + WebSocket on http://localhost:8080
```

No build step. Three.js and urdf-loader load from CDN at runtime. `workers=1` is intentional — the serial bus cannot be shared across processes.

**Without hardware** (dev/simulation): skip `requirements-hardware.txt`. The server starts in degraded mode — servo reads return zeros, IMU returns a static quaternion. All UI and kinematics work normally.

**On Jetson** only:
```bash
pip3 install -r requirements-hardware.txt   # adafruit-blinka + Jetson.GPIO
```

## Repository Structure & Dependency Layers

```
hardware/    ← standalone servo bus + IMU library (no project imports)
kinematics/  ← standalone FK/IK library (no project imports)
robot/       ← orchestrator (imports hardware/ and kinematics/)
web/         ← debug UI (imports robot/ only)
```

`hardware/` and `kinematics/` have their own `README.md` files with usage examples. They can be used without the robot orchestrator.

**Import rule:** Never add imports that go against the dependency arrows. `web/` must not import from `hardware/` or `kinematics/` directly — always go through `robot/robot.py`'s public API.

## Configuration

Two config layers, merged at startup by `robot/config.py:Settings.load()`:

- `.env` — overrides `UART_PORT` and `I2C_BUS` (takes highest priority)
- `config/hardware.yaml` — UART/I2C settings
- `config/robot.yaml` — servo IDs, `zero_offset_steps`, `direction_sign`, `default_position_deg`, PID gains, named poses

**`zero_offset_steps`**: raw encoder value (0–4095) when the joint is at 0° in URDF space.  
**`direction_sign`**: ±1 — maps physical servo rotation to URDF joint convention. Validated at load time: must be exactly -1 or 1.  
**`default_position_deg`**: the robot's calibrated standing angle for that joint; defines logical zero (see coordinate spaces below).

Named poses under `poses:` are in **logical space** (0 = default standing position). The `home_pose` key points to which pose the Home button executes.

## Architecture

### Coordinate spaces

Two angle spaces coexist — be explicit about which one you're working in:

| Space | Definition | Used by |
|-------|-----------|---------|
| **URDF space** | 0° = physical servo centre after `zero_offset_steps` + `direction_sign` | FK/IK solver, servo driver internals, Debug tab |
| **Logical space** | 0° = calibrated standing position (`default_position_deg` offset) | Poses, Servos tab sliders, Robot tab FK/IK sliders, telemetry display |

Conversion: `logical = urdf − default_position_deg`

`Robot.sync_write_positions(angles, raw=False)` applies the offset (logical → URDF) unless `raw=True`. The telemetry frame exposes both `position_deg` (URDF, for the 3D model) and `logical_deg` (for sliders).

Use `robot.urdf_to_logical(leg, angles)` and `robot.logical_to_urdf(leg, angles)` for conversions in `web/` — never access `robot._default_offsets` directly.

### Bus Manager (50 Hz loop)

`hardware/servo_bus_manager.py:ServoBusManager` owns the serial bus exclusively in a daemon thread. Every cycle (~20 ms):

1. **SYNC_READ** (SCS instruction `0x82`) — one broadcast packet, 12 sequential responses; all servo states updated atomically
2. Get latest IMU reading from `_IMUReaderThread` (~2 µs cache read — no I2C blocking)
3. Update thread-safe state buffer
4. Policy hook (RL inference), which queues its command
5. Consume latest-wins command from command buffer
6. **SYNC_WRITE** (SCS instruction `0x83`) if a command is pending

The write comes **after** the policy hook so the action goes out in the cycle it was computed
(~3 ms after the read). Until 2026-10-12 the write came first, so every action landed one full
cycle (~23 ms) late. The policy never trained with that delay, and it made the robot shake and fall.

The IMU runs in its own daemon thread (`_IMUReaderThread`) that reads BNO055 continuously over I2C in parallel with UART. The bus thread reads a cached value, removing the 4 ms I2C latency from the servo cycle.

External code never touches the bus directly for position I/O. Interaction is non-blocking:
- **Write**: `bus_manager.set_target_positions(urdf_angles, speed)` — merges into pending dict (latest-wins per joint)
- **Read**: `bus_manager.get_servo_states()` / `get_cached_positions()` / `get_imu_state()` — returns a copy of the last state
- **RL**: `bus_manager.get_rl_state()` — one-shot dict with positions, velocities, velocities_deg_s, linear_accel, angular_vel, projected_gravity

The bus lock in `SerialBus` still protects infrequent direct bus calls (ping, PID write, torque toggle, scan) from racing with the manager thread.

**Exclusive jobs.** `bus_manager.run_exclusive(job)` runs `job.run(bus)` *on the bus thread*
between normal cycles, so the bus keeps a single owner. The SysID recorder uses it. While a job
runs there is no SYNC_READ of all servos, no policy hook and no queued SYNC_WRITE, and
commands queued meanwhile are **dropped** rather than replayed. Inside a job, never use
`transfer()`-based calls in a loop: `flush()` costs ~10 ms on L4T. Use one-id `sync_read` and
SYNC_WRITE (broadcast, so no reply ends up in the RX buffer).

**Goal block layout.** `0x2A` is position(2) | **time**(2) | speed(2), matching Feetech SDK
`GOAL_POSITION` 42, `GOAL_TIME` 44, `GOAL_SPEED` 46. Always write it with
`st3215.servo.goal_bytes()` and `GOAL_BLOCK_LEN = 6`. Until 2026-10-02 every write was 4
bytes, so "speed" landed in goal time and speed was never set.

**Acceleration cap.** The STS3215 ships with an undocumented cap in `0x55` (factory 50) that
gates `0x29`. At ~10–28 rad/s² it is far below what a 50 Hz policy needs. Both registers are
RAM, so `Robot.initialize()` clears them at every startup, 0x55 first.

### Request → hardware flow

```
WebSocket command (set_foot_ik)
  → websocket.py: run_in_executor(robot.set_foot_position)   # IK off event loop
      → robot.py: _get_current_urdf_angles()                  # reads BusManager cache
      → solver.py: ik() [DLS]                                  # URDF space
      → robot.py: sync_write_positions(urdf_angles, raw=True) # skips offset layer
          → bus_manager.set_target_positions()                 # non-blocking enqueue
              → BusManager thread: _sync_write()               # next 20ms cycle

Telemetry (20 Hz, separate thread)
  → robot.py: _telemetry_loop()
      → bus_manager.get_servo_states()    # non-blocking cache read
      → bus_manager.get_imu_state()
      → asyncio.run_coroutine_threadsafe(queue.put(frame))
          → websocket.py: broadcast_loop() → WebSocket clients
```

### Key files

| File | Role |
|------|------|
| `hardware/servo_bus_manager.py` | 50 Hz bus thread, `_IMUReaderThread`, SYNC_READ/SYNC_WRITE, RL state API |
| `hardware/serial_bus.py` | Thread-safe half-duplex UART; `transfer()`, `sync_read()`, `send_no_reply()` |
| `hardware/st3215/protocol.py` | SCS packet encoding: `encode_sync_read`, `encode_sync_write`, checksum |
| `hardware/st3215/registers.py` | Full ST3215 register + instruction map (`Reg`, `Instr`) |
| `hardware/st3215/servo.py` | Per-servo driver: `deg_to_steps`, `steps_to_deg`, `get_status` |
| `hardware/imu/bno055.py` | BNO055 IMU driver: single-bulk-read, quaternion → Euler |
| `hardware/config.py` | `PIDConfig`, `ServoConfig`, `HardwareConfig` (Pydantic) |
| `kinematics/chain.py` | URDF parser → 6-joint kinematic chain, FK via Rodrigues transforms |
| `kinematics/solver.py` | `fk()` and `ik()` (DLS — Damped Least Squares, warm-start from current angles) |
| `robot/robot.py` | Orchestrator: offset layer, motion commands, telemetry loop |
| `robot/config.py` | Pydantic `Settings` — merges `.env` + both YAMLs |
| `web/websocket.py` | WS dispatcher: IK in executor, uses `robot.urdf_to_logical()` / `robot.logical_to_urdf()` |
| `web/app.py` | FastAPI factory, lifespan (starts telemetry, mounts routes) |
| `web/guards.py` | Motion guard: 409 / WS error for servo-moving endpoints while the policy is armed or running |
| `tools/bus_profiler.py` | Bus timing profiler — real `ServoBusManager` harness, per-phase stats + histogram |
| `tools/return_delay.py` | Read/zero the `RETURN_DELAY` EEPROM register on all servos |
| `tools/servo_registers.py` | Audit/clear the acceleration cap (0x55/0x29), audit torque ceilings, step-response test |
| `hardware/sysid.py` | High-rate single-servo recorder + the four BAM trajectories (runs on the bus thread) |
| `robot/sysid.py` | SysID sessions: hanging zero, BAM-format recordings, 12-joint hysteresis screen |
| `web/routers/sysid.py` + `web/static/js/sysid.js` | SysID tab |

### Half-duplex UART protocol notes

The ST3215 bus is half-duplex — TX bytes loop back on RX. `SerialBus` drains `len(packet)` echo bytes after every write before reading the servo response. `sync_read()` holds the lock for the full transaction (one request + N responses) to prevent interleaving.

SCS packet: `0xFF 0xFF ID LEN INSTR [PARAMS...] CHECKSUM`  
Checksum: `~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF`

### IK

IK is numerical (DLS — Damped Least Squares), not analytical — the hip offset geometry prevents closed-form solution. Always warm-started from current URDF-space joint angles; retries from standing pose if the warm-start fails. Returns angles in URDF space; `websocket.py` converts to logical before sending to the UI via `robot.urdf_to_logical()`.

## Adding a new pose

Add it to `config/robot.yaml` under `poses:` with joint angles in **logical space** (0 = default standing position for each joint). The UI picks it up automatically via `/api/kinematics/poses`.

## RL policy (implemented)

`policy/` runs an exported ONNX actor inside the bus thread. Layering: it imports
`hardware/` and `robot/`; only `web/` and `robot/` import it.

| File | Role |
|------|------|
| `policy/deploy_config.py` | Loads/validates an exported bundle |
| `policy/observation.py` | Pure numpy: joint permutation, 50-dim frame, history ring, gait clock |
| `policy/cadence.py` | Pure numpy: cadence-grid gait clock (v5), trilinear freq(cmd) + validation |
| `policy/policy_runner.py` | ONNX session, EMA filter, arming ramp; `step()` called per bus cycle |
| `policy/safety.py` | Tilt fault, action-rate limit, observation sanity, stale-sensor age (`sensor_age`) |
| `policy/state_machine.py` | `IDLE → ARMING → ARMED → RUNNING → FAULT` |
| `policy/autopilot.py` | Opt-in auto-recover after a fall (own thread): upright → re-arm → countdown → start |
| `policy/selftest.py` | Dry run with torque off — **run before first arm** |

**Stale sensors**: `get_rl_state()` returns `servo_stamp` (oldest per-servo reply, `time.monotonic()`),
`servo_missing` (ids that never replied) and `imu_stamp` (last good I2C read; the cached fallback keeps its old stamp).
RUNNING faults `stale_sensor` if the oldest is > `max_sensor_age_s` (0.15 s, ~7 missed 50 Hz cycles) or any servo is missing;
`arm()`/`start()` refuse while a servo is missing.

### Bundle

Produced by `BipedRobot/src/isaaclab/export_onnx.py`, dropped in `models/<name>/`:
`policy.onnx` + `deploy_config.json` + `checkpoint_info.json`. Nothing from the
training repo is imported at runtime.

### Things that will silently break it

**Joint order differs.** Isaac interleaves L/R (`l_hip_yaw, r_hip_yaw, l_hip_roll, …`);
this robot is per-leg (all left, then all right). `JointMapper` permutes **by name** —
never index into `get_rl_state()["positions"]` positionally, it is ordered by
`config/robot.yaml` and that is not an enforced invariant.

**Two normalizers.** Stage 1 (`obs_mean`/`obs_var` in `deploy_config.json`) is applied
by the runtime; stage 2 is baked into the ONNX. Both are trained and non-identity.

**The BNO055 `accel` field is gravity-free.** The policy trained on specific force
(gravity included, ~+9.81 z upright). `specific_force_from_linear_accel()` reconstructs
it as `linear_accel − 9.81 × projected_gravity`.

**The EMA filter runs 4× per control step**, not once — Isaac applies it inside its
decimation loop, so effective α is `1 − 0.6⁴ = 0.87`, not 0.4.

**Joint velocity comes from the servo's speed register** (`CURRENT_SPEED`, 0x3A), in steps/s —
1 LSB = 0.00153 rad/s, ~50× finer than differencing position at 50 Hz. Two traps:

- It is **sign-magnitude, bit 15 = direction**, not two's complement. Decode with
  `unpack_sign_magnitude16`, matching Feetech's own `sts_tohost(v, 15)`. Reading it unsigned
  turns −100 steps/s into +50.4 rad/s. `CURRENT_LOAD` has the same encoding.
- `direction_sign` must be applied, exactly as `steps_to_deg` does for position. Use
  `get_rl_state()["velocities_deg_s"]`, which has already done both.

Needs `BIPED_FAST_MODE` 0 or 2 — mode 1 omits the speed bytes, so the channel would read a
silent, plausible zero. `PolicyRunner.arm()` and `policy.selftest` both refuse to run under it
(check `ServoBusManager.has_velocity`, don't compare the mode integer). **Mode 2 is the one to
run the policy under**: pos+speed only, 480 µs/cycle cheaper than the full status block.

### IMU mounting correction — settled, do not re-derive

This robot's BNO055 is physically mounted rotated **180° about the body's forward (X) axis**
relative to the URDF/policy convention (identity == chip axes == body axes: X forward, Y left,
Z up). Confirmed empirically 2026-09-30 from three static `projected_gravity` readings (level,
~45° forward tilt, ~45° side tilt, torque off) — all three matched the expected physical
pattern only after negating Y and Z. See `hardware/imu/bno055.py`'s `_MOUNT_FLIP_YZ` comment
for the numbers.

Fixed at the source: `accel`/`gyro` are corrected in `bno055.py::read()`;
`projected_gravity` (derived independently from the quaternion) is corrected in
`ServoBusManager.get_rl_state()`. **The raw `quaternion`/`euler_deg` fields are NOT
corrected** — that needs a verified quaternion composition, and nobody has captured a raw
quaternion sample to verify one against yet. The IMU tab's orientation readout therefore still
reflects the raw sensor frame; everything the RL policy consumes (accel, gyro,
projected_gravity) is correct.

If the IMU is ever re-mounted, re-derive this with `policy.selftest`'s live `projected_gravity`
output at those same three poses before trusting anything past it.

### Bundle format v2: action map and servo model

`deploy_config.json` format v2 adds `action_map` and `actuator`. **The map must match training
exactly**, or every action executes wrongly:

- `absolute` (v1, e.g. `walk_v1`): `target = min + (a+1)/2·(max−min)`, so `a = 0` is the limit
  midpoint.
- `centered` (v2): `a = 0` is the default pose, `a = ±1` reach each limit.
- `residual` (v4): `target = clamp(home + action_scale_rad·a, min, max)` with `home =
  default_joint_pos` (a crouch) and a per-joint `action_scale_rad`. Requires `format_version >= 4`.

`deploy_config.py` refuses an unknown map or a newer format: **v5 is the newest and the runtime
refuses anything newer than v5** (see below). A v2 bundle's
`actuator.requires_pid` (P-only: the BAM servo model has no D) is checked against
`config/robot.yaml` in `PolicyRunner.arm()`, which is why every servo's PID is
`{p: 32, d: 0, i: 0}`. `arm()` also refuses while a SysID job owns the bus, and both e-stop
paths abort SysID.

### Bundle format v3: fixed-period gait clock (gait-table policies)

Policies trained with Gait Studio imitation (`BipedRobot/config/config_gait_reference.yaml`) read
the gait clock as an observation, `[sin, cos]` of the phase in `obs[48:50]`, and it runs
differently from the older policies. `deploy_config.json` carries it in `gait_clock`:

| mode | advance per control step | bundle |
|---|---|---|
| `speed_proportional` | `step_dt · 2π · freq · |v_xy|` (a zero command holds it) | v1, v2 |
| `fixed_period` | `step_dt · 2π / period`, **only while** `|v_xy| ≥ stand_threshold` or `|wz| ≥ stand_threshold`, otherwise held | v3 |
| `cadence_grid` | `step_dt · 2π · freq(cmd)`, held the same way (see v5 below) | v5 |

`ObservationBuilder._advance_phase` does this once per control step, before the frame is built,
which is the order training uses. The rate does not depend on speed. `load_bundle` refuses a v3
bundle with no `gait_clock`, an unknown mode, or an implausible period (0.2–5 s), and refuses
anything newer than v5: a runtime that fell back to the old clock would hand the policy a phase it
never saw. Phase 0 = right-foot touchdown, `[0, 0.5)` right stance. The runtime starts at phase 0
on `start()`; training started at a random phase, so any start phase is in-distribution.

The command is the one `PolicyRunner` hands the policy, i.e. already clamped to
`command_ranges` and multiplied by `velocity_scale`, so a small velocity scale can drop it below
`stand_threshold` and hold the clock. That matches what the policy sees as its command input.
`command_ranges` now comes from the run's own config (gait-table runs: vx ±0.2, vy ±0.15, wz ±1.2).
`selftest` prints the clock mode and ranges first; check them against the run before arming.

### Bundle format v5: cadence-grid gait clock (cadence follows the command)

Gait Studio's v2 gait tables follow human gait scaling: the cycle length depends on the command
(about 0.9 s at 0.2 m/s and 1.8 s at 0.05 m/s for this leg; turning quickens it so one step never
turns more than the style's `max_step_yaw`). The clock then runs at `freq(cmd)` cycles/s,
interpolated **trilinearly in (vx, vy, wz) with each axis clamped to the grid**, the same rule as
training (Isaac `GaitTableReference.frequency`) and sim2sim (MuJoCo `CadenceGrid`).
`gait_clock = {"mode": "cadence_grid", "grid_vx", "grid_vy", "grid_wz", "freq": [vx][vy][wz] Hz,
"stand_threshold", "period" (zero-command cycle, display only), "table", "style_digest", ...}`.

`policy/cadence.py` holds the interpolator and its validation: strictly increasing axes, freq
shaped to the grid, every rate inside 0.2–5 Hz. `load_bundle` refuses `cadence_grid` below format
v5. A v5 bundle may also carry the v4 residual map. A v2 table with a constant cadence (a legacy
fixed-period style) still exports as `fixed_period` (v3/v4). The interpolator is checked against
`tests/data/cadence_grid_golden.json`, which is a copy of `BipedRobot/tests/data/` (regenerate both
together).

### Bundle format v4: residual action map, limits from the bundle

v4 adds `action_map: "residual"` (above); `default_joint_pos` is then the **home pose** and
`obs[12:24] = joint_pos − default_joint_pos`. The arming ramp goes to the bundle home
(`_step_arming`), not to URDF zero. Training no longer uses an in-env action-rate limiter, so
`max_action_rate` is 0/absent in current bundles.

Runtime limits are derived from the bundle on every `load()`, from the **defaults** (never from
the previous value): action-rate clip = bundle `max_action_rate` only if > 0 (clamped to 0.05..2.0), else
`DEFAULT_MAX_ACTION_RATE` 0.5; tilt fault = `min(bundle tilt_fault_rad, 0.784)` (only ever
tightened). Operator limits (`POST /api/policy/safety`) are range-checked: tilt 10–45°, action rate
0.05–2.0, velocity scale 0–1 (pydantic 422, and `PolicyRunner.set_limits` raises ValueError).
`status()` returns `limits`, `format_version`, `action_map`, `gait_clock`, `home_deg`, which the
Control tab uses to initialise its sliders and show the one-line model summary.

### Auto-recover and gamepad

`PolicyRunner.auto` (`policy/autopilot.py`) is **off at boot** and is toggled in the Control tab
(`GET/POST /api/policy/auto`). It recovers only a `tilt_exceeded` fault. While the robot lies there,
it waits until the robot is upright (`< upright_deg`, gyro norm `< 0.6 rad/s`, sensors fresh) for
`settle_s`. Then it runs clear → `attach_policy_hook` → `arm()`, and after the ramp counts down
`countdown_s` in ARMED before `start()`. Tipping past the tilt limit during the countdown faults
again. Any other fault, `disarm()` or `estop()` disables it (both call `auto.disable()` first). An
ARMED reached by hand, or left by an operator Stop, is never auto-started. Three failed arms in a row
disable it. It runs on its own thread, never the bus thread (arm does blocking bus I/O). It never
calls the runner while holding its own lock (lock order is `runner._lock` → auto lock), and the fault
clear re-checks under `runner._lock`, so an e-stop between decision and action always wins.

The gamepad runs in the **browser** (`web/static/js/gamepad.js`, Gamepad API, standard mapping):
- Sticks: LS = vx/vy, RS-X = wz, scaled by the Teleop card's max-speed sliders. Those sliders and the
  command sliders are bounded by `status().command_ranges`.
- Buttons: A arm/start, B stop, X e-stop (works even with driving off), Y toggles auto, View disarms.

It sends `set_velocity_command` with `stream: true` every 50 ms. `set_command(stream=True)` arms a
deadman: `step()` zeroes a streamed command not refreshed within `COMMAND_STREAM_TIMEOUT_S` (0.5 s),
which covers a closed tab or a dropped WiFi link. Slider and keyboard commands are non-stream and
hold. Browsers may expose gamepads only in a secure context, so over plain `http://<jetson>:8080`
use an SSH tunnel to `http://localhost:8080`.

### Motion guard (web)

While the policy is ARMING / ARMED / RUNNING (`StateMachine.is_moving()`), `Robot.policy_motion_locked()`
is true (also while `arm()` is mid-sequence, via `PolicyRunner.arming_in_progress`: the state is
still IDLE during the goal preload and torque enable) and `web/guards.py` makes the endpoints that move or reconfigure servos return **HTTP 409**
(WebSocket: `{"type":"error","cmd":...}`): `POST /api/kinematics/poses/{name}`, `/home`, `/ik/{leg}`
(execute), the servo `position`, `raw_position`, torque **enable**, `pid`, `zero`, `zero_offset`,
`direction_sign`, `default_position`, `id`, `speed`, `accel`, `torque_limit`, `sync_positions`,
`simulation` endpoints, and WS `set_position`, `set_pose`, `set_foot_ik`, `set_joints_fk`, `set_pid`, `set_torque`
enable. Torque **disable** is a safety action and is never gated, nor are the policy endpoints. The
policy's own writes go through `Robot.sync_write_positions` from the bus thread and are not gated
(the guard lives in `web/`, not in `Robot`). IDLE and FAULT are not locked. New web endpoints that
move servos must call `require_motion_unlocked(robot)`.

### Arm-time register audit and run traces

`PolicyRunner.arm()` first reads back, from the servos themselves, the RAM registers the policy's
servo model depends on (`_audit_servo_registers`):
- **Acceleration cap 0x55/0x29:** cleared again if it has come back (power-up order, a brown-out).
- **P/D/I:** must equal the bundle's `requires_pid`.
- **TORQUE_LIMIT 0x30:** must be 1000.

Arming is refused otherwise. The audit runs outside the runner lock, so the bus thread never stalls
behind it.

After the audit, and **before** `enable_all_torques()`, `arm()` calls
`Robot.preload_goal_positions()`: every servo's goal = its present position at `ARM_SPEED`, written
directly from the request thread (bus lock per transfer, like the audit). Otherwise the goal
registers may hold a stale goal (web Home at speed 300, or the last policy target at `RUN_SPEED` 0
= full speed) and torque-on would make the servo chase it. If any write fails the arm is refused
with torque still off (also refused if the cached positions are older than `max_sensor_age_s`). The same targets are also queued on the bus manager so a stale queued
command cannot overwrite them.

Every RUNNING episode is recorded at 50 Hz by `policy/trace.py`, from Start until stop, fault,
disarm or e-stop. It is saved on a background thread to `data/traces/*.npz` (gitignored). The
Control tab's *Run traces* card lists them for download (`GET /api/policy/traces`,
`/api/policy/traces/{name}`). Format is in the `trace.py` docstring. `meta` (built by
`PolicyRunner._trace_meta`) is what is needed to interpret a trace offline (re-running the policy on it
still needs the bundle's ONNX graph and obs normalisation): format version,
action map, `default_joint_pos`, `action_scale_rad`, joint limits (policy order, rad), history size,
control Hz, EMA alpha/applications, gait clock, `default_position_deg` per joint from `robot.yaml`,
the actuator block and the limits in force.

`start()` (not arming) resets runtime state like a training episode: `ObservationBuilder.reset()`
zero-fills the history, phase 0, previous action 0, filtered targets at the measured pose. Arming
does not touch the history; `policy.selftest` seeds its history the same way (zero-fill).

### SysID (servo identification for the training sim)

The SysID tab records the servo for Rhoban BAM; fitting and validation happen in
`BipedRobot/actuator/` (see its README). Session = one mounting of one servo. Bench
trajectories swing ±90° about a captured hanging zero, so they are refused for robot-joint
servo ids unless forced. Recordings are BAM raw JSON in `data/sysid/<session>/`, with
sidecars in `meta/` (kept out of the dir `bam.process` globs). `data/sysid/loads.json` is a
copy of `BipedRobot/actuator/rig/loads.json`. *Screen all joints* sweeps each joint ±10°
in situ and reports hysteresis, which sets the training backlash range.

### Calibration prerequisite

The policy's home pose is the bundle's `default_joint_pos`, not URDF zero: a residual (v4) bundle
has a crouched home, centered/absolute bundles have home 0. Policy space is **logical**: a joint's
policy angle is `radians(urdf_deg − robot.yaml default_position_deg)`, so `default_position_deg` must be
right for every joint (`config/robot.yaml` has e.g. `l_hip_yaw = −33.66°`, `l_knee_joint = +134.38°` as
mechanical offsets). If it is wrong, observations and targets are shifted and part of the commanded
range is clamped. Verify with `python3 -m policy.selftest --model <name>` before arming; it prints the
bundle home and the measured pose against it (WARN beyond 10° per joint).

## RL integration point (low-level)

Use `get_rl_state()` for a single-call observation snapshot:

```python
obs = bus_manager.get_rl_state()
# obs keys:
#   positions         list[float]               servo deg in config order (URDF space)
#   velocities        list[int]                 signed speed counts (0 if BIPED_FAST_MODE=1)
#   velocities_deg_s  list[float]               URDF deg/s — what the policy wants
#   linear_accel      tuple[float, float, float] body-frame m/s²
#   angular_vel       tuple[float, float, float] body-frame rad/s
#   projected_gravity tuple[float, float, float] world [0,0,-1] rotated into body frame
```

Or compose individually at 50 Hz:

```python
urdf_angles = bus_manager.get_cached_positions()   # fast, non-blocking
imu = bus_manager.get_imu_state()
# → run NN policy inference here
bus_manager.set_target_positions(policy_output_urdf, speed=0)  # non-blocking
```

Policy inputs/outputs are in **logical** space, not URDF space: `JointMapper` converts with `radians(urdf_deg − robot.yaml default_position_deg)` on the way in and `Robot.sync_write_positions(raw=False)` adds the offset back on the way out. Inside the policy, `joint_pos_rel = q − bundle default_joint_pos` (the bundle home, 0 for centered/absolute bundles).

Or use the standalone `kinematics/` library directly alongside your own hardware access — it has no dependencies on `robot/` or `web/`.
