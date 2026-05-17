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
4. Consume latest-wins command from command buffer
5. **SYNC_WRITE** (SCS instruction `0x83`) if a command is pending

The IMU runs in its own daemon thread (`_IMUReaderThread`) that reads BNO055 continuously over I2C in parallel with UART. The bus thread reads a cached value, removing the 4 ms I2C latency from the servo cycle.

External code never touches the bus directly for position I/O. Interaction is non-blocking:
- **Write**: `bus_manager.set_target_positions(urdf_angles, speed)` — merges into pending dict (latest-wins per joint)
- **Read**: `bus_manager.get_servo_states()` / `get_cached_positions()` / `get_imu_state()` — returns a copy of the last state
- **RL**: `bus_manager.get_rl_state()` — one-shot dict with positions, velocities, linear_accel, angular_vel, projected_gravity

The bus lock in `SerialBus` still protects infrequent direct bus calls (ping, PID write, torque toggle, scan) from racing with the manager thread.

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
| `tools/bus_profiler.py` | Bus timing profiler — real `ServoBusManager` harness, per-phase stats + histogram |
| `tools/return_delay.py` | Read/zero the `RETURN_DELAY` EEPROM register on all servos |

### Half-duplex UART protocol notes

The ST3215 bus is half-duplex — TX bytes loop back on RX. `SerialBus` drains `len(packet)` echo bytes after every write before reading the servo response. `sync_read()` holds the lock for the full transaction (one request + N responses) to prevent interleaving.

SCS packet: `0xFF 0xFF ID LEN INSTR [PARAMS...] CHECKSUM`  
Checksum: `~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF`

### IK

IK is numerical (DLS — Damped Least Squares), not analytical — the hip offset geometry prevents closed-form solution. Always warm-started from current URDF-space joint angles; retries from standing pose if the warm-start fails. Returns angles in URDF space; `websocket.py` converts to logical before sending to the UI via `robot.urdf_to_logical()`.

## Adding a new pose

Add it to `config/robot.yaml` under `poses:` with joint angles in **logical space** (0 = default standing position for each joint). The UI picks it up automatically via `/api/kinematics/poses`.

## RL integration point

Use `get_rl_state()` for a single-call observation snapshot:

```python
obs = bus_manager.get_rl_state()
# obs keys:
#   positions         list[float]               servo deg in config order (URDF space)
#   velocities        list[int]                 servo speed counts (0 in BIPED_FAST_MODE)
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

Policy inputs/outputs are in URDF space. `default_position_deg` can normalise inputs (subtract) and denormalise outputs (add) at the policy boundary.

Or use the standalone `kinematics/` library directly alongside your own hardware access — it has no dependencies on `robot/` or `web/`.
