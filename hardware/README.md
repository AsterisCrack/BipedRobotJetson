# hardware

Standalone Python library for **ST3215 servo bus control** and **BNO085 IMU** integration.

No dependencies on `robot/`, `kinematics/`, or `web/`. Drop this directory into any project and import directly.

---

## Modules

### `config.py` — Hardware Configuration Models

Pydantic models shared across all hardware modules.

| Class | Purpose |
|-------|---------|
| `PIDConfig` | P / D / I gain set (defaults: 32 / 16 / 0) |
| `ServoConfig` | Per-servo parameters: `servo_id`, `joint_name`, `direction_sign` (±1 validated), `zero_offset_steps`, `default_position_deg`, `pid` |
| `HardwareConfig` | Bus parameters: `uart_port`, `baud_rate`, `i2c_bus`, `i2c_address`, `uart_timeout_s`, `max_retries` |

`direction_sign` is validated at construction time — must be exactly `-1` or `1`.

---

### `serial_bus.py` — Thread-Safe Half-Duplex UART

`SerialBus` manages a single half-duplex UART where TX and RX share one wire. Every byte sent by the host appears on its own RX line (echo); the bus drains those echo bytes before reading the servo response. All transfers hold an internal `threading.Lock`.

```python
from hardware.serial_bus import SerialBus, SerialBusError

bus = SerialBus("/dev/ttyTHS1", baud_rate=1_000_000, timeout=0.05)
bus.open()
# ... use bus ...
bus.close()
# or: with SerialBus(...) as bus: ...
```

**Key methods:**

| Method | Description |
|--------|-------------|
| `transfer(packet, response_data_len)` | Send one instruction packet, return response data bytes |
| `sync_read(packet, servo_ids, data_len)` | SYNC_READ broadcast, collect one response per servo under one lock |
| `send_no_reply(packet)` | SYNC_WRITE broadcast (no response expected) |

---

### `st3215/` — ST3215 Servo Driver

Implements the SCS/Feetech serial protocol from scratch using `pyserial`.

**Packet format:**
```
0xFF 0xFF ID LEN INSTR [PARAMS...] CHECKSUM
CHECKSUM = ~(ID + LEN + INSTR + sum(PARAMS)) & 0xFF
```

#### `registers.py`
Full register map (`Reg`) and instruction set constants (`Instr`).

Notable registers:
- `Reg.TARGET_POS_L` (0x2A) — 2-byte target position
- `Reg.STATUS_START` (0x38) — 8-byte bulk status read (pos + speed + load + voltage + temp)
- `Reg.TORQUE_ENABLE` (0x28)
- `Reg.PID_P/D/I` (0x15–0x17)

#### `protocol.py`
SCS packet encoding functions:

| Function | Description |
|----------|-------------|
| `encode_ping(id)` | PING packet |
| `encode_read(id, reg, len)` | READ packet |
| `encode_write(id, reg, data)` | WRITE packet |
| `encode_sync_read(reg, len, ids)` | SYNC_READ broadcast |
| `encode_sync_write(reg, len, servo_data)` | SYNC_WRITE broadcast |
| `pack_u16` / `unpack_u16` | Little-endian 16-bit helpers |
| `steps_to_bytes` / `bytes_to_steps` | 12-bit position encoding |

#### `servo.py`
`ST3215` per-servo driver. Unit conversion:

```
steps = zero_offset_steps + direction_sign × angle_deg × (4096 / 360)
```

**Public API:**

```python
from hardware.config import ServoConfig
from hardware.serial_bus import SerialBus
from hardware.st3215.servo import ST3215

bus = SerialBus("/dev/ttyTHS1", 1_000_000)
bus.open()

cfg = ServoConfig(servo_id=1, joint_name="l_hip_yaw", direction_sign=-1, zero_offset_steps=2048)
servo = ST3215(cfg, bus)

servo.ping()                    # → bool
servo.get_position()            # → float (degrees, URDF space)
servo.set_position(45.0, speed=300)
servo.enable_torque()
servo.disable_torque()
servo.set_pid(32, 16, 0)
servo.get_status()              # → ServoStatus dataclass

# Public properties (no _cfg access needed)
servo.zero_offset_steps         # int (readable + settable)
servo.direction_sign            # int
servo.default_position_deg      # float (readable + settable)
```

---

### `servo_bus_manager.py` — 50 Hz Read/Write Loop

`ServoBusManager` runs a dedicated daemon thread at ~50 Hz. Each cycle:

1. **SYNC_READ** all servo positions, speeds, loads, voltages, temperatures
2. **Read IMU**
3. Atomically update the thread-safe state cache under `_state_lock`
4. Consume the latest pending position command (latest-wins per joint)
5. **SYNC_WRITE** if a command is pending

This design ensures all servo state updates are atomic and external code never touches the bus directly for position I/O.

```python
from hardware.servo_bus_manager import ServoBusManager

manager = ServoBusManager(servos, bus, imu)
manager.start()

# Non-blocking writes — latest-wins per joint
manager.set_target_positions({"l_hip_yaw": 47.0, "l_knee_joint": -20.0}, speed=300)

# Non-blocking reads — returns copy of last cached state
states = manager.get_servo_states()    # list[ServoStatus]
imu    = manager.get_imu_state()       # IMUReading
cache  = manager.get_cached_positions()  # dict[joint_name → urdf_deg]

print(f"Bus running at {manager.cycle_hz:.1f} Hz")

manager.stop()
```

---

### `imu/bno085.py` — BNO085 Quaternion/Euler Driver

Wraps the Adafruit CircuitPython BNO08x library. Returns an `IMUReading` dataclass with:

| Field | Type | Description |
|-------|------|-------------|
| `quaternion` | `(w, x, y, z)` | Unit quaternion |
| `euler_deg` | `(roll, pitch, yaw)` | Euler angles in degrees |
| `accel` | `(x, y, z)` | Acceleration in m/s² |
| `gyro` | `(x, y, z)` | Angular velocity in rad/s |
| `calibration_status` | `int` | 0 (uncalibrated) – 3 (fully calibrated) |

If hardware is unavailable, `read()` returns the last valid reading (identity quaternion on startup).

**Requires** `adafruit-blinka` and `adafruit-circuitpython-bno08x` (Jetson only).

```python
from hardware.imu.bno085 import BNO085

imu = BNO085(i2c_bus=7, address=0x4A)
imu.initialize()
reading = imu.read()
print(reading.euler_deg)   # (roll, pitch, yaw) in degrees
imu.close()
```

---

## Minimal Standalone Example

```python
from hardware.config import HardwareConfig, ServoConfig
from hardware.serial_bus import SerialBus
from hardware.st3215.servo import ST3215

hw = HardwareConfig(uart_port="/dev/ttyTHS1", baud_rate=1_000_000)
bus = SerialBus(hw.uart_port, hw.baud_rate, hw.uart_timeout_s)
bus.open()

cfg = ServoConfig(servo_id=1, joint_name="l_hip_yaw", direction_sign=-1, zero_offset_steps=2048)
servo = ST3215(cfg, bus)

if servo.ping():
    print("Position:", servo.get_position(), "°")
    servo.enable_torque()
    servo.set_position(45.0, speed=300)
    servo.disable_torque()

bus.close()
```

## 50 Hz Loop Example

```python
from hardware.config import HardwareConfig, ServoConfig
from hardware.serial_bus import SerialBus
from hardware.servo_bus_manager import ServoBusManager
from hardware.st3215.servo import ST3215
from hardware.imu.bno085 import BNO085

hw = HardwareConfig(uart_port="/dev/ttyTHS1", baud_rate=1_000_000)
bus = SerialBus(hw.uart_port, hw.baud_rate, hw.uart_timeout_s)
bus.open()

configs = [
    ServoConfig(servo_id=1, joint_name="l_hip_yaw",    direction_sign=-1, zero_offset_steps=2048),
    ServoConfig(servo_id=2, joint_name="l_hip_roll",   direction_sign= 1, zero_offset_steps=2048),
    # ... all servos
]
servos = [ST3215(cfg, bus) for cfg in configs]
imu = BNO085(i2c_bus=7)

manager = ServoBusManager(servos, bus, imu)
manager.start()

# In your control loop (any thread):
while True:
    states = manager.get_cached_positions()  # fast, non-blocking
    imu_data = manager.get_imu_state()
    # ... compute new targets ...
    manager.set_target_positions({"l_hip_yaw": 10.0}, speed=300)

manager.stop()
bus.close()
```
