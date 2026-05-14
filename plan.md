# Plan: Robot Codebase Cleanup, Bug Fixes & Library Separation

## Context

The user wants a stable, reliable codebase for their biped robot. Three goals:
1. **Fix all identified bugs** — correctness and robustness issues in hardware, kinematics, and web layers
2. **Promote `robot/hardware/` and `robot/kinematics/` to top-level standalone libraries** — cleanly separated from the web debug UI and the robot orchestrator, with their own READMEs
3. **Decouple `web/` from robot internals** — web layer should only call `robot.Robot`'s public API, never access private fields or import from `hardware/` or `kinematics/` directly

---

## New Directory Structure

```
BipedRobotJetson/
├── main.py                        (unchanged)
├── hardware/                      ← NEW (was robot/hardware/)
│   ├── __init__.py
│   ├── README.md                  ← NEW
│   ├── config.py                  ← NEW (PIDConfig, ServoConfig, HardwareConfig)
│   ├── serial_bus.py
│   ├── servo_bus_manager.py
│   ├── st3215/
│   │   ├── __init__.py
│   │   ├── protocol.py
│   │   ├── registers.py
│   │   └── servo.py
│   └── imu/
│       ├── __init__.py
│       └── bno085.py
├── kinematics/                    ← NEW (was robot/kinematics/)
│   ├── __init__.py
│   ├── README.md                  ← NEW
│   ├── chain.py
│   └── solver.py
├── robot/
│   ├── __init__.py                ← re-exports SerialBusError, ServoStatus for web layer
│   ├── config.py                  ← RobotConfig + Settings only; re-exports hardware.config types
│   └── robot.py                   ← orchestrator; new public API methods
└── web/                           ← debug UI only; no private field access
    ├── app.py
    ├── websocket.py
    └── routers/
        ├── servos.py
        ├── imu.py
        └── kinematics.py
```

**Dependency rules (must hold after refactor):**
- `hardware/` → no imports from `robot/`, `kinematics/`, `web/`
- `kinematics/` → no imports from `robot/`, `hardware/`, `web/`
- `robot/` → imports from `hardware/` and `kinematics/` only
- `web/` → imports from `robot/` only

---

## Step-by-Step Implementation

### Phase 1 — Create `hardware/` Package

#### 1.1 `hardware/__init__.py`
Docstring listing public surface (SerialBus, ServoBusManager, ST3215, BNO085).

#### 1.2 `hardware/config.py` (NEW file)
Move `PIDConfig`, `ServoConfig`, `HardwareConfig` verbatim from `robot/config.py`.
Add Pydantic field validator:
```python
@field_validator("direction_sign")
@classmethod
def _validate_direction_sign(cls, v: int) -> int:
    if v not in (-1, 1):
        raise ValueError(f"direction_sign must be -1 or 1, got {v!r}")
    return v
```

#### 1.3 `hardware/serial_bus.py` (copy + bug fix)
**Bug fix — echo detection permanently disabled after single zero read:**
In `transfer()` (lines 94-96), `sync_read()` (lines 133-134), `send_no_reply()` (lines 162-163):
```python
# Remove in all three places:
self._expect_echo = False

# Replace with:
logger.warning("Echo drain returned 0 bytes on %s; skipping this drain", self._port)
```
No import changes (no project imports).

#### 1.4 `hardware/servo_bus_manager.py` (copy + import update + bug fix)
Update imports (`robot.hardware.*` → `hardware.*`).

**Bug fix — `data[6]`/`data[7]` accessed without bounds check:**
The existing guard `len(data) < Reg.STATUS_LEN` (STATUS_LEN = 8) already protects this.
Add a `logger.debug(...)` message to the guard branch for visibility:
```python
if servo is None or len(data) < Reg.STATUS_LEN:
    if servo is not None:
        logger.debug("SYNC_READ: short response from servo %d (%d bytes)", sid, len(data))
    continue
```

#### 1.5–1.7 `hardware/st3215/__init__.py`, `protocol.py`, `registers.py`
`protocol.py`: update `from robot.hardware.st3215.registers import Instr` → `from hardware.st3215.registers import Instr`.
`registers.py`: no changes (pure constants).

#### 1.8 `hardware/st3215/servo.py` (copy + import update + bug fixes)
Update all imports (`robot.config` → `hardware.config`, `robot.hardware.*` → `hardware.*`).

**Bug fix — `deg_to_steps()` missing direction_sign==0 guard (asymmetric with `steps_to_deg`):**
```python
def deg_to_steps(self, deg: float) -> int:
    if self._cfg.direction_sign == 0:
        return self._cfg.zero_offset_steps
    raw = self._cfg.zero_offset_steps + self._cfg.direction_sign * deg * STEPS_PER_DEG
    return int(round(max(0, min(4095, raw))))
```

**Add public properties** to eliminate `web/routers/servos.py` private field access:
```python
@property
def zero_offset_steps(self) -> int:
    return self._cfg.zero_offset_steps

@zero_offset_steps.setter
def zero_offset_steps(self, value: int) -> None:
    self._cfg.zero_offset_steps = value

@property
def direction_sign(self) -> int:
    return self._cfg.direction_sign

@property
def default_position_deg(self) -> float:
    return self._cfg.default_position_deg

@default_position_deg.setter
def default_position_deg(self, value: float) -> None:
    self._cfg.default_position_deg = value
```

#### 1.9–1.10 `hardware/imu/__init__.py` and `hardware/imu/bno085.py` (copy + bug fix)
**Bug fix — hardcoded `calibration_status=3`:**
```python
try:
    cal_status = int(self._device.calibration_status)
except Exception:
    logger.debug("calibration_status unavailable, defaulting to 0")
    cal_status = 0
# then use cal_status in IMUReading(...)
```

#### 1.11 `hardware/README.md` — See README section below.

---

### Phase 2 — Create `kinematics/` Package

#### 2.1 `kinematics/__init__.py`
Docstring listing KinematicChain, KinematicSolver, IKResult.

#### 2.2 `kinematics/chain.py` (copy + bug fixes)
Add `import logging`, `import os` at top. Add `logger = logging.getLogger(__name__)`.
No project import changes needed (stdlib + numpy only).

**Bug fix — unsafe file write in `_ensure_flat_urdf()` (no error handling):**
```python
# Atomic write: tmp file → os.replace
tmp = flat.parent / (flat.name + ".tmp")
try:
    tmp.write_text(content)
    os.replace(str(tmp), str(flat))
except OSError as exc:
    tmp.unlink(missing_ok=True)
    raise RuntimeError(f"Failed to write flat URDF to {flat}: {exc}") from exc
```

**Bug fix — zero-norm axis silently continues (lines 133-135):**
```python
norm = np.linalg.norm(axis)
if norm > 1e-9:
    axis = axis / norm
else:
    logger.warning("Joint %r has near-zero axis norm; defaulting to [0,0,1]", name)
    axis = np.array([0.0, 0.0, 1.0])
```

#### 2.3 `kinematics/solver.py` (copy + import update + bug fix)
Update: `from robot.kinematics.chain import ...` → `from kinematics.chain import ...`

**Bug fix — hardcoded DOF count:**
```python
# Before:
x0 = np.radians(initial_angles_deg) if initial_angles_deg else np.zeros(6)
# After:
x0 = np.radians(initial_angles_deg) if initial_angles_deg else np.zeros(len(bounds))
```

#### 2.4 `kinematics/README.md` — See README section below.

---

### Phase 3 — Update `robot/config.py`

Strip `PIDConfig`, `ServoConfig`, `HardwareConfig` class bodies. Replace with re-exports:
```python
from hardware.config import PIDConfig, ServoConfig, HardwareConfig  # noqa: F401
```

**Bug fix — YAML errors swallowed without path context:**
```python
def _read_yaml(path: str) -> dict[str, Any]:
    p = Path(path)
    if not p.exists():
        return {}
    with p.open() as f:
        try:
            return yaml.safe_load(f) or {}
        except yaml.YAMLError as exc:
            raise yaml.YAMLError(f"Failed to parse YAML at {p}: {exc}") from exc
```

---

### Phase 4 — Update `robot/robot.py`

**Import changes:**
```python
from robot.config import Settings           # keep
from hardware.config import ServoConfig     # was robot.config
from hardware.imu.bno085 import BNO085, IMUReading
from hardware.serial_bus import SerialBus, SerialBusError
from hardware.servo_bus_manager import ServoBusManager
from hardware.st3215.protocol import encode_ping
from hardware.st3215.servo import ST3215, ServoStatus
from kinematics.chain import KinematicChain
from kinematics.solver import IKResult, KinematicSolver
```

**Bug fix — silent IK failure (no log):**
```python
if result.success:
    ...
else:
    logger.warning(
        "IK failed for leg=%s target=[%.4f, %.4f, %.4f] error=%.6fm msg=%r",
        leg, x, y, z, result.position_error_m, result.message,
    )
```

**Bug fix — FK positions with full float64 precision in telemetry:**
```python
"left_foot": {
    "x": round(left_foot["position"][0], 4),
    "y": round(left_foot["position"][1], 4),
    "z": round(left_foot["position"][2], 4),
},
```

**Add public API methods** (after `compute_fk`):
```python
def urdf_to_logical(self, leg: str, urdf_angles: list[float]) -> list[float]:
    names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
    return [a - self._default_offsets.get(n, 0.0) for a, n in zip(urdf_angles, names)]

def logical_to_urdf(self, leg: str, logical_angles: list[float]) -> list[float]:
    names = KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS
    return [a + self._default_offsets.get(n, 0.0) for a, n in zip(logical_angles, names)]

def leg_joint_names(self, leg: str) -> list[str]:
    return KinematicChain.LEFT_JOINTS if leg == "left" else KinematicChain.RIGHT_JOINTS

def compute_ik(self, leg: str, x: float, y: float, z: float) -> IKResult:
    """IK without executing motion — returns result in URDF space."""
    current = self._get_current_urdf_angles(leg)
    return self._solver.ik(leg, target_pos=np.array([x, y, z]), initial_angles_deg=current)

def list_pose_names(self) -> list[str]:
    return list(self._settings.robot.poses.keys())

def home_pose_name(self) -> str:
    return self._settings.robot.home_pose

def calibrate_imu(self) -> None:
    self._imu.calibrate()

def get_servo_id_info(self, servo_id: int) -> dict | None:
    servo = self._servos_by_id.get(servo_id)
    return {"id": servo_id, "joint": servo.joint_name} if servo else None
```

---

### Phase 5 — Update `robot/__init__.py`

Re-export types that `web/` routers need, so web stays within `robot.*`:
```python
from hardware.serial_bus import SerialBusError  # noqa: F401
from hardware.st3215.servo import ServoStatus   # noqa: F401
```

---

### Phase 6 — Update `web/` Layer

#### `web/websocket.py`
- Remove `import numpy as np` (no longer used after fix)
- Remove `from robot.kinematics.chain import KinematicChain`
- **Bug fix — exceptions silently swallowed in broadcast loop:**
  ```python
  except Exception as exc:
      logger.warning("Broadcast error, dropping client: %s", exc)
      dead.add(ws)
  ```
- `set_foot_ik` handler: replace 8-line URDF→logical conversion block with:
  ```python
  logical_angles = robot.urdf_to_logical(leg, result.angles_deg)
  ```
- `set_joints_fk` handler: replace with:
  ```python
  names = robot.leg_joint_names(leg)
  urdf_angles = robot.logical_to_urdf(leg, logical_angles)
  robot.sync_write_positions(dict(zip(names, urdf_angles)), raw=True)
  ```

#### `web/routers/servos.py`
- Update imports: `from robot.hardware.*` → `from robot import SerialBusError, ServoStatus`
- Fix `scan_servos` line 77 — private `_servos_by_id` access:
  ```python
  info = robot.get_servo_id_info(servo_id)
  result.append({"id": servo_id, "joint": info["joint"] if info else None})
  ```
- Fix `get_servo_raw` lines 104-105 — use public properties `servo.zero_offset_steps`, `servo.direction_sign`
- Fix `set_zero` line 174 — use `servo.zero_offset_steps` (property)
- Fix `set_zero_offset` lines 248-249 — use property setters: `servo.zero_offset_steps = steps`, `servo.default_position_deg = 0.0`
- **Bug fix — relative path `Path("config/robot.yaml")` fails if cwd changes:**
  ```python
  path = Path(__file__).parents[2] / "config" / "robot.yaml"
  ```
- **Bug fix — non-atomic YAML write (crash leaves corrupt file):**
  ```python
  import os, tempfile
  tmp_fd, tmp_path = tempfile.mkstemp(dir=path.parent, suffix=".tmp")
  try:
      with os.fdopen(tmp_fd, "w") as f:
          f.write(yaml.safe_dump(data, sort_keys=False))
      os.replace(tmp_path, str(path))
  except Exception:
      os.unlink(tmp_path)
      raise
  ```

#### `web/routers/kinematics.py`
- Remove `import numpy as np`
- Fix `inverse_kinematics` (line 51) — private `robot._solver.ik()`:
  ```python
  result = robot.set_foot_position(leg, cmd.x, cmd.y, cmd.z) if cmd.execute \
      else robot.compute_ik(leg, cmd.x, cmd.y, cmd.z)
  ```
- Fix `list_poses` (line 63) — private `robot._settings.robot.poses.keys()`:
  ```python
  return _robot(request).list_pose_names()
  ```
- Fix `go_home` (lines 79-80) — private `robot._settings.robot.home_pose`:
  ```python
  robot.go_to_pose(robot.home_pose_name())
  return {"ok": True, "pose": robot.home_pose_name()}
  ```

#### `web/routers/imu.py`
- Fix `calibrate` (line 26) — private `robot._imu.calibrate()`:
  ```python
  _robot(request).calibrate_imu()
  ```

---

### Phase 7 — Delete Old Directories

```bash
git rm -r robot/hardware/ robot/kinematics/
```

---

## READMEs

### `hardware/README.md`
Covers: overview, `serial_bus.py` (half-duplex UART, echo drain, thread safety), `servo_bus_manager.py` (50 Hz loop, latest-wins semantics), `st3215/` (registers, protocol, deg↔steps), `imu/` (BNO085 quaternion/Euler), `config.py` (Pydantic models, direction_sign validation). Minimal usage example for standalone servo control (no robot orchestrator needed).

### `kinematics/README.md`
Covers: overview, `chain.py` (URDF parser, Rodrigues FK, coordinate system, atomic flat-URDF generation), `solver.py` (SLSQP IK, warm-start, L-BFGS-B fallback, IKResult), joint limits, URDF vs logical space note. Minimal usage example for standalone FK/IK (no robot orchestrator needed).

---

## All Bug Fixes Summary

| # | File (after refactor) | Issue | Fix |
|---|---|---|---|
| 1 | `hardware/serial_bus.py` | `_expect_echo=False` set permanently on single zero read | Log warning, don't disable |
| 2 | `hardware/config.py` | `direction_sign=0` accepted silently | Pydantic validator: must be ±1 |
| 3 | `hardware/st3215/servo.py` | `deg_to_steps()` missing direction_sign==0 guard | Add same guard as `steps_to_deg()` |
| 4 | `hardware/servo_bus_manager.py` | No log when short response received | Add `logger.debug` to guard branch |
| 5 | `hardware/imu/bno085.py` | `calibration_status=3` hardcoded | Read from device with try/except fallback to 0 |
| 6 | `kinematics/chain.py` | Unsafe `flat.write_text()` with no error handling | Atomic write via tmp + `os.replace` |
| 7 | `kinematics/chain.py` | Zero-norm axis silently continues | Log warning, use default [0,0,1] |
| 8 | `kinematics/solver.py` | `np.zeros(6)` hardcodes DOF count | `np.zeros(len(bounds))` |
| 9 | `robot/config.py` | `yaml.YAMLError` swallowed without path context | Re-raise with file path |
| 10 | `robot/robot.py` | Silent IK failure (no log) | `logger.warning` on `not result.success` |
| 11 | `robot/robot.py` | FK positions with full float64 precision | `round(..., 4)` |
| 12 | `web/websocket.py` | Broadcast exception silently drops client | `logger.warning` with exc |
| 13 | `web/websocket.py` | Accesses `robot._default_offsets` (private) | Use new `urdf_to_logical` / `logical_to_urdf` |
| 14 | `web/routers/servos.py` | `Path("config/robot.yaml")` relative path | `Path(__file__).parents[2] / "config/robot.yaml"` |
| 15 | `web/routers/servos.py` | Non-atomic YAML config write | `tempfile.mkstemp` + `os.replace` |
| 16 | `web/routers/servos.py` | `servo._cfg.*` private access | Public property setters |
| 17 | `web/routers/kinematics.py` | `robot._solver.ik()` private access | Use `robot.compute_ik()` |
| 18 | `web/routers/kinematics.py` | `robot._settings.*` private access | Use `list_pose_names()`, `home_pose_name()` |
| 19 | `web/routers/imu.py` | `robot._imu.calibrate()` private access | Use `robot.calibrate_imu()` |

---

## Verification

```bash
# 1. Dependency graph — must print nothing
grep -rn "from robot\." hardware/ kinematics/
grep -rn "from hardware\.\|from kinematics\." web/

# 2. Import smoke test
python -c "from hardware.config import PIDConfig, ServoConfig, HardwareConfig; print('OK')"
python -c "from hardware.serial_bus import SerialBus, SerialBusError; print('OK')"
python -c "from hardware.st3215.servo import ST3215, ServoStatus; print('OK')"
python -c "from hardware.imu.bno085 import BNO085, IMUReading; print('OK')"
python -c "from kinematics.chain import KinematicChain; print('OK')"
python -c "from kinematics.solver import KinematicSolver, IKResult; print('OK')"
python -c "from robot.config import Settings; print('OK')"
python -c "from robot.robot import Robot; print('OK')"
python -c "from web.app import create_app; print('OK')"

# 3. Validator test
python -c "
from hardware.config import ServoConfig
try: ServoConfig(servo_id=1, joint_name='x', direction_sign=0)
except Exception as e: print('PASS:', e)
"

# 4. Server starts (simulation mode, no hardware)
python main.py &
sleep 2
curl -s http://localhost:8080/api/servos/ | python -m json.tool
curl -s http://localhost:8080/api/kinematics/poses | python -m json.tool
kill %1
```
