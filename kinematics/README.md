# kinematics

Standalone Python library for **forward kinematics (FK)** and **inverse kinematics (IK)** on URDF-described serial chains.

No dependencies on `robot/`, `hardware/`, or `web/`. Requires only `numpy` and `scipy`. Drop this directory into any project and import directly.

---

## Modules

### `chain.py` — URDF Parser + Rodrigues FK

`KinematicChain` parses revolute joints from a URDF file using Python's stdlib `xml.etree.ElementTree` — no URDF library or mesh loading required.

On first load, if the source URDF contains `<xacro:include>` lines, a flat version (`robot_flat.urdf`) is generated automatically using an atomic write (temp file → `os.replace`) so a crash during generation cannot leave a corrupt file.

#### Coordinate Frame

- Root: `base_link`
- FK returns a 4×4 homogeneous transform of the foot end-effector in the `base_link` frame
- Angles are in **URDF space**: 0° = physical servo centre position (set by `zero_offset_steps`)
- Joint limits are extracted from URDF `<limit lower=... upper=...>` in radians

#### Predefined Joint Chains

```python
KinematicChain.LEFT_JOINTS   # ['l_hip_yaw', 'l_hip_roll_joint', 'l_hip_pitch_joint', 'l_knee_joint', 'l_ankle_roll_joint', 'l_ankle_pitch_joint']
KinematicChain.RIGHT_JOINTS  # ['r_hip_yaw', ...]
```

#### FK Implementation

```
T_foot = T_joint1(θ₁) · T_joint2(θ₂) · ... · T_joint6(θ₆)
```

Each `T_joint` = `T_origin × R(axis, θ)` using **Rodrigues' rotation formula**:

```
R = I + sin(θ)·K + (1 − cos(θ))·K²    where K = [axis]×  (skew-symmetric)
```

```python
from kinematics.chain import KinematicChain

chain = KinematicChain("RobotDescription/urdf/Robot.urdf")

# FK: angles in radians, returns 4×4 transform
import math
angles_rad = [math.radians(a) for a in [0, 0, -37, 141, -6.5, 17]]
T = chain.fk("left", angles_rad)
print(T[:3, 3])  # foot position [x, y, z] in metres

# Joint limits
limits = chain.joint_limits("left")  # list of (lower_rad, upper_rad)
```

---

### `solver.py` — scipy SLSQP IK with Fallback

`KinematicSolver` wraps `KinematicChain` with degree-based FK and numerical IK.

#### Forward Kinematics

```python
from kinematics.solver import KinematicSolver
solver = KinematicSolver(chain)

fk = solver.fk("left", [0.0, 0.0, -37.0, 141.0, -6.5, 17.0])
# Returns:
# {
#   'position':        [x, y, z]     metres, base_link frame
#   'rotation_matrix': [[...], ...]  3×3 rotation
#   'transform':       [[...], ...]  4×4 homogeneous transform
# }
print(fk["position"])
```

#### Inverse Kinematics

IK is solved numerically with `scipy.optimize.minimize` (SLSQP method, joint-limit bounds). Analytical IK is not used because the hip joint offsets in this URDF prevent the hip pitch/roll axes from intersecting, which breaks standard closed-form humanoid IK.

**Key properties:**
- **Warm start** — initialises from provided `initial_angles_deg`; avoids large joint jumps and converges to the physically correct branch (e.g., knee bending forward)
- **Joint limits** — applied as scipy bounds from URDF `<limit>` elements
- **Cost** — `position_weight × ‖pos_error‖² + orientation_weight × ‖R_diff − I‖²`
- **Fallback** — if SLSQP fails, retries with L-BFGS-B
- **DOF-agnostic** — warm-start size derived from `len(bounds)`, not hardcoded

```python
import numpy as np
from kinematics.solver import KinematicSolver, IKResult

result = solver.ik(
    leg="left",
    target_pos=np.array([0.0, 0.08, -0.35]),  # metres
    initial_angles_deg=[0, 0, -37, 141, -6.5, 17],  # warm start (degrees)
)

if result.success:
    print("Joint angles (deg):", result.angles_deg)
    print("Position error:", result.position_error_m * 1000, "mm")
else:
    print("IK failed:", result.message)
    print("Best error:", result.position_error_m * 1000, "mm")
```

`IKResult` fields:

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | True if converged AND position error < 5 mm |
| `angles_deg` | `list[float]` | 6 joint angles in degrees (URDF space) |
| `position_error_m` | `float` | Residual position error in metres |
| `message` | `str` | Solver message on failure |

---

## Minimal Usage Example

```python
from kinematics.chain import KinematicChain
from kinematics.solver import KinematicSolver
import numpy as np

# Load URDF (generates robot_flat.urdf if needed)
chain = KinematicChain("RobotDescription/urdf/Robot.urdf")
solver = KinematicSolver(chain)

# Forward kinematics — angles in degrees, URDF space
fk = solver.fk("left", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print("Foot position at zero angles:", fk["position"])

# Inverse kinematics
result = solver.ik("left", target_pos=np.array([0.0, 0.032, -0.253]))
if result.success:
    print("IK solution:", result.angles_deg)
else:
    print("IK failed:", result.message)
```

---

## Coordinate System Notes

All positions are in **metres** in the `base_link` frame.

Angles are in **URDF space** (0° = physical servo centre). To work in **logical space** (0° = standing pose), subtract `default_position_deg` for each joint — that conversion belongs in the robot orchestrator, not here.

FK at zero angles (from this robot's URDF):
- Left foot:  `x ≈ −0.001 m, y ≈ +0.032 m, z ≈ −0.253 m`
- Right foot: `x ≈ −0.001 m, y ≈ −0.032 m, z ≈ −0.253 m`
