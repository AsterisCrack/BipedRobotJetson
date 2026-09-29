"""ONNX policy inference for the biped.

Consumes a bundle exported by BipedRobot/src/isaaclab/export_onnx.py and drives the
robot from inside the ServoBusManager's 50 Hz loop.

Layering: this package imports from ``hardware/`` and ``robot/`` but nothing imports
it except ``web/`` and ``main.py`` -- same position in the dependency graph as
``kinematics/``.
"""

from .deploy_config import DeployConfig, discover_bundles, load_bundle
from .observation import JointMapper, ObservationBuilder, specific_force_from_linear_accel
from .policy_runner import PolicyRunner
from .safety import FaultReason, SafetyLimits, SafetyMonitor, tilt_from_projected_gravity
from .state_machine import PolicyState, StateMachine

__all__ = [
    "DeployConfig", "load_bundle", "discover_bundles",
    "JointMapper", "ObservationBuilder", "specific_force_from_linear_accel",
    "PolicyRunner",
    "FaultReason", "SafetyLimits", "SafetyMonitor", "tilt_from_projected_gravity",
    "PolicyState", "StateMachine",
]
