from __future__ import annotations

import copy
import logging
from pathlib import Path

import yaml
from fastapi import APIRouter, Request

logger = logging.getLogger(__name__)
router = APIRouter()


def _robot(request: Request):
    return request.app.state.robot


@router.get("/servos")
def get_servo_configs(request: Request) -> list[dict]:
    return _robot(request).get_servo_configs()


@router.post("/export")
def export_config(request: Request):
    data = _robot(request).export_config()
    yaml_str = _to_yaml(data)

    logger.info("=== Config Export ===\n%s=== End Config ===", yaml_str)

    out_path = Path("/tmp/robot_config.yaml")
    out_path.write_text(yaml_str)
    logger.info("Config written to %s", out_path)

    return {"ok": True, "path": str(out_path)}


# ---------------------------------------------------------------------------
# YAML formatting: pid dicts rendered inline {p: 32, d: 16, i: 0}
# ---------------------------------------------------------------------------

class _FlowDict(dict):
    pass


class _RobotDumper(yaml.Dumper):
    pass


_RobotDumper.add_representer(
    _FlowDict,
    lambda d, v: d.represent_mapping("tag:yaml.org,2002:map", v.items(), flow_style=True),
)


def _to_yaml(data: dict) -> str:
    d = copy.deepcopy(data)
    for servo in d.get("servos", []):
        if "pid" in servo:
            servo["pid"] = _FlowDict(servo["pid"])
    return yaml.dump(d, Dumper=_RobotDumper, sort_keys=False, allow_unicode=True,
                     default_flow_style=False)
