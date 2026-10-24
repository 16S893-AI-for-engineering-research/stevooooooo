"""SO-101 servo-bus discovery: scan the bus and persist the joint→ID map.

Usage (dry-run, safe by default)::

    from inchworm.servo_discovery import scan_and_persist
    from inchworm.backends import create_backend
    from inchworm.config import load_config

    config = load_config()
    backend = create_backend(config)
    transport = backend.open_serial("arm_left")
    results = scan_and_persist(transport, arm_role="arm_left",
                               config_path="robot/config/hardware.yaml")

On a mock backend ``scan_and_persist`` returns an empty list (no servos
respond).  On the hardware backend it returns every discovered servo and
writes the joint→ID map to the config file under ``arms.<role>.servo_joints``.

The resulting config entry looks like::

    arms:
      left:
        servo_joints:       # written by scan_and_persist; updated by live scan
          shoulder_pan: 1
          shoulder_lift: 2
          elbow_flex: 3
          wrist_flex: 4
          wrist_roll: 5
          gripper: 6

Joint names are assigned by the owner after the scan (the scan only reports
which IDs respond and their current position).  The ``scan_and_persist``
function writes discovered IDs with a placeholder name until the owner fills
in the mapping.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional

import yaml

from inchworm.backends import SerialTransport
from inchworm.servo_bus import (
    ArmServoBus,
    JOINT_NAMES,
    _GRIPPER_JOINT_NAME,
)

# Placeholder sentinel: servo IDs not yet named get this prefix.
_UNASSIGNED_PREFIX = "servo"


def scan_bus_ids(
    transport: SerialTransport,
    arm_role: str,
    id_range: range = range(1, 20),
    timeout_s: float = 0.05,
) -> List[Dict[str, object]]:
    """Scan the bus and return info dicts for every servo that responds.

    Parameters
    ----------
    transport:
        Open serial transport for this arm's CH343 adapter.
    arm_role:
        ``"arm_left"`` or ``"arm_right"``.
    id_range:
        Range of servo IDs to probe.  Default 1–19 covers typical SO-101
        setups (6 joints use IDs in the single digits).
    timeout_s:
        Per-servo timeout.  Keep short for a live scan.

    Returns a list of dicts with ``servo_id``, ``model_num``,
    ``position_deg``.
    """
    # Build a minimal joint_id_map with a dummy gripper entry so ArmServoBus
    # can be constructed — the scan itself ignores joint names.
    dummy_map: Dict[str, int] = {name: 0 for name in JOINT_NAMES}
    # We don't actually use the joint map for scanning; we call scan_bus directly.
    bus = ArmServoBus(
        transport,
        role=arm_role,
        joint_id_map=dummy_map,
        timeout_s=timeout_s,
    )
    return bus.scan_bus(id_range=id_range)


def scan_and_persist(
    transport: SerialTransport,
    arm_role: str,
    config_path: Optional[str | Path] = None,
    id_range: range = range(1, 20),
    timeout_s: float = 0.05,
) -> List[Dict[str, object]]:
    """Scan the bus and persist discovered IDs to the hardware config.

    The joint→ID mapping under ``arms.<left|right>.servo_joints`` is
    written with placeholder names for newly discovered IDs.  IDs already
    present in the config under a named joint are preserved.

    Parameters
    ----------
    transport:
        Open serial transport for this arm's CH343 adapter.
    arm_role:
        ``"arm_left"`` or ``"arm_right"``.
    config_path:
        Path to ``hardware.yaml``.  Defaults to ``robot/config/hardware.yaml``.
    id_range:
        Servo IDs to probe.
    timeout_s:
        Per-servo probe timeout.

    Returns the list of discovered servo info dicts.
    """
    from inchworm.config import DEFAULT_CONFIG_PATH

    path = Path(config_path) if config_path else DEFAULT_CONFIG_PATH
    discovered = scan_bus_ids(transport, arm_role, id_range, timeout_s)

    if not discovered:
        return discovered

    # Load the raw YAML (preserve comments where possible)
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}

    arm_key = "left" if arm_role == "arm_left" else "right"
    arm_section = raw.setdefault("arms", {}).setdefault(arm_key, {})
    existing: Dict[str, int] = arm_section.get("servo_joints", {}) or {}

    # Invert existing map: id → name
    id_to_name = {v: k for k, v in existing.items()}

    new_joints: Dict[str, int] = dict(existing)
    for entry in discovered:
        sid = int(entry["servo_id"])  # type: ignore[arg-type]
        if sid not in id_to_name:
            # Assign a placeholder name; owner fills in the real name later
            placeholder = f"{_UNASSIGNED_PREFIX}_{sid}"
            new_joints[placeholder] = sid

    arm_section["servo_joints"] = new_joints
    path.write_text(yaml.dump(raw, default_flow_style=False, allow_unicode=True), encoding="utf-8")
    return discovered
