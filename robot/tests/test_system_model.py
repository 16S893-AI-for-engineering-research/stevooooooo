"""
test_system_model.py
====================
Tests for the inchworm.system_model package.

Core tests (no model extra required):
  - JSON calibration files are well-formed
  - Base spacing from calibration matches expected 182.5 ± 0.5 mm

Model tests (require model extra / trimesh):
  - Scene builds at zero and fold
  - Wrist frame bookkeeping self-check < 1 mm
  - Left/right base spacing in chassis frame ≈ 182.5 mm

All model tests are skipped if trimesh is not installed.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import List

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

_ROBOT = Path(__file__).resolve().parents[1]
_REPO  = _ROBOT.parent
_CALIB = _ROBOT / "calibration_data"
_EXPORTS = _REPO / "CAD" / "exports"

# ---------------------------------------------------------------------------
# Skip guard
# ---------------------------------------------------------------------------

_HAS_TRIMESH = False
try:
    import trimesh  # noqa: F401
    import scipy  # noqa: F401
    import networkx  # noqa: F401
    _HAS_TRIMESH = True
except ImportError:
    pass

requires_model = pytest.mark.skipif(
    not _HAS_TRIMESH,
    reason="requires model extra: cd robot && uv sync --extra model",
)

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

# ---------------------------------------------------------------------------
# Core tests (no trimesh required)
# ---------------------------------------------------------------------------

def test_base_fit_json_exists():
    p = _CALIB / "aa40_base_fit_2026-10-25.json"
    assert p.exists(), f"Missing: {p}"


def test_fit_all_json_exists():
    p = _CALIB / "aa40_fit_all_2026-10-25.json"
    assert p.exists(), f"Missing: {p}"


def test_base_fit_fields():
    with open(_CALIB / "aa40_base_fit_2026-10-25.json") as f:
        d = json.load(f)
    assert "left" in d
    assert "right" in d
    T_left = np.array(d["left"]["T_chassis_baselink"])
    T_right = np.array(d["right"]["T_chassis_baselink"])
    assert T_left.shape == (4, 4)
    assert T_right.shape == (4, 4)
    # Last row should be [0,0,0,1]
    np.testing.assert_allclose(T_left[3], [0, 0, 0, 1], atol=1e-9)
    np.testing.assert_allclose(T_right[3], [0, 0, 0, 1], atol=1e-9)


def test_base_spacing_from_json():
    """Base spacing from calibration JSON must be 182.5 ± 0.5 mm."""
    with open(_CALIB / "aa40_base_fit_2026-10-25.json") as f:
        d = json.load(f)
    spacing = d["base_spacing_mm"]
    assert abs(spacing - 182.5) < 0.5, f"spacing={spacing:.2f} mm outside 182.5 ± 0.5 mm"


def test_wrist_fit_fields():
    with open(_CALIB / "aa40_fit_all_2026-10-25.json") as f:
        d = json.load(f)
    for key in ["wrist_camera.stl:wrist#0", "wrist_gripper.stl:wrist#0", "dexarm_base#0"]:
        assert key in d, f"Missing key: {key}"
    T_dex = np.array(d["dexarm_base#0"]["T_chassis_dexarm"])
    assert T_dex.shape == (4, 4)
    np.testing.assert_allclose(T_dex[3], [0, 0, 0, 1], atol=1e-9)


# ---------------------------------------------------------------------------
# Model tests (require trimesh)
# ---------------------------------------------------------------------------

@requires_model
def test_system_model_imports():
    from inchworm.system_model.model import SystemModel  # noqa: F401
    from inchworm.system_model.assets import forward_kinematics, make_tf  # noqa: F401


@requires_model
def test_base_spacing_from_transforms():
    """Left/right base_link origins must be 182.5 ± 0.5 mm apart in chassis frame."""
    from inchworm.system_model.model import SystemModel
    m = SystemModel()
    d = np.linalg.norm(m.T_left[:3, 3] - m.T_right[:3, 3]) * 1000.0
    assert abs(d - 182.5) < 0.5, f"base spacing = {d:.2f} mm, expected 182.5 ± 0.5 mm"
    # Also check against the reported value
    assert abs(m.base_spacing_mm - 182.5) < 0.5


@requires_model
def test_scene_zero_builds():
    """Scene at q=[0]*5 must return at least 5 meshes and no trimesh errors."""
    from inchworm.system_model.model import SystemModel
    m = SystemModel()
    scene = m.scene([0.0]*5, [0.0]*5, include_custom_wrist=False)
    names, meshes = zip(*scene) if scene else ([], [])
    assert len(scene) >= 5, f"Too few scene items: {len(scene)}"
    # All meshes must be valid trimesh objects
    import trimesh
    for name, mesh in scene:
        assert isinstance(mesh, trimesh.Trimesh), f"{name} is not Trimesh"
        assert len(mesh.vertices) > 0, f"{name} has no vertices"


@requires_model
def test_scene_fold_builds():
    """Scene at canonical fold pose must build without error."""
    from inchworm.system_model.model import SystemModel
    Q_FOLD = [0.0, -1.25, 1.64, -1.31, 1.36]
    m = SystemModel()
    scene = m.scene(Q_FOLD, Q_FOLD, include_custom_wrist=False)
    assert len(scene) >= 5


@requires_model
@pytest.mark.skipif(
    not (_EXPORTS / "wrist_camera.stl").exists(),
    reason="CAD/exports/wrist_camera.stl not present",
)
def test_wrist_frame_bookkeeping():
    """Wrist frame bookkeeping self-check: roundtrip via inv(T_export_wristlink)
    must be < 1 mm (numerical identity check on the frame math).

    Also verifies that the FK wrist_link position at the CAD-drawn fold
    is within 5 mm of T_export_wristlink (the fit used to derive q_cad).
    """
    from inchworm.system_model.model import SystemModel
    Q_CAD = [-0.0506, -1.578, 1.574, -1.561, 0.0]
    m = SystemModel()
    result = m.self_check_wrist_frame(Q_CAD, verbose=False)
    max_dist = result.get("max_dist_mm", float("inf"))
    assert max_dist < 1.0, (
        f"Wrist roundtrip failed: max={max_dist:.4f} mm >= 1.0 mm "
        "(frame bookkeeping error)"
    )
    # FK offset should be < 5 mm (fit residual is expected ~3 mm)
    fk_offset = result.get("wrist_fk_offset_mm", float("inf"))
    assert fk_offset < 5.0, (
        f"FK wrist_link offset from export fit: {fk_offset:.2f} mm >= 5 mm"
    )


@requires_model
def test_forward_kinematics_consistency():
    """FK at q=[0]*5 should place wrist_link at a sane Z height above base."""
    from inchworm.system_model.assets import forward_kinematics
    from inchworm.system_model.model import SystemModel
    m = SystemModel()
    frames = forward_kinematics(m.T_left, [0.0]*5)
    # At q=0, wrist should be ~200-300 mm above the base Z
    base_z = m.T_left[2, 3]
    wrist_z = frames["wrist_link"][2, 3]
    assert wrist_z > base_z + 0.10, f"wrist Z {wrist_z*1000:.0f} mm not well above base {base_z*1000:.0f} mm"
    assert wrist_z < base_z + 0.50, f"wrist Z {wrist_z*1000:.0f} mm implausibly high"
