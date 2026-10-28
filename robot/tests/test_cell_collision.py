"""
test_cell_collision.py
======================
AA-40 stage 2: mesh collision on the owner cell URDF
(CAD/robot_urdf_new/three_arm_cell.urdf) and its wiring into MotionGuard.

The checker loads once per module (meshes take about 11 s).
"""
from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np
import pytest

fcl = pytest.importorskip("fcl")
pytest.importorskip("trimesh")

from inchworm.config import load_config  # noqa: E402
from inchworm.motion_guard import (  # noqa: E402
    LiveMotionBlockedError,
    MeshCollisionGuard,
    build_mesh_collision_guard,
    build_motion_guard,
)
from inchworm.system_model.collision import (  # noqa: E402
    DEFAULT_CELL_URDF,
    TUBE_LINK,
    CellCollisionChecker,
    CellState,
    CollisionModelError,
    JointMapping,
    PoseScopedAllowance,
    get_cell_checker,
)
from inchworm.system_model.urdf import URDFError, load_urdf  # noqa: E402

_ROBOT = Path(__file__).resolve().parents[1]
_VENDOR = _ROBOT / "vendor" / "so101" / "so101_new_calib.urdf"
_POSES = _ROBOT / "calibration_data" / "poses_2026-10-25.jsonl"

PITCH_M = 0.183676  # doc-1 attachment pitch (also in hardware.yaml geometry)


@pytest.fixture(scope="module")
def cfg():
    return load_config()


@pytest.fixture(scope="module")
def mesh_cfg(cfg):
    return cfg.motion_guard.collision.mesh


@pytest.fixture(scope="module")
def checker(mesh_cfg):
    """Configured checker (margin, roll mapping, fold allowance) as the guard builds it."""
    g = build_mesh_collision_guard(load_config(), "left")
    assert g is not None and g.checker is not None, g and g.load_error
    return g.checker


def _measured_fold():
    """Measured folded_transit, pan recomputed from ticks with the landmark zero."""
    cfg = load_config()
    sc = cfg.motion_guard.servo_calibration
    names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
    out = {}
    for line in _POSES.read_text().splitlines():
        d = json.loads(line)
        if d.get("label") != "folded_transit":
            continue
        blk = sc.left_arm if d["arm"] == "left" else sc.right_arm
        out[d["arm"]] = np.array([
            math.radians(getattr(blk, f"{n}_sign") * (d["ticks"][i] * 360 / 4096 - getattr(blk, f"{n}_zero_offset_deg")))
            for i, n in enumerate(names)
        ])
    return out["left"], out["right"]


# Other arm parked away (pan 86 deg) and DexArm turned away: cell states for
# single-arm tests that should only see the arm under test.
PARK = np.array([1.5, 0.0, 0.0, 0.0, 0.0])
DEX_AWAY = {"bases": 1.2}


# ---------------------------------------------------------------------------
# URDF loading and joint mapping
# ---------------------------------------------------------------------------


def test_cell_urdf_loads_without_ros():
    u = load_urdf(DEFAULT_CELL_URDF)
    assert u.root == "world"
    for arm in ("left", "right"):
        for s in ("Rotation", "Pitch", "Elbow", "Wrist_Pitch", "Wrist_Roll"):
            assert f"{arm}_{s}" in u.joints
    assert TUBE_LINK in u.collisions
    for link in ("baseforreal", "B_Fix", "left_gripper", "right_camera_mount", "lasercamera"):
        assert link in u.collisions


def test_unknown_joint_name_is_rejected():
    u = load_urdf(DEFAULT_CELL_URDF)
    with pytest.raises(URDFError):
        u.link_poses({"left_Rotaton": 0.1})


def _mesh_pose(m, link, fname, P):
    g = [x for x in m.collisions[link] if x.mesh_path.name == fname][0]
    return P[link] @ g.T_link_geom


@pytest.mark.parametrize("q", [
    [0.0, 0.0, 0.0, 0.0],
    [0.4, -0.7, 0.9, -0.3],
    [-1.0, 1.2, -1.1, 1.4],
    [0.0576, -1.3823, 1.6877, -1.3055],
])
def test_owner_joints_match_vendored_chain(q):
    """Pan/lift/elbow/wrist_flex map 1:1 (sign +1, offset 0): link meshes coincide."""
    cell = load_urdf(DEFAULT_CELL_URDF)
    ven = load_urdf(_VENDOR)
    Pv = ven.link_poses(dict(zip(["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"], q)))
    Pc = cell.link_poses(dict(zip(["left_Rotation", "left_Pitch", "left_Elbow", "left_Wrist_Pitch"], q)))
    Bv = _mesh_pose(ven, "base_link", "base_so101_v2.stl", Pv)
    Bc = _mesh_pose(cell, "left_base", "base_so101_v2.stl", Pc)
    for lv, lc, f in [
        ("shoulder_link", "left_shoulder", "rotation_pitch_so101_v1.stl"),
        ("upper_arm_link", "left_upper_arm", "upper_arm_so101_v1.stl"),
        ("lower_arm_link", "left_lower_arm", "under_arm_so101_v1.stl"),
        ("wrist_link", "left_wrist", "wrist_roll_pitch_so101_v2.stl"),
    ]:
        A = np.linalg.inv(Bv) @ _mesh_pose(ven, lv, f, Pv)
        B = np.linalg.inv(Bc) @ _mesh_pose(cell, lc, f, Pc)
        assert np.linalg.norm(A[:3, 3] - B[:3, 3]) < 1e-5, lc  # 0.01 mm
        assert np.allclose(A[:3, :3], B[:3, :3], atol=1e-4), lc


def test_arm_bases_spacing_matches_doc1():
    """Cell URDF arm bases are 182.5 mm apart along Y (doc-1 / owner tape)."""
    u = load_urdf(DEFAULT_CELL_URDF)
    P = u.link_poses({})
    d = (P["right_base"][:3, 3] - P["left_base"][:3, 3]) * 1000
    assert abs(d[1] - 182.5) < 0.5 and abs(d[0]) < 0.1 and abs(d[2]) < 0.1


def test_roll_mapping_puts_our_range_inside_cell_range(mesh_cfg, cfg):
    """Our measured wrist_roll soft range maps inside the cell URDF -254..0 deg range."""
    lim = cfg.motion_guard.left_arm_limits
    lo = mesh_cfg.wrist_roll_cell_sign * lim.wrist_roll_lower_rad + mesh_cfg.wrist_roll_cell_offset_rad
    hi = mesh_cfg.wrist_roll_cell_sign * lim.wrist_roll_upper_rad + mesh_cfg.wrist_roll_cell_offset_rad
    assert -4.433136 <= min(lo, hi) and max(lo, hi) <= 0.0


# ---------------------------------------------------------------------------
# Tube
# ---------------------------------------------------------------------------


def test_tube_axis_repeats_at_pitch(checker):
    """Tube = 3 sections, one attachment point each: shifting by one pitch maps it onto itself."""
    a = checker.tube_axis
    assert abs(np.linalg.norm(a) - 1) < 1e-9
    assert a[1] > 0.999  # along cell +Y (away from the DexArm), ~1.1 deg tilt
    v = np.asarray(checker._mesh[TUBE_LINK].vertices)
    assert (v[:, 1].max() - v[:, 1].min()) == pytest.approx(3 * PITCH_M, abs=0.003)


def test_tube_moves_with_advance(checker):
    s0 = checker.link_poses(CellState(PARK, PARK, tube_advance_m=0.0))[TUBE_LINK]
    s1 = checker.link_poses(CellState(PARK, PARK, tube_advance_m=PITCH_M))[TUBE_LINK]
    assert np.allclose(s1[:3, 3] - s0[:3, 3], checker.tube_axis * PITCH_M)


def test_tube_not_checked_when_absent(checker):
    pairs = checker.checked_pairs("left", with_tube=False)
    assert not any(TUBE_LINK in p for p in pairs)
    assert any(TUBE_LINK in p for p in checker.checked_pairs("left", with_tube=True))


def test_gripper_in_tube_allowed_by_owner_srdf(checker):
    assert checker.allowed_reason("left_gripper", TUBE_LINK) is not None
    assert checker.allowed_reason("left_wrist", TUBE_LINK) is None
    assert checker.allowed_reason("left_camera_mount", TUBE_LINK) is None


def test_tube_hits_dexarm_when_advanced_backwards(checker):
    """Moving the tube one pitch toward the DexArm drives it into the print head."""
    q = np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0])
    st = CellState(q, q, tube_advance_m=-PITCH_M)
    pairs = {frozenset((c.link_a, c.link_b)) for c in checker.check(st, first_only=False)}
    # DexArm-vs-tube is static-vs-static for SO-101 motion and therefore not
    # an arm check; confirm the geometry with an explicit distance instead.
    assert checker.min_distance(st, "EE", TUBE_LINK) == 0.0
    assert not any("left_gripper" in p and TUBE_LINK in p for p in pairs)


# ---------------------------------------------------------------------------
# Poses
# ---------------------------------------------------------------------------


def test_measured_fold_is_collision_free(checker):
    L, R = _measured_fold()
    assert checker.check(CellState(L, R), first_only=False) == []
    # Tube one pitch on (a regular post-advance state) does not change that.
    assert checker.check(CellState(L, R, tube_advance_m=PITCH_M), first_only=False) == []


def test_measured_fold_needs_the_owner_allowance(mesh_cfg):
    """Without the owner-approved fold allowance the model reports contacts there."""
    c = get_cell_checker(margin_m=mesh_cfg.margin_m, joint_mapping=JointMapping(
        offset_rad=(0, 0, 0, 0, mesh_cfg.wrist_roll_cell_offset_rad)))
    L, R = _measured_fold()
    found = {frozenset((x.link_a, x.link_b)) for x in c.check(CellState(L, R), first_only=False)}
    assert frozenset(("left_lower_arm", "left_shoulder")) in found
    assert frozenset(("left_upper_arm", "right_wrist")) in found
    allowed = {frozenset(p) for p in mesh_cfg.fold_allowance_pairs}
    assert found <= allowed


def test_fold_allowance_is_pose_scoped(checker):
    """The same pairs are detected 5 deg of pan away from the measured fold."""
    L, R = _measured_fold()
    L2 = L + np.radians([5.0, 0, 0, 0, 0])
    found = {frozenset((x.link_a, x.link_b)) for x in checker.check(CellState(L2, R), first_only=False)}
    assert frozenset(("left_upper_arm", "right_wrist")) in found
    assert frozenset(("left_lower_arm", "left_shoulder")) in found


def test_working_pose_is_collision_free(checker):
    """Owner working pose (Pitch -90, Elbow 90, Wrist_Pitch -90, roll mid): no contact.

    At the 5 mm margin each lower arm is reported 2.8 mm from its shoulder
    (a near miss, not contact); a slightly opened pose is clear even with margin.
    """
    q = np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0])
    got = checker.check(CellState(q, q, dexarm_q=DEX_AWAY), first_only=False)
    assert got and all(c.distance_m > 0.0 for c in got)
    w = np.array([0.0, -1.4, 1.4, -1.4, 0.0])
    assert checker.check(CellState(w, w, dexarm_q=DEX_AWAY), first_only=False) == []


@pytest.mark.parametrize("label,left,right,expect", [
    ("arm-arm", [0.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0], {"right_gripper"}),
    ("arm-DexArm", [0.0, 0.0, 0.0, 0.0, 0.0], list(PARK), {"left"}),
    ("arm-chassis", [1.9, 1.33, 1.5, 0.0, 0.0], [0.0, -1.4, 1.4, -1.4, 0.0], {"baseforreal"}),
    ("self (wrist into shoulder)", [0.0, 0.0, 1.69, 1.6, 0.0], [0.0, -1.4, 1.4, -1.4, 0.0], {"left_shoulder"}),
])
def test_known_collisions_detected(checker, label, left, right, expect):
    dex = {} if label == "arm-DexArm" else DEX_AWAY
    contacts = checker.check(CellState(np.array(left), np.array(right), dexarm_q=dex), arm="left", first_only=False)
    assert contacts, label
    names = " ".join(f"{c.link_a} {c.link_b}" for c in contacts)
    for e in expect:
        assert e in names, (label, names)
    if label == "arm-DexArm":
        assert any(c.link_a in ("EE", "In_2") or c.link_b in ("EE", "In_2") for c in contacts)


def test_arm_tube_collision_detected(checker):
    """Camera mount into the tube (not an allowed pair) is caught."""
    q0 = np.zeros(5)
    contacts = checker.check(CellState(q0, PARK, dexarm_q=DEX_AWAY, tube_advance_m=0.0), arm="left", first_only=False)
    assert any({c.link_a, c.link_b} == {"left_camera_mount", TUBE_LINK} for c in contacts)


def test_margin_reports_near_misses(mesh_cfg):
    c0 = get_cell_checker(margin_m=0.0)
    c5 = get_cell_checker(margin_m=0.005)
    q = np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0])
    st = CellState(q, PARK, dexarm_q=DEX_AWAY)
    assert c0.check(st, arm="left") == []
    near = c5.check(st, arm="left", first_only=False)
    assert near and all(0.0 < x.distance_m < 0.005 for x in near)


def test_check_timing(checker):
    L, R = _measured_fold()
    t = time.perf_counter()
    for _ in range(5):
        checker.check(CellState(L, R), arm="left")
    per = (time.perf_counter() - t) / 5
    print(f"\nAA-40 mesh check per pose (left arm, measured fold): {per*1000:.1f} ms")
    assert per < 1.0


# ---------------------------------------------------------------------------
# Fail closed and MotionGuard wiring
# ---------------------------------------------------------------------------


def test_missing_urdf_fails_closed(tmp_path):
    with pytest.raises(CollisionModelError):
        CellCollisionChecker(urdf_path=tmp_path / "nope.urdf")


def test_missing_mesh_fails_closed(tmp_path):
    src = DEFAULT_CELL_URDF.read_text()
    bad = tmp_path / "cell.urdf"
    bad.write_text(src)  # mesh paths are relative and do not exist under tmp_path
    with pytest.raises(CollisionModelError):
        CellCollisionChecker(urdf_path=bad, srdf_path=None)


def test_bad_joint_vector_fails_closed(checker):
    with pytest.raises(CollisionModelError):
        checker.check(CellState(np.array([0, 0, 0, 0, float("nan")]), PARK))
    with pytest.raises(CollisionModelError):
        checker.check(CellState(np.zeros(4), PARK))


def test_guard_reports_mesh_collision(cfg):
    g = build_motion_guard(cfg, "left", other_arm_pose_source=lambda: np.array([0.0, -1.4, 1.4, -1.4, 0.0]),
                           dexarm_pose_source=lambda: DEX_AWAY)
    q = np.array([1.9, 1.33, 1.5, 0.0, 0.0])  # inside the soft limits, into the base mount
    assert g.limit_guard.check(q) is None
    v = g.check_pose(q)
    assert v is not None and "mesh collision" in v


def test_guard_trajectory_catches_interior_collision(cfg):
    """Both endpoints clear; the pan sweep between them passes through the DexArm base and base mount."""
    g = build_motion_guard(cfg, "left", other_arm_pose_source=lambda: np.array([0.0, -1.4, 1.4, -1.4, 0.0]),
                           dexarm_pose_source=lambda: DEX_AWAY)
    a = np.array([-1.9, 1.0, 0.0, 0.0, 0.0])  # pan sweep, lift 1.0: clear at both ends,
    b = np.array([1.9, 1.0, 0.0, 0.0, 0.0])   # hits B_Fix / baseforreal mid-sweep
    mesh = g.mesh
    assert mesh.check_pose(a) is None
    assert mesh.check_pose(b) is None
    v = mesh.check_trajectory(a, b)
    assert v is not None and "trajectory sample" in v


def test_guard_load_failure_fails_closed(cfg, tmp_path):
    mc = cfg.motion_guard.collision.mesh.model_copy(update={"urdf_path": str(tmp_path / "missing.urdf")})
    coll = cfg.motion_guard.collision.model_copy(update={"mesh": mc})
    mg = cfg.motion_guard.model_copy(update={"collision": coll})
    bad = cfg.model_copy(update={"motion_guard": mg})
    g = build_motion_guard(bad, "left", other_arm_pose_source=lambda: PARK)
    v = g.mesh.check_pose(np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0]))
    assert v is not None and "unavailable" in v


def test_live_without_other_arm_pose_fails_closed(cfg):
    g = MeshCollisionGuard("left", cfg.motion_guard.collision.mesh.model_copy(update={"unverified": False}),
                           checker=get_cell_checker(), tube_source=lambda: None,
                           dexarm_pose_source=lambda: DEX_AWAY)
    v = g.check_pose(np.array([0.0, -1.4, 1.4, -1.4, 0.0]), live=True)
    assert v is not None and "other arm pose" in v


def test_live_without_tube_state_fails_closed(cfg):
    g = MeshCollisionGuard("left", cfg.motion_guard.collision.mesh.model_copy(update={"unverified": False}),
                           checker=get_cell_checker(), other_arm_pose_source=lambda: PARK,
                           dexarm_pose_source=lambda: DEX_AWAY)
    v = g.check_pose(np.array([0.0, -1.5708, 1.5708, -1.5708, 0.0]), live=True)
    assert v is not None and "tube state" in v


def test_live_blocked_while_mesh_model_unverified(cfg):
    assert cfg.motion_guard.collision.mesh.unverified is True
    g = build_motion_guard(cfg, "left", other_arm_pose_source=lambda: PARK,
                           tube_source=lambda: None, dexarm_pose_source=lambda: DEX_AWAY)
    with pytest.raises(LiveMotionBlockedError):
        g.mesh.check_pose(np.zeros(5), live=True)


def test_live_blocked_when_mesh_layer_not_configured(cfg):
    coll = cfg.motion_guard.collision.model_copy(update={"mesh": None})
    mg = cfg.motion_guard.model_copy(update={"collision": coll})
    g = build_motion_guard(cfg.model_copy(update={"motion_guard": mg}), "left")
    assert g.mesh is None
    lim = g.limit_guard

    class _Pass:
        def check(self, q, live=False):
            return None
    g.limit_guard = _Pass()
    try:
        with pytest.raises(LiveMotionBlockedError):
            g.check_pose(np.zeros(5), live=True)
    finally:
        g.limit_guard = lim


def test_dry_run_without_inputs_skips_but_never_live(cfg):
    g = build_motion_guard(cfg, "left")  # no cell-state sources
    assert g.mesh.check_pose(np.array([0.0, 1.6, 0.0, 0.0, 0.0])) is None  # dry-run: skipped (warned)
    with pytest.raises(LiveMotionBlockedError):
        g.mesh.check_pose(np.array([0.0, 1.6, 0.0, 0.0, 0.0]), live=True)


def test_table_is_not_modelled(checker):
    """Owner 2026-10-27: arms never go near the table; no table collision object."""
    assert not any("table" in l for l in checker._geom)


def test_viewer_renders_trajectory_and_reports_contacts(tmp_path):
    """view_pose.py renders a planned trajectory without per-pose setup and flags contacts."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("view_pose", _ROBOT / "scripts" / "view_pose.py")
    vp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vp)
    traj = tmp_path / "plan.jsonl"
    w = [0.0, -1.4, 1.4, -1.4, 0.0]
    traj.write_text("\n".join(json.dumps({"left": l, "right": w}) for l in (
        [-1.9, 1.0, 0, 0, 0], [0.0, 1.0, 0, 0, 0], [1.9, 1.0, 0, 0, 0])) + "\n")
    rc = vp.main(["--trajectory", str(traj), "--out", str(tmp_path / "png"), "--dexarm", '{"bases": 1.2}'])
    assert rc == 1  # the middle sample is in contact
    pngs = sorted((tmp_path / "png").glob("traj_*.png"))
    assert len(pngs) == 3 and all(p.stat().st_size > 10_000 for p in pngs)
    rc = vp.main(["--presets", "--check-only"])
    assert rc == 1  # pan-zero reading / placeholder named poses are in contact; measured fold is not
