"""AA-40: DexArm firmware kinematics, cell-URDF mapping and the DexArm cell guard."""
from __future__ import annotations

import math

import numpy as np
import pytest

pytest.importorskip("fcl")

from inchworm.config import load_config
from inchworm.dexarm import DexArmDriver, MockDexArmSerial, open_live_dexarm
from inchworm.system_model.dexarm_guard import DexArmCellGuard, DexArmGuardRefused, build_dexarm_cell_guard
from inchworm.system_model.dexarm_kinematics import (
    DexArmAngles, DexArmKinematicsError, forward, inverse, leveling_dz, parse_m114_angles, to_cell_joints,
)
from inchworm.system_model.urdf import load_urdf
from inchworm.system_model.collision import DEFAULT_CELL_URDF

SLOPES = (0.004, 0.147)  # this DexArm's M892, 2026-10-28
FOLD_L = [0.0092, -1.5677, 1.5953, -1.5731, -0.0468]  # measured fold 2026-11-11 (AA-47 lift zero, 2026-11-12 elbow zero)
FOLD_R = [0.0138, -1.557, 1.5923, -1.5739, -0.0491]  # measured fold 2026-11-11 (AA-47 lift zero, 2026-11-12 elbow zero)
NOW = DexArmAngles(-2.0, 12.0, -79.0)  # live reading 2026-10-28


def test_forward_matches_live_reading():
    """Live 2026-10-28: Theta (-2.29, 12.57, -79.63) -> firmware M895 (-5.96, 149.45, 8.87)."""
    x, y, z = forward(DexArmAngles(-2.29, 12.57, -79.63))
    assert abs(x + 5.96) < 0.05 and abs(y - 149.45) < 0.3 and abs(z - 8.87) < 0.05
    # M114 logical Z = sensor Z minus leveling: 31.05 reported
    assert abs((z - leveling_dz(x, y, SLOPES)) - 31.05) < 0.05


def test_inverse_round_trips():
    rng = np.random.default_rng(0)
    n = 0
    for _ in range(500):
        ang = DexArmAngles(rng.uniform(-80, 80), rng.uniform(-10, 60), rng.uniform(-95, -30))
        x, y, z = forward(ang)
        try:
            b = inverse(x, y, z - leveling_dz(x, y, SLOPES), SLOPES)
        except DexArmKinematicsError:
            continue
        n += 1
        assert max(abs(b.a - ang.a), abs(b.b - ang.b), abs(b.c - ang.c)) < 1e-6
    assert n > 300


def test_inverse_refuses_unreachable():
    for tgt in [(0, 500, 0), (0, -10, 0), (0, 150, 200), (0, 50, 0), (float("nan"), 200, 0)]:
        with pytest.raises(DexArmKinematicsError):
            inverse(*tgt, SLOPES)


def test_cell_mapping_reproduces_firmware_geometry():
    """URDF EE joint origin tracks the firmware wrist point (0.31 mm max at fit time)."""
    u = load_urdf(DEFAULT_CELL_URDF)
    Bi = np.linalg.inv(u.link_poses({})["B_Fix"])
    rng = np.random.default_rng(7)
    res = []
    for _ in range(40):
        ang = DexArmAngles(rng.uniform(-80, 80), rng.uniform(-10, 60), rng.uniform(-95, -30))
        x = (Bi @ u.link_poses(to_cell_joints(ang))["EE"])[:3, 3] * 1000 - np.array([0, 5.0, 0])
        ux, uy, uz = -x[0], -x[1], x[2]
        b0 = math.radians(105.24 - ang.b); c0 = math.radians(99.60 + ang.c)
        l1 = 150 * math.cos(b0) + 150 * math.sin(c0); z = 150 * math.sin(b0) - 150 * math.cos(c0)
        a = math.radians(ang.a)
        fx, fy = l1 * math.sin(a), l1 * math.cos(a)
        radial = (ux - fx) * math.sin(a) + (uy - fy) * math.cos(a)
        tang = (ux - fx) * math.cos(a) - (uy - fy) * math.sin(a)
        res.append((radial, tang, uz - z))
    r = np.array(res)
    assert r[:, 0].std() < 0.3 and abs(r[:, 1]).max() < 0.5 and r[:, 2].std() < 0.3


def test_parse_m114_angles():
    lines = ["X:-5.21 Y:149.31 Z:9.35 E:0.00 Count A:-965 B:5790 C:-38121",
             "DEXARM Theta A:-2.00  Theta B:12.00  Theta C:-79.00", "ok"]
    assert parse_m114_angles(lines) == DexArmAngles(-2.0, 12.0, -79.0)
    assert parse_m114_angles(["ok"]) is None


@pytest.fixture(scope="module")
def guard():
    g = build_dexarm_cell_guard(load_config(), so101_pose_source=lambda: {"left": FOLD_L, "right": FOLD_R},
                                tube_source=lambda: None)
    assert g is not None and g.leveling_slopes == SLOPES
    return g


def test_present_pose_is_clear(guard):
    assert guard.contacts_at(NOW, first_only=False) == []


def test_small_moves_and_home_are_clear(guard):
    guard.check_move(NOW, -5.2, 149.3, 41.5)
    guard.check_move(NOW, 0.0, 300.0, 0.0)  # firmware G28 target


def test_move_into_parked_arm_refused(guard):
    with pytest.raises(DexArmGuardRefused, match="left_"):
        guard.check_move(NOW, 0.0, 149.0, -60.0)


def test_tube_counts_when_present():
    g = build_dexarm_cell_guard(load_config(), so101_pose_source=lambda: {"left": FOLD_L, "right": FOLD_R},
                                tube_source=lambda: 0.0)
    hits = g.contacts_at(NOW, first_only=False)
    assert any("tube_1" in (c.link_a, c.link_b) for c in hits)


def test_fail_closed_inputs():
    g = build_dexarm_cell_guard(load_config())
    with pytest.raises(DexArmGuardRefused, match="SO-101"):
        g.check_move(NOW, 0, 200, 50)
    g.so101_pose_source = lambda: {"left": FOLD_L}
    g.tube_source = lambda: None
    with pytest.raises(DexArmGuardRefused, match="missing"):
        g.check_move(NOW, 0, 200, 50)
    g.so101_pose_source = lambda: {"left": FOLD_L, "right": FOLD_R}
    g.leveling_slopes = None
    with pytest.raises(DexArmGuardRefused, match="slopes"):
        g.check_move(NOW, 0, 200, 50)
    g.leveling_slopes = SLOPES
    with pytest.raises(DexArmGuardRefused, match="unreachable"):
        g.check_move(NOW, 0, 600, 0)


def _driver_with_guard(guard, angles_line="DEXARM Theta A:-2.00  Theta B:12.00  Theta C:-79.00"):
    ser = MockDexArmSerial()
    orig = ser._handle_command

    def handle(raw):
        if raw.strip().upper().startswith("M114"):
            ser._enqueue(f"X:-5.21 Y:149.31 Z:31.5 E:0.00\n{angles_line}\nok")
            return
        orig(raw)
    ser._handle_command = handle
    d = DexArmDriver(ser, workspace=load_config().dexarm.workspace, cell_guard=guard)
    d.connect()
    return d, ser


def test_driver_refuses_before_transmission(guard):
    d, ser = _driver_with_guard(guard)
    n0 = len(ser.written)
    with pytest.raises(DexArmGuardRefused):
        d.move_absolute(0.0, 149.0, -60.0)
    with pytest.raises(DexArmGuardRefused):
        d.move_relative(5.2, -0.3, -91.5)
    sent = [w.decode().strip().upper() for w in ser.written[n0:]]
    assert sent and all(c.startswith("M114") for c in sent)  # only position reads went out


def test_driver_allows_clear_move_and_dry_run_checks(guard):
    d, _ = _driver_with_guard(guard)
    d.move_absolute(-5.2, 149.3, 41.5)
    with pytest.raises(DexArmGuardRefused):
        d.move_absolute(0.0, 149.0, -60.0, dry_run=True)
    d.home(dry_run=True)


def test_driver_without_angles_fails_closed(guard):
    d, _ = _driver_with_guard(guard, angles_line="")
    with pytest.raises(DexArmGuardRefused, match="joint angles"):
        d.move_absolute(-5.2, 149.3, 41.5)


def test_open_live_dexarm_requires_inputs():
    cfg = load_config()
    with pytest.raises(DexArmGuardRefused, match="SO-101 poses and tube state"):
        open_live_dexarm(cfg, MockDexArmSerial())
    d = open_live_dexarm(cfg, MockDexArmSerial(), so101_pose_source=lambda: {"left": FOLD_L, "right": FOLD_R},
                         tube_source=lambda: None)
    assert d.cell_guard is not None
