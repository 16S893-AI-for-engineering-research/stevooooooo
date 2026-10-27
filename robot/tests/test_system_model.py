"""
test_system_model.py
====================
Tests for the inchworm.system_model package.

Tests that do NOT require the ``model`` extra (trimesh etc.) run always.
Tests that require ``model`` extras are skipped if the package is missing.

Tests marked ``@pytest.mark.slow`` require the real STEP file (57 MB) and
parse it fully; they are skipped unless explicitly enabled.

Enable slow tests:
    cd robot && uv run pytest tests/test_system_model.py -m slow
"""
from __future__ import annotations

import io
import json
import math
from pathlib import Path
from textwrap import dedent
from typing import List

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Skip guards
# ---------------------------------------------------------------------------

_HAS_TRIMESH = False
try:
    import trimesh  # noqa: F401
    _HAS_TRIMESH = True
except ImportError:
    pass

_REPO = Path(__file__).resolve().parents[2]
_STEP = _REPO / "CAD" / "Assembly 1.step"
_VENDOR = _REPO / "robot" / "vendor" / "so101"

requires_model = pytest.mark.skipif(
    not _HAS_TRIMESH,
    reason="requires model extra (uv sync --extra model)",
)


# ---------------------------------------------------------------------------
# Synthetic STEP snippet for unit tests (no large file required)
# ---------------------------------------------------------------------------

_SYNTHETIC_STEP = dedent("""\
ISO-10303-21;
HEADER;
FILE_SCHEMA (('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF'));
FILE_NAME('test','2026-01-01T00:00:00Z',(''),(''),'' ,'','');
FILE_DESCRIPTION(('test'),'2;1');
ENDSEC;

DATA;
#1=SI_UNIT(.LENGTH.,.METRE.);
#2=NAMED_UNIT(*,*,#1);
#10=NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO1','PartA','PartA',#100,#101,'');
#11=NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO2','PartB','PartB',#100,#102,'');
#20=PRODUCT_DEFINITION_SHAPE(' ','NAUO PRDDFN',#10);
#21=PRODUCT_DEFINITION_SHAPE(' ','NAUO PRDDFN',#11);
#30=(
REPRESENTATION_RELATIONSHIP(' ',' ',#200,#201)
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#40)
SHAPE_REPRESENTATION_RELATIONSHIP()
);
#31=(
REPRESENTATION_RELATIONSHIP(' ',' ',#200,#201)
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#41)
SHAPE_REPRESENTATION_RELATIONSHIP()
);
#40=ITEM_DEFINED_TRANSFORMATION(' ',' ',#50,#51);
#41=ITEM_DEFINED_TRANSFORMATION(' ',' ',#50,#52);
#50=AXIS2_PLACEMENT_3D('',#60,#61,#62);
#51=AXIS2_PLACEMENT_3D('',#63,#64,#65);
#52=AXIS2_PLACEMENT_3D('',#66,#67,#68);
#60=CARTESIAN_POINT('',(0.,0.,0.));
#61=DIRECTION('',(0.,0.,1.));
#62=DIRECTION('',(1.,0.,0.));
#63=CARTESIAN_POINT('',(0.1,0.2,0.3));
#64=DIRECTION('',(0.,0.,1.));
#65=DIRECTION('',(1.,0.,0.));
#66=CARTESIAN_POINT('',(0.4,0.5,0.6));
#67=DIRECTION('',(0.,0.,1.));
#68=DIRECTION('',(1.,0.,0.));
#200=SHAPE_REPRESENTATION('Assembly1',(#50),#999);
#201=SHAPE_REPRESENTATION('PartA',(#50),#999);
#30A=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#30,#20);
#31A=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#31,#21);
ENDSEC;
END-ISO-10303-21;
""")

# Note: the synthetic STEP above uses #30A / #31A which is not valid STEP (non-integer IDs).
# We will use a valid synthetic snippet below:

_SYNTHETIC_STEP_VALID = dedent("""\
ISO-10303-21;
HEADER;
FILE_SCHEMA (('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF'));
FILE_NAME('test','2026-01-01T00:00:00Z',(''),(''),'' ,'','');
FILE_DESCRIPTION(('test'),'2;1');
ENDSEC;

DATA;
#1=SI_UNIT(.LENGTH.,.METRE.);
#2=NAMED_UNIT(*,*,#1);
#10=NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO1','PartA','PartA',#100,#101,'');
#11=NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO2','PartB','PartB',#100,#102,'');
#20=PRODUCT_DEFINITION_SHAPE(' ','NAUO PRDDFN',#10);
#21=PRODUCT_DEFINITION_SHAPE(' ','NAUO PRDDFN',#11);
#30=(
REPRESENTATION_RELATIONSHIP(' ',' ',#200,#201)
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#40)
SHAPE_REPRESENTATION_RELATIONSHIP()
);
#31=(
REPRESENTATION_RELATIONSHIP(' ',' ',#200,#201)
REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#41)
SHAPE_REPRESENTATION_RELATIONSHIP()
);
#40=ITEM_DEFINED_TRANSFORMATION(' ',' ',#50,#51);
#41=ITEM_DEFINED_TRANSFORMATION(' ',' ',#50,#52);
#50=AXIS2_PLACEMENT_3D('',#60,#61,#62);
#51=AXIS2_PLACEMENT_3D('',#63,#64,#65);
#52=AXIS2_PLACEMENT_3D('',#66,#67,#68);
#60=CARTESIAN_POINT('',(0.,0.,0.));
#61=DIRECTION('',(0.,0.,1.));
#62=DIRECTION('',(1.,0.,0.));
#63=CARTESIAN_POINT('',(0.1,0.2,0.3));
#64=DIRECTION('',(0.,0.,1.));
#65=DIRECTION('',(1.,0.,0.));
#66=CARTESIAN_POINT('',(0.4,0.5,0.6));
#67=DIRECTION('',(0.,0.,1.));
#68=DIRECTION('',(1.,0.,0.));
#200=SHAPE_REPRESENTATION('Assembly1',(#50),#999);
#201=SHAPE_REPRESENTATION('PartA',(#50),#999);
#301=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#30,#20);
#311=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#31,#21);
ENDSEC;
END-ISO-10303-21;
""")


# ---------------------------------------------------------------------------
# Helper: write synthetic STEP to a temp file
# ---------------------------------------------------------------------------

@pytest.fixture()
def synthetic_step(tmp_path: Path) -> Path:
    p = tmp_path / "test.step"
    p.write_text(_SYNTHETIC_STEP_VALID)
    return p


# ---------------------------------------------------------------------------
# Unit tests: parser on synthetic STEP
# ---------------------------------------------------------------------------

class TestStepParser:
    """Pure-Python STEP parser tests — no model extra required."""

    def test_parse_synthetic(self, synthetic_step: Path) -> None:
        """Parser extracts both placements from synthetic STEP snippet."""
        from inchworm.system_model.step_parser import parse_step_placements

        placements = parse_step_placements(synthetic_step)
        assert len(placements) == 2, f"Expected 2 placements, got {len(placements)}"

    def test_names(self, synthetic_step: Path) -> None:
        """Part names are correctly extracted."""
        from inchworm.system_model.step_parser import parse_step_placements

        placements = parse_step_placements(synthetic_step)
        names = {p.component_name for p in placements}
        assert "PartA" in names
        assert "PartB" in names

    def test_part_a_origin(self, synthetic_step: Path) -> None:
        """PartA placement origin is (100, 200, 300) mm (0.1,0.2,0.3 m)."""
        from inchworm.system_model.step_parser import parse_step_placements

        placements = parse_step_placements(synthetic_step)
        part_a = next(p for p in placements if p.component_name == "PartA")
        o = part_a.origin_mm
        assert abs(o[0] - 100.0) < 1.0, f"Expected X≈100, got {o[0]}"
        assert abs(o[1] - 200.0) < 1.0, f"Expected Y≈200, got {o[1]}"
        assert abs(o[2] - 300.0) < 1.0, f"Expected Z≈300, got {o[2]}"

    def test_part_b_origin(self, synthetic_step: Path) -> None:
        """PartB placement origin is (400, 500, 600) mm."""
        from inchworm.system_model.step_parser import parse_step_placements

        placements = parse_step_placements(synthetic_step)
        part_b = next(p for p in placements if p.component_name == "PartB")
        o = part_b.origin_mm
        assert abs(o[0] - 400.0) < 1.0
        assert abs(o[1] - 500.0) < 1.0
        assert abs(o[2] - 600.0) < 1.0

    def test_identity_rotation(self, synthetic_step: Path) -> None:
        """Synthetic parts have identity rotation (Z axis up, X ref right)."""
        from inchworm.system_model.step_parser import parse_step_placements

        placements = parse_step_placements(synthetic_step)
        for p in placements:
            R = p.matrix4x4[:3, :3]
            # Should be close to identity
            assert np.allclose(R, np.eye(3), atol=1e-6), f"Non-identity rotation for {p.component_name}"

    def test_matrix4x4_homogeneous(self, synthetic_step: Path) -> None:
        """matrix4x4 has homogeneous bottom row [0,0,0,1]."""
        from inchworm.system_model.step_parser import parse_step_placements

        placements = parse_step_placements(synthetic_step)
        for p in placements:
            m = p.matrix4x4
            assert np.allclose(m[3, :], [0, 0, 0, 1], atol=1e-12)

    def test_unit_scale_metres(self, synthetic_step: Path) -> None:
        """Unit scale is 1000 (metres → mm) for SI_UNIT(.METRE.)."""
        from inchworm.system_model.step_parser import parse_step_placements

        placements = parse_step_placements(synthetic_step)
        for p in placements:
            assert p.unit_scale_mm == 1000.0


# ---------------------------------------------------------------------------
# Slow integration tests: real STEP file
# ---------------------------------------------------------------------------

@pytest.mark.slow
class TestStepParserRealFile:
    """Integration tests on the real 57 MB STEP file."""

    @pytest.fixture(scope="class")
    def real_placements(self):
        if not _STEP.exists():
            pytest.skip(f"STEP file not found: {_STEP}")
        from inchworm.system_model.step_parser import parse_step_placements
        return parse_step_placements(_STEP)

    def test_total_count(self, real_placements) -> None:
        """Real STEP file has 545 NAUO placements."""
        assert len(real_placements) == 545

    def test_base_so101_count(self, real_placements) -> None:
        """There are exactly 2 Base_SO101 v12 instances."""
        base = [p for p in real_placements if "Base_SO101 v12" in p.component_name]
        assert len(base) == 2, f"Expected 2, got {len(base)}"

    def test_arm_spacing(self, real_placements) -> None:
        """The two Base_SO101 v12 instances are 182.5 ± 1 mm apart."""
        base = [p for p in real_placements if "Base_SO101 v12" in p.component_name]
        base.sort(key=lambda p: p.origin_mm[1])
        dist = float(np.linalg.norm(base[1].origin_mm - base[0].origin_mm))
        assert abs(dist - 182.5) < 1.0, f"Arm spacing {dist:.2f} mm, expected 182.5 mm"

    def test_arm_identity_rotation(self, real_placements) -> None:
        """Both Base_SO101 v12 placements have identity rotation."""
        base = [p for p in real_placements if "Base_SO101 v12" in p.component_name]
        for b in base:
            R = b.matrix4x4[:3, :3]
            assert np.allclose(R, np.eye(3), atol=1e-4), (
                f"Non-identity rotation for {b.component_name}: R={R}"
            )

    def test_left_arm_lower_y(self, real_placements) -> None:
        """Left arm (nearer DexArm) has lower STEP Y than right arm."""
        base = [p for p in real_placements if "Base_SO101 v12" in p.component_name]
        base.sort(key=lambda p: p.origin_mm[1])
        left_y = base[0].origin_mm[1]
        right_y = base[1].origin_mm[1]
        assert left_y < right_y, f"Expected left_y < right_y, got {left_y:.1f} vs {right_y:.1f}"
        # Y values from collision report
        assert abs(left_y - 203.9) < 2.0, f"Left arm Y={left_y:.1f}, expected ≈203.9 mm"
        assert abs(right_y - 386.4) < 2.0, f"Right arm Y={right_y:.1f}, expected ≈386.4 mm"

    def test_tube_instances(self, real_placements) -> None:
        """There are exactly 3 Part 13 instances (three inchworm positions)."""
        tube = [p for p in real_placements if p.component_name.strip() == "Part 13"]
        assert len(tube) == 3, f"Expected 3 tube instances, got {len(tube)}"


# ---------------------------------------------------------------------------
# Slow model tests: arm placement + FK residuals
# ---------------------------------------------------------------------------

@pytest.mark.slow
@requires_model
class TestArmPlacements:
    """Arm placement and FK tests — require model extra and real STEP file."""

    @pytest.fixture(scope="class")
    def arms(self):
        if not _STEP.exists():
            pytest.skip(f"STEP file not found: {_STEP}")
        from inchworm.system_model.arm_placer import extract_arm_placements
        return extract_arm_placements(_STEP, _VENDOR)

    def test_arm_separation(self, arms) -> None:
        """URDF base_link separation matches expected 182.5 ± 1 mm."""
        dist = np.linalg.norm(
            np.array(arms.left_info["urdf_baselink_origin_mm"]) -
            np.array(arms.right_info["urdf_baselink_origin_mm"])
        )
        assert abs(dist - 182.5) < 1.0, f"base_link separation {dist:.2f} mm"

    def test_rotation_rz_neg90(self, arms) -> None:
        """Both arms have Rz(-90°) base_link rotation in STEP world frame."""
        expected_R = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=float)
        for label, T in [("left", arms.T_world_left_mm), ("right", arms.T_world_right_mm)]:
            R = T[:3, :3]
            assert np.allclose(R, expected_R, atol=1e-4), (
                f"{label} arm rotation != Rz(-90°): R={R}"
            )

    def test_left_arm_y_position(self, arms) -> None:
        """Left arm base_link Y ≈ 235 mm (CAD origin Y≈204 + 31mm offset)."""
        y = arms.T_world_left_mm[1, 3]
        assert abs(y - 235.0) < 5.0, f"Left arm base_link Y={y:.1f} mm"

    def test_right_arm_y_position(self, arms) -> None:
        """Right arm base_link Y ≈ 418 mm."""
        y = arms.T_world_right_mm[1, 3]
        assert abs(y - 418.0) < 5.0, f"Right arm base_link Y={y:.1f} mm"

    def test_fold_shoulder_residual(self, arms) -> None:
        """Shoulder-link centroid residual at folded pose < 20 mm (honest threshold).

        The shoulder_link has a short kinematic chain (only shoulder_pan),
        so its residual should be the tightest.  We accept up to 20 mm because
        (a) the CAD centroid is a bbox centre, not the mesh centroid, and
        (b) the folded-pose joint angles are owner-measured, not back-solved.
        """
        from inchworm.system_model.arm_placer import compute_fold_residuals
        T_left_m = arms.T_world_left_mm.copy()
        T_left_m[:3, 3] /= 1000.0
        q_fold = [0.0, -1.25, 1.64, -1.31, 1.36]
        residuals = compute_fold_residuals(T_left_m, q_fold, _VENDOR)
        shoulder = next(r for r in residuals if r["link_name"] == "shoulder_link")
        err = shoulder["error_mm"]
        # Honest threshold: shoulder is the best-constrained link
        assert err < 20.0, (
            f"Shoulder residual {err:.1f} mm exceeds 20 mm threshold. "
            f"FK centroid: {shoulder['fk_centroid_mm']}, "
            f"CAD centroid: {shoulder['cad_centroid_mm']}"
        )


# ---------------------------------------------------------------------------
# Part classification tests
# ---------------------------------------------------------------------------

class TestPartClassification:
    """Part classification logic — no extra required."""

    def test_tube_classification(self) -> None:
        """Part 13 is classified as 'tube'."""
        from inchworm.system_model.model_builder import classify_part

        assert classify_part("Part 13", np.array([0, 0, 0])) == "tube"
        assert classify_part("Part 13", np.array([0, 300, 0])) == "tube"

    def test_arm_left_classification(self) -> None:
        """SO-101 parts at low Y are classified as arm_left."""
        from inchworm.system_model.model_builder import classify_part

        # Left arm is at Y ≈ 204 mm
        assert classify_part("Base_SO101 v12:1__Body5", np.array([0, 200, 0])) == "arm_left"

    def test_arm_right_classification(self) -> None:
        """SO-101 parts at high Y are classified as arm_right."""
        from inchworm.system_model.model_builder import classify_part

        # Right arm is at Y ≈ 386 mm
        assert classify_part("Base_SO101 v12:1__Body5", np.array([0, 400, 0])) == "arm_right"

    def test_dexarm_classification(self) -> None:
        """Parts at Y < 50 mm (not arm/tube) are classified as dexarm."""
        from inchworm.system_model.model_builder import classify_part

        assert classify_part("Part 7", np.array([0, -100, 0])) == "dexarm"
        assert classify_part("Part 2", np.array([0, -10, 0])) == "dexarm"
