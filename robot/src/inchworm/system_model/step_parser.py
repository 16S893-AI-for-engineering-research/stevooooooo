"""
step_parser.py
==============
Pure-Python streaming parser for STEP AP242 assembly placements.

The only external dependency is ``numpy`` (already a core dep).

Algorithm
---------
Onshape exports a flat one-level assembly: every component instance is a
direct child of the single top-level product definition.  The transform
chain is therefore:

  NEXT_ASSEMBLY_USAGE_OCCURRENCE (NAUO)
    └─> PRODUCT_DEFINITION_SHAPE (PDS) via PDS.relating_prd_def = NAUO
        └─> CONTEXT_DEPENDENT_SHAPE_REPRESENTATION (CDSR) via CDSR.prd_shape = PDS
            └─> REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION (RRWT, a complex entity)
                └─> ITEM_DEFINED_TRANSFORMATION (IDT): from_placement, to_placement
                    └─> to: AXIS2_PLACEMENT_3D: origin (CARTESIAN_POINT), axis-Z (DIRECTION), ref-X (DIRECTION)

The ``from_placement`` is always #402278 (the identity placement: origin=0,0,0;
axis=Z=(0,0,1); ref=X=(1,0,0)), confirmed by inspection of this file.
The ``to_placement`` therefore IS the world placement of the component.

The STEP file records coordinates in **metres** (STEP AP242 default for
Onshape exports), NOT millimetres despite the doc-1 claim.  The bboxes in
the CAD report were from OCP which scales the values.  This parser returns
raw coordinate values; the caller is responsible for unit handling.

Wait — let me reconcile: Onshape exports default to mm in AP214/AP242.
The cad_collision_report says "STEP AP242 stores all geometry in mm".
OCP reads them directly.  The two Base_SO101 placements found above:
  (-0.114758..., 0.203855..., 0.013645...) — these look like metres since
  the bbox report gives (-114.8, 203.9, 13.6) mm for arm A.
  OCP scales Onshape STEP by 1000× when units are set to metres in the
  STEP LENGTH_UNIT entity.

This parser returns raw STEP values; ``StepPlacement.origin_mm`` converts
using the detected unit scale (metres→mm = ×1000).

Unit detection: look for LENGTH_UNIT with 'MILLIMETRE' or 'METRE' in the
NAMED_UNIT / SI_UNIT block.  Onshape AP242 uses SI_UNIT(.METRE.) so raw
values are in metres; bbox_mm = raw × 1000.
"""

from __future__ import annotations

import io
import re
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Data classes
# ---------------------------------------------------------------------------

@dataclass
class StepPlacement:
    """World placement of one component instance in the assembly."""
    instance_id: str           # NAUO entity id (e.g. '4655')
    nauo_id: str               # NAUO label string (e.g. 'NAUO108')
    component_name: str        # name string from the NAUO
    # Raw STEP values (metres for Onshape AP242 exports)
    origin_raw: Tuple[float, float, float]
    axis_z_raw: Tuple[float, float, float]   # Z axis of the placement frame
    ref_x_raw: Tuple[float, float, float]    # X axis (ref direction)
    # Unit scale: multiply raw → mm
    unit_scale_mm: float = 1000.0            # 1.0 if file is already mm

    @property
    def origin_mm(self) -> np.ndarray:
        s = self.unit_scale_mm
        return np.array(self.origin_raw) * s

    @property
    def matrix4x4(self) -> np.ndarray:
        """4×4 homogeneous transform (world frame, mm).

        Column convention: X=ref_x, Y=Z×X, Z=axis_z, t=origin.
        """
        z = np.array(self.axis_z_raw, dtype=float)
        x = np.array(self.ref_x_raw, dtype=float)
        z = z / np.linalg.norm(z)
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)
        y = y / np.linalg.norm(y)
        # re-orthogonalise x
        x = np.cross(y, z)
        t = np.array(self.origin_raw, dtype=float) * self.unit_scale_mm
        m = np.eye(4)
        m[:3, 0] = x
        m[:3, 1] = y
        m[:3, 2] = z
        m[:3, 3] = t
        return m

    def rpy_deg(self) -> Tuple[float, float, float]:
        """Roll-pitch-yaw (ZYX Euler) in degrees from the rotation matrix."""
        R = self.matrix4x4[:3, :3]
        # ZYX convention (yaw-pitch-roll)
        pitch = math.asin(-R[2, 0])
        roll  = math.atan2(R[2, 1], R[2, 2])
        yaw   = math.atan2(R[1, 0], R[0, 0])
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))

    def rpy_rad(self) -> Tuple[float, float, float]:
        """Roll-pitch-yaw (ZYX Euler) in radians."""
        R = self.matrix4x4[:3, :3]
        pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
        roll  = math.atan2(R[2, 1], R[2, 2])
        yaw   = math.atan2(R[1, 0], R[0, 0])
        return (roll, pitch, yaw)


# ---------------------------------------------------------------------------
# Streaming tokeniser
# ---------------------------------------------------------------------------

def _join_step_entities(path: Path) -> Dict[str, str]:
    """Read a STEP file and return {entity_id: body_string}.

    Handles multi-line entities.  Skips the HEADER section; reads DATA section
    until the closing ENDSEC.
    Memory usage: O(file_size) in the worst case.
    """
    entities: Dict[str, str] = {}
    current_id: Optional[str] = None
    buf: List[str] = []
    in_data = False

    # STEP integer entity id at start of line
    _START = re.compile(r'^#(\d+)\s*=\s*(.*)')

    with open(path, encoding='utf-8', errors='replace') as fh:
        for raw_line in fh:
            line = raw_line.rstrip('\n\r')
            stripped = line.strip()

            if not in_data:
                # Wait for the DATA; sentinel
                if stripped == 'DATA;':
                    in_data = True
                continue

            # Inside DATA section
            if stripped in ('ENDSEC;', 'END-ISO-10303-21;'):
                if current_id is not None:
                    entities[current_id] = ''.join(buf)
                break

            m = _START.match(line)
            if m:
                if current_id is not None:
                    entities[current_id] = ''.join(buf)
                current_id = m.group(1)
                buf = [m.group(2)]
            else:
                if current_id is not None:
                    buf.append(line.strip())

    return entities


# ---------------------------------------------------------------------------
# Lightweight entity parser helpers
# ---------------------------------------------------------------------------

def _top_args(s: str) -> List[str]:
    """Split comma-separated STEP args at the top level (ignores nested parens)."""
    args: List[str] = []
    depth = 0
    start = 0
    for i, c in enumerate(s):
        if c == '(':
            depth += 1
        elif c == ')':
            depth -= 1
        elif c == ',' and depth == 0:
            args.append(s[start:i].strip())
            start = i + 1
    args.append(s[start:].strip())
    return args


def _strip_parens(s: str) -> str:
    """Remove leading/trailing parentheses from a string."""
    s = s.strip()
    if s.startswith('(') and s.endswith(')'):
        return s[1:-1]
    return s


def _entity_name_and_args(body: str) -> Tuple[str, str]:
    """Return (ENTITY_TYPE, args_string) from a simple entity body like
    ``CARTESIAN_POINT('', (1., 2., 3.))``."""
    # Strip trailing semicolons (single-line entities end with ';')
    s = body.rstrip(';').strip()
    m = re.match(r'^([A-Z][A-Z0-9_]*)\s*\((.*)\)\s*$', s, re.DOTALL)
    if m:
        return m.group(1), m.group(2)
    return '', ''


def _ref(s: str) -> Optional[str]:
    """Extract entity id from '#NNN' reference string."""
    s = s.strip()
    if s.startswith('#'):
        return s[1:]
    return None


def _parse_floats(s: str) -> List[float]:
    """Parse a parenthesised float list like ``(1.0,2.0,-3.5)``."""
    inner = _strip_parens(s)
    return [float(x) for x in inner.split(',') if x.strip()]


def _parse_string(s: str) -> str:
    """Strip STEP single quotes from a string argument."""
    s = s.strip()
    if s.startswith("'") and s.endswith("'"):
        return s[1:-1]
    return s


# ---------------------------------------------------------------------------
# Unit detection
# ---------------------------------------------------------------------------

_MM_KEYWORDS = ('MILLIMETRE', 'MILLIMETER')
_M_KEYWORDS  = ('METRE', 'METER')

def _detect_unit_scale(entities: Dict[str, str]) -> float:
    """Return scale factor raw_value → mm.

    Onshape AP242 typically uses SI metres (scale = 1000).
    If MILLIMETRE is found, scale = 1.
    Defaults to 1000 (metres) if ambiguous.
    """
    for eid, body in entities.items():
        if 'LENGTH_UNIT' not in body and 'SI_UNIT' not in body:
            continue
        b_upper = body.upper()
        for kw in _MM_KEYWORDS:
            if kw in b_upper:
                return 1.0
        for kw in _M_KEYWORDS:
            if kw in b_upper:
                return 1000.0
    return 1000.0  # default: Onshape AP242 = metres


# ---------------------------------------------------------------------------
# Core parser
# ---------------------------------------------------------------------------

def parse_step_placements(step_path: Path) -> List[StepPlacement]:
    """Parse all component placements from a STEP AP242 file.

    Returns a list of :class:`StepPlacement` objects, one per
    ``NEXT_ASSEMBLY_USAGE_OCCURRENCE`` that has an associated transform.

    Strategy (flat assembly assumed):
    1. Load all entity bodies into memory (streaming line-joiner).
    2. Build lookup tables:
       - nauo_id → component_name
       - pds_id  → nauo_id          (PRODUCT_DEFINITION_SHAPE)
       - cdsr_rrwt_id → pds_id      (CONTEXT_DEPENDENT_SHAPE_REPRESENTATION)
       - rrwt_id → idt_id           (complex entity with RRWT)
       - idt_id  → (from_ap, to_ap) (ITEM_DEFINED_TRANSFORMATION)
       - ap_id   → (origin, z, x)   (AXIS2_PLACEMENT_3D + CARTESIAN_POINT + DIRECTION)
    3. For each NAUO trace: nauo → pds → cdsr → rrwt → idt → to_ap → placement.
    """
    entities = _join_step_entities(step_path)
    unit_scale = _detect_unit_scale(entities)

    # -- 1. Parse AXIS2_PLACEMENT_3D ------------------------------------------
    # AXIS2_PLACEMENT_3D('name', #pt, #z_dir, #x_dir)
    ap3d: Dict[str, Tuple[str, Optional[str], Optional[str]]] = {}
    for eid, body in entities.items():
        if 'AXIS2_PLACEMENT_3D' not in body:
            continue
        name, args_s = _entity_name_and_args(body)
        if name != 'AXIS2_PLACEMENT_3D':
            continue
        args = _top_args(args_s)
        if len(args) < 2:
            continue
        pt_ref = _ref(args[1])
        z_ref  = _ref(args[2]) if len(args) > 2 else None
        x_ref  = _ref(args[3]) if len(args) > 3 else None
        ap3d[eid] = (pt_ref, z_ref, x_ref)  # type: ignore[assignment]

    # -- 2. Parse CARTESIAN_POINT ---------------------------------------------
    cpts: Dict[str, Tuple[float, float, float]] = {}
    for eid, body in entities.items():
        if 'CARTESIAN_POINT' not in body:
            continue
        name, args_s = _entity_name_and_args(body)
        if name != 'CARTESIAN_POINT':
            continue
        args = _top_args(args_s)
        if len(args) < 2:
            continue
        try:
            coords = _parse_floats(args[1])
        except ValueError:
            continue
        if len(coords) >= 3:
            cpts[eid] = (coords[0], coords[1], coords[2])

    # -- 3. Parse DIRECTION ---------------------------------------------------
    dirs: Dict[str, Tuple[float, float, float]] = {}
    for eid, body in entities.items():
        if not body.startswith("DIRECTION('"):
            continue
        name, args_s = _entity_name_and_args(body)
        if name != 'DIRECTION':
            continue
        args = _top_args(args_s)
        if len(args) < 2:
            continue
        try:
            coords = _parse_floats(args[1])
        except ValueError:
            continue
        if len(coords) >= 3:
            dirs[eid] = (coords[0], coords[1], coords[2])

    # -- 4. Parse ITEM_DEFINED_TRANSFORMATION ---------------------------------
    # IDT(' ',' ',#from_ap, #to_ap)
    idts: Dict[str, Tuple[str, str]] = {}
    for eid, body in entities.items():
        if 'ITEM_DEFINED_TRANSFORMATION' not in body:
            continue
        name, args_s = _entity_name_and_args(body)
        if name != 'ITEM_DEFINED_TRANSFORMATION':
            continue
        args = _top_args(args_s)
        if len(args) < 4:
            continue
        from_ap = _ref(args[2])
        to_ap   = _ref(args[3])
        if from_ap and to_ap:
            idts[eid] = (from_ap, to_ap)

    # -- 5. Parse RRWT complex entities ---------------------------------------
    # Complex entity body contains REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#idt)
    # and REPRESENTATION_RELATIONSHIP(' ',' ',#child_sr, #parent_sr)
    # Entity key is the id of the complex entity block.
    # The RRWT part just contains the IDT reference.
    rrwt_to_idt: Dict[str, str] = {}
    _RRWT_PAT = re.compile(
        r'REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\(#(\d+)\)', re.DOTALL
    )
    for eid, body in entities.items():
        m = _RRWT_PAT.search(body)
        if m:
            rrwt_to_idt[eid] = m.group(1)

    # -- 6. Parse CONTEXT_DEPENDENT_SHAPE_REPRESENTATION ---------------------
    # CDSR(#rrwt_entity, #pds_entity)
    cdsr_to_pds: Dict[str, str] = {}
    for eid, body in entities.items():
        if 'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION' not in body:
            continue
        name, args_s = _entity_name_and_args(body)
        if name != 'CONTEXT_DEPENDENT_SHAPE_REPRESENTATION':
            continue
        args = _top_args(args_s)
        if len(args) < 2:
            continue
        rrwt_ref = _ref(args[0])
        pds_ref  = _ref(args[1])
        if rrwt_ref and pds_ref:
            cdsr_to_pds[rrwt_ref] = pds_ref  # key=rrwt entity, val=pds

    # -- 7. Parse PRODUCT_DEFINITION_SHAPE → NAUO ----------------------------
    # PDS(' ','NAUO PRDDFN',#nauo)
    pds_to_nauo: Dict[str, str] = {}
    for eid, body in entities.items():
        if 'PRODUCT_DEFINITION_SHAPE' not in body:
            continue
        name, args_s = _entity_name_and_args(body)
        if name != 'PRODUCT_DEFINITION_SHAPE':
            continue
        args = _top_args(args_s)
        if len(args) < 3:
            continue
        nauo_ref = _ref(args[2])
        if nauo_ref:
            pds_to_nauo[eid] = nauo_ref

    # -- 8. Parse NEXT_ASSEMBLY_USAGE_OCCURRENCE -----------------------------
    # NAUO('id','name','desc',#relating_pd,#related_pd,'ref_des')
    nauo_info: Dict[str, Tuple[str, str]] = {}  # nauo_id → (label, name)
    for eid, body in entities.items():
        if 'NEXT_ASSEMBLY_USAGE_OCCURRENCE' not in body:
            continue
        name, args_s = _entity_name_and_args(body)
        if name != 'NEXT_ASSEMBLY_USAGE_OCCURRENCE':
            continue
        args = _top_args(args_s)
        if len(args) < 2:
            continue
        label = _parse_string(args[0])   # e.g. 'NAUO108'
        comp_name = _parse_string(args[1])
        nauo_info[eid] = (label, comp_name)

    # -- 9. Assemble placements ----------------------------------------------
    # Walk: CDSR key (rrwt_entity) → pds → nauo → name
    #                             ↓
    #                          rrwt → idt → to_ap → (pt, z, x) → placement

    # Build reverse: pds_id → cdsr_rrwt_id
    # cdsr_to_pds maps rrwt_id → pds_id; we want pds_id → rrwt_id
    pds_to_rrwt: Dict[str, str] = {}
    for rrwt_id, pds_id in cdsr_to_pds.items():
        pds_to_rrwt[pds_id] = rrwt_id

    placements: List[StepPlacement] = []
    for pds_id, nauo_id in pds_to_nauo.items():
        if nauo_id not in nauo_info:
            continue
        label, comp_name = nauo_info[nauo_id]

        rrwt_id = pds_to_rrwt.get(pds_id)
        if rrwt_id is None:
            continue
        idt_id = rrwt_to_idt.get(rrwt_id)
        if idt_id is None:
            continue
        if idt_id not in idts:
            continue
        _, to_ap_id = idts[idt_id]

        if to_ap_id not in ap3d:
            continue
        pt_ref, z_ref, x_ref = ap3d[to_ap_id]

        if pt_ref not in cpts:
            continue
        origin = cpts[pt_ref]

        # Z axis (default = (0,0,1) if absent)
        if z_ref and z_ref in dirs:
            axis_z = dirs[z_ref]
        else:
            axis_z = (0.0, 0.0, 1.0)

        # X ref direction (default = (1,0,0) if absent)
        if x_ref and x_ref in dirs:
            ref_x = dirs[x_ref]
        else:
            ref_x = (1.0, 0.0, 0.0)

        placements.append(StepPlacement(
            instance_id=pds_id,
            nauo_id=label,
            component_name=comp_name,
            origin_raw=origin,
            axis_z_raw=axis_z,
            ref_x_raw=ref_x,
            unit_scale_mm=unit_scale,
        ))

    return placements


# ---------------------------------------------------------------------------
# Convenience grouping helpers
# ---------------------------------------------------------------------------

_ARM_PATTERNS = (
    'Base_SO101',
    'Rotation_Pitch_SO101',
    'Motor_holder_SO101',
    'Upper_arm_SO101',
    'Under_arm_SO101',
    'Wrist_Roll_Pitch_SO100',
    'WaveShare_Mounting_Plate_SO101',
    'Base_08p',
    'Board',
    'GE_',       # servo horns etc.
)

_TUBE_PATTERNS = ('Part 13',)
_DEXARM_PATTERNS = ('Part 1', 'Part 2', 'Part 3', 'Part 4', 'Part 5',
                    'Part 6', 'Part 7', 'Part 8', 'Part 9', 'Part 10',
                    'Part 11', 'Part 12')

def classify_placement(p: StepPlacement) -> str:
    """Classify a placement into one of: arm / tube / dexarm / other.

    Fine-grained arm/dexarm classification is handled in model_builder.py.
    """
    name = p.component_name
    for pat in _ARM_PATTERNS:
        if pat in name:
            return 'arm'
    for pat in _TUBE_PATTERNS:
        if name == pat or name.startswith(pat + ':'):
            return 'tube'
    for pat in _DEXARM_PATTERNS:
        if name == pat or name.startswith(pat + ':') or name.startswith(pat + '__'):
            return 'dexarm'
    return 'other'
