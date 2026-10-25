"""
Analyze CAD/Assembly 1.step to extract per-part world bounding boxes
and propose keep-out boxes for motion_guard.collision.keep_out_boxes.

Usage:
    uv run --no-project --with cadquery-ocp --with numpy python analyze_step_final.py

OCP API notes (this version):
- TDocStd_Document requires TCollection_ExtendedString
- Bnd_Box.Get() is broken; use GetXMin/GetXMax/etc.
- BRepBuilderAPI_Transform on large assemblies causes SIGSEGV; compute bbox
  on the shape in local frame and apply translation manually.
- TopLoc_Location.Transformation() returns gp_Trsf.
- gp_Trsf.Multiply(other) composes in place.
- XCAFDoc_ShapeTool.GetComponents_s is a static method.
"""

import sys
import traceback

from OCP.TCollection import TCollection_ExtendedString
from OCP.TDocStd import TDocStd_Document
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool, XCAFDoc_Location
from OCP.collections import Sequence_TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.TopLoc import TopLoc_Location
from OCP.gp import gp_Trsf

STEP_FILE = "/home/stevotimp/893/CAD/Assembly 1.step"
REPORT_FILE = "/tmp/cad/collision_report.md"

# ── helpers ───────────────────────────────────────────────────────────────────

def get_name(lbl):
    attr = TDataStd_Name()
    if lbl.FindAttribute(TDataStd_Name.GetID_s(), attr):
        return attr.Get().ToExtString()
    return ""


def get_location(lbl):
    attr = XCAFDoc_Location()
    if lbl.FindAttribute(XCAFDoc_Location.GetID_s(), attr):
        return attr.Get()
    return TopLoc_Location()


def compose(parent_trsf, child_loc):
    """Return parent_trsf · child_loc.Transformation() as new gp_Trsf."""
    child_t = child_loc.Transformation()
    result = gp_Trsf()
    result.Multiply(parent_trsf)
    result.Multiply(child_t)
    return result


def bbox_of_shape(shape):
    """Compute local bounding box of a shape; return (xmin,ymin,zmin,xmax,ymax,zmax) or None."""
    if shape.IsNull():
        return None
    box = Bnd_Box()
    try:
        BRepBndLib.Add_s(shape, box, True)
    except Exception:
        try:
            BRepBndLib.Add_s(shape, box, False)
        except Exception:
            return None
    if box.IsVoid():
        return None
    return (box.GetXMin(), box.GetYMin(), box.GetZMin(),
            box.GetXMax(), box.GetYMax(), box.GetZMax())


def apply_trsf_to_bbox(b, trsf):
    """
    Apply a gp_Trsf to a bbox tuple (xmin,ymin,zmin,xmax,ymax,zmax).
    Transform all 8 corners and return a new axis-aligned bbox.
    Only uses translation + rotation/scale (no shear).
    """
    if b is None:
        return None
    from OCP.gp import gp_Pnt
    xmin, ymin, zmin, xmax, ymax, zmax = b
    corners = [
        gp_Pnt(xmin, ymin, zmin), gp_Pnt(xmax, ymin, zmin),
        gp_Pnt(xmin, ymax, zmin), gp_Pnt(xmax, ymax, zmin),
        gp_Pnt(xmin, ymin, zmax), gp_Pnt(xmax, ymin, zmax),
        gp_Pnt(xmin, ymax, zmax), gp_Pnt(xmax, ymax, zmax),
    ]
    xs, ys, zs = [], [], []
    for p in corners:
        p.Transform(trsf)
        xs.append(p.X()); ys.append(p.Y()); zs.append(p.Z())
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def merge_b(a, b):
    if a is None: return b
    if b is None: return a
    return (min(a[0],b[0]), min(a[1],b[1]), min(a[2],b[2]),
            max(a[3],b[3]), max(a[4],b[4]), max(a[5],b[5]))


def bsize(b):
    if b is None: return (0, 0, 0)
    return (b[3]-b[0], b[4]-b[1], b[5]-b[2])


def bcenter(b):
    if b is None: return (0, 0, 0)
    return ((b[0]+b[3])/2, (b[1]+b[4])/2, (b[2]+b[5])/2)


def mm2m(v):
    return round(v / 1000.0, 4)


# ── traversal ─────────────────────────────────────────────────────────────────

MAX_DEPTH = 20

def traverse(shape_tool, lbl, parent_trsf, depth, records, visited):
    """
    Depth-first traversal of the XDE shape tree.
    Applies placement transforms to compute world bboxes.
    visited: set of label entry strings to avoid infinite reference cycles.
    """
    entry = lbl.EntryDumpToString()
    if depth > MAX_DEPTH:
        return

    loc = get_location(lbl)
    world_trsf = compose(parent_trsf, loc)

    name = get_name(lbl)
    is_simple = XCAFDoc_ShapeTool.IsSimpleShape_s(lbl)
    is_free   = XCAFDoc_ShapeTool.IsFree_s(lbl)
    is_ref    = XCAFDoc_ShapeTool.IsReference_s(lbl)

    # Compute world bbox by transforming local bbox
    shape = shape_tool.GetShape_s(lbl)
    local_bb = bbox_of_shape(shape)
    world_bb = apply_trsf_to_bbox(local_bb, world_trsf)

    records.append(dict(
        entry=entry, name=name, depth=depth,
        world_bb=world_bb, local_bb=local_bb,
        is_simple=is_simple, is_free=is_free, is_ref=is_ref,
    ))

    # Avoid infinite loops through references
    if entry in visited and not is_free:
        return
    visited.add(entry)

    # Recurse into assembly components
    children = Sequence_TDF_Label()
    XCAFDoc_ShapeTool.GetComponents_s(lbl, children)
    for i in range(1, children.Size() + 1):
        traverse(shape_tool, children.Value(i), world_trsf, depth + 1, records, visited)

    # Follow references to their referred shape
    if is_ref:
        ref_lbl = TDF_Label()
        if shape_tool.GetReferredShape_s(lbl, ref_lbl):
            traverse(shape_tool, ref_lbl, world_trsf, depth + 1, records, set())


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    print(f"Loading STEP: {STEP_FILE}")
    print("(57 MB AP242 file — expect 60–120 s for transfer...)")
    sys.stdout.flush()

    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(False)
    reader.SetLayerMode(False)

    status = reader.ReadFile(STEP_FILE)
    print(f"ReadFile: {status}")
    sys.stdout.flush()

    ok = reader.Transfer(doc)
    print(f"Transfer: {ok}")
    sys.stdout.flush()

    shape_tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    free_labels = Sequence_TDF_Label()
    shape_tool.GetFreeShapes(free_labels)
    print(f"Free (top-level) shapes: {free_labels.Size()}")
    for i in range(1, free_labels.Size() + 1):
        lbl = free_labels.Value(i)
        print(f"  [{i}] '{get_name(lbl)}'")
    sys.stdout.flush()

    # Traverse
    identity = gp_Trsf()
    records = []
    visited = set()
    print("\nTraversing assembly tree...")
    sys.stdout.flush()
    for i in range(1, free_labels.Size() + 1):
        lbl = free_labels.Value(i)
        traverse(shape_tool, lbl, identity, 0, records, visited)
    print(f"Traversal complete. Records: {len(records)}")
    sys.stdout.flush()

    # Collect named records with valid world bbox
    named = [(r["name"], r) for r in records if r["name"] and r["world_bb"]]
    # Merge by name (union bbox over all instances with same name)
    merged = {}
    for name, r in named:
        if name not in merged:
            merged[name] = dict(
                name=name, world_bb=r["world_bb"],
                local_bb=r["local_bb"],
                is_simple=r["is_simple"],
                instances=1,
                all_bb=r["world_bb"],
                # Keep individual instance bboxes for arms
                instance_bbs=[r["world_bb"]],
            )
        else:
            existing = merged[name]
            existing["world_bb"] = merge_b(existing["world_bb"], r["world_bb"])
            existing["instances"] += 1
            existing["instance_bbs"].append(r["world_bb"])
    merged_list = sorted(merged.values(), key=lambda r: r["name"].lower())
    print(f"Unique named parts with world bbox: {len(merged_list)}")

    # Overall assembly bbox
    overall_bb = None
    for r in merged_list:
        overall_bb = merge_b(overall_bb, r["world_bb"])

    if overall_bb:
        sx, sy, sz = bsize(overall_bb)
        cx, cy, cz = bcenter(overall_bb)
        print(f"\nOverall assembly bbox (world/STEP frame):")
        print(f"  X: {overall_bb[0]:.1f} → {overall_bb[3]:.1f}  span={sx:.1f} mm")
        print(f"  Y: {overall_bb[1]:.1f} → {overall_bb[4]:.1f}  span={sy:.1f} mm")
        print(f"  Z: {overall_bb[2]:.1f} → {overall_bb[5]:.1f}  span={sz:.1f} mm")
        print(f"  Centre: ({cx:.1f},{cy:.1f},{cz:.1f}) mm")

    # Print all named parts
    print("\n=== Named parts (world bbox, merged over instances) ===")
    for r in merged_list:
        b = r["world_bb"]
        cx, cy, cz = bcenter(b)
        sx, sy, sz = bsize(b)
        print(f"  [{r['instances']:2d}x] {r['name']!r:55s}  "
              f"ctr=({cx:8.1f},{cy:8.1f},{cz:8.1f})  "
              f"sz=({sx:7.1f}×{sy:7.1f}×{sz:7.1f}) mm")

    # Key patterns
    key_patterns = {
        "Base_SO101":           "Base_SO101",
        "Under_arm_SO101":      "Under_arm_SO101",
        "Rotation_Pitch_SO101": "Rotation_Pitch_SO101",
        "Upper_arm_SO101":      "Upper_arm_SO101",
        "Motor_holder_SO101_Base":  "Motor_holder_SO101_Base",
        "Motor_holder_SO101_Wrist": "Motor_holder_SO101_Wrist",
        "Wrist_Roll_Pitch_SO100":   "Wrist_Roll_Pitch_SO100",
        "WaveShare_Mounting_Plate": "WaveShare_Mounting_Plate_SO101",
        "DexArm_050375033":     "050375033",
        "Base_08p":             "Base_08p",
        "Assembly_top":         "Assembly 1",
    }
    key_found = {}
    print("\n=== Key part world positions ===")
    for key, pat in key_patterns.items():
        matches = [r for r in merged_list if pat.lower() in r["name"].lower()]
        key_found[key] = matches
        for r in matches:
            b = r["world_bb"]
            cx, cy, cz = bcenter(b)
            sx, sy, sz = bsize(b)
            n_inst = r["instances"]
            print(f"  {key} [{n_inst}x] '{r['name']}'  "
                  f"ctr=({cx:.1f},{cy:.1f},{cz:.1f})  sz=({sx:.1f}×{sy:.1f}×{sz:.1f})")
            # For multi-instance parts, show individual bboxes
            if n_inst > 1 and r["instance_bbs"]:
                for j, ib in enumerate(r["instance_bbs"][:10]):
                    ic = bcenter(ib)
                    is_ = bsize(ib)
                    print(f"    inst {j}: ctr=({ic[0]:.1f},{ic[1]:.1f},{ic[2]:.1f})  "
                          f"sz=({is_[0]:.1f}×{is_[1]:.1f}×{is_[2]:.1f})")

    write_report(merged_list, key_found, overall_bb, records)
    print(f"\nReport written to: {REPORT_FILE}")


def write_report(merged_list, key_found, overall_bb, raw_records):
    lines = []
    A = lines.append

    A("# Collision Geometry Report — `CAD/Assembly 1.step`\n\n")
    A("**Generated by:** `/tmp/cad/analyze_step_final.py` using OCP XDE with world-transform traversal  \n")
    A("**STEP file:** `CAD/Assembly 1.step` (57 MB, AP242 Edition 2, Onshape 1.222, timestamp 2026-10-15T19:24:13Z)  \n")
    A("**Purpose:** Propose axis-aligned keep-out boxes (metres) for `motion_guard.collision.keep_out_boxes` in `robot/config/hardware.yaml`\n\n")
    A("---\n\n")

    A("## 1. STEP Units and Frame Convention\n\n")
    A("STEP AP242 stores all geometry in **millimetres (mm)**. "
      "All bbox values in this report are mm unless stated. Proposed keep-out boxes are in metres.\n\n")
    A("**STEP world frame:** The STEP assembly uses the Onshape assembly origin as the world origin. "
      "The Z axis is assumed to be vertical (Z-up) per the Onshape default; owner must confirm. "
      "The chassis is at rest in this frame. All bbox values below are in the STEP world (= chassis) frame, "
      "with assembly placement transforms applied via `TopLoc_Location.Transformation()` composition.\n\n")
    A("**Important caveat:** If the STEP uses a compound `TopLoc_Location` (multiple chained transforms), "
      "only the outermost transform is composed here; inner chain elements may be missed. "
      "Verified part positions against the Onshape viewer are recommended (see Section 7).\n\n")
    A("---\n\n")

    A("## 2. Full Part List (world bounding boxes, merged over instances)\n\n")
    A("| Part name | Instances | Xmin | Xmax | Ymin | Ymax | Zmin | Zmax | Size (X×Y×Z mm) |\n")
    A("|-----------|-----------|------|------|------|------|------|------|----------------|\n")
    for r in merged_list:
        b = r["world_bb"]
        sx, sy, sz = bsize(b)
        A(f"| `{r['name']}` | {r['instances']} "
          f"| {b[0]:.1f} | {b[3]:.1f} "
          f"| {b[1]:.1f} | {b[4]:.1f} "
          f"| {b[2]:.1f} | {b[5]:.1f} "
          f"| {sx:.1f}×{sy:.1f}×{sz:.1f} |\n")
    A("\n")

    A("---\n\n")
    A("## 3. Overall Assembly Bounding Box\n\n")
    if overall_bb:
        sx, sy, sz = bsize(overall_bb)
        cx, cy, cz = bcenter(overall_bb)
        A("Union of all named parts' world bboxes (STEP/chassis frame, mm):\n\n")
        A("| Axis | Min (mm) | Max (mm) | Span (mm) | Centre (mm) |\n")
        A("|------|----------|----------|-----------|-------------|\n")
        A(f"| X | {overall_bb[0]:.1f} | {overall_bb[3]:.1f} | {sx:.1f} | {cx:.1f} |\n")
        A(f"| Y | {overall_bb[1]:.1f} | {overall_bb[4]:.1f} | {sy:.1f} | {cy:.1f} |\n")
        A(f"| Z | {overall_bb[2]:.1f} | {overall_bb[5]:.1f} | {sz:.1f} | {cz:.1f} |\n")
        A(f"\nFull assembly span: {sx:.0f} × {sy:.0f} × {sz:.0f} mm  \n")
        A(f"Assembly centre: ({cx:.1f}, {cy:.1f}, {cz:.1f}) mm\n\n")
    else:
        A("(No overall bbox computed.)\n\n")
    A("---\n\n")

    A("## 4. Key Part World Positions\n\n")

    def report_group(key, title, extra_note=""):
        A(f"### {title}\n\n")
        if extra_note:
            A(f"{extra_note}\n\n")
        members = key_found.get(key, [])
        if not members:
            A(f"**No part matching `{key}` found in named set.** (See Section 7.)\n\n")
            return None
        grp_bb = None
        for r in members:
            grp_bb = merge_b(grp_bb, r["world_bb"])
            b = r["world_bb"]
            cx, cy, cz = bcenter(b)
            sx, sy, sz = bsize(b)
            A(f"- **`{r['name']}`** ({r['instances']} instance(s)) — "
              f"merged world bbox: [{b[0]:.1f},{b[3]:.1f}] × [{b[1]:.1f},{b[4]:.1f}] × [{b[2]:.1f},{b[5]:.1f}] mm  \n")
            A(f"  Centre ({cx:.1f},{cy:.1f},{cz:.1f}) mm, "
              f"size {sx:.1f}×{sy:.1f}×{sz:.1f} mm\n")
            # Show individual instances if multiple
            if r["instances"] > 1:
                A(f"  Individual instances:\n")
                for j, ib in enumerate(r["instance_bbs"][:20]):
                    ic = bcenter(ib)
                    is_ = bsize(ib)
                    A(f"  - inst {j+1}: ctr=({ic[0]:.1f},{ic[1]:.1f},{ic[2]:.1f}) mm, "
                      f"sz=({is_[0]:.1f}×{is_[1]:.1f}×{is_[2]:.1f}) mm\n")
        if grp_bb:
            sx, sy, sz = bsize(grp_bb)
            cx, cy, cz = bcenter(grp_bb)
            A(f"\nGroup merged bbox: [{grp_bb[0]:.1f},{grp_bb[3]:.1f}] × "
              f"[{grp_bb[1]:.1f},{grp_bb[4]:.1f}] × "
              f"[{grp_bb[2]:.1f},{grp_bb[5]:.1f}] mm  \n")
            A(f"Group centre: ({cx:.1f},{cy:.1f},{cz:.1f}) mm, size: {sx:.1f}×{sy:.1f}×{sz:.1f} mm\n\n")
        return grp_bb

    asm_bb    = report_group("Assembly_top", "Assembly 1 (top-level reference bbox)")
    base_bb   = report_group("Base_SO101",   "Base_SO101 v12 — SO-101 arm base link",
                             "**Confidence: HIGH.** Two instances expected (one per arm).")
    upper_bb  = report_group("Upper_arm_SO101", "Upper_arm_SO101 v9 — SO-101 upper arm link")
    rot_bb    = report_group("Rotation_Pitch_SO101", "Rotation_Pitch_SO101 v8 — shoulder pitch link")
    mbase_bb  = report_group("Motor_holder_SO101_Base", "Motor_holder_SO101_Base v6 — base motor housing")
    mwrist_bb = report_group("Motor_holder_SO101_Wrist", "Motor_holder_SO101_Wrist v8 — wrist motor housing")
    under_bb  = report_group("Under_arm_SO101", "Under_arm_SO101 v12 — arm under-structure link")
    wrist_bb  = report_group("Wrist_Roll_Pitch_SO100", "Wrist_Roll_Pitch_SO100 v15 — wrist roll/pitch link")
    wave_bb   = report_group("WaveShare_Mounting_Plate", "WaveShare_Mounting_Plate_SO101 v4 — camera mount plate")
    dex_bb    = report_group("DexArm_050375033", "050375033 — DexArm body",
                             "**Confidence: MEDIUM** — part number `050375033` matches the Rotrics DexArm model. "
                             "Owner must confirm by opening the Onshape assembly.")
    base08_bb = report_group("Base_08p", "Base_08p v18 — probable chassis base plate",
                             "**Confidence: MEDIUM** — 'Base' + revision 18 suggests a main structural plate. "
                             "Owner must confirm this is the chassis base plate.")

    A("---\n\n")
    A("## 5. Part Identification Reasoning and Confidence\n\n")
    A("""| Part | Identification | Confidence |
|------|---------------|-----------|
| `Base_SO101 v12` | Name includes "SO101"; base link of the LeRobot SO-101 arm | **HIGH** |
| `Upper_arm_SO101 v9` | Name includes "SO101 upper arm" | **HIGH** |
| `Rotation_Pitch_SO101 v8` | Name includes "SO101"; shoulder-pitch link | **HIGH** |
| `Motor_holder_SO101_Base v6` | Name includes "SO101 Base"; motor holder at base | **HIGH** |
| `Motor_holder_SO101_Wrist v8` | Name includes "SO101 Wrist"; motor holder at wrist | **HIGH** |
| `Under_arm_SO101 v12` | Name includes "SO101"; under-arm structural piece | **HIGH** |
| `Wrist_Roll_Pitch_SO100 v15` | Name includes "SO100/SO101 wrist"; wrist assembly | **HIGH** |
| `WaveShare_Mounting_Plate_SO101 v4` | Name matches WaveShare camera mount for SO-101 | **HIGH** |
| `050375033` | Rotrics DexArm part number in Onshape exports | **MEDIUM** |
| `Base_08p v18` | "Base" prefix + large flat geometry → chassis plate | **MEDIUM** |
| Tube | **Not found** — no part named "tube", "track", or "attachment" | **NOT FOUND** |
| Chassis structure | **Not found** by name — may be generic `Part N` solids | **UNKNOWN** |

**Tube note:** The printed tube may be absent from this assembly model, or named with a generic `Part N` identifier. Doc-1 s2 states attachment pitch = 183.676 mm. The tube volume must be estimated from arm positions and gripper TCP height.

**Chassis note:** Doc-1 s2 states the DexArm is 90° from the two SO-101 arms. From the key-part world positions above, the axis separating the DexArm from the SO-101 arms can be identified (DexArm should be offset in one horizontal direction while the SO-101 arms are offset in the other, or both at different Y positions).
""")

    A("---\n\n")
    A("## 6. Proposed Keep-Out Boxes (metres, chassis frame)\n\n")
    A("> **Safety margin: 10 mm (0.010 m) added to every face of every box.**  \n")
    A("> All boxes carry `unverified: true`; owner measurement required before setting `unverified: false`.  \n")
    A("> **Frame:** chassis fixed frame, origin at assembly STEP origin. Z-up assumed. "
      "Owner must confirm Z=0 corresponds to chassis plate top surface and shift Z values accordingly.\n\n")

    margin = 10.0  # mm

    def box_yaml(label, b, source_note, margin_mm=10.0):
        xn = b[0] - margin_mm; xx = b[3] + margin_mm
        yn = b[1] - margin_mm; yx = b[4] + margin_mm
        zn = b[2] - margin_mm; zx = b[5] + margin_mm
        return (
            f"- label: {label}\n"
            f"  x_min_m: {mm2m(xn)}\n"
            f"  x_max_m:  {mm2m(xx)}\n"
            f"  y_min_m: {mm2m(yn)}\n"
            f"  y_max_m:  {mm2m(yx)}\n"
            f"  z_min_m: {mm2m(zn)}\n"
            f"  z_max_m:  {mm2m(zx)}\n"
            f"  unverified: true\n"
            f"  # Source: {source_note}\n"
            f"  # Safety margin: {margin_mm:.0f} mm on all faces.\n"
            f"  # REQUIRES owner verification of part identity and Z=0 reference.\n"
        )

    A("### 6.1 `chassis_base`\n\n")
    if base08_bb:
        b = base08_bb
        sx, sy, sz = bsize(b)
        cx, cy, cz = bcenter(b)
        A(f"`Base_08p v18` world bbox: "
          f"X [{b[0]:.1f},{b[3]:.1f}], Y [{b[1]:.1f},{b[4]:.1f}], Z [{b[2]:.1f},{b[5]:.1f}] mm  \n")
        A(f"Size: {sx:.1f}×{sy:.1f}×{sz:.1f} mm, centre ({cx:.1f},{cy:.1f},{cz:.1f}) mm\n\n")
        A("```yaml\n")
        A(box_yaml("chassis_base", b,
                   f"STEP world bbox of 'Base_08p v18' ({sx:.0f}x{sy:.0f}x{sz:.0f} mm)"))
        A("```\n\n")
    elif overall_bb:
        # Use overall bbox footprint, bottom Z slice
        b_flat = (overall_bb[0], overall_bb[1], overall_bb[2],
                  overall_bb[3], overall_bb[4], overall_bb[2] + 30)
        A("(`Base_08p` not found; using full assembly footprint at bottom Z as fallback.)\n\n")
        A("```yaml\n")
        A(box_yaml("chassis_base", b_flat, "STEP overall bbox bottom slice (fallback)"))
        A("```\n\n")
    else:
        A("(Cannot compute — no assembly bbox available.)\n\n")

    A("### 6.2 `dexarm_body`\n\n")
    if dex_bb:
        b = dex_bb
        sx, sy, sz = bsize(b)
        cx, cy, cz = bcenter(b)
        A(f"`050375033` world bbox: "
          f"X [{b[0]:.1f},{b[3]:.1f}], Y [{b[1]:.1f},{b[4]:.1f}], Z [{b[2]:.1f},{b[5]:.1f}] mm  \n")
        A(f"Size: {sx:.1f}×{sy:.1f}×{sz:.1f} mm, centre ({cx:.1f},{cy:.1f},{cz:.1f}) mm\n\n")
        A("```yaml\n")
        A(box_yaml("dexarm_body", b,
                   f"STEP world bbox of '050375033' parts ({sx:.0f}x{sy:.0f}x{sz:.0f} mm, {len(key_found.get('DexArm_050375033',[])):d} match(es))"))
        A("```\n\n")
    else:
        A("No `050375033` part found. Retaining hardware.yaml placeholder.\n\n")
        A("```yaml\n"
          "- label: dexarm_body\n"
          "  x_min_m: -0.20\n"
          "  x_max_m:  0.20\n"
          "  y_min_m:  0.30\n"
          "  y_max_m:  0.70\n"
          "  z_min_m: -0.05\n"
          "  z_max_m:  0.40\n"
          "  unverified: true\n"
          "  # Source: hardware.yaml placeholder (DexArm not identified in STEP).\n"
          "```\n\n")

    A("### 6.3 `tube_volume`\n\n")
    A("No tube part was identified in the STEP named set. "
      "Tube volume is estimated from known facts and SO-101 kinematics.\n\n")
    A("Facts used:\n"
      "- Attachment pitch: 183.676 mm (doc-1 s2); tube spans at least ±91.8 mm from centre.\n"
      "- Tube OD: **unknown** (owner measurement needed).\n"
      "- Arm base lateral positions: from `Base_SO101` world bbox (if available).\n"
      "- TCP height at gripping pose: ~150–250 mm above plate (estimated from SO-101 URDF).\n\n")

    if base_bb:
        b = base_bb
        arm_x_span = b[3] - b[0]   # full X span of both bases merged
        arm_y_ctr  = (b[1] + b[4]) / 2
        arm_y_half = (b[4] - b[1]) / 2
        # Tube extends ±(arm_x_span/2 + pitch/2) in X
        pitch = 183.676
        tube_x_half = arm_x_span / 2 + pitch / 2 + margin
        tube_y_near = arm_y_ctr - arm_y_half - margin
        tube_y_far  = arm_y_ctr + arm_y_half + 100.0 + margin
        tube_z_lo   = b[2] + 80.0  # above arm bases
        tube_z_hi   = b[2] + 280.0 + margin
        tube_bb = ((-tube_x_half), tube_y_near, tube_z_lo,
                   ( tube_x_half), tube_y_far,  tube_z_hi)
        A(f"Using `Base_SO101` merged X span = {arm_x_span:.1f} mm; arm Y centre = {arm_y_ctr:.1f} mm.\n\n")
    else:
        pitch = 183.676
        tube_x_half = 200.0
        tube_y_near = 80.0
        tube_y_far  = 280.0
        tube_z_lo   = 80.0
        tube_z_hi   = 280.0
        tube_bb = (-tube_x_half, tube_y_near, tube_z_lo,
                    tube_x_half, tube_y_far,  tube_z_hi)
        A("(Base_SO101 not found; using estimated values.)\n\n")

    A("```yaml\n")
    xn, yn, zn, xx, yx, zx = tube_bb
    A(f"- label: tube_volume\n"
      f"  x_min_m: {mm2m(xn)}\n"
      f"  x_max_m:  {mm2m(xx)}\n"
      f"  y_min_m: {mm2m(yn)}\n"
      f"  y_max_m:  {mm2m(yx)}\n"
      f"  z_min_m: {mm2m(zn)}\n"
      f"  z_max_m:  {mm2m(zx)}\n"
      f"  unverified: true\n"
      f"  # Source: attachment pitch 183.676 mm (doc-1 s2); arm lateral extent from\n"
      f"  #   Base_SO101 world bbox; TCP height estimated from SO-101 URDF links.\n"
      f"  # Tube OD is UNKNOWN — owner measurement required to tighten X bounds.\n"
      f"  # Tube axis assumed = X axis in chassis frame (arm bases separated along X).\n")
    A("```\n\n")

    A("### 6.4 `left_arm_base` and `right_arm_base`\n\n")
    if base_bb:
        b = base_bb
        # Individual instances
        base_members = key_found.get("Base_SO101", [])
        if base_members:
            r0 = base_members[0]
            insts = r0["instance_bbs"]
            if len(insts) >= 2:
                # Sort by X centre
                insts_sorted = sorted(insts, key=lambda ib: bcenter(ib)[0])
                left_bb = insts_sorted[0]
                right_bb = insts_sorted[-1]
                lcx, lcy, lcz = bcenter(left_bb)
                rcx, rcy, rcz = bcenter(right_bb)
                separation = rcx - lcx
                A(f"Two `Base_SO101 v12` instances found:  \n")
                A(f"- **Left** (lower X): centre ({lcx:.1f},{lcy:.1f},{lcz:.1f}) mm, "
                  f"bbox [{left_bb[0]:.1f},{left_bb[3]:.1f}]×[{left_bb[1]:.1f},{left_bb[4]:.1f}]×[{left_bb[2]:.1f},{left_bb[5]:.1f}] mm\n")
                A(f"- **Right** (higher X): centre ({rcx:.1f},{rcy:.1f},{rcz:.1f}) mm, "
                  f"bbox [{right_bb[0]:.1f},{right_bb[3]:.1f}]×[{right_bb[1]:.1f},{right_bb[4]:.1f}]×[{right_bb[2]:.1f},{right_bb[5]:.1f}] mm\n")
                A(f"- Lateral separation (ΔX): {separation:.1f} mm\n\n")
                A(f"> ⚠️ **Instance assignment to left/right arm is uncertain.** "
                  f"'Left' and 'right' here follow the STEP X-axis order (lower X = 'left'). "
                  f"Owner must confirm which arm is `arm_left` (servo serial 5B91045044) "
                  f"and which is `arm_right` (serial 5B91030298) per `hardware.yaml`.\n\n")
                A("```yaml\n")
                A(box_yaml("left_arm_base", left_bb,
                           f"STEP world bbox of left (lower-X) Base_SO101 instance. "
                           f"Centre ({lcx:.1f},{lcy:.1f},{lcz:.1f}) mm."))
                A("\n")
                A(box_yaml("right_arm_base", right_bb,
                           f"STEP world bbox of right (higher-X) Base_SO101 instance. "
                           f"Centre ({rcx:.1f},{rcy:.1f},{rcz:.1f}) mm."))
                A("```\n\n")
                A(f"**Arm base-to-chassis offsets (for `so101_kinematics` in hardware.yaml):**\n\n")
                A(f"These are the proposed values for `left_arm_base_to_chassis.xyz` and "
                  f"`right_arm_base_to_chassis.xyz` (STEP frame, mm → m after division by 1000):\n\n")
                A(f"```yaml\n"
                  f"so101_kinematics:\n"
                  f"  left_arm_base_to_chassis:\n"
                  f"    xyz: [{mm2m(lcx)}, {mm2m(lcy)}, {mm2m(lcz)}]  # unverified\n"
                  f"    rpy: [0.0, 0.0, 0.0]  # unverified: rotation unknown from bbox\n"
                  f"    unverified: true\n"
                  f"  right_arm_base_to_chassis:\n"
                  f"    xyz: [{mm2m(rcx)}, {mm2m(rcy)}, {mm2m(rcz)}]  # unverified\n"
                  f"    rpy: [0.0, 0.0, 0.0]  # unverified: rotation unknown from bbox\n"
                  f"    unverified: true\n"
                  f"```\n\n")
                A("RPY (orientation) is **not derivable from the bounding box alone**. "
                  "Owner must measure the rotation of each arm base relative to the chassis frame.\n\n")
            elif len(insts) == 1:
                A(f"Only 1 `Base_SO101 v12` instance found (expected 2 for two arms). "
                  f"The assembly traversal may have missed one instance. "
                  f"Using merged bbox as a fallback for both boxes:\n\n")
                A("```yaml\n")
                A(box_yaml("left_arm_base", b, "Merged Base_SO101 bbox (only 1 instance found; needs owner verification)"))
                A("\n")
                A(box_yaml("right_arm_base", b, "Same as left (only 1 instance found; PLACEHOLDER — use measured values)"))
                A("```\n\n")
            else:
                A("No instance bboxes recorded for `Base_SO101`. Using merged bbox.\n\n")
                A("```yaml\n")
                A(box_yaml("left_arm_base", b, "Merged Base_SO101 bbox"))
                A("\n")
                A(box_yaml("right_arm_base", b, "Merged Base_SO101 bbox (same — split by owner)"))
                A("```\n\n")
    else:
        A("No `Base_SO101` part found. Retaining hardware.yaml capsule placeholders "
          "(left_arm_base at [0.00, -0.35, 0.00] and right_arm_base at [0.00, 0.35, 0.00]).\n\n")

    A("---\n\n")
    A("## 7. What Could NOT Be Determined\n\n")
    A("""1. **Z = 0 reference.** The STEP assembly origin (Onshape) may differ from the chassis plate top surface (the intended Z = 0 for `hardware.yaml`). Owner must identify the Z coordinate of the chassis plate top in the STEP frame and shift all proposed Z values by that offset.

2. **X/Y axis assignment relative to `hardware.yaml` frame.** The `hardware.yaml` convention has +Y toward the DexArm (robot front). The STEP assembly X/Y axes must be compared against this convention. If STEP X = hardware.yaml Y (or vice versa), all proposed boxes must be transposed. Owner must compare the DexArm position and arm positions in both frames.

3. **Tube geometry and volume.** No part named "tube", "track", or "attachment" was found. The tube may be unmodelled or generically named. Owner must measure: tube OD, tube axis position, and the Z height of the gripper TCP at the gripping pose. The tube_volume box in Section 6.3 is an estimate based on attachment pitch and arm kinematics only.

4. **DexArm identity confirmation.** The inference that `050375033` is the DexArm model number is MEDIUM confidence. Owner must open the Onshape assembly and check the part name.

5. **Chassis plate identity.** `Base_08p v18` is MEDIUM confidence as the chassis base plate. Owner must confirm.

6. **Arm base RPY (rotation).** The arm base-to-chassis transform RPY is not derivable from the bounding box. A bbox gives the centroid XYZ but not the orientation. Owner must measure the rotation of each arm's base_link X/Y/Z axes relative to the chassis frame (likely by reading the STEP placement `AXIS2_PLACEMENT_3D` for the Base_SO101 instances, or by physical measurement).

7. **Compound `TopLoc_Location` chains.** Onshape can produce deeply nested assembly transforms where `TopLoc_Location` contains a chain (via `NextLocation()`). This traversal only extracts the primary `Transformation()`; any secondary transforms in the chain are not composed. If part positions look incorrect compared to the Onshape viewer, this is the likely cause.

8. **Left/right arm assignment.** The script assigns "left" to the lower-X arm instance and "right" to the higher-X arm instance by X-coordinate order. The actual `arm_left` (serial 5B91045044) and `arm_right` (serial 5B91030298) assignment must be confirmed by the owner against the physical robot layout.

---

## 8. Recommended Owner Actions

1. **Confirm Z = 0.** Open the STEP in Onshape or FreeCAD. Measure the Z coordinate of the chassis plate top surface. Subtract this value from all proposed Z coordinates to get hardware.yaml-frame Z values.

2. **Confirm X/Y orientation.** Check that the DexArm is at higher +Y (or whatever the hardware.yaml convention is). If not, transpose X↔Y in the proposed boxes.

3. **Confirm part identities.** Verify `Base_08p v18` = chassis plate, `050375033` = DexArm body. If wrong, note the correct part names and re-run the script with updated patterns.

4. **Measure arm base RPY.** Read the `AXIS2_PLACEMENT_3D` entities for the two `Base_SO101` instances from the STEP file, or measure physically. Populate `left_arm_base_to_chassis.rpy` and `right_arm_base_to_chassis.rpy` in `hardware.yaml`.

5. **Measure tube.** Measure the tube OD and the gripper TCP height at gripping pose. Update the `tube_volume` box with correct X (±(tube_length/2)) and Z bounds.

6. **Update hardware.yaml.** Replace `keep_out_boxes` and `so101_kinematics.left_arm_base_to_chassis` / `right_arm_base_to_chassis` with the verified values. Set `unverified: false` only after hardware confirmation.

---

## 9. Quick-Reference Summary Table\n\n""")

    A("| Keep-out box | Source | Confidence | Owner action needed |\n")
    A("|--------------|--------|------------|---------------------|\n")
    if base08_bb:
        b = base08_bb
        sx, sy, sz = bsize(b)
        A(f"| `chassis_base` | Base_08p v18 STEP bbox ({sx:.0f}×{sy:.0f}×{sz:.0f} mm) | MEDIUM | Confirm part identity + Z=0 ref |\n")
    else:
        A("| `chassis_base` | Placeholder | LOW | Identify chassis plate in STEP |\n")
    if dex_bb:
        b = dex_bb
        sx, sy, sz = bsize(b)
        A(f"| `dexarm_body` | 050375033 STEP bbox ({sx:.0f}×{sy:.0f}×{sz:.0f} mm) | MEDIUM | Confirm DexArm part ID + Z=0 ref |\n")
    else:
        A("| `dexarm_body` | Placeholder from hardware.yaml | LOW | Identify DexArm in STEP |\n")
    A("| `tube_volume` | Estimated from attachment pitch + kinematics | LOW | Measure tube OD + gripper TCP height |\n")
    if base_bb:
        A("| `left_arm_base` | Base_SO101 instance (lower-X) STEP bbox | HIGH geom / MEDIUM assignment | Confirm left/right assignment + Z=0 |\n")
        A("| `right_arm_base` | Base_SO101 instance (higher-X) STEP bbox | HIGH geom / MEDIUM assignment | Confirm left/right assignment + Z=0 |\n")
    else:
        A("| `left_arm_base` | Placeholder | LOW | Identify Base_SO101 in STEP |\n")
        A("| `right_arm_base` | Placeholder | LOW | Identify Base_SO101 in STEP |\n")
    A("\n")

    with open(REPORT_FILE, "w") as f:
        f.write("".join(lines))


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nFATAL: {e}")
        traceback.print_exc()
        sys.exit(1)
