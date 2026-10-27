"""AA-40: fit vendored model meshes onto owner exports; reuse fit() from fit_bases.py."""
import json, math, sys
from pathlib import Path
import numpy as np, trimesh
sys.path.insert(0, str(Path(__file__).parent))
from fit_bases import fit, rpy_to_T, T_to_xyz_rpy, ROOT, EXPORTS, VENDOR

def load_export(name):
    m = trimesh.load(EXPORTS / name, force="mesh")
    if m.bounds[1].max() > 5:
        m.apply_scale(0.001)
    return m.split(only_watertight=False)

def match(bodies, ref, tol=0.03):
    v = ref.volume
    return [b for b in bodies if abs(abs(b.volume) - v) / v < tol]

out = {}
# 1. Wrist_Roll_Pitch in both wrist files (URDF wrist_link visual origin)
wrist = trimesh.load(VENDOR / "wrist_roll_pitch_so101_v2.stl")
wrist.apply_transform(rpy_to_T([0, -0.028, 0.0181], [-1.5708, -1.5708, 0]))
for f in ("wrist_camera.stl", "wrist_gripper.stl"):
    bodies = load_export(f)
    c = match(bodies, wrist)
    print(f, "wrist_roll_pitch candidates", len(c), [round(abs(b.volume) * 1e6, 2) for b in c], flush=True)
    for i, b in enumerate(c):
        T, res = fit(wrist, b)
        xyz, rpy = T_to_xyz_rpy(T)
        out[f"{f}:wrist#{i}"] = {"T_export_wristlink": T.tolist(), "xyz_mm": xyz, "rpy_rad": rpy, **res,
                                 "vol_cm3": abs(b.volume) * 1e6}
        print("  ", i, "xyz_mm", xyz, "rpy", rpy, {k: round(v, 3) for k, v in res.items()}, flush=True)
# 2. DexArm base (B_Fix, mm-scaled STL, origin = link frame)
dex = trimesh.load(ROOT / "robot/vendor/dexarm/STL/00-B_Fix.stl"); dex.apply_scale(0.001)
bodies = load_export("bases.stl")
big = sorted(bodies, key=lambda b: -abs(b.volume))
print("DexArm B_Fix vol", round(dex.volume * 1e6, 1), "extents mm", (dex.extents * 1000).round(1),
      "| largest export bodies:", [(round(abs(b.volume) * 1e6, 1), (b.extents * 1000).round(1).tolist()) for b in big[:3]], flush=True)
cand = [b for b in big[:3] if np.allclose(sorted(b.extents), sorted(dex.extents), atol=0.006)]
for i, b in enumerate(cand):
    T, res = fit(dex, b)
    xyz, rpy = T_to_xyz_rpy(T)
    out[f"dexarm_base#{i}"] = {"T_chassis_dexarm": T.tolist(), "xyz_mm": xyz, "rpy_rad": rpy, **res}
    print("dexarm", i, "xyz_mm", xyz, "rpy", rpy, {k: round(v, 3) for k, v in res.items()}, flush=True)
json.dump(out, open(ROOT / "robot/out/system_model/fit_all.json", "w"), indent=1)
