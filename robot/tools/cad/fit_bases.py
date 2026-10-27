"""AA-40: fit vendored URDF meshes onto the owner's assembly-position STL exports.

Registration: for a URDF link mesh M (in its link frame via the URDF visual origin) and the
matching owner-exported body B (assembly frame, metres), find the rigid T with T*M ~= B.
Method: PCA-axis initial guesses (all 24 proper axis permutations/sign flips), then ICP
(trimesh.registration.icp, point-to-point, no scaling). Report RMS residual (mm) of
sampled B surface points to the transformed M surface, both ways.
"""
from __future__ import annotations

import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np
import trimesh
from trimesh.registration import icp

ROOT = Path(__file__).resolve().parents[3]
VENDOR = ROOT / "robot" / "vendor" / "so101" / "assets"
EXPORTS = ROOT / "CAD" / "exports"


def rpy_to_T(xyz, rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    Ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    T = np.eye(4); T[:3, :3] = Rz @ Ry @ Rx; T[:3, 3] = xyz
    return T


def proper_rotations():
    out = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, signs)):
                R[i, j] = s
            if np.linalg.det(R) > 0:
                out.append(R)
    return out


def principal(m):
    pts = m.sample(20000, return_index=False) if len(m.faces) else m.vertices
    c = pts.mean(0)
    _, _, Vt = np.linalg.svd(pts - c, full_matrices=False)
    R = Vt.T
    if np.linalg.det(R) < 0:
        R[:, 2] *= -1
    return c, R


def fit(model: trimesh.Trimesh, target: trimesh.Trimesh, n=1500, seed=0):
    rng = np.random.default_rng(seed)
    tgt_pts, _ = trimesh.sample.sample_surface(target, n, seed=seed)
    cm, Rm = principal(model)
    ct, Rt = principal(target)
    best = None
    # Coarse pre-screen: score each of the 24 PCA-aligned orientations by a cheap
    # nearest-vertex distance, then run ICP only on the best 4.
    from scipy.spatial import cKDTree
    tree = cKDTree(model.vertices)
    scored = []
    for P in proper_rotations():
        R0 = Rt @ P @ Rm.T
        moved = (tgt_pts - ct) @ R0 + cm  # target -> model frame
        scored.append((tree.query(moved[::5])[0].mean(), P))
    scored.sort(key=lambda x: x[0])
    for _, P in scored[:4]:
        R0 = Rt @ P @ Rm.T
        T0 = np.eye(4); T0[:3, :3] = R0; T0[:3, 3] = ct - R0 @ cm
        # ICP aligns target points onto model: returns T_model<-target; invert later
        try:
            T_mt, _, cost = icp(tgt_pts, model, initial=np.linalg.inv(T0), threshold=1e-9,
                                max_iterations=40, reflection=False, scale=False)
        except Exception:
            continue
        if best is None or cost < best[1]:
            best = (np.linalg.inv(T_mt), cost)
    T = best[0]
    # residuals both ways (mm)
    moved = model.copy(); moved.apply_transform(T)
    d1 = np.abs(trimesh.proximity.signed_distance(moved, tgt_pts[:1000])) * 1000
    mod_pts, _ = trimesh.sample.sample_surface(moved, 1000, seed=seed + 1)
    d2 = np.abs(trimesh.proximity.signed_distance(target, mod_pts)) * 1000
    return T, {"rms_target_to_model_mm": float(np.sqrt((d1 ** 2).mean())),
               "p95_target_to_model_mm": float(np.percentile(d1, 95)),
               "rms_model_to_target_mm": float(np.sqrt((d2 ** 2).mean())),
               "p95_model_to_target_mm": float(np.percentile(d2, 95))}


def T_to_xyz_rpy(T):
    R = T[:3, :3]
    p = math.asin(-max(-1, min(1, R[2, 0])))
    r = math.atan2(R[2, 1], R[2, 2]); y = math.atan2(R[1, 0], R[0, 0])
    return (T[:3, 3] * 1000).round(3).tolist(), [round(r, 5), round(p, 5), round(y, 5)]


def main():
    bases = trimesh.load(EXPORTS / "bases.stl", force="mesh")
    if bases.bounds[1].max() < 5:  # exported in metres
        pass
    else:
        bases.apply_scale(0.001)
    bodies = bases.split(only_watertight=False)
    # URDF base mesh in base_link frame (visual origin from so101_new_calib.urdf)
    base_mesh = trimesh.load(VENDOR / "base_so101_v2.stl")
    base_mesh.apply_transform(rpy_to_T([-0.00636471, -8.97657e-09, -0.0024], [1.5708, 0, 1.5708]))
    vol = base_mesh.volume
    cands = [b for b in bodies if abs(abs(b.volume) - vol) / vol < 0.02]
    cands.sort(key=lambda b: b.centroid[1])
    print(flush=True); print(f"base_so101_v2 volume {vol*1e6:.1f} cm3; matching export bodies: {len(cands)}")
    results = {}
    for name, body in zip(("left", "right"), cands):
        T, res = fit(base_mesh, body)
        xyz, rpy = T_to_xyz_rpy(T)
        results[name] = {"T_chassis_baselink": T.tolist(), "xyz_mm": xyz, "rpy_rad": rpy, **res,
                         "body_centroid_mm": (body.centroid * 1000).round(2).tolist()}
        print(name, "xyz_mm", xyz, "rpy", rpy, {k: round(v, 3) for k, v in res.items()})
    if len(cands) == 2:
        d = np.linalg.norm(np.array(results["left"]["xyz_mm"]) - np.array(results["right"]["xyz_mm"]))
        print("base_link origin distance left-right mm:", round(d, 2))
        results["base_spacing_mm"] = d
    out = ROOT / "robot" / "out" / "system_model"; out.mkdir(parents=True, exist_ok=True)
    json.dump(results, open(out / "base_fit.json", "w"), indent=1)


if __name__ == "__main__":
    main()
