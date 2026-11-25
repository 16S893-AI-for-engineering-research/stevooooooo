#!/usr/bin/env python3
"""Export the AA-45 inchworm cycle for the Blender render (offline, no hardware).

Reads ``out/inchworm/timeline.jsonl`` (the validated crawl cycle), computes every
cell-URDF link pose per frame with the project's own kinematics and joint
mapping (the same code the mesh collision checker uses), and re-expresses the
motion in the TUBE frame: the tube stays still and the whole robot crawls along
it. Writes ``out/blender/crawl_data.npz`` (visual meshes in link frames) and
``out/blender/crawl_frames.json`` (per-frame link world poses, step names, lock
states, tube section placements). ``robot/blender/blender_crawl_scene.py`` (run inside Blender) turns that into a
.blend and an MP4.

Run from robot/:
    uv run python scripts/blender_crawl_export.py [--sections 12]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

_ROBOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROBOT / "src"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeline", type=Path, default=_ROBOT / "out" / "inchworm" / "timeline.jsonl")
    ap.add_argument("--out", type=Path, default=_ROBOT / "out" / "blender")
    ap.add_argument("--sections", type=int, default=12,
                    help="tube sections to show (model tube_1 = 3; extra ones are the older print)")
    args = ap.parse_args()

    import trimesh
    from inchworm.config import load_config
    from inchworm.constants import ATTACHMENT_PITCH_MM
    from inchworm.motion_guard import build_mesh_collision_guard
    from inchworm.system_model.collision import TUBE_LINK, CellState
    from inchworm.system_model.urdf import load_geometry_mesh

    cfg = load_config()
    g = build_mesh_collision_guard(cfg, "left")
    if g is None or g.checker is None:
        print(f"mesh model unavailable: {g and g.load_error}", file=sys.stderr)
        return 2
    chk = g.checker
    model = chk.model
    pitch = ATTACHMENT_PITCH_MM / 1000.0
    axis = np.asarray(chk.tube_axis, dtype=float)  # +Y-ish, away from the DexArm

    frames = [json.loads(l) for l in args.timeline.read_text().splitlines() if l.strip()]

    def state(fr):
        A = np.eye(4)
        A[:3, 3] = -axis * fr["tube_advance_m"]
        G = np.asarray(fr["tube_motion"]) @ np.asarray(fr["tube_base"]) @ A
        return CellState(fr["left"], fr["right"], dexarm_q={},
                         tube_advance_m=fr["tube_advance_m"], tube_transform=G)

    # Per frame: cell-frame link poses; tube motion M_f (tube pose = M_f @ P0).
    # Tube-fixed display: W_f = M_0 @ inv(M_f) applied to everything, so the
    # tube sits still at its frame-0 pose and the robot moves around it.
    out_frames = []
    M0 = None
    for fr in frames:
        st = state(fr)
        poses = chk.link_poses(st)
        T = np.eye(4)
        T[:3, 3] = axis * st.tube_advance_m
        M = np.asarray(st.tube_transform) @ T
        if M0 is None:
            M0 = M
        W = M0 @ np.linalg.inv(M)
        out_frames.append({
            "step": fr["step"],
            "left_locked": fr["left_locked"],
            "right_locked": fr["right_locked"],
            "poses": {l: (W @ P).tolist() for l, P in poses.items() if l != TUBE_LINK},
        })
    # link_poses already places the tube at M0 @ P_cad (frame 0, W_0 = I).
    tube_pose = chk.link_poses(state(frames[0]))[TUBE_LINK]

    # Visual meshes in link frames (vertices merged so smooth shading works).
    arrays = {}
    links = []
    for link in model.links:
        gs = model.visuals.get(link, [])
        # Owner 2026-11-24: no DexArm claw gripper in the render (print head only).
        if not gs or link.startswith(("GripperMount", "GripperAxis", "GripperFinger")):
            continue
        parts = [load_geometry_mesh(gm) for gm in gs]
        # Per-face material: 1 = STS3215 servo (rendered black), 0 = printed part.
        mat_id = np.concatenate([
            np.full(len(pm.faces), 1 if gm.mesh_path.name.startswith("sts3215") else 0, np.uint8)
            for gm, pm in zip(gs, parts)])
        m = trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
        m.merge_vertices()  # keeps face order, so mat_id still lines up
        arrays[f"{link}__v"] = np.asarray(m.vertices, dtype=np.float32)
        arrays[f"{link}__f"] = np.asarray(m.faces, dtype=np.int32)
        arrays[f"{link}__m"] = mat_id
        links.append(link)

    # Extended tube: model tube = 3 sections; older sections lie on the side
    # away from the DexArm (+axis), the newest end stays at the DexArm.
    # The displayed tube is rotated by M0 (its start placement on the grippers,
    # ~1 deg yaw from CAD), so the copies must step along the ROTATED axis.
    disp_axis = M0[:3, :3] @ axis
    n_copies = max(1, int(np.ceil(args.sections / 3)))
    tube_sections = []
    for k in range(n_copies):
        S = np.eye(4)
        S[:3, 3] = disp_axis * (3 * pitch * k)
        tube_sections.append((S @ tube_pose).tolist())

    # --- Pretend prints (owner 2026-11-24) ---------------------------------
    # 1) Before the crawl: the last layer of the current section finishes on the
    #    tube's newest end face (this is what triggers the regrip).
    # 2) After the crawl: printing restarts on the same face, one layer on top.
    #    The tube never moves in this view; the robot (and the DexArm with it)
    #    sits one pitch farther back, so the head reaches farther out.
    # Each print = a spiral bead on the face annulus; the head follows it by
    # position IK on the 3 DexArm joints with the cell URDF FK, expressed in the
    # robot's display pose of that moment. Visual only: no G-code, not
    # collision-checked, not a plan.
    from scipy.optimize import least_squares
    tv = arrays[f"{TUBE_LINK}__v"].astype(float)
    tw = tv @ np.asarray(tube_pose)[:3, :3].T + np.asarray(tube_pose)[:3, 3]
    s_ax = tw @ disp_axis
    face_s = float(s_ax.min())
    rim = tw[s_ax < face_s + 0.003]
    centre = rim.mean(0)
    centre = centre - disp_axis * ((centre @ disp_axis) - face_s)
    r_out = float(np.median(np.linalg.norm(
        (rim - centre) - np.outer((rim - centre) @ disp_axis, disp_axis), axis=1)[
        np.linalg.norm(rim - centre, axis=1) > 0.032]))
    e1 = np.cross(disp_axis, [0.0, 0.0, 1.0]); e1 /= np.linalg.norm(e1)
    e2 = np.cross(disp_axis, e1)
    BEAD_R = 0.00175
    q_names = ("bases", "B_Rot->In_1", "Join->In_2")
    lim = [(model.joints[j].lower, model.joints[j].upper) for j in q_names]
    bnd = ([l[0] for l in lim], [l[1] for l in lim])
    P0 = model.link_poses({})
    ee_v = arrays["EE__v"].astype(float) @ P0["EE"][:3, :3].T + P0["EE"][:3, 3]
    head_front = max(float((arrays[l + "__v"].astype(float) @ P0[l][:3, :3].T + P0[l][:3, 3]).max(0) @ disp_axis)
                     for l in ("EE", "lasercamera"))
    front = ee_v[ee_v @ disp_axis > ee_v.max(0) @ disp_axis - 0.0015]
    nozzle_l = np.linalg.inv(P0["EE"]) @ np.r_[front.mean(0), 1.0]  # head's tube-facing face
    lead = head_front - float(front.mean(0) @ disp_axis)  # parts ahead of the nozzle
    dex_links = [l for l in ("B_Rot", "In_1", "Join", "In_2", "EE", "lasercamera", "Out_1", "Out_2")
                 if l in links]
    r0, r1 = r_out - 0.002, r_out - 0.010  # inside the outer wall
    turns, n_s = 3.0, 240

    def pretend_print(W, layer):
        """Spiral bead `layer` beads out from the face; head poses in display frame W."""
        face_off = disp_axis * (2 * BEAD_R * layer)
        gap = 2 * BEAD_R + lead + 0.0005  # nozzle clears its bead, head clears the face
        bead, poses, q, worst = [], [], None, 0.0
        for k in range(n_s):
            u = k / (n_s - 1)
            th = 2 * np.pi * turns * u
            ring = centre - face_off + (r0 + (r1 - r0) * u) * (np.cos(th) * e1 + np.sin(th) * e2)
            bead.append((ring - disp_axis * BEAD_R).tolist())
            tgt = ring - disp_axis * gap

            def res(x):
                return (W @ model.link_poses(dict(zip(q_names, x)))["EE"] @ nozzle_l)[:3] - tgt
            seeds = [q] if q is not None else [np.array(x) for x in (
                [0, .2, 0], [.3, .5, .3], [-.3, .5, .3], [0, 1, -.5], [0, 1.2, .9])]
            sol = min((least_squares(res, x0, bounds=bnd) for x0 in seeds), key=lambda r_: r_.cost)
            q = sol.x
            worst = max(worst, float(np.linalg.norm(sol.fun)))
            Pq = model.link_poses(dict(zip(q_names, q)))
            poses.append({l: (W @ Pq[l]).tolist() for l in dex_links})
        return bead, poses, worst

    W_first = np.eye(4)  # display = cell frame at frame 0
    W_last = np.asarray(out_frames[-1]["poses"]["B_Fix"]) @ np.linalg.inv(P0["B_Fix"])
    prints = []
    for name, W, layer in (("final layer", W_first, 0), ("next section", W_last, 1)):
        bead, poses, worst = pretend_print(W, layer)
        if worst > 1e-3:
            print(f"pretend print ({name}): DexArm cannot reach the bead path "
                  f"(worst {worst * 1000:.1f} mm)", file=sys.stderr)
            return 3
        print(f"pretend print ({name}): spiral {r0 * 1000:.1f}->{r1 * 1000:.1f} mm on the face, "
              f"layer {layer}, worst IK residual {worst * 1000:.2f} mm")
        prints.append({"name": name, "bead_pts": bead, "poses": poses})
    print_meta = {"bead_r": BEAD_R, "prints": prints}

    args.out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out / "crawl_data.npz", **arrays)
    meta = {
        "links": links,
        "tube_link": TUBE_LINK,
        "tube_sections": tube_sections,
        "tube_axis": disp_axis.tolist(),
        "pitch_m": pitch,
        "frames": out_frames,
        "print": print_meta,
    }
    (args.out / "crawl_frames.json").write_text(json.dumps(meta))
    base0 = np.array(out_frames[0]["poses"]["baseforreal"])[:3, 3]
    base1 = np.array(out_frames[-1]["poses"]["baseforreal"])[:3, 3]
    print(f"{len(out_frames)} frames, {len(links)} links, {n_copies * 3} tube sections")
    print(f"robot base travel along the tube: {(base1 - base0) @ disp_axis * 1000:+.1f} mm "
          f"(pitch {pitch * 1000:.1f} mm)")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
