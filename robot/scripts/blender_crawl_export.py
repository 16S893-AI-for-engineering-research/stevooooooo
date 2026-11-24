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
    tube_pose = M0 @ chk.link_poses(state(frames[0]))[TUBE_LINK]

    # Visual meshes in link frames (vertices merged so smooth shading works).
    arrays = {}
    links = []
    for link in model.links:
        gs = model.visuals.get(link, [])
        if not gs:
            continue
        parts = [load_geometry_mesh(gm) for gm in gs]
        m = trimesh.util.concatenate(parts) if len(parts) > 1 else parts[0]
        m.merge_vertices()
        arrays[f"{link}__v"] = np.asarray(m.vertices, dtype=np.float32)
        arrays[f"{link}__f"] = np.asarray(m.faces, dtype=np.int32)
        links.append(link)

    # Extended tube: model tube = 3 sections; older sections lie on the side
    # away from the DexArm (+axis), the newest end stays at the DexArm.
    n_copies = max(1, int(np.ceil(args.sections / 3)))
    tube_sections = []
    for k in range(n_copies):
        S = np.eye(4)
        S[:3, 3] = axis * (3 * pitch * k)
        tube_sections.append((S @ tube_pose).tolist())

    args.out.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.out / "crawl_data.npz", **arrays)
    meta = {
        "links": links,
        "tube_link": TUBE_LINK,
        "tube_sections": tube_sections,
        "tube_axis": axis.tolist(),
        "pitch_m": pitch,
        "frames": out_frames,
    }
    (args.out / "crawl_frames.json").write_text(json.dumps(meta))
    base0 = np.array(out_frames[0]["poses"]["baseforreal"])[:3, 3]
    base1 = np.array(out_frames[-1]["poses"]["baseforreal"])[:3, 3]
    print(f"{len(out_frames)} frames, {len(links)} links, {n_copies * 3} tube sections")
    print(f"robot base travel along the tube: {(base1 - base0) @ axis * 1000:+.1f} mm "
          f"(pitch {pitch * 1000:.1f} mm)")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
