"""AA-50: replay the planned inchworm cycle on the real arms WITHOUT the tube.

Owner request 2026-11-12: a full movement test of both arms, the whole cycle, no tube, with
the grippers unlocking and relocking (45-degree position lock) at the regrips.

Dry-run (default, no hardware):
    uv run python scripts/replay_cycle.py
  loads out/inchworm/timeline.jsonl, pulls goals past the EEPROM limits inside them,
  runs the offline preflight (both arms together, cell mesh model, no tube; cached per
  timeline + config), then plays the whole cycle against simulated servos with the same
  code the live run uses, and prints the plan.

Live (owner present, hand on the power switch):
    uv run python scripts/replay_cycle.py --live \\
        --allow-unverified-soft-limits "owner 2026-11-13: AA-50 no-tube replay"
  1. refuses unless all 12 servos are torque-off and drive registers are sane;
  2. reads the pose; if every joint is within 10 deg of frame 0 and the straight move there
     is clear in the cell model, the robot makes the final adjustment itself (step 5);
  3. asks you to confirm: no tube, DexArm parked, hands clear;
  4. slow profile on; both grippers lock (45-degree turn; look that they turn);
  5. the final adjustment to frame 0, then the whole cycle with no further prompts. Default
     (owner 2026-11-13): FLUID motion, both arms at the same time, never stopping between
     frames, at --tempo x the animation speed (default 1.0, slowed only where a joint would pass ~57 deg/s); the arms rest only while a
     gripper unlocks or locks. --step-by-step = the old frame-by-frame replay at --speed;
  6. at the end the arms are back at the fold: support them, press Enter, torque goes off.
  Abort any time: Ctrl-C or `touch /tmp/inchworm_stop_move`. The arms then HOLD in
  place (torque on); support them and press Enter to release.

Demo start with the tube (AA-23 / AA-44, owner decisions 2026-11-15):
    uv run python scripts/replay_cycle.py --stop-after-frame 42 [--tube-in-grippers] --live ...
  plays frames 0..42 (to the end of the left arm's "move under target", 50 mm under its next
  attachment point, right gripper LOCKED) and STOPS there with both arms HOLDING (torque on)
  after the script exits. It writes out/handoff_state.json for the LeRobot follower's
  tube_support=handoff mode. --tube-in-grippers: the tube's attachment keys sit in the open
  grippers before the start; every preflight then models the tube where the timeline puts it,
  the fold press is skipped, and no path of this script ever switches a gripper's torque off.
  Without it (first live step, owner 2026-11-15) the run is the no-tube rehearsal that checks
  the servos keep holding after the script exits.

Every arm goal goes through GuardedArm (soft-limit values, keep-out boxes, mesh collision
with the other arm's target pose). The soft limits are still marked unverified; the owner
waived only that gate for this test (2026-11-13), and the waiver text is logged.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import logging
import sys
import time
from pathlib import Path
from typing import List, Optional

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from inchworm import cycle_replay as cr  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
TIMELINE = ROOT / "out" / "inchworm" / "timeline.jsonl"
PREFLIGHT_CACHE = ROOT / "out" / "inchworm" / "replay_preflight.json"
STOP_FILE = Path("/tmp/inchworm_stop_move")
HANDOFF_STATE = ROOT / "out" / "handoff_state.json"


def stop_frame_problems(frames, n: int) -> List[str]:
    """Owner 2026-11-15: the stop leaves the left gripper open and the right one LOCKED."""
    if not 1 <= n < len(frames) - 1:
        return [f"--stop-after-frame must be 1..{len(frames) - 2}"]
    f = frames[n]
    out = []
    if not any(f.locked.values()) or all(f.locked.values()):
        out.append(f"frame {n} ({f.step}): exactly one gripper must be open at the handoff "
                   f"(locked: {f.locked})")
    if f.tube is None:
        out.append(f"frame {n}: the timeline has no tube pose (the handoff mesh check needs it)")
    return out


def handoff_record(cfg, r, frames, a, buses_read: bool = False) -> dict:
    """Read the stop pose and write the handoff state file (dry-run: a sim_ copy next to it)."""
    from inchworm import handoff as ho
    f = frames[-1]
    present = {x: r._ticks(x) for x in cr.ARMS}
    grip = {x: round(r.arms[x].bus.read_position_deg(6) * 4096 / 360) for x in cr.ARMS}
    path = a.handoff_state if not buses_read else a.handoff_state.with_name("sim_" + a.handoff_state.name)
    return ho.write_state(path, stop_frame=f.index, step=f.step, timeline_digest=_digest(a.timeline),
                          tube_in_grippers=a.tube_in_grippers,
                          goal_ticks={x: cr.q_to_ticks(cfg, x, f.q[x]) for x in cr.ARMS},
                          present_ticks=present, gripper_ticks=grip, locked=f.locked,
                          q_rad={x: f.q[x] for x in cr.ARMS}, tube_pose=f.tube)
_REG_TORQUE_ENABLE = 0x28


def _digest(*paths: Path) -> str:
    h = hashlib.sha256()
    for p in paths:
        h.update(Path(p).read_bytes())
    return h.hexdigest()


def run_preflight(cfg, frames, timeline: Path, cfg_path: Path, use_cache: bool = True,
                  tube: bool = False) -> List[str]:
    key = _digest(timeline, cfg_path, Path(cr.__file__)) + f":n{len(frames)}:tube{int(tube)}"
    if use_cache and PREFLIGHT_CACHE.exists():
        c = json.loads(PREFLIGHT_CACHE.read_text())
        if c.get("key") == key and c.get("problems") == []:
            print(f"preflight: cached pass ({c.get('utc')})")
            return []
    from inchworm.motion_guard import build_mesh_collision_guard
    print("preflight: checking every move against the cell model (about 1-2 min) ...", flush=True)
    t0 = time.time()
    mesh = build_mesh_collision_guard(cfg, "left")
    problems = cr.preflight(cfg, frames, None if mesh is None else mesh.checker, tube=tube)
    print(f"preflight: {len(problems)} problem(s) in {time.time() - t0:.0f} s")
    PREFLIGHT_CACHE.parent.mkdir(parents=True, exist_ok=True)
    PREFLIGHT_CACHE.write_text(json.dumps({"key": key, "problems": problems,
                                           "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}))
    return problems


def run_stream_preflight(cfg, segs, timeline: Path, cfg_path: Path, tempo: float,
                         tube_frames=None) -> List[str]:
    """Owner option (b) 2026-11-13: the whole fluid path checked before the start prompt.
    ``tube_frames``: the tube is in the grippers, placed per these timeline frames."""
    n = segs[-1].last if segs else 0
    key = (_digest(timeline, cfg_path, Path(cr.__file__)) + f":fluid:{tempo:g}:to{n}"
           f":tube{int(tube_frames is not None)}")
    cache = PREFLIGHT_CACHE.with_name("replay_stream_preflight.json")
    if cache.exists():
        c = json.loads(cache.read_text())
        if c.get("key") == key and c.get("problems") == []:
            print(f"fluid-path preflight: cached pass ({c.get('utc')})")
            return []
    from inchworm.motion_guard import build_mesh_collision_guard
    print("fluid-path preflight: checking the whole path every 25 ms, both arms together (about 30 s) ...",
          flush=True)
    t0 = time.time()
    mesh = build_mesh_collision_guard(cfg, "left")
    problems = cr.stream_preflight(cfg, segs, None if mesh is None else mesh.checker, frames=tube_frames)
    print(f"fluid-path preflight: {len(problems)} problem(s) in {time.time() - t0:.0f} s")
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps({"key": key, "problems": problems,
                                 "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}))
    return problems


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeline", type=Path, default=TIMELINE)
    ap.add_argument("--speed", type=int, default=cr.DEFAULT_VMAX_TICKS_S,
                    help=f"ticks/s of the joint that moves most per frame (max {cr.MAX_VMAX_TICKS_S})")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--allow-unverified-soft-limits", metavar="OWNER_APPROVAL", default=None,
                    help="owner approval text; required for --live while the soft limits are unverified")
    ap.add_argument("--tempo", type=float, default=cr.DEFAULT_TEMPO,
                    help=f"fluid mode: fraction of the animation speed (0.1..{cr.MAX_TEMPO}, default {cr.DEFAULT_TEMPO})")
    ap.add_argument("--step-by-step", action="store_true",
                    help="frame-by-frame replay (each frame reached before the next) at --speed")
    ap.add_argument("--no-fold-press", action="store_true",
                    help="AA-52: do NOT rest the lower arms on their shoulders at the start and end")
    ap.add_argument("--stop-after-frame", type=int, default=None, metavar="N",
                    help="AA-23 demo start: play frames 0..N, then stop with both arms HOLDING after exit "
                         "and write out/handoff_state.json (owner 2026-11-15: N = 42)")
    ap.add_argument("--tube-in-grippers", action="store_true",
                    help="the tube's keys are in the open grippers at the start: preflights model the tube, "
                         "no fold press, grippers are never switched off; needs --stop-after-frame")
    ap.add_argument("--handoff-state", type=Path, default=None, help="handoff state path (default out/handoff_state.json)")
    ap.add_argument("--skip-sim", action="store_true", help="dry-run: skip the simulated run")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    if not 0.1 <= a.tempo <= cr.MAX_TEMPO:
        print(f"REFUSED: --tempo must be 0.1..{cr.MAX_TEMPO}"); return 2
    if not 20 <= a.speed <= cr.MAX_VMAX_TICKS_S:
        print(f"REFUSED: --speed must be 20..{cr.MAX_VMAX_TICKS_S}"); return 2
    if a.live and not (a.allow_unverified_soft_limits or "").strip():
        print("REFUSED: --live needs --allow-unverified-soft-limits \"<owner approval>\" "
              "(the soft limits are still marked unverified)"); return 2

    if a.tube_in_grippers and a.stop_after_frame is None:
        print("REFUSED: --tube-in-grippers is only for the demo start (--stop-after-frame N); "
              "the full cycle with the tube is a separate task"); return 2
    if a.tube_in_grippers and a.step_by_step:
        print("REFUSED: --tube-in-grippers runs only the fluid mode (its whole path is checked with the tube "
              "before the YES; the step-by-step per-frame guard models no tube)"); return 2
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    from inchworm.config import load_config, resolve_config_path
    cfg = load_config()
    frames, notes = cr.clamp_to_eeprom(cfg, cr.load_timeline(a.timeline), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    for n in notes:
        print(f"adjusted: {n}")
    a.handoff_state = a.handoff_state or HANDOFF_STATE
    if a.stop_after_frame is not None:
        probs = stop_frame_problems(frames, a.stop_after_frame)
        if probs:
            print("REFUSED: " + "; ".join(probs)); return 2
        frames = frames[:a.stop_after_frame + 1]
        f = frames[-1]
        print(f"demo start: frames 0..{f.index} then STOP holding ({f.step}; locked: "
              + ", ".join(f"{x} {'LOCKED' if f.locked[x] else 'open'}" for x in cr.ARMS)
              + f"); tube {'IN the grippers (modelled in every check)' if a.tube_in_grippers else 'NOT in the cell'}")
        if a.tube_in_grippers and not a.no_fold_press:
            print("fold press: skipped (tube in the grippers)")
            a.no_fold_press = True
    problems = run_preflight(cfg, frames, a.timeline, resolve_config_path(), tube=a.tube_in_grippers)
    if problems:
        print("REFUSED: preflight problems:"); [print("  " + p) for p in problems[:20]]; return 3
    print(cr.summary(frames, a.speed))
    a.fold_press = not a.no_fold_press
    if a.fold_press:
        from inchworm.motion_guard import build_mesh_collision_guard
        mesh = build_mesh_collision_guard(cfg, "left")
        chk = None if mesh is None else mesh.checker
        problems = [p for f in ((frames[0],) if a.stop_after_frame is not None else (frames[0], frames[-1]))
                    for p in cr.fold_press_preflight(cfg, f, chk)]
        if problems:
            print("REFUSED: fold-press preflight problems:"); [print("  " + p) for p in problems[:20]]; return 3
        print(f"fold press (AA-52) at the start" + ("" if a.stop_after_frame is not None else " and the end") + f": elbows only, {cr.FOLD_PRESS_SPEED_TICKS_S} ticks/s, "
              f"torque limit {cr.FOLD_PRESS_TORQUE_LIMIT}/1000, goal {cr.FOLD_PRESS_GOAL_TICKS} "
              f"(measured contact {cr.FOLD_CONTACT_TICKS}); preflight OK")
    a.segments = None
    if not a.step_by_step:
        a.segments = cr.build_segments(frames, a.tempo)
        problems = run_stream_preflight(cfg, a.segments, a.timeline, resolve_config_path(), a.tempo,
                                        tube_frames=frames if a.tube_in_grippers else None)
        if problems:
            print("REFUSED: fluid-path preflight problems:"); [print("  " + p) for p in problems[:20]]; return 3
        print(f"fluid motion at {a.tempo:g} x the animation speed: "
              f"{sum(s.duration_s for s in a.segments):.0f} s of arm motion in {len(a.segments)} pieces, "
              f"peak joint speed {cr.peak_speed_ticks_s(a.segments) * 360 / 4096:.0f} deg/s; "
              f"the arms rest only for the gripper actions")
    print(f"Abort: Ctrl-C or touch {STOP_FILE}")

    if not a.live:
        if not a.skip_sim:
            import replay_sim
            print("simulated run of the whole cycle (same code as live) ...", flush=True)
            logging.getLogger().setLevel(logging.ERROR)
            recs: List[dict] = []
            try:
                contact = ({arm: {3: cr.FOLD_CONTACT_TICKS[arm]} for arm in cr.ARMS}
                           if a.fold_press else None)     # the shoulder stops the elbow (one-sided)
                r, buses = replay_sim.simulate(cfg, frames, vmax_ticks_s=a.speed, log=recs.append,
                                               segments=a.segments, fold_press=a.fold_press,
                                               contact=contact, end_press=a.stop_after_frame is None)
            except Exception as exc:  # noqa: BLE001
                print(f"SIMULATION FAILED: {type(exc).__name__}: {exc}"); return 4
            if a.segments is not None:
                worst = max(v for x in recs if x["event"] == "segment" for v in x["worst_path_error_ticks"].values())
            else:
                worst = max(abs(e) for x in recs if x["event"] == "frame" for v in x["error_ticks"].values() for e in v)
            print(f"simulation OK: {sum(x['event'] in ('frame', 'segment') for x in recs)} moves, "
                  f"{sum(x['event'].startswith('gripper') for x in recs)} gripper actions, worst error {worst} ticks, "
                  f"{buses['left'].clock():.0f} s simulated")
            for x in recs:
                if x["event"] == "fold_press":
                    print(f"  fold press {x['where']}: " + "; ".join(
                        f"{arm} elbow {x[arm]['elbow_ticks']} ({x[arm]['short_of_contact_ticks']} short)"
                        for arm in cr.ARMS) + (f"  WARNINGS {x['warnings']}" if x["warnings"] else ""))
            if a.stop_after_frame is not None:
                st = handoff_record(cfg, r, frames, a, buses_read=True)
                print(f"simulated stop at frame {st['stop_frame']}: both arms holding (torque on: "
                      + ", ".join(f"{x} {all(buses[x].torque[s] for s in range(1, 6))}" for x in cr.ARMS)
                      + "); the live run writes " + str(a.handoff_state))
        print("DRY-RUN: nothing opened or moved. Add --live with the owner present.")
        return 0
    return live(cfg, frames, a)


def live(cfg, frames, a) -> int:
    from inchworm.backends import live_backend
    from inchworm.servo_bus import ArmServoBus
    from inchworm.start_pose_check import compare_start_pose
    from inchworm.so101_pose_reader import ticks_to_q
    from gripper_lock_probe import drive_config_faults

    STOP_FILE.unlink(missing_ok=True)
    out = a.out or ROOT / "calibration_data" / f"replay_cycle_{time.strftime('%Y-%m-%d')}.jsonl"
    out.parent.mkdir(parents=True, exist_ok=True)
    fh = open(out, "a")

    def log(rec):
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), **rec}
        fh.write(json.dumps(rec) + "\n"); fh.flush()
        if rec.get("event") == "fold_press":
            print(f"  fold press {rec['where']}: " + "; ".join(
                f"{arm} elbow {rec[arm].get('elbow_ticks')} ({rec[arm].get('short_of_contact_ticks')} short of "
                f"contact, load {rec[arm].get('load_abs_mean')}, current {rec[arm].get('current_raw_mean')})"
                for arm in cr.ARMS) + (f"  WARNINGS {rec['warnings']}" if rec["warnings"] else ""), flush=True)
            return
        if rec.get("event") == "stream":
            return
        if rec.get("event") == "segment":
            print(f"  frames {rec['frames'][0]}-{rec['frames'][1]} done in {rec['took_s']} s "
                  f"(planned {rec['planned_s']}; worst lag {rec['worst_path_error_ticks']} ticks; "
                  f"max temp read {rec['max_temp_read_c']} C)", flush=True)
        elif rec.get("event") != "frame":
            print(json.dumps(rec))
        else:
            err = {k: max(map(abs, v)) for k, v in rec["error_ticks"].items()}
            print(f"  frame {rec['frame']:3d} reached (worst error {err}) in {rec['took_s']} s", flush=True)

    backend = live_backend(cfg)
    transports, buses = {}, {}
    keep = {"holding": False}
    try:
        for arm in cr.ARMS:
            role = f"arm_{arm}"
            sj = getattr(cfg.arms, arm).servo_joints
            jm = {n: getattr(sj, n) for n in (*cr.JOINTS, "gripper")}
            if list(jm.values()) != [1, 2, 3, 4, 5, 6]:
                print(f"REFUSED: {role}: expected servo IDs 1-6 in joint order, got {jm}"); return 4
            transports[arm] = backend.open_serial(role)
            buses[arm] = ArmServoBus(transports[arm], role=role, joint_id_map=jm)
        for arm, b in buses.items():
            raw = b._bus
            on = [s for s in range(1, 7) if raw.read_bytes(s, _REG_TORQUE_ENABLE, 1)[0]]
            if on:
                print(f"REFUSED: arm_{arm}: torque is ON on servos {on}; switch it off first"); return 4
            bad = [f"servo {s}: {f}" for s in range(1, 7) for f in drive_config_faults(raw.read_bytes, s)
                   if not f.startswith("torque_limit")]
            if bad:
                print(f"REFUSED: arm_{arm} drive config: {bad}"); return 5
        ticks = {arm: [round(buses[arm].read_position_deg(s) * 4096 / 360) for s in range(1, 6)] for arm in cr.ARMS}
        grip = {arm: round(buses[arm].read_position_deg(6) * 4096 / 360) for arm in cr.ARMS}
        measured = {arm: ticks_to_q(cfg, arm, ticks[arm]) for arm in cr.ARMS}
        res = compare_start_pose(measured, {arm: list(frames[0].q[arm]) for arm in cr.ARMS})
        from inchworm.motion_guard import build_mesh_collision_guard
        mesh = build_mesh_collision_guard(cfg, "left")
        appr = cr.approach_problems(cfg, measured, frames[0], None if mesh is None else mesh.checker,
                                    tube=a.tube_in_grippers)
        log({"event": "start", "ticks": ticks, "gripper_ticks": grip, "speed": a.speed,
             "mode": "step-by-step" if a.segments is None else f"fluid tempo {a.tempo:g}",
             "soft_limits_override": a.allow_unverified_soft_limits,
             "stop_after_frame": a.stop_after_frame, "tube_in_grippers": a.tube_in_grippers,
             "diff_to_start_deg": res.diff_deg, "approach_problems": appr})
        if appr:
            print("REFUSED: the robot will not adjust to the start pose from here:")
            for p in appr:
                print("  " + p)
            return 6
        print(f"Start pose: the robot will make the final adjustment itself (each joint within "
              f"{cr.APPROACH_MAX_DEG:.0f} deg; move checked against the cell model). Degrees to move:")
        for arm in cr.ARMS:
            print(f"  {arm:5}: " + "  ".join(f"{j} {-d:+.1f}" for j, d in zip(cr.JOINTS, res.diff_deg[arm])))

        tube_line = ("  - the tube's attachment keys are IN both open grippers (they lock on it next)\n"
                     "  - the DexArm head is CLEAR of the tube and of its 170 mm lift path (the model does "
                     "not know the DexArm pose; in its model pose the head touches the tube)\n"
                     if a.tube_in_grippers else "  - NO tube in the grippers or the cell\n")
        stop_line = ("" if a.stop_after_frame is None else
                     f"  - the run STOPS at frame {frames[-1].index} and both arms KEEP HOLDING after this "
                     "script exits (the arm power switch is the stop)\n")
        print("\nCONFIRM before anything moves:\n" + tube_line
              + "  - DexArm parked where it is now and powered as usual\n  - hands and cables clear of both arms\n"
              + stop_line + "  - you are at the power switch")
        if input("Type YES to start: ").strip() != "YES":
            print("not confirmed; nothing moved"); return 0

        r = cr.build_runner(cfg, frames, buses, live=True,
                            soft_limits_override=a.allow_unverified_soft_limits,
                            vmax_ticks_s=a.speed, should_abort=STOP_FILE.exists, log=log,
                            abort_on_faults=a.segments is None)
        try:
            r.energise()
            print("locking both grippers (45-degree turn) ...")
            r.lock_both()
            print("final adjustment to the start pose ...")
            r.move_to_start()
            if a.fold_press:
                print("fold press: resting both lower arms on their shoulders (15 % elbow torque) ...")
                r.fold_press(frames[0], "start")
                if input("Do BOTH lower arms rest on their shoulders now? [y/n]: ").strip().lower() == "y":
                    obs = "y"
                else:
                    obs = "n"
                log({"event": "owner_observation", "where": "start", "both_resting": obs})
                r.fold_unpress(frames[0])
            if a.segments is None:
                r.run()
            else:
                print("running the cycle (fluid, no more prompts). It stops only for Ctrl-C or the stop "
                      "file, or if a gripper fails to lock; readings are logged, never acted on.")
                r.run_fluid(a.segments)
            if a.stop_after_frame is not None:
                st = handoff_record(cfg, r, frames, a)
                log({"event": "handoff_stop", **st})
                keep["holding"] = True
                print(f"\nSTOPPED at frame {st['stop_frame']} ({st['step']}). Both arms are HOLDING (torque on) "
                      f"and keep holding after this script exits.\nState: {a.handoff_state}\n"
                      "Next: the LeRobot follower with --robot.tube_support=handoff. To end instead: support "
                      "the tube and both arms, then switch the arm power off or run\n"
                      "  uv run python scripts/hold_tube.py release --arm left --live  (and --arm right)")
                return 0
            if a.fold_press:
                print("fold press at the end ...")
                r.fold_press(frames[-1], "end")
                obs = input("Do BOTH lower arms rest on their shoulders now? [y/n]: ").strip().lower()
                log({"event": "owner_observation", "where": "end", "both_resting": "y" if obs == "y" else "n"})
            log({"event": "done"})
            print("\nCYCLE COMPLETE. Support both arms (they go limp when torque turns off).")
            input("Press Enter to release torque: ")
        except (cr.ReplayAborted, KeyboardInterrupt, Exception) as exc:  # noqa: BLE001
            what = "Ctrl-C" if isinstance(exc, KeyboardInterrupt) else f"{type(exc).__name__}: {exc}"
            print(f"\nABORTED: {what}\nHolding both arms in place (torque on).")
            log({"event": "aborted", "reason": what, "hold": {k: str(v) for k, v in r.hold().items()}})
            try:
                input(("Support the TUBE and both arms" if a.tube_in_grippers else "Support both arms")
                      + ", then press Enter to release torque: ")
            except (KeyboardInterrupt, EOFError):
                pass
            return_code = 7
        else:
            return_code = 0
        errs = r.release(grippers=not a.tube_in_grippers)
        log({"event": "released", "errors": {k: str(v) for k, v in errs.items() if v}})
        print(f"torque released. Log: {out}")
        return return_code
    finally:
        # never leave a joint energised when the program exits, EXCEPT the deliberate demo-start
        # stop (both arms hold for the handoff). With the tube the grippers are never switched off.
        for b in ([] if keep["holding"] else buses.values()):
            try:
                for s in range(1, 6 if a.tube_in_grippers else 7):
                    if b._bus.read_bytes(s, _REG_TORQUE_ENABLE, 1)[0]:
                        # never leave a joint energised when the program exits
                        b._bus.write_bytes(s, _REG_TORQUE_ENABLE, b"\x00")
            except Exception as exc:  # noqa: BLE001
                print(f"WARNING: final torque-off sweep failed: {exc}")
        for t in transports.values():
            t.close()
        fh.close()


if __name__ == "__main__":
    raise SystemExit(main())
