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
  5. the final adjustment to frame 0, then the cycle, pausing before each phase
     (--no-pause to run straight through);
  6. at the end the arms are back at the fold: support them, press Enter, torque goes off.
  Abort any time: Ctrl-C or `touch /tmp/inchworm_stop_move`. The arms then HOLD in
  place (torque on); support them and press Enter to release.

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
_REG_TORQUE_ENABLE = 0x28


def _digest(*paths: Path) -> str:
    h = hashlib.sha256()
    for p in paths:
        h.update(Path(p).read_bytes())
    return h.hexdigest()


def run_preflight(cfg, frames, timeline: Path, cfg_path: Path, use_cache: bool = True) -> List[str]:
    key = _digest(timeline, cfg_path, Path(cr.__file__))
    if use_cache and PREFLIGHT_CACHE.exists():
        c = json.loads(PREFLIGHT_CACHE.read_text())
        if c.get("key") == key and c.get("problems") == []:
            print(f"preflight: cached pass ({c.get('utc')})")
            return []
    from inchworm.motion_guard import build_mesh_collision_guard
    print("preflight: checking every move against the cell model (about 1-2 min) ...", flush=True)
    t0 = time.time()
    mesh = build_mesh_collision_guard(cfg, "left")
    problems = cr.preflight(cfg, frames, None if mesh is None else mesh.checker)
    print(f"preflight: {len(problems)} problem(s) in {time.time() - t0:.0f} s")
    PREFLIGHT_CACHE.parent.mkdir(parents=True, exist_ok=True)
    PREFLIGHT_CACHE.write_text(json.dumps({"key": key, "problems": problems,
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
    ap.add_argument("--no-pause", action="store_true", help="do not pause before each phase")
    ap.add_argument("--skip-sim", action="store_true", help="dry-run: skip the simulated run")
    ap.add_argument("--out", type=Path, default=None)
    a = ap.parse_args(argv)
    if not 20 <= a.speed <= cr.MAX_VMAX_TICKS_S:
        print(f"REFUSED: --speed must be 20..{cr.MAX_VMAX_TICKS_S}"); return 2
    if a.live and not (a.allow_unverified_soft_limits or "").strip():
        print("REFUSED: --live needs --allow-unverified-soft-limits \"<owner approval>\" "
              "(the soft limits are still marked unverified)"); return 2

    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    from inchworm.config import load_config, resolve_config_path
    cfg = load_config()
    frames, notes = cr.clamp_to_eeprom(cfg, cr.load_timeline(a.timeline))
    for n in notes:
        print(f"adjusted: {n}")
    problems = run_preflight(cfg, frames, a.timeline, resolve_config_path())
    if problems:
        print("REFUSED: preflight problems:"); [print("  " + p) for p in problems[:20]]; return 3
    print(cr.summary(frames, a.speed))
    print(f"Abort: Ctrl-C or touch {STOP_FILE}")

    if not a.live:
        if not a.skip_sim:
            import replay_sim
            print("simulated run of the whole cycle (same code as live) ...", flush=True)
            logging.getLogger().setLevel(logging.ERROR)
            recs: List[dict] = []
            try:
                r, buses = replay_sim.simulate(cfg, frames, vmax_ticks_s=a.speed, log=recs.append)
            except Exception as exc:  # noqa: BLE001
                print(f"SIMULATION FAILED: {type(exc).__name__}: {exc}"); return 4
            worst = max(abs(e) for x in recs if x["event"] == "frame" for v in x["error_ticks"].values() for e in v)
            print(f"simulation OK: {sum(x['event'] == 'frame' for x in recs)} moves, "
                  f"{sum(x['event'].startswith('gripper') for x in recs)} gripper actions, worst error {worst} ticks, "
                  f"{buses['left'].clock():.0f} s simulated")
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
        if rec.get("event") != "frame":
            print(json.dumps(rec))
        else:
            err = {k: max(map(abs, v)) for k, v in rec["error_ticks"].items()}
            print(f"  frame {rec['frame']:3d} reached (worst error {err}) in {rec['took_s']} s", flush=True)

    backend = live_backend(cfg)
    transports, buses = {}, {}
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
        appr = cr.approach_problems(cfg, measured, frames[0], None if mesh is None else mesh.checker)
        log({"event": "start", "ticks": ticks, "gripper_ticks": grip, "speed": a.speed,
             "soft_limits_override": a.allow_unverified_soft_limits,
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

        print("\nCONFIRM before anything moves:\n  - NO tube in the grippers or the cell\n"
              "  - DexArm parked where it is now and powered as usual\n  - hands and cables clear of both arms\n"
              "  - you are at the power switch")
        if input("Type YES to start: ").strip() != "YES":
            print("not confirmed; nothing moved"); return 0

        r = cr.build_runner(cfg, frames, buses, live=True,
                            soft_limits_override=a.allow_unverified_soft_limits,
                            vmax_ticks_s=a.speed, should_abort=STOP_FILE.exists,
                            pause_each_phase=not a.no_pause, log=log)
        try:
            r.energise()
            print("locking both grippers (45-degree turn) ...")
            r.lock_both()
            print("final adjustment to the start pose ...")
            r.move_to_start()
            r.run()
            log({"event": "done"})
            print("\nCYCLE COMPLETE. Support both arms (they go limp when torque turns off).")
            input("Press Enter to release torque: ")
        except (cr.ReplayAborted, KeyboardInterrupt, Exception) as exc:  # noqa: BLE001
            what = "Ctrl-C" if isinstance(exc, KeyboardInterrupt) else f"{type(exc).__name__}: {exc}"
            print(f"\nABORTED: {what}\nHolding both arms in place (torque on).")
            log({"event": "aborted", "reason": what, "hold": {k: str(v) for k, v in r.hold().items()}})
            try:
                input("Support both arms, then press Enter to release torque: ")
            except (KeyboardInterrupt, EOFError):
                pass
            return_code = 7
        else:
            return_code = 0
        errs = r.release()
        log({"event": "released", "errors": {k: str(v) for k, v in errs.items() if v}})
        print(f"torque released. Log: {out}")
        return return_code
    finally:
        for b in buses.values():
            try:
                for s in range(1, 7):
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
