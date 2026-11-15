#!/usr/bin/env python3
"""AA-44 AC #4: supervised slow leader -> follower teleop (mapping check). Dry-run by default.

The follower (default left) follows the dedicated leader arm, joints 1-5 only. The gripper is
never commanded. The other follower arm is NOT touched (leave it torque-off where it is, no
tube for this check).

    uv run python scripts/teleop_live.py                      # dry-run: reads nothing, explains
    uv run python scripts/teleop_live.py --check              # read-only: start gate vs leader, no torque
    uv run python scripts/teleop_live.py --live \\
        --allow-unverified-soft-limits "owner YYYY-MM-DD: AA-44 teleop check" \\
        --no-collision-check "owner 2026-11-13: first teleop without mesh check"

Live sequence: refuse unless the follower is torque-off with sane drive registers, the leader
map is fitted, and every joint's mapped target is within 10 deg of the follower's pose (match
the LEADER to the follower). Then CONFIRM, then torque on at --speed (default 150 ticks/s,
~13 deg/s) and the follower tracks the leader at --hz (default 30). Limits are clamped (soft
limits within the EEPROM limits, 10 ticks inside). Ctrl-C or the stop file = HOLD in place;
support the arm and press Enter to release torque. Log: calibration_data/teleop_live_<date>.jsonl.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
STOP_FILE = ROOT / "out" / "STOP_TELEOP"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--follower", choices=("left", "right"), default="left")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--check", action="store_true", help="read-only start-gate check, no torque")
    ap.add_argument("--speed", type=int, default=150, help="Goal_Speed ticks/s (max 600)")
    ap.add_argument("--hz", type=float, default=30.0)
    ap.add_argument("--smooth", type=float, default=None,
                    help="leader filter 0..1 (1 = off; default 0.35, lower = smoother but more lag)")
    ap.add_argument("--allow-unverified-soft-limits", default=None, metavar="OWNER_APPROVAL")
    ap.add_argument("--no-collision-check", default=None, metavar="OWNER_REASON")
    ap.add_argument("--map", type=Path, default=None)
    a = ap.parse_args(argv)
    if not (a.live or a.check):
        print("DRY-RUN: nothing opened. --check = read-only start gate; --live = supervised slow teleop "
              f"of the {a.follower} follower from the leader (joints 1-5, gripper never commanded).")
        return 0
    if not 1 <= a.speed <= 600 or not 1 <= a.hz <= 60:
        print("REFUSED: --speed 1..600, --hz 1..60"); return 4

    from inchworm import teleop_live as tl
    from inchworm.backends import live_backend
    from inchworm.config import load_config
    from inchworm.servo_bus import ArmServoBus, _ServoBus
    from gripper_lock_probe import drive_config_faults

    cfg = load_config()
    if a.live:
        if not a.no_collision_check:
            print("REFUSED: this path has no mesh collision check; pass --no-collision-check \"<owner reason>\""); return 4
        lim_cfg = getattr(cfg.motion_guard, f"{a.follower}_arm_limits")
        if getattr(lim_cfg, "unverified", True) and not a.allow_unverified_soft_limits:
            print("REFUSED: soft limits are unverified; pass --allow-unverified-soft-limits \"<owner approval>\""); return 4
    try:
        mapping = tl.load_leader_map(a.map or ROOT / "calibration_data" / f"leader_map_{a.follower}.json", a.follower)
    except tl.TeleopRefused as e:
        print(f"REFUSED: {e}"); return 4

    out = ROOT / "calibration_data" / f"teleop_live_{time.strftime('%Y-%m-%d')}.jsonl"
    fh = open(out, "a")

    def log(rec):
        rec = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), **rec}
        fh.write(json.dumps(rec) + "\n"); fh.flush()
        if rec.get("event") == "teleop_stats":
            print(f"  t {rec['t']:6.1f}s  {rec['hz']:5.1f} Hz  lag {rec['worst_lag_ticks']}  "
                  f"clamps {sum(rec['clamps'].values())}", flush=True)
        else:
            print(json.dumps(rec), flush=True)

    be = live_backend(cfg)
    tlead = be.open_serial("arm_leader")
    role = f"arm_{a.follower}"
    tfol = be.open_serial(role)
    fbus = ArmServoBus(tfol, role=role, joint_id_map={n: i + 1 for i, n in enumerate((*tl.J5, "gripper"))})
    lraw, fraw = _ServoBus(tlead), fbus._bus
    rc = 0
    try:
        on = [s for s in range(1, 7) if fraw.read_bytes(s, tl.REG_TORQUE_ENABLE, 1)[0]]
        if on:
            print(f"REFUSED: {role}: torque is ON on servos {on}; switch it off first"); return 4
        bad = [f"servo {s}: {f}" for s in range(1, 6) for f in drive_config_faults(fraw.read_bytes, s)
               if not f.startswith("torque_limit")]
        if bad:
            print(f"REFUSED: {role} drive config: {bad}"); return 5
        try:
            yaml_ticks = getattr(cfg.motion_guard, f"{a.follower}_arm_limits").servo_tick_limits or {}
            yaml_lim = {j: (yaml_ticks[j][0] + tl.EEPROM_INSIDE_TICKS, yaml_ticks[j][1] - tl.EEPROM_INSIDE_TICKS)
                        for j in tl.J5}   # hardware.yaml tick limits (tighter than EEPROM on pan/lift)
            limits = tl.intersect(tl.intersect(tl.soft_limit_ticks(cfg, a.follower), tl.eeprom_limits(fraw.read_bytes)),
                                  yaml_lim)
        except tl.TeleopRefused as e:
            print(f"REFUSED: {e}"); return 4
        core = tl.TeleopCore(leader_read=lraw.read_bytes_tolerating, follower_bus=fbus, mapping=mapping,
                             limits=limits, speed_ticks_s=a.speed, log=log,
                             smooth_alpha=a.smooth if a.smooth is not None else tl.DEFAULT_SMOOTH_ALPHA)
        probs = core.start_problems()
        log({"event": "start", "mode": "live" if a.live else "check", "follower": a.follower,
             "speed": a.speed, "hz": a.hz, "smooth_alpha": core.smooth_alpha, "leader_ticks": core.leader_ticks(),
             "follower_ticks": core.follower_ticks(), "limits": {j: [round(x) for x in v] for j, v in limits.items()},
             "map_utc": mapping.get("utc"), "start_problems": probs,
             "soft_limits_override": a.allow_unverified_soft_limits, "no_collision_check": a.no_collision_check})
        if probs:
            print("REFUSED (start gate):\n  " + "\n  ".join(probs)); return 6
        if a.check:
            print("CHECK OK: every joint within the start gate. Nothing energised."); return 0
        print("\nCONFIRM before anything moves:\n  - NO tube in the cell; the other arm stays where it is\n"
              f"  - leader held in the {a.follower} follower's pose; hands and cables clear of the follower\n"
              "  - you are at the power switch\n"
              f"The {a.follower} arm will follow the leader at about {a.speed * 360 / 4096:.0f} deg/s. Ctrl-C = hold.")
        if input("Type YES to start: ").strip() != "YES":
            print("not confirmed; nothing moved"); return 0
        STOP_FILE.unlink(missing_ok=True)
        try:
            core.energise()
            log({"event": "energised"})
            res = tl.run_loop(core, hz=a.hz, should_stop=STOP_FILE.exists)
            log({"event": "stopped", "why": "stop file", **res})
        except (KeyboardInterrupt, Exception) as exc:  # noqa: BLE001
            what = "Ctrl-C" if isinstance(exc, KeyboardInterrupt) else f"{type(exc).__name__}: {exc}"
            from inchworm.torque import TorqueManager
            held = TorqueManager({role: fbus}).hold_all()
            log({"event": "hold", "reason": what, "hold": {k: str(v) for k, v in held.items()},
                 "clamps": dict(core.clamps)})
            rc = 0 if isinstance(exc, KeyboardInterrupt) else 7
        print(f"\nHolding the {a.follower} arm in place (torque on).")
        try:
            input("Support the arm, then press Enter to release torque: ")
        except (KeyboardInterrupt, EOFError):
            pass
        return rc
    finally:
        try:   # joints 1-5 only: this script never touches the gripper (AA-17)
            for s in range(1, 6):
                if fraw.read_bytes(s, tl.REG_TORQUE_ENABLE, 1)[0]:
                    fraw.write_bytes(s, tl.REG_TORQUE_ENABLE, b"\x00")
            if a.live:
                log({"event": "released"})
        except Exception as exc:  # noqa: BLE001
            print(f"WARNING: final torque-off sweep failed: {exc}")
        tlead.close(); tfol.close(); fh.close()


if __name__ == "__main__":
    sys.exit(main())
