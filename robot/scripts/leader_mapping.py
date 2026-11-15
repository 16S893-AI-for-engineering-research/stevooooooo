#!/usr/bin/env python3
"""AA-44: leader-to-follower joint mapping by matched hand poses. READ-ONLY, nothing moves.

Owner decision 2026-11-13: the mapping lives in software. NO EEPROM writes on either arm.
You put the leader and the follower (torque off) in the same pose by eye, at 2-3 different poses.
For each joint 1-5 the script fits the mapping, in servo ticks, mod 4096:

    follower_ticks = sign * leader_ticks + offset

Joint 6 (gripper) is never mapped: only the AA-17 state machine commands the gripper.

Subcommands (run from robot/; without --live nothing opens a port)::

    uv run python scripts/leader_mapping.py capture --follower left --label P1 --live
    uv run python scripts/leader_mapping.py capture --follower left --label P2 --live
    uv run python scripts/leader_mapping.py fit --follower left
    uv run python scripts/leader_mapping.py watch --follower left --live   # read-only live check

* ``capture`` reads the leader joints 1-6 (tolerating ONLY the firmware-3.10 error bit 0x08) and the
  follower joints 1-6. It refuses if any follower servo has torque ON, and appends one line to
  ``calibration_data/leader_map_<follower>.jsonl``.
* ``fit`` fits sign and offset per joint from every capture in that log, or from the labels given
  with ``--labels``. It needs at least 2 poses whose leader ticks differ by
  ``--min-span-deg`` (default 15) on a joint; otherwise that joint's sign is NOT DETERMINED.
  It reports each pose's residual and refuses to save if any residual is above ``--tol-deg``
  (default 5). The fit is saved to ``calibration_data/leader_map_<follower>.json`` with
  ``unverified: true``. It is cleared only by the supervised low-speed teleop check (AA-44 AC #4).
* ``watch`` streams, read-only, the follower ticks predicted from the leader next to the measured
  follower ticks. Move both arms together by hand and check that the error stays small.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence

_ROBOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_ROBOT / "src"))

J5 = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
TICKS = 4096
DEG_PER_TICK = 360.0 / TICKS
REG_TORQUE_ENABLE = 0x28
REG_PRESENT_POSITION = 56
DEFAULT_DIR = _ROBOT / "calibration_data"


class MappingError(RuntimeError):
    pass


def wrap(d: float) -> float:
    """Wrap a tick difference into [-2048, 2048)."""
    return ((d + TICKS / 2) % TICKS) - TICKS / 2


def fit_joint(leader: Sequence[int], follower: Sequence[int], min_span_ticks: float) -> dict:
    """Fit follower = sign*leader + offset (mod 4096) for one joint. Pure function."""
    if len(leader) != len(follower) or len(leader) < 2:
        raise MappingError("need at least 2 paired poses")
    span = max(abs(wrap(a - leader[0])) for a in leader)
    best, worsts = None, {}
    for sign in (1, -1):
        offs = [wrap(f - sign * l) for l, f in zip(leader, follower)]
        ref = offs[0]
        mean = (ref + sum(wrap(o - ref) for o in offs) / len(offs)) % TICKS
        res = [wrap(o - mean) for o in offs]
        worst = max(abs(r) for r in res)
        worsts[sign] = worst
        if best is None or worst < best["worst_ticks"]:
            best = {"sign": sign, "offset_ticks": round(mean, 1), "residual_ticks": [round(r, 1) for r in res],
                    "worst_ticks": round(worst, 1)}
    best["leader_span_ticks"] = round(span, 1)
    other = worsts[-best["sign"]]
    #: mod-4096 cannot tell +1 from -1 when the joint moved about half a turn between poses
    best["sign_ambiguous"] = other - best["worst_ticks"] < min_span_ticks / 2
    best["sign_determined"] = span >= min_span_ticks and not best["sign_ambiguous"]
    return best


def fit_mapping(captures: List[dict], min_span_deg: float = 15.0, tol_deg: float = 5.0) -> dict:
    """Fit all five joints from captures (each has leader_ticks[0:5], follower_ticks[0:5])."""
    if len(captures) < 2:
        raise MappingError(f"need at least 2 captured poses, have {len(captures)}")
    joints, problems = {}, []
    for i, j in enumerate(J5):
        r = fit_joint([c["leader_ticks"][i] for c in captures], [c["follower_ticks"][i] for c in captures],
                      min_span_deg / DEG_PER_TICK)
        r["worst_deg"] = round(r["worst_ticks"] * DEG_PER_TICK, 2)
        joints[j] = r
        if r["sign_ambiguous"] and r["leader_span_ticks"] >= min_span_deg / DEG_PER_TICK:
            problems.append(f"{j}: both directions fit about equally (the joint moved about half a turn between "
                            "poses); add a pose with this joint about a quarter turn from the others")
        elif not r["sign_determined"]:
            problems.append(f"{j}: leader moved only {r['leader_span_ticks'] * DEG_PER_TICK:.1f} deg across the poses "
                            f"(need {min_span_deg}); sign NOT DETERMINED, move this joint more between poses")
        if r["worst_deg"] > tol_deg:
            problems.append(f"{j}: a pose is {r['worst_deg']} deg off the fit (tolerance {tol_deg}); "
                            "re-match that pose or recapture")
    return {"joints": joints, "problems": problems, "labels": [c.get("label") for c in captures],
            "min_span_deg": min_span_deg, "tol_deg": tol_deg}


def predict_follower(mapping: dict, leader_ticks: Sequence[int]) -> List[int]:
    """Follower ticks for joints 1-5 from leader ticks. Gripper is never mapped."""
    out = []
    for i, j in enumerate(J5):
        m = mapping["joints"][j]
        out.append(int(round(m["sign"] * leader_ticks[i] + m["offset_ticks"])) % TICKS)
    return out


# ---------------------------------------------------------------- hardware (read-only)

def read_leader(raw) -> List[int]:
    """Leader joints 1-6 present ticks; tolerates ONLY error bit 0x08 (fw 3.10 encoder-only)."""
    return [int.from_bytes(raw.read_bytes_tolerating(s, REG_PRESENT_POSITION, 2), "little") for s in range(1, 7)]


def read_follower(raw) -> List[int]:
    """Follower joints 1-6 present ticks. Refuses if any servo has torque ON (read-only snapshot)."""
    on = [s for s in range(1, 7) if raw.read_bytes(s, REG_TORQUE_ENABLE, 1)[0]]
    if on:
        raise MappingError(f"follower torque is ON on servos {on}; switch it off first (this tool is read-only)")
    return [int.from_bytes(raw.read_bytes(s, REG_PRESENT_POSITION, 2), "little") for s in range(1, 7)]


def _open(cfg, backend, role: str):
    from inchworm.servo_bus import _ServoBus
    t = backend.open_serial(role)
    return t, _ServoBus(t)   # raw framing only; this tool only issues READ packets


def load_captures(path: Path, labels: Optional[Sequence[str]] = None) -> List[dict]:
    if not path.exists():
        raise MappingError(f"no captures yet: {path}")
    caps = [json.loads(l) for l in path.read_text().splitlines() if l.strip()]
    caps = [c for c in caps if c.get("event") == "capture"]
    last = {c["label"]: c for c in caps}          # a re-captured label replaces the earlier one
    if not labels:
        return list(last.values())
    if labels:
        missing = [l for l in labels if l not in last]
        if missing:
            raise MappingError(f"labels not in {path.name}: {missing}")
        caps = [last[l] for l in labels]
    return caps


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", nargs="?", choices=("capture", "fit", "watch"))
    ap.add_argument("--follower", choices=("left", "right"), default="left")
    ap.add_argument("--label", default=None, help="capture: pose label, e.g. P1")
    ap.add_argument("--labels", default=None, help="fit: comma-separated labels to use (default: all)")
    ap.add_argument("--min-span-deg", type=float, default=15.0)
    ap.add_argument("--tol-deg", type=float, default=5.0)
    ap.add_argument("--dir", type=Path, default=DEFAULT_DIR)
    ap.add_argument("--seconds", type=float, default=60.0, help="watch: duration")
    ap.add_argument("--live", action="store_true")
    a = ap.parse_args(argv)
    if a.action is None:
        print("DRY-RUN: no action given. Steps: capture --label P1 --live, capture --label P2 --live "
              "(and P3), fit, watch --live. Read-only; nothing moves; no writes.")
        return 0
    log = a.dir / f"leader_map_{a.follower}.jsonl"
    out = a.dir / f"leader_map_{a.follower}.json"

    if a.action == "fit":
        try:
            caps = load_captures(log, a.labels.split(",") if a.labels else None)
            fit = fit_mapping(caps, a.min_span_deg, a.tol_deg)
        except MappingError as e:
            print(f"REFUSED: {e}"); return 4
        for j, r in fit["joints"].items():
            print(f"{j:13s} sign {r['sign']:+d}  offset {r['offset_ticks']:7.1f} ticks  "
                  f"worst {r['worst_deg']:5.2f} deg  span {r['leader_span_ticks'] * DEG_PER_TICK:5.1f} deg"
                  + ("" if r["sign_determined"] else "  SIGN NOT DETERMINED"))
        if fit["problems"]:
            print("NOT SAVED:\n  " + "\n  ".join(fit["problems"])); return 3
        fit.update({"follower": a.follower, "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                    "unverified": True, "note": "cleared only by the supervised low-speed teleop check (AA-44 AC #4)"})
        a.dir.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(fit, indent=1))
        print(f"saved {out} (unverified: true)")
        return 0

    if a.action == "capture" and not a.label:
        print("REFUSED: capture needs --label"); return 4
    if not a.live:
        print(f"DRY-RUN: would {a.action} (read-only) the leader and the {a.follower} follower "
              f"(torque must be off; nothing moves; no writes). Add --live.")
        return 0

    from inchworm.backends import live_backend
    from inchworm.config import load_config
    cfg = load_config()
    if cfg.arms.leader is None:
        print("REFUSED: arms.leader is not configured in hardware.yaml"); return 4
    be = live_backend(cfg)
    tl, lraw = _open(cfg, be, "arm_leader")
    tf, fraw = _open(cfg, be, f"arm_{a.follower}")
    worst: List[float] = []
    try:
        if a.action == "capture":
            lt, ft = read_leader(lraw), read_follower(fraw)
            rec = {"event": "capture", "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "label": a.label,
                   "follower": a.follower, "leader_ticks": lt, "follower_ticks": ft}
            a.dir.mkdir(parents=True, exist_ok=True)
            with open(log, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
            print(json.dumps(rec))
            print(f"appended to {log}")
            return 0
        try:
            mapping = json.loads(out.read_text())
        except FileNotFoundError:
            print(f"REFUSED: no fit yet ({out}); run capture x2-3 and fit first"); return 4
        print("watch: move both arms together by hand; Ctrl-C to stop. err = measured - predicted (deg)")
        t0, worst = time.monotonic(), [0.0] * 5
        while time.monotonic() - t0 < a.seconds:
            lt, ft = read_leader(lraw), read_follower(fraw)
            pred = predict_follower(mapping, lt)
            err = [wrap(f - p) * DEG_PER_TICK for f, p in zip(ft[:5], pred)]
            worst = [max(w, abs(e)) for w, e in zip(worst, err)]
            print("  ".join(f"{j[:8]} {e:+6.1f}" for j, e in zip(J5, err)), end="\r", flush=True)
            time.sleep(0.1)
        return 0
    except MappingError as e:
        print(f"REFUSED: {e}"); return 4
    except KeyboardInterrupt:
        print("\nstopped")
        return 0
    finally:
        if worst:
            print("\nworst |err| deg: " + ", ".join(f"{j} {w:.1f}" for j, w in zip(J5, worst)))
        tl.close(); tf.close()


if __name__ == "__main__":
    sys.exit(main())
