"""AA-43 2026-11-12. Run from robot/: uv run python scripts/autoenable_probe.py [--live]
Probe: does a register write energise left servo 5 with torque off? goal == present only. Dry-run by default."""
import sys, time, json
sys.path.insert(0, "src")
LIVE = "--live" in sys.argv
SID = 5
plan = ["check torque off on all 6 left servos",
        "A: write goal_speed=0 (the value it holds already), read Torque_Enable",
        "B: write goal := present (read back), read Torque_Enable at 0/50/200/500 ms, watch present (abort if it moves >10 ticks)",
        "torque off on all 6, read back"]
print("PLAN:", *plan, sep="\n  ")
if not LIVE:
    print("DRY-RUN: nothing opened."); sys.exit(0)
from inchworm.backends import live_backend
from inchworm.config import load_config
from inchworm.servo_bus import _ServoBus
u16 = lambda x: x[0] | (x[1] << 8)
t = live_backend(load_config()).open_serial("arm_left"); b = _ServoBus(t)
log = {"utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "arm": "left", "servo_id": SID}
try:
    te = [b.read_bytes(s, 40, 1)[0] for s in range(1, 7)]
    log["torque_before"] = te
    if any(te): print("REFUSED: torque on", te); sys.exit(4)
    p0 = b.read_bytes(SID, 0x38, 2); log["present_before"] = u16(p0); log["goal_before"] = u16(b.read_bytes(SID, 0x2A, 2))
    b.write_bytes(SID, 0x2E, bytes([0, 0]))
    log["A_te_after_speed_write"] = b.read_bytes(SID, 40, 1)[0]
    p = b.read_bytes(SID, 0x38, 2); b.write_bytes(SID, 0x2A, p)
    t0 = time.monotonic(); samples = []
    for dt in (0, 0.05, 0.2, 0.5):
        while time.monotonic() - t0 < dt: time.sleep(0.005)
        s = {"t_ms": round((time.monotonic() - t0) * 1000), "te": b.read_bytes(SID, 40, 1)[0],
             "present": u16(b.read_bytes(SID, 0x38, 2)), "goal": u16(b.read_bytes(SID, 0x2A, 2))}
        samples.append(s)
        if abs(s["present"] - u16(p)) > 10: log["abort"] = "moved"; break
    log["B_goal_written"] = u16(p); log["B_samples"] = samples
finally:
    for s in range(1, 7):
        try: b.write_bytes(s, 40, b"\x00")
        except Exception as e: print("WARNING torque-off failed", s, e, "-- SWITCH POWER OFF")
    log["torque_after"] = [b.read_bytes(s, 40, 1)[0] for s in range(1, 7)]
    log["present_after"] = u16(b.read_bytes(SID, 0x38, 2))
    t.close()
    print(json.dumps(log))
    with open("calibration_data/autoenable_probe_2026-11-12.jsonl", "a") as fh: fh.write(json.dumps(log) + "\n")
