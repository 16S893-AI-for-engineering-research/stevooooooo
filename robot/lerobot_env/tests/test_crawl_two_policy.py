"""AA-55 two-policy crawl (crawl_two_policy.py) end to end on simulated servos: real cell mesh,
real timeline, stand-in policies that drive to each arm's finish reference (no checkpoint, no GPU)."""
from __future__ import annotations

import importlib.util
import json

import numpy as np
import pytest

from inchworm import training_poses as tp

from tests.test_inchworm_plugin import ROBOT

_s = importlib.util.spec_from_file_location("crawl_two_policy", ROBOT / "lerobot_env" / "crawl_two_policy.py")
ctp = importlib.util.module_from_spec(_s); _s.loader.exec_module(ctp)
fpt = ctp.fpt
pytestmark = pytest.mark.skipif(not ctp.TIMELINE.exists() or not ctp.LEFT_DATASET.exists()
                                or not ctp.RIGHT_DATASET.exists(), reason="needs the timeline and both training datasets")
ARMS = ("left", "right")
LOCK_BAND = (2013, 2066)                     # ticks: both grippers' LOCKED bands lie inside this


class ToRef:
    """Stand-in policy: always the arm's finish reference, at most 5 deg ahead (inside the jump guard)."""

    def __init__(self, ref_ticks):
        self.ref = np.array(ref_ticks, float) * ctp.DEG_PER_TICK

    def reset(self):
        pass

    def step(self, obs):
        p = np.asarray(obs.joint_positions_deg, float)
        return {j: float(v) for j, v in zip(ctp.J5, p + np.clip(self.ref - p, -5.0, 5.0))}


def _state42(tmp_path, nudge=None):
    from inchworm import cycle_replay as cr
    from inchworm import handoff as ho
    from inchworm.config import load_config
    cfg = load_config()
    fr, _ = cr.clamp_to_eeprom(cfg, cr.load_timeline(ctp.TIMELINE), elbow_cap=cr.FOLD_ELBOW_CAP_TICKS)
    f = fr[42]
    t = {a: cr.q_to_ticks(cfg, a, f.q[a]) for a in ARMS}
    present = {a: list(t[a]) for a in ARMS}
    for (arm, k), d in (nudge or {}).items():
        present[arm][k] += d
    p = tmp_path / "h42.json"
    ho.write_state(p, stop_frame=42, step=f.step, timeline_digest=fpt._digest(ctp.TIMELINE), tube_in_grippers=True,
                   goal_ticks=t, present_ticks=present, gripper_ticks={"left": 1547, "right": 2038},
                   locked=f.locked, q_rad=f.q, tube_pose=f.tube)
    return p


def _run(tmp_path, monkeypatch, answers=None, max_s="30", nudge=None, left_aim=None):  # right elbow ~43 deg at 10 deg/s
    import lerobot_robot_inchworm.inchworm_follower as fmod
    monkeypatch.setattr(fmod, "STOP_FILE", tmp_path / "STOP_TELEOP")
    monkeypatch.setattr(ctp, "RUNNER_STOP", tmp_path / "stop_move")
    monkeypatch.setattr(ctp, "OUT_DIR", tmp_path / "out")
    monkeypatch.setattr(ctp, "STATE_TRAIN_RIGHT", tmp_path / "state_tr.json")
    refs = {"left": left_aim or ctp.LEFT_FINISH_REF_TICKS, "right": ctp.RIGHT_FINISH_REF_TICKS}
    monkeypatch.setattr(ctp, "load_policy", lambda ckpt, dev, arm: (ToRef(refs[arm]), f"stub-{arm}"))
    ctp.SIM.clear()
    ui = fpt.UI(True, auto=answers or {})
    rc = ctp.main(["--handoff-state", str(_state42(tmp_path, nudge)), "--dry-run", "--max-deg-s", "10",
                   "--max-s", max_s, "--left-checkpoint", str(tmp_path), "--right-checkpoint", str(tmp_path)], ui=ui)
    log = [json.loads(l) for p in (tmp_path / "out").glob("crawl_two_policy_dry_*.jsonl")
           for l in p.read_text().splitlines()]
    return rc, log, ctp.SIM.get("buses")


def _locked(buses, arm):
    return LOCK_BAND[0] <= buses[arm].pos[6] <= LOCK_BAND[1] and buses[arm].torque[6]


def _joints_off(buses):
    return not any(buses[a].torque[s] for a in ARMS for s in range(1, 6))


def test_finish_refs_are_the_dataset_demo_end_medians():
    assert tuple(ctp.demo_end_median_ticks(ctp.LEFT_DATASET)) == ctp.LEFT_FINISH_REF_TICKS
    assert tuple(ctp.demo_end_median_ticks(ctp.RIGHT_DATASET)) == ctp.RIGHT_FINISH_REF_TICKS


def test_full_crawl_phase_order_and_gripper_sequence(tmp_path, monkeypatch):
    rc, log, buses = _run(tmp_path, monkeypatch)
    ev = [e["event"] for e in log]
    assert rc == 0 and "crawl_done" in ev, [e for e in log if e["event"] == "stopped"]
    # AC #3: never both unlocked: left locks before the right unlocks, the right relocks at the end
    g = [(e["event"], e["arm"]) for e in log if e["event"] in ("gripper_lock", "gripper_unlock")]
    assert g == [("gripper_lock", "left"), ("gripper_unlock", "right"), ("gripper_lock", "right")]
    i_unlock = next(i for i, e in enumerate(log) if e["event"] == "gripper_unlock")
    assert any(e["event"] == "left_lock_readback" for e in log[:i_unlock])
    # AC #4: phase order; the left moves to train_right only after the right unlocked and is out
    started = [e["name"] for e in log if e["event"] == "phase" and e["status"] == "start"]
    assert started == ["right_out", "left_to_train_right", "right_to_train_right", "home"]
    pos = {n: next(i for i, e in enumerate(log) if e.get("name") == n and e.get("status") == "start") for n in started}
    done_out = next(i for i, e in enumerate(log) if e.get("name") == "right_out" and e.get("status") == "done")
    assert i_unlock < pos["right_out"] < done_out < pos["left_to_train_right"]
    # AC #2: both policy phases passed the frozen-pose gate
    gates = [(e["pose"], e["problems"]) for e in log if e["event"] == "training_pose_gate"]
    assert gates == [("train_left", []), ("train_right", [])]
    # AC #5: end = both LOCKED, joints 1-5 of both arms off after the Enter, grippers untouched
    assert _locked(buses, "left") and _locked(buses, "right") and _joints_off(buses)
    st = json.loads((tmp_path / "sim_state_tr.json").read_text())
    assert st["locked"] == {"left": True, "right": False} and st["stop_frame"] == tp.position("train_right")["timeline_frame"]


def test_left_policy_refused_off_the_frozen_train_left_pose(tmp_path, monkeypatch):
    # AC #2: right shoulder_lift 35 ticks off the frozen pose -> refused before anything moves
    rc, log, buses = _run(tmp_path, monkeypatch, nudge={("right", 1): 35})
    gate = next(e for e in log if e["event"] == "training_pose_gate")
    assert rc == 4 and gate["problems"] and "right arm off the frozen train_left pose" in gate["problems"][0]
    assert not any(e["event"].startswith(("gripper", "policy_attempt")) for e in log)
    assert buses["left"].pos == {i + 1: float(t) for i, t in enumerate(json.loads(
        (tmp_path / "h42.json").read_text())["present_ticks"]["left"] + [1547])}


def test_owner_says_the_left_key_is_not_in_the_right_never_unlocks(tmp_path, monkeypatch):
    rc, log, buses = _run(tmp_path, monkeypatch, answers={"left_look": "n"})
    ev = [e["event"] for e in log]
    assert rc == 0 and "gripper_unlock" not in ev and not any(e["event"] == "phase" for e in log)
    assert _locked(buses, "right") and _locked(buses, "left") and _joints_off(buses)


def test_unlock_right_refused_unless_the_left_reads_locked(tmp_path):
    """AC #3 unit: the owner's y is not enough; the left must read inside its LOCKED band."""
    from types import SimpleNamespace
    from inchworm.config import load_config
    from inchworm.gripper_lock import GripperState
    cfg = load_config()
    calls = []
    left = SimpleNamespace(bus=SimpleNamespace(read_position_deg=lambda sid: 1547 * ctp.DEG_PER_TICK),
                           sm=SimpleNamespace(state=GripperState.UNLOCKED))
    r = SimpleNamespace(cfg=cfg, arms={"left": left}, _unlock=lambda arm: calls.append(arm))
    with pytest.raises(ctp.Stop):
        ctp.unlock_right(r, "y", lambda rec: None)
    band = cfg.gripper_lock.arm_left
    left.bus.read_position_deg = lambda sid: (band.locked_low_deg + band.locked_high_deg) / 2
    with pytest.raises(ctp.Stop):                       # reads locked but the SM is not LOCKED
        ctp.unlock_right(r, "y", lambda rec: None)
    left.sm.state = GripperState.LOCKED
    with pytest.raises(ctp.Stop):                       # owner did not confirm
        ctp.unlock_right(r, "n", lambda rec: None)
    assert calls == []
    ctp.unlock_right(r, "y", lambda rec: None)
    assert calls == ["right"]


def test_quit_in_the_right_policy_keeps_both_locked_left(tmp_path, monkeypatch):
    # AC #5: a stop holds and gives the release prompt; the left stays LOCKED on the tube
    rc, log, buses = _run(tmp_path, monkeypatch, answers={"right_choice": "q"})
    ev = [e["event"] for e in log]
    assert rc == 0 and "crawl_done" not in ev and ("quit" in ev)
    assert _locked(buses, "left") and not _locked(buses, "right") and _joints_off(buses)


def test_lock_far_from_the_finish_ref_is_logged_and_the_crawl_continues(tmp_path, monkeypatch):
    # OWNER 2026-11-21: no 8 deg lock stop. The left stand-in ends 12 deg (elbow) off the finish ref
    # (like a perturbed demo end): the offset is logged and the right still unlocks and comes out.
    aim = list(ctp.LEFT_FINISH_REF_TICKS); aim[2] -= round(12.0 / ctp.DEG_PER_TICK)
    rc, log, buses = _run(tmp_path, monkeypatch, answers={"right_choice": "q"}, left_aim=tuple(aim))
    off = next(e for e in log if e["event"] == "lock_offset_from_finish_ref")
    assert off["arm"] == "left" and off["max_deg"] > 8.0
    ev = [e["event"] for e in log]
    assert "gripper_unlock" in ev and not any("finish reference" in str(e.get("reason", "")) for e in log)
    assert [e["name"] for e in log if e["event"] == "phase" and e["status"] == "done"] == [
        "right_out", "left_to_train_right", "right_to_train_right"]
    assert _locked(buses, "left") and _joints_off(buses)
