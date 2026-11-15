"""AA-44: teleop_live core against simulated servos (no hardware)."""
import importlib.util
import pathlib

import pytest

from inchworm import teleop_live as tl

_s = importlib.util.spec_from_file_location("replay_sim", pathlib.Path(__file__).parents[1] / "scripts" / "replay_sim.py")
sim = importlib.util.module_from_spec(_s); _s.loader.exec_module(sim)

MAP = {"follower": "left", "problems": [],
       "joints": {j: {"sign": 1, "offset_ticks": o, "sign_determined": True}
                  for j, o in zip(tl.J5, (22.3, 2177.0, 3083.7, 3741.7, 1443.0))}}
LIMITS = {j: (700.0, 3400.0) for j in tl.J5}


class Leader:
    def __init__(self, ticks):
        self.ticks = list(ticks)
        self.reads = []

    def read(self, sid, reg, n):
        assert reg == tl.REG_PRESENT_POSITION and 1 <= sid <= 5   # never reads/writes anything else
        self.reads.append(sid)
        return int(self.ticks[sid - 1]).to_bytes(2, "little")


def _leader_for(follower_ticks):
    return [(f - MAP["joints"][j]["offset_ticks"]) % 4096 for f, j in zip(follower_ticks, tl.J5)]


def _setup(f_ticks=(2000, 2000, 2000, 2000, 2000), **kw):
    clock = sim.SimClock()
    bus = sim.SimArmBus("arm_left", list(f_ticks) + [2034], clock)
    lead = Leader(_leader_for(f_ticks))
    core = tl.TeleopCore(leader_read=lead.read, follower_bus=bus, mapping=MAP, limits=dict(LIMITS), **kw)
    return clock, bus, lead, core


def test_follows_leader_and_never_writes_gripper():
    clock, bus, lead, core = _setup()
    assert core.start_problems() == []
    core.energise()
    lead.ticks[0] = (lead.ticks[0] + 200) % 4096
    for _ in range(200):
        core.step(); clock.sleep(1 / 30)
    assert abs(bus.pos[1] - 2200) <= 3
    assert bus.goal[6] == 2034 and not bus.torque[6]          # gripper untouched


def test_limit_is_clamped_not_exceeded():
    clock, bus, lead, core = _setup()
    core.energise()
    lead.ticks[2] = (lead.ticks[2] + 1800) % 4096   # elbow target 3800 > limit 3400
    for _ in range(400):
        out = core.step(); clock.sleep(1 / 30)
    assert 3395 <= out["goal"][2] <= 3400 and "elbow_flex" in out["clamped"]
    assert bus.pos[3] <= 3400 and core.clamps["elbow_flex"] > 0


def test_no_seam_jump():
    # follower near 0/4096 seam is excluded by EEPROM limits; check the representative logic
    assert tl.nearest_rep(4090.0, 10.0) == -6.0
    assert tl.nearest_rep(5.0, 4093.0) == 4101.0


def test_start_gate_refuses_far_leader():
    clock, bus, lead, core = _setup()
    lead.ticks[1] = (lead.ticks[1] + 200) % 4096     # ~17.6 deg
    p = core.start_problems()
    assert len(p) == 1 and p[0].startswith("shoulder_lift")


def test_speed_bounds_and_not_energised():
    with pytest.raises(tl.TeleopRefused):
        _setup(speed_ticks_s=0)
    with pytest.raises(tl.TeleopRefused):
        _setup(speed_ticks_s=601)
    _, _, _, core = _setup()
    with pytest.raises(tl.TeleopRefused):
        core.step()


def test_goal_speed_is_capped_to_setting():
    clock, bus, lead, core = _setup(speed_ticks_s=150)
    core.energise()
    lead.ticks[0] = (lead.ticks[0] + 400) % 4096
    core.step()
    assert bus.speed[1] == 150


def test_load_leader_map_refusals(tmp_path):
    import json
    p = tmp_path / "m.json"
    with pytest.raises(tl.TeleopRefused):
        tl.load_leader_map(p, "left")
    p.write_text(json.dumps(MAP))
    with pytest.raises(tl.TeleopRefused):
        tl.load_leader_map(p, "right")
    bad = json.loads(json.dumps(MAP)); bad["joints"]["elbow_flex"]["sign_determined"] = False
    p.write_text(json.dumps(bad))
    with pytest.raises(tl.TeleopRefused):
        tl.load_leader_map(p, "left")
    p.write_text(json.dumps(MAP))
    assert tl.load_leader_map(p, "left")["follower"] == "left"


def test_soft_limits_intersect_eeprom():
    from inchworm.config import load_config
    cfg = load_config()
    soft = tl.soft_limit_ticks(cfg, "left")
    ee = {j: tuple(v) for j, v in cfg.motion_guard.left_arm_limits.servo_tick_limits.items() if j in tl.J5}
    both = tl.intersect(soft, ee)
    for j in tl.J5:
        assert both[j][0] >= max(soft[j][0], ee[j][0]) and both[j][1] <= min(soft[j][1], ee[j][1])


def test_run_loop_logs_stats():
    clock, bus, lead, core = _setup()
    core.energise()
    recs = []
    core.log = recs.append
    res = tl.run_loop(core, hz=30, should_stop=lambda: False, clock=clock, sleep=clock.sleep, max_s=3.0)
    assert res["steps"] >= 80
    assert any(r["event"] == "teleop_stats" for r in recs)


def test_live_script_refuses_without_collision_reason(capsys):
    sp = importlib.util.spec_from_file_location("tlive", pathlib.Path(__file__).parents[1] / "scripts" / "teleop_live.py")
    m = importlib.util.module_from_spec(sp); sp.loader.exec_module(m)
    assert m.main([]) == 0
    assert m.main(["--live"]) == 4
    assert "no-collision-check" in capsys.readouterr().out


def test_live_script_end_to_end_with_simulated_servos(monkeypatch, tmp_path):
    """Runs the real --live path of scripts/teleop_live.py against simulated servos.
    Found 2026-11-15 on hardware: a wrong run_loop() keyword crashed the live path right after
    energising (it held, nothing moved). This test drives that exact path."""
    import threading, time as _t
    import inchworm.backends as be
    import inchworm.servo_bus as sb
    from inchworm.config import load_config

    cfg = load_config()
    lim = cfg.motion_guard.left_arm_limits.servo_tick_limits
    f_ticks = [1948, 1390, 3122, 1032, 990]

    class Bus(sim.SimArmBus):
        def _read_bytes(self, sid, reg, n):
            if reg in (16, 21, 28, 48):          # drive registers: sane, non-zero
                return bytes([100, 0])[:n]
            return super()._read_bytes(sid, reg, n)

    bus = Bus("arm_left", f_ticks + [2034], _t.monotonic,
              limits={i + 1: tuple(lim[j]) for i, j in enumerate((*tl.J5, "gripper"))})
    bus._bus.read_bytes = bus._read_bytes
    m = tl.load_leader_map(pathlib.Path(__file__).parents[1] / "calibration_data" / "leader_map_left.json", "left")
    lead_ticks = [round((f - m["joints"][j]["offset_ticks"]) % 4096) for f, j in zip(f_ticks, tl.J5)]
    lead = Leader(lead_ticks)

    class T:
        def close(self): pass

    monkeypatch.setattr(be, "live_backend", lambda cfg: type("B", (), {"open_serial": lambda self, r: T()})())
    monkeypatch.setattr(sb, "ArmServoBus", lambda t, role, joint_id_map: bus)
    monkeypatch.setattr(sb, "_ServoBus", lambda t: type("L", (), {"read_bytes_tolerating": staticmethod(lead.read)})())
    sp = importlib.util.spec_from_file_location("tlive2", pathlib.Path(__file__).parents[1] / "scripts" / "teleop_live.py")
    mod = importlib.util.module_from_spec(sp); sp.loader.exec_module(mod)
    stop = tmp_path / "STOP"
    monkeypatch.setattr(mod, "STOP_FILE", stop)
    monkeypatch.setattr(mod, "ROOT", tmp_path)
    (tmp_path / "calibration_data").mkdir()
    import shutil
    shutil.copy(pathlib.Path(__file__).parents[1] / "calibration_data" / "leader_map_left.json",
                tmp_path / "calibration_data" / "leader_map_left.json")
    answers = iter(["YES", ""])
    monkeypatch.setattr("builtins.input", lambda *a: next(answers))

    def later():
        _t.sleep(0.5)
        lead.ticks[0] = (lead.ticks[0] + 50) % 4096
        _t.sleep(1.0)
        stop.touch()
    threading.Thread(target=later, daemon=True).start()
    rc = mod.main(["--live", "--allow-unverified-soft-limits", "test", "--no-collision-check", "test"])
    assert rc == 0
    log = (next((tmp_path / "calibration_data").glob("teleop_live_*.jsonl"))).read_text()
    assert '"event": "stopped"' in log and '"event": "released"' in log and '"hold"' not in log
    assert abs(bus.pos[1] - (f_ticks[0] + 50)) <= 15        # pan followed the leader
    assert not any(bus.torque[s] for s in range(1, 6))       # torque off on exit
    assert bus.goal[6] == 2034 and not bus.torque[6]          # gripper never touched


def test_smoothing_filters_noise_and_speed_scales_with_step():
    clock, bus, lead, core = _setup(speed_ticks_s=150, smooth_alpha=0.35)
    core.energise()
    base = lead.ticks[0]
    goals = []
    for i in range(60):                       # +/-1 tick leader noise: deadband holds the goal
        lead.ticks[0] = base + (1 if i % 2 else 0)
        goals.append(core.step()["goal"][0]); clock.sleep(1 / 30)
    assert max(goals) - min(goals) <= 1
    lead.ticks[0] = base + 4                  # small correction: Goal_Speed well below the cap
    core.step()
    assert 5 <= bus.speed[1] < 150
    lead.ticks[0] = base + 30
    for _ in range(60):
        core.step(); clock.sleep(1 / 30)
    assert abs(core.last_goal[0] - (2000 + 30)) <= 2   # converges to the leader


def test_smooth_alpha_bounds():
    with pytest.raises(tl.TeleopRefused):
        _setup(smooth_alpha=0)
    with pytest.raises(tl.TeleopRefused):
        _setup(smooth_alpha=1.5)


def test_owner_defaults_2026_11_15():
    assert tl.DEFAULT_SPEED_TICKS_S == 250 and tl.DEFAULT_SMOOTH_ALPHA == 1.0
    clock, bus, lead, core = _setup()
    core.energise()
    lead.ticks[0] = (lead.ticks[0] + 400) % 4096
    core.step()
    assert abs(core.last_goal[0] - 2400) < 1 and bus.speed[1] == 250   # unfiltered goal, cap 250
