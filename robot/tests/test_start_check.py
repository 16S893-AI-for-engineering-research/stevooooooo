"""AA-23 start_check.problems: torque, gripper band and the 10 deg start window."""
import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).parents[1]
_s = importlib.util.spec_from_file_location("start_check", ROOT / "scripts" / "start_check.py")
sc = importlib.util.module_from_spec(_s); _s.loader.exec_module(sc)
START = {"left": [1972, 1427, 3081, 1003, 2062], "right": [2063, 1491, 3036, 968, 2055]}


def _st(ticks, grip=1560, band="open", torque=None):
    return {"torque": torque or {s: 0 for s in range(1, 7)}, "ticks": {**{i + 1: t for i, t in enumerate(ticks)}, 6: grip},
            "gripper_band": band, "open_band_ticks": [1506, 1610]}


def test_ready():
    assert sc.problems({a: _st(START[a]) for a in START}, START, 10.0) == []


def test_not_ready_reasons():
    st = {"left": _st([1979, 2429, 1614, 1483, 1989], torque={**{s: 0 for s in range(1, 6)}, 6: 1}),
          "right": _st(START["right"], grip=2036, band="locked", torque={**{s: 0 for s in range(1, 6)}, 6: 1})}
    p = sc.problems(st, START, 10.0)
    assert any("left: torque ON on servos [6]" in x for x in p)
    assert any("right: gripper is locked" in x for x in p)
    assert any("left shoulder_lift" in x for x in p) and any("left elbow_flex" in x for x in p)
    assert not any("right shoulder" in x for x in p)
    near = list(START["left"]); near[0] += 113                          # 9.9 deg: inside
    assert sc.problems({"left": _st(near), "right": _st(START["right"])}, START, 10.0) == []
