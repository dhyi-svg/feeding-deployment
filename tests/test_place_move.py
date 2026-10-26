"""Offline checks for microwave/placement/place_move.py: the placement motion is Part 1's cuRobo
HOLD_LEVEL plan, executed in joint impedance. cuRobo is the LinePlanner stand-in from test_xyz_move;
the arm is a fake SheppyArm that records what move_joints was sent.

    PYTHONPATH=src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_place_move.py -q
"""
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "microwave"))
sys.path.insert(0, str(ROOT / "microwave" / "placement"))
sys.path.insert(0, str(ROOT / "tests"))
import place_move as pm  # noqa: E402
import xyz_move as xm  # noqa: E402
from test_xyz_move import PARK, LinePlanner, quiet, sim  # noqa: E402,F401 -- `sim` is the shared fixture

try:
    import sheppy_arm  # noqa: E402
except ImportError:      # no rclpy / rammp_arm_interfaces outside the sheppy env: what execute_legs reads of it
    import types
    sheppy_arm = types.ModuleType("sheppy_arm")
    sheppy_arm.IMPEDANCE = {"stub": "joint impedance gains"}
    sheppy_arm.ROT_EQUIV_M = 0.15
    sys.modules["sheppy_arm"] = sheppy_arm


class FakeArm:
    """Joint commands land instantly at the last knot; records the impedance each move was sent with."""

    def __init__(self, sim, q, gripper=0.9):
        self.sim, self.q, self.gripper, self.sent = sim, np.asarray(q, float), gripper, []

    def raw_joints(self):
        return self.q.copy()

    def get_state(self):
        pos, rot = xm.fk(self.sim, self.q)
        return {"position": self.q.tolist(), "ee_pos": [*pos, *rot.as_quat()], "gripper_pos": self.gripper}

    def move_joints(self, knots, cart_lengths=None, speed_scale=1.0, impedance=None, monitor=None):
        self.sent.append(impedance)
        self.q = np.asarray(knots[-1], float)
        return True


def args_for(*extra):
    return pm.parse_args(["--target", "tool", *extra])


def test_no_point_is_a_refusal_until_part2():
    with pytest.raises(xm.Refused, match="Part 2"):
        pm.placement_point(args_for())
    assert np.allclose(pm.placement_point(args_for("--xyz", "0.5", "0", "0.3")), [0.5, 0, 0.3])


def test_defaults_are_the_task_roll_and_container():
    a = pm.parse_args([])
    assert a.roll_deg == pm.TASK_ROLL_DEG == -90.0 and a.target == "container" and not a.execute


def test_config_file_and_unknown_keys(tmp_path):
    a = pm.parse_args(["--config", str(ROOT / "microwave" / "placement" / "place_move.yaml")])
    assert a.xyz == [0.55, 0.0, 0.30] and a.roll_deg == -90
    bad = tmp_path / "c.yaml"
    bad.write_text("impedance: false\n")                          # impedance is not optional here
    with pytest.raises(SystemExit):
        pm.parse_args(["--config", str(bad)])


def test_plan_uses_hold_level_and_execute_is_impedance(sim):
    lp = LinePlanner(sim)
    a = args_for("--xyz", "0.45", "-0.20", "0.30")
    legs = pm.plan_placement(sim, lp, PARK, pm.placement_point(a), a, xm.Gates(), log=quiet)
    assert legs[-1].name == "move" and legs[-1].hold == xm.HOLD_LEVEL and lp.calls[-1] == xm.HOLD_LEVEL
    arm = FakeArm(sim, PARK)
    pm.execute_placement(arm, sim, lp, legs, xm.Gates(), 0.5, log=quiet)
    assert len(arm.sent) == len(legs) and all(imp is sheppy_arm.IMPEDANCE for imp in arm.sent)
    pos, _ = xm.fk(sim, arm.q)
    assert np.linalg.norm(pos - [0.45, -0.20, 0.30]) < xm.END_POS_TOL_M


def test_container_needs_a_closed_gripper(sim):
    a = pm.parse_args(["--grasp-offset", "0", "0", "0.1"])
    with pytest.raises(xm.Refused, match="gripper is open"):
        pm.check_holding(FakeArm(sim, PARK, gripper=0.01), a)
    assert pm.check_holding(FakeArm(sim, PARK, gripper=0.9), a) == pytest.approx(0.9)
