"""Offline checks for microwave/xyz_move.py: frames, offsets, orientation, the HOLD_LEVEL/roll split
and the gates, on the repo's PyBullet Gen3. No ROS, no arm, no planner: cuRobo is stood in for by a
straight line of the repo's own solve_ik that refuses a HOLD_LEVEL goal exactly as PlanToPose does.

    PYTHONPATH=src PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_xyz_move.py -q
"""
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation as R, Slerp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "microwave"))
import microwave_common as mc  # noqa: E402
import xyz_move as xm  # noqa: E402

PARK = np.asarray(json.loads((ROOT / "microwave" / "park_pose.json").read_text())["joints"], float)


@pytest.fixture(scope="module")
def sim(request):
    import os
    old = os.getcwd()
    os.chdir(ROOT)                      # make_sim's config path is relative to the repo root
    request.addfinalizer(lambda: os.chdir(old))
    return mc.make_sim()


class LinePlanner:
    """cuRobo stand-in: straight line, slerped orientation, 1 cm steps, each IK'd from the last. Like
    PlanToPose it refuses a HOLD_LEVEL goal whose tilt disagrees with the start's."""

    def __init__(self, sim):
        self.sim, self.calls = sim, []

    def plan(self, pos, rot, start_q, hold):
        self.calls.append(hold)
        p0, r0 = xm.fk(self.sim, start_q)
        if hold == xm.HOLD_LEVEL and xm.tilt_deg(r0, rot) > xm.HOLD_TOL_DEG:
            raise xm.Refused(f"CONSTRAINT_UNSATISFIABLE: tilt {xm.tilt_deg(r0, rot):.1f} deg")
        n = max(2, int(np.ceil(max(np.linalg.norm(np.asarray(pos) - p0) / 0.01,
                                   np.degrees((rot * r0.inv()).magnitude()) / 2.0))) + 1)
        sl = Slerp([0, 1], R.concatenate([r0, rot]))
        q, out = np.asarray(start_q, float), [np.asarray(start_q, float)]
        for f in np.linspace(0, 1, n)[1:]:
            for _ in range(4):
                q, err = mc.solve_ik(*self.sim, p0 + (np.asarray(pos) - p0) * f, sl([f]).as_quat()[0], q)
            out.append(q.copy())
        if err > xm.END_POS_TOL_M:
            raise xm.Refused(f"no IK ({err * 100:.1f} cm)")
        return np.array(out)


def quiet(*_):
    pass


# ---- orientation -------------------------------------------------------------------------------
def test_horizontal_reference_is_the_handle_grasp():
    # handle_detect.HANDLE_QUAT, turned pi about y in grasp_pose(): what closed on the vertical bar
    handle = R.from_quat([-0.5, 0.5, 0.5, -0.5]) * R.from_euler("y", np.pi)
    assert (handle * xm.R_HORIZONTAL.inv()).magnitude() < 1e-9


def test_park_pose_is_horizontal(sim):
    _, rot = xm.fk(sim, PARK)
    m = rot.as_matrix()
    assert m[0, 2] > 0.9                                  # approach ~ +x
    assert abs(m[2, 0]) < 0.15                            # fingers close ~horizontally
    assert abs(xm.roll_of(rot)) < 10.0


@pytest.mark.parametrize("roll", [90.0, -90.0])
def test_roll_makes_fingers_vertical_and_keeps_the_approach(roll):
    r0, r1 = xm.target_orientation(30.0, 0.0), xm.target_orientation(30.0, roll)
    assert np.allclose(r0.as_matrix()[:, 2], r1.as_matrix()[:, 2])          # same approach
    assert abs(abs(r1.as_matrix()[2, 0]) - 1.0) < 1e-9                       # fingers along base z
    assert xm.roll_of(r1) == pytest.approx(roll)
    assert xm.tilt_deg(r0, r1) == pytest.approx(90.0)                        # HOLD_LEVEL can't do it


def test_roll_sign_is_ccw_seen_facing_the_fingers():
    # +90 right-handed about the approach (+x here): the top of the hand (tool y, base +z) goes to base -y,
    # i.e. to the left of someone FACING the fingers (looking along -x, their left is -y): counter-clockwise.
    r = xm.target_orientation(0.0, 90.0)
    assert np.allclose(r.as_matrix()[:, 1], [0, -1, 0], atol=1e-9)
    assert np.allclose(xm.target_orientation(0.0, -90.0).as_matrix()[:, 1], [0, 1, 0], atol=1e-9)


def test_tilt_ignores_yaw_but_not_pitch():
    a = xm.target_orientation(0.0, 90.0)
    assert xm.tilt_deg(a, xm.target_orientation(70.0, 90.0)) < 1e-6
    assert xm.tilt_deg(a, a * R.from_euler("x", 10, degrees=True)) < 1e-6   # rolled: tool x is vertical -> a yaw
    pitched = a * R.from_euler("y", 10, degrees=True)                       # tool y is horizontal -> a pitch
    assert xm.tilt_deg(a, pitched) == pytest.approx(10.0, abs=1e-6)


def test_vertical_approach_has_no_heading():
    down = R.from_matrix(np.column_stack([[0, 1, 0], [1, 0, 0], [0, 0, -1]]))
    assert xm.approach_yaw_deg(down) is None
    assert xm.approach_yaw_deg(xm.target_orientation(-40.0, 90.0)) == pytest.approx(-40.0)


# ---- targets and offsets -----------------------------------------------------------------------
def test_tool_goal_frames_and_container_offset():
    rot_now, rot_goal = xm.target_orientation(0, 0), xm.target_orientation(0, 90)
    now = np.array([0.4, 0.1, 0.3])
    assert np.allclose(xm.tool_goal([0.5, 0, 0.2], "base", "tool", rot_goal, now, rot_now), [0.5, 0, 0.2])
    assert np.allclose(xm.tool_goal([0, 0, 0.05], "relative", "tool", rot_goal, now, rot_now), [0.4, 0.1, 0.35])
    off = [0.0, 0.0, 0.10]                                                    # container 10 cm out along the approach
    g = xm.tool_goal([0.6, 0, 0.2], "base", "container", rot_goal, now, rot_now, off)
    assert np.allclose(g, [0.5, 0, 0.2])
    off = [0.03, 0.0, 0.10]                                                   # ... and 3 cm along the finger axis
    g = xm.tool_goal([0.6, 0, 0.2], "base", "container", rot_goal, now, rot_now, off)
    assert np.allclose(g + rot_goal.apply(off), [0.6, 0, 0.2])
    assert np.allclose(rot_goal.apply(off), [0.10, 0, 0.03])                 # finger axis is vertical after the roll
    with pytest.raises(xm.Refused):
        xm.tool_goal([0.6, 0, 0.2], "base", "container", rot_goal, now, rot_now, None)


def test_relative_container_keeps_the_centre_offset_consistent():
    rot = xm.target_orientation(20, 90)
    now, off = np.array([0.45, -0.1, 0.3]), np.array([0.0, 0.02, 0.12])
    g = xm.tool_goal([0, 0, 0.04], "relative", "container", rot, now, rot, off)
    assert np.allclose(g, now + [0, 0, 0.04])                                 # same orientation: tool moves by the offset


# ---- sim FK and the gates ----------------------------------------------------------------------
def test_fk_inverts_solve_ik(sim):
    rot = xm.target_orientation(0, 90)
    q, err = mc.solve_ik(*sim, [0.5, -0.1, 0.3], rot.as_quat(), PARK)
    for _ in range(4):
        q, err = mc.solve_ik(*sim, [0.5, -0.1, 0.3], rot.as_quat(), q)
    pos, r = xm.fk(sim, q)
    assert np.linalg.norm(pos - [0.5, -0.1, 0.3]) < 0.002


def test_j7_roll_turns_in_place(sim):
    _, rot0 = xm.fk(sim, PARK)
    goal = rot0 * R.from_euler("z", 90, degrees=True)                         # same approach, rolled 90
    q, left = xm.j7_roll(sim, PARK, goal)
    assert q is not None and np.allclose(q[:6], PARK[:6]) and left < 0.5
    p0, _ = xm.fk(sim, PARK)
    p1, r1 = xm.fk(sim, q)
    assert np.linalg.norm(p1 - p0) < 1e-3                                     # the tool point stays put
    assert np.degrees((r1 * goal.inv()).magnitude()) < 0.5
    q, left = xm.j7_roll(sim, PARK, xm.target_orientation(0, 90) * R.from_euler("y", 30, degrees=True))
    assert q is not None and np.allclose(q[:6], PARK[:6]) and left > 5.0    # J7 does the twist, not the pitch


def test_gates_refuse_jumps_deviation_and_a_wrong_end(sim):
    p0, r0 = xm.fk(sim, PARK)
    g = xm.Gates()
    jump = np.array([PARK, PARK + np.radians([0, 0, 0, 0, 0, 0, 15])])
    with pytest.raises(xm.Refused, match="joint step"):
        xm.gate_path(sim, jump, p0, r0, False, g, "t")
    with pytest.raises(xm.Refused, match="off the goal"):
        xm.gate_path(sim, np.array([PARK, PARK]), p0 + [0.02, 0, 0], r0, False, g, "t")
    lp = LinePlanner(sim)
    qs = lp.plan(p0 + [0, 0.08, 0], r0, PARK, xm.HOLD_NONE)
    with pytest.raises(xm.Refused, match="straight line"):                  # this path, judged against another goal
        xm.gate_path(sim, qs, p0 + [0, 0, 0.001], r0, False, xm.Gates(max_deviation=0.02), "t")


# ---- whole plans -------------------------------------------------------------------------------
def test_plan_from_park_rolls_then_holds_level(sim):
    lp = LinePlanner(sim)
    legs = xm.plan_move(sim, lp, PARK, [0.45, -0.20, 0.30], roll_deg=90.0, log=quiet)
    assert [l.name for l in legs] == ["roll", "level", "move"]            # park is pitched: J7 rolls, cuRobo levels
    assert lp.calls == [xm.HOLD_NONE, xm.HOLD_LEVEL]
    assert np.allclose(legs[0].joints[-1][:6], PARK[:6])                      # the roll is J7 alone
    pos, rot = xm.fk(sim, legs[-1].joints[-1])
    assert np.linalg.norm(pos - [0.45, -0.20, 0.30]) < xm.END_POS_TOL_M
    assert xm.roll_of(rot) == pytest.approx(90.0, abs=1.0)                   # HOLD_LEVEL kept the roll
    assert legs[-1].stats["tilt_deg"] <= xm.HOLD_TOL_DEG


def test_hold_level_alone_would_refuse_the_roll(sim):
    lp = LinePlanner(sim)
    _, r0 = xm.fk(sim, PARK)
    with pytest.raises(xm.Refused, match="CONSTRAINT_UNSATISFIABLE"):
        lp.plan([0.45, -0.2, 0.3], xm.target_orientation(xm.approach_yaw_deg(r0), 90.0), PARK, xm.HOLD_LEVEL)


def test_already_rolled_start_uses_one_held_leg_and_j7_for_a_pure_roll(sim):
    lp = LinePlanner(sim)
    legs = xm.plan_move(sim, lp, PARK, [0.45, -0.20, 0.30], roll_deg=90.0, log=quiet)
    q_rolled = legs[-1].joints[-1]
    lp2 = LinePlanner(sim)
    legs2 = xm.plan_move(sim, lp2, q_rolled, [0.0, 0.0, 0.05], frame="relative", roll_deg=90.0, log=quiet)
    assert [l.name for l in legs2] == ["move"] and lp2.calls == [xm.HOLD_LEVEL]
    lp3 = LinePlanner(sim)                                                    # back to roll 0: J7 alone, in place
    legs3 = xm.plan_move(sim, lp3, q_rolled, [0.0, 0.0, 0.0], frame="relative", roll_deg=0.0, log=quiet)
    assert legs3[0].name == "roll" and lp3.calls == [xm.HOLD_LEVEL]
    assert np.allclose(legs3[0].joints[-1][:6], q_rolled[:6])


def test_container_target_lands_the_centre(sim):
    lp = LinePlanner(sim)
    legs = xm.plan_move(sim, lp, PARK, [0.45, -0.20, 0.30], roll_deg=90.0, log=quiet)
    q = legs[-1].joints[-1]
    off = [0.0, 0.0, 0.08]
    legs = xm.plan_move(sim, LinePlanner(sim), q, [0.55, -0.15, 0.25], target="container", roll_deg=90.0,
                        grasp_offset=off, log=quiet)
    pos, rot = xm.fk(sim, legs[-1].joints[-1])
    assert np.linalg.norm(pos + rot.apply(off) - [0.55, -0.15, 0.25]) < xm.END_POS_TOL_M


def test_container_refuses_to_roll_the_held_box(sim):
    with pytest.raises(xm.Refused, match="held container would be rolled"):
        xm.plan_move(sim, LinePlanner(sim), PARK, [0.5, -0.2, 0.3], target="container", roll_deg=90.0,
                     grasp_offset=[0, 0, 0.08], log=quiet)


@pytest.mark.parametrize("xyz, why", [([1.2, 0.0, 0.3], "outside reach"), ([0.5, 0.0, 0.02], "outside reach"),
                                      ([0.5, 0.0, 0.90], "outside reach")])
def test_unreachable_goals_are_refused_before_planning(sim, xyz, why):
    lp = LinePlanner(sim)
    with pytest.raises(xm.Refused, match=why):
        xm.plan_move(sim, lp, PARK, xyz, log=quiet)
    assert lp.calls == []


def test_planner_failure_is_a_refusal(sim):
    class NoPath:
        def plan(self, *a):
            raise xm.Refused("cuRobo: IK_FAIL")
    with pytest.raises(xm.Refused, match="IK_FAIL"):
        xm.plan_move(sim, NoPath(), PARK, [0.5, -0.2, 0.3], log=quiet)


def test_cli_parsing_and_config(tmp_path):
    a = xm.parse_args(["--xyz", "0.4", "0", "0.3"])
    assert a.frame == "base" and a.target == "tool" and a.roll_deg == 90.0 and not a.execute
    c = tmp_path / "t.yaml"
    c.write_text("xyz: [0.5, 0.1, 0.2]\ntarget: container\ngrasp_offset: [0, 0, 0.1]\nroll_deg: -90\n")
    a = xm.parse_args(["--config", str(c)])
    assert a.xyz == [0.5, 0.1, 0.2] and a.target == "container" and a.roll_deg == -90
    assert xm.parse_args(["--config", str(c), "--roll-deg", "0"]).roll_deg == 0     # command line wins
    c.write_text("xyz: [0.5, 0.1, 0.2]\nexecute: true\n")
    with pytest.raises(SystemExit):
        xm.parse_args(["--config", str(c)])                                  # --execute never from a file
    with pytest.raises(SystemExit):
        xm.parse_args(["--xyz", "0", "0", "0", "--start-joints", *["0"] * 7, "--execute"])
