"""Move the tool -- or the centre of a held container -- to an XYZ, with the gripper rolled about its
approach axis (default 90 deg CCW), on sheppy: cuRobo plans, the arm driver executes.

    source microwave/sheppy/env.sh      # sheppy profile real-arm-camera-planner (arm driver + cuRobo planner)
    python3 -u microwave/xyz_move.py --xyz 0.45 0.00 0.30                  # dry run: plan + gates, nothing moves
    python3 -u microwave/xyz_move.py --xyz 0.45 0.00 0.30 --execute        # attended, e-stop in hand
    python3 -u microwave/xyz_move.py --frame relative --xyz 0 0 0.05 --roll-deg 0     # 5 cm up, no roll
    python3 -u microwave/xyz_move.py --target container --grasp-offset 0 0 0.10 --xyz 0.50 0.0 0.20
    python3 -u microwave/xyz_move.py --start-joints <7 deg> --xyz ...      # no arm: plan from given joints
    python3 -u microwave/xyz_move.py --config microwave/xyz_move.yaml      # the same options from a file
Run from the repo root (the sim config path is relative).

FRAMES AND UNITS. Metres, degrees on the command line. `--frame base` (default): the target is in the
arm's `base_link` (= `arm_base_link`; +x forward, +z up, origin at the arm base) -- the frame of the
driver's /ee_state, of cuRobo and of the detectors. `--frame relative`: the target is an offset from
where the chosen point is now, along the base axes.
TOOL CENTRE = Kinova's `tool_frame`: the flange (`end_effector_link`, /ee_state) + 0.120 m along the
flange z, between the fingertips -- what SheppyArm.get_state()["ee_pos"] and cuRobo's ee_link both mean.
`--target container`: the XYZ is the held container's centre; the tool goes to
centre - R_target @ grasp_offset, `--grasp-offset` being tool frame -> container centre IN THE TOOL
FRAME (x = finger-closing axis, y, z = approach). No default: it has to be measured.

ORIENTATION. Tool axes: z = approach (out between the fingers), x = the axis the fingers close along,
y = z cross x. "Horizontal" (roll 0) is the handle grasp that closed on the vertical bar on hardware
(handle_detect.HANDLE_QUAT rotated pi about y): approach level, fingers closing horizontally, wrist
camera on top. The target orientation is that, yawed to `--yaw-deg` (default: the current approach
direction), then rolled `--roll-deg` about the approach axis. roll_deg is right-handed about the
approach (pointing away from the wrist) -- placement/camera_roll.py's convention: +90 is CCW seen from
the FRONT (facing the fingers), CW seen from behind the wrist. Either sign leaves the fingers closing
vertically; the sign decides which side the wrist camera ends up on, which the dry run prints.

TWO LEGS.
 1. roll (only when the tilt has to change): the turn about the approach axis as a joint move of J7
    alone -- the tool point is on J7's axis, so it turns in place (sign found in sim FK). Whatever is
    left (the approach itself pointing elsewhere, e.g. park's 15 deg nose-down) is a separate in-place
    cuRobo leg, HOLD_NONE to the current position, the tool kept within --max-deviation of it.
 2. move: cuRobo plan_to_pose with HOLD_LEVEL to (target position, target orientation). HOLD_LEVEL keeps
    the tool's tilt -- incl. the roll from leg 1 -- the whole way and only frees spin about base z in
    transit; the plan must end at the full target orientation. It refuses a goal whose tilt differs from
    the start's, which is why the roll is its own leg first.
Every plan is gated here before anything moves (sim FK of every point: reach, z band, J4/J6 guards,
joint step, distance from the straight line, held tilt, end pose), then executed exactly as gated by
SheppyArm.move_joints -> /execute_joint_trajectory: position control, or joint impedance (--impedance,
the driver's starting gains, sheppy_arm.IMPEDANCE) where the path tolerance then also acts as a force stop.

cuRobo's collision world (gen3_real.yaml) is a placeholder table + pedestal: it does NOT contain the
microwave, the counter, the wheelchair or the user. The straight-line gate is what keeps the path near
the segment you asked for; clear that segment yourself.
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pybullet as p
import yaml
from scipy.spatial.transform import Rotation as R

from microwave_common import ARM, CONTINUOUS, J4_GUARD_DEG, J6_GUARD_DEG, short_delta

# "Horizontal" gripper, approach along base +x: tool x -> base +y (fingers close horizontally), tool y ->
# base +z (wrist camera on top), tool z -> base +x. = R.from_quat(handle_detect.HANDLE_QUAT) * Ry(pi).
R_HORIZONTAL = R.from_matrix(np.column_stack([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]))

# Gates on a cuRobo path: rammp_button_press's (RammpArm.gate_joint_path), which ran attended on this arm.
MAX_REACH_M = 0.91
Z_RANGE_M = (0.12, 0.75)
MAX_JOINT_STEP_DEG = 10.0          # between consecutive points of a plan (cuRobo's are 20 ms apart)
END_POS_TOL_M = 0.005
END_ROT_TOL_DEG = 1.0
HOLD_TOL_DEG = 2.0                 # = cuRobo's constraint_tolerance_deg (it checks the hold itself too)
MAX_DEVIATION_M = 0.05             # tool point off the straight start->goal segment (new; conservative)
ROLL_MATCH_DEG = 1.0               # a J7-only roll must land this close to the target orientation
START_DRIFT_DEG = 0.5              # the arm must still be where a leg was planned from when it is sent
TRACK_ABORT_M = 0.01               # executed: final tool position this far off the target -> stop
CUROBO_ACTION = "/rammp_curobo/plan_to_pose"
CUROBO_TIMEOUT_S = 30.0
HOLD_NONE, HOLD_LEVEL = 0, 1       # PlanToPose.Goal constants (checked against the message at runtime)
CALIB = Path.home() / ".ros2/easy_handeye2/calibrations/wrist_camera_calib.calib"


class Refused(RuntimeError):
    """A gate or the planner said no. Nothing was commanded for the leg that raised it."""


# ---- orientation and targets (pure) ------------------------------------------------------------
def approach_yaw_deg(rot):
    """Heading of the tool's approach axis in the base xy plane (deg); None if it points ~vertically."""
    a = rot.as_matrix()[:, 2]
    if np.hypot(a[0], a[1]) < 0.17:                 # within ~10 deg of vertical: no heading
        return None
    return float(np.degrees(np.arctan2(a[1], a[0])))


def target_orientation(yaw_deg, roll_deg):
    """Horizontal gripper yawed `yaw_deg` about base z, rolled `roll_deg` about its approach axis."""
    return R.from_euler("z", yaw_deg, degrees=True) * R_HORIZONTAL * R.from_euler("z", roll_deg, degrees=True)


def roll_of(rot):
    """Roll about the approach relative to the horizontal reference with the same heading (deg). Only
    meaningful when the approach is level."""
    yaw = approach_yaw_deg(rot)
    if yaw is None:
        return None
    rel = (R.from_euler("z", yaw, degrees=True) * R_HORIZONTAL).inv() * rot
    return float(np.degrees(rel.as_rotvec()[2]))


def tilt_deg(rot_a, rot_b):
    """How differently two tool orientations are tilted -- the components HOLD_LEVEL holds: the angle
    between base +z seen in each tool frame (a spin about base z changes nothing)."""
    up = np.array([0.0, 0.0, 1.0])
    ua, ub = rot_a.inv().apply(up), rot_b.inv().apply(up)
    return float(np.degrees(np.arccos(np.clip(ua @ ub, -1.0, 1.0))))


def tool_goal(xyz, frame, target, rot_goal, tool_now, rot_now, grasp_offset=None):
    """Tool-centre goal (base frame, m) for the requested point. `target` 'tool' or 'container'
    (centre = tool + R @ grasp_offset, offset in the tool frame); `frame` 'base' or 'relative' (an offset
    from where that point is now, base axes)."""
    xyz, tool_now = np.asarray(xyz, float), np.asarray(tool_now, float)
    if target == "tool":
        off = np.zeros(3)
    elif target == "container":
        if grasp_offset is None:
            raise Refused("--target container needs --grasp-offset (tool frame -> container centre, measured)")
        off = np.asarray(grasp_offset, float)
    else:
        raise ValueError(f"target {target!r}: 'tool' or 'container'")
    if frame == "base":
        point = xyz
    elif frame == "relative":
        point = tool_now + rot_now.apply(off) + xyz
    else:
        raise ValueError(f"frame {frame!r}: 'base' or 'relative'")
    return point - rot_goal.apply(off)


def describe(rot):
    """Plain words for an orientation: approach, finger-closing axis, where the wrist camera sits."""
    m = rot.as_matrix()
    words = {0: "x", 1: "y", 2: "z"}

    def axis(v):
        k = int(np.argmax(np.abs(v)))
        return f"{'+' if v[k] > 0 else '-'}{words[k]} ({np.round(v, 2).tolist()})"
    out = f"approach {axis(m[:, 2])}, fingers close along {axis(m[:, 0])}"
    try:
        t = yaml.safe_load(CALIB.read_text())["transform"]["translation"]
        cam = rot.apply([t["x"], t["y"], t["z"] - 0.120])       # camera relative to the tool centre
        a = m[:, 2]
        side = np.cross([0.0, 0.0, 1.0], a)                     # the hand's left, seen from behind the wrist
        side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-6 else m[:, 1]
        lr, ud = float(cam @ side), float(cam[2])
        out += (f", wrist camera {abs(lr) * 100:.1f} cm {'left' if lr > 0 else 'right'} / {abs(ud) * 100:.1f} cm "
                f"{'above' if ud > 0 else 'below'} the tool (left/right seen from behind the wrist)")
    except Exception:  # noqa: BLE001 -- the calibration file is only for this printout
        pass
    return out


# ---- sim FK gate (pure sim) --------------------------------------------------------------------
def fk(sim, q):
    """Tool-frame pose (base frame) at joints q, from the repo's PyBullet Gen3 (its ee link IS tool_frame)."""
    scene, rb = sim
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=rb.physics_client_id)
    w = rb.get_end_effector_pose()
    b = scene.robot_base_pose
    rb_inv = R.from_quat(b.orientation).inv()
    return rb_inv.apply(np.asarray(w.position) - np.asarray(b.position)), rb_inv * R.from_quat(w.orientation)


def seg_distance(x, a, b):
    ab = b - a
    t = 0.0 if ab @ ab < 1e-12 else float(np.clip((x - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(x - (a + t * ab)))


@dataclass
class Gates:
    max_reach: float = MAX_REACH_M
    z_range: tuple = Z_RANGE_M
    max_step_deg: float = MAX_JOINT_STEP_DEG
    max_deviation: float = MAX_DEVIATION_M


def gate_path(sim, qs, p_goal, rot_goal, hold_level, gates, name):
    """Refuse a joint path unless every point's FK is in reach / the z band / the joint guards, no step
    jumps, the tool stays near the straight segment from its start to `p_goal` and (HOLD_LEVEL) keeps the
    goal's tilt, and the end is on the goal pose. Returns a stats dict."""
    p_start, _ = fk(sim, qs[0])
    worst = {"step_deg": 0.0, "deviation_m": 0.0, "tilt_deg": 0.0}
    for i, q in enumerate(qs):
        pos, rot = fk(sim, q)
        bad = []
        if np.linalg.norm(pos) > gates.max_reach:
            bad.append(f"reach {np.linalg.norm(pos):.3f} > {gates.max_reach}")
        if not gates.z_range[0] <= pos[2] <= gates.z_range[1]:
            bad.append(f"z {pos[2]:.3f} outside {gates.z_range}")
        if abs(np.degrees(q[3])) > J4_GUARD_DEG or abs(np.degrees(q[5])) > J6_GUARD_DEG:
            bad.append(f"J4 {np.degrees(q[3]):.1f} / J6 {np.degrees(q[5]):.1f} past the guards "
                       f"({J4_GUARD_DEG}/{J6_GUARD_DEG})")
        if i:
            step = float(np.degrees(np.max(np.abs(short_delta(qs[i - 1], q)))))
            worst["step_deg"] = max(worst["step_deg"], step)
            if step > gates.max_step_deg:
                bad.append(f"joint step {step:.1f} deg > {gates.max_step_deg}")
        dev = seg_distance(pos, p_start, np.asarray(p_goal, float))
        worst["deviation_m"] = max(worst["deviation_m"], dev)
        if dev > gates.max_deviation:
            bad.append(f"tool {dev * 100:.1f} cm off the straight line (max {gates.max_deviation * 100:.0f})")
        if hold_level:
            t = tilt_deg(rot, rot_goal)
            worst["tilt_deg"] = max(worst["tilt_deg"], t)
            if t > HOLD_TOL_DEG:
                bad.append(f"tilt {t:.1f} deg off the held one (max {HOLD_TOL_DEG})")
        if bad:
            raise Refused(f"{name}: point {i + 1}/{len(qs)}: {'; '.join(bad)}")
    pos, rot = fk(sim, qs[-1])
    worst["end_pos_m"] = float(np.linalg.norm(pos - p_goal))
    worst["end_rot_deg"] = float(np.degrees((rot * rot_goal.inv()).magnitude()))
    if worst["end_pos_m"] > END_POS_TOL_M or worst["end_rot_deg"] > END_ROT_TOL_DEG:
        raise Refused(f"{name}: ends {worst['end_pos_m'] * 1000:.1f} mm / {worst['end_rot_deg']:.1f} deg off the "
                      f"goal (max {END_POS_TOL_M * 1000:.0f} mm / {END_ROT_TOL_DEG} deg)")
    return worst


def j7_roll(sim, q0, rot_goal):
    """Joints that turn the tool about its own approach axis -- J7 alone, so the tool point stays put -- as
    far toward `rot_goal` as a turn about that axis gets (the twist part of the rotation still needed).
    Returns (joints, orientation error left in deg); joints None if the twist is under ROLL_MATCH_DEG.
    The J7 sign is found in sim FK, not assumed."""
    _, rot0 = fk(sim, q0)
    x, y, z, w = (rot0.inv() * rot_goal).as_quat()                 # needed rotation, tool frame
    twist = 2.0 * np.arctan2(z, w)                                 # its component about tool z
    twist = (twist + np.pi) % (2 * np.pi) - np.pi
    if abs(np.degrees(twist)) < ROLL_MATCH_DEG:
        return None, float(np.degrees((rot0 * rot_goal.inv()).magnitude()))
    best = None
    for sign in (1.0, -1.0):
        q = np.asarray(q0, float).copy()
        q[6] += sign * twist
        _, rot = fk(sim, q)
        err = float(np.degrees((rot * rot_goal.inv()).magnitude()))
        if best is None or err < best[1]:
            best = (q, err)
    return best


# ---- cuRobo (Sheppy's planner) -----------------------------------------------------------------
class CuroboPlanner:
    """/rammp_curobo/plan_to_pose from a running node (planning only -- nothing here moves the arm)."""

    def __init__(self, node):
        from rclpy.action import ActionClient
        from rammp_curobo_interfaces.action import PlanToPose
        assert (PlanToPose.Goal.HOLD_NONE, PlanToPose.Goal.HOLD_LEVEL) == (HOLD_NONE, HOLD_LEVEL)
        self._Goal = PlanToPose.Goal
        self._client = ActionClient(node, PlanToPose, CUROBO_ACTION)

    def plan(self, pos, rot, start_q, hold):
        """Joint path (N, 7) from start_q (driver convention, unwrapped from it) to the tool pose."""
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise Refused(f"no cuRobo planner ({CUROBO_ACTION}) -- sheppy profile real-arm-camera-planner up?")
        g = self._Goal()
        g.target.position.x, g.target.position.y, g.target.position.z = map(float, pos)
        q = rot.as_quat()
        g.target.orientation.x, g.target.orientation.y, g.target.orientation.z, g.target.orientation.w = map(float, q)
        g.start_joints = [float(v) for v in start_q]
        g.hold = int(hold)
        gh = _wait(self._client.send_goal_async(g), 10.0)
        if gh is None or not gh.accepted:
            raise Refused("cuRobo refused the request")
        res = _wait(gh.get_result_async(), CUROBO_TIMEOUT_S)
        if res is None:
            gh.cancel_goal_async()
            raise Refused(f"cuRobo did not answer in {CUROBO_TIMEOUT_S:.0f} s")
        r = res.result
        if not r.success:
            raise Refused(f"cuRobo: {r.message}")
        names = list(r.trajectory.joint_names)
        idx = [names.index(f"joint_{i}") for i in range(1, 8)]
        qs = [np.asarray(pt.positions, float)[idx] for pt in r.trajectory.points]
        out, prev = [], np.asarray(start_q, float)
        for k in qs:                                       # continuous joints the short way, from start_q
            d = k - prev
            d[CONTINUOUS] = (d[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
            prev = prev + d
            out.append(prev.copy())
        return np.array(out)


def _wait(fut, timeout_s):
    t0 = time.time()
    while not fut.done():
        if time.time() - t0 > timeout_s:
            return None
        time.sleep(0.01)
    return fut.result()


# ---- plan + execute ----------------------------------------------------------------------------
@dataclass
class Leg:
    name: str
    joints: np.ndarray        # (N, 7), joints[0] = where it starts
    goal_pos: np.ndarray
    goal_rot: R
    stats: dict
    hold: int | None = None   # the cuRobo hold it was planned with; None = J7-only (not re-plannable)


def plan_move(sim, planner, q0, xyz, frame="base", target="tool", roll_deg=90.0, yaw_deg=None,
              grasp_offset=None, gates=None, log=print):
    """The legs (roll if needed, then the HOLD_LEVEL move) from joints q0, gated. Raises Refused."""
    gates = gates or Gates()
    q0 = np.asarray(q0, float)
    p_now, rot_now = fk(sim, q0)
    if yaw_deg is None:
        yaw_deg = approach_yaw_deg(rot_now)
        if yaw_deg is None:
            raise Refused("the approach points ~vertically, so it has no heading -- give --yaw-deg")
    rot_goal = target_orientation(yaw_deg, roll_deg)
    p_goal = tool_goal(xyz, frame, target, rot_goal, p_now, rot_now, grasp_offset)
    log(f"now : tool {np.round(p_now, 3).tolist()}  {describe(rot_now)}")
    log(f"goal: tool {np.round(p_goal, 3).tolist()}  {describe(rot_goal)}")
    log(f"      yaw {yaw_deg:+.1f} deg, roll {roll_deg:+.1f} deg (+ = CCW seen facing the fingers); "
        f"tilt change {tilt_deg(rot_now, rot_goal):.1f} deg")
    if np.linalg.norm(p_goal) > gates.max_reach or not gates.z_range[0] <= p_goal[2] <= gates.z_range[1]:
        raise Refused(f"goal {np.round(p_goal, 3).tolist()} outside reach {gates.max_reach} m / z band {gates.z_range}")
    legs, q = [], q0
    if tilt_deg(rot_now, rot_goal) > HOLD_TOL_DEG:
        if target == "container":
            raise Refused(f"the held container would be rolled {tilt_deg(rot_now, rot_goal):.0f} deg: roll before "
                          f"grasping (--target tool), or pass the current roll (now {roll_of(rot_now)})")
        qr, left = j7_roll(sim, q, rot_goal)
        if qr is not None:                         # 1a. the turn about the approach: J7 alone, in place
            n = max(2, int(np.ceil(abs(qr[6] - q[6]) / np.radians(gates.max_step_deg / 2))) + 1)
            qs = np.linspace(q, qr, n)
            _, rot_r = fk(sim, qr)
            stats = gate_path(sim, qs, p_now, rot_r, False, gates, "roll")
            legs.append(Leg("roll", qs, p_now, rot_r, stats, None))
            log(f"leg roll : J7 alone {np.degrees(qr[6] - q[6]):+.1f} deg, tool point fixed; "
                f"{left:.1f} deg of tilt left to the goal")
            q = qr
        if left > ROLL_MATCH_DEG:                   # 1b. the rest (e.g. pitch): cuRobo, in place
            qs = planner.plan(p_now, rot_goal, q, HOLD_NONE)
            stats = gate_path(sim, qs, p_now, rot_goal, False, gates, "level")
            legs.append(Leg("level", qs, p_now, rot_goal, stats, HOLD_NONE))
            log(f"leg level: cuRobo HOLD_NONE in place, {len(qs)} pts, tool within "
                f"{stats['deviation_m'] * 100:.1f} cm of where it was")
            q = qs[-1]
    qs = planner.plan(p_goal, rot_goal, q, HOLD_LEVEL)
    stats = gate_path(sim, qs, p_goal, rot_goal, True, gates, "move")
    legs.append(Leg("move", qs, p_goal, rot_goal, stats, HOLD_LEVEL))
    log(f"leg move : cuRobo HOLD_LEVEL, {len(qs)} pts, joint step <= {stats['step_deg']:.1f} deg, "
        f"<= {stats['deviation_m'] * 100:.1f} cm off the line, tilt held to {stats['tilt_deg']:.2f} deg, "
        f"ends {stats['end_pos_m'] * 1000:.1f} mm / {stats['end_rot_deg']:.2f} deg off")
    _, rot_end = fk(sim, qs[-1])
    log(f"end : roll {roll_of(rot_end):+.1f} deg (asked {roll_deg:+.1f}) -- {describe(rot_end)}"
        if roll_of(rot_end) is not None else f"end : {describe(rot_end)}")
    return legs


def execute_legs(arm, sim, planner, legs, gates=None, impedance=False, speed_scale=1.0, log=print):
    """Each leg as gated, through SheppyArm.move_joints. The move leg after a roll is re-planned (and
    re-gated) from where the roll actually ended. Stops at the first problem."""
    import sheppy_arm
    gates = gates or Gates()
    for k, leg in enumerate(legs):
        drift = float(np.degrees(np.max(np.abs(short_delta(arm.raw_joints(), leg.joints[0])))))
        if drift > START_DRIFT_DEG and k > 0 and leg.hold is not None:
            qs = planner.plan(leg.goal_pos, leg.goal_rot, arm.raw_joints(), leg.hold)
            leg = Leg(leg.name, qs, leg.goal_pos, leg.goal_rot,
                      gate_path(sim, qs, leg.goal_pos, leg.goal_rot, leg.hold == HOLD_LEVEL, gates,
                                f"{leg.name} (re-planned)"), leg.hold)
            log(f"{leg.name}: re-planned from where the last leg really ended ({drift:.1f} deg off), gates pass")
        elif drift > START_DRIFT_DEG:
            raise Refused(f"{leg.name}: the arm moved {drift:.1f} deg since it was planned -- nothing sent")
        tool = [fk(sim, q) for q in leg.joints]
        lengths = [max(1e-4, float(np.linalg.norm(b[0] - a[0])), (b[1] * a[1].inv()).magnitude() * sheppy_arm.ROT_EQUIV_M)
                   for a, b in zip(tool[:-1], tool[1:])]
        ok = arm.move_joints(list(leg.joints[1:]), cart_lengths=lengths, speed_scale=speed_scale,
                             impedance=sheppy_arm.IMPEDANCE if impedance else None)
        time.sleep(0.3)
        ee = np.asarray(arm.get_state()["ee_pos"], float)
        err = float(np.linalg.norm(ee[:3] - leg.goal_pos))
        rerr = float(np.degrees((R.from_quat(ee[3:7]) * leg.goal_rot.inv()).magnitude()))
        log(f"{leg.name}: {'done' if ok else 'STOPPED'} -- tool {np.round(ee[:3], 3).tolist()}, "
            f"{err * 1000:.1f} mm / {rerr:.1f} deg from the goal")
        if not ok:
            raise Refused(f"{leg.name}: the driver stopped the trajectory -- holding here")
        if err > TRACK_ABORT_M:
            raise Refused(f"{leg.name}: ended {err * 100:.1f} cm from the goal -- holding here")


# ---- CLI ---------------------------------------------------------------------------------------
def parse_args(argv=None):
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--config", type=Path, help="YAML with any of the options below (keys use _); the command line wins")
    a.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"), help="m, in --frame")
    a.add_argument("--frame", choices=("base", "relative"), default="base")
    a.add_argument("--target", choices=("tool", "container"), default="tool")
    a.add_argument("--grasp-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="tool frame -> held container centre, in the TOOL frame (m); required for --target container")
    a.add_argument("--roll-deg", type=float, default=90.0, help="about the approach; + = CCW seen facing the fingers")
    a.add_argument("--yaw-deg", type=float, help="approach heading about base z (default: the current one)")
    a.add_argument("--max-deviation", type=float, default=MAX_DEVIATION_M, help="m off the straight line")
    a.add_argument("--z-min", type=float, default=Z_RANGE_M[0])
    a.add_argument("--z-max", type=float, default=Z_RANGE_M[1])
    a.add_argument("--impedance", action="store_true", help="execute in joint impedance (default: position)")
    a.add_argument("--speed-scale", type=float, default=0.67,
                   help="x SheppyArm's speed caps (0.67 ~ the 0.06 m/s, 0.35 rad/s first-run speeds)")
    a.add_argument("--start-joints", type=float, nargs=7, metavar="DEG",
                   help="plan from these joints instead of the arm's (no arm needed; never executes)")
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run")
    args = a.parse_args(argv)
    if args.config:
        cfg = yaml.safe_load(args.config.read_text()) or {}
        unknown = set(cfg) - (set(vars(args)) - {"execute", "config"})   # --execute only on the command line
        if unknown:
            a.error(f"unknown keys in {args.config}: {sorted(unknown)}")
        a.set_defaults(**cfg)
        args = a.parse_args(argv)
    if args.xyz is None:
        a.error("--xyz (or xyz: in --config) is required")
    if args.start_joints is not None and args.execute:
        a.error("--start-joints plans from given joints: it never executes")
    if not 0.01 <= args.speed_scale <= 1.0:
        a.error("--speed-scale 0.01..1.0")
    return args


def main(argv=None):
    args = parse_args(argv)
    from microwave_common import make_sim
    sim = make_sim()
    arm = None
    if args.start_joints is not None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        import threading
        rclpy.init()
        node = rclpy.create_node("xyz_move_planner")
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        threading.Thread(target=ex.spin, daemon=True).start()
        q0 = np.radians(args.start_joints)
    else:
        from sheppy_arm import SheppyArm
        arm = SheppyArm(node_name="xyz_move")
        node, q0 = arm.node, arm.raw_joints()
        print(f"gripper {arm.get_state()['gripper_pos']:.3f}  joints {np.round(np.degrees(q0), 1).tolist()}")
    gates = Gates(z_range=(args.z_min, args.z_max), max_deviation=args.max_deviation)
    try:
        planner = CuroboPlanner(node)
        legs = plan_move(sim, planner, q0, args.xyz, args.frame, args.target, args.roll_deg,
                         args.yaw_deg, args.grasp_offset, gates)
        if not args.execute:
            print("\nDRY RUN -- planned and gated, nothing commanded.")
            return 0
        execute_legs(arm, sim, planner, legs, gates, impedance=args.impedance, speed_scale=args.speed_scale)
        print("DONE")
        return 0
    except Refused as e:
        print(f"REFUSED: {e}" + ("" if args.execute else " (dry run)"))
        return 2


if __name__ == "__main__":
    sys.exit(main())
