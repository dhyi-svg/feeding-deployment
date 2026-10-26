"""Shared motion pieces for the microwave open/close scripts on `rchi-cpu-5`.

Used by `real_gen3_ros2_grasp_and_swing_microwave.py` (release after opening) and
`real_gen3_ros2_close_microwave.py` (re-grasp before closing, release after the push).

* `release_and_back_off` -- records the current grasp (pose + joints) to
  `LAST_GRASP_FILE`, opens the gripper, then backs straight off the handle along the
  gripper's approach axis in ~2 cm Cartesian steps.
* `regrasp` -- for the close task, from ANY arm pose (other tasks run between opening
  and closing): cuRobo to the recorded back-off joints (goal door-model-checked), then
  straight back onto the recorded grasp, then close. No re-detection (the detector's
  plane fit assumes a door facing the camera, which an open door is not). Needs the
  door geometry the grasp/swing save to `DOOR_FILE`.

cuRobo (RAMMP's planner, through the driver's go_to_ee_pose / go_to_joint_config) does the
free-space moves: to park, to the regrasp back-off point, to the open-door view -- as
rammp_button_press does. Everything on or along the door (straight on/off the handle, the
swings, the pushes) stays on joint paths planned and checked here: the planner takes a goal
only and can't follow a line or an arc. `check_curobo_goal` gates a cuRobo goal.

All gripper motion goes through `execute_command(Open/CloseGripperCommand())` --
`ArmInterfaceClient` has no `open_gripper()` method.
"""
import json, subprocess, threading, time
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.command_interface import (
    CartesianTrajectoryCommand, CloseGripperCommand, JointCommand, OpenGripperCommand)
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

ARM = [1, 2, 3, 4, 5, 6, 7]
LAST_GRASP_FILE = Path.home() / ".microwave_last_grasp.json"
STEP_M = 0.02
MAX_IK_ERR = 0.02
MAX_STEP_JUMP_DEG = 20.0
J6_GUARD_DEG = 115.0
# J4's configured soft limit is +-147.8 deg (09-07: a command past it fails METHOD_FAILED; 09-28: a
# Cartesian grasp path drove J4 from -145 into it and Kortex aborted). PyBullet only knows the URDF's
# wider limit, so every sim check gates J4 with a margin.
J4_GUARD_DEG = 144.0
# J1/J3/J5/J7 spin freely; every commanded angle is wrapped to [-180, 180). 09-29 hardware
# test (tools/wrap_test.py, J7 and J3): a JointCommand across +-180 takes the SHORT way, so a
# step is judged (and interpolated for clearance checks) by its short-way change -- 179 -> -179
# is 2 deg. Only a short-way change near 180 (direction ambiguous) is refused.
CONTINUOUS = [0, 2, 4, 6]
MAX_CONTINUOUS_DELTA_DEG = 170.0
PARK_FILE = Path(__file__).resolve().parent / "park_pose.json"


def wrap_joints(q):
    q = np.asarray(q, dtype=float).copy()
    q[CONTINUOUS] = (q[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
    return q


def short_delta(q_from, q_to):
    """q_to - q_from with the free-spinning joints taken the short way round (what Kortex does)."""
    d = np.asarray(q_to, dtype=float) - np.asarray(q_from, dtype=float)
    d[CONTINUOUS] = (d[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
    return d


def continuous_ok(q_from, q_to):
    d = np.degrees(np.abs(short_delta(q_from, q_to)[CONTINUOUS]))
    return bool(np.all(d <= MAX_CONTINUOUS_DELTA_DEG))


def make_sim():
    scene = create_scene_description_from_config(
        "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
    return scene, FeedingDeploymentPyBulletSimulator(scene, use_gui=False).robot


def door_arc_waypoints(start_pose, hinge_position, arc_length_m, waypoint_spacing_m, direction=1,
                       rotate_orientation=True):
    """Waypoints along a door arc about a vertical hinge axis, starting from (not including)
    `start_pose`, at constant height. Copied from `PerceptionInterface._generate_door_arc_waypoints`
    so these scripts don't import the lab's whole perception stack for one function."""
    if arc_length_m <= 0:
        return []
    if waypoint_spacing_m <= 0:
        raise ValueError("waypoint_spacing_m must be > 0")
    if direction not in (-1, 1):
        raise ValueError("direction must be either +1 or -1")
    hx, hy, hz = start_pose.position
    cx, cy, _ = hinge_position
    radius_vec = np.array([hx - cx, hy - cy], dtype=float)
    radius = np.linalg.norm(radius_vec)
    if radius < 1e-8:
        raise ValueError("Handle pose is too close to hinge pose; radius is ~0.")
    start_theta = np.arctan2(radius_vec[1], radius_vec[0])
    total_angle = arc_length_m / radius
    num_segments = max(1, int(np.ceil(arc_length_m / waypoint_spacing_m)))
    start_rot = R.from_quat(start_pose.orientation)
    waypoints = []
    for i in range(1, num_segments + 1):
        delta_angle = direction * i / num_segments * total_angle
        theta = start_theta + delta_angle
        orientation = ((R.from_euler("z", delta_angle) * start_rot).as_quat() if rotate_orientation
                       else start_pose.orientation)
        waypoints.append(Pose(position=(cx + radius * np.cos(theta), cy + radius * np.sin(theta), hz),
                              orientation=orientation))
    return waypoints


KEEP_OUT = None     # fridge only (fridge_push.enable_keep_out): (rb, joints) -> message if the arm is in a keep-out
                    # zone (the wheelchair), else None. None (the microwave): no such check anywhere
KEEP_OUT_ERR = 9.0  # solve_ik's error for a solution in the keep-out zone: fails every IK gate


def keep_out_message(rb, q):
    """Why joints `q` are in the keep-out zone, or None (always None unless a fridge run switched it on)."""
    return None if KEEP_OUT is None else KEEP_OUT(rb, q)


def solve_ik(scene, rb, pos, quat, seed_joints):
    """PyBullet IK seeded from `seed_joints`; returns (joints, position error in m). A solution in the keep-out zone
    (fridge runs only, KEEP_OUT) returns KEEP_OUT_ERR as its error."""
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(seed_joints[j]), physicsClientId=rb.physics_client_id)
    wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
    sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
        list(wpose.position), list(wpose.orientation),
        physicsClientId=rb.physics_client_id, maxNumIterations=200)
    joints = wrap_joints([sol[k] for k in range(7)])
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(joints[j]), physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    err = float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(wpose.position)))
    if KEEP_OUT is not None and KEEP_OUT(rb, joints) is not None:
        return joints, KEEP_OUT_ERR
    return joints, err


def wait_for_joints(ai, q, tol_deg=1.0, timeout_s=6.0):
    """Wait until the arm is within `tol_deg` of the COMMANDED joints (Kortex can
    silently drop a chained command, so velocity ~ 0 is not proof of arrival)."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        cur = np.array(ai.get_state()["position"], dtype=float)
        if np.degrees(np.max(np.abs((cur - q + np.pi) % (2 * np.pi) - np.pi))) < tol_deg:
            return True
        time.sleep(0.05)
    return False


def fit_circle_xy(xy):
    """Least-squares circle through 2-D points (Kasa fit). Returns (centre xy, radius, rms residual m)."""
    xy = np.asarray(xy, dtype=float)
    A = np.c_[2 * xy, np.ones(len(xy))]
    b = (xy ** 2).sum(axis=1)
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    r = float(np.sqrt(c + cx ** 2 + cy ** 2))
    resid = np.linalg.norm(xy - [cx, cy], axis=1) - r
    return np.array([cx, cy]), r, float(np.sqrt(np.mean(resid ** 2)))


def run_cartesian_trajectory(ai, traj, tol_m=0.01, settle_s=2.0, timeout_s=60.0, impedance=None, speed_scale=1.0):
    """Send a CartesianTrajectoryCommand and judge it by where the arm ACTUALLY ends up, not
    the RPC return: the server's wait can be tripped by a stale END/ABORT notification and
    return False at once while Kortex runs the whole trajectory anyway (09-28: returned False
    with the arm still at the start, then reached every waypoint). Waits until the EE is within
    `tol_m` of the last waypoint, or has not moved for `settle_s` (after at least `settle_s`
    had passed, so a not-yet-started motion isn't mistaken for a stop). Returns (reached, err_m, rpc_ok).
    `impedance` (SheppyArm only): run it compliant, e.g. the door swing (SheppyArm.IMPEDANCE).
    `speed_scale` (SheppyArm only): e.g. sheppy_arm.contact_speed_scale() for a move that pushes on the door."""
    if (impedance is not None or speed_scale != 1.0) and hasattr(ai, "move_cartesian"):
        # sheppy: compliant (joint impedance) and/or slowed execution -- see SheppyArm.IMPEDANCE
        rpc_ok = ai.move_cartesian([(list(pp), list(qq)) for pp, qq in traj], impedance=impedance,
                                   speed_scale=speed_scale)
    else:
        rpc_ok = ai.execute_command(CartesianTrajectoryCommand([(list(pp), list(qq)) for pp, qq in traj]))
    goal = np.asarray(traj[-1][0], dtype=float)
    t0 = time.time()
    last = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
    last_move = t0
    while time.time() - t0 < timeout_s:
        cur = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
        err = float(np.linalg.norm(cur - goal))
        if err < tol_m:
            time.sleep(0.3)   # let the final taper finish
            cur = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
            return True, float(np.linalg.norm(cur - goal)), rpc_ok
        if np.linalg.norm(cur - last) > 0.002:
            last, last_move = cur, time.time()
        elif time.time() - last_move > settle_s and time.time() - t0 > settle_s:
            return False, err, rpc_ok
        time.sleep(0.1)
    cur = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
    return False, float(np.linalg.norm(cur - goal)), rpc_ok


def plan_straight_line(scene, rb, start_pos, quat, direction, dist, seed_joints, label):
    """Joint solutions for a straight Cartesian line, ~2 cm per step, fixed orientation.
    Returns the list of joint vectors, or None if any step fails a gate."""
    n = max(1, int(np.ceil(dist / STEP_M)))
    q, plan = np.asarray(seed_joints, dtype=float), []
    for k in range(1, n + 1):
        tgt = np.asarray(start_pos) + np.asarray(direction) * dist * k / n
        nq, err = solve_ik(scene, rb, tgt, quat, q)
        jump = float(np.degrees(np.max(np.abs(short_delta(q, nq)))))
        j6, j7 = float(np.degrees(nq[5])), float(np.degrees(nq[6]))
        print(f"  {label} {k}/{n} -> {np.round(tgt, 3)}  IK err {err * 100:.2f}cm  jump {jump:.1f}deg  "
              f"J6 {j6:.1f}deg  J7 {j7:.1f}deg")
        if (err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or abs(j6) > J6_GUARD_DEG
                or abs(np.degrees(nq[3])) > J4_GUARD_DEG
                or not continuous_ok(q, nq)):
            print(f"  {label}: step {k} fails the IK/jump/J6/wrap gate.")
            return None
        plan.append(nq)
        q = nq
    return plan


BIG_MOVE_TIMEOUT_S = 40.0


def plan_cartesian(scene, rb, p0, q0_quat, p1, q1_quat, seed_joints, label, bodies=None, min_clear=None,
                   closed=False):
    """Straight line p0 -> p1 with the orientation slerped q0 -> q1, ~2 cm / <= 5 deg per step,
    each step IK-solved from the previous one. Gates: IK, per-step jump, J6, wrap and (if
    `bodies`) >= MIN_CLEAR from the door model, checked along each step too."""
    from scipy.spatial.transform import Slerp
    rots = R.from_quat([q0_quat, q1_quat])
    ang = float(np.degrees((rots[0].inv() * rots[1]).magnitude()))
    n = max(1, int(np.ceil(max(np.linalg.norm(np.asarray(p1) - p0) / STEP_M, ang / 5.0))))
    slerp = Slerp([0, 1], rots)
    q, plan, worst = np.asarray(seed_joints, float), [], (9.0, "")
    for k in range(1, n + 1):
        f = k / n
        tgt = np.asarray(p0) + (np.asarray(p1) - p0) * f
        nq, err = solve_ik(scene, rb, tgt, slerp([f]).as_quat()[0], q)
        # one PyBullet IK pass sometimes stalls (joints barely move, ~2.3 cm error), mostly on
        # rotate-in-place steps; re-solving from its own result converges (09-28 sim check)
        for _ in range(3):
            if err <= MAX_IK_ERR:
                break
            nq, err = solve_ik(scene, rb, tgt, slerp([f]).as_quat()[0], nq)
        # wrapped: a free joint going 179.8 -> -179.8 moved 0.4 deg, not 359.6 (Kortex takes the short way)
        jump = float(np.degrees(np.max(np.abs((nq - q + np.pi) % (2 * np.pi) - np.pi))))
        if bodies is not None:
            worst = min([worst] + [clearance(rb, q + short_delta(q, nq) * s_ / 3, bodies, closed) for s_ in (1, 2, 3)],
                        key=lambda w: w[0])
        bad = (err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or abs(np.degrees(nq[5])) > J6_GUARD_DEG
               or abs(np.degrees(nq[3])) > J4_GUARD_DEG
               or not continuous_ok(q, nq) or (bodies is not None and worst[0] < (min_clear or MIN_CLEAR)))
        if bad or k in (1, n) or k % 5 == 0:
            print(f"  {label} {k}/{n} -> {np.round(tgt, 3)}  IK err {err * 100:.2f}cm  jump {jump:.1f}deg  "
                  f"J4 {np.degrees(nq[3]):.1f}  J6 {np.degrees(nq[5]):.1f}  clear {worst[0] * 100:.1f}cm ({worst[1]})"
                  f"{'' if continuous_ok(q, nq) else '  WRAP'}")
        if bad:
            print(f"  {label}: step {k} fails a gate.")
            return None
        plan.append(nq)
        q = nq
    print(f"  {label}: {n} steps, orientation change {ang:.0f} deg, worst clearance {worst[0] * 100:.1f} cm")
    return plan


ELBOW_LINK, WRIST_LINK = 4, 6     # forearm_link (at J4, the elbow), spherical_wrist_2_link (at J6, the wrist)


def wrist_offset(scene, rb, q):
    """How far (m, sideways = base y) the wrist is to the RIGHT of the elbow at joints q (negative: to its
    left). Right/left as seen from behind the arm looking out along +x, i.e. at the microwave."""
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=rb.physics_client_id)
    y = lambda l: p.getLinkState(rb.robot_id, l, physicsClientId=rb.physics_client_id)[4][1]
    return float(y(ELBOW_LINK) - y(WRIST_LINK))


SIDE_MARGIN = 0.02                # plan_side_path: the wrist at least this far on its side of the elbow
SIDE_RANDOM_SEEDS = 60            # IK seeds per step: this many random, plus each kept solution of the step before
SIDE_KEEP = 40                    # solutions kept per step (lowest cost, >= 3 deg apart). 30/25 missed the 10-07
                                  #   close's narrow passage (hand at y ~0, moving left); 60/40 finds it, ~30 s
SIDE_RNG_SEED = 0


def _steps(p0, q0, p1, q1):
    """The plan_cartesian steps of one straight leg: [(pos, quat)], ~2 cm / <= 5 deg apart."""
    from scipy.spatial.transform import Slerp
    rots = R.from_quat([q0, q1])
    ang = float(np.degrees((rots[0].inv() * rots[1]).magnitude()))
    n = max(1, int(np.ceil(max(np.linalg.norm(np.asarray(p1) - p0) / STEP_M, ang / 5.0))))
    sl = Slerp([0, 1], rots)
    return [(np.asarray(p0) + (np.asarray(p1) - p0) * k / n, sl([k / n]).as_quat()[0]) for k in range(1, n + 1)]


def reachable(scene, rb, pos, quat, side=None, seeds=SIDE_RANDOM_SEEDS):
    """Some IK solution reaches (pos, quat) within plan_side_path's gates (IK, J4/J6 guards, and with `side`
    the wrist >= SIDE_MARGIN on that side of the elbow)? Random seeds, no clearance -- a quick pre-check."""
    sign = 1.0 if side == "right" else -1.0
    rng = np.random.default_rng(SIDE_RNG_SEED)
    for _ in range(seeds):
        q, e = solve_ik(scene, rb, pos, quat, rng.uniform(-np.pi, np.pi, 7))
        for _ in range(3):
            if e <= 0.002:
                break
            q, e = solve_ik(scene, rb, pos, quat, q)
        qd = np.degrees(q)
        if e > 0.005 or abs(qd[3]) > J4_GUARD_DEG or abs(qd[5]) > J6_GUARD_DEG:
            continue
        if side and sign * wrist_offset(scene, rb, q) < SIDE_MARGIN:
            continue
        return True
    return False


def trim_to_reach(scene, rb, p0, q0, p1, q1, side=None, max_trim=0.08, step=0.01):
    """End of the straight leg p0 -> p1 pulled back toward p0 until every step of the leg is `reachable`
    (orientation as plan_side_path would slerp it). Returns (new p1, trimmed metres), or (None, None) if
    more than `max_trim` would be needed."""
    d = np.asarray(p1, float) - p0
    length = float(np.linalg.norm(d))
    trim = 0.0
    while trim <= max_trim + 1e-9:
        end = np.asarray(p0, float) + d * (1.0 - trim / length)
        bad = next((pp for pp, qq in _steps(p0, q0, end, q1) if not reachable(scene, rb, pp, qq, side)), None)
        if bad is None:
            return end, trim
        # jump straight past the first unreachable step (steps are ~STEP_M apart)
        trim = max(trim + step, length - float(np.linalg.norm(bad - p0)) + step)
    return None, None


def plan_side_path(scene, rb, pts, q_start, side, bodies_per_leg, clear_per_leg, label="leg"):
    """Joint path through the straight legs pts[0] -> pts[1] -> ... ([(pos, quat)]; pts[0] = where the hand
    is, at joints q_start) with the wrist >= SIDE_MARGIN on `side` ("right"/"left") of the elbow from the
    end of leg 1 on (leg 1 may change sides). Plain chained IK takes the nearest solution each step and
    can walk into a configuration that dead-ends later, so this searches: every step gets sampled IK
    solutions that pass the gates (IK, J4/J6, side, clearance to that leg's bodies); consecutive steps
    connect when no joint moves > MAX_STEP_JUMP_DEG (no wrap); the path with the least joint motion wins,
    and is re-checked like plan_cartesian (clearance along each step). Returns [joints per leg] or None."""
    sign = 1.0 if side == "right" else -1.0
    rng = np.random.default_rng(SIDE_RNG_SEED)
    steps = []                                         # (pos, quat, leg index)
    for i in range(len(pts) - 1):
        steps += [(pp, qq, i) for pp, qq in _steps(pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1])]
    layer = [(np.asarray(q_start, float), 0.0, None)]   # (joints, cost, back-pointer into the previous layer)
    layers = [layer]
    for k, (pos, quat, leg) in enumerate(steps):
        prev = layers[-1]
        seeds = [rng.uniform(-np.pi, np.pi, 7) for _ in range(SIDE_RANDOM_SEEDS)]
        seeds += [q for q, _, _ in prev] + [q + rng.normal(0.0, 0.15, 7) for q, _, _ in prev]
        cands = []
        for s0 in seeds:
            q, e = solve_ik(scene, rb, pos, quat, s0)
            for _ in range(3):
                if e <= 0.002:
                    break
                q, e = solve_ik(scene, rb, pos, quat, q)
            qd = np.degrees(q)
            if (e > 0.005 or abs(qd[3]) > J4_GUARD_DEG or abs(qd[5]) > J6_GUARD_DEG):
                continue
            last_of_leg1 = leg == 0 and (k + 1 == len(steps) or steps[k + 1][2] != 0)
            if (leg > 0 or last_of_leg1) and sign * wrist_offset(scene, rb, q) < SIDE_MARGIN:
                continue
            if bodies_per_leg[leg] is not None and clearance(rb, q, bodies_per_leg[leg])[0] < clear_per_leg[leg]:
                continue
            # cheapest way here from the previous step
            best = None
            for j, (pq, pc, _) in enumerate(prev):
                d = short_delta(pq, q)
                if np.degrees(np.max(np.abs(d))) <= MAX_STEP_JUMP_DEG and continuous_ok(pq, q):
                    c = pc + float(np.linalg.norm(d))
                    if best is None or c < best[0]:
                        best = (c, j)
            if best is not None:
                cands.append((q, best[0], best[1]))
        cands.sort(key=lambda t: t[1])
        kept = []
        for c in cands:
            if all(np.degrees(np.max(np.abs(short_delta(o[0], c[0])))) > 3.0 for o in kept):
                kept.append(c)
            if len(kept) >= SIDE_KEEP:
                break
        if not kept:
            print(f"  {label} {leg + 1}: no joints with the wrist {side} of the elbow reach step {k + 1} "
                  f"({np.round(pos, 3)}) from the step before -- refusing.")
            return None
        layers.append(kept)
    # back-track the cheapest, then re-check it the way plan_cartesian does
    path, j = [], 0
    for li in range(len(layers) - 1, 0, -1):
        q, _, j_prev = layers[li][j]
        path.append(q)
        j = j_prev
    path.reverse()
    legs, q_prev, worst = [[] for _ in range(len(pts) - 1)], np.asarray(q_start, float), {}
    for (pos, quat, leg), q in zip(steps, path):
        if bodies_per_leg[leg] is not None:
            w = min([clearance(rb, q_prev + short_delta(q_prev, q) * s_ / 3, bodies_per_leg[leg]) for s_ in (1, 2, 3)],
                    key=lambda t: t[0])
            if w[0] < clear_per_leg[leg]:
                print(f"  {label} {leg + 1}: clearance {w[0] * 100:.1f} cm between steps ({w[1]}) -- refusing.")
                return None
            worst[leg] = min(worst.get(leg, w), w, key=lambda t: t[0])
        legs[leg].append(q)
        q_prev = q
    for i, leg in enumerate(legs):
        off = [sign * wrist_offset(scene, rb, q) for q in leg]
        jumps = [np.degrees(np.max(np.abs(short_delta(a, b)))) for a, b in zip(([q_start] + path)[:-1], [q_start] + path)][1:]
        print(f"  {label} {i + 1}: {len(leg)} steps, wrist {min(off) * 100:+.0f}..{max(off) * 100:+.0f} cm {side} of "
              f"the elbow, worst clearance {worst.get(i, (9.0, ''))[0] * 100:.1f} cm, J4 "
              f"{np.degrees(min(q[3] for q in leg)):.0f}..{np.degrees(max(q[3] for q in leg)):.0f}")
    print(f"  largest joint step {max(np.degrees(np.max(np.abs(short_delta(a, b)))) for a, b in zip([q_start] + path[:-1], path)):.1f} deg")
    return legs


def check_curobo_goal(scene, rb, label, q=None, pos=None, quat=None, seed=None, bodies=None):
    """Gate the GOAL of a cuRobo move (SheppyArm.go_to_*): cuRobo plans the path in the driver, which can't be
    previewed, so only where it ends is checked here -- joints `q`, or (pos, quat) IK'd from `seed`: IK, reach,
    J4/J6 guards, the keep-out zone (fridge) and, with `bodies`, >= MIN_CLEAR from the door model. For a pose goal
    cuRobo may end on another joint branch than this IK; the checked branch is the one nearest the seed.
    Returns the checked goal joints, or None (refused)."""
    if q is None:
        if np.linalg.norm(pos) > MAX_REACH_M:
            print(f"  {label}: goal {np.round(pos, 3)} is {np.linalg.norm(pos):.3f} m out (max {MAX_REACH_M}) -- refusing.")
            return None
        q, err = solve_ik(scene, rb, pos, quat, seed)
        for _ in range(3):
            if err <= MAX_IK_ERR:
                break
            q, err = solve_ik(scene, rb, pos, quat, q)
        if err > MAX_IK_ERR:
            print(f"  {label}: goal {np.round(pos, 3)} unreachable (IK err {err * 100:.1f} cm) -- refusing.")
            return None
    q = np.asarray(q, float)
    j4, j6 = abs(np.degrees(q[3])), abs(np.degrees(q[5]))
    if j4 > J4_GUARD_DEG or j6 > J6_GUARD_DEG:
        print(f"  {label}: goal joints J4 {j4:.1f} / J6 {j6:.1f} deg past the guards -- refusing.")
        return None
    ko = keep_out_message(rb, q)
    if ko is not None:
        print(f"  {label}: goal is in the keep-out zone ({ko}) -- refusing.")
        return None
    if bodies is not None:
        c = clearance(rb, q, bodies)
        if c[0] < MIN_CLEAR:
            print(f"  {label}: goal {c[0] * 100:.1f} cm from the door model ({c[1]}) -- refusing.")
            return None
        print(f"  {label}: goal ok, {c[0] * 100:.1f} cm from the door model; cuRobo plans the path (not previewed)")
    else:
        print(f"  {label}: goal ok; cuRobo plans the path (not previewed)")
    return q


MAX_REACH_M = 0.90


def execute_joint_plan(ai, plan, label):
    """One JointCommand per step, waiting for convergence to the COMMANDED joints. Kortex
    sometimes rejects a command (ROBOT_MOVEMENT_IN_PROGRESS) and the arm just stays put --
    re-send that step once before giving up (kortex-dropped-substep-bug).
    With `SheppyArm` (sheppy) the whole plan is ONE smooth trajectory through every step instead:
    one JointCommand per ~2 cm step became a stop-and-go per step there (loud, 10-07)."""
    if hasattr(ai, "move_joints"):
        ok = ai.move_joints(list(plan))
        if not (ok and wait_for_joints(ai, plan[-1])):
            print(f"  {label}: not at the last step's joints -- stopping here.")
            return False
        return True
    for k, q in enumerate(plan, 1):
        for attempt in (1, 2):
            ai.execute_command(JointCommand(pos=q.tolist()))
            if wait_for_joints(ai, q):
                break
            if attempt == 1:
                print(f"  {label} step {k}: not at the commanded joints after 6 s -- re-sending once.")
                time.sleep(0.5)
        else:
            print(f"  {label} step {k}: still not there after a re-send -- stopping here.")
            return False
    return True


def load_park():
    if not PARK_FILE.exists():
        return None
    return wrap_joints(json.loads(PARK_FILE.read_text())["joints"])


def release_and_back_off(ai, back_off_m, execute, save_path=LAST_GRASP_FILE, park=True, pre_park_x=None):
    """Record the current grasp, open the gripper, back straight off along -approach, then
    (park=True) cuRobo (go_to_joint_config) to the hand-placed park joints -- the end
    position of both the open and the close task. `pre_park_x`: before the park move, go
    straight along world x (toward the base) to this x, same height and orientation -- on
    the open task this clears the open door's free edge, which sticks out toward the arm.
    The straight legs are planned and door-checked, and the park goal checked, before
    anything moves; a failed check means nothing is commanded. cuRobo's path to park is
    not previewed and its world has no door in it -- the straight legs get the hand out
    of the door's way first."""
    st = ai.get_state()
    ee = list(st["ee_pos"])
    grasp_pos, quat = np.array(ee[:3]), tuple(ee[3:7])
    approach = R.from_quat(quat).as_matrix()[:, 2]
    q0 = np.array(st["position"], dtype=float)
    print(f"current EE {np.round(grasp_pos, 4)}  gripper {float(st.get('gripper_pos')):.4f}")
    scene, rb = make_sim()
    plan = plan_straight_line(scene, rb, grasp_pos, quat, -approach, back_off_m, q0, "back-off")
    if plan is None:
        print("back-off plan fails a gate -- NOT releasing, arm untouched.")
        return False
    q_park = load_park() if park else None
    legs = [("back-off", plan)]
    if park:
        if q_park is None or not DOOR_FILE.exists():
            print(f"Need {PARK_FILE} and {DOOR_FILE} to plan the move to park -- NOT releasing.")
            return False
        bodies, info = add_door_model(scene, rb, json.loads(DOOR_FILE.read_text()), grasp_pos)
        print(f"door model: open {abs(info['open_deg']):.1f} deg, free edge at {np.round(info['free_edge'], 3)}")
        worst = min((clearance(rb, q, bodies) for q in plan), key=lambda w: w[0])
        print(f"  back-off worst clearance {worst[0] * 100:.1f} cm ({worst[1]})")
        q_end = plan[-1]
        if pre_park_x is not None:
            start = grasp_pos - approach * back_off_m
            dx = pre_park_x - start[0]
            if dx < 0:
                leg = plan_straight_line(scene, rb, start, quat, np.array([-1.0, 0, 0]), -dx, q_end, "toward base")
                if leg is None:
                    print("toward-base leg fails a gate -- NOT releasing, arm untouched.")
                    return False
                worst = min((clearance(rb, q, bodies) for q in leg), key=lambda w: w[0])
                print(f"  toward-base leg worst clearance {worst[0] * 100:.1f} cm ({worst[1]})")
                if worst[0] < MIN_CLEAR:
                    print("toward-base leg too close to the door -- NOT releasing, arm untouched.")
                    return False
                legs.append(("toward base", leg))
        if check_curobo_goal(scene, rb, "to park", q=q_park, bodies=bodies) is None:
            print("park goal fails a check -- NOT releasing, arm untouched.")
            return False
    if not execute:
        print(f"\nDRY RUN (release + back-off{' + park' if park else ''}) -- nothing commanded.")
        return True

    back_off_pos = grasp_pos - approach * back_off_m
    Path(save_path).write_text(json.dumps({
        "time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "grasp_pos": grasp_pos.tolist(), "quat": list(quat), "grasp_joints": q0.tolist(),
        "back_off_pos": back_off_pos.tolist(), "back_off_joints": plan[-1].tolist(),
        "back_off_m": back_off_m}, indent=1))
    print(f"grasp recorded to {save_path} (for a later --phase regrasp)")

    print("Releasing ...")
    ai.execute_command(OpenGripperCommand())
    time.sleep(1.0)
    ok = True
    for label, leg in legs:
        ok = ok and execute_joint_plan(ai, leg, label)
    if ok and park:
        ok = ai.go_to_joint_config(q_park, "to park")
        print(f"  park: {'reached' if ok else 'did NOT reach the park joints'}")
    final = ai.get_state()
    print(f"\nRELEASE {'DONE' if ok else 'STOPPED EARLY'}. final EE:", np.round(final["ee_pos"][:3], 4),
          "gripper:", final.get("gripper_pos"))
    return ok


# ---------------------------------------------------------------------------
# Door model (for collision-checking moves near the OPEN door)
# ---------------------------------------------------------------------------
DOOR_FILE = Path.home() / ".microwave_door.json"
# tool_frame -> fingertip distance along the approach axis, measured twice on 09-23
# (6.3 / 6.1 cm); the closed door face sits just past the fingertips at grasp time.
FINGERTIP_PAST_TOOL = 0.062
DOOR_T = 0.04                     # door thickness
DOOR_PAST_HANDLE = 0.06           # free edge beyond the handle. Was 0.122 (detector span 47.2 - 35.0 cm), but that
                                  # span includes the control panel: on 09-25 the hand passed the 90-deg door's edge
                                  # at x ~0.27, which only fits a door ending <= ~6 cm past the handle.
DOOR_Z = (0.25, 0.80)             # unmeasured -> deliberately generous (fallback: no closed_handle in the door file)
DOOR_Z_HANDLE_Z = 0.537           # closed-handle z when DOOR_Z was set (09-23); the model keeps DOOR_Z's extent
                                  # relative to the handle, so a lower/higher microwave (09-28: handle ~0.26) is covered
DOOR_Z_MARGIN = 0.02              # padding on the detected door_z range (door file), when the grasp saved one
BODY_DEPTH = 0.40
MIN_CLEAR = 0.03                  # every arm/gripper link must stay >= 3 cm from door/body
FINGER_JOINTS = [12, 14, 16, 17, 19, 21]


def door_dims(door):
    """(face behind the closed grasp, door thickness, free edge past the grasp, body depth) for the door model. A door
    file may carry its own (the fridge's, written by its open); the microwave's doesn't -> the constants above."""
    return (float(door.get("face_behind_grasp", FINGERTIP_PAST_TOOL + 0.005)), float(door.get("door_t", DOOR_T)),
            float(door.get("door_past_handle", DOOR_PAST_HANDLE)), float(door.get("body_depth", BODY_DEPTH)))


def save_door_geometry(door_file=None, **kw):
    """Merge door facts into `door_file` (default DOOR_FILE): closed_normal / closed_grasp_pos (grasp), hinge (swing)."""
    door_file = door_file or DOOR_FILE
    d = json.loads(door_file.read_text()) if door_file.exists() else {}
    d.update({k: (np.asarray(v).tolist() if not isinstance(v, str) else v) for k, v in kw.items()})
    d["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    door_file.write_text(json.dumps(d, indent=1))


def _yaw_about(center, xy, ang):
    d = np.asarray(xy) - center[:2]
    c, s_ = np.cos(ang), np.sin(ang)
    return center[:2] + np.array([c * d[0] - s_ * d[1], s_ * d[0] + c * d[1]])


def add_door_model(scene, rb, door, open_grasp_pos):
    """Add the door slab (rotated to its current opening) and the fixed microwave body to
    the sim as collision-only boxes. Returns (bodies, info) -- bodies is [(id, name)]."""
    c = rb.physics_client_id
    n = np.asarray(door["closed_normal"], float); n[2] = 0; n /= np.linalg.norm(n)    # out of the door
    g0, hinge = np.asarray(door["closed_grasp_pos"], float), np.asarray(door["hinge"], float)
    t = np.cross([0, 0, 1.0], n)                                                     # along the door
    if np.dot(t, hinge - g0) < 0:
        t = -t                                                                       # t points to the hinge
    face_off, door_t, past_handle, body_depth = door_dims(door)
    face = g0[:2] - n[:2] * face_off                                                 # a point on the closed face
    lo = float(np.dot(g0[:2] - face, t[:2])) - past_handle                           # free edge (along t)
    hi = float(np.dot(hinge[:2] - face, t[:2]))                                      # hinge edge
    ang = (np.arctan2(*(np.asarray(open_grasp_pos[:2]) - hinge[:2])[::-1])
           - np.arctan2(*(g0[:2] - hinge[:2])[::-1]))
    ang = (ang + np.pi) % (2 * np.pi) - np.pi
    if door.get("door_z"):
        # the detector's door-face height range (1st/99th pct of the plane fit), saved by the grasp
        z_lo, z_hi = door["door_z"][0] - DOOR_Z_MARGIN, door["door_z"][1] + DOOR_Z_MARGIN
    else:
        dz = float(door["closed_handle"][2]) - DOOR_Z_HANDLE_Z if "closed_handle" in door else 0.0
        z_lo, z_hi = DOOR_Z[0] + dz, DOOR_Z[1] + dz
    zc, zh = (z_lo + z_hi) / 2, (z_hi - z_lo) / 2
    base = np.asarray(scene.robot_base_pose.position)                                # identity rotation
    yaw_closed = np.arctan2(n[1], n[0])

    def box(center_xy, half, yaw):
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=c)
        return p.createMultiBody(0, cs, basePosition=(base + [center_xy[0], center_xy[1], zc]).tolist(),
                                 baseOrientation=R.from_euler("z", yaw).as_quat().tolist(), physicsClientId=c)

    mid = (lo + hi) / 2
    door_c_closed = face + t[:2] * mid - n[:2] * door_t / 2
    door_id = box(_yaw_about(hinge, door_c_closed, ang), [door_t / 2, (hi - lo) / 2, zh], yaw_closed + ang)
    body_c = face + t[:2] * mid - n[:2] * (door_t + body_depth / 2)
    body_id = box(body_c, [body_depth / 2, (hi - lo) / 2, zh], yaw_closed)
    free_edge = _yaw_about(hinge, face + t[:2] * lo, ang)
    return [(door_id, "door"), (body_id, "microwave body")], {
        "open_deg": float(np.degrees(ang)), "free_edge": free_edge}


# Robotiq 2F-85 linkage in the sim URDF: one closing angle, mirrored on the inner-finger joints.
FINGER_CLOSE_SIGN = {12: 1.0, 14: -1.0, 16: 1.0, 17: 1.0, 19: -1.0, 21: 1.0}
FINGER_CLOSED_RAD = 0.7           # a little short of the 0.8 full close -> slightly wide, i.e. conservative


def clearance(rb, q, bodies, closed=False):
    """Smallest distance from any robot link to the door/body at joints q, with the gripper
    OPEN (default, the widest hand) or `closed`."""
    c = rb.physics_client_id
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=c)
    for jj in FINGER_JOINTS:
        p.resetJointState(rb.robot_id, jj, FINGER_CLOSE_SIGN[jj] * FINGER_CLOSED_RAD if closed else 0.0,
                          physicsClientId=c)
    best = (9.0, "")
    for bid, name in bodies:
        for pt in p.getClosestPoints(rb.robot_id, bid, 0.5, physicsClientId=c):
            if pt[8] < best[0]:
                ln = p.getJointInfo(rb.robot_id, pt[3], physicsClientId=c)[12].decode() if pt[3] >= 0 else "base"
                best = (float(pt[8]), f"{ln} vs {name}")
    return best


def check_joint_path(rb, q_from, q_to, bodies, label):
    """Joint-linear path q_from -> q_to, sampled every <= 1 deg; returns worst clearance."""
    q_from = wrap_joints(q_from)
    q_to = q_from + short_delta(q_from, q_to)          # the path Kortex takes (short way round)
    n = max(2, int(np.ceil(np.degrees(np.max(np.abs(q_to - q_from))))))
    worst = min((clearance(rb, q_from + (q_to - q_from) * k / n, bodies) for k in range(n + 1)),
                key=lambda w: w[0])
    wrap_ok = continuous_ok(q_from, q_to)
    print(f"  {label}: {np.degrees(np.max(np.abs(q_to - q_from))):.1f} deg joint move, "
          f"worst clearance {worst[0] * 100:.1f} cm ({worst[1]})"
          f"{'' if wrap_ok else '  -- a free-spinning joint would change > 170 deg (ambiguous direction)'}")
    return worst[0] >= MIN_CLEAR and wrap_ok, worst


# ---------------------------------------------------------------------------
# Re-grasp the open door's handle (close task), from wherever the arm is now
# ---------------------------------------------------------------------------
def regrasp(ai, execute, save_path=LAST_GRASP_FILE):
    """Get back onto the handle recorded by the last release, from ANY arm pose (other
    tasks run between opening and closing):
      1. cuRobo (go_to_joint_config) to the recorded back-off joints -- the goal is checked
         against the door model; cuRobo's path is not (its world has no door in it);
      2. straight Cartesian line from the back-off point onto the recorded grasp (the
         exact reverse of the back-off the arm already executed);
      3. close the gripper.
    Assumes the DOOR has not moved since the release (nothing here can re-check that)."""
    if not Path(save_path).exists() or not DOOR_FILE.exists():
        print(f"Need both {save_path} (from a release) and {DOOR_FILE} (from grasp/swing) -- refusing.")
        return False
    rec, door = json.loads(Path(save_path).read_text()), json.loads(DOOR_FILE.read_text())
    if "back_off_joints" not in rec:
        print("Recorded release has no back-off joints -- refusing.")
        return False
    st = ai.get_state()
    g = float(st.get("gripper_pos"))
    q_cur = np.array(st["position"], dtype=float)
    cur = np.array(st["ee_pos"][:3])
    grasp_pos, quat = np.array(rec["grasp_pos"]), tuple(rec["quat"])
    back_off_pos, q_bo = np.array(rec["back_off_pos"]), np.array(rec["back_off_joints"], float)
    print(f"recorded release ({rec['time']}): grasp {np.round(grasp_pos, 4)}, back-off {np.round(back_off_pos, 4)}")
    print(f"current EE {np.round(cur, 4)}  gripper {g:.4f}")
    if g > 0.2:
        print("Gripper is not open -- refusing to re-grasp.")
        return False

    scene, rb = make_sim()
    bodies, info = add_door_model(scene, rb, door, grasp_pos)
    print(f"door model: open {abs(info['open_deg']):.1f} deg, free edge at {np.round(info['free_edge'], 3)}, "
          f"hinge {np.round(door['hinge'], 3)}")
    c0 = clearance(rb, q_cur, bodies)
    print(f"  clearance at the current pose: {c0[0] * 100:.1f} cm ({c0[1]})")

    # 1. to the back-off point: cuRobo (go_to_joint_config) -- only its goal can be checked here
    if check_curobo_goal(scene, rb, "to back-off", q=q_bo, bodies=bodies) is None:
        return False

    # 2. straight in -- the reverse of the executed back-off
    dist = float(np.linalg.norm(grasp_pos - back_off_pos))
    line = plan_straight_line(scene, rb, back_off_pos, quat, (grasp_pos - back_off_pos) / dist, dist,
                              q_bo, "approach")
    if line is None:
        return False
    if not execute:
        print(f"\nDRY RUN (re-grasp) -- would go to the back-off point (cuRobo), then {len(line)} straight "
              "steps onto the handle, then close. Nothing commanded.")
        return True

    if not ai.go_to_joint_config(q_bo, "to back-off"):
        print("  to back-off: did not get there -- stopping here.")
        return False
    print(f"  to back-off: EE {np.round(ai.get_state()['ee_pos'][:3], 4)}")
    if not execute_joint_plan(ai, line, "approach"):
        return False
    fin = np.array(ai.get_state()["ee_pos"][:3])
    print(f"at grasp: EE {np.round(fin, 4)}  ({np.linalg.norm(fin - grasp_pos) * 100:.1f} cm from recorded)")
    print("closing gripper ...")
    ai.execute_command(CloseGripperCommand())
    time.sleep(3.5)
    print(f"gripper after close: {float(ai.get_state().get('gripper_pos')):.4f} -- "
          "gripper_pos cannot confirm a grasp on this rig; CHECK VISUALLY before swinging.")
    return True


# ---------------------------------------------------------------------------
# View the OPEN door square-on (close task), so the live detector sees the door face
# ---------------------------------------------------------------------------
DOOR_VIEW_STANDOFF = 0.35         # camera->handle distance the closed-door detections worked at (30-34 cm)
DOOR_VIEW_OPEN_DEG = 70.0         # view the door as if opened this far (the real open task will go past 50)


def open_door_state(door, rec, open_deg=None):
    """Open door: handle position, outward face normal, a grasp orientation square to the
    face, and the opening angle. From the last release, or -- with `open_deg` -- the same
    door rotated about the hinge to that opening angle (the opening SENSE still comes from
    the recorded release, so which way the door swings is not assumed)."""
    hinge = np.asarray(door["hinge"], float)
    g0 = np.asarray(door["closed_grasp_pos"], float)
    g = np.asarray(rec["grasp_pos"], float)
    ang_rec = (np.arctan2(*(g[:2] - hinge[:2])[::-1]) - np.arctan2(*(g0[:2] - hinge[:2])[::-1]))
    ang_rec = (ang_rec + np.pi) % (2 * np.pi) - np.pi
    ang = ang_rec if open_deg is None else np.sign(ang_rec) * np.radians(open_deg)
    quat = (R.from_euler("z", ang - ang_rec) * R.from_quat(rec["quat"])).as_quat()
    g = np.array([*_yaw_about(hinge, g0[:2], ang), g[2]])
    n0 = np.asarray(door["closed_normal"], float); n0[2] = 0; n0 /= np.linalg.norm(n0)
    c, s_ = np.cos(ang), np.sin(ang)
    n = np.array([c * n0[0] - s_ * n0[1], s_ * n0[0] + c * n0[1], 0.0])
    return g, n, tuple(quat), float(np.degrees(ang))


def move_to_door_view(ai, execute, standoff=DOOR_VIEW_STANDOFF, open_deg=DOOR_VIEW_OPEN_DEG):
    """Move from wherever the arm is to a pose `standoff` out along the open door's face
    normal from its handle, gripper (and wrist camera) facing the door square-on --
    the same kind of view the closed-door detection works from. cuRobo (go_to_ee_pose):
    the goal is checked against the door model, cuRobo's path is not."""
    if not LAST_GRASP_FILE.exists() or not DOOR_FILE.exists():
        print(f"Need {LAST_GRASP_FILE} (from the open task's release) and {DOOR_FILE} -- refusing.")
        return False
    rec, door = json.loads(LAST_GRASP_FILE.read_text()), json.loads(DOOR_FILE.read_text())
    handle, n, quat, open_deg = open_door_state(door, rec, open_deg)
    view = handle + n * standoff
    st = ai.get_state()
    cur = np.array(st["ee_pos"][:3])
    print(f"open door: {abs(open_deg):.1f} deg, handle ~{np.round(handle, 3)}, outward normal {np.round(n, 3)}")
    print(f"view pose {np.round(view, 3)} ({standoff * 100:.0f} cm out, facing the door); current EE {np.round(cur, 3)}")
    if float(st.get("gripper_pos")) > 0.2:
        print("Gripper is closed -- refusing (expected empty-handed at park).")
        return False

    scene, rb = make_sim()
    bodies, info = add_door_model(scene, rb, door, handle)
    c0 = clearance(rb, np.array(st["position"], float), bodies)
    print(f"  clearance now: {c0[0] * 100:.1f} cm ({c0[1]})")
    if check_curobo_goal(scene, rb, "to view", pos=view, quat=quat, seed=np.array(st["position"], float),
                             bodies=bodies) is None:
        print("View pose fails a check -- refusing, arm untouched.")
        return False
    if not execute:
        print("\nDRY RUN (to door view) -- nothing commanded.")
        return True
    ok = ai.go_to_ee_pose(view, quat, "to view")
    print(f"{'AT VIEW POSE' if ok else 'STOPPED EARLY'}: EE {np.round(ai.get_state()['ee_pos'][:3], 4)}")
    return ok


# ---------------------------------------------------------------------------
# Wrist-camera frames for the run log (and the open task's detection frames)
# ---------------------------------------------------------------------------
WRIST_COLOR_TOPIC = "/wrist_camera/color/image_raw"
WRIST_INFO_TOPIC = "/wrist_camera/color/camera_info"
WRIST_DEPTH_TOPIC = "/wrist_camera/aligned_depth_to_color/image_raw"
_camera = None


def grab_frames(n=1, gap_s=0.3):
    """`n` RGB-D frames from the wrist camera, each with K and base_T_cam (tf2 arm_base_link <- camera).
    Reads topics only -- nothing is commanded. The subscriptions are made once and kept."""
    global _camera
    if _camera is None:
        from feeding_deployment.perception.tf_interface import TFInterface
        from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
        rs = RealSenseROS2Interface(color_topic=WRIST_COLOR_TOPIC, camera_info_topic=WRIST_INFO_TOPIC,
                                    depth_topic=WRIST_DEPTH_TOPIC)
        if not rs.wait_for_frames(30.0):
            raise RuntimeError("no RGB-D frames from the wrist camera")
        _camera = (rs, TFInterface())
    rs, tf = _camera
    frames = []
    for i in range(n):
        if i:
            time.sleep(gap_s)
        d = rs.get_camera_data()
        t = tf.get_frame_to_frame_transform(d["camera_info"])
        if t is None:
            raise RuntimeError("no arm_base_link <- camera transform")
        frames.append({"rgb": d["rgb_image"], "depth_mm": d["depth_image"],
                       "K": np.asarray(d["camera_info"].K, dtype=float).reshape(3, 3),
                       "base_T_cam": tf.make_homogeneous_transform(t), "time": time.time()})
    return frames


def save_frame(frame, tag, out_dir, result=None, arm_state=None):
    """<time>_<tag>: _rgb.jpg + .npz (uint16 depth mm, K, base_T_cam, arm joints/tool pose if given) +
    _result.json. ~0.3 MB a frame. Returns the path stem (None if it could not save)."""
    try:
        import cv2
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = out_dir / f"{time.strftime('%Y%m%d_%H%M%S', time.localtime(frame.get('time', time.time())))}_{tag}"
        if frame.get("rgb") is not None:
            cv2.imwrite(str(stem) + "_rgb.jpg", frame["rgb"], [cv2.IMWRITE_JPEG_QUALITY, 90])
        depth = np.nan_to_num(np.asarray(frame["depth_mm"], dtype=np.float32), nan=0.0)
        extra = {}
        if arm_state is not None:
            extra = {"joints": np.asarray(arm_state["position"], float), "ee": np.asarray(arm_state["ee_pos"], float)}
        if frame.get("base_T_cam") is not None:
            extra["base_T_cam"] = frame["base_T_cam"]
        np.savez_compressed(str(stem) + ".npz", depth_mm=np.clip(depth, 0, 65535).astype(np.uint16),
                            K=frame["K"], **extra)
        if result is not None:
            Path(str(stem) + "_result.json").write_text(json.dumps(_jsonable(result), indent=1))
        return stem
    except Exception as e:     # never let debugging output stop a run
        print(f"  (could not save the frame: {e})")
        return None


# ---------------------------------------------------------------------------
# Run log: one directory per door_task run
# ---------------------------------------------------------------------------
RUNS_DIR = Path.home() / "microwave_runs"
STREAM_HZ = 25.0                  # arm state sampling while a run is on
CMD_EVERY = 5                     # keep every 5th commanded trajectory sample (20 ms -> 100 ms)


def _jsonable(x):
    if isinstance(x, dict):
        return {str(k): _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    if isinstance(x, np.ndarray):
        return np.round(x.astype(float), 5).tolist()
    if isinstance(x, (np.floating, np.integer, np.bool_)):
        return x.item()
    if isinstance(x, Path):
        return str(x)
    return x


def _git_info():
    repo = Path(__file__).resolve().parent.parent
    try:
        run = lambda *a: subprocess.run(["git", "-C", str(repo), *a], capture_output=True, text=True, timeout=5).stdout
        return {"commit": run("rev-parse", "--short", "HEAD").strip(), "branch": run("branch", "--show-current").strip(),
                "modified": [l[3:] for l in run("status", "--porcelain", "--untracked-files=no").splitlines()]}
    except Exception as e:
        return {"error": str(e)}


def arm_snapshot(st):
    return {"joints_deg": np.degrees(np.asarray(st["position"], float)), "ee": np.asarray(st["ee_pos"], float),
            "gripper": float(st["gripper_pos"]), "effort": np.asarray(st.get("effort", []), float)}


class RunLog:
    """Everything a door_task run leaves behind, in RUNS_DIR/<time>_<what>/:

    run.json  -- args, git commit, and a timestamped event list: arm state at start/end, the door used
                 for the close (source + values) and its decoded keypoints, every trajectory goal sent
                 (commanded joints every CMD_EVERY samples, ok or not, measured end joints), every gripper
                 command, step outcomes. Rewritten after each event, so a crash keeps what came before.
    arm.npz   -- the arm sampled at STREAM_HZ for the whole run: t, joints, velocity, effort (torques),
                 tool pose, gripper (float32).
    frames    -- camera frames (save_frame: real depth, K, camera pose, arm joints): `frame(tag)`.
    Read-only towards the arm: it only wraps SheppyArm's send/gripper calls to note them."""

    def __init__(self, what, args, ai):
        self.ai, self.t0 = ai, time.time()
        self.dir = RUNS_DIR / f"{time.strftime('%Y%m%d_%H%M%S')}_{what}"
        self.dir.mkdir(parents=True, exist_ok=True)
        self.data = {"what": what, "args": args, "start": time.strftime("%Y-%m-%d %H:%M:%S"), "git": _git_info(),
                     "events": []}
        self._samples, self._stop = [], threading.Event()
        self._thread = threading.Thread(target=self._sample, daemon=True)
        self._thread.start()
        self._wrap(ai)
        self.event("start", arm=arm_snapshot(ai.get_state()))
        print(f"run log: {self.dir}")

    def event(self, kind, **kw):
        self.data["events"].append({"t": round(time.time() - self.t0, 3), "kind": kind, **_jsonable(kw)})
        try:
            (self.dir / "run.json").write_text(json.dumps(self.data, indent=1))
        except Exception as e:
            print(f"  (run log: could not write run.json: {e})")

    def frame(self, tag):
        """One RGB-D frame from the wrist camera, saved here with the arm's state (camera read only)."""
        try:
            f = grab_frames(1)[0]
            stem = save_frame(f, tag, self.dir, arm_state=self.ai.get_state())
            self.event("frame", tag=tag, path=stem)
        except Exception as e:      # a missing camera must not stop the run
            print(f"  (run log: no '{tag}' frame: {e})")
            self.event("frame_failed", tag=tag, error=str(e))

    def _sample(self):
        while not self._stop.is_set():
            try:
                st = self.ai.get_state()
                self._samples.append((time.time() - self.t0, st["position"], st.get("velocity", np.zeros(7)),
                                      st.get("effort", np.zeros(7)), st["ee_pos"], st["gripper_pos"]))
            except Exception:
                pass
            time.sleep(1.0 / STREAM_HZ)

    def _wrap(self, ai):
        """Note every trajectory goal and gripper command SheppyArm sends (the calls themselves unchanged)."""
        if hasattr(ai, "_send"):
            send = ai._send

            def logged_send(t, q, qd, impedance=None):
                start = round(time.time() - self.t0, 3)
                ok = send(t, q, qd, impedance)
                q = np.asarray(q)
                self.event("trajectory", start=start, ok=ok, planned_s=float(t[-1]), n_points=len(t), impedance=impedance,
                           q_cmd_deg=np.degrees(np.vstack([q[::CMD_EVERY], q[-1:]])),
                           end=arm_snapshot(ai.get_state()))
                return ok
            ai._send = logged_send
        if hasattr(ai, "gripper"):
            grip = ai.gripper

            def logged_gripper(position, timeout_s=4.0, force=None):
                r = grip(position, timeout_s=timeout_s, force=force)
                self.event("gripper", cmd=position, force=force, measured=float(ai.get_state()["gripper_pos"]))
                return r
            ai.gripper = logged_gripper

    def close(self, ok):
        self._stop.set()
        self._thread.join(timeout=1.0)
        try:
            self.event("end", ok=bool(ok), arm=arm_snapshot(self.ai.get_state()))
        except Exception as e:
            self.event("end", ok=bool(ok), error=str(e))
        if self._samples:
            t, q, dq, tau, ee, g = zip(*self._samples)
            np.savez_compressed(self.dir / "arm.npz", t=np.asarray(t, np.float32), joints=np.asarray(q, np.float32),
                                velocity=np.asarray(dq, np.float32), effort=np.asarray(tau, np.float32),
                                ee=np.asarray(ee, np.float32), gripper=np.asarray(g, np.float32))
        print(f"run log: {self.dir} ({len(self._samples)} arm samples)")

