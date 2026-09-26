"""Shared motion pieces for the microwave open/close scripts on `rchi-cpu-5`.

Used by `real_gen3_ros2_grasp_and_swing_microwave.py` (release after opening) and
`real_gen3_ros2_close_microwave.py` (re-grasp before closing, release after the push).

* `release_and_back_off` -- records the current grasp (pose + joints) to
  `LAST_GRASP_FILE`, opens the gripper, then backs straight off the handle along the
  gripper's approach axis in ~2 cm Cartesian steps.
* `regrasp` -- for the close task, from ANY arm pose (other tasks run between opening
  and closing): a door-model-checked joint move to the recorded back-off pose, then
  straight back onto the recorded grasp, then close. No re-detection (the detector's
  plane fit assumes a door facing the camera, which an open door is not). Needs the
  door geometry the grasp/swing save to `DOOR_FILE`.

All gripper motion goes through `execute_command(Open/CloseGripperCommand())` --
`ArmInterfaceClient` has no `open_gripper()` method.
"""
import json, time
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.command_interface import (
    CloseGripperCommand, JointCommand, OpenGripperCommand)
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

ARM = [1, 2, 3, 4, 5, 6, 7]
LAST_GRASP_FILE = Path.home() / ".microwave_last_grasp.json"
STEP_M = 0.02
MAX_IK_ERR = 0.02
MAX_STEP_JUMP_DEG = 20.0
J6_GUARD_DEG = 115.0
# J1/J3/J5/J7 spin freely. Whether Kortex goes the short way across +-180 is unverified,
# so every commanded angle is wrapped to [-180, 180) and a move is refused if any of these
# joints would change by more than 180 deg -- the only case where "straight from old to new
# value" and "the short way round" are different paths. Near-180 values are fine otherwise.
CONTINUOUS = [0, 2, 4, 6]
MAX_CONTINUOUS_DELTA_DEG = 170.0
PARK_FILE = Path(__file__).resolve().parent / "park_pose.json"


def wrap_joints(q):
    q = np.asarray(q, dtype=float).copy()
    q[CONTINUOUS] = (q[CONTINUOUS] + np.pi) % (2 * np.pi) - np.pi
    return q


def continuous_ok(q_from, q_to):
    d = np.degrees(np.abs(wrap_joints(q_to)[CONTINUOUS] - wrap_joints(q_from)[CONTINUOUS]))
    return bool(np.all(d <= MAX_CONTINUOUS_DELTA_DEG))


def make_sim():
    scene = create_scene_description_from_config(
        "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
    return scene, FeedingDeploymentPyBulletSimulator(scene, use_gui=False).robot


def solve_ik(scene, rb, pos, quat, seed_joints):
    """PyBullet IK seeded from `seed_joints`; returns (joints, position error in m)."""
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
    return joints, float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(wpose.position)))


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


def plan_straight_line(scene, rb, start_pos, quat, direction, dist, seed_joints, label):
    """Joint solutions for a straight Cartesian line, ~2 cm per step, fixed orientation.
    Returns the list of joint vectors, or None if any step fails a gate."""
    n = max(1, int(np.ceil(dist / STEP_M)))
    q, plan = np.asarray(seed_joints, dtype=float), []
    for k in range(1, n + 1):
        tgt = np.asarray(start_pos) + np.asarray(direction) * dist * k / n
        nq, err = solve_ik(scene, rb, tgt, quat, q)
        jump = float(np.degrees(np.max(np.abs(nq - q))))
        j6, j7 = float(np.degrees(nq[5])), float(np.degrees(nq[6]))
        print(f"  {label} {k}/{n} -> {np.round(tgt, 3)}  IK err {err * 100:.2f}cm  jump {jump:.1f}deg  "
              f"J6 {j6:.1f}deg  J7 {j7:.1f}deg")
        if (err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or abs(j6) > J6_GUARD_DEG
                or not continuous_ok(q, nq)):
            print(f"  {label}: step {k} fails the IK/jump/J6/wrap gate.")
            return None
        plan.append(nq)
        q = nq
    return plan


BIG_MOVE_TIMEOUT_S = 40.0


def plan_cartesian(scene, rb, p0, q0_quat, p1, q1_quat, seed_joints, label, bodies=None):
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
        jump = float(np.degrees(np.max(np.abs(nq - q))))
        if bodies is not None:
            worst = min([worst] + [clearance(rb, q + (nq - q) * s_ / 3, bodies) for s_ in (1, 2, 3)],
                        key=lambda w: w[0])
        bad = (err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or abs(np.degrees(nq[5])) > J6_GUARD_DEG
               or not continuous_ok(q, nq) or (bodies is not None and worst[0] < MIN_CLEAR))
        if bad or k in (1, n) or k % 5 == 0:
            print(f"  {label} {k}/{n} -> {np.round(tgt, 3)}  IK err {err * 100:.2f}cm  jump {jump:.1f}deg  "
                  f"J6 {np.degrees(nq[5]):.1f}  clear {worst[0] * 100:.1f}cm ({worst[1]})")
        if bad:
            print(f"  {label}: step {k} fails a gate.")
            return None
        plan.append(nq)
        q = nq
    print(f"  {label}: {n} steps, orientation change {ang:.0f} deg, worst clearance {worst[0] * 100:.1f} cm")
    return plan


def execute_joint_plan(ai, plan, label):
    """One JointCommand per step, waiting for convergence to the COMMANDED joints. Kortex
    sometimes rejects a command (ROBOT_MOVEMENT_IN_PROGRESS) and the arm just stays put --
    re-send that step once before giving up (kortex-dropped-substep-bug)."""
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
    (park=True) a door-model-checked joint move to the hand-placed park pose -- the end
    position of both the open and the close task. `pre_park_x`: before the park move, go
    straight along world x (toward the base) to this x, same height and orientation -- on
    the open task this clears the open door's free edge, which sticks out toward the arm.
    Everything is planned and checked before anything moves; a failed check means nothing
    is commanded."""
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
                q_end = leg[-1]
        park_rec = json.loads(PARK_FILE.read_text())
        p_start = grasp_pos - approach * back_off_m
        if pre_park_x is not None and pre_park_x < p_start[0]:
            p_start = np.array([pre_park_x, p_start[1], p_start[2]])
        leg = plan_cartesian(scene, rb, p_start, quat, park_rec["ee_pos"], park_rec["quat"], q_end,
                             "to park", bodies)
        if leg is None:
            print("path to park fails a gate -- NOT releasing, arm untouched.")
            return False
        snap = float(np.degrees(np.max(np.abs(wrap_joints(leg[-1]) - q_park))))
        print(f"  stepped path ends {snap:.1f} deg (max joint) from the recorded park joints")
        legs.append(("to park", leg))
        if snap > 10.0:
            print("  -> will stop at the park POSE on this joint branch instead of snapping to the recorded joints")
            q_park = None
        else:
            ok, _ = check_joint_path(rb, leg[-1], q_park, bodies, "snap to recorded park joints")
            if not ok:
                q_park = None
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
    if ok and park and q_park is not None:
        ai.execute_command(JointCommand(pos=q_park.tolist()))
        ok = wait_for_joints(ai, q_park, timeout_s=BIG_MOVE_TIMEOUT_S)
        print(f"  park: {'reached' if ok else 'did NOT reach the commanded joints'}")
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
DOOR_Z = (0.25, 0.80)             # unmeasured -> deliberately generous
BODY_DEPTH = 0.40
MIN_CLEAR = 0.03                  # every arm/gripper link must stay >= 3 cm from door/body
FINGER_JOINTS = [12, 14, 16, 17, 19, 21]


def save_door_geometry(**kw):
    """Merge door facts into DOOR_FILE: closed_normal / closed_grasp_pos (grasp), hinge (swing)."""
    d = json.loads(DOOR_FILE.read_text()) if DOOR_FILE.exists() else {}
    d.update({k: (np.asarray(v).tolist() if not isinstance(v, str) else v) for k, v in kw.items()})
    d["updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    DOOR_FILE.write_text(json.dumps(d, indent=1))


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
    face = g0[:2] - n[:2] * (FINGERTIP_PAST_TOOL + 0.005)                            # a point on the closed face
    lo = float(np.dot(g0[:2] - face, t[:2])) - DOOR_PAST_HANDLE                      # free edge (along t)
    hi = float(np.dot(hinge[:2] - face, t[:2]))                                      # hinge edge
    ang = (np.arctan2(*(np.asarray(open_grasp_pos[:2]) - hinge[:2])[::-1])
           - np.arctan2(*(g0[:2] - hinge[:2])[::-1]))
    ang = (ang + np.pi) % (2 * np.pi) - np.pi
    zc, zh = (DOOR_Z[0] + DOOR_Z[1]) / 2, (DOOR_Z[1] - DOOR_Z[0]) / 2
    base = np.asarray(scene.robot_base_pose.position)                                # identity rotation
    yaw_closed = np.arctan2(n[1], n[0])

    def box(center_xy, half, yaw):
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=c)
        return p.createMultiBody(0, cs, basePosition=(base + [center_xy[0], center_xy[1], zc]).tolist(),
                                 baseOrientation=R.from_euler("z", yaw).as_quat().tolist(), physicsClientId=c)

    mid = (lo + hi) / 2
    door_c_closed = face + t[:2] * mid - n[:2] * DOOR_T / 2
    door_id = box(_yaw_about(hinge, door_c_closed, ang), [DOOR_T / 2, (hi - lo) / 2, zh], yaw_closed + ang)
    body_c = face + t[:2] * mid - n[:2] * (DOOR_T + BODY_DEPTH / 2)
    body_id = box(body_c, [BODY_DEPTH / 2, (hi - lo) / 2, zh], yaw_closed)
    free_edge = _yaw_about(hinge, face + t[:2] * lo, ang)
    return [(door_id, "door"), (body_id, "microwave body")], {
        "open_deg": float(np.degrees(ang)), "free_edge": free_edge}


def clearance(rb, q, bodies):
    """Smallest distance from any robot link (gripper OPEN) to the door/body at joints q."""
    c = rb.physics_client_id
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=c)
    for jj in FINGER_JOINTS:
        p.resetJointState(rb.robot_id, jj, 0.0, physicsClientId=c)
    best = (9.0, "")
    for bid, name in bodies:
        for pt in p.getClosestPoints(rb.robot_id, bid, 0.5, physicsClientId=c):
            if pt[8] < best[0]:
                ln = p.getJointInfo(rb.robot_id, pt[3], physicsClientId=c)[12].decode() if pt[3] >= 0 else "base"
                best = (float(pt[8]), f"{ln} vs {name}")
    return best


def check_joint_path(rb, q_from, q_to, bodies, label):
    """Joint-linear path q_from -> q_to, sampled every <= 1 deg; returns worst clearance."""
    q_from, q_to = wrap_joints(q_from), wrap_joints(q_to)
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
REGRASP_VIA = (0.10, 0.10)        # fallback via point: +10 cm further out, +10 cm up


def regrasp(ai, execute, save_path=LAST_GRASP_FILE):
    """Get back onto the handle recorded by the last release, from ANY arm pose (other
    tasks run between opening and closing):
      1. joint move to the recorded back-off joints -- the whole joint path is checked
         against the door model; if it clips, via a point further out and higher;
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
    approach = R.from_quat(quat).as_matrix()[:, 2]
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

    # 1. to the back-off point: direct, else via a point further out + up
    moves = []
    ok, _ = check_joint_path(rb, q_cur, q_bo, bodies, "direct -> back-off")
    if ok:
        moves = [("to back-off", q_bo)]
    else:
        via_pos = back_off_pos - approach * REGRASP_VIA[0] + np.array([0, 0, REGRASP_VIA[1]])
        q_via, err = solve_ik(scene, rb, via_pos, quat, q_bo)
        print(f"  trying via {np.round(via_pos, 3)} (IK err {err * 100:.2f}cm)")
        ok1, _ = check_joint_path(rb, q_cur, q_via, bodies, "current -> via")
        ok2, _ = check_joint_path(rb, q_via, q_bo, bodies, "via -> back-off")
        if err > MAX_IK_ERR or not (ok1 and ok2):
            print("No door-clear path to the back-off point found -- refusing. Move the arm somewhere "
                  "clearer (e.g. further back, in front of the door) and retry.")
            return False
        moves = [("to via", q_via), ("to back-off", q_bo)]

    # 2. straight in -- the reverse of the executed back-off
    dist = float(np.linalg.norm(grasp_pos - back_off_pos))
    line = plan_straight_line(scene, rb, back_off_pos, quat, (grasp_pos - back_off_pos) / dist, dist,
                              q_bo, "approach")
    if line is None:
        return False
    if not execute:
        print(f"\nDRY RUN (re-grasp) -- would do {len(moves)} joint move(s), then {len(line)} straight "
              "steps onto the handle, then close. Nothing commanded.")
        return True

    for label, q in moves:
        ai.execute_command(JointCommand(pos=q.tolist()))
        if not wait_for_joints(ai, q, timeout_s=BIG_MOVE_TIMEOUT_S):
            print(f"  {label}: did not reach the commanded joints -- stopping here.")
            return False
        print(f"  {label}: EE {np.round(ai.get_state()['ee_pos'][:3], 4)}")
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
    the same kind of view the closed-door detection works from. Straight-line path with
    the orientation slerped, every step checked against the door model."""
    if not LAST_GRASP_FILE.exists() or not DOOR_FILE.exists():
        print(f"Need {LAST_GRASP_FILE} (from the open task's release) and {DOOR_FILE} -- refusing.")
        return False
    rec, door = json.loads(LAST_GRASP_FILE.read_text()), json.loads(DOOR_FILE.read_text())
    handle, n, quat, open_deg = open_door_state(door, rec, open_deg)
    view = handle + n * standoff
    st = ai.get_state()
    cur, cur_quat = np.array(st["ee_pos"][:3]), tuple(st["ee_pos"][3:7])
    print(f"open door: {abs(open_deg):.1f} deg, handle ~{np.round(handle, 3)}, outward normal {np.round(n, 3)}")
    print(f"view pose {np.round(view, 3)} ({standoff * 100:.0f} cm out, facing the door); current EE {np.round(cur, 3)}")
    if float(st.get("gripper_pos")) > 0.2:
        print("Gripper is closed -- refusing (expected empty-handed at park).")
        return False

    scene, rb = make_sim()
    bodies, info = add_door_model(scene, rb, door, handle)
    c0 = clearance(rb, np.array(st["position"], float), bodies)
    print(f"  clearance now: {c0[0] * 100:.1f} cm ({c0[1]})")
    path = plan_cartesian(scene, rb, cur, cur_quat, view, quat, np.array(st["position"], float), "to view", bodies)
    if path is None:
        print("No door-clear straight path to the view pose -- refusing, arm untouched.")
        return False
    if not execute:
        print("\nDRY RUN (to door view) -- nothing commanded.")
        return True
    ok = execute_joint_plan(ai, path, "to view")
    print(f"{'AT VIEW POSE' if ok else 'STOPPED EARLY'}: EE {np.round(ai.get_state()['ee_pos'][:3], 4)}")
    return ok
