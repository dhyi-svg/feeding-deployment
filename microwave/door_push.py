"""Push the microwave door with the SIDE of the gripper -- no grasp.

* `push_close` (close task, VALIDATED on hardware 2026-09-25: door latched). The hand may be
  holding something else (a container), so the gripper is never touched and the hand's
  orientation never changes. The user's hand demo, as a plan: left (toward the hinge) at the
  current depth to ~8 cm past the hinge, forward to 28 cm from the hinge, swing about the hinge
  (radius spiralling in to 26 cm, clear of the handle) until 3 cm short of the model's closed
  door, then a torque-watched push in 5 mm steps that stops when the door hits its frame
  (`_push_until_shut`), then back out.
* `push_open` (open task, sim-only so far): after the pull (still holding the handle,
  ~50 deg), release, back off, go round the door's free edge to its INNER face, and push it on
  to ~85 deg -- the pull can't get there itself (J6 limit at ~55-80 deg). Here the hand is held
  rigid in the DOOR's frame (tool at distance `u` along the door, approach along the door toward
  the hinge, turning with the door), so a few cm of hinge error only slides the contact along
  the face. How far off the face it sits is bisected in the PyBullet door model
  (`add_door_model`), and the push runs `preload` past contact.

Contact: the Kortex wrench/torque zero moves with pose (kinova-wrench-bias-not-noise), so a
single threshold can't tell contact from drift -- the close's final push only trusts a torque
change that keeps RISING, at small steps where the pose barely changes. The sweeps themselves
are geometric.

Everything is planned and gated (IK, per-step jump, J6, +-180 wrap, clearance to the door
model and microwave body) before anything moves; a failed gate means nothing is commanded.
The door model doesn't include the handle or whatever the gripper is holding.
"""
import json, time

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

from feeding_deployment.control.robot_controller.command_interface import (
    CartesianTrajectoryCommand, JointCommand, OpenGripperCommand)

from microwave_common import (
    BIG_MOVE_TIMEOUT_S, DOOR_FILE, DOOR_PAST_HANDLE, DOOR_T, FINGERTIP_PAST_TOOL, J6_GUARD_DEG, MAX_IK_ERR,
    MAX_STEP_JUMP_DEG, MIN_CLEAR, PARK_FILE, add_door_model, check_joint_path, clearance,
    continuous_ok, execute_joint_plan, load_park, make_sim, plan_cartesian, plan_straight_line,
    save_door_geometry, solve_ik, wait_for_joints, wrap_joints)

ARC_STEP_M = 0.02          # tool travel per sweep waypoint
TIP_INSIDE_EDGE = 0.02     # default u: tool point this far in from the free edge (tips ~8 cm in)
HANDLE_MARGIN = 0.02       # refuse a u whose fingertips come closer than this to the handle line
VIA_PAST_EDGE = 0.15       # route round the free edge with the tool this far beyond it
VIA_OFF_FACE = 0.12        # ... and this far off the face, either side
GAP = 0.04                 # approach pose: closest link this far from the door (>= MIN_CLEAR)
CLOSE_FRONT_MARGIN = 0.06  # push-close: sideways leg passes this far in front of the free edge (past fingertips)
PUSH_STEP_M = 0.005        # push-close: final push step
OPEN_RIGHT_MARGIN = 0.12   # push-open: go this far right of the open door's free edge before going in
OPEN_OFF_DOOR = 0.12       # push-open: after the push, move this far right, off the door, before backing out
OPEN_PUSH_LAG = 5.0        # push-open: door assumed this many deg past the hand after the push (way-out check)
CLOSE_HAND_HALF = 0.04     # push-close: half-width of the hand along the door, for the handle check (the 09-25
                           # hand demo ended 27.6 cm out, handle bar at 33 cm, clear)
V_SEARCH = 0.35            # bisection range for v, either side of the door


class DoorFrame:
    """Door geometry from DOOR_FILE. Door-frame coordinates (u, v) at opening angle `deg`:
    u along the door from the hinge toward the free edge, v along the outward normal
    (toward the arm when closed) measured from the slab's mid-plane."""

    def __init__(self, door, sign):
        self.door, self.sign = door, float(sign)
        self.hinge = np.asarray(door["hinge"], float)
        self.g0 = np.asarray(door["closed_grasp_pos"], float)
        n = np.asarray(door["closed_normal"], float); n[2] = 0
        self.n = n[:2] / np.linalg.norm(n[:2])
        e = self.g0[:2] - self.hinge[:2]
        e -= self.n * np.dot(e, self.n)
        self.e = e / np.linalg.norm(e)                                         # hinge -> free edge
        face = self.g0[:2] - self.n * (FINGERTIP_PAST_TOOL + 0.005)           # as add_door_model
        self.mid = float(np.dot(face - self.hinge[:2], self.n)) - DOOR_T / 2
        self.u_handle = float(np.dot(self.g0[:2] - self.hinge[:2], self.e))
        self.length = self.u_handle + DOOR_PAST_HANDLE

    def _rot(self, deg):
        a = self.sign * np.radians(deg)
        return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])

    def angle_of(self, xy):
        """Opening angle (deg, positive = open) of a point that was on the grasp line when closed."""
        d = np.asarray(xy[:2]) - self.hinge[:2]
        g = self.g0[:2] - self.hinge[:2]
        a = np.arctan2(g[0] * d[1] - g[1] * d[0], g @ d)
        return float(np.degrees(a) * self.sign)

    def pos(self, u, v, deg, z):
        xy = self.hinge[:2] + self._rot(deg) @ (self.e * u + self.n * (self.mid + v))
        return np.array([xy[0], xy[1], z])

    def quat(self, deg, y_up, yaw_deg=0.0, roll_deg=0.0):
        """Approach (tool z) horizontal, along the door toward the hinge; tool y vertical
        (`y_up` = +-1, keep whatever the wrist has now). `yaw_deg` turns the approach off the
        door line, `roll_deg` rolls the hand about it (which face of the gripper meets the door)."""
        z2 = self._rot(deg + yaw_deg * self.sign) @ -self.e
        zc = np.array([z2[0], z2[1], 0.0])
        yc = np.array([0.0, 0.0, y_up])
        m = np.column_stack([np.cross(yc, zc), yc, zc])
        return (R.from_matrix(m) * R.from_euler("z", roll_deg, degrees=True)).as_quat()

    def bodies_point(self, deg):
        """The closed grasp point swung to `deg` -- what add_door_model reads the angle from."""
        return np.array([*(self.hinge[:2] + self._rot(deg) @ (self.g0[:2] - self.hinge[:2])), self.g0[2]])

    def bodies(self, scene, rb, deg):
        """Door slab at `deg` + microwave body, as [(id, name)] -- remove with `drop`."""
        return add_door_model(scene, rb, self.door, self.bodies_point(deg))[0]


def record_door_angle(ee):
    """After a pull (hand still on the handle): save the opening sense and angle to DOOR_FILE,
    which push-close reads."""
    door = json.loads(DOOR_FILE.read_text())
    a = DoorFrame(door, 1.0).angle_of(ee)
    save_door_geometry(door_open_deg=abs(a), open_sign=float(np.sign(a) or 1.0))
    print(f"door file: open ~{abs(a):.1f} deg (sense {int(np.sign(a) or 1):+d})")


def drop(rb, bodies):
    for bid, _ in bodies:
        p.removeBody(bid, physicsClientId=rb.physics_client_id)


def plan_to_park(scene, rb, p_start, quat, q_start, bodies):
    """Door-model-checked straight/slerped leg to the park pose, then (if close) a snap to
    the recorded park joints. Returns (leg, q_park or None) or (None, None)."""
    if not PARK_FILE.exists():
        print(f"no {PARK_FILE} -- can't plan to park")
        return None, None
    rec = json.loads(PARK_FILE.read_text())
    leg = plan_cartesian(scene, rb, p_start, quat, rec["ee_pos"], rec["quat"], q_start, "to park", bodies)
    if leg is None:
        return None, None
    q_park = load_park()
    snap = float(np.degrees(np.max(np.abs(wrap_joints(leg[-1]) - q_park))))
    print(f"  to park ends {snap:.1f} deg (max joint) from the recorded park joints")
    if snap > 10.0:
        print("  -> will stop at the park POSE on this joint branch")
        return leg, None
    ok, _ = check_joint_path(rb, leg[-1], q_park, bodies, "snap to park joints")
    return leg, (q_park if ok else None)


def _chain(scene, rb, points, q, bodies, label):
    """plan_cartesian through a list of (pos, quat) points; returns (legs, q_end) or (None, q)."""
    legs = []
    for i in range(len(points) - 1):
        (p0, q0), (p1, q1) = points[i], points[i + 1]
        leg = plan_cartesian(scene, rb, p0, q0, p1, q1, q, f"{label} {i + 1}", bodies)
        if leg is None:
            return None, q
        legs.append((f"{label} {i + 1}", leg))
        q = leg[-1]
    return legs, q


def _facing_forward(quat, fwd):
    """`quat` turned by the smallest rotation that points its approach (tool z) along `fwd` --
    keeps the hand's roll, e.g. a container held upright stays upright."""
    z = R.from_quat(quat).as_matrix()[:, 2]
    axis = np.cross(z, fwd)
    s, c = np.linalg.norm(axis), float(np.dot(z, fwd))
    turn = R.identity() if s < 1e-9 else R.from_rotvec(axis / s * np.arctan2(s, c))
    return (turn * R.from_quat(quat)).as_quat()


def _plan_arc(scene, rb, pts, quat, q, body, label):
    """Chained IK through `pts` at fixed `quat`; gates IK, jump, J6, wrap, and >= MIN_CLEAR from
    the microwave body (not the door -- this is the leg that pushes it). Returns (plan, poses)."""
    plan, poses = [], []
    for k, pt in enumerate(pts, 1):
        nq, err = solve_ik(scene, rb, pt, quat, q)
        jump = float(np.degrees(np.max(np.abs((nq - q + np.pi) % (2 * np.pi) - np.pi))))
        c_body = clearance(rb, nq, body)
        j6 = float(np.degrees(nq[5]))
        bad = (err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or abs(j6) > J6_GUARD_DEG
               or not continuous_ok(q, nq) or c_body[0] < MIN_CLEAR)
        if bad or k in (1, len(pts)) or k % 4 == 0:
            print(f"  {label} {k}/{len(pts)} -> {np.round(pt, 3)}  IK err {err * 100:.2f}cm  jump {jump:.1f}deg  "
                  f"J6 {j6:.1f}  body {c_body[0] * 100:.1f}cm")
        if bad:
            print(f"  {label}: step {k} fails a gate.")
            return None, None
        plan.append(nq)
        poses.append((pt, quat))
        q = nq
    return plan, poses


def _plan_open(scene, rb, df, st, args, deg0, roll):
    """The user's teleop demo (09-25), in door-frame coordinates about the hinge:
    back straight out -> turn the wrist to face into the microwave -> right, past the open
    door's free edge -> forward into the gap between the door and the microwave front -> left
    to the door's inner face -> arc about the hinge (hand fixed, radius spiralling in) pushing
    the door on -> right, off the door -> back -> park."""
    fwd = np.r_[-df.n, 0.0]                        # into the microwave
    side = np.r_[-df.e, 0.0]                       # along the closed door, toward the hinge
    ee, q0 = np.array(st["ee_pos"], float), np.array(st["position"], float)
    z = ee[2]
    h = np.r_[df.hinge[:2], z]
    a = lambda pt: float(np.dot(pt - h, side))
    b = lambda pt: float(np.dot(pt - h, fwd))
    pos = lambda aa, bb: h + side * aa + fwd * bb
    at = lambda phi, r: pos(-r * np.cos(np.radians(phi)), -r * np.sin(np.radians(phi)))   # phi: deg from closed
    g_quat = tuple(ee[3:7])
    quat = (R.from_quat(_facing_forward(g_quat, fwd)) * R.from_euler("z", roll, degrees=True)).as_quat()

    free = add_door_model(scene, rb, df.door, df.bodies_point(deg0))
    drop(rb, free[0])
    edge = np.r_[free[1]["free_edge"], z]
    p_back = ee[:3] - fwd * args.push_back
    a_right = a(edge) - OPEN_RIGHT_MARGIN
    b_in = args.push_in
    r0 = args.push_radius
    if r0 <= abs(b_in):
        print("--push-radius must exceed |--push-in| -- refusing.")
        return None
    phi0 = float(np.degrees(np.arcsin(-b_in / r0)))
    phi1 = args.push_target_deg
    print(f"door ~{deg0:.0f} deg; free edge {np.round(edge[:2], 3)} (a {a(edge) * 100:+.0f} / b {b(edge) * 100:+.0f} cm)")
    print(f"plan: back {args.push_back * 100:.0f} cm, turn the wrist, right to a {a_right * 100:+.0f} cm, forward to "
          f"b {b_in * 100:+.0f} cm, left to the inner face ({phi0:.0f} deg round the hinge, r {r0 * 100:.0f} cm), push "
          f"to {phi1:.0f} deg (r -> {args.push_radius_end * 100:.0f} cm), right, back, park")
    if phi0 >= deg0 - 5:
        print(f"push start ({phi0:.0f} deg) is not behind the door ({deg0:.0f} deg) -- refusing.")
        return None

    bodies = df.bodies(scene, rb, deg0)
    try:
        pts = [(ee[:3], g_quat), (p_back, g_quat), (p_back, quat),
               (pos(a_right, b(p_back)), quat), (pos(a_right, b_in), quat), (at(phi0, r0), quat)]
        legs, q = _chain(scene, rb, pts, q0, bodies, "approach")
    finally:
        drop(rb, bodies)
    if legs is None:
        return None
    n = max(2, int(np.ceil(np.radians(phi1 - phi0) * r0 / ARC_STEP_M)))
    arc_pts = [at(phi0 + (phi1 - phi0) * k / n, r0 + (args.push_radius_end - r0) * k / n) for k in range(1, n + 1)]
    closed = df.bodies(scene, rb, 0.0)
    try:
        arc, arc_poses = _plan_arc(scene, rb, arc_pts, quat, q, [closed[1]], "push")
    finally:
        drop(rb, closed)
    if arc is None:
        return None
    # the door ends a little past the hand; model it OPEN_PUSH_LAG beyond it for the way out
    deg1 = phi1 + OPEN_PUSH_LAG
    end = arc_pts[-1]
    bodies1 = df.bodies(scene, rb, deg1)
    try:
        out_pts = [(end, quat), (pos(a(end) - OPEN_OFF_DOOR, b(end)), quat),
                   (pos(a(end) - OPEN_OFF_DOOR, b(p_back)), quat)]
        out, q = _chain(scene, rb, out_pts, arc[-1], bodies1, "out")
        if out is None:
            return None
        park, q_park = plan_to_park(scene, rb, out_pts[-1][0], quat, q, bodies1)
    finally:
        drop(rb, bodies1)
    if park is None:
        return None
    return {"before": legs, "sweep": (arc, arc_poses), "after": out + [("to park", park)],
            "q_park": q_park, "deg1": deg1}


def push_open(ai, args):
    """From the end of the pull (holding the handle): release, go round to the door's inner face
    and push it open further, then park. Tries the hand as it is and rolled 180 deg (the gripper
    is symmetric) and keeps the first plan that passes every gate."""
    if not DOOR_FILE.exists():
        print(f"need {DOOR_FILE} (grasp + swing write it) -- refusing.")
        return False
    door = json.loads(DOOR_FILE.read_text())
    st = ai.get_state()
    ee = np.array(st["ee_pos"], float)
    if float(st["gripper_pos"]) < 0.2:
        print("gripper is open -- push-open starts from the end of the pull, still holding the handle. Refusing.")
        return False
    a = DoorFrame(door, 1.0).angle_of(ee)              # opening sense + angle from the held handle
    df = DoorFrame(door, np.sign(a) or 1.0)
    deg0 = abs(a)
    print(f"door open ~{deg0:.1f} deg now (handle vs closed grasp about the hinge)")
    if not 20.0 <= deg0 <= 80.0:
        print("that doesn't look like the end of a pull -- refusing.")
        return False
    scene, rb = make_sim()
    plan = None
    for roll in (0.0, 180.0):
        print(f"\n--- hand roll +{roll:.0f} deg ---")
        plan = _plan_open(scene, rb, df, st, args, deg0, roll)
        if plan:
            break
    if plan is None:
        print("\nno plan passes every gate -- NOT releasing, arm untouched.")
        return False
    if not args.execute:
        print(f"\nDRY RUN (push-open {deg0:.0f} -> ~{plan['deg1']:.0f} deg) -- nothing commanded.")
        return True
    print("Releasing ...")
    ai.execute_command(OpenGripperCommand())
    time.sleep(1.0)
    for label, leg in plan["before"]:
        if not execute_joint_plan(ai, leg, label):
            return False
    if not execute_joint_plan(ai, plan["sweep"][0], "push"):
        return False
    save_door_geometry(door_open_deg=plan["deg1"], open_sign=df.sign)
    for label, leg in plan["after"]:
        if not execute_joint_plan(ai, leg, label):
            return False
    if plan["q_park"] is not None:
        ai.execute_command(JointCommand(pos=plan["q_park"].tolist()))
        wait_for_joints(ai, plan["q_park"], timeout_s=BIG_MOVE_TIMEOUT_S)
    print(f"\nPUSH-OPEN DONE: door pushed to ~{plan['deg1']:.0f} deg. final EE {np.round(ai.get_state()['ee_pos'][:3], 4)}")
    return True


def _plan_close(scene, rb, df, st, args, deg0):
    """To `side_past_hinge` beyond the hinge and `push_radius` from it (diagonally, straight
    there, if that clears the door; else sideways in front of the free edge first), then swing
    about the hinge -- hand orientation unchanged the whole way, radius spiralling in to
    `push_radius_end` -- until it is `preload` into the closed door, then straight back out.
    This is the user's hand-demonstrated close (09-25): straight there kept J3 moving AWAY from
    +-180, where sideways-first drove it across."""
    fwd = np.r_[-df.n, 0.0]                        # into the microwave, closed-door normal
    side = np.r_[-df.e, 0.0]                       # along the closed door, toward the hinge
    q0 = np.array(st["position"], float)
    p_now = np.array(st["ee_pos"][:3])
    quat = _facing_forward(st["ee_pos"][3:7], fwd) if args.face_door else tuple(st["ee_pos"][3:7])
    z = args.push_z if args.push_z is not None else p_now[2]
    h = np.r_[df.hinge[:2], z]
    a = lambda pt: float(np.dot(pt - h, side))     # + toward/past the hinge
    b = lambda pt: float(np.dot(pt - h, fwd))      # + into the microwave (hinge at 0)

    bodies = df.bodies(scene, rb, deg0)
    free = add_door_model(scene, rb, df.door, df.bodies_point(deg0))
    drop(rb, free[0])
    free_edge = np.r_[free[1]["free_edge"], z]
    # sideways leg must pass in front of the free edge: fingertips + margin short of it
    b_safe = min(b(p_now), b(free_edge) - FINGERTIP_PAST_TOOL - CLOSE_FRONT_MARGIN)
    pos = lambda aa, bb: h + side * aa + fwd * bb
    # swing start: `side_past_hinge` past the hinge, or further until it clears the door at its
    # actual angle (a door open ~90 deg needs more than one open ~110 deg)
    a_side = args.side_past_hinge
    while True:
        if args.push_radius <= abs(a_side) or a_side > 0.25:
            print("no swing start clears the open door within 25 cm past the hinge -- refusing.")
            drop(rb, bodies)
            return None
        b_start = -np.sqrt(args.push_radius ** 2 - a_side ** 2)
        qs, err = solve_ik(scene, rb, pos(a_side, b_start), quat, q0)
        if err < MAX_IK_ERR and clearance(rb, qs, bodies)[0] >= GAP:
            break
        a_side += 0.01
    print(f"hinge {np.round(df.hinge[:2], 3)}; open door free edge ~{np.round(free_edge[:2], 3)} "
          f"(door {deg0:.0f} deg); hand now a {a(p_now) * 100:+.0f} cm / b {b(p_now) * 100:+.0f} cm from the hinge")
    print(f"plan: to a {a_side * 100:+.0f} cm (past the hinge) / b {b_start * 100:+.0f} cm (radius "
          f"{args.push_radius * 100:.0f} cm), swing shut spiralling in to {args.push_radius_end * 100:.0f} cm, back out "
          f"{args.retreat_dist * 100:.0f} cm; z {z:.3f}, approach "
          f"{np.round(R.from_quat(quat).as_matrix()[:, 2], 3)}")
    try:
        c0 = clearance(rb, q0, bodies)
        print(f"  clearance now: {c0[0] * 100:.1f} cm ({c0[1]})")
        start = (p_now, tuple(st["ee_pos"][3:7]))
        goal = (pos(a_side, b_start), quat)
        # 1. the demo (09-25): left at the current depth, then straight forward. 2. straight there. 3. round the free edge's corner.
        # 4. back to in front of the free edge first, then left, then forward.
        corner = pos(a(free_edge) + CLOSE_FRONT_MARGIN, b(free_edge) - FINGERTIP_PAST_TOOL - CLOSE_FRONT_MARGIN)
        routes = [("left, then forward", [start, (pos(a_side, b(p_now)), quat), goal]),
                  ("straight there", [start, goal]),
                   (f"round the free edge's corner via {np.round(corner[:2], 3)}", [start, (corner, quat), goal]),
                   (f"back to b {b_safe * 100:+.0f} cm, left, then forward",
                    [start, (pos(a(p_now), b_safe), quat), (pos(a_side, b_safe), quat), goal])]
        route = None
        for name, pts in routes:
            print(f" route: {name}")
            route, q = _chain(scene, rb, pts, q0, bodies, "approach")
            if route is not None:
                break
    finally:
        drop(rb, bodies)
    if route is None:
        return None

    # the swing: rotate the start point about the hinge, fixed orientation, until the hand is
    # `push_stop_short` from the CLOSED door (model), found step by step then bisected. The
    # model's closed face came out 1-1.5 cm too deep on 09-25, so the swing stops short and the
    # last bit is the torque-watched push (_push_until_shut).
    closed = df.bodies(scene, rb, 0.0)
    stop = args.push_stop_short
    r0 = pos(a_side, b_start) - h
    # radius spirals in from push_radius to push_radius_end over the swing
    # hand's angle from the closed door line at the start; it meets the closed door ~15 deg out
    span = max(float(np.degrees(np.arccos(np.clip(np.dot(r0[:2], df.e) / np.linalg.norm(r0[:2]), -1, 1)))) - 15.0, 10.0)
    shrink = lambda d: 1.0 - (1.0 - args.push_radius_end / args.push_radius) * min(d / span, 1.0)
    at = lambda d: h + np.r_[df._rot(-d) @ r0[:2] * shrink(d), 0.0]
    step = np.degrees(ARC_STEP_M / args.push_radius)
    plan, poses, d, qq = [], [], 0.0, q
    try:
        while True:
            d_next = d + step
            nq, err = solve_ik(scene, rb, at(d_next), quat, qq)
            c_door = clearance(rb, nq, [closed[0]])[0]
            if c_door <= stop:
                lo, hi = d, d_next                       # last step: land exactly on the preload
                for _ in range(12):
                    mid = (lo + hi) / 2
                    mq, _ = solve_ik(scene, rb, at(mid), quat, qq)
                    if clearance(rb, mq, [closed[0]])[0] <= stop:
                        hi = mid
                    else:
                        lo = mid
                d_next = hi
                nq, err = solve_ik(scene, rb, at(d_next), quat, qq)
                c_door = clearance(rb, nq, [closed[0]])[0]
            jump = float(np.degrees(np.max(np.abs((nq - qq + np.pi) % (2 * np.pi) - np.pi))))
            c_body = clearance(rb, nq, [closed[1]])
            j6 = float(np.degrees(nq[5]))
            last = c_door <= stop + 1e-4
            bad = (err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or abs(j6) > J6_GUARD_DEG
                   or not continuous_ok(qq, nq) or c_body[0] < MIN_CLEAR)
            k = len(plan) + 1
            if bad or last or k == 1 or k % 4 == 0:
                print(f"  swing {k} ({d_next:5.1f} deg round the hinge) -> {np.round(at(d_next), 3)}  IK err "
                      f"{err * 100:.2f}cm  jump {jump:.1f}deg  J6 {j6:.1f}  to closed door {c_door * 100:+.1f}cm  "
                      f"body {c_body[0] * 100:.1f}cm ({c_body[1]})")
            if bad:
                print(f"  swing: step {k} fails a gate.")
                return None
            plan.append(nq)
            poses.append((at(d_next), quat))
            qq, d = nq, d_next
            if last:
                break
            if d > deg0 + 60:
                print("  swing never reached the closed door in the model -- refusing.")
                return None
        end = at(d)
        u_end = float(np.dot(end[:2] - df.hinge[:2], df.e))
        print(f"  swing: {len(plan)} steps, {d:.0f} deg round the hinge; ends with the hand {u_end * 100:.0f} cm "
              f"along the door from the hinge (handle at {df.u_handle * 100:.0f} cm)")
        if u_end + CLOSE_HAND_HALF > df.u_handle - HANDLE_MARGIN:
            print("  the hand would end on the handle (not in the door model) -- refusing; lower --push-radius.")
            return None
        # final push: straight in, PUSH_STEP_M at a time, up to push_max (stopped at runtime by
        # the torque check once the door is shut)
        push, pq_ = [], qq
        n_push = int(round(args.push_max / PUSH_STEP_M))
        for k in range(1, n_push + 1):
            tgt = end + fwd * PUSH_STEP_M * k
            nq, err = solve_ik(scene, rb, tgt, quat, pq_)
            jump = float(np.degrees(np.max(np.abs((nq - pq_ + np.pi) % (2 * np.pi) - np.pi))))
            if err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or not continuous_ok(pq_, nq):
                print(f"  push step {k} fails a gate (IK err {err * 100:.2f}cm, jump {jump:.1f}deg) -- push capped there")
                break
            push.append(nq)
            pq_ = nq
        c_max = clearance(rb, pq_, [closed[0]])[0]
        print(f"  push: up to {len(push)} x {PUSH_STEP_M * 1000:.0f} mm straight in, stopping when the joint "
              f"torques say the door is shut (model: last step {c_max * 100:+.1f} cm vs the closed door)")
        back = None
        for dist in sorted({args.retreat_dist, 0.15, 0.10, 0.06}, reverse=True):
            if dist <= args.retreat_dist:
                back = plan_cartesian(scene, rb, end, quat, end - fwd * dist, quat, qq, f"back out {dist * 100:.0f} cm", None)
                if back is not None:
                    break
        if back is None:
            return None
        c = clearance(rb, back[-1], closed)
        print(f"  after backing out: {c[0] * 100:.1f} cm from door/body ({c[1]})")
    finally:
        drop(rb, closed)
    return {"before": route, "sweep": (plan, poses), "push": push, "after": [("back out", back)]}


def push_close(ai, args):
    """Push the open door shut with the hand facing forward throughout. The gripper is never
    touched (it may be holding a container)."""
    if not DOOR_FILE.exists():
        print(f"need {DOOR_FILE} -- refusing.")
        return False
    door = json.loads(DOOR_FILE.read_text())
    deg0 = args.door_deg if args.door_deg is not None else door.get("door_open_deg")
    if deg0 is None or "open_sign" not in door:
        print("door file has no door_open_deg/open_sign (the pull and push-open write them) -- refusing.")
        return False
    df = DoorFrame(door, door["open_sign"])
    st = ai.get_state()
    print(f"door open ~{deg0:.1f} deg ({'--door-deg' if args.door_deg is not None else 'door file'}); "
          f"gripper {float(st['gripper_pos']):.3f} (left as is)")
    scene, rb = make_sim()
    plan = _plan_close(scene, rb, df, st, args, deg0)
    if plan is None:
        print("\nno plan passes every gate -- refusing, arm untouched.")
        return False
    if not args.execute:
        print(f"\nDRY RUN (push-close from {deg0:.0f} deg) -- nothing commanded.")
        return True
    for label, leg in plan["before"]:
        if not execute_joint_plan(ai, leg, label):
            return False
    if not _run_sweep(ai, *plan["sweep"], args.smooth):
        return False
    done = _push_until_shut(ai, plan["push"], args.push_contact_nm)
    # back along the push steps actually taken, to the swing's end, then the planned back-out
    back_steps = plan["push"][:max(done - 1, 0)][::-1] + [plan["sweep"][0][-1]]
    if not execute_joint_plan(ai, back_steps, "off the door"):
        return False
    save_door_geometry(door_open_deg=0.0)
    for label, leg in plan["after"]:
        if not execute_joint_plan(ai, leg, label):
            return False
    st = ai.get_state()
    print(f"\nPUSH-CLOSE DONE. final EE {np.round(st['ee_pos'][:3], 4)}  gripper {float(st['gripper_pos']):.3f}")
    return True


def _push_until_shut(ai, push, contact_nm):
    """Step the planned push joints one at a time; after each, compare the joint torques
    (`effort`, J1-J4) with a baseline taken before the push. The door hitting its frame shows as
    a change that keeps RISING -- stop when it is above `contact_nm` and rose on two steps in a
    row, or when a step can't be reached. A single threshold can't do this: the torque zero
    drifts with pose (kinova-wrench-bias-not-noise). Returns the number of steps taken."""
    time.sleep(1.0)
    base = np.array(ai.get_state()["effort"], float)
    prev, rises = 0.0, 0
    print(f"push: baseline torques J1-J4 {np.round(base[:4], 2)} Nm; contact = change > {contact_nm} Nm, rising twice")
    for k, q in enumerate(push, 1):
        ai.execute_command(JointCommand(pos=q.tolist()))
        reached = wait_for_joints(ai, q, tol_deg=0.5, timeout_s=3.0)
        time.sleep(0.3)
        e = np.array(ai.get_state()["effort"], float)
        sig = float(np.linalg.norm(e[:4] - base[:4]))
        rises = rises + 1 if sig > prev + 0.05 else 0
        print(f"  push {k}: +{k * PUSH_STEP_M * 1000:.0f} mm  torque change {sig:5.2f} Nm  "
              f"{'rising' if rises else '-'}{'' if reached else '  NOT REACHED (blocked)'}")
        if not reached or (sig > contact_nm and rises >= 2):
            print(f"  -> door shut (push {k}); stopping the push")
            return k
        prev = sig
    print("  push: reached the planned maximum without a clear contact -- check the door by eye")
    return len(push)


def _run_sweep(ai, sweep, poses, smooth):
    if smooth:
        print(f"sweep: one blended Cartesian trajectory over {len(poses)} waypoints ...")
        ok = ai.execute_command(CartesianTrajectoryCommand([(list(pp), list(qq)) for pp, qq in poses]))
        err = float(np.linalg.norm(np.array(ai.get_state()["ee_pos"][:3]) - poses[-1][0]))
        print(f"  sweep {'done' if ok else 'RETURNED FALSE'}, {err * 100:.1f} cm from the last waypoint")
        return bool(ok) and err < 0.02
    return execute_joint_plan(ai, sweep, "sweep")


def add_push_args(a, close=False):
    a.add_argument("--push-preload", type=float, default=0.005 if close else 0.01,
                   help="push: how far past touching the hand is driven into the door face (m)")
    if close:
        a.add_argument("--door-deg", type=float, default=None,
                       help="push-close: current door opening (deg); default: the door file's last value")
        a.add_argument("--side-past-hinge", type=float, default=0.08,
                       help="push-close: swing start this far past the hinge, sideways (m)")
        a.add_argument("--push-radius", type=float, default=0.28,
                       help="push-close: swing start this far from the hinge (m)")
        a.add_argument("--push-radius-end", type=float, default=0.26,
                       help="push-close: the swing spirals in to this radius by the end, clear of the handle "
                            "(~33 cm out) -- the 09-25 hand demo went left to ~8 cm past the hinge, forward "
                            "to 28 cm, swung at 26-27 cm")
        a.add_argument("--face-door", action="store_true",
                       help="push-close: turn the approach to the closed door's normal first (default: keep "
                            "the hand's orientation as it is, like the demo)")
        a.add_argument("--push-stop-short", type=float, default=0.03,
                       help="push-close: end the swing this far short of the model's closed door (m); the model "
                            "put the closed face 1-1.5 cm too deep on 09-25")
        a.add_argument("--push-max", type=float, default=0.045,
                       help="push-close: final push straight in, at most this far (m), stopped by the torque check")
        a.add_argument("--push-contact-nm", type=float, default=3.0,
                       help="push-close: joint-torque change (J1-J4 norm, Nm) that, rising twice, means the door is shut")
        a.add_argument("--push-z", type=float, default=None,
                       help="push-close: height (m); default: where the hand is now")
        return
    a.add_argument("--push-back", type=float, default=0.21,
                   help="push-open: after releasing, back straight out this far (m)")
    a.add_argument("--push-in", type=float, default=-0.17,
                   help="push-open: the forward leg goes this far in front of the hinge's closed-door line (m, <0)")
    a.add_argument("--push-radius", type=float, default=0.345,
                   help="push-open: push starts this far from the hinge (m)")
    a.add_argument("--push-radius-end", type=float, default=0.29,
                   help="push-open: ... and spirals in to this (m)")
    a.add_argument("--push-target-deg", type=float, default=70.0,
                   help="push-open: hand ends this far round the hinge from the closed door (deg); the door ends a "
                        "few deg further (the 09-25 teleop demo: ~70)")
