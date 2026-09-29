"""Push the microwave door with the SIDE of the gripper -- no grasp.

* `push_close` (close task, VALIDATED on hardware 2026-09-25: door latched). The hand may be
  holding something else (a container), so the gripper is never touched and the hand's
  orientation never changes. The user's hand demo, as a plan: left (toward the hinge) at the
  current depth to ~8 cm past the hinge, forward to 28 cm from the hinge, swing about the hinge
  (radius spiralling in to 26 cm, clear of the handle) until 3 cm short of the model's closed
  door, then a torque-watched push in 5 mm steps that stops when the door hits its frame
  (`_push_until_shut`), then back out.
* `retreat_over` (open task, sim-only so far): after the pull, release, back off the handle, go up
  over the door's top, across to the microwave's middle, turn to face in, and come down -- the
  user's 09-28 hand demo, with every number from the detection (door_z / door_mid in DOOR_FILE).
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
    CartesianTrajectoryCommand, CloseGripperCommand, JointCommand, OpenGripperCommand)

from microwave_common import (
    BIG_MOVE_TIMEOUT_S, DOOR_FILE, J4_GUARD_DEG, DOOR_PAST_HANDLE, DOOR_T, FINGERTIP_PAST_TOOL, J6_GUARD_DEG, MAX_IK_ERR,
    MAX_STEP_JUMP_DEG, MIN_CLEAR, PARK_FILE, add_door_model, check_joint_path, clearance,
    continuous_ok, execute_joint_plan, run_cartesian_trajectory, load_park, make_sim, plan_cartesian, plan_straight_line,
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


def _chain(scene, rb, points, q, bodies, label, min_clear=None, closed=False):
    """plan_cartesian through a list of (pos, quat) points; returns (legs, q_end) or (None, q).
    `min_clear`: optional per-leg clearance, {leg index (0-based): metres}, default MIN_CLEAR."""
    legs = []
    for i in range(len(points) - 1):
        (p0, q0), (p1, q1) = points[i], points[i + 1]
        leg = plan_cartesian(scene, rb, p0, q0, p1, q1, q, f"{label} {i + 1}", bodies,
                             min_clear=(min_clear or {}).get(i), closed=closed)
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
               or abs(np.degrees(nq[3])) > J4_GUARD_DEG
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
    back straight out -> right, past the open door's free edge -> forward into the gap between
    the door and the microwave front, turning the wrist to face into the microwave on the way -> left
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
    a_right = a(edge) - args.push_right_margin
    b_in = args.push_in
    r0 = args.push_radius
    if r0 <= abs(b_in):
        print("--push-radius must exceed |--push-in| -- refusing.")
        return None
    phi0 = float(np.degrees(np.arcsin(-b_in / r0)))
    phi1 = args.push_target_deg
    print(f"door ~{deg0:.0f} deg; free edge {np.round(edge[:2], 3)} (a {a(edge) * 100:+.0f} / b {b(edge) * 100:+.0f} cm)")
    print(f"plan: back {args.push_back * 100:.0f} cm, right to a {a_right * 100:+.0f} cm, forward (turning the wrist) to "
          f"b {b_in * 100:+.0f} cm, left to the inner face ({phi0:.0f} deg round the hinge, r {r0 * 100:.0f} cm), push "
          f"to {phi1:.0f} deg (r -> {args.push_radius_end * 100:.0f} cm), right, back, park")
    if phi0 >= deg0 - 5:
        print(f"push start ({phi0:.0f} deg) is not behind the door ({deg0:.0f} deg) -- refusing.")
        return None

    bodies = df.bodies(scene, rb, deg0)
    try:
        # the wrist turns to face in DURING the forward leg, not in place after the back-off:
        # close to the base and low (09-28, handle z ~0.26) facing in there needs J6 ~119 deg
        pts = [(ee[:3], g_quat), (p_back, g_quat), (pos(a_right, b(p_back)), g_quat),
               (pos(a_right, b_in), quat), (at(phi0, r0), quat)]
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


PUSH_Y_STEP_M = 0.01        # push-open (axes): step of the +y push leg in the sim check
OFF_DOOR_M = 0.06           # push-open (axes): after the push, this far -y off the door before backing out


def _wrapped_jump_deg(q_from, q_to):
    return float(np.degrees(np.max(np.abs((np.asarray(q_to) - q_from + np.pi) % (2 * np.pi) - np.pi))))


def _plan_open_axes(scene, rb, df, st, args, deg0):
    """Push-open with straight moves (the user's 09-28 sequence): release -> slightly out along the
    gripper's own approach axis (off the handle) -> back (-x) -> straighten the hand to face into the
    microwave (-closed normal; kept from here on -- a hand still turned with the door leaves the wrist
    over the door at the end of the push) -> right (-y) past the open door's free edge -> forward (+x)
    behind the door -> left (+y) pushing the door's inner face until the hand reaches the door as
    modeled at --push-target-deg -> off the door (-y) -> back (-x). The approach and way-out legs
    are clearance-checked against the door model; the push leg only against the microwave body
    (it is the leg that touches the door). Returns the corner points plus the checked joints."""
    ee, q0 = np.array(st["ee_pos"], float), np.array(st["position"], float)
    g_quat, z = tuple(ee[3:7]), float(ee[2])
    quat = tuple(_facing_forward(g_quat, np.r_[-df.n, 0.0]))     # straightened: facing into the microwave
    free = add_door_model(scene, rb, df.door, df.bodies_point(deg0))
    drop(rb, free[0])
    edge = free[1]["free_edge"]
    appr = R.from_quat(g_quat).as_matrix()[:, 2]
    appr = np.r_[appr[:2] / np.linalg.norm(appr[:2]), 0.0]
    p_rel = ee[:3] - appr * args.push_release_out                  # slightly out, off the handle
    p_back = p_rel + np.array([-args.push_back, 0.0, 0.0])
    y_right = min(float(edge[1]) - args.push_right_margin, float(p_back[1]))
    x_push = float(df.hinge[0]) - args.push_depth
    p_right = np.array([p_back[0], y_right, z])
    p_fwd = np.array([x_push, y_right, z])
    deg1 = args.push_target_deg
    print(f"door ~{deg0:.0f} deg; free edge {np.round(edge, 3)}; hinge {np.round(df.hinge[:2], 3)}")
    print(f"plan: out {args.push_release_out * 100:.0f} cm along the approach {np.round(-appr[:2], 2)}, "
          f"back -x to x {p_back[0]:.3f}, right -y to y {y_right:.3f} "
          f"({args.push_right_margin * 100:.0f} cm past the edge), forward +x (straightening on the way) to x {x_push:.3f}, "
          f"left +y until the door is at ~{deg1:.0f} deg, off -y {OFF_DOOR_M * 100:.0f} cm, back -x {args.push_back * 100:.0f} cm")
    if x_push <= p_back[0] + 0.02:
        print("forward target is not in front of the back-off point -- refusing.")
        return None

    bodies = df.bodies(scene, rb, deg0)
    try:
        # straightening happens DURING the forward leg: in place at the back-off point (close to the
        # base, low) the facing-in pose is at the edge of the workspace (09-28 sim: IK err > 2 cm),
        # and during the right leg the wrist spins across +-180
        legs, q = _chain(scene, rb, [(ee[:3], g_quat), (p_rel, g_quat), (p_back, g_quat),
                                     (p_right, g_quat), (p_fwd, quat)], q0, bodies, "approach")
    finally:
        drop(rb, bodies)
    if legs is None:
        return None

    door0, door1 = df.bodies(scene, rb, deg0), df.bodies(scene, rb, deg1)
    try:
        push, pts, y, first = [], [], y_right, None
        while True:
            y += PUSH_Y_STEP_M
            if y - y_right > 0.60:
                print("push: 60 cm of +y without reaching the modeled door -- refusing.")
                return None
            pt = np.array([x_push, y, z])
            nq, err = solve_ik(scene, rb, pt, quat, q)
            jump = _wrapped_jump_deg(q, nq)
            c_body = clearance(rb, nq, [door1[1]])
            c0, c1 = clearance(rb, nq, [door0[0]]), clearance(rb, nq, [door1[0]])
            j6 = float(np.degrees(nq[5]))
            if first is None and c0[0] <= 0.0:
                first = (y, c0[1])
            bad = (err > MAX_IK_ERR or jump > MAX_STEP_JUMP_DEG or abs(j6) > J6_GUARD_DEG
               or abs(np.degrees(nq[3])) > J4_GUARD_DEG
                   or abs(np.degrees(nq[3])) > J4_GUARD_DEG
                   or not continuous_ok(q, nq) or c_body[0] < MIN_CLEAR)
            if bad:
                print(f"  push -> {np.round(pt, 3)}  IK err {err * 100:.2f}cm  jump {jump:.1f}deg  J6 {j6:.1f}  "
                      f"body {c_body[0] * 100:.1f}cm ({c_body[1]}) -- fails a gate.")
                return None
            push.append(nq)
            pts.append(pt)
            q = nq
            if c1[0] <= 0.0:
                break
    finally:
        drop(rb, door0 + door1)
    if first is None:
        print("push: the hand never meets the door at its current angle -- forward target too shallow? Refusing.")
        return None
    print(f"  push: first touch at y {first[0]:.3f} with the {first[1]} (door ~{deg0:.0f} deg); "
          f"ends at y {pts[-1][1]:.3f} (door ~{deg1:.0f} deg); J6 {np.degrees(q[5]):.1f}; {len(push)} checked steps")

    p_end = pts[-1]
    # off the door (-y) first, then back out (-x): with the hand straightened, -y moves straight
    # away from the pushed face; backing out first slides the fingers along it (09-28 sim: 2.1 cm)
    p_off = p_end + np.array([0.0, -OFF_DOOR_M, 0.0])
    p_out = p_off + np.array([-args.push_back, 0.0, 0.0])
    bodies1 = df.bodies(scene, rb, deg1 + OPEN_PUSH_LAG)
    try:
        out, q = _chain(scene, rb, [(p_end, quat), (p_off, quat), (p_out, quat)], push[-1], bodies1, "out")
    finally:
        drop(rb, bodies1)
    if out is None:
        return None
    corners = ([(p_rel, g_quat), (p_back, g_quat)]
               + [(p_right, g_quat)] + [(c, quat) for c in (p_fwd, p_end, p_off, p_out)])
    return {"corners": [(np.asarray(c).tolist(), list(qq)) for c, qq in corners], "deg1": deg1}


# The user's hand demo of the push-open (09-28, microwave/demos/microwave_manual_push_open_2026-09-28.csv), from the
# end of a 45-deg pull, after releasing: corner points as (fwd, left, yaw) OFFSETS FROM THE RELEASE POSE
# in the CLOSED door's frame -- fwd = into the microwave (-closed normal), left = 90 deg left of that,
# yaw = hand turn about vertical (deg) -- so it follows the microwave when it moves or turns.
# out (off the handle) -> back -> right past the free edge -> turn the hand in place -> forward behind
# the door -> push left along the door's arc (the door just stays where it is pushed to).
# "back" is at the demo's t=23.9 s, not its end (t=25), and "right" goes there diagonally instead of
# along the demo's x ~0.41 slide: the hand-guided demo held J4 at -146..-147.8 there, which a commanded
# move can't (soft limit -147.8, guard 144). The push is cut where J4/J6 would pass their guards.
DEMO_OPEN_POINTS = [(-0.005, +0.035, 0.0, "out"),
                    (-0.086, +0.021, 0.0, "back"),
                    (+0.066, -0.250, 0.0, "right"),
                    (+0.066, -0.250, 36.5, "turn"),
                    (+0.111, -0.218, 36.5, "forward")]
DEMO_PUSH_POINTS = [(+0.104, -0.158, 36.5, "push 1"),
                    (+0.080, -0.082, 36.5, "push 2"),
                    (+0.038, +0.005, 36.5, "push 3"),
                    (-0.007, +0.085, 36.5, "push end")]
DEMO_MIN_PUSH_POINTS = 2             # refuse unless at least this many push points pass the gates
# The leg to "right" passes the open door's free edge with the elbow near its J4 limit: backing off
# more runs J4 into its guard, less passes the edge closer. No route clears both at MIN_CLEAR (09-28
# sim sweep), so this one leg is allowed 2 cm (user's call, 09-28: the door edge model is an estimate
# and a graze won't damage the door or arm).
DEMO_RIGHT_MIN_CLEAR = 0.02


def _plan_open_demo(scene, rb, df, st, args, deg0):
    """Replay the demo from the end of the pull (after releasing). The legs up to "forward" are
    clearance-checked against the door model at its current angle and must all pass; the push only
    against the microwave body (it is the leg that touches the door) and is cut short at the first
    point that fails a gate (J4 / J6 / IK / jump / wrap), keeping at least DEMO_MIN_PUSH_POINTS."""
    ee, q0 = np.array(st["ee_pos"], float), np.array(st["position"], float)
    fwd = -df.n / np.linalg.norm(df.n)
    left = np.array([-fwd[1], fwd[0]])
    if abs(deg0 - 45.0) > 10.0:
        print(f"door ~{deg0:.0f} deg, but the demo was recorded from a 45-deg pull -- refusing.")
        return None
    q_rel = R.from_quat(ee[3:7])

    def pose(dfwd, dleft, dyaw):
        return (ee[:3] + np.r_[fwd * dfwd + left * dleft, 0.0],
                tuple((R.from_euler("z", dyaw, degrees=True) * q_rel).as_quat()))

    pts = [(ee[:3], tuple(ee[3:7]))] + [pose(*pt[:3]) for pt in DEMO_OPEN_POINTS]
    print(f"door ~{deg0:.0f} deg; demo from {np.round(ee[:3], 3)}: "
          + "; ".join(f"{pt[3]} {np.round(pp, 3)}" for pt, (pp, _) in zip(DEMO_OPEN_POINTS, pts[1:])))
    # open hand (just released) for "out" and "back"; the gripper closes there and the rest -- round
    # the free edge and the push -- is done with a fist, which is much narrower (09-28: with the hand
    # open, every route round the edge put a finger pad inside the door model)
    i_back = next(i for i, pt in enumerate(DEMO_OPEN_POINTS) if pt[3] == "back") + 1   # index into pts
    i_right = next(i for i, pt in enumerate(DEMO_OPEN_POINTS) if pt[3] == "right") + 1
    door_now = df.bodies(scene, rb, deg0)
    try:
        legs, q = _chain(scene, rb, pts[:i_back + 1], q0, door_now, "demo (open hand)")
        if legs is not None:
            more, q = _chain(scene, rb, pts[i_back:], q, door_now, "demo (closed hand)", closed=True,
                             min_clear={i_right - i_back - 1: DEMO_RIGHT_MIN_CLEAR})
            legs = None if more is None else legs + more
    finally:
        drop(rb, door_now)
    if legs is None:
        return None
    body = df.bodies(scene, rb, deg0)
    drop(rb, [body[0]])                      # the push touches the door: check it against the body only
    kept = []
    try:
        prev = pts[-1]
        for pt in DEMO_PUSH_POINTS:
            nxt = pose(*pt[:3])
            leg, q2 = _chain(scene, rb, [prev, nxt], q, [body[1]], pt[3], closed=True)
            if leg is None:
                print(f"  push cut before '{pt[3]}' ({len(kept)}/{len(DEMO_PUSH_POINTS)} push points kept)")
                break
            kept.append(nxt)
            prev, q = nxt, q2
    finally:
        drop(rb, [body[1]])
    if len(kept) < DEMO_MIN_PUSH_POINTS:
        print(f"only {len(kept)} push point(s) pass the gates (need {DEMO_MIN_PUSH_POINTS}) -- refusing.")
        return None
    print(f"  push ends at {np.round(kept[-1][0], 3)} ({len(kept)}/{len(DEMO_PUSH_POINTS)} of the demo's push points)")
    corners = [(np.asarray(pp).tolist(), list(qq)) for pp, qq in pts[1:] + kept]
    return {"corners": corners, "deg1": None, "close_after": i_back}   # close the gripper after corner i_back


def push_open(ai, args, sim=None):
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
    scene, rb = sim if sim is not None else make_sim()   # sim: reuse a caller's (scene, robot)
    style = getattr(args, "push_style", "arc")
    if style in ("axes", "demo"):
        plan = (_plan_open_axes if style == "axes" else _plan_open_demo)(scene, rb, df, st, args, deg0)
        if plan is None:
            print("\nthe axis push-open fails a gate -- NOT releasing, arm untouched.")
            return False
        if not args.execute:
            print(f"\nDRY RUN (push-open {deg0:.0f} -> ~{plan['deg1']:.0f} deg) -- nothing commanded.")
            return True
        print("Releasing ...")
        ai.execute_command(OpenGripperCommand())
        time.sleep(1.0)
        k = plan.get("close_after")
        if k:
            # demo: out + back with the hand open, close the gripper, then the rest with a fist
            print(f"push-open: {k} corner(s) with the hand open ...")
            ok, err, _ = run_cartesian_trajectory(ai, plan["corners"][:k])
            if not ok:
                print(f"  stopped {err * 100:.1f} cm short of the back-off point -- stopping here.")
                return False
            ai.execute_command(CloseGripperCommand())
            time.sleep(1.5)
            print(f"push-open: gripper closed; {len(plan['corners']) - k} corner(s) round the edge and the push ...")
            ok, err, _ = run_cartesian_trajectory(ai, plan["corners"][k:])
        else:
            # all the straight legs as one blended Cartesian trajectory (<= 1 cm corner blends)
            print(f"push-open: one blended trajectory through {len(plan['corners'])} corners ...")
            ok, err, _ = run_cartesian_trajectory(ai, plan["corners"])
        if plan["deg1"] is not None:
            save_door_geometry(door_open_deg=plan["deg1"], open_sign=df.sign)
        print(f"\nPUSH-OPEN {'DONE' if ok else 'STOPPED SHORT'}: {err * 100:.1f} cm from the last corner"
              + (f"; door ~{plan['deg1']:.0f} deg (model)" if plan["deg1"] is not None else ""))
        return bool(ok)
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


# ---------------------------------------------------------------------------
# Retreat over the top of the open door (user's hand demo, 09-28)
# ---------------------------------------------------------------------------
RETREAT_OUT_MIN_CLEAR = 0.005   # the "out" leg starts with the fingertips ~0.5 cm off the door face (the model
                                # puts the face there at grasp time) and moves straight away from it


def _plan_retreat_over(scene, rb, door, st, args):
    """From the end of the pull (holding the handle): out along -approach first (the bar is fixed to
    the door at its top, so the fingers must clear it before going up), up to the detected door top
    + `retreat_above`, across to the door's middle (y of `door_mid`), straighten (approach along the
    closed door's inward normal), down to the grasp height. Every leg is IK/jump/J4/J6/wrap-gated and
    clearance-checked, open hand, against the door model at its current angle + the microwave body.
    Returns {"corners": [(pos, quat)], "deg": opening} or None."""
    if not door.get("door_z") or door.get("door_mid") is None:
        print("door file has no door_z/door_mid (a grasp with the updated detector writes them) -- refusing.")
        return None
    ee, q0 = np.array(st["ee_pos"], float), np.array(st["position"], float)
    a = DoorFrame(door, 1.0).angle_of(ee)
    df = DoorFrame(door, np.sign(a) or 1.0)
    deg = abs(a)
    quat0 = tuple(ee[3:7])
    approach = R.from_quat(quat0).as_matrix()[:, 2]
    z_over = float(door["door_z"][1]) + args.retreat_above
    z_down = float(door["closed_grasp_pos"][2])
    p_out = ee[:3] - approach * args.retreat_out
    if z_over <= p_out[2]:
        print(f"door top + {args.retreat_above * 100:.0f} cm (z {z_over:.3f}) is not above the hand (z {p_out[2]:.3f}) "
              "-- door_z looks wrong, refusing.")
        return None
    p_up = np.array([p_out[0], p_out[1], z_over])
    p_across = np.array([p_out[0], float(door["door_mid"][1]), z_over])
    q_face = tuple(_facing_forward(quat0, np.r_[-df.n, 0.0]))
    p_down = np.array([p_out[0], p_across[1], z_down])
    pts = [(ee[:3], quat0), (p_out, quat0), (p_up, quat0), (p_across, quat0), (p_across, q_face), (p_down, q_face)]
    print(f"door ~{deg:.0f} deg, top z {door['door_z'][1]:.3f}, mid y {door['door_mid'][1]:.3f}; retreat-over: "
          + "; ".join(f"{nm} {np.round(pp, 3)}" for nm, (pp, _) in
                      zip(("out", "up", "across", "straighten", "down"), pts[1:])))
    bodies = df.bodies(scene, rb, deg)
    try:
        legs, _ = _chain(scene, rb, pts, q0, bodies, "retreat-over", min_clear={0: RETREAT_OUT_MIN_CLEAR})
    finally:
        drop(rb, bodies)
    if legs is None:
        return None
    return {"corners": [(np.asarray(pp).tolist(), list(qq)) for pp, qq in pts[1:]], "deg": deg}


def retreat_over(ai, args, sim=None):
    """After the pull (still holding the handle): plan the retreat from the real state, and only if
    every leg passes, release and run it as one blended Cartesian trajectory."""
    if not DOOR_FILE.exists():
        print(f"need {DOOR_FILE} (grasp + swing write it) -- refusing.")
        return False
    door = json.loads(DOOR_FILE.read_text())
    st = ai.get_state()
    if float(st["gripper_pos"]) < 0.2:
        print("gripper is open -- retreat-over starts from the end of the pull, still holding the handle. Refusing.")
        return False
    scene, rb = sim if sim is not None else make_sim()
    plan = _plan_retreat_over(scene, rb, door, st, args)
    if plan is None:
        print("\nretreat-over fails a gate -- NOT releasing, arm untouched.")
        return False
    if not args.execute:
        print(f"\nDRY RUN (retreat-over from ~{plan['deg']:.0f} deg) -- nothing commanded.")
        return True
    print("Releasing ...")
    ai.execute_command(OpenGripperCommand())
    time.sleep(1.0)
    print(f"retreat-over: one blended trajectory through {len(plan['corners'])} corners ...")
    ok, err, rpc_ok = run_cartesian_trajectory(ai, plan["corners"])
    print(f"\nRETREAT-OVER {'DONE' if ok else 'STOPPED SHORT'} (RPC returned {rpc_ok}): {err * 100:.1f} cm from the "
          f"last corner. final EE {np.round(ai.get_state()['ee_pos'][:3], 4)}")
    return bool(ok)


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
    if args.route_over:
        # --route-over: the swing/push run at mid door height (unless --push-z)
        if not df.door.get("door_z"):
            print("--route-over needs door_z in the door file (the grasp/detection writes it) -- refusing.")
            return None
        z_top = float(df.door["door_z"][1])
        if args.push_z is None:
            z = float(np.mean(df.door["door_z"]))
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
        if args.route_over:
            # 09-28 (user): going left at hand height drives J4 into its limit, so instead go UP over
            # the open door, LEFT past the swing start, DOWN to mid microwave height, then RIGHT and
            # FORWARD into the swing start -- the swing and push after that are unchanged
            z_over = z_top + args.over_above
            up = np.r_[p_now[:2], z_over]
            left_hi = pos(a_side + args.over_past, b(p_now))
            left_hi[2] = z_over
            down = pos(a_side + args.over_past, b(p_now))
            print(f"route-over: up to z {z_over:.3f} (door top {z_top:.3f} + {args.over_above * 100:.0f} cm), left to "
                  f"a {(a_side + args.over_past) * 100:+.0f} cm, down to z {z:.3f} (mid door), right + forward to the swing start")
            routes = [("over the door: up, left, down, right + forward",
                       [start, (up, quat), (left_hi, quat), (down, quat), goal])]
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
               or abs(np.degrees(nq[3])) > J4_GUARD_DEG
                   or abs(np.degrees(nq[3])) > J4_GUARD_DEG
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
    if args.hold_offset_x:
        # holding a container that sticks out past the fingertips: plan against the door/microwave
        # shifted by this much in x (planning copy only, the door file is not changed), so the hand
        # stays that far back and the container fills the gap
        for k in ("hinge", "closed_grasp_pos", "closed_handle", "door_mid"):
            if k in door:
                door[k] = [door[k][0] + args.hold_offset_x, *door[k][1:]]
        print(f"hold offset: planning with the door shifted {args.hold_offset_x * 100:+.1f} cm in x "
              f"(container); hinge for the plan {np.round(door['hinge'][:2], 3)}")
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
        a.add_argument("--route-over", action="store_true",
                       help="push-close: approach the swing start over the top of the open door (up, left, down to "
                            "mid door height, right + forward) instead of left at hand height -- avoids J4's limit")
        a.add_argument("--over-above", type=float, default=0.08,
                       help="push-close --route-over: go this far above the detected door top (m)")
        a.add_argument("--over-past", type=float, default=0.05,
                       help="push-close --route-over: go this far further left than the swing start before coming "
                            "down, then right + forward into it (m)")
        a.add_argument("--hold-offset-x", type=float, default=0.0,
                       help="push-close: shift the door model this far in arm-base x for planning (m); "
                            "-0.152 (6 in) when holding a container, so the hand stays back by its length")
        return
    a.add_argument("--retreat-out", type=float, default=0.05,
                   help="retreat-over: after releasing, first move this far back along the approach axis, "
                        "off the handle, before going up (m)")
    a.add_argument("--retreat-above", type=float, default=0.08,
                   help="retreat-over: go up to this far above the detected door top (m)")
    a.add_argument("--push-style", choices=["arc", "axes", "demo"], default="arc",
                   help="push-open: 'arc' = the 09-25 demo (wrist turned to face in, push along an arc); "
                        "'axes' = straight back / right / forward / left legs; 'demo' = replay the 09-28 hand "
                        "demo (back, push forward-left while turning in, reposition right)")
    a.add_argument("--push-right-margin", type=float, default=OPEN_RIGHT_MARGIN,
                   help="push-open: go this far right (-y) of the open door's free edge before going forward (m)")
    a.add_argument("--push-release-out", type=float, default=0.05,
                   help="push-open (axes): after releasing, first move this far back along the gripper's approach "
                        "axis, off the handle (m)")
    a.add_argument("--push-depth", type=float, default=0.25,
                   help="push-open (axes): the forward (+x) leg ends this far in front of the hinge, in x (m)")
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
