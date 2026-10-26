"""The fridge's push-open and push-close: the microwave's, plus what a tall fridge door needs.

    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --fridge --push-open [--execute]   # open
    python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --fridge [--execute]       # close
    python3 -u microwave/fridge_push.py [--execute]   # just the push-open, from the end of a pull (still holding)

* `push_open` -- after a PUSH_OPEN_PULL_DEG pull (still holding the handle), the microwave's push-open idea: let go
  and push the door further open, as ONE smooth sim-gated trajectory (`demo_replay`'s planning and execution, as
  the microwave's). The microwave's demo goes up and over the door and faces the hand into the cabinet; neither works
  on a tall fridge door this close to the arm, so the route is computed for today's door: straight out off the
  handle, along the open door to past its free edge, round it -- turning the hand PARALLEL to the door (fingers
  along it toward the hinge, rolled vertical) and dropping a little -- in behind the door, then the side of the hand
  onto the inner face and the push, held in the door's frame, round to PUSH_OPEN_TO_DEG. A few hand placements are
  tried in order (PUSH_OPEN_AT_M x PUSH_OPEN_APPROACH_DEG x tool y up/down); the first whose whole route passes
  every gate is used.
* `plan_release_around` -- after the full pull (the default open): let go, back off the handle, then back along the
  open door past its free edge, round it and in beside the door, facing the fridge -- so the arm ends on the door's
  inner side. Planned (back-off + route, door-model-checked) before letting go; the open plans it with the grasp too.
* `close` -- the microwave's push-close (`door_push.push_close`: swing about the hinge, hand orientation fixed) with
  three changes: (1) the user first moves the wheelchair back ~CHAIR_BACK_M (prompted; the recorded fridge is
  shifted by that much for the plan), (2) the route to the swing start is back out in front of the open door's
  free edge, left behind it, then forward -- no going over the door, (3) the swing stops when the door is at
  CLOSE_TO_DEG (the fridge door swings shut by itself from there): no torque push.

Both use the fridge's door file (fridge_constants.DOOR_FILE, written by the `--fridge` open), never the microwave's.
"""
import argparse, contextlib, io, json, sys

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R

import demo_replay
import fridge_constants as F
import microwave_common
from door_push import DoorFrame, _chain, drop, push_close
from microwave_common import ARM, MAX_IK_ERR, clearance, make_sim, plan_straight_line, save_door_geometry, solve_ik


_SIM = None
_KEEP_OUT_BOX = {}                  # physics client id -> the keep-out box's body id


def keep_out(rb, q):
    """Why joints `q` put the arm in the wheelchair keep-out zone (more than CHAIR_KEEP_OUT_BEHIND_M behind the arm base,
    on its right), or None. The zone is a collision box in `rb`'s sim, added on first use; the arm is left at `q`."""
    c = rb.physics_client_id
    if c not in _KEEP_OUT_BOX:
        base = np.asarray(p.getBasePositionAndOrientation(rb.robot_id, physicsClientId=c)[0])
        half = np.array([2.0, 2.0, 3.0])
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=half.tolist(), physicsClientId=c)
        _KEEP_OUT_BOX[c] = p.createMultiBody(0, cs, basePosition=(base + [-F.CHAIR_KEEP_OUT_BEHIND_M - half[0], -half[1],
                                                                          0.0]).tolist(), physicsClientId=c)
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=c)
    pts = p.getClosestPoints(rb.robot_id, _KEEP_OUT_BOX[c], 0.0, physicsClientId=c)
    if not pts:
        return None
    worst = min(pts, key=lambda pt: pt[8])
    link = p.getJointInfo(rb.robot_id, worst[3], physicsClientId=c)[12].decode() if worst[3] >= 0 else "base"
    return (f"{link} {-worst[8] * 100:.1f} cm into the wheelchair keep-out (> {F.CHAIR_KEEP_OUT_BEHIND_M * 100:.0f} cm "
            "behind the arm base, on its right)")


def enable_keep_out():
    """Fridge runs: every plan (microwave_common.solve_ik, the open's swing check) and every trajectory SheppyArm sends
    is checked against the wheelchair keep-out (keep_out). The microwave never calls this."""
    microwave_common.KEEP_OUT = keep_out


def placement_advice(handle, normal, start_joints=None):
    """Where to move the wheelchair to get the handle into the placement that passed in sim."""
    x, y = float(handle[0]), float(handle[1])
    yaw = float(np.degrees(np.arctan2(-normal[1], -normal[0])))
    cm = lambda d: f"~{max(1, round(d * 100))} cm"
    moves = []
    if x > F.PLACEMENT_X[1]:
        moves.append(f"move the wheelchair {cm(x - F.PLACEMENT_X[1])} closer to the fridge")
    elif x < F.PLACEMENT_X[0]:
        moves.append(f"move the wheelchair {cm(F.PLACEMENT_X[0] - x)} back from the fridge")
    if y < F.PLACEMENT_Y[0]:
        moves.append(f"move the wheelchair {cm(F.PLACEMENT_Y[0] - y)} to its right")
    elif y > F.PLACEMENT_Y[1]:
        moves.append(f"move the wheelchair {cm(y - F.PLACEMENT_Y[1])} to its left")
    if yaw > F.PLACEMENT_YAW_DEG[1]:
        moves.append(f"turn the wheelchair ~{yaw - F.PLACEMENT_YAW_DEG[1]:.0f} deg so its front swings left")
    elif yaw < F.PLACEMENT_YAW_DEG[0]:
        moves.append(f"turn the wheelchair ~{F.PLACEMENT_YAW_DEG[0] - yaw:.0f} deg so its front swings right")
    lines = [f"placement: handle x {x:.3f} y {y:.3f}, yaw {yaw:+.1f} deg (passed in sim: x {F.PLACEMENT_X[0]:.2f}-"
             f"{F.PLACEMENT_X[1]:.2f}, y {F.PLACEMENT_Y[0]:.2f} to {F.PLACEMENT_Y[1]:.2f}, yaw "
             f"{F.PLACEMENT_YAW_DEG[0]:+.0f} to {F.PLACEMENT_YAW_DEG[1]:+.0f}, with --push-open --swing-deg "
             f"{F.PLACEMENT_PULL_DEG} from the Fridge start pose)"]
    lines += [f"  -> {m}" for m in moves] or ["  -> inside that placement"]
    if moves and len(moves) > 1:
        lines.append("  (re-check after each move: turning the chair also shifts the handle)")
    if start_joints is not None and np.degrees(start_joints[0]) < 0:
        lines.append(f"  -> start from the web app's Fridge start pose (J1 now {np.degrees(start_joints[0]):.0f} deg)")
    return "\n".join(lines)


def _sim():
    """One planning sim for the candidate search (demo_replay.replay makes its own for the run itself)."""
    global _SIM
    if _SIM is None:
        _SIM = make_sim()
    return _SIM


class _State:
    """An arm state to plan from (the real arm's, or a simulated end of the pull)."""

    def __init__(self, st):
        self.st = st

    def get_state(self):
        return self.st


def hand_quat(df, deg, approach_deg, y_up):
    """The hand behind the door open `deg`: approach level, `approach_deg` off "along the door toward the hinge"
    toward "into the cabinet"; tool y up/down (`y_up` +-1); then rolled PUSH_OPEN_ROLL_DEG about the approach."""
    e2, n2 = df._rot(deg) @ df.e, df._rot(deg) @ df.n
    a = np.radians(approach_deg)
    z = np.r_[-(np.cos(a) * e2 + np.sin(a) * n2), 0.0]
    y = np.array([0.0, 0.0, y_up])
    return (R.from_matrix(np.column_stack([np.cross(y, z), y, z]))
            * R.from_euler("z", F.PUSH_OPEN_ROLL_DEG, degrees=True)).as_quat()


def _touch_v(scene, rb, df, deg, u, z, quat, v_far, q_seed):
    """Door-frame v (off the mid-plane) where the hand at (u, z, quat) just touches the door model at `deg`, found by
    bisection from `v_far` (clear) toward the inner face; None if no IK there."""
    bodies = df.bodies(scene, rb, deg)
    try:
        def clear(v):
            q, err = solve_ik(scene, rb, df.pos(u, v, deg, z), quat, q_seed)
            for _ in range(3):
                if err <= MAX_IK_ERR:
                    break
                q, err = solve_ik(scene, rb, df.pos(u, v, deg, z), quat, q)
            return None if err > MAX_IK_ERR else clearance(rb, q, bodies)[0]
        c = clear(v_far)
        if c is None or c <= 0.0:
            return None
        lo, hi = v_far, -df.t / 2                     # lo: clear, hi: the inner face itself
        for _ in range(12):
            mid = (lo + hi) / 2
            cm = clear(mid)
            if cm is None:
                return None
            lo, hi = (mid, hi) if cm > 0.0 else (lo, mid)
        return hi
    finally:
        drop(rb, bodies)


def push_open_points(door, st, u_push, approach_deg, y_up, scene=None, rb=None):
    """Keypoints [(pos, quat, label)] from the hand on the handle (`st`, the end of the pull) to the end of the push for
    one hand placement, and the index of the first push keypoint (legs from there are checked against the cabinet only:
    they touch the door on purpose). None if the hand can't be placed against the inner face."""
    if scene is None:
        scene, rb = _sim()
    df = DoorFrame(door, door["open_sign"])
    deg0 = float(door["door_open_deg"])
    ee = np.asarray(st["ee_pos"], float)
    z, q_hold = float(ee[2]), ee[3:7]
    d = df._rot(deg0).T @ (ee[:2] - df.hinge[:2])               # the hand in the open door's frame (u along, v off it)
    u_hold, v_out = float(d @ df.e), float(d @ df.n) - df.mid + F.PUSH_OPEN_OFF_HANDLE_M
    u_edge = df.length + F.PUSH_OPEN_EDGE_CLEAR_M
    v_in, z_in = -(df.t / 2 + F.PUSH_OPEN_GAP_M), z + F.PUSH_OPEN_DZ_M
    q_in = hand_quat(df, deg0, approach_deg, y_up)
    v_touch = _touch_v(scene, rb, df, deg0, u_push, z_in, q_in, v_in, np.asarray(st["position"], float))
    if v_touch is None:
        return None, None
    v_push = v_touch + F.PUSH_OPEN_PRELOAD_M                     # past touching, into the inner face
    pts = [(ee[:3], q_hold, "end of the pull, holding"),
           (df.pos(u_hold, v_out, deg0, z), q_hold, "off the handle"),
           (df.pos(u_edge, v_out, deg0, z), q_hold, "past the free edge"),
           (df.pos(u_edge, v_in, deg0, z_in), q_in, "round the edge, turned"),
           (df.pos(u_push, v_in, deg0, z_in), q_in, "in behind the door")]
    first_push = len(pts)
    pts.append((df.pos(u_push, v_push, deg0, z_in), q_in, "onto the inner face"))
    n = max(1, int(np.ceil((F.PUSH_OPEN_TO_DEG - deg0) / F.PUSH_OPEN_ARC_STEP_DEG)))
    for k in range(1, n + 1):
        deg = deg0 + (F.PUSH_OPEN_TO_DEG - deg0) * k / n
        pts.append((df.pos(u_push, v_push, deg, z_in), hand_quat(df, deg, approach_deg, y_up), f"push to {deg:.0f} deg"))
    return pts, first_push


def plan_push_open(door, st):
    """Try the hand placements in order; the first whose whole route passes every gate (demo_replay.plan, from `st`).
    Returns (pts, first_push, placement) or (None, None, None)."""
    scene, rb = _sim()
    tried = 0
    for u_push in F.PUSH_OPEN_AT_M:
        for approach_deg in F.PUSH_OPEN_APPROACH_DEG:
            for y_up in (1.0, -1.0):
                tried += 1
                with contextlib.redirect_stdout(io.StringIO()):
                    pts, first_push = push_open_points(door, st, u_push, approach_deg, y_up, scene, rb)
                    legs = None if pts is None else demo_replay.plan(
                        scene, rb, door, st, [(pp, qq) for pp, qq, _ in pts[1:]], first_push, lift_off=True)
                if legs is not None:
                    return pts, first_push, (u_push, approach_deg, y_up, tried)
    return None, None, None


def _facing_fridge(df, y_up):
    """Approach along the closed door's inward normal (into the fridge), tool y up (+1, upright -- as the grasp) or
    down (-1)."""
    fwd = -np.r_[df.n, 0.0]
    y = np.array([0.0, 0.0, y_up])
    return R.from_matrix(np.column_stack([np.cross(y, fwd), y, fwd])).as_quat()


def release_around_points(door, p_from, quat_from, edge_clear, y_round):
    """Keypoints [(pos, quat, label)] from the hand after the back-off (`p_from`, `quat_from`) back round the open door
    to its inner side: along the door to `edge_clear` past its free edge (outside), round it turning to face the fridge
    (tool y up/down `y_round`: the hand may be either way up on the way), then in beside the door -- always ending
    UPRIGHT (tool y up, as the grasp), rolling on that last leg if it went round the edge upside down."""
    df = DoorFrame(door, door["open_sign"])
    deg = float(door["door_open_deg"])
    z = float(p_from[2])
    d = df._rot(deg).T @ (np.asarray(p_from[:2]) - df.hinge[:2])
    v_now = float(d @ df.n) - df.mid
    u_past = df.length + edge_clear
    return [(np.asarray(p_from[:3], float), np.asarray(quat_from, float), "after the back-off"),
            (df.pos(u_past, max(v_now, F.AROUND_OUT_M), deg, z), np.asarray(quat_from, float),
             f"back past the free edge (+{edge_clear * 100:.0f} cm)"),
            (df.pos(u_past, -F.AROUND_IN_M, deg, z), _facing_fridge(df, y_round), "round the edge, facing the fridge"),
            (df.pos(F.AROUND_END_M[0], -F.AROUND_END_M[1], deg, z), _facing_fridge(df, 1.0),
             "beside the door, inner side, upright")]


def plan_release_around(scene, rb, door, ee, q):
    """From the hand on the handle at the end of the full pull (`ee` [pos, quat], joints `q`): the release's back-off
    (planned exactly as release_and_back_off plans it) and then the route back round the door
    (release_around_points), every leg checked against the door model at its current angle. Returns (keypoints,
    message) or (None, message). Prints nothing."""
    ee = np.asarray(ee, float)
    approach = R.from_quat(ee[3:7]).as_matrix()[:, 2]
    with contextlib.redirect_stdout(io.StringIO()):
        back = plan_straight_line(scene, rb, ee[:3], ee[3:7], -approach, F.BACK_OFF_M, q, "back-off")
    if back is None:
        return None, f"the {F.BACK_OFF_M * 100:.0f} cm back-off off the handle fails a gate"
    p_bo = ee[:3] - approach * F.BACK_OFF_M
    df = DoorFrame(door, door["open_sign"])
    bodies = df.bodies(scene, rb, float(door["door_open_deg"]))
    try:
        for edge_clear in F.AROUND_EDGE_CLEAR_M:
            for y_round in (1.0, -1.0):
                pts = release_around_points(door, p_bo, ee[3:7], edge_clear, y_round)
                with contextlib.redirect_stdout(io.StringIO()):
                    legs, _ = _chain(scene, rb, [(pp, qq) for pp, qq, _ in pts], back[-1], bodies, "around")
                if legs is not None:
                    return pts, (f"release + back-off + round the door OK ({edge_clear * 100:.0f} cm past its free "
                                 f"edge; round it {'upright' if y_round > 0 else 'upside down, rolling upright after'}"
                                 f"; ends upright)")
    finally:
        drop(rb, bodies)
    return None, "the route back round the door fails a gate"


def push_open(ai, execute):
    """After the pull: let go and push the door further open. Planned and sim-gated from where the arm really is,
    before anything moves (a failed gate: nothing moves, the hand stays on the handle)."""
    print("\n===== fridge: release + push further open =====")
    if not execute:
        print("  (starts where the swing ends: planned and sim-gated then, before it moves)")
        return True
    door = json.loads(F.DOOR_FILE.read_text())
    st = ai.get_state()
    if float(st["gripper_pos"]) < 0.2:
        print("gripper is open -- the push-open starts from the end of the pull, still holding the handle. Refusing.")
        return False
    return _replay(ai, door, st, True)


def _replay(ai, door, st, execute):
    """Plan the push-open from `st` (the hand on the handle at the end of the pull) and, with `execute`, run it."""
    deg0 = float(door["door_open_deg"])
    if deg0 >= F.PUSH_OPEN_TO_DEG - 5:
        print(f"the door is already ~{deg0:.0f} deg open (push to {F.PUSH_OPEN_TO_DEG}) -- refusing.")
        return False
    pts, first_push, placed = plan_push_open(door, st)
    if pts is None:
        n = len(F.PUSH_OPEN_AT_M) * len(F.PUSH_OPEN_APPROACH_DEG) * 2
        print(f"no hand placement of {n} gives a push-open route that passes every gate -- refusing, arm untouched.")
        return False
    u_push, approach_deg, y_up, tried = placed
    print(f"door ~{deg0:.0f} deg; placement {tried}: hand {u_push * 100:.0f} cm from the hinge, fingers {approach_deg:+.0f} deg "
          f"off along-the-door, tool y {'up' if y_up > 0 else 'down'}; push to {F.PUSH_OPEN_TO_DEG} deg")
    for pp, _, label in pts:
        print(f"  {label[:30]:30s} -> {np.round(pp, 3)}")
    grippers = [float(st["gripper_pos"])] + [0.0] * (len(pts) - 1)   # let go first, open hand from there
    ok = demo_replay.replay([(pp, qq) for pp, qq, _ in pts], grippers, door, execute, first_push, ai=ai)
    if ok and execute:
        save_door_geometry(door_open_deg=float(F.PUSH_OPEN_TO_DEG), door_file=F.DOOR_FILE)   # the close's default
    return ok


def close_heights(ai, args, shift):
    """The swing height for the close: `args.push_z` if given (refused outside the door), else the first height within
    the door (CLOSE_Z_MARGIN_M in from its top and bottom, every CLOSE_Z_STEP_M, nearest the hand first) whose whole
    push_close plan passes every gate (planned quietly, nothing moves). Returns the height or None."""
    door = json.loads(F.DOOR_FILE.read_text())
    # the door's height from the measured constants (door files written before they were measured hold a guess)
    door_z = [float(door["closed_handle"][2]) - F.DOOR_BELOW_HANDLE_M, float(door["closed_handle"][2]) + F.DOOR_ABOVE_HANDLE_M]
    if not np.allclose(door.get("door_z") or [0.0, 0.0], door_z):
        save_door_geometry(door_z=door_z, door_file=F.DOOR_FILE)
    lo, hi = door_z[0] + F.CLOSE_Z_MARGIN_M, door_z[1] - F.CLOSE_Z_MARGIN_M
    print(f"door z {door_z[0]:.3f}-{door_z[1]:.3f}; the swing may run at z {lo:.3f}-{hi:.3f}")
    if args.push_z is not None:
        if not lo <= args.push_z <= hi:
            print(f"--push-z {args.push_z:.3f} is not within the door -- refusing, arm untouched.")
            return None
        return args.push_z
    z_now = float(ai.get_state()["ee_pos"][2])
    for z in sorted(np.arange(lo, hi + 1e-9, F.CLOSE_Z_STEP_M), key=lambda z: abs(z - z_now)):
        trial = argparse.Namespace(**{**vars(args), "push_z": float(z), "execute": False})
        with contextlib.redirect_stdout(io.StringIO()):
            ok = push_close(ai, trial, door_file=F.DOOR_FILE, door_shift=shift, behind=True, stop_deg=args.close_to_deg,
                            max_tilt_deg=F.CLOSE_MAX_TILT_DEG)
        print(f"  z {z:.3f}: {'passes' if ok else 'fails a gate'}")
        if ok:
            return float(z)
    print("no height within the door gives a plan that passes every gate -- refusing, arm untouched.")
    return None


def close(ai, args):
    """The wheelchair-back prompt, then `door_push.push_close` against the fridge's door file: shifted by the chair's
    move, routed in front of the free edge, swinging the door only to `args.close_to_deg`, at a height within the door
    (`close_heights`)."""
    enable_keep_out()
    shift = np.asarray(F.CHAIR_BACK_DIR, float) * args.chair_back
    print(f"fridge close: door from {F.DOOR_FILE}, planned {args.chair_back * 100:.0f} cm further back (the "
          f"wheelchair), swung to ~{args.close_to_deg:.0f} deg, hand held back {-args.hold_offset_x * 100:.1f} cm "
          "(container)")
    z = close_heights(ai, args, shift)
    if z is None:
        return False
    args.push_z = z
    print(f"swing height z {z:.3f}")
    if not args.execute:
        print("(dry run: no prompt; planned as if the wheelchair has already moved back)")
    elif args.chair_back > 0:
        try:
            ans = input(f"\nMove the WHEELCHAIR straight back ~{args.chair_back * 100:.0f} cm (away from the fridge, "
                        "without turning it). If it is still back from the last close, leave it.\n"
                        "Type y + Enter once it is there (anything else stops): ")
        except EOFError:
            ans = ""
        if ans.strip().lower() not in ("y", "yes"):
            print("not confirmed -- stopping, arm untouched.")
            return False
    ok = push_close(ai, args, door_file=F.DOOR_FILE, door_shift=shift, behind=True, stop_deg=args.close_to_deg,
                    max_tilt_deg=F.CLOSE_MAX_TILT_DEG)
    if ok and args.execute:
        print(f"door left at ~{args.close_to_deg:.0f} deg to swing shut. Move the wheelchair forward again before the "
              "next open (it detects the door afresh).")
    return ok


def main():
    a = argparse.ArgumentParser(description="the fridge push-open alone, from the end of a pull (still holding the "
                                            "handle)")
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run (plan + sim gates)")
    args = a.parse_args()
    enable_keep_out()
    from sheppy_arm import SheppyArm
    ai = SheppyArm()
    try:
        if args.execute:
            ok = push_open(ai, True)
        else:                      # a dry run here CAN plan: the arm is already at the end of the pull
            ok = _replay(ai, json.loads(F.DOOR_FILE.read_text()), ai.get_state(), False)
        sys.exit(0 if ok else 1)
    finally:
        ai.close()


if __name__ == "__main__":
    main()
