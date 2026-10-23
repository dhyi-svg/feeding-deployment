"""The fridge's push-open and push-close: the microwave's, plus what a tall fridge door needs.

    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --fridge --push-open [--execute]   # open
    python3 -u microwave/real_gen3_ros2_close_microwave.py --phase push-close --fridge [--execute]       # close
    python3 -u microwave/fridge_push.py [--execute]   # just the push-open, from the end of a pull (still holding)

* `push_open` -- after a PUSH_OPEN_PULL_DEG pull (still holding the handle), the microwave's push-open: let go
  and replay the user's push-open demo relative to the door, as ONE smooth sim-gated trajectory (`demo_replay`,
  the same decoding, planning and execution as the microwave's). The microwave's demo goes UP and over the door
  first; a fridge door is too tall, so that part is replaced by a route round the door's free edge at hand height,
  computed for today's door: straight out off the handle, along the open door to past its free edge, round it
  (in the door's plane), then the demo's own last two keypoints -- into the gap behind the door, and the push.
* `close` -- the microwave's push-close (`door_push.push_close`: swing about the hinge, hand orientation fixed) with
  three changes: (1) the user first moves the wheelchair back ~CHAIR_BACK_M (prompted; the recorded fridge is
  shifted by that much for the plan), (2) the route to the swing start is back out in front of the open door's
  free edge, left behind it, then forward -- no going over the door, (3) the swing stops when the door is at
  CLOSE_TO_DEG (the fridge door swings shut by itself from there): no torque push.

Both use the fridge's door file (fridge_constants.DOOR_FILE, written by the `--fridge` open), never the microwave's.
"""
import argparse, json, sys

import numpy as np
from scipy.spatial.transform import Rotation as R

import demo_replay
import fridge_constants as F
from door_push import DoorFrame, push_close
from microwave_common import door_dims


def demo_door(door):
    """`door` with its hinge moved forward onto the closed door's face plane -- the door frame the microwave demo was
    recorded in (the microwave's door file holds the door's detected edge; the fridge's holds the pivot behind it)."""
    d = dict(door)
    n = np.asarray(door["closed_normal"][:2], float)
    n /= np.linalg.norm(n)
    face = np.asarray(door["closed_grasp_pos"][:2], float) - n * door_dims(door)[0]
    h = np.asarray(door["hinge"], float)
    d["hinge"] = [*(h[:2] - n * float((h[:2] - face) @ n)), h[2]]
    return d


def push_open_points(door, st):
    """Keypoints [(pos, quat, label)] from the hand on the handle (`st`, the end of the pull) to the end of the push,
    and the index of the push leg (checked against the cabinet only: it touches the door on purpose)."""
    df = DoorFrame(door, door["open_sign"])
    deg0 = float(door["door_open_deg"])
    ee = np.asarray(st["ee_pos"], float)
    z, q_hold = float(ee[2]), ee[3:7]
    d = df._rot(deg0).T @ (ee[:2] - df.hinge[:2])               # the hand in the open door's frame (u along, v off it)
    u_hold, v_out = float(d @ df.e), float(d @ df.n) - df.mid + F.PUSH_OPEN_OFF_HANDLE_M
    u_edge = df.length + F.PUSH_OPEN_EDGE_CLEAR_M
    enc = json.loads(F.PUSH_OPEN_DEMO.read_text())
    enc["keypoints"] = enc["keypoints"][F.PUSH_OPEN_DEMO_FROM:]
    demo = demo_replay.decode(enc, demo_door(door))
    print(f"door ~{deg0:.0f} deg; hand {u_hold * 100:.0f} cm along it from the hinge; free edge at "
          f"{df.length * 100:.0f} cm; then {F.PUSH_OPEN_DEMO.name} from keypoint {F.PUSH_OPEN_DEMO_FROM}")
    pts = [(ee[:3], q_hold, "end of the pull, holding"),
           (df.pos(u_hold, v_out, deg0, z), q_hold, "off the handle"),
           (df.pos(u_edge, v_out, deg0, z), q_hold, "past the free edge"),
           (df.pos(u_edge, 0.0, deg0, z), q_hold, "round the edge")]
    pts += [(pp, qq, f"demo {k['label']}") for (pp, qq), k in zip(demo, enc["keypoints"])]
    return pts, len(pts) - 1                                      # leg i goes to pts[i]: the last one is the push


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
    pts, push_leg = push_open_points(door, st)
    for pp, _, label in pts:
        print(f"  {label[:30]:30s} -> {np.round(pp, 3)}")
    grippers = [float(st["gripper_pos"])] + [0.0] * (len(pts) - 1)   # let go first, open hand from there
    ok = demo_replay.replay([(pp, qq) for pp, qq, _ in pts], grippers, door, execute, push_leg, ai=ai)
    if ok and execute:
        print("the door's angle after the push isn't measured: give the close --door-deg")
    return ok


def close(ai, args):
    """The wheelchair-back prompt, then `door_push.push_close` against the fridge's door file: shifted by the chair's
    move, routed in front of the free edge, swinging the door only to `args.close_to_deg`."""
    shift = np.asarray(F.CHAIR_BACK_DIR, float) * args.chair_back
    print(f"fridge close: door from {F.DOOR_FILE}, planned {args.chair_back * 100:.0f} cm further back (the "
          f"wheelchair), swung to ~{args.close_to_deg:.0f} deg")
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
    ok = push_close(ai, args, door_file=F.DOOR_FILE, door_shift=shift, behind=True, stop_deg=args.close_to_deg)
    if ok and args.execute:
        print(f"door left at ~{args.close_to_deg:.0f} deg to swing shut. Move the wheelchair forward again before the "
              "next open (it detects the door afresh).")
    return ok


def main():
    a = argparse.ArgumentParser(description="the fridge push-open alone, from the end of a pull (still holding the "
                                            "handle)")
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run (plan + sim gates)")
    args = a.parse_args()
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
