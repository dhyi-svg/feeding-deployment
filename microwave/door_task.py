"""Open and close the microwave door, end to end. Dry run unless --execute.

    python3 -u microwave/door_task.py open  [--execute]   # detect, grasp, swing, release + push it further open
    python3 -u microwave/door_task.py close [--execute]   # over the open door, push it shut, retract to park
    python3 -u microwave/door_task.py full  [--execute]   # open, then close
    ... close|full [--close-wrist-side left|right]          # arm configuration for the close (default left)
    ... close|full [--push-offset]                          # push 10 cm further toward the handle (right10 demo)
    ... [--speed 0.3]                                       # scale every arm motion's speed (default 1.0)
    ... close|full [--max-push-trim 0.08]                   # end the push up to this short if it's out of reach

* open  = `open_task` (detect -> tilt search -> grasp -> stiff swing of SWING_DEG, see
  real_gen3_ros2_grasp_and_swing_microwave.py), then the user's push-open demo replayed relative
  to the door (`demo_replay.py`, demos/push_open_2026-10-07.door.json).
* close = the user's close demo (starts where the push-open ends; the hand may hold a container
  -- the gripper is never touched), then RETRACT: straight back off the closed door and on to the
  park pose (park_pose.json), sim-checked against the closed-door model.
The close replays against the open task's closed-door detection (~/.microwave_door.json).
--close-wrist-side: `left` (default) = the close as run on 10-07: IK from the arm's current joints, which keeps the
elbow put, so the wrist ends left of it once the hand moves left. `right` = a searched joint path that keeps the
wrist right of the elbow (demo_replay --wrist-side right).
--push-offset: the close demo with its push (kp5, END) 10 cm further along the door toward the handle
(demos/push_close_2026-10-07_right10.door.json); without it the original 10-07 encoding.
Every run (dry too) is logged to ~/microwave_runs/<time>_<what>/ (microwave_common.RunLog): run.json
(events, door used, every trajectory goal + result), arm.npz (joints/torques/tool pose at 25 Hz), and
RGB-D frames with real depth: at the start of the run, before the close (in `full`) and after it.
A dry run checks the open fully; the later steps start where the earlier ones end, so a dry run
only shows them mapped onto today's door -- a real run re-checks every step from where the arm is.
"""
import argparse, json, sys
from pathlib import Path

import numpy as np

import demo_replay
import sheppy_arm
import real_gen3_ros2_grasp_and_swing_microwave as mw
from door_push import DoorFrame, drop
from microwave_common import DOOR_FILE, PARK_FILE, RunLog, make_sim, plan_cartesian, run_cartesian_trajectory
from sheppy_arm import SheppyArm

HERE = Path(__file__).resolve().parent
PUSH_OPEN = mw.PUSH_OPEN
CLOSE = HERE / "demos" / "push_close_2026-10-07.door.json"
CLOSE_PUSH_OFFSET = HERE / "demos" / "push_close_2026-10-07_right10.door.json"   # --push-offset: push (kp5, END) 10 cm
                                                                                 # further toward the handle
SWING_DEG = mw.PUSH_OPEN_SWING_DEG
CLOSE_TRUST_FROM = 1         # the door's angle after the push is not measured: trust the whole close demo
RETRACT_M = 0.15             # after the close: straight back off the door (along its normal) before parking
CLOSE_WRIST_SIDE = "left"    # the close as run on 10-07 (plain IK); "right" = keep the wrist right of the elbow
CLOSE_MAX_PUSH_TRIM = 0.08   # the push's end (door-relative) can land out of reach when the microwave sits further out
                             # (10-21: hinge x 0.867 vs 0.826 on 10-07 -> end 0.94 m from the base): pull it back along
                             # the push by up to this much. The door may then not latch -- watch it.


def show(enc_path):
    door = json.loads(DOOR_FILE.read_text())
    enc = json.loads(Path(enc_path).read_text())
    print(f"  {enc['source']}: mapped onto today's door")
    for k, (p, _) in zip(enc["keypoints"], demo_replay.decode(enc, door)):
        print(f"    {k['label'][:30]:30s} -> {np.round(p, 3)}")


def retract(ai, execute):
    """Back off the closed door along its normal, then straight to the park pose -- gripper untouched."""
    door = json.loads(DOOR_FILE.read_text())
    st = ai.get_state()
    ee, q0 = np.asarray(st["ee_pos"], float), np.asarray(st["position"], float)
    n = np.r_[np.asarray(door["closed_normal"][:2], float), 0.0]
    n /= np.linalg.norm(n)
    back = ee[:3] + n * RETRACT_M
    park = json.loads(PARK_FILE.read_text())
    park_p, park_q = np.asarray(park["ee_pos"], float), np.asarray(park["quat"], float)
    scene, rb = make_sim()
    bodies = DoorFrame(door, door.get("open_sign", -1.0)).bodies(scene, rb, 0.0)
    try:
        # the hand ends AT the door face (it pushed it shut): the back-off only moves away from it -> body only
        leg1 = plan_cartesian(scene, rb, ee[:3], ee[3:7], back, ee[3:7], q0, "back off",
                              [b for b in bodies if b[1] == "microwave body"])
        leg2 = None if leg1 is None else plan_cartesian(scene, rb, back, ee[3:7], park_p, park_q, leg1[-1], "to park", bodies)
    finally:
        drop(rb, bodies)
    if leg2 is None:
        print("retract fails a gate -- arm left where it is.")
        return False
    print(f"retract: back {RETRACT_M * 100:.0f} cm to {np.round(back, 3)}, then park {np.round(park_p, 3)} -- passes")
    if not execute:
        return True
    ok, err, _ = run_cartesian_trajectory(ai, [(list(back), list(ee[3:7])), (list(park_p), list(park_q))])
    print(f"RETRACT {'DONE' if ok else 'STOPPED SHORT'} ({err * 100:.1f} cm from park)")
    return bool(ok)


def do_open(ai, execute):
    print("\n===== OPEN: grasp + swing =====")
    if not mw.open_task(ai, execute, SWING_DEG):
        print("swing did not finish -- stopping (still holding the handle).")
        return False
    if not execute:
        print("\n===== OPEN: release + push further open =====")
        show(PUSH_OPEN)
    return mw.push_open(ai, execute)


def do_close(ai, execute, standalone, log=None, wrist_side=CLOSE_WRIST_SIDE, push_offset=False,
             max_push_trim=CLOSE_MAX_PUSH_TRIM):
    close = CLOSE_PUSH_OFFSET if push_offset else CLOSE
    print(f"\n===== CLOSE: over the door, push it shut ({close.name}, wrist {wrist_side} of the elbow) =====")
    if not execute and not standalone:
        show(close)
        print("  (starts at the end of the push-open: checked on the real run)")
        return True
    if log and not standalone:
        log.frame("close_start")              # (standalone: the run's start frame is this one)
    door = json.loads(DOOR_FILE.read_text())
    kps = demo_replay.decode(json.loads(close.read_text()), door)
    if log:
        log.event("close_plan", demo=close.name, door=door, trust_from=CLOSE_TRUST_FROM, wrist_side=wrist_side,
                  keypoints=[list(p) for p, _ in kps], quats=[list(q) for _, q in kps])
    ok = demo_replay.run(close, execute, CLOSE_TRUST_FROM, ai=ai,
                         wrist_side=None if wrist_side == "left" else wrist_side, max_end_trim=max_push_trim)
    if log:
        hand = np.asarray(ai.get_state()["ee_pos"][:3])
        log.event("close_replay", ok=ok, executed=execute, hand=hand,
                  to_last_keypoint_m=float(np.linalg.norm(hand - kps[-1][0])))
    if ok and execute:
        print("\n===== CLOSE: retract =====")
        ok = retract(ai, execute)
        if log:
            log.event("retract", ok=ok, hand=np.asarray(ai.get_state()["ee_pos"][:3]))
    if log:
        log.frame("close_end")
    return ok


def main():
    a = argparse.ArgumentParser()
    a.add_argument("what", choices=["open", "close", "full", "retract"])
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run")
    a.add_argument("--close-wrist-side", choices=["left", "right"], default=CLOSE_WRIST_SIDE,
                   help="left (default) = the close as run on 10-07 (plain IK, wrist ends left of the elbow); "
                        "right = keep the wrist right of the elbow")
    a.add_argument("--push-offset", action="store_true",
                   help="push 10 cm further along the door toward the handle (demos/push_close_2026-10-07_right10)")
    a.add_argument("--max-push-trim", type=float, default=CLOSE_MAX_PUSH_TRIM, metavar="M",
                   help=f"close: if the push's end is out of reach, end it up to M metres short (default "
                        f"{CLOSE_MAX_PUSH_TRIM}; 0 = refuse instead)")
    a.add_argument("--speed", type=float, default=1.0,
                   help="scale the arm's speed for every motion (0 < s <= 1; e.g. 0.3 for a first test)")
    args = a.parse_args()
    if not 0.0 < args.speed <= 1.0:
        sys.exit("--speed must be in (0, 1]")
    # SheppyArm times every trajectory from these at call time (Cartesian m/s, joint rad/s peaks)
    sheppy_arm.CART_SPEED *= args.speed
    sheppy_arm.JOINT_SPEED *= args.speed
    ai = SheppyArm()
    log = RunLog(args.what, vars(args), ai)
    ok = False
    try:
        log.frame("run_start")
        ok = True
        if args.what in ("open", "full"):
            ok = do_open(ai, args.execute)
            log.event("open", ok=ok)
        if args.what == "retract":
            ok = retract(ai, args.execute)
            log.event("retract", ok=ok)
        if ok and args.what in ("close", "full"):
            ok = do_close(ai, args.execute, standalone=args.what == "close", log=log,
                          wrist_side=args.close_wrist_side, push_offset=args.push_offset,
                          max_push_trim=args.max_push_trim)
        print(f"\n{args.what.upper()} {'OK' if ok else 'STOPPED'}{'' if args.execute else ' (dry run)'}")
        sys.exit(0 if ok else 1)
    except BaseException as e:               # incl. the open task's sys.exit refusals and Ctrl-C
        if not (isinstance(e, SystemExit) and isinstance(e.code, int)):    # not our own exit(0/1) above
            ok = False
            log.event("stopped", error=repr(e))
        raise
    finally:
        log.close(ok)
        ai.close()


if __name__ == "__main__":
    main()
