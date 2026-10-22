"""Open and close the microwave door, end to end. Dry run unless --execute.

    python3 -u microwave/door_task.py open  [--execute]   # detect, grasp, swing, release + push it further open
    python3 -u microwave/door_task.py close [--execute]   # over the open door, push it shut, retract to park
    python3 -u microwave/door_task.py full  [--execute]   # open, then close
    ... close|full [--close-wrist-side left|right]          # arm configuration for the close (default left)
    ... close|full [--push-offset]                          # push 10 cm further toward the handle (right10 demo)
    ... [--speed 0.3]                                       # scale every arm motion's speed (default 1.0)
    ... close|full [--max-push-trim 0.08]                   # end the push up to this short if it's out of reach
    ... close --skip-opening                                # test the close without the open (prompts; see below)

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
--skip-opening (close only): test the close without running the open. Joint move to the view pose (VIEW_JOINTS_DEG,
camera forward at the closed microwave), detect it and write ~/.microwave_door.json as the open does, then ask: open
the door by hand -> close the gripper? (y: closes it, the arm doesn't move) -> start the close? Only on that last yes
does the arm move again: straight to the close demo's start (sim-checked against the door at SKIP_OPEN_DOOR_DEG), then
the close as usual. Dry run: no motion, no prompts, the door file is not written -- detects from where the arm is
and checks the view pose -> close start move against that detection.
Every run (dry too) is logged to ~/microwave_runs/<time>_<what>/ (microwave_common.RunLog): run.json
(events, door used, every trajectory goal + result), arm.npz (joints/torques/tool pose at 25 Hz), and
RGB-D frames with real depth: at the start of the run, before the close (in `full`) and after it.
A dry run checks the open fully; the later steps start where the earlier ones end, so a dry run
only shows them mapped onto today's door -- a real run re-checks every step from where the arm is.
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np

import demo_replay
import sheppy_arm
import real_gen3_ros2_grasp_and_swing_microwave as mw
from door_push import DoorFrame, drop
from microwave_common import (DOOR_FILE, PARK_FILE, RunLog, make_sim, plan_cartesian, run_cartesian_trajectory,
                              save_door_geometry)
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
# --skip-opening: Kinova home (EE 0.577, 0.001, 0.434) moved 15 cm back, 20 cm right and 13 cm down, camera tilted
# 15 deg down -- where the 10-21 opens were started by hand (all detected). 20 cm back puts J4 at its 144 guard; at
# home height the level camera looks over this low microwave.
VIEW_JOINTS_DEG = (43.5, 21.1, -167.5, -141.2, 60.2, 69.0, 71.5)
VIEW_EE = (0.427, -0.199, 0.304)
VIEW_QUAT = (0.5607, 0.5611, 0.4299, 0.4313)
VIEW_TOL = 0.02              # the arm must end this close to VIEW_EE before detecting
SKIP_OPEN_DOOR_DEG = 90.0    # door angle written to the door file (opened by hand: ~90, never more than 110)


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


def ask(question):
    return input(f"\n>>> {question} [y/N] ").strip().lower() == "y"


def to_close_start(ai, door, close, ee, q, execute):
    """Straight from the hand pose `ee` (joints `q`) to the close demo's first keypoint, sim-checked against the
    microwave body and the door at SKIP_OPEN_DOOR_DEG."""
    p0, q0 = demo_replay.decode(json.loads(close.read_text()), door)[0]
    scene, rb = make_sim()
    bodies = DoorFrame(door, door.get("open_sign", -1.0)).bodies(scene, rb, SKIP_OPEN_DOOR_DEG)
    try:
        leg = plan_cartesian(scene, rb, ee[:3], ee[3:7], p0, q0, q, "to the close start", bodies)
    finally:
        drop(rb, bodies)
    if leg is None:
        print("the move to the close start fails a gate -- arm left where it is.")
        return False
    print(f"to the close start {np.round(p0, 3)} -- passes")
    if not execute:
        return True
    ok, err, _ = run_cartesian_trajectory(ai, [(list(p0), list(q0))])
    print(f"{'AT THE CLOSE START' if ok else 'STOPPED SHORT'} ({err * 100:.1f} cm from it)")
    return bool(ok)


def skip_opening(ai, execute, log, close):
    """--skip-opening: view pose -> detect -> door file -> the user opens the door -> gripper? -> start? -> the
    close demo's start. Returns True when the arm is at the close start (or the dry run passed)."""
    print("\n===== SKIP OPENING: view pose + detect =====")
    view = np.radians(VIEW_JOINTS_DEG)
    if execute:
        print(f"joint move to the view pose J {VIEW_JOINTS_DEG} (EE {VIEW_EE})")
        ai.move_joints([view])
        time.sleep(0.3)
        off = float(np.linalg.norm(np.asarray(ai.get_state()["ee_pos"][:3]) - VIEW_EE))
        if off > VIEW_TOL:
            print(f"ended {off * 100:.1f} cm from the view pose -- stopping.")
            return False
        log.frame("view")
    else:
        print(f"DRY RUN: would joint-move to the view pose J {VIEW_JOINTS_DEG} (EE {VIEW_EE}); detecting from here")

    # the closed door, as the open's grasp + swing record it
    h_raw, orient, n, door_z, door_mid, hinge_seen = mw.detect_handle()
    hinge = mw.hinge_pivot(hinge_seen, h_raw, n)
    h, grasp = mw.grasp_pose(h_raw, orient, n)
    radius = float(np.linalg.norm(np.asarray(grasp.position)[:2] - hinge[:2]))
    if not mw.HINGE_RADIUS_RANGE[0] <= radius <= mw.HINGE_RADIUS_RANGE[1]:
        print(f"hinge radius {radius * 100:.1f} cm is implausible (door ~35 cm) -- refusing.")
        return False
    if door_z is None:
        print("no door top/bottom in the detection (the close needs the door top) -- refusing.")
        return False
    door = {k: np.asarray(v).tolist() for k, v in dict(
        closed_normal=n, closed_grasp_pos=grasp.position, closed_handle=h, door_z=door_z, door_mid=door_mid,
        hinge=hinge_seen, hinge_pivot=hinge, door_open_deg=SKIP_OPEN_DOOR_DEG).items()}
    log.event("skip_opening_detect", door=door, executed=execute)

    if not execute:
        print(f"door (not written to {DOOR_FILE.name} in a dry run): hinge {np.round(hinge_seen, 3)}, "
              f"handle {np.round(h, 3)}, top z {door_z[1]:.3f}")
        return to_close_start(ai, door, close, np.r_[VIEW_EE, VIEW_QUAT], view, False)

    DOOR_FILE.unlink(missing_ok=True)       # a fresh file, no hinge fit / angle left from an earlier run
    save_door_geometry(**door)
    print(f"door file written: {DOOR_FILE}")

    input(f"\n>>> Open the microwave door by hand (~{SKIP_OPEN_DOOR_DEG:.0f} deg, never more than 110), "
          "then press Enter")
    if ask("Close the gripper? (the arm does not move)"):
        ai.gripper(1.0)
        print(f"gripper {float(ai.get_state()['gripper_pos']):.2f}")
        log.event("skip_opening_gripper", closed=True, gripper=float(ai.get_state()["gripper_pos"]))
    if not ask("Start the closing sequence?"):
        print("not starting the close -- arm stays at the view pose.")
        log.event("skip_opening_declined")
        return False
    st = ai.get_state()
    ok = to_close_start(ai, door, close, np.asarray(st["ee_pos"], float), np.asarray(st["position"], float), True)
    log.event("to_close_start", ok=ok, hand=np.asarray(ai.get_state()["ee_pos"][:3]))
    return ok


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
    a.add_argument("--skip-opening", action="store_true",
                   help="close only: view pose, detect, then prompts (open the door by hand, close the gripper?, "
                        "start?) instead of running the open first")
    args = a.parse_args()
    if not 0.0 < args.speed <= 1.0:
        sys.exit("--speed must be in (0, 1]")
    if args.skip_opening and args.what != "close":
        sys.exit("--skip-opening goes with `close`")
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
        if args.skip_opening:
            ok = skip_opening(ai, args.execute, log, CLOSE_PUSH_OFFSET if args.push_offset else CLOSE)
            if not args.execute:          # the close itself starts at the close start: checked on the real run
                print("  (the close is planned from the close start on the real run)")
        if ok and args.what in ("close", "full") and (args.execute or not args.skip_opening):
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
