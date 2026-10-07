"""Open and close the microwave door, end to end. Dry run unless --execute.

    python3 -u microwave/door_task.py open  [--execute]   # detect, grasp, swing, release + push it further open
    python3 -u microwave/door_task.py close [--execute]   # over the open door, push it shut, retract to park
    python3 -u microwave/door_task.py full  [--execute]   # open, then close

* open  = `open_task` (detect -> tilt search -> grasp -> stiff swing of SWING_DEG, see
  real_gen3_ros2_grasp_and_swing_microwave.py), then the user's push-open demo replayed relative
  to the door (`demo_replay.py`, demos/push_open_2026-10-07.door.json).
* close = the user's close demo (starts where the push-open ends; the hand may hold a container
  -- the gripper is never touched), then RETRACT: straight back off the closed door and on to the
  park pose (park_pose.json), sim-checked against the closed-door model.
A dry run checks the open fully; the later steps start where the earlier ones end, so a dry run
only shows them mapped onto today's door -- a real run re-checks every step from where the arm is.
"""
import argparse, json, sys
from pathlib import Path

import numpy as np

import demo_replay
import real_gen3_ros2_grasp_and_swing_microwave as mw
from door_push import DoorFrame, drop
from microwave_common import DOOR_FILE, PARK_FILE, make_sim, plan_cartesian, run_cartesian_trajectory
from sheppy_arm import SheppyArm

HERE = Path(__file__).resolve().parent
PUSH_OPEN = HERE / "demos" / "push_open_2026-10-07.door.json"
CLOSE = HERE / "demos" / "push_close_2026-10-07.door.json"
SWING_DEG = 50               # the push-open demo starts from a ~50 deg swing
PUSH_TRUST_FROM = 3          # the 50-deg door model contradicts the user's real push-open poses from leg 3 on
CLOSE_TRUST_FROM = 1         # the door's angle after the push is not measured: trust the whole close demo
RETRACT_M = 0.15             # after the close: straight back off the door (along its normal) before parking


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
    print("\n===== OPEN: release + push further open =====")
    if not execute:
        show(PUSH_OPEN)
        print("  (starts at the end of the swing: checked on the real run)")
        return True
    return demo_replay.run(PUSH_OPEN, execute, PUSH_TRUST_FROM, ai=ai)


def do_close(ai, execute, standalone):
    print("\n===== CLOSE: over the door, push it shut =====")
    if not execute and not standalone:
        show(CLOSE)
        print("  (starts at the end of the push-open: checked on the real run)")
        return True
    if not demo_replay.run(CLOSE, execute, CLOSE_TRUST_FROM, ai=ai):
        return False
    print("\n===== CLOSE: retract =====")
    return retract(ai, execute) if execute else True


def main():
    a = argparse.ArgumentParser()
    a.add_argument("what", choices=["open", "close", "full", "retract"])
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run")
    args = a.parse_args()
    ai = SheppyArm()
    try:
        ok = True
        if args.what in ("open", "full"):
            ok = do_open(ai, args.execute)
        if args.what == "retract":
            ok = retract(ai, args.execute)
        if ok and args.what in ("close", "full"):
            ok = do_close(ai, args.execute, standalone=args.what == "close")
        print(f"\n{args.what.upper()} {'OK' if ok else 'STOPPED'}{'' if args.execute else ' (dry run)'}")
        sys.exit(0 if ok else 1)
    finally:
        ai.close()


if __name__ == "__main__":
    main()
