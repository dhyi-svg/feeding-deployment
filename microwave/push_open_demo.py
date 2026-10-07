"""Push the door further open after the swing, along the user's hand demo (10-07). Dry run unless --execute.

    python3 -u microwave/push_open_demo.py            # plan + sim-check, print the mapped keypoints
    python3 -u microwave/push_open_demo.py --execute  # open the gripper, follow the demo, end at its last keypoint

Starts where the demo started: the end of a ~50 deg swing, still holding the handle. The demo
(`demos/push_open_2026-10-07.json`, keypoints logged with `tools/kp.sh`) is replayed relative to
the door: each keypoint is moved and turned with the door (the detected hinge and the closed
door's facing direction in `demos/..._door.json` vs `~/.microwave_door.json` now), heights shift
with the grasp height, and the over-the-door points stay >= OVER_TOP_MIN above the door top.
Every leg is sim-gated (IK, joint jump, J4/J6, wrap) with >= 2 cm to the door model at its current
angle; the last leg is the push, so it is checked against the microwave body only.
"""
import argparse, json, sys, time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

from sheppy_arm import SheppyArm
from door_push import DoorFrame, drop
from microwave_common import DOOR_FILE, make_sim, plan_cartesian, run_cartesian_trajectory

HERE = Path(__file__).resolve().parent
DEMO = HERE / "demos" / "push_open_2026-10-07.json"
DEMO_DOOR = HERE / "demos" / "push_open_2026-10-07_door.json"
OVER_TOP_MIN = 0.08          # over-the-door keypoints at least this far above the detected door top
SIDE_MARGIN = {2: 0.03, 3: 0.03}   # keypoint index -> extra metres away from the hinge side (the descent beside the
                             # microwave passed the door top at 1.6 cm when mapped onto the 10-07 afternoon door)
START_TOL = 0.06             # the hand must be this close to the mapped first keypoint (end of the swing)
TRUST_DEMO_FROM = None       # --trust-demo: from this leg on, check against the microwave body only (set in main)
PUSH_LEGS = 1                # the last N legs touch the door on purpose: checked against the body only
DEMO_MIN_CLEAR = 0.02        # the user's demonstrated route: 2 cm to the door model (it already pads the top by 2 cm);
                             # the descent beside the microwave passes the door top at 2.8 cm (10-07)


def door_map(demo_door, door):
    """Rigid map demo -> now: rotate about z by the change in the closed door's facing, carry the
    detected hinge onto today's, shift heights by the change in grasp height."""
    yaw = lambda n: np.arctan2(n[1], n[0])
    dyaw = yaw(door["closed_normal"]) - yaw(demo_door["closed_normal"])
    rot = R.from_euler("z", dyaw)
    h0, h1 = np.asarray(demo_door["hinge"][:2]), np.asarray(door["hinge"][:2])
    dz = float(door["closed_grasp_pos"][2]) - float(demo_door["closed_grasp_pos"][2])
    top = float(door["door_z"][1])
    demo_top = float(demo_door["door_z"][1])

    def f(pos, quat):
        p = np.asarray(pos, float)
        xy = rot.as_matrix()[:2, :2] @ (p[:2] - h0) + h1
        z = p[2] + dz
        if p[2] > demo_top:                      # an over-the-door point: keep it over today's door
            z = max(z, top + OVER_TOP_MIN)
        return np.r_[xy, z], (rot * R.from_quat(quat)).as_quat()
    return f, float(np.degrees(dyaw)), dz


def plan(scene, rb, door, st, kps):
    """Chain plan_cartesian through the keypoints from the arm's real pose; returns the legs or None."""
    sign = door.get("open_sign", -1.0)
    df = DoorFrame(door, sign)
    deg = float(door.get("door_open_deg", 50.0))
    bodies = df.bodies(scene, rb, deg)
    q = np.asarray(st["position"], float)
    legs = []
    try:
        pts = [(np.asarray(st["ee_pos"][:3]), np.asarray(st["ee_pos"][3:7]))] + kps
        n_legs = len(pts) - 1
        for i in range(n_legs):
            push = i >= n_legs - PUSH_LEGS or (TRUST_DEMO_FROM is not None and i + 1 >= TRUST_DEMO_FROM)
            check = [b for b in bodies if b[1] == "microwave body"] if push else bodies
            leg = plan_cartesian(scene, rb, pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], q,
                                 f"{'push' if push else 'leg'} {i + 1}", check, min_clear=DEMO_MIN_CLEAR)
            if leg is None:
                return None
            legs.append(leg)
            q = leg[-1]
    finally:
        drop(rb, bodies)
    return legs


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run")
    a.add_argument("--trust-demo", type=int, default=None, metavar="LEG",
                   help="from leg LEG on, skip the door-model clearance (body only): the user's REAL demo poses sit "
                        "2-7 cm inside the 50-deg door model from keypoint 3 on (10-07), i.e. the model is wrong there")
    args = a.parse_args()
    global TRUST_DEMO_FROM
    TRUST_DEMO_FROM = args.trust_demo

    demo = json.loads(DEMO.read_text())["keypoints"]
    demo_door, door = json.loads(DEMO_DOOR.read_text()), json.loads(DOOR_FILE.read_text())
    f, dyaw, dz = door_map(demo_door, door)
    mapped = [f(k["tool"], k["quat"]) for k in demo]
    away = np.asarray(door["closed_handle"][:2]) - np.asarray(door["hinge"][:2])
    away = np.r_[away / np.linalg.norm(away), 0.0]          # along the closed door, hinge -> handle side
    mapped = [(p + away * SIDE_MARGIN.get(i, 0.0), q) for i, (p, q) in enumerate(mapped)]
    print(f"demo -> now: door turned {dyaw:+.1f} deg, hinge {np.round(demo_door['hinge'][:2], 3)} -> "
          f"{np.round(door['hinge'][:2], 3)}, heights {dz * 100:+.1f} cm; door now ~{door.get('door_open_deg', 0):.0f} deg")
    for k, (p, _) in zip(demo, mapped):
        print(f"  {k['label'][:28]:28s} demo {np.round(k['tool'], 3)} -> {np.round(p, 3)}")

    ai = SheppyArm()
    try:
        st = ai.get_state()
        off = float(np.linalg.norm(np.asarray(st["ee_pos"][:3]) - mapped[0][0]))
        print(f"hand now {np.round(st['ee_pos'][:3], 3)}, {off * 100:.1f} cm from the demo's start; gripper {st['gripper_pos']:.2f}")
        if off > START_TOL:
            sys.exit(f"not at the end of the swing (> {START_TOL * 100:.0f} cm from it) -- refusing.")
        scene, rb = make_sim()
        legs = plan(scene, rb, door, st, [(p, q) for p, q in mapped[1:]])
        if legs is None:
            sys.exit("a leg fails a gate -- refusing, arm untouched.")
        print(f"all {len(legs)} legs pass ({sum(len(l) for l in legs)} steps).")
        if not args.execute:
            print("\nDRY RUN -- would open the gripper and follow the demo. Nothing commanded.")
            return
        print("opening the gripper ...")
        ai.gripper(0.0)
        time.sleep(0.5)
        ok, err, _ = run_cartesian_trajectory(ai, [(list(p), list(q)) for p, q in mapped[1:]])
        print(f"PUSH-OPEN {'DONE' if ok else 'STOPPED SHORT'}: hand {np.round(ai.get_state()['ee_pos'][:3], 3)} "
              f"({err * 100:.1f} cm from the last keypoint)")
    finally:
        ai.close()


if __name__ == "__main__":
    main()
