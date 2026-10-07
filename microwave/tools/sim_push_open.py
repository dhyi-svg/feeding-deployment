"""Sim-only: would push-open work after the pull, for a given detection? No arm, no camera.

    python3 microwave/tools/sim_push_open.py                 # today's (10-07) detection + view pose
    python3 microwave/tools/sim_push_open.py --swing-deg 25  # force the pull angle

Plans exactly what the open script would (grasp_pose -> plan_cartesian from the view joints ->
largest swing passing _check_arc), takes the swing's END joints as the post-pull state, builds the
door model from the same numbers the grasp would save to the door file, and runs every
door_push push-open planner (arc with hand roll 0/180, axes, demo) on it.
"""
import argparse, contextlib, io, sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # microwave/
import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

import door_push as dp
import real_gen3_ros2_grasp_and_swing_microwave as mw
from handle_detect import HANDLE_QUAT
from microwave_common import make_sim, plan_cartesian

# 10-07 dry run (Jetson + sheppy): view pose joints (deg) and the mean detection
# (second placement, the one the first sheppy open ran from: grasp tilt 25, swing 75 worked on hardware)
VIEW_Q_DEG = [-17.2, -26.8, -106.7, -130.5, 123.4, 43.4, 15.7]
HANDLE = [0.7392, -0.2534, 0.1391]
NORMAL = [-0.998, -0.07, 0.0]
HINGE = [0.7517, 0.0811, 0.1339]
DOOR_Z = [0.039, 0.326]
DOOR_MID = None                     # not printed by the dry run; only retreat-over/close read it


def fk_tool(scene, rb, q):
    for j, jj in enumerate(mw.ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, computeForwardKinematics=True,
                        physicsClientId=rb.physics_client_id)
    t = multiply_poses(Pose(scene.robot_base_pose.position, scene.robot_base_pose.orientation).invert(),
                       Pose(tuple(ls[4]), tuple(ls[5])))
    return np.r_[t.position, t.orientation]


def main():
    a = argparse.ArgumentParser()
    a.add_argument("--swing-deg", type=int, default=None, help="pull angle (default: largest passing, 75 down to 10)")
    a.add_argument("--tilt", type=float, default=25.0, help="grasp nose-down tilt (deg), as the open script picked")
    a.add_argument("--quiet", action="store_true", help="hide the planners' per-step output")
    dp.add_push_args(a)
    args = a.parse_args()
    args.execute = False

    scene, rb = make_sim()
    q_view = np.radians(VIEW_Q_DEG)
    ee_view = fk_tool(scene, rb, q_view)
    h, n, hinge = np.array(HANDLE), np.array(NORMAL) / np.linalg.norm(NORMAL), np.array(HINGE)
    with contextlib.redirect_stdout(io.StringIO()):
        h_corr, grasp = mw.grasp_pose(h, HANDLE_QUAT, n, args.tilt)
        leg = plan_cartesian(scene, rb, ee_view[:3], ee_view[3:], np.array(grasp.position),
                             np.array(grasp.orientation), q_view, "to grasp")
    if leg is None:
        sys.exit("grasp path fails a gate")
    degs = [args.swing_deg] if args.swing_deg else list(range(75, 9, -5))
    for deg in degs:
        ok, q_end, msg = mw._check_arc(scene, rb, mw._swing_waypoints(grasp, hinge, deg), leg[-1])
        if ok:
            break
    else:
        sys.exit(f"no swing passes ({msg})")
    print(f"pull: {deg} deg ({msg}); end joints {np.round(np.degrees(q_end), 1)}")

    door = {"closed_normal": n.tolist(), "closed_grasp_pos": list(grasp.position), "closed_handle": h_corr.tolist(),
            "door_z": DOOR_Z, "door_mid": DOOR_MID, "hinge": hinge.tolist()}
    st = {"ee_pos": fk_tool(scene, rb, q_end), "position": q_end, "gripper_pos": 1.0}
    ang = dp.DoorFrame(door, 1.0).angle_of(st["ee_pos"])
    df = dp.DoorFrame(door, np.sign(ang) or 1.0)
    deg0 = abs(ang)
    print(f"door ~{deg0:.1f} deg open at the end of the pull (sign {df.sign:+.0f})")

    tries = [("arc, roll 0", lambda: dp._plan_open(scene, rb, df, st, args, deg0, 0.0)),
             ("arc, roll 180", lambda: dp._plan_open(scene, rb, df, st, args, deg0, 180.0)),
             ("axes", lambda: dp._plan_open_axes(scene, rb, df, st, args, deg0)),
             ("demo", lambda: dp._plan_open_demo(scene, rb, df, st, args, deg0))]
    for name, fn in tries:
        print(f"\n===== push-open: {name} =====")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf if args.quiet else sys.stdout):
            plan = fn()
        if args.quiet:
            lines = [l for l in buf.getvalue().splitlines() if l.strip()]
            print("\n".join(lines[-6:]))
        print(f"-> {name}: {'PASSES' if plan else 'fails'}" + (f" (door ~{plan['deg1']:.0f} deg)" if plan and plan.get('deg1') else ""))


if __name__ == "__main__":
    main()
