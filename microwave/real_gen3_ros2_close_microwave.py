"""Close the microwave door, adapted from the other lab's real `CloseDoorHLA.close_microwave()`
(`src/feeding_deployment/actions/close_door.py` + `perceive_handle_closing_poses`) to this rig
and this session's validated values, kept as simple as the two-phase shape they proved out
lets it be.

Their version and why it can't be used as-is here:
* `perceive_handle_closing_poses()` doesn't re-detect anything -- it just reloads the pickle
  `perceive_handle_opening_poses()` wrote out when the door was OPENED earlier in the same
  `log_dir`. We have no such cache wired up (our whole session has been live re-detection with
  the YOLO shim), so this script does a small LIVE push instead of replaying cached geometry.
* Their push/pull phases run inside `collision_threshold(...)` -- a REAL torque-based safety
  check via a ROS 1 `/set_collision_threshold` service (`feeding_deployment/safety/
  collision_threshold.py`). That needs `rospy`, which this ROS 2 rig doesn't have; the context
  manager itself degrades to a no-op off ROS 1 (see its own docstring), so calling it here would
  be cosmetic. Position-tracking-abort (validated all session) is the real safety net instead.
* Their preset joint configs (`behind_back_retract_pos`, `microwave_push_starting_pos`, ...) are
  specific to the original lab rig's mount and don't transfer (CLAUDE.md already flags this for
  every `preset_actions/*.py`). This script never needs a named preset -- everything is relative
  to the arm's actual current pose.

What's kept, because it's the actual proven shape: TWO phases, not one continuous pull to fully
closed. (1) swing the door MOST of the way shut via the handle, stopping a few waypoints short
of the computed end (their `closing_waypoints[-3]` pattern) rather than trying to pull all the
way to 0 deg, since the geometry gets awkward and the door's own latch resistance takes over
near fully closed. (2) a small additional PUSH along the same local approach axis to seat it the
rest of the way, then release and retreat.

Reuses, unmodified: the hinge geometry, per-waypoint seeded-IK / joint-jump / tracking-abort /
proactive-J6 guards from `real_gen3_ros2_grasp_and_swing_microwave.py`'s swing (same file that
validated the OPENING direction tonight) -- just with `direction` flipped for closing.

**Not run.** Written and syntax-checked only, same as
`real_gen3_ros2_grasp_and_swing_microwave.py` was before this. Dry-run (`--execute` omitted)
before trusting it on hardware. The push phase in particular (phase 2) has no analogue that was
actually executed tonight -- the session's own push-the-door-open attempt was abandoned after a
collision (see the joint-space-interp-hits-door finding), so treat `--phase push` with extra
caution and a human ready to intervene.
"""
import argparse, sys, time

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import CartesianTrajectoryCommand, JointCommand

from door_push import add_push_args, push_close
from microwave_common import move_to_door_view, regrasp, release_and_back_off
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

ARM = [1, 2, 3, 4, 5, 6, 7]

# Same corroborated local hinge estimate as the swing-open script tonight -- see that
# file's docstring for why its absolute position is trusted less than its value AS A
# LOCAL PIVOT (radius held stable across three real swings from a fresh grasp).
FIXED_HINGE = np.array([0.7177, 0.0699, 0.5585])
WAYPOINT_SPACING_M = 0.02
MAX_JUMP_DEG = 25.0
TRACK_ABORT = 0.02
MAX_IK_ERR = 0.02
MAX_REACH = 0.90
J6_LIMIT_DEG = 119.7
J6_GUARD_DEG = 115.0
PUSH_MAX_JUMP_DEG = 20.0  # phase 2 is a small move; tighter guard is cheap


def _sim():
    scene = create_scene_description_from_config(
        "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
    return scene, FeedingDeploymentPyBulletSimulator(scene, use_gui=False).robot


def _solve(scene, rb, pos, quat, seed_joints, iters=1000):
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(seed_joints[j]), physicsClientId=rb.physics_client_id)
    wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
    sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
        list(wpose.position), list(wpose.orientation),
        physicsClientId=rb.physics_client_id, maxNumIterations=iters, residualThreshold=1e-6)
    joints = np.array([sol[k] for k in range(7)])
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(joints[j]), physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    ikerr = float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(wpose.position)))
    return joints, ikerr


def run_swing_closed(ai, args):
    """Phase 1: swing the door most of the way shut via the still-grasped handle.
    Same generator/guards as the validated opening swing, direction reversed."""
    st = ai.get_state()
    g = float(st.get("gripper_pos"))
    if g < 0.2:
        sys.exit("Gripper is OPEN -- nothing grasped. Refusing to swing closed.")

    ee = list(st["ee_pos"])
    grasp_pose = Pose(position=tuple(ee[:3]), orientation=tuple(ee[3:7]))
    hinge = np.array(args.hinge) if args.hinge is not None else FIXED_HINGE
    radius = float(np.linalg.norm(np.array(ee[:3]) - hinge))
    arc_length_m = radius * np.radians(args.target_close_deg)
    print(f"current ee_pos: {np.round(ee[:3], 4)}  gripper {g:.4f}")
    print(f"hinge ({'--hinge' if args.hinge is not None else 'fixed'}): {np.round(hinge, 4)} (radius {radius * 100:.1f}cm), "
          f"arc_length {arc_length_m * 100:.1f}cm for {args.target_close_deg}deg closing")

    from feeding_deployment.interfaces.perception_interface import PerceptionInterface
    wps_pose = PerceptionInterface._generate_door_arc_waypoints(
        None, start_pose=grasp_pose, hinge_position=tuple(hinge),
        arc_length_m=arc_length_m, waypoint_spacing_m=WAYPOINT_SPACING_M,
        direction=args.direction, rotate_orientation=True)
    wps = [list(w.position) + list(w.orientation) for w in wps_pose]

    # Stop short of the computed end -- their own closing routine does the same
    # (move_to_ee_pose(handle_closing_poses["closing_waypoints"][-3]) after the
    # trajectory) rather than trying to pull all the way to a fully-shut door,
    # since the geometry gets awkward and the door's own latch resistance takes
    # over near the end -- that's what phase 2 (the push) is for.
    if args.stop_short > 0 and len(wps) > args.stop_short:
        wps = wps[:-args.stop_short]
    print(f"first {np.round(wps[0][:3], 3)} -> last {np.round(wps[-1][:3], 3)}")
    print(f"{len(wps)} waypoints planned (stopped {args.stop_short} short of the full computed arc)")

    if not args.execute:
        print("\nDRY RUN (swing-closed) -- nothing commanded.")
        return

    scene, rb = _sim()
    if args.smooth:
        # whole arc checked in sim first (chained IK from the real joints, same gates), then
        # one blended Cartesian trajectory -- same mechanism as the open script's --smooth
        # 200-iteration IK as a singularity probe: where it can't get within 1 cm, Kortex's own
        # Cartesian trajectory aborted with SINGULARITY_REGION (09-23) -> use the step loop instead
        q = np.array(ai.get_state()["position"], dtype=float)
        smooth_ok = True
        for i, w in enumerate(wps):
            nq, ikerr = _solve(scene, rb, w[:3], w[3:], q, iters=200)
            jump = float(np.max(np.degrees(np.abs(nq - q))))
            j6 = float(np.degrees(nq[5]))
            if ikerr > 0.01 or np.linalg.norm(w[:3]) > MAX_REACH or jump > MAX_JUMP_DEG or abs(j6) > J6_GUARD_DEG:
                print(f"smooth pre-check FAILED at waypoint {i + 1}: ik_err {ikerr * 100:.2f}cm, jump {jump:.1f}deg, "
                      f"J6 {j6:.1f}deg -- likely near a wrist singularity; falling back to the stop-and-go swing.")
                smooth_ok = False
                break
            q = nq
    if args.smooth and smooth_ok:
        print(f"smooth pre-check OK over all {len(wps)} waypoints; sending one blended Cartesian trajectory ...")
        ok = ai.execute_command(CartesianTrajectoryCommand([(w[:3], w[3:]) for w in wps]))
        final = ai.get_state()
        print(f"\nSMOOTH SWING-CLOSED {'DONE' if ok else 'RETURNED FALSE'}. final EE: "
              f"{np.round(final['ee_pos'][:3], 4)} ({np.linalg.norm(np.array(final['ee_pos'][:3]) - np.array(wps[-1][:3])) * 100:.1f} cm "
              f"from the last waypoint), gripper: {final.get('gripper_pos')}")
        return
    for i, w in enumerate(wps):
        pos, quat = w[:3], w[3:]
        real_joints = np.array(ai.get_state()["position"], dtype=float)
        joints, ikerr = _solve(scene, rb, pos, quat, real_joints)
        max_delta = float(np.max(np.degrees(np.abs(joints - real_joints))))
        d = float(np.linalg.norm(pos))
        j6 = float(np.degrees(joints[5]))
        print(f"step {i + 1}/{len(wps)} -> {np.round(pos, 3)} ({d * 100:.0f}cm, "
              f"ik_err {ikerr * 100:.2f}cm, jump {max_delta:.1f}deg, J6 {j6:.1f}deg)")
        if ikerr > MAX_IK_ERR or d > MAX_REACH:
            print("  STOP: target unreachable / past reach limit.")
            break
        if max_delta > MAX_JUMP_DEG:
            print(f"  STOP: joint jump {max_delta:.1f}deg > {MAX_JUMP_DEG}deg guard.")
            break
        if abs(j6) > J6_GUARD_DEG:
            print(f"  STOP: J6 would reach {j6:.1f}deg, closing in on its +-{J6_LIMIT_DEG}deg hard limit.")
            break

        ai.execute_command(JointCommand(pos=joints.tolist()))
        time.sleep(0.15)
        for _ in range(20):
            stt = ai.get_state()
            if max(abs(x) for x in stt["velocity"]) < 0.02:
                break
            time.sleep(0.08)
        got = np.array(ai.get_state()["ee_pos"][:3])
        err = float(np.linalg.norm(got - np.array(pos)))
        if err > TRACK_ABORT:
            print(f"  ABORT: tracking err {err * 100:.1f}cm > {TRACK_ABORT * 100:.0f}cm "
                  "-- door binding / latch / natural limit.")
            break

    final = ai.get_state()
    print("\nSWING-CLOSED DONE. final EE:", np.round(final["ee_pos"][:3], 4), "gripper:", final.get("gripper_pos"))


def run_push_shut(ai, args):
    """Phase 2: a small additional push along the SAME local approach axis the
    grasp used, to seat the door the rest of the way -- deliberately NOT a fresh
    face-normal re-approach (that's the maneuver that led to a real collision
    earlier tonight; see joint-space-interp-hits-door). Still gripping the
    handle throughout the push; releases and retreats only at the end.

    UNVALIDATED on hardware -- go slowly and be ready to stop.
    """
    st = ai.get_state()
    g = float(st.get("gripper_pos"))
    if g < 0.2:
        sys.exit("Gripper is OPEN -- nothing grasped. Refusing to push.")

    ee = list(st["ee_pos"])
    cur_pos, quat = np.array(ee[:3]), tuple(ee[3:7])
    R_ee = R.from_quat(quat).as_matrix()
    local_z = R_ee[:, 2]  # the approach axis used throughout this session's off()
    push_target = cur_pos + local_z * args.push_dist
    print(f"current EE {np.round(cur_pos, 4)}  gripper {g:.4f}")
    print(f"push target {np.round(push_target, 4)}  ({args.push_dist * 100:.1f}cm further along local approach axis)")

    if not args.execute:
        print("\nDRY RUN (push) -- nothing commanded.")
        return

    scene, rb = _sim()
    real_joints = np.array(ai.get_state()["position"], dtype=float)
    joints, ikerr = _solve(scene, rb, push_target, quat, real_joints)
    jump = float(np.degrees(np.max(np.abs(joints - real_joints))))
    d = float(np.linalg.norm(push_target))
    print(f"IK err {ikerr * 100:.2f}cm  jump {jump:.1f}deg  range {d:.3f}m")
    if ikerr > MAX_IK_ERR or d > MAX_REACH:
        sys.exit("Push target unreachable / past reach limit -- refusing.")
    if jump > PUSH_MAX_JUMP_DEG:
        sys.exit(f"Push jump {jump:.1f}deg > {PUSH_MAX_JUMP_DEG}deg guard -- refusing (should be small; "
                 "something is off if it isn't).")

    ai.execute_command(JointCommand(pos=joints.tolist()))
    time.sleep(0.15)
    for _ in range(20):
        stt = ai.get_state()
        if max(abs(x) for x in stt["velocity"]) < 0.02:
            break
        time.sleep(0.08)
    got = np.array(ai.get_state()["ee_pos"][:3])
    err = float(np.linalg.norm(got - push_target))
    print(f"reached {np.round(got, 3)}  tracking err {err * 100:.1f}cm")
    if err > TRACK_ABORT:
        print(f"ABORT: tracking err {err * 100:.1f}cm > {TRACK_ABORT * 100:.0f}cm -- door resisted more than expected. "
              "Not releasing/retreating automatically -- check the door by hand before doing anything else.")
        return

    if args.no_release:
        print("push landed within tracking tolerance. --no-release: still gripping, arm left here.")
        return
    print("push landed within tracking tolerance.")
    release_and_back_off(ai, args.retreat_dist, execute=True)


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--execute", action="store_true", help="actually command the arm; omit for a dry-run/plan-only pass")
    a.add_argument("--phase", choices=["view", "regrasp", "swing", "push", "both", "release", "push-close"],
                   default="both",
                   help="'view' = move to where the wrist camera sees the OPEN door square-on (then run the "
                        "grasp script's --phase grasp to detect + grasp); 'regrasp' = drive back onto the handle recorded by the last release and close the "
                        "gripper (needed when the door was opened and released); then 'swing' the door mostly "
                        "shut via the handle, 'push' it the rest of the way, or 'both' (with a pause in "
                        "between); 'release' = just open the gripper and back off; 'push-close' = no grasp: "
                        "push the door shut with the side of the hand (gripper left as is, e.g. holding a "
                        "container), then back away --retreat-dist")
    a.add_argument("--direction", type=int, default=1, choices=[-1, 1],
                   help="arc direction for the closing swing -- opposite of whatever the opening script "
                        "used (that was -1 tonight, so closing defaults to +1)")
    a.add_argument("--target-close-deg", type=float, default=50.0,
                   help="how many degrees to swing back toward closed, before the stop-short waypoints")
    a.add_argument("--stop-short", type=int, default=3,
                   help="stop this many waypoints before the computed end of the closing swing (their own "
                        "close_microwave does the same, [-3]) -- the push phase covers the last bit")
    a.add_argument("--push-dist", type=float, default=0.04,
                   help="phase 2: how far to push along the local approach axis, meters")
    a.add_argument("--retreat-dist", type=float, default=0.20,
                   help="after the push and release, how far to back straight off along the approach axis")
    a.add_argument("--no-release", action="store_true",
                   help="push phase: stay gripping after the push (no release/retreat), e.g. to push again")
    a.add_argument("--smooth", action="store_true",
                   help="swing: pre-check every waypoint in sim, then ONE blended Cartesian trajectory")
    a.add_argument("--view-open-deg", type=float, default=70.0,
                   help="--phase view: face the door as if it is open this far (the recorded release only "
                        "gives the swing SENSE); 70 because the real open task will open past 50")
    a.add_argument("--hinge", type=float, nargs=3, metavar=("X", "Y", "Z"), default=None,
                   help="override FIXED_HINGE (arm_base_link, m) -- pass the same hinge the opening swing used")
    add_push_args(a, close=True)
    args = a.parse_args()

    ai = ArmInterfaceClient()

    if args.phase == "push-close":
        push_close(ai, args)
        return

    if args.phase == "view":
        move_to_door_view(ai, args.execute, open_deg=args.view_open_deg)
        return

    if args.phase == "regrasp":
        regrasp(ai, args.execute)
        return

    if args.phase in ("swing", "both"):
        run_swing_closed(ai, args)
        if args.phase == "both":
            if not args.execute:
                print("\n(dry run -- would pause here before the push phase)")
                return
            input("\nCheck the door/handle position, then press Enter to continue to the push phase (Ctrl-C to stop here) ...")

    if args.phase in ("push", "both"):
        run_push_shut(ai, args)

    if args.phase == "release":
        release_and_back_off(ai, args.retreat_dist, args.execute)


if __name__ == "__main__":
    main()
