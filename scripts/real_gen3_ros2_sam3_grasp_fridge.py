"""Full perception-driven grasp: detect ONCE, then pre-grasp -> grasp -> close.

--phase approach|grasp|swing|both (default both), mirroring the microwave's
combined script. "approach" replaces the former real_gen3_ros2_approach_fridge.py,
which duplicated the detection call, constants and gates -- copies that drifted.
Dry-run by default; --execute to move. In --phase both, the swing waits for a
typed confirmation after the grasp so the grip can be checked by eye first.

    ARM_RPC_HOST=127.0.0.1 python3 scripts/real_gen3_ros2_sam3_grasp_fridge.py --phase approach --steps 6
    ARM_RPC_HOST=127.0.0.1 python3 scripts/real_gen3_ros2_sam3_grasp_fridge.py --phase grasp --steps 6 --execute
    ARM_RPC_HOST=127.0.0.1 python3 scripts/real_gen3_ros2_sam3_grasp_fridge.py --phase swing --target-angle-deg 20 --execute

Detection is SAM 3 + RealSense depth (detect_handle_sam3.py): SAM 3 segments
the handle from the colour image by text prompt, the RealSense gives depth at
those pixels, and the median 3D point goes through the usual TF into
arm_base_link. Nothing detects the fridge, the door, or a colour any more.

Why (2026-09-20, replacing two earlier attempts in this same file):
  * YOLO appliance box -> plane-fit/DBSCAN failed twice over on this fridge --
    the door fills the frame at grasp range so YOLO sees no appliance, and the
    handle sits ~11 mm off the door plane, inside RANSAC's 20 mm inlier band,
    so the plane fit swallowed it. Not a depth-noise problem: measured door
    depth was 1.8 mm plane residual.
  * HSV colour threshold hardcoded white; dead for a future black handle.
SAM 3 is colour-agnostic and works at the close range where the box step
could not even start. Full rationale in detect_handle_sam3.py.

The old path also returned a "top of appliance" z from the YOLO box, used only
to write /tmp/fridge_top_z.txt for a post-release lift. No fridge arc script
exists to consume it, so it is gone with the box.

GRIP_EXT is +0.015 (walked back from -0.020 in four 1 cm steps against real grasps;
comment). VERTICAL_CORR stays at 0 -- not yet calibrated for this fridge; the
microwave's +0.025 was tuned on that appliance's own detection bias and does
not transfer.

SWING (--phase swing / both): opens the door by sweeping the grasped handle
about the hinge, ported from real_gen3_ros2_grasp_and_swing_microwave.py
(microwave-task branch) -- the continuous per-waypoint pattern validated on
this rig 2026-09-19, with its IK / reach / joint-jump / tracking guards and
the proactive J6 check. The hinge is the one fridge-specific input: 13 in
(33 cm) to the handle's left per the user's measurement, expressed relative
to the grasp point (this fridge gets moved) -- see HINGE_OFFSET_FROM_GRASP.
Keep --target-angle-deg small on a first run: swing direction has needed a
live visual check on every rig so far.
"""
import argparse, json, os, sys, time
from pathlib import Path
import numpy as np, pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

import detect_handle_sam3
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import (
    CloseGripperCommand, JointCommand)
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

# Arm START POSE for the fridge task -- captured from the real arm 2026-09-20
# with the arm parked by hand where the user wants every run to begin. Joint
# angles, not an EE pose: replaying joints reproduces this exact configuration
# with no IK solve, so there is no alternate-branch wrist-flip risk.
#   joints_deg  [62.78, 11.48, -151.29, -85.14, 87.29, 93.26, 5.79]
#   ee_xyz      [0.3232, -0.3757, 0.6544]   gripper open (0.0087)
START_JOINTS_RAD = np.array([1.0957, 0.2004, -2.6405, -1.4859, 1.5234, 1.6277, 0.1011])

PRE_STANDOFF = 0.12
# GRIP_EXT: how far PAST the detected point to drive before closing. Detection
# returns the median of the handle's visible pixels, i.e. its FRONT FACE -- the
# camera cannot see the back -- so with GRIP_EXT=0 the fingertips meet the
# surface instead of wrapping the handle (2026-09-20: first grasp on this
# fridge "closed too early and didn't catch the handle" for exactly this
# reason). Set to half the handle's front-to-back depth so the fingers close
# on its centre. Sign: grasp = off([LATERAL, 0, -GRIP_EXT]) below, so NEGATIVE
# extends past the surface and positive falls short.
#   2026-09-20: -0.020, from a ~4 cm handle depth estimated by eye. VALIDATED
#   on hardware the same day: first --execute grasp with this value closed
#   cleanly on the handle ("that was perfect"), with VERTICAL_CORR=0 and
#   LATERAL=0 -- so no residual height/lateral bias needed correcting on this
#   rig at the ~0.5 m detection standoff used.
#   2026-09-20, later: the 5.1 cm calibration error that had been hiding
#   inside this number was moved to detect_handle_sam3.HANDLE_DEPTH_CORR_M
#   (measured against a touched ground truth). With that in place -0.020
#   grasped cleanly but overshot; walked back 1 cm at a time at the user's
#   direction: -0.015, -0.005, +0.005, then +0.015 (each "one waypoint" back).
#   POSITIVE now: the fingers close 1.5 cm SHORT of the detected front face.
#   Net of the depth correction the target sits 3.6 cm past the raw detection,
#   vs the 7.1 cm the hand-placed touch test gave -- a 3.5 cm gap between
#   where a hand put the gripper and where real grasps want it. Tuned against
#   real grasps, not an estimate; do not "fix" it back toward -0.020.
GRIP_EXT = +0.015
VERTICAL_CORR = 0.0   # 0 validated 2026-09-20 -- no height bias observed
LATERAL = 0.0
# 0.91: the arm demonstrably reached 0.858 under teleop, and the Gen3 spec is
# ~0.90 (confirmed: Kinova's published Gen3 7DoF max reach is 902mm). 0.85 was
# over-tight; 0.88 blocked a since-reverted GRIP_EXT change; 0.90 (the spec
# ceiling) blocked the sign-fixed GRIP_EXT=-0.024 grasp target by 4mm
# (0.904m) -- nudged 1cm past spec at the user's direction. Untested.
#
# 2026-09-19: briefly bumped to 0.95 then 1.0 at the user's explicit direction
# (this fridge's grasp point measured 0.936m) -- then reverted back to 0.91
# here at the user's request rather than keep either bump. 1.0m in particular
# would have been past the arm's actual physical max reach (902mm, confirmed
# via Kinova's datasheet), not just a safety-margin nudge.
#
# 2026-09-20: 0.91 -> 0.92 at the user's direction. After taping the fridge down
# the grasp point measured 0.915 m; IK solved at 0.00 cm and the arm had been
# hand-moved to 0.99 m earlier the same day, so 1 cm was a margin nudge, not a
# physical-limit question. The swing moves TOWARD the arm from the grasp, so
# the grasp is always the farthest point this guard sees.
# 2026-09-20, later: 0.92 -> 0.94 at the user's explicit direction. This is
# ~4 cm past Kinova's 902 mm spec figure, which is measured to the wrist
# interface -- the tool frame this gate measures sits beyond that, and the arm
# was hand-moved to 0.99 m (tool frame) earlier the same day. The IK gate
# (MAX_IK_ERR) is the real backstop: an unreachable target shows up there.
MAX_REACH, MIN_Z, MAX_Z = 0.94, 0.25, 0.75
MAX_IK_ERR, MAX_JUMP_DEG = 0.02, 90.0
TRACK_ABORT = 0.03
ARM = [1,2,3,4,5,6,7]


# ---------------------------------------------------------------------------
# SWING: ported from scripts/real_gen3_ros2_grasp_and_swing_microwave.py
# (microwave-task branch), the pattern validated on this rig 2026-09-19 -- a
# continuous per-waypoint set_joint_position loop with every gate kept (IK
# error, reach, joint jump, tracking abort) plus the PROACTIVE J6 check that
# root-caused the earlier "growing error on wide swings" mystery.
#
# The one fridge-specific input is the hinge. The microwave used a FIXED
# absolute hinge; this fridge gets moved around between runs, so the hinge is
# expressed RELATIVE TO THE GRASP POINT and re-derived every swing:
#   user measurement 2026-09-20: hinge is 14.5 in (36.8 cm) to the LEFT of the
#   MIDDLE of the handle (the grasp point, since GRIP_EXT centres the fingers).
#   With the door facing the arm head-on, camera image-left maps to
#   arm_base_link +y (checked live from the camera TF: image-left = [-0.002,
#   1.000, 0]). Hence +0.368 m in y. Same height as the handle (vertical axis).
#   History: a first eyeballed 13 in (33.0 cm) produced a "90 deg" swing that
#   left the door at only ~79 deg -- the gripper's 46.7 cm chord subtends 78.6
#   deg about the true 36.8 cm pivot. Radius error shows up as door angle
#   error, so measure it, don't guess it.
#
# DEPTH of the pin (x, away from the arm). The grasp point is the handle's
# CENTRE, which GRIP_EXT puts ~2 cm in front of the door panel -- but the pin is
# at or behind the panel, never in front of it. Leaving x at 0 put the assumed
# hinge ~2+ cm too close to the arm; at 90 deg that error appears fully as the
# gripper dragging the handle toward the arm, and on 2026-09-20 the fridge's
# hinge-side corner was seen coming FORWARD partway through a 100 deg swing.
#   HANDLE_CENTRE_TO_FACE_M: the known part (half the ~4 cm handle depth).
#   PIN_BEHIND_FACE_M: how far the pin sits behind the door's front face --
#   MEASURE THIS at the top hinge bracket (0 if flush; usually 0.5-2 cm).
HANDLE_CENTRE_TO_FACE_M = 0.020
# 1.25 in measured 2026-09-20 (door thickness, pin at the door's back face).
# With that alone the hinge side still crept forward SLIGHTLY on a taped-down
# 100 deg swing -- same signature, smaller -- so an empirical +1.5 cm was added
# at the user's direction. If the creep reverses (hinge side moves BACK), this
# overshot: split the difference.
PIN_BEHIND_FACE_M = 1.25 * 0.0254 + 0.015
HINGE_OFFSET_FROM_GRASP = np.array([
    HANDLE_CENTRE_TO_FACE_M + PIN_BEHIND_FACE_M,   # +x: farther from the arm
    14.5 * 0.0254,                                 # +y: along the door, to the pin
    0.0,
])
# -1 pulls the handle TOWARD the arm (x decreases) = door opens; +1 pushes.
# Verified in sim 2026-09-20 from the actual grasp pose: -1 -> dx=-8.5cm at 15deg.
# Both rigs so far still needed a live visual check of swing direction on the
# first real batch -- keep --target-angle-deg small the first time.
SWING_DIRECTION = -1
# The hinge offset above is only correct while the door is CLOSED. Once the door
# has swung, the handle is no longer straight in front of the hinge, so
# re-deriving "hinge = current grasp + offset" would put the hinge ~radius*sin(theta)
# off (34 cm at 20 deg -- caught 2026-09-20 before it was run). So the hinge is
# fixed the moment the door is grasped closed and persisted here, along with
# the cumulative swing so far; later swings reuse it. A new grasp resets it.
HINGE_STATE = Path("/tmp/fridge_door_hinge.json")
SWING_WAYPOINT_SPACING_M = 0.02
SWING_MAX_JUMP_DEG = 25.0
SWING_TRACK_ABORT = 0.02
SWING_MAX_IK_ERR = 0.02
SWING_MAX_REACH = MAX_REACH
# J6's real hard limit is +-119.7 deg (+-2.09 rad, from the Gen3 URDF). Stop
# well before it: past ~115 deg the IK/tracking error grows every step.
J6_LIMIT_DEG = 119.7
J6_GUARD_DEG = 115.0


def _wait_converged(ai, q_cmd, tol_deg=1.0, timeout_s=6.0):
    """Block until the arm's joints are within tol_deg of q_cmd and at rest.

    Returns the final joint error in degrees. A plain "velocity ~ 0" wait is
    not enough here: Kortex's blocking move returns on ACTION_END *or*
    ACTION_ABORT, and a stale END from an earlier action can release it early,
    so the next command lands while the arm is still moving and is rejected
    with ROBOT_MOVEMENT_IN_PROGRESS -- that sub-step is silently skipped. Its
    own 5 deg "did it arrive" check cannot see this either, because a --steps
    sub-step is smaller than 5 deg. Seen 9 times in the 2026-09-20 arm log; when
    the skipped step was the last grasp step the gripper closed 2 cm short.
    """
    q_cmd = np.asarray(q_cmd, dtype=float)
    deadline = time.time() + timeout_s
    derr = float("inf")
    while time.time() < deadline:
        time.sleep(0.12)
        st = ai.get_state()
        qa = np.asarray(st["position"], dtype=float)
        derr = float(np.degrees(np.max(np.abs((qa - q_cmd + np.pi) % (2 * np.pi) - np.pi))))
        vel = float(np.max(np.abs(np.asarray(st["velocity"], dtype=float))))
        if derr < tol_deg and vel < 1e-3:
            break
    return derr


def _move_joints_checked(ai, q, name):
    """execute_command + convergence check, re-sending once if a step was dropped."""
    ai.execute_command(JointCommand(pos=np.asarray(q).tolist()))
    derr = _wait_converged(ai, q)
    if derr >= 1.0:
        print(f"  {name}: arm settled {derr:.1f} deg short of the commanded step "
              f"(Kortex likely rejected it as ROBOT_MOVEMENT_IN_PROGRESS) -- re-sending once")
        time.sleep(0.5)
        ai.execute_command(JointCommand(pos=np.asarray(q).tolist()))
        derr = _wait_converged(ai, q)
    if derr >= 1.0:
        sys.exit(f"{name}: still {derr:.1f} deg off after retry -- ABORT, gripper untouched.")
    return derr


def run_grasp(ai, args):
    st = ai.get_state()
    ee0 = np.asarray(list(st["ee_pos"])[:3], dtype=float); g0 = float(st.get("gripper_pos"))
    print(f"start EE {np.round(ee0,4)}  gripper {g0:.4f}")
    if g0 > 0.2: sys.exit("Gripper not open -- refusing.")

    _log_dir = os.environ.get("DETECTION_LOG_DIR")
    try:
        det, rs, tf, _log_dir = detect_handle_sam3.build_detector(
            prompt=args.prompt, log_dir=_log_dir)
        handle, h = detect_handle_sam3.detect_until_agree(
            det, rs, tf, args.max_detects, log_dir=_log_dir)
        detect_handle_sam3.check_plausible(h)
    except RuntimeError as e:
        sys.exit(str(e))
    print(f"handle (mean) {np.round(h,4)}")
    print(f"camera->handle {np.linalg.norm(h-ee0)*100:.0f} cm (viewing distance)")
    h = h + np.array([0.0, 0.0, VERTICAL_CORR])  # world-frame z bias; currently 0, see module docstring
    print(f"handle (vertical-corrected) {np.round(h,4)}")

    quat = (R.from_quat(np.asarray(handle.orientation)) * R.from_euler("y", np.pi)).as_quat()
    hp = Pose(tuple(h), tuple(quat))
    def _pose_to_matrix(pose):
        m = np.zeros((4,4)); m[:3,3] = pose[0]
        m[:3,:3] = R.from_quat(pose[1]).as_matrix(); m[3,3] = 1
        return m
    def off(v):
        m = np.eye(4); m[:3,3] = v
        out = _pose_to_matrix(hp) @ m
        return Pose(out[:3,3], R.from_matrix(out[:3,:3]).as_quat())
    pre   = off([0.0, 0.0, -PRE_STANDOFF])
    grasp = off([LATERAL, 0.0, -GRIP_EXT])
    print(f"pre-grasp {np.round(pre.position,4)}  range {np.linalg.norm(pre.position):.3f}")
    print(f"grasp     {np.round(grasp.position,4)}  range {np.linalg.norm(grasp.position):.3f}")

    scene = create_scene_description_from_config(
        "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
    sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False); rb = sim.robot

    def solve(pose, seed):
        for i,jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(seed[i]), physicsClientId=rb.physics_client_id)
        w = multiply_poses(scene.robot_base_pose, pose)
        sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
            list(w.position), list(w.orientation), maxNumIterations=400,
            residualThreshold=1e-5, physicsClientId=rb.physics_client_id)
        q = np.asarray(sol[:7])
        for i,jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(q[i]), physicsClientId=rb.physics_client_id)
        ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
        return q, float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(w.position)))

    cur = np.asarray(st["position"], dtype=float); plan = []
    phases = (("pre-grasp", pre),) if args.phase == "approach" else (("pre-grasp", pre), ("grasp", grasp))
    for name, pose in phases:
        t = np.asarray(pose.position)
        bad = []
        if np.linalg.norm(t) > MAX_REACH: bad.append(f"range {np.linalg.norm(t):.3f}>{MAX_REACH}")
        if not (MIN_Z <= t[2] <= MAX_Z):  bad.append(f"z {t[2]:.3f} out of range")
        q, err = solve(pose, cur)
        jump = float(np.degrees(np.max(np.abs(q-cur))))
        if err > MAX_IK_ERR:   bad.append(f"IK err {err*100:.1f}cm")
        if jump > MAX_JUMP_DEG: bad.append(f"jump {jump:.0f}deg")
        # NOTE: PyBullet's IK does not enforce the real arm's joint limits. A solution
        # can show 0.00 cm residual and still be rejected by Kortex (METHOD_FAILED,
        # no motion -- looks like the arm silently did nothing). There is no
        # pre-validation here; if that happens, check arm_server's log.
        print(f"  {name:10s} IK {err*100:5.2f} cm  jump {jump:5.1f} deg  {'FAIL: '+'; '.join(bad) if bad else 'OK'}")
        if bad: sys.exit(f"GATE FAILED at {name}")
        plan.append((name, t, q)); cur = q

    def fk(joints):
        """EE position in arm_base_link for a joint vector."""
        for i, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(joints[i]), physicsClientId=rb.physics_client_id)
        ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
        return np.asarray(ls[4]) - np.asarray(scene.robot_base_pose.position)

    # Split each move into small increments. JOINT-space interpolation (same choice as the
    # approach script's default): it ends at exactly the IK solution the gates above passed,
    # per-step motion is bounded by construction at total/N, and no null-space drift is
    # possible. The EE follows the same curve the unchained move would have taken -- just
    # cut into pieces that can be checked and aborted between.
    if args.steps > 1:
        chained, prev = [], np.asarray(st["position"], dtype=float)
        print(f"\nchaining each move into {args.steps} joint sub-steps "
              f"(guard {args.max_step_deg} deg/step):")
        for name, t, q in plan:
            # Shortest path per joint: a raw difference can read ~350 deg for what is
            # physically a few degrees the other way round.
            total = (q - prev + np.pi) % (2 * np.pi) - np.pi
            base_q, worst = prev.copy(), 0.0
            for s in range(1, args.steps + 1):
                qs = base_q + total * (s / args.steps)
                delta = float(np.degrees(np.max(np.abs(qs - prev))))
                worst = max(worst, delta)
                if delta > args.max_step_deg:
                    sys.exit(f"Sub-step {s} of '{name}' moves {delta:.0f} deg "
                             f"(> {args.max_step_deg}) -- refusing the whole move.")
                chained.append((f"{name} {s:02d}/{args.steps}", fk(qs), qs))
                prev = qs
            end_err = float(np.linalg.norm(fk(prev) - t))
            print(f"  {name:10s} {args.steps} steps, worst {worst:5.1f} deg  "
                  f"endpoint {end_err*100:.3f} cm from the gated target")
            if end_err > MAX_IK_ERR:
                sys.exit(f"Chain for '{name}' ends {end_err*100:.1f} cm off target -- refusing.")
        plan = chained

    if not args.execute:
        print("\nDRY RUN (grasp) -- nothing commanded.")
        return grasp   # planned grasp pose, so a dry-run swing can plan from it

    print(f"\nspeed: {ai.get_speed()}  -- commanding {len(plan)} move(s) ...")
    for name, t, q in plan:
        _move_joints_checked(ai, q, name)
        time.sleep(0.2)
        fin = np.asarray(list(ai.get_state()["ee_pos"])[:3], dtype=float)
        e = float(np.linalg.norm(fin - t))
        print(f"  {name:18s} EE {np.round(fin,4)}  tracking {e*100:4.1f} cm")
        if e > TRACK_ABORT:
            sys.exit(f"Tracking {e*100:.1f} cm at {name} -- ABORT, gripper untouched.")

    if args.phase == "approach":
        print("\nAPPROACH ONLY -- at pre-grasp standoff, gripper untouched.")
        return None

    print("\nclosing gripper ...")
    ai.execute_command(CloseGripperCommand()); time.sleep(3.5)
    gf = float(ai.get_state().get("gripper_pos"))
    print(f"gripper after close: {gf:.4f}")
    # This grasp is on the CLOSED door: fix the hinge from it (see HINGE_STATE).
    ee_c = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
    HINGE_STATE.write_text(json.dumps({
        "hinge": (ee_c + HINGE_OFFSET_FROM_GRASP).tolist(),
        "grasp_closed": ee_c.tolist(), "cum_deg": 0.0}))
    print(f"hinge fixed at {np.round(ee_c + HINGE_OFFSET_FROM_GRASP, 4)} -> {HINGE_STATE}")
    # Deliberately no automated verdict. gripper_pos saturates near 1.0 whether or
    # not the handle is between the fingers (CLAUDE.md records ~0.99 while holding the
    # door), so it cannot tell success from a miss -- an earlier version called a
    # confirmed-good grasp "closed empty". The proven flow uses a human grip check.
    print("Gripper closed. gripper_pos cannot confirm a grasp on this rig --")
    print("CHECK VISUALLY.")

def run_swing(ai, args, start_override=None):
    """Open the door by sweeping the (already grasped) handle about the hinge.

    start_override: a Pose to plan from instead of the arm's current pose. Used
    by a --phase both DRY RUN, where the grasp has not actually happened yet, so
    the swing must be planned from the grasp the dry run just computed -- not
    from wherever the arm happens to be parked.
    """
    st = ai.get_state()
    g = float(st.get("gripper_pos"))
    if start_override is None and g < 0.2:
        sys.exit("Gripper is OPEN -- nothing grasped. Refusing to run the swing.")

    if start_override is not None:
        ee = list(start_override.position) + list(start_override.orientation)
        print(f"(planning from the PLANNED grasp pose, not the arm's current pose)")
    else:
        ee = list(st["ee_pos"])
    grasp_pose = Pose(position=tuple(ee[:3]), orientation=tuple(ee[3:7]))
    if args.reset_hinge and HINGE_STATE.exists():
        HINGE_STATE.unlink()
    if HINGE_STATE.exists():
        hs = json.loads(HINGE_STATE.read_text())
        hinge, cum = np.asarray(hs["hinge"]), float(hs.get("cum_deg", 0.0))
        print(f"hinge (persisted from the closed-door grasp): {np.round(hinge, 4)}  "
              f"door already open {cum:.0f}deg")
    else:
        # No record: assume the door is CLOSED right now and derive from here.
        hinge, cum = np.asarray(ee[:3]) + HINGE_OFFSET_FROM_GRASP, 0.0
        print(f"hinge = grasp + {np.round(HINGE_OFFSET_FROM_GRASP, 3)} = {np.round(hinge, 4)}  "
              f"(no {HINGE_STATE.name}: ASSUMING the door is closed)")
    radius = float(np.linalg.norm(np.asarray(ee[:3]) - hinge))
    arc_length_m = radius * np.radians(args.target_angle_deg)
    print(f"current ee_pos: {np.round(ee[:3], 4)}  gripper {g:.4f}  radius {radius * 100:.1f}cm")
    print(f"this swing: {args.target_angle_deg}deg (arc {arc_length_m * 100:.1f}cm) -> door would be at "
          f"{cum + args.target_angle_deg:.0f}deg, direction {SWING_DIRECTION:+d}")

    from feeding_deployment.interfaces.perception_interface import PerceptionInterface
    wps_pose = PerceptionInterface._generate_door_arc_waypoints(
        None, start_pose=grasp_pose, hinge_position=tuple(hinge),
        arc_length_m=arc_length_m, waypoint_spacing_m=SWING_WAYPOINT_SPACING_M,
        direction=SWING_DIRECTION, rotate_orientation=True)
    wps = [list(w.position) + list(w.orientation) for w in wps_pose]
    print(f"{len(wps)} waypoints planned")

    scene = create_scene_description_from_config(
        "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
    sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
    rb = sim.robot

    # Dry run: walk the whole chain in sim, seeding each IK from the previous
    # solution, so every gate is reported BEFORE anything moves (the microwave
    # version only printed the waypoint count here).
    if not args.execute:
        prev = np.array(st["position"], dtype=float)
        if start_override is not None:
            # seed from the grasp pose's own IK solution, seeded in turn from the real joints
            for j, jj in enumerate(ARM):
                p.resetJointState(rb.robot_id, jj, float(prev[j]), physicsClientId=rb.physics_client_id)
            gw = multiply_poses(scene.robot_base_pose, grasp_pose)
            gsol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
                list(gw.position), list(gw.orientation), physicsClientId=rb.physics_client_id,
                maxNumIterations=400, residualThreshold=1e-5)
            prev = np.array([gsol[k] for k in range(7)])
        for i, w in enumerate(wps):
            pos, quat = w[:3], w[3:]
            for j, jj in enumerate(ARM):
                p.resetJointState(rb.robot_id, jj, float(prev[j]), physicsClientId=rb.physics_client_id)
            wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
            sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
                list(wpose.position), list(wpose.orientation),
                physicsClientId=rb.physics_client_id, maxNumIterations=200)
            joints = np.array([sol[k] for k in range(7)])
            for j, jj in enumerate(ARM):
                p.resetJointState(rb.robot_id, jj, float(joints[j]), physicsClientId=rb.physics_client_id)
            ikerr = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wpose.position))
            max_delta = float(np.max(np.degrees(np.abs(joints - prev))))
            d = float(np.linalg.norm(pos)); j6 = float(np.degrees(joints[5]))
            flags = []
            if ikerr > SWING_MAX_IK_ERR: flags.append(f"ik_err>{SWING_MAX_IK_ERR*100:.0f}cm")
            if d > SWING_MAX_REACH: flags.append(f"reach>{SWING_MAX_REACH}")
            if max_delta > SWING_MAX_JUMP_DEG: flags.append(f"jump>{SWING_MAX_JUMP_DEG:.0f}deg")
            if abs(j6) > J6_GUARD_DEG: flags.append(f"J6>{J6_GUARD_DEG:.0f}deg")
            print(f"  wp {i + 1:2d}/{len(wps)} {np.round(pos, 3)} {d*100:4.0f}cm  ik_err {ikerr*100:.2f}cm  "
                  f"jump {max_delta:4.1f}deg  J6 {j6:6.1f}deg  {'WOULD STOP: ' + ', '.join(flags) if flags else 'OK'}")
            if flags:
                print(f"  (execution would stop cleanly at wp {i + 1}; {len(wps) - i - 1} not reached)")
                break
            prev = joints
        print("\nDRY RUN (swing) -- nothing commanded.")
        return

    for i, w in enumerate(wps):
        pos, quat = w[:3], w[3:]
        real_joints = np.array(ai.get_state()["position"], dtype=float)
        for j, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(real_joints[j]), physicsClientId=rb.physics_client_id)
        wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
        sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
            list(wpose.position), list(wpose.orientation),
            physicsClientId=rb.physics_client_id, maxNumIterations=200)
        joints = np.array([sol[k] for k in range(7)])
        for j, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(joints[j]), physicsClientId=rb.physics_client_id)
        ikerr = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wpose.position))
        max_delta = float(np.max(np.degrees(np.abs(joints - real_joints))))
        d = float(np.linalg.norm(pos))
        j6 = float(np.degrees(joints[5]))
        print(f"step {i + 1}/{len(wps)} -> {np.round(pos, 3)} ({d * 100:.0f}cm, "
              f"ik_err {ikerr * 100:.2f}cm, jump {max_delta:.1f}deg, J6 {j6:.1f}deg)")
        if ikerr > SWING_MAX_IK_ERR or d > SWING_MAX_REACH:
            print("  STOP: target unreachable / past reach limit.")
            break
        if max_delta > SWING_MAX_JUMP_DEG:
            print(f"  STOP: joint jump {max_delta:.1f}deg > {SWING_MAX_JUMP_DEG}deg guard.")
            break
        if abs(j6) > J6_GUARD_DEG:
            print(f"  STOP: J6 would reach {j6:.1f}deg, closing in on its "
                  f"+-{J6_LIMIT_DEG}deg hard limit -- stopping cleanly rather than pushing further.")
            break

        _move_joints_checked(ai, joints, f"swing wp {i + 1}")
        got = np.array(ai.get_state()["ee_pos"][:3])
        err = float(np.linalg.norm(got - np.array(pos)))
        if err > SWING_TRACK_ABORT:
            print(f"  ABORT: tracking err {err * 100:.1f}cm > {SWING_TRACK_ABORT * 100:.0f}cm "
                  "-- door binding / latch / natural limit.")
            break

    final = ai.get_state()
    fe = np.asarray(final["ee_pos"][:3])
    # Actual angle swept = angle between start and end radius vectors about the hinge.
    v0, v1 = (np.asarray(ee[:3]) - hinge)[:2], (fe - hinge)[:2]
    swept = float(np.degrees(np.arccos(np.clip(np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)), -1, 1))))
    HINGE_STATE.write_text(json.dumps({"hinge": hinge.tolist(), "cum_deg": cum + swept}))
    print(f"\nSWING DONE. swept {swept:.1f}deg this run, door now ~{cum + swept:.0f}deg open. "
          f"final EE {np.round(fe, 4)} gripper {final.get('gripper_pos'):.4f}")



def main():
    a = argparse.ArgumentParser()
    a.add_argument("--execute", action="store_true")
    a.add_argument("--phase", choices=["approach", "grasp", "swing", "both"], default="both",
                   help="approach: detect -> pre-grasp standoff, gripper untouched. "
                        "grasp: detect -> pre-grasp -> grasp -> close. "
                        "swing: sweep the ALREADY-grasped handle about the hinge (no detection). "
                        "both: grasp then swing (default, same as the microwave script).")
    a.add_argument("--approach-only", action="store_true", help="alias for --phase approach")
    a.add_argument("--reset-hinge", action="store_true",
                   help="swing: forget the persisted hinge and re-derive from the current pose "
                        "(only correct if the door is closed). Grasping resets it automatically.")
    a.add_argument("--target-angle-deg", type=float, default=20.0,
                   help="swing: how far to open, in degrees (default 20 -- keep it small the "
                        "first time so the swing DIRECTION can be checked by eye).")
    a.add_argument(
        "--steps", type=int, default=1,
        help="split EACH of the two moves (pre-grasp, grasp) into N interpolated joint "
             "sub-steps. Default 1 = single command per move, the historical behaviour. "
             "12 is the proven value on the approach script.")
    a.add_argument(
        "--max-step-deg", type=float, default=20.0,
        help="with --steps, abort if any ONE sub-step moves a joint more than this")
    a.add_argument(
        "--prompt", default=detect_handle_sam3.DEFAULT_PROMPT,
        help='SAM 3 text prompt for the handle (default "handle")')
    a.add_argument(
        "--max-detects", type=int, default=5,
        help="keep detecting until two agree within 3 cm, up to this many looks (default 5).")
    args = a.parse_args()
    if args.steps < 1:
        sys.exit("--steps must be >= 1")
    if args.approach_only:
        args.phase = "approach"

    ai = ArmInterfaceClient()
    planned_grasp = None
    if args.phase in ("approach", "grasp", "both"):
        planned_grasp = run_grasp(ai, args)
    if args.phase in ("swing", "both"):
        if args.phase == "both" and args.execute:
            print("\n--- grasp phase done. CHECK the gripper actually has the handle. ---")
            if input("Type 'swing' to start the door swing, anything else to stop: ").strip() != "swing":
                sys.exit("Stopped before the swing.")
        # dry-run of "both": plan the swing from the grasp we just planned
        run_swing(ai, args, start_override=None if args.execute else planned_grasp)


def _clean_exit(code):
    """Exit without the interpreter's own teardown.

    The shared rclpy executor runs in a daemon thread; at interpreter exit that
    thread is killed mid-call inside rclpy's C++ and the process aborts with
    "terminate called without an active exception" / core dump -- AFTER all the
    work is done, so it is harmless but alarming. Shut the node down explicitly
    first, then os._exit so the C++ side is never torn down underneath it.
    """
    sys.stdout.flush(); sys.stderr.flush()
    try:
        from feeding_deployment.ros2.node import shutdown
        shutdown()
    except Exception:
        pass
    os._exit(code)


if __name__ == "__main__":
    _code = 0
    try:
        main()
    except SystemExit as e:  # sys.exit("message") is how every gate reports a refusal
        if isinstance(e.code, int) or e.code is None:
            _code = e.code or 0
        else:
            print(e.code, file=sys.stderr); _code = 1
    _clean_exit(_code)
