"""Combined, robustified grasp + door-swing for the microwave on `rchi-cpu-5`.

Organizes two SEPARATELY-VALIDATED pieces from the 2026-09-19 session into one
file, with the "right" ROS 2 data path for each (unchanged from what was
proven tonight -- nothing about the perception/motion math below was
reimplemented):

* GRASP: byte-identical constants/gates/execution to
  `real_gen3_ros2_yolo_grasp_microwave.py` (YOLO26-backed
  `AppliancePerception.detect_handle_and_placement`, all the tuned
  corrections -- GRIP_EXT, VERTICAL_CORR, GRASP_X_ADJUST -- and the
  IK/reach/jump gates), reading RGB-D via `RealSenseROS2Interface`, i.e. ROS 2
  topics `/camera/color/image_raw`, `/camera/color/camera_info`,
  `/camera/aligned_depth_to_color/image_raw` (already the right topics --
  see that file's own docstring for why no new subscription was needed).
* SWING: the pattern validated tonight as a genuine "smooth take" workaround
  for the still-broken `JointTrajectoryCommand` (see the
  joint-trajectory-command-bug finding) -- per-waypoint seeded IK computed
  fresh from the arm's ACTUAL current joints each step (not planned once
  upfront), with a joint-jump guard, a tracking-abort, and -- new tonight,
  the actual fix for the recurring "arm looks like it's at its limit"
  mystery -- a PROACTIVE J6 angle check that stops the swing cleanly before
  a waypoint would push J6 past its real ~119.7 deg hard limit, rather than
  waiting for the growing-error pattern to show up.

Robustness added here, deliberately NOT changing any validated math/gate:
* Detection retries on an outright empty read (`NO DETECTION`) instead of
  hard-exiting -- this session needed anywhere from 1 to 5+ manual re-runs
  of the grasp script due to viewpoint-dependent YOLO misclassification
  (the microwave read as "train"/"bus" at some angles); that retry now
  happens INSIDE the script instead of requiring the operator to notice the
  exit and re-invoke it. Bounded by `--max-detect-retries` (default 8); the
  existing "two independent detections must agree" self-consistency gate
  (`DETECT_AGREE`) is untouched.
* A gripper-open check before grasping and a gripper-closed check before
  swinging (each refuses with a clear message rather than silently doing
  the wrong thing).

Deliberately NOT automated: chaining grasp -> swing without a human grip
check in between. `gripper_pos` cannot tell a real grasp from an empty
close on this rig (documented repeatedly, e.g. ~0.99 while genuinely
holding the door, but also seen just as high on a miss) -- so `--phase
both` still hard-pauses with an `input()` prompt after the grasp, exactly
like every session this constant has been rediscovered has insisted on. If
you want to skip pieces (resume a swing on an already-grasped handle after
a session restart, etc.), use `--phase swing` on its own.

**Not yet run end-to-end as a single file.** Every gate, constant, and math
step is carried over verbatim from a script that WAS run and worked
tonight (grasp: `real_gen3_ros2_yolo_grasp_microwave.py`, 4+ successful real
grasps this session; swing: the inline continuous-motion script run twice,
once for a clean 50 deg batch and once for a clean ~62 deg take that
self-stopped on the new J6 guard) -- so this is a reorganization + the
retry robustness above, not new logic. Dry-run (`--execute` omitted) before
trusting it on hardware, same as always.

`FIXED_HINGE` below is this SESSION's corroborated hinge estimate for
`rchi-cpu-5` -- it drifted from the 09-09 session's own ~35cm-radius value
(this session's fresh grasp point gave ~25-27cm to the same fixed point,
which held stable across three separate real swings tonight, so it's
trustworthy as a LOCAL pivot even though its absolute position likely isn't
the true hinge -- CLAUDE.md documents this door is not a simple single-axis
pivot). Re-derive with `scripts/scratch/left_edge_hinge.py` if the
microwave has moved since.
"""
import argparse, json, os, sys, time
from pathlib import Path

import numpy as np
import pybullet as p
import supervision as sv
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses
from ultralytics import YOLO

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import (
    CartesianTrajectoryCommand, CloseGripperCommand, JointCommand, OpenGripperCommand)
from feeding_deployment.perception.appliance_perception.appliance_perception import AppliancePerception
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

from door_push import (DoorFrame, _plan_open_axes, _plan_open_demo, _plan_retreat_over, add_push_args, push_open,
                       record_door_angle, retreat_over)
from microwave_common import DOOR_FILE, J4_GUARD_DEG, plan_cartesian, run_cartesian_trajectory, release_and_back_off, save_door_geometry

ARM = [1, 2, 3, 4, 5, 6, 7]

# ============================================================================
# GRASP constants -- verbatim from real_gen3_ros2_yolo_grasp_microwave.py.
# See that file's own comment trail for the full history of each value; kept
# here unabridged so this file is a complete, standalone record.
# ============================================================================
COCO_MICROWAVE_CLASS_ID = 68
YOLO_MODEL = "yolo26s.pt"
# COCO classes YOLO confuses this microwave with (bus 5, train 6, suitcase 28, oven 69,
# refrigerator 72), used only when it finds no 'microwave' at all. Empty = off.
YOLO_FALLBACK_CLASS_IDS = [5, 6, 28, 69, 72]


class _YoloGroundingDinoAdapter:
    """Duck-types groundingdino.util.inference.Model's predict_with_classes
    interface using YOLO's closed-set COCO detection instead of a text
    prompt. `classes` only labels which entry of the CALLER's list (e.g.
    ["microwave handle"]) the detection is reported as -- it never steers
    YOLO's own detection."""

    def __init__(self, model_name=YOLO_MODEL):
        self._model = YOLO(model_name)

    def predict_with_classes(self, image, classes, box_threshold, text_threshold):
        del classes, text_threshold
        results = self._model.predict(
            image, classes=[COCO_MICROWAVE_CLASS_ID], conf=box_threshold, verbose=False,
        )
        boxes = results[0].boxes
        if (boxes is None or len(boxes) == 0) and YOLO_FALLBACK_CLASS_IDS:
            # YOLO often calls this red microwave a bus/train/suitcase from some views (and
            # a person reflected in the glass door makes it worse). Take its best box among
            # those confusions -- it only picks the image region; the plane fit, handle
            # cluster, two-looks agreement and plausibility box all still have to pass.
            results = self._model.predict(
                image, classes=YOLO_FALLBACK_CLASS_IDS, conf=box_threshold, verbose=False,
            )
            boxes = results[0].boxes
            if boxes is not None and len(boxes) > 0:
                best = int(boxes.conf.argmax())
                names = results[0].names
                print(f"  (YOLO fallback: no 'microwave'; using '{names[int(boxes.cls[best])]}' "
                      f"{float(boxes.conf[best]):.2f} as the appliance box)")
                boxes = boxes[best:best + 1]
        if boxes is None or len(boxes) == 0:
            return sv.Detections(
                xyxy=np.zeros((0, 4), dtype=float),
                confidence=np.zeros((0,), dtype=float),
                class_id=np.zeros((0,), dtype=int),
            )
        return sv.Detections(
            xyxy=boxes.xyxy.cpu().numpy(),
            confidence=boxes.conf.cpu().numpy(),
            class_id=np.zeros(len(boxes), dtype=int),
        )


class _YoloGroundedSamShim:
    """Stands in for GroundedSAM -- AppliancePerception only ever touches
    `.grounding_dino_model` on the appliance/handle path."""

    def __init__(self):
        self.grounding_dino_model = _YoloGroundingDinoAdapter()


STALE_TRUTH = np.array([0.6932, -0.1171, 0.5031])  # reporting only, not gated on
DETECT_AGREE = 0.03
PLAUSIBLE_X = (0.30, 0.85)   # 0.45 -> 0.30 (2026-09-23): a ~50-deg-open door's handle sits at x ~0.41
PLAUSIBLE_Y = (-0.50, 0.20)   # -0.40 -> -0.50 (2026-09-28): microwave placed further right, handle y -0.401
PLAUSIBLE_Z = (0.15, 0.65)   # 0.25 -> 0.15 (2026-09-26): microwave on a lower surface, handle z ~0.21
PRE_STANDOFF = 0.12
GRIP_EXT = -0.010          # local-frame, along the approach axis
VERTICAL_CORR = 0.025      # world-frame z bias
LATERAL = 0.0              # no lateral correction -- see history in the source script
GRASP_X_ADJUST = 0.040     # 0.045 -> 0.035 -> 0.040 (2026-09-23, user-tuned on the arm)
# 2026-09-19: bumped 0.91 -> 0.915 at the user's direction after two consecutive
# live grasp targets (0.910, 0.911) both failed this gate by a hair with
# GRASP_X_ADJUST=0.045 -- consistent, not detection noise.
MAX_REACH, MIN_Z, MAX_Z = 0.915, 0.15, 0.75   # MIN_Z 0.25 -> 0.15 (2026-09-26, user-approved): lower microwave, handle z ~0.21
GRASP_MAX_IK_ERR, GRASP_MAX_JUMP_DEG = 0.02, 90.0
GRASP_TRACK_ABORT = 0.03
# Largest yaw away from the head-on default the squared-to-face grasp will accept (an
# open door is ~50 deg; beyond this the plane fit is more likely wrong than the door).
MAX_FACE_YAW_DEG = 70.0

# ============================================================================
# SWING constants -- validated tonight (2026-09-19), this session's own
# corroborated local hinge estimate + the J6 guard that actually explains the
# "growing error near wide swings" pattern seen in earlier sessions.
# ============================================================================
FIXED_HINGE = np.array([0.7177, 0.0699, 0.5585])
SWING_WAYPOINT_SPACING_M = 0.02
SWING_DIRECTION = -1
SWING_MAX_JUMP_DEG = 25.0
SWING_TRACK_ABORT = 0.02
SWING_MAX_IK_ERR = 0.02
SWING_MAX_REACH = 0.90
# J6's real hard limit is +-119.7 deg (+-2.09 rad, from the Gen3 URDF). Stop
# BEFORE commanding a waypoint that would cross this -- proactive, not a
# reaction to a growing-error pattern after the fact.
J6_LIMIT_DEG = 119.7
J6_GUARD_DEG = 115.0
# --smooth pre-check uses a deliberately modest 200-iteration IK as a singularity probe: where it
# can't get within 1 cm, Kortex's Cartesian trajectory aborted with SINGULARITY_REGION (09-23).
SMOOTH_SINGULARITY_PROBE_M = 0.01
# --swing-max never asks for more than this: the door's own stop is somewhere around 100-110 deg
# (09-25 notes) and the smooth swing has no tracking abort mid-trajectory, so don't pull into it
SWING_MAX_TRY_DEG = 90.0


def _pose_to_matrix(pose):
    m = np.zeros((4, 4))
    m[:3, 3] = pose[0]
    m[:3, :3] = R.from_quat(pose[1]).as_matrix()
    m[3, 3] = 1
    return m


_SIM = None


def _get_sim():
    """One PyBullet sim per process (grasp + swing in one run share it)."""
    global _SIM
    if _SIM is None:
        scene = create_scene_description_from_config(
            "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
        _SIM = (scene, FeedingDeploymentPyBulletSimulator(scene, use_gui=False))
    return _SIM


def _swing_waypoints(start_pose, hinge, target_deg):
    """Door-arc waypoints (pos + quat lists) from `start_pose` about `hinge`; returns (wps, radius)."""
    from feeding_deployment.interfaces.perception_interface import PerceptionInterface
    radius = float(np.linalg.norm(np.asarray(start_pose.position) - hinge))
    wps_pose = PerceptionInterface._generate_door_arc_waypoints(
        None, start_pose=start_pose, hinge_position=tuple(hinge),
        arc_length_m=radius * np.radians(target_deg), waypoint_spacing_m=SWING_WAYPOINT_SPACING_M,
        direction=SWING_DIRECTION, rotate_orientation=True)
    return [list(w.position) + list(w.orientation) for w in wps_pose], radius


def _check_arc(wps, q0):
    """The --smooth pre-check: chained sim IK over every swing waypoint from joints `q0`, gated
    on the singularity probe, reach, joint jump and J6. Returns (ok, q_end, message)."""
    scene, sim = _get_sim()
    rb = sim.robot
    q = np.asarray(q0, dtype=float)
    for i, w in enumerate(wps):
        pos, quat = w[:3], w[3:]
        for j, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=rb.physics_client_id)
        wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
        sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
            list(wpose.position), list(wpose.orientation),
            physicsClientId=rb.physics_client_id, maxNumIterations=200)
        nq = np.array([sol[k] for k in range(7)])
        for j, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(nq[j]), physicsClientId=rb.physics_client_id)
        ikerr = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wpose.position))
        jump = float(np.max(np.degrees(np.abs(nq - q))))
        j6 = float(np.degrees(nq[5]))
        if (ikerr > SMOOTH_SINGULARITY_PROBE_M or np.linalg.norm(pos) > SWING_MAX_REACH
                or jump > SWING_MAX_JUMP_DEG or abs(j6) > J6_GUARD_DEG
                or abs(np.degrees(nq[3])) > J4_GUARD_DEG):
            return False, q, (f"waypoint {i + 1}: ik_err {ikerr * 100:.2f}cm, reach {np.linalg.norm(pos):.3f}, "
                              f"jump {jump:.1f}deg, J6 {j6:.1f}deg")
        q = nq
    return True, q, f"all {len(wps)} waypoints OK, final J6 {np.degrees(q[5]):.1f}deg"


def _transfer_hinge(prev, h, n):
    """Carry the last run's hinge over to this detection in the DOOR's frame (depth along the
    normal + distance along the face from the handle), so a turned microwave is handled too --
    the 09-25 note: translation-only shifting was 3 cm off after a ~9 deg turn."""
    n0 = np.asarray(prev["closed_normal"], float)[:2]
    n0 /= np.linalg.norm(n0)
    t0 = np.array([-n0[1], n0[0]])
    d = np.asarray(prev["hinge"], float) - np.asarray(prev["closed_handle"], float)
    a, b = float(d[:2] @ n0), float(d[:2] @ t0)
    n1 = np.asarray(n, float)[:2] / np.linalg.norm(np.asarray(n, float)[:2])
    t1 = np.array([-n1[1], n1[0]])
    xy = np.asarray(h, float)[:2] + a * n1 + b * t1
    return np.array([xy[0], xy[1], float(h[2]) + float(d[2])])


def _one_take_preflight(args, st, pre, grasp, q_grasp, h, n):
    """--one-take: plan and sim-check EVERYTHING before the first motion -- the blended grasp
    path, the hinge, the swing from the planned grasp, and (with --push-open-after) the
    release + back/right/forward/left push-open. Any failure exits here, arm untouched."""
    scene, sim = _get_sim()
    rb = sim.robot
    print("\n--- one-take preflight (all checked before any motion) ---")
    ee0 = np.asarray(st["ee_pos"], dtype=float)
    q0 = np.asarray(st["position"], dtype=float)
    pre_p, pre_q = np.asarray(pre.position), np.asarray(pre.orientation)
    g_p, g_q = np.asarray(grasp.position), np.asarray(grasp.orientation)
    if args.via_pregrasp:
        leg1 = plan_cartesian(scene, rb, ee0[:3], ee0[3:7], pre_p, pre_q, q0, "to pre-grasp")
        leg2 = (plan_cartesian(scene, rb, pre_p, pre_q, g_p, g_q, leg1[-1], "to grasp")
                if leg1 is not None else None)
    else:
        # default (09-28, user's call): the grasp pose as the ONLY waypoint -- one straight line from
        # the view pose, hand rotating on the way, no corners (every corner made Kortex slow or stop)
        d = g_p - ee0[:3]
        off_axis = float(np.degrees(np.arccos(np.clip(
            d @ R.from_quat(g_q).as_matrix()[:, 2] / np.linalg.norm(d), -1, 1))))
        print(f"direct grasp: {np.linalg.norm(d) * 100:.0f} cm straight to the handle, "
              f"{off_axis:.0f} deg off the approach axis")
        leg2 = plan_cartesian(scene, rb, ee0[:3], ee0[3:7], g_p, g_q, q0, "to grasp (direct)")
    args.grasp_blended = leg2 is not None
    if not args.grasp_blended:
        # usually a free joint (J1/J3/J5/J7) crossing +-180 on the way in (Kortex's wrap behaviour
        # is unverified), or J4 running into its +-147.8 soft limit (09-28: Kortex aborted mid-path)
        q_now = np.degrees(q0)
        sys.exit("blended grasp path fails a gate (see the step above) -- refusing before any motion. "
                 f"Start joints {np.round(q_now, 1)}: start from a view pose with |J4| well under "
                 f"{J4_GUARD_DEG:.0f} and J1/J3/J5/J7 well away from +-180.")
    q_g = leg2[-1]

    if args.hinge is not None:
        hinge, src = np.array(args.hinge, dtype=float), "--hinge"
    else:
        prev = args.prev_door
        if not all(k in prev for k in ("hinge", "closed_handle", "closed_normal")):
            sys.exit("--one-take needs --hinge or a previous hinge/closed_handle/closed_normal in the door file.")
        hinge, src = _transfer_hinge(prev, h, n), "last hinge carried over in the door frame"
    radius = float(np.linalg.norm(g_p - hinge))
    print(f"hinge ({src}): {np.round(hinge, 4)}  radius {radius * 100:.1f} cm")
    if not 0.25 <= radius <= 0.45:
        sys.exit(f"hinge radius {radius * 100:.1f} cm is implausible (door ~35 cm) -- refusing.")
    args.derived_hinge = hinge

    door = {"hinge": hinge.tolist(), "closed_grasp_pos": g_p.tolist(),
            "closed_normal": np.asarray(n, float).tolist(), "closed_handle": np.asarray(h, float).tolist()}
    for k in ("door_z", "door_mid"):
        if getattr(args, k, None) is not None:
            door[k] = np.asarray(getattr(args, k), float).tolist()

    def retreat_ok(wps, q_end):
        st_end = {"ee_pos": np.asarray(wps[-1], dtype=float), "position": q_end}
        return _plan_retreat_over(scene, rb, door, st_end, args) is not None

    if args.swing_max:
        # largest angle (SWING_MAX_TRY_DEG down to 45, 5-deg steps) whose whole arc passes the sim gates
        # -- and, with --retreat-over, from whose end the retreat passes too (J6 ends near its guard at
        # 90 deg, and backing off the handle raises it further: 09-28 dry run)
        for ang in range(int(SWING_MAX_TRY_DEG), 44, -5):
            wps, _ = _swing_waypoints(grasp, hinge, ang)
            ok, q_end, msg = _check_arc(wps, q_g)
            if ok and args.retreat_over and not retreat_ok(wps, q_end):
                ok, msg = False, "retreat-over from its end fails a gate"
            print(f"  --swing-max: {ang} deg {'OK' if ok else 'fails -- ' + msg}")
            if ok:
                args.target_angle_deg = float(ang)
                break
    wps, _ = _swing_waypoints(grasp, hinge, args.target_angle_deg)
    ok, q_end, msg = _check_arc(wps, q_g)
    print(f"swing {args.target_angle_deg:.0f} deg from the planned grasp: {'OK' if ok else 'FAILED'} -- {msg}")
    if not ok:
        sys.exit("swing pre-check failed -- refusing before any motion.")

    if args.retreat_over:
        if not retreat_ok(wps, q_end):
            sys.exit("retreat-over (out / up / across / straighten / down) fails a gate -- refusing before any motion.")
        print("retreat-over OK from the predicted end of the swing")
    if args.push_open_after:
        ang = DoorFrame(door, 1.0).angle_of(wps[-1][:3])
        df = DoorFrame(door, np.sign(ang) or 1.0)
        st_end = {"ee_pos": np.asarray(wps[-1], dtype=float), "position": q_end}
        plan = (_plan_open_demo if args.push_style == "demo" else _plan_open_axes)(
            scene, rb, df, st_end, args, abs(ang))
        if plan is None:
            sys.exit("push-open (back / right / forward / left) fails a gate -- refusing before any motion.")
        print(f"push-open ({args.push_style}) OK from the door at ~{abs(ang):.0f} deg")
    print("--- preflight OK ---\n")


def run_grasp(ai, args):
    st = ai.get_state()
    ee0 = np.asarray(list(st["ee_pos"])[:3], dtype=float)
    g0 = float(st.get("gripper_pos"))
    print(f"start EE {np.round(ee0, 4)}  gripper {g0:.4f}")
    if g0 > 0.2:
        # open it rather than refuse (user, 09-28): a closed gripper at the view pose just means it
        # was left closed after the last reset
        if not args.execute:
            print("gripper is closed -- the real run opens it first (dry run: not commanded)")
        else:
            print("gripper is closed -- opening it before the grasp ...")
            ai.execute_command(OpenGripperCommand())
            time.sleep(1.5)
            g0 = float(ai.get_state().get("gripper_pos"))
            if g0 > 0.2:
                sys.exit(f"Gripper still reads {g0:.3f} after opening -- refusing to grasp.")

    rs = RealSenseROS2Interface()
    if not rs.wait_for_frames(30.0):
        sys.exit("No RGB-D frames")
    _log_dir = os.environ.get("DETECTION_LOG_DIR")
    _data_logger = None
    if _log_dir:
        from feeding_deployment.integration.data_logger import DataLogger
        _data_logger = DataLogger(Path(_log_dir), day=1)
        _data_logger.begin_hla("grasp_microwave_yolo")
        print(f"detection logging -> {_log_dir}")
    apc = AppliancePerception(_YoloGroundedSamShim(), data_logger=_data_logger)
    depth_corr, _lat_corr = apc.handle_depth_corr, apc.handle_lat_corr
    if depth_corr == 0.0:
        sys.exit("HANDLE_DEPTH_CORR not set -- refusing.")

    def detect_once(tag):
        """Unlike the source script, retries on an outright empty read
        (bounded by --max-detect-retries) instead of exiting immediately --
        this was the single biggest practical friction point tonight
        (viewpoint-dependent YOLO misreads needing manual re-runs)."""
        for attempt in range(1, args.max_detect_retries + 1):
            d = rs.get_camera_data()
            hh, _, _, top = apc.detect_handle_and_placement(
                "microwave handle", d["rgb_image"], d["camera_info"], d["depth_image"])
            if hh is not None:
                break
            print(f"  no detection on look {tag} (empty-read retry {attempt}/{args.max_detect_retries})")
            time.sleep(0.5)
        else:
            sys.exit(f"NO DETECTION ({tag}) after {args.max_detect_retries} retries")
        top_z_val = float(np.asarray(top.position)[2]) if top is not None else float("nan")
        v = np.asarray(hh.position)
        n = apc.last_door_normal_base
        zr, mid = apc.last_door_z_range_base, apc.last_door_mid_base
        print(f"  detect {tag}: {np.round(v, 4)}   top_of_appliance z {top_z_val:.4f}   "
              f"door normal {np.round(n, 3) if n is not None else None}   "
              f"door z {np.round(zr, 3) if zr is not None else None}   "
              f"door mid {np.round(mid, 3) if mid is not None else None}")
        return hh, v, top_z_val, n, zr, mid

    dets, pair = [], None
    for i in range(1, args.max_detects + 1):
        dets.append(detect_once(str(i)))
        for j in range(len(dets) - 1):
            spread = float(np.linalg.norm(dets[j][1] - dets[-1][1]))
            if spread <= DETECT_AGREE:
                pair = (dets[j], dets[-1], spread)
                break
        if pair:
            break
        if args.fast:
            pair = (dets[0], dets[0], 0.0)   # --fast: one look, no two-looks agreement check
            break
        if i > 1:
            print(f"  no pair within {DETECT_AGREE * 100:.0f} cm yet after {i} looks; re-detecting")
    if pair is None:
        sys.exit(f"No two of {len(dets)} detections agreed within {DETECT_AGREE * 100:.0f} cm -- refusing.")
    (_, ha, tza, na, zra, mida), (hbo, hb, tzb, nb, zrb, midb), spread = pair
    print(f"detections agree to {spread * 100:.1f} cm (limit {DETECT_AGREE * 100:.0f}), used {len(dets)} look(s)")
    h = (ha + hb) / 2.0
    # door face extent (for the door model and --retreat-over), averaged over both looks
    zrs = [np.asarray(z, float) for z in (zra, zrb) if z is not None]
    mids = [np.asarray(m, float) for m in (mida, midb) if m is not None]
    door_z = np.mean(zrs, axis=0) if zrs else None
    door_mid = np.mean(mids, axis=0) if mids else None
    if door_z is not None and not (door_z[0] < h[2] < door_z[1] and 0.15 <= door_z[1] - door_z[0] <= 0.60):
        print(f"door z range {np.round(door_z, 3)} doesn't fit a door around the handle (z {h[2]:.3f}) "
              "-- not using it (door model falls back to the generous default)")
        door_z = None
    print(f"door face z {np.round(door_z, 3) if door_z is not None else None}  "
          f"mid {np.round(door_mid, 3) if door_mid is not None else None}")
    args.door_z, args.door_mid = door_z, door_mid
    handle = hbo
    top_z = float(np.nanmean([tza, tzb]))
    print(f"top_of_appliance z {top_z:.4f} -> post-release lift target {top_z + 0.05:.4f}")
    Path("/tmp/microwave_top_z.txt").write_text(str(top_z))
    for nm, v, lo_hi in (("x", h[0], PLAUSIBLE_X), ("y", h[1], PLAUSIBLE_Y), ("z", h[2], PLAUSIBLE_Z)):
        if not (lo_hi[0] <= v <= lo_hi[1]):
            sys.exit(f"handle {nm}={v:.3f} outside plausible {lo_hi} -- refusing.")
    print(f"handle (mean) {np.round(h, 4)}   {np.linalg.norm(h - STALE_TRUTH) * 100:.1f} cm from the STALE pre-arc truth (informational)")
    print(f"camera->handle {np.linalg.norm(h - ee0) * 100:.0f} cm (viewing distance)")
    h = h + np.array([0.0, 0.0, VERTICAL_CORR])
    print(f"handle (vertical-corrected) {np.round(h, 4)}")

    quat = (R.from_quat(np.asarray(handle.orientation)) * R.from_euler("y", np.pi)).as_quat()
    # Square the approach to the door face. The detector's pose orientation is a fixed
    # constant (right only for a door facing the arm head-on); yaw it about world z so the
    # approach axis points straight into the face (-normal). Yaw only, so the validated
    # roll -- fingers closing across the vertical bar -- is kept.
    if na is None or nb is None:
        sys.exit("Detector returned no door normal -- cannot square the grasp to the face. Refusing.")
    n = na + nb
    n = n / np.linalg.norm(n)
    z0 = R.from_quat(quat).as_matrix()[:, 2]
    into = -n
    yaw = float(np.arctan2(z0[0] * into[1] - z0[1] * into[0], z0[0] * into[0] + z0[1] * into[1]))
    print(f"door normal {np.round(n, 3)} (two looks agree to "
          f"{np.degrees(np.arccos(np.clip(np.dot(na, nb), -1, 1))):.1f} deg); "
          f"default approach {np.round(z0, 3)} -> yaw {np.degrees(yaw):+.1f} deg to face the door square")
    if abs(np.degrees(yaw)) > MAX_FACE_YAW_DEG:
        sys.exit(f"Face yaw {np.degrees(yaw):.1f} deg > {MAX_FACE_YAW_DEG} deg -- implausible plane fit? Refusing.")
    quat = (R.from_euler("z", yaw) * R.from_quat(quat)).as_quat()
    approach = R.from_quat(quat).as_matrix()[:, 2]
    hp = Pose(tuple(h), tuple(quat))

    def off(v):
        m = np.eye(4)
        m[:3, 3] = v
        out = _pose_to_matrix(hp) @ m
        return Pose(out[:3, 3], R.from_matrix(out[:3, :3]).as_quat())

    pre = off([0.0, 0.0, -PRE_STANDOFF])
    grasp = off([LATERAL, 0.0, -GRIP_EXT])
    # GRASP_X_ADJUST was tuned as world +x on a door facing the arm; with the grasp squared
    # to the face it acts along the approach axis instead (identical for a head-on door).
    grasp = Pose(grasp.position + approach * GRASP_X_ADJUST, grasp.orientation)
    print(f"pre-grasp {np.round(pre.position, 4)}  range {np.linalg.norm(pre.position):.3f}")
    print(f"grasp     {np.round(grasp.position, 4)}  range {np.linalg.norm(grasp.position):.3f}")

    scene, sim = _get_sim()
    rb = sim.robot

    def solve(pose, seed):
        for i, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(seed[i]), physicsClientId=rb.physics_client_id)
        w = multiply_poses(scene.robot_base_pose, pose)
        sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
            list(w.position), list(w.orientation), maxNumIterations=400,
            residualThreshold=1e-5, physicsClientId=rb.physics_client_id)
        q = np.asarray(sol[:7])
        for i, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(q[i]), physicsClientId=rb.physics_client_id)
        ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
        return q, float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(w.position)))

    cur = np.asarray(st["position"], dtype=float)
    plan = []
    for name, pose in (("pre-grasp", pre), ("grasp", grasp)):
        t = np.asarray(pose.position)
        bad = []
        if np.linalg.norm(t) > MAX_REACH:
            bad.append(f"range {np.linalg.norm(t):.3f}>{MAX_REACH}")
        if not (MIN_Z <= t[2] <= MAX_Z):
            bad.append(f"z {t[2]:.3f} out of range")
        q, err = solve(pose, cur)
        jump = float(np.degrees(np.max(np.abs(q - cur))))
        if err > GRASP_MAX_IK_ERR:
            bad.append(f"IK err {err * 100:.1f}cm")
        if jump > GRASP_MAX_JUMP_DEG:
            bad.append(f"jump {jump:.0f}deg")
        if abs(np.degrees(q[3])) > J4_GUARD_DEG:
            bad.append(f"J4 {np.degrees(q[3]):.1f}deg past the {J4_GUARD_DEG} guard (soft limit 147.8)")
        print(f"  {name:10s} IK {err * 100:5.2f} cm  jump {jump:5.1f} deg  {'FAIL: ' + '; '.join(bad) if bad else 'OK'}")
        if bad:
            sys.exit(f"GATE FAILED at {name}")
        plan.append((name, t, q))
        cur = q

    if getattr(args, "one_take", False):
        _one_take_preflight(args, st, pre, grasp, plan[-1][2], h, n)

    def fk(joints):
        for i, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(joints[i]), physicsClientId=rb.physics_client_id)
        ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
        return np.asarray(ls[4]) - np.asarray(scene.robot_base_pose.position)

    if args.steps > 1:
        chained, prev = [], np.asarray(st["position"], dtype=float)
        print(f"\nchaining each move into {args.steps} joint sub-steps (guard {args.max_step_deg} deg/step):")
        for name, t, q in plan:
            total = (q - prev + np.pi) % (2 * np.pi) - np.pi
            base_q, worst = prev.copy(), 0.0
            for s in range(1, args.steps + 1):
                qs = base_q + total * (s / args.steps)
                delta = float(np.degrees(np.max(np.abs(qs - prev))))
                worst = max(worst, delta)
                if delta > args.max_step_deg:
                    sys.exit(f"Sub-step {s} of '{name}' moves {delta:.0f} deg (> {args.max_step_deg}) -- refusing.")
                chained.append((f"{name} {s:02d}/{args.steps}", fk(qs), qs))
                prev = qs
            end_err = float(np.linalg.norm(fk(prev) - t))
            print(f"  {name:10s} {args.steps} steps, worst {worst:5.1f} deg  endpoint {end_err * 100:.3f} cm from the gated target")
            if end_err > GRASP_MAX_IK_ERR:
                sys.exit(f"Chain for '{name}' ends {end_err * 100:.1f} cm off target -- refusing.")
        plan = chained

    if not args.execute:
        print("\nDRY RUN (grasp) -- nothing commanded.")
        return False

    if getattr(args, "grasp_blended", False):
        if args.via_pregrasp:
            # pre-grasp is the first waypoint, so Kortex stops there briefly (the first waypoint
            # gets no blend), then goes straight in along the approach axis
            traj = [(list(pre.position), list(pre.orientation)), (list(grasp.position), list(grasp.orientation))]
        else:
            traj = [(list(grasp.position), list(grasp.orientation))]
        print(f"\nspeed: {ai.get_speed()}  -- one blended Cartesian motion into the grasp ...")
        _, e, ok = run_cartesian_trajectory(ai, traj)
        fin = np.asarray(list(ai.get_state()["ee_pos"])[:3], dtype=float)
        print(f"  grasp (blended) EE {np.round(fin, 4)}  tracking {e * 100:4.1f} cm  (RPC returned {ok}; judged by position)")
        if e > GRASP_TRACK_ABORT:
            sys.exit(f"Tracking {e * 100:.1f} cm at the grasp -- ABORT, gripper untouched.")
        plan = []
    else:
        print(f"\nspeed: {ai.get_speed()}  -- commanding {len(plan)} move(s) ...")
    for name, t, q in plan:
        ai.execute_command(JointCommand(pos=q.tolist()))
        for _ in range(120):
            time.sleep(0.2)
            s = ai.get_state()
            if float(np.max(np.abs(np.asarray(s["velocity"], dtype=float)))) < 1e-3:
                break
        time.sleep(0.4)
        fin = np.asarray(list(ai.get_state()["ee_pos"])[:3], dtype=float)
        e = float(np.linalg.norm(fin - t))
        print(f"  {name:18s} EE {np.round(fin, 4)}  tracking {e * 100:4.1f} cm")
        if e > GRASP_TRACK_ABORT:
            sys.exit(f"Tracking {e * 100:.1f} cm at {name} -- ABORT, gripper untouched.")

    # closed-door geometry for the close task's door model (re-grasp path checks)
    # door_z/door_mid are written even when None, so a stale value from an earlier run can't linger
    save_door_geometry(closed_normal=n, closed_grasp_pos=np.asarray(grasp.position), closed_handle=h,
                       door_z=door_z, door_mid=door_mid)
    print("\nclosing gripper ...")
    ai.execute_command(CloseGripperCommand())
    if getattr(args, "one_take", False):
        # move on as soon as the fingers stop closing (up to the old fixed 3.5 s)
        t0, last = time.time(), -1.0
        time.sleep(0.6)
        while time.time() - t0 < 3.5:
            g = float(ai.get_state().get("gripper_pos"))
            if abs(g - last) < 0.003:
                break
            last = g
            time.sleep(0.2)
    else:
        time.sleep(3.5)
    gf = float(ai.get_state().get("gripper_pos"))
    print(f"gripper after close: {gf:.4f}")
    print("Gripper closed. gripper_pos cannot confirm a grasp on this rig --")
    print("CHECK VISUALLY before running the swing.")
    return True


def run_swing(ai, args):
    st = ai.get_state()
    g = float(st.get("gripper_pos"))
    if g < 0.2:
        sys.exit("Gripper is OPEN -- nothing grasped. Refusing to run the swing.")

    ee = list(st["ee_pos"])
    grasp_pose = Pose(position=tuple(ee[:3]), orientation=tuple(ee[3:7]))
    if args.hinge is not None:
        hinge = np.array(args.hinge)
    elif getattr(args, "derived_hinge", None) is not None:
        hinge = args.derived_hinge
    else:
        hinge = FIXED_HINGE
    radius = float(np.linalg.norm(np.array(ee[:3]) - hinge))
    arc_length_m = radius * np.radians(args.target_angle_deg)
    print(f"current ee_pos: {np.round(ee[:3], 4)}  gripper {g:.4f}")
    src = "--hinge" if args.hinge is not None else ("last hinge + grasp shift" if getattr(args, "derived_hinge", None) is not None else "fixed")
    print(f"hinge ({src}): {np.round(hinge, 4)} (radius {radius * 100:.1f}cm), "
          f"arc_length {arc_length_m * 100:.1f}cm for {args.target_angle_deg}deg")

    from feeding_deployment.interfaces.perception_interface import PerceptionInterface
    wps_pose = PerceptionInterface._generate_door_arc_waypoints(
        None, start_pose=grasp_pose, hinge_position=tuple(hinge),
        arc_length_m=arc_length_m, waypoint_spacing_m=SWING_WAYPOINT_SPACING_M,
        direction=SWING_DIRECTION, rotate_orientation=True)
    wps = [list(w.position) + list(w.orientation) for w in wps_pose]
    print(f"{len(wps)} waypoints planned: first {np.round(wps[0][:3], 3)} -> last {np.round(wps[-1][:3], 3)}")

    if not args.execute:
        print("\nDRY RUN (swing) -- nothing commanded.")
        return
    save_door_geometry(hinge=hinge)

    scene, sim = _get_sim()
    rb = sim.robot

    smooth_ok = False
    if args.smooth:
        # Check the WHOLE arc in sim first (chained IK from the real joints, same gates as
        # the step loop), then hand it to Kortex as one blended Cartesian waypoint
        # trajectory -- one continuous motion, like the lab's open_microwave(). Kortex does
        # its own IK for this, so the sim chain is a reachability/limit proxy, not a guarantee.
        q = np.array(ai.get_state()["position"], dtype=float)
        for i, w in enumerate(wps):
            pos, quat = w[:3], w[3:]
            for j, jj in enumerate(ARM):
                p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=rb.physics_client_id)
            wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
            sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
                list(wpose.position), list(wpose.orientation),
                physicsClientId=rb.physics_client_id, maxNumIterations=200)
            nq = np.array([sol[k] for k in range(7)])
            for j, jj in enumerate(ARM):
                p.resetJointState(rb.robot_id, jj, float(nq[j]), physicsClientId=rb.physics_client_id)
            ikerr = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wpose.position))
            jump = float(np.max(np.degrees(np.abs(nq - q))))
            j6 = float(np.degrees(nq[5]))
            if (ikerr > SMOOTH_SINGULARITY_PROBE_M or np.linalg.norm(pos) > SWING_MAX_REACH
                    or jump > SWING_MAX_JUMP_DEG or abs(j6) > J6_GUARD_DEG
                or abs(np.degrees(nq[3])) > J4_GUARD_DEG):
                print(f"smooth pre-check FAILED at waypoint {i + 1}: ik_err {ikerr * 100:.2f}cm, jump "
                      f"{jump:.1f}deg, J6 {j6:.1f}deg -- likely near a wrist singularity, where Kortex's own "
                      "Cartesian IK aborts (seen 09-23). Falling back to the stop-and-go joint swing.")
                smooth_ok = False
                break
            q = nq
        else:
            smooth_ok = True
    if args.smooth and smooth_ok:
        print(f"smooth pre-check OK over all {len(wps)} waypoints (final J6 {np.degrees(q[5]):.1f}deg); "
              "sending one blended Cartesian trajectory ...")
        ok, err, rpc_ok = run_cartesian_trajectory(ai, [(w[:3], w[3:]) for w in wps])
        final = ai.get_state()
        print(f"\nSMOOTH SWING {'DONE' if ok else 'STOPPED SHORT'} (RPC returned {rpc_ok}). final EE: {np.round(final['ee_pos'][:3], 4)} "
              f"({err * 100:.1f} cm from the last waypoint), gripper: {final.get('gripper_pos')}")
        record_door_angle(final["ee_pos"])
        return

    for i, w in enumerate(wps):
        pos, quat = w[:3], w[3:]
        real_joints = np.array(ai.get_state()["position"], dtype=float)
        for j, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(real_joints[j]), physicsClientId=rb.physics_client_id)
        wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
        sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id,
            list(wpose.position), list(wpose.orientation),
            physicsClientId=rb.physics_client_id, maxNumIterations=1000, residualThreshold=1e-6)
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
        if abs(np.degrees(joints[3])) > J4_GUARD_DEG:
            print(f"  STOP: J4 would reach {np.degrees(joints[3]):.1f}deg, past the {J4_GUARD_DEG} guard.")
            break
        if abs(j6) > J6_GUARD_DEG:
            print(f"  STOP: J6 would reach {j6:.1f}deg, closing in on its "
                  f"+-{J6_LIMIT_DEG}deg hard limit -- stopping cleanly rather than "
                  f"pushing further (this is the fix for the earlier 'growing error' mystery).")
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
        if err > SWING_TRACK_ABORT:
            print(f"  ABORT: tracking err {err * 100:.1f}cm > {SWING_TRACK_ABORT * 100:.0f}cm "
                  "-- door binding / latch / natural limit.")
            break

    final = ai.get_state()
    print("\nSWING DONE. final EE:", np.round(final["ee_pos"][:3], 4), "gripper:", final.get("gripper_pos"))
    record_door_angle(final["ee_pos"])


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--execute", action="store_true", help="actually command the arm; omit for a dry-run/plan-only pass")
    a.add_argument("--phase", choices=["grasp", "swing", "both", "release", "push-open", "retreat-over"],
                   default="both",
                   help="run just the grasp, just the swing (on an already-grasped handle), or both "
                        "(with a mandatory visual-check pause in between)")
    a.add_argument("--steps", type=int, default=1,
                   help="split EACH grasp move (pre-grasp, grasp) into N interpolated joint sub-steps. "
                        "Default 1 = single clean motion (validated as a 'smooth take' pattern tonight).")
    a.add_argument("--max-step-deg", type=float, default=20.0,
                   help="with --steps > 1, abort if any ONE grasp sub-step moves a joint more than this")
    a.add_argument("--max-detects", type=int, default=5,
                   help="keep detecting until two agree within 3cm, up to this many looks")
    a.add_argument("--max-detect-retries", type=int, default=8,
                   help="NEW: retries on an outright EMPTY detection (not the two-agree check), "
                        "since a bad viewing angle intermittently misreads the microwave as another "
                        "COCO class with zero microwave candidates")
    a.add_argument("--target-angle-deg", type=float, default=50.0,
                   help="swing target, degrees of additional rotation from the grasp pose. Default 50: "
                        "at ~68deg J6 (113deg) leaves no room to back off the handle after release "
                        "(2026-09-23). 75 is the "
                        "largest single-take value tried tonight (self-stopped safely on the J6 guard "
                        "at ~62deg actual); use a smaller value for a batch you intend to complete in full.")
    a.add_argument("--back-off", type=float, default=0.20,
                   help="--phase release: open the gripper and back straight off the handle this far (m); "
                        "records the grasp so the close script's --phase regrasp can return to it")
    a.add_argument("--pre-park-x", type=float, default=0.216,
                   help="--phase release: after the back-off, go straight toward the base (world -x) to "
                        "this x before the joint move to park (clears the open door's free edge)")
    a.add_argument("--fast", action="store_true",
                   help="--phase both: one detection look, no pause between grasp and swing, swing deps "
                        "loaded up front, hinge = last hinge shifted by the grasp change (unless --hinge)")
    a.add_argument("--smooth", action="store_true",
                   help="swing: pre-check every waypoint in sim, then send the arc as ONE blended "
                        "Cartesian waypoint trajectory (continuous motion) instead of stop-and-go steps")
    a.add_argument("--hinge", type=float, nargs=3, metavar=("X", "Y", "Z"), default=None,
                   help="override FIXED_HINGE (arm_base_link, m) for this run -- use when the "
                        "microwave has moved since FIXED_HINGE was measured")
    a.add_argument("--one-take", action="store_true",
                   help="--phase both: plan + sim-check the grasp path, hinge, swing (and --push-open-after) "
                        "BEFORE any motion, then run it with no pauses: one blended motion into the grasp, "
                        "close, one smooth swing. Implies --smooth.")
    a.add_argument("--swing-max", action="store_true",
                   help="with --one-take: swing as far as the sim gates allow (J6/J4 guards, IK, reach, jumps), "
                        f"trying {SWING_MAX_TRY_DEG:.0f} deg down to 45 in 5-deg steps; overrides --target-angle-deg")
    a.add_argument("--via-pregrasp", action="store_true",
                   help="with --one-take: go through the pre-grasp point and straight in along the approach "
                        "axis (one brief stop at pre-grasp) instead of straight from the view pose to the grasp")
    a.add_argument("--push-open-after", action="store_true",
                   help="with --one-take: after the swing, release and push the door the rest of the way open "
                        "(back, right past the free edge, forward, left along the arc -- door_push.push_open)")
    a.add_argument("--retreat-over", action="store_true",
                   help="with --one-take: after the swing, release and go out, up over the door's top, across to "
                        "the microwave's middle, turn to face in, and down to handle height (door_push.retreat_over)")
    add_push_args(a)
    args = a.parse_args()
    if args.steps < 1:
        sys.exit("--steps must be >= 1")
    if args.one_take:
        if args.phase != "both":
            sys.exit("--one-take is for --phase both")
        if args.steps != 1:
            sys.exit("--one-take plans one blended grasp motion; drop --steps")
        args.smooth = True
        if args.push_style == "arc":
            args.push_style = "demo"   # the 09-28 hand demo; --push-style axes for the straight-legs planner
        if args.retreat_over and args.push_open_after:
            sys.exit("--retreat-over and --push-open-after both start from the end of the swing -- pick one")
    elif args.push_open_after or args.retreat_over:
        sys.exit("--push-open-after / --retreat-over need --one-take")

    ai = ArmInterfaceClient()

    if args.phase == "release":
        release_and_back_off(ai, args.back_off, args.execute, pre_park_x=args.pre_park_x)
        return
    if args.phase == "push-open":
        # after the pull, still holding: release, round the free edge, push the inner face on
        push_open(ai, args)
        return
    if args.phase == "retreat-over":
        # after the pull, still holding: release, out, over the door's top, across, face in, down
        retreat_over(ai, args)
        return

    if (args.fast or args.one_take) and args.phase == "both":
        # load the slow swing dependencies BEFORE the grasp, so nothing sits between
        # grasp and swing; and remember the last hinge + closed-door grasp to derive
        # this run's hinge (the grasp overwrites closed_grasp_pos in the door file)
        from feeding_deployment.interfaces.perception_interface import PerceptionInterface  # noqa: F401
        _get_sim()
        prev = json.loads(DOOR_FILE.read_text()) if DOOR_FILE.exists() else {}
        args.prev_door = prev

    if args.phase in ("grasp", "both"):
        grasped = run_grasp(ai, args)
        if args.phase == "both":
            if not args.execute:
                print("\n(dry run -- " + ("one take: grasp, swing" + (", push-open" if args.push_open_after else "")
                      + (", retreat-over" if args.retreat_over else "")
                      + " would run with no pauses)" if args.one_take
                      else "would have paused here for a visual grip check before the swing)"))
                return
            if not grasped:
                return
            if args.one_take:
                pass   # hinge already set by the preflight (args.derived_hinge), no pause
            elif args.fast:
                if args.hinge is None:
                    if "hinge" not in prev or "closed_grasp_pos" not in prev:
                        sys.exit("--fast needs --hinge or a previous hinge in the door file -- stopping (still gripping).")
                    cur = json.loads(DOOR_FILE.read_text())
                    key = "closed_handle" if "closed_handle" in prev else "closed_grasp_pos"   # handle: immune to grasp-offset tuning
                    shift = np.array(cur[key]) - np.array(prev[key])
                    shift[2] = 0.0
                    args.derived_hinge = np.array(prev["hinge"]) + shift
                    print(f"derived hinge: last {np.round(prev['hinge'], 4)} + grasp shift {np.round(shift, 4)}")
            else:
                input("\nVisually confirm the gripper is actually on the handle, then press Enter to continue to the swing (Ctrl-C to stop here) ...")

    if args.phase in ("swing", "both"):
        run_swing(ai, args)
    if args.one_take and args.push_open_after:
        # re-planned from the real post-swing state (the preflight proved it feasible); refuses
        # without releasing if the replan fails
        scene, sim = _get_sim()
        push_open(ai, args, sim=(scene, sim.robot))
    if args.one_take and args.retreat_over:
        # re-planned from the real post-swing state; refuses without releasing if the replan fails
        scene, sim = _get_sim()
        retreat_over(ai, args, sim=(scene, sim.robot))


if __name__ == "__main__":
    main()
