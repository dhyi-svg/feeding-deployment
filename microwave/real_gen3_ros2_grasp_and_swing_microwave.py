"""Open the microwave on `rchi-cpu-5`: detect the handle, grasp it, swing the door open.

    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py             # dry run
    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --execute
    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --release --execute

1. Detect (`handle_detect.py`): YOLO box around the microwave, depth -> door plane, the
   vertical cluster sticking out of it = handle, the door edge farthest from it = hinge. Looks
   until two agree within 3 cm (handle and hinge).
2. Plan everything before any motion: the grasp (squared to the door face, one straight
   Cartesian line from the view pose), and the largest swing (75 down to 45 deg) whose whole
   arc passes the sim gates (IK, reach, joint jump, J4/J6 guards).
3. Execute: one blended motion into the grasp, close, one blended swing. The hand stays on the
   handle at the end.

`--release` (separate run, after the swing): open the gripper, back straight off the handle,
go toward the base to clear the door's free edge, joint move to `park_pose.json`.

Run from the repo root (the sim config path is relative). See NOTES.md for the bring-up and env.
"""
import argparse, contextlib, io, json, sys, time
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from sheppy_arm import SheppyArm
from feeding_deployment.control.robot_controller.command_interface import CloseGripperCommand, OpenGripperCommand
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface

from door_push import record_door_angle
from handle_detect import HandleDetector
from microwave_common import (J4_GUARD_DEG, door_arc_waypoints, fit_circle_xy, make_sim, plan_cartesian,
                              release_and_back_off, run_cartesian_trajectory, save_door_geometry, save_frame)

ARM = [1, 2, 3, 4, 5, 6, 7]

# --- detection ---
MAX_LOOKS = 5              # keep looking until two detections agree within DETECT_AGREE
MAX_EMPTY_RETRIES = 8      # per look: retries on an empty read (viewpoint-dependent YOLO misses)
DETECT_AGREE = 0.03
HINGE_AGREE = 0.03         # the two agreeing looks' hinges must agree this well too (horizontal)
PLAUSIBLE_X = (0.30, 0.85)
PLAUSIBLE_Y = (-0.50, 0.20)
PLAUSIBLE_Z = (0.05, 0.65)   # was 0.15 (rchi-cpu-5); Jetson+sheppy 10-07: handle z 0.142, microwave ~4 cm above the arm base
MAX_FACE_YAW_DEG = 70.0    # beyond this the plane fit is more likely wrong than the door

# --- grasp (tuned on the arm, see NOTES.md) ---
GRIP_EXT = 0.088           # along the approach axis (negative = past the detected handle). Was -0.010 (rchi-cpu-5);
                           # Jetson+sheppy+Capybara calib 10-07 touch check: planned grasp 9.8 cm too deep
VERTICAL_CORR = 0.0        # world z. 0.025 (rchi-cpu-5) -> 0.046 (10-07 touch) -> 0: the detector now returns the
                           # MIDDLE of the bar, not a point near its bottom end
GRASP_X_ADJUST = 0.040     # along the approach axis
GRASP_PITCHES_DEG = (0, 10, 17, 25, 30, 35)   # nose-down tilts tried about the finger axis (fingers stay
                           # horizontal across the vertical bar); the smallest reaching the largest swing wins. A low
                           # handle needs one: 10-07, handle z ~0.14, level -> swing max 25-35 deg (J6 hits its guard),
                           # 17 -> 70, 25 -> 75 with J6 ending at 92 (sim); the user's hand-placed touches were ~17.
MAX_REACH, MIN_Z, MAX_Z = 0.915, 0.15, 0.75
GRASP_TRACK_ABORT = 0.03

# --- swing ---
SWING_WAYPOINT_SPACING_M = 0.02   # sim-checked spacing
SWING_SEND_EVERY = 4              # Kortex slows at every waypoint: send every 4th (8 cm chords) + the last
SWING_DIRECTION = -1
SWING_MAX_TRY_DEG = 75            # tries this down to 45 in 5-deg steps; 80 dragged the microwave (09-29)
SWING_MIN_DEG = 45
SWING_MAX_JUMP_DEG = 25.0
SWING_MAX_REACH = 0.90
SWING_IK_PROBE_M = 0.01           # 200-iteration IK must get within 1 cm (else Kortex hits SINGULARITY_REGION)
J6_GUARD_DEG = 115.0              # J6 hard limit is +-119.7
SWING_COMPLIANT = False           # swing in joint impedance (SheppyArm.IMPEDANCE): the hand gives way to the door's
                                  # real arc instead of dragging the microwave along a circle about a slightly wrong
                                  # hinge (09-29 / 10-07); the driver's path tolerance (0.08 rad) is the force stop
GRASP_FORCE = 0.25                # gripper current ceiling on the handle (SheppyArm default 0.5): a looser hold lets the
                                  # bar turn/shift in the fingers instead of tipping the microwave forward (10-07)
HINGE_BEHIND_FACE = 0.016         # real pivot vs the detected hinge (the door's visible edge): this far behind the door
HINGE_PAST_EDGE = 0.021           # face and this far beyond the edge, away from the handle. Measured 10-07 from the
                                  # hand's path in a stiff 50-deg swing (circle fit rms 0.6 mm; r 36.0 vs 33.7 cm)
SWING_STIFF_DEG = 10              # ... but the first part stiff: the latch needs a firm pull, and on 10-07 the soft
                                  # arm's lag tripped the force stop before the door opened (~5 deg)
SWING_FIT_MIN_DEG = 15            # fit a hinge to the measured path only over at least this much arc
SWING_ARRIVE_TOL = 0.03           # compliant: the hand may end this far off the commanded end (the door decides)
HINGE_RADIUS_RANGE = (0.25, 0.45)

# --- release ---
BACK_OFF_M = 0.20
PRE_PARK_X = 0.216


def _swing_waypoints(start_pose, hinge, deg):
    """Door-arc waypoints [x, y, z, qx, qy, qz, qw] from `start_pose` about `hinge`."""
    radius = float(np.linalg.norm(np.asarray(start_pose.position)[:2] - np.asarray(hinge)[:2]))
    wps = door_arc_waypoints(
        start_pose=start_pose, hinge_position=tuple(hinge), arc_length_m=radius * np.radians(deg),
        waypoint_spacing_m=SWING_WAYPOINT_SPACING_M, direction=SWING_DIRECTION, rotate_orientation=True)
    return [list(w.position) + list(w.orientation) for w in wps]


def _check_arc(scene, rb, wps, q0):
    """Chained sim IK over every swing waypoint from joints `q0`. Returns (ok, q_end, message)."""
    q = np.asarray(q0, dtype=float)
    for i, w in enumerate(wps):
        pos, quat = w[:3], w[3:]
        for j, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=rb.physics_client_id)
        wpose = multiply_poses(scene.robot_base_pose, Pose(tuple(pos), tuple(quat)))
        sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id, list(wpose.position),
                                           list(wpose.orientation), physicsClientId=rb.physics_client_id,
                                           maxNumIterations=200)
        nq = np.array(sol[:7])
        for j, jj in enumerate(ARM):
            p.resetJointState(rb.robot_id, jj, float(nq[j]), physicsClientId=rb.physics_client_id)
        ikerr = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wpose.position))
        jump = float(np.max(np.degrees(np.abs(nq - q))))
        j6 = float(np.degrees(nq[5]))
        if (ikerr > SWING_IK_PROBE_M or np.linalg.norm(pos) > SWING_MAX_REACH or jump > SWING_MAX_JUMP_DEG
                or abs(j6) > J6_GUARD_DEG or abs(np.degrees(nq[3])) > J4_GUARD_DEG):
            return False, q, (f"waypoint {i + 1}: ik_err {ikerr * 100:.2f}cm, reach {np.linalg.norm(pos):.3f}, "
                              f"jump {jump:.1f}deg, J6 {j6:.1f}deg")
        q = nq
    return True, q, f"all {len(wps)} waypoints OK, final J6 {np.degrees(q[5]):.1f}deg"


DETECT_FRAMES_DIR = Path.home() / "microwave_detections"


def _save_frame(d, tag, attempt, det, detector):
    """Keep every detection's colour + depth frame (and the result) for looking at misses afterwards: real
    depth (uint16 mm), K and the camera pose (microwave_common.save_frame). Until 10-21 the depth was written
    as a float32 PNG, which saturated to 0/255 -- those older frames have no usable depth."""
    try:
        t = detector.get_frame_to_frame_transform(d["camera_info"])
        frame = {"rgb": d["rgb_image"], "depth_mm": d["depth_image"],          # bgr8, float32 mm
                 "K": np.asarray(d["camera_info"].K, dtype=float).reshape(3, 3),
                 "base_T_cam": None if t is None else detector.make_homogeneous_transform(t)}
        save_frame(frame, f"look{tag}_try{attempt}", DETECT_FRAMES_DIR, result=det)
    except Exception as e:  # never let debugging output stop a run
        print(f"  (could not save the detection frame: {e})")


def detect_handle():
    """Two agreeing looks. Returns (handle position, handle orientation, door normal, door_z, door_mid, hinge)."""
    rs = RealSenseROS2Interface(color_topic="/wrist_camera/color/image_raw",
                                camera_info_topic="/wrist_camera/color/camera_info",
                                depth_topic="/wrist_camera/aligned_depth_to_color/image_raw")
    if not rs.wait_for_frames(30.0):
        sys.exit("No RGB-D frames")
    detector = HandleDetector()

    def look(tag):
        for attempt in range(1, MAX_EMPTY_RETRIES + 1):
            d = rs.get_camera_data()
            det = detector.detect(d["rgb_image"], d["camera_info"], d["depth_image"])
            _save_frame(d, tag, attempt, det, detector)
            if det is not None:
                break
            print(f"  no detection on look {tag} (retry {attempt}/{MAX_EMPTY_RETRIES})")
            time.sleep(0.5)
        else:
            sys.exit(f"NO DETECTION ({tag}) after {MAX_EMPTY_RETRIES} retries")
        print(f"  look {tag}: handle {np.round(det['handle'], 4)}  hinge {np.round(det['hinge'], 4)}  "
              f"door normal {np.round(det['normal'], 3)}  door z {np.round(det['door_z'], 3)}")
        return det["handle"], det["quat"], det["normal"], det["door_z"], det["door_mid"], det["hinge"]

    looks, pair = [], None
    for i in range(1, MAX_LOOKS + 1):
        looks.append(look(str(i)))
        pair = next((looks[j] for j in range(len(looks) - 1)
                     if np.linalg.norm(looks[j][0] - looks[-1][0]) <= DETECT_AGREE), None)
        if pair is not None:
            break
    if pair is None:
        sys.exit(f"No two of {len(looks)} detections agreed within {DETECT_AGREE * 100:.0f} cm -- refusing.")
    (ha, _, na, zra, mida, hga), (hb, orient, nb, zrb, midb, hgb) = pair, looks[-1]
    print(f"detections agree to {np.linalg.norm(ha - hb) * 100:.1f} cm after {len(looks)} look(s); "
          f"hinges to {np.linalg.norm(hga[:2] - hgb[:2]) * 100:.1f} cm")
    if np.linalg.norm(hga[:2] - hgb[:2]) > HINGE_AGREE:
        sys.exit(f"the two hinge detections differ by more than {HINGE_AGREE * 100:.0f} cm -- refusing.")
    h = (ha + hb) / 2.0
    hinge = (hga + hgb) / 2.0
    n = (na + nb) / np.linalg.norm(na + nb)
    door_z = np.mean([zra, zrb], axis=0)
    door_mid = np.mean([mida, midb], axis=0)
    if not (door_z[0] < h[2] < door_z[1] and 0.15 <= door_z[1] - door_z[0] <= 0.60):
        print(f"door z range {np.round(door_z, 3)} doesn't fit a door around the handle -- not saving it")
        door_z = None
    for nm, v, (lo, hi) in (("x", h[0], PLAUSIBLE_X), ("y", h[1], PLAUSIBLE_Y), ("z", h[2], PLAUSIBLE_Z)):
        if not lo <= v <= hi:
            sys.exit(f"handle {nm}={v:.3f} outside plausible {(lo, hi)} -- refusing.")
    print(f"handle (mean) {np.round(h, 4)}  hinge (mean) {np.round(hinge, 4)}")
    return h, orient, n, door_z, door_mid, hinge


def grasp_pose(h, orient, n, pitch_deg=0.0):
    """Grasp pose for handle position `h` (vertical-corrected here), squared to the door face, tilted
    `pitch_deg` nose-down about the finger axis. The tool point doesn't depend on the tilt."""
    h = h + np.array([0.0, 0.0, VERTICAL_CORR])
    quat = R.from_quat(orient) * R.from_euler("y", np.pi)
    # The detector's orientation is a fixed constant (right only for a door facing the arm
    # head-on); yaw it about world z so the approach axis points into the face (-normal).
    # Yaw only, so the roll -- fingers closing across the vertical bar -- is kept.
    z0, into = quat.as_matrix()[:, 2], -n
    yaw = float(np.arctan2(z0[0] * into[1] - z0[1] * into[0], z0[0] * into[0] + z0[1] * into[1]))
    print(f"door normal {np.round(n, 3)} -> yaw {np.degrees(yaw):+.1f} deg to face the door square")
    if abs(np.degrees(yaw)) > MAX_FACE_YAW_DEG:
        sys.exit(f"Face yaw {np.degrees(yaw):.1f} deg > {MAX_FACE_YAW_DEG} deg -- implausible plane fit? Refusing.")
    quat = (R.from_euler("z", yaw) * quat * R.from_euler("x", pitch_deg, degrees=True)).as_quat()
    approach = R.from_quat(quat).as_matrix()[:, 2]
    # the tool point is placed along the LEVEL approach -- where the touch check put it -- whatever the tilt
    approach[2] = 0.0
    approach /= np.linalg.norm(approach)
    pos = h + approach * (GRASP_X_ADJUST - GRIP_EXT)
    print(f"grasp {np.round(pos, 4)}  range {np.linalg.norm(pos):.3f}")
    if np.linalg.norm(pos) > MAX_REACH or not MIN_Z <= pos[2] <= MAX_Z:
        sys.exit(f"GATE FAILED at grasp: range {np.linalg.norm(pos):.3f} (max {MAX_REACH}), "
                 f"z {pos[2]:.3f} (range {MIN_Z}-{MAX_Z})")
    return h, Pose(tuple(pos), tuple(quat))


def open_task(ai, execute, max_swing=SWING_MAX_TRY_DEG):
    """Detect, plan, grasp, swing. Returns True when the swing finished (or the dry run passed); exits on any refusal."""
    scene, rb = make_sim()

    st = ai.get_state()
    ee0, q0 = np.asarray(st["ee_pos"], dtype=float), np.asarray(st["position"], dtype=float)
    if float(st["gripper_pos"]) > 0.2:
        print("gripper is closed -- " + ("opening it first" if execute else "the real run opens it first"))
        if execute:
            ai.execute_command(OpenGripperCommand())
            time.sleep(1.5)
            if float(ai.get_state()["gripper_pos"]) > 0.2:
                sys.exit("Gripper still closed after opening -- refusing to grasp.")

    h_raw, orient, n, door_z, door_mid, hinge_seen = detect_handle()
    # the real pivot sits behind the face and beyond the visible edge (HINGE_*), expressed in the door's own frame
    away = np.r_[hinge_seen[:2] - h_raw[:2], 0.0]
    away /= np.linalg.norm(away)
    hinge = hinge_seen - np.r_[n[:2], 0.0] / np.linalg.norm(n[:2]) * HINGE_BEHIND_FACE + away * HINGE_PAST_EDGE
    print(f"hinge: detected edge {np.round(hinge_seen, 4)} -> corrected pivot {np.round(hinge, 4)} "
          f"({HINGE_BEHIND_FACE * 100:.1f} cm behind the face, {HINGE_PAST_EDGE * 100:.1f} cm past the edge)")
    h, grasp = grasp_pose(h_raw, orient, n)       # runs the grasp gates once, prints the grasp point
    radius = float(np.linalg.norm(np.asarray(grasp.position)[:2] - hinge[:2]))
    print(f"hinge (detected) {np.round(hinge, 4)}  radius {radius * 100:.1f} cm")
    if not HINGE_RADIUS_RANGE[0] <= radius <= HINGE_RADIUS_RANGE[1]:
        sys.exit(f"hinge radius {radius * 100:.1f} cm is implausible (door ~35 cm) -- refusing.")

    # --- plan everything before any motion: grasp path + largest swing, for each nose-down tilt ---
    best = None                                   # (swing deg, pitch, grasp, leg)
    for pitch in GRASP_PITCHES_DEG:
        with contextlib.redirect_stdout(io.StringIO()):
            _, g = grasp_pose(h_raw, orient, n, pitch)
            leg_p = plan_cartesian(scene, rb, ee0[:3], ee0[3:7], np.asarray(g.position), np.asarray(g.orientation),
                                   q0, "to grasp")
        if leg_p is None:
            print(f"  tilt {pitch:2d} deg: grasp path fails a gate")
            continue
        for d in range(max_swing, min(SWING_MIN_DEG, max_swing) - 1, -5):
            ok, _, msg = _check_arc(scene, rb, _swing_waypoints(g, hinge, d), leg_p[-1])
            if ok:
                break
        else:
            d, ok = None, False
        print(f"  tilt {pitch:2d} deg: grasp J4 {np.degrees(leg_p[-1][3]):6.1f} J6 {np.degrees(leg_p[-1][5]):5.1f} -> "
              + (f"swing {d} deg OK ({msg})" if ok else f"no swing >= {SWING_MIN_DEG} deg ({msg})"))
        if ok and (best is None or d > best[0]):
            best = (d, pitch, g, leg_p)
        if ok and d == max_swing:
            break                                 # smallest tilt that reaches the largest swing
    if best is None:
        sys.exit(f"no tilt in {GRASP_PITCHES_DEG} gives a grasp + swing of >= {SWING_MIN_DEG} deg that passes the "
                 f"sim gates -- refusing before any motion. Start joints {np.round(np.degrees(q0), 1)}.")
    deg, pitch, grasp, leg = best
    g_p, g_q = np.asarray(grasp.position), np.asarray(grasp.orientation)
    print(f"chosen: tilt {pitch} deg nose-down, swing {deg} deg; grasp {np.round(g_p, 4)}")
    plan_cartesian(scene, rb, ee0[:3], ee0[3:7], g_p, g_q, q0, "to grasp")      # re-print the chosen path

    if not execute:
        print(f"\nDRY RUN -- would grasp, then swing {deg} deg. Nothing commanded.")
        return True

    # --- grasp ---
    _, e, _ = run_cartesian_trajectory(ai, [(list(g_p), list(g_q))])
    print(f"grasp: tracking {e * 100:.1f} cm")
    if e > GRASP_TRACK_ABORT:
        sys.exit(f"Tracking {e * 100:.1f} cm at the grasp -- ABORT, gripper untouched.")
    # closed-door geometry for the close task (door_z/door_mid written even when None, so stale ones can't linger)
    save_door_geometry(closed_normal=n, closed_grasp_pos=g_p, closed_handle=h, door_z=door_z, door_mid=door_mid)
    if hasattr(ai, "gripper"):
        ai.gripper(1.0, force=GRASP_FORCE)         # looser hold (sheppy)
    else:
        ai.execute_command(CloseGripperCommand())
    t0, last = time.time(), -1.0
    time.sleep(0.6)
    while time.time() - t0 < 3.5:   # until the fingers stop closing
        g = float(ai.get_state()["gripper_pos"])
        if abs(g - last) < 0.003:
            break
        last = g
        time.sleep(0.2)
    print(f"gripper after close: {g:.4f} (can't confirm a grasp on this rig -- watch it)")
    if g < 0.2:
        sys.exit("Gripper is OPEN -- nothing grasped. Refusing to swing.")

    # --- swing, re-checked from where the arm actually is ---
    st = ai.get_state()
    ee = list(st["ee_pos"])
    wps = _swing_waypoints(Pose(tuple(ee[:3]), tuple(ee[3:7])), hinge, deg)
    ok, _, msg = _check_arc(scene, rb, wps, st["position"])
    if not ok:
        sys.exit(f"swing re-check from the real grasp failed ({msg}) -- stopping, still holding the handle.")
    save_door_geometry(hinge=hinge_seen, hinge_pivot=hinge)   # demos + door model use the detected edge; swing uses the pivot
    sent = wps[SWING_SEND_EVERY - 1::SWING_SEND_EVERY]
    if sent[-1] is not wps[-1]:
        sent.append(wps[-1])
    print(f"swing {deg} deg: {len(wps)} waypoints checked, sending {len(sent)} ...")
    imp = None
    if SWING_COMPLIANT and hasattr(ai, "move_cartesian"):
        from sheppy_arm import IMPEDANCE as imp
    radius = float(np.linalg.norm(np.asarray(ee[:2]) - hinge[:2]))
    n_stiff = min(len(wps), int(np.ceil(radius * np.radians(SWING_STIFF_DEG) / SWING_WAYPOINT_SPACING_M)))
    if imp is None or n_stiff >= len(wps):
        ok, err, _ = run_cartesian_trajectory(ai, [(w[:3], w[3:]) for w in sent])
        path = np.array([p_[1:] for p_ in getattr(ai, "last_path", [])])
    else:
        stiff, soft = wps[:n_stiff], wps[n_stiff:]
        soft_sent = soft[SWING_SEND_EVERY - 1::SWING_SEND_EVERY]
        if not soft_sent or soft_sent[-1] is not soft[-1]:
            soft_sent.append(soft[-1])
        print(f"  first {SWING_STIFF_DEG} deg stiff ({len(stiff)} waypoints, breaks the latch), then compliant: joint "
              f"impedance kq {imp['kq']}, zeta {imp['zeta']}, stop at {imp['path_tol']} rad lag")
        ok, err, _ = run_cartesian_trajectory(ai, [(w[:3], w[3:]) for w in stiff], tol_m=0.01)
        path = np.array([p_[1:] for p_ in getattr(ai, "last_path", [])])
        if not ok:
            print(f"  stiff part stopped {err * 100:.1f} cm short -- not starting the compliant part.")
        else:
            ok, err, _ = run_cartesian_trajectory(ai, [(w[:3], w[3:]) for w in soft_sent], impedance=imp,
                                                  tol_m=SWING_ARRIVE_TOL)
            soft_path = np.array([p_[1:] for p_ in getattr(ai, "last_path", [])])
            path = np.vstack([path, soft_path]) if len(soft_path) else path
    final = ai.get_state()
    arc_deg = 0.0
    if len(path) > 1:
        a0, a1 = path[0, :2] - hinge[:2], path[-1, :2] - hinge[:2]
        arc_deg = abs(np.degrees(np.arctan2(a0[0] * a1[1] - a0[1] * a1[0], a0 @ a1)))
    if len(path) > 20 and arc_deg >= SWING_FIT_MIN_DEG:
        c, r, rms = fit_circle_xy(path[:, :2])
        cmd = np.array([w[:3] for w in wps])
        dev = np.max(np.linalg.norm(path[:, None, :2] - cmd[None, :, :2], axis=2).min(axis=1))
        print(f"  measured hand path: {len(path)} samples, max {dev * 100:.1f} cm off the commanded arc; circle fit "
              f"hinge {np.round(c, 4)} r {r * 100:.1f} cm (rms {rms * 1000:.1f} mm) vs detected hinge "
              f"{np.round(hinge[:2], 4)} r {np.linalg.norm(np.asarray(ee[:2]) - hinge[:2]) * 100:.1f} cm -- "
              f"off by {np.linalg.norm(c - hinge[:2]) * 100:.1f} cm")
        if imp is not None and rms < 0.005:
            save_door_geometry(hinge_fit=[float(c[0]), float(c[1]), float(hinge[2])], hinge_fit_r=r)
    print(f"SWING {'DONE' if ok else 'STOPPED SHORT'}: EE {np.round(final['ee_pos'][:3], 4)} "
          f"({err * 100:.1f} cm from the last waypoint)")
    record_door_angle(final["ee_pos"])
    return bool(ok)


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run")
    a.add_argument("--swing-deg", type=int, default=SWING_MAX_TRY_DEG,
                   help=f"largest swing to try (deg, default {SWING_MAX_TRY_DEG}); e.g. 50 leaves the door where the "
                        "over-the-door retreat + push-open can reach it (10-07)")
    a.add_argument("--release", action="store_true",
                   help="after the swing: open the gripper, back off the handle, park")
    args = a.parse_args()
    ai = SheppyArm()
    if args.release:
        release_and_back_off(ai, BACK_OFF_M, args.execute, pre_park_x=PRE_PARK_X)
    else:
        open_task(ai, args.execute, args.swing_deg)


if __name__ == "__main__":
    main()
