"""Open the microwave on `rchi-cpu-5`: detect the handle, grasp it, swing the door open.

    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py             # dry run
    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --execute
    python3 -u microwave/real_gen3_ros2_grasp_and_swing_microwave.py --release --execute

1. Detect (`handle_detect.py`): YOLO box around the microwave, depth -> door plane, the
   vertical cluster sticking out of it = handle. Looks until two agree within 3 cm.
2. Plan everything before any motion: the grasp (squared to the door face, one straight
   Cartesian line from the view pose), the hinge (last run's hinge carried over in the door's
   frame, from `~/.microwave_door.json`), and the largest swing (75 down to 45 deg) whose whole
   arc passes the sim gates (IK, reach, joint jump, J4/J6 guards).
3. Execute: one blended motion into the grasp, close, one blended swing. The hand stays on the
   handle at the end.

`--release` (separate run, after the swing): open the gripper, back straight off the handle,
go toward the base to clear the door's free edge, joint move to `park_pose.json`.

Run from the repo root (the sim config path is relative). See NOTES.md for the bring-up and env.
"""
import argparse, json, sys, time

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import CloseGripperCommand, OpenGripperCommand
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

from door_push import record_door_angle
from handle_detect import HandleDetector
from microwave_common import (DOOR_FILE, J4_GUARD_DEG, plan_cartesian, release_and_back_off, run_cartesian_trajectory,
                              save_door_geometry)

ARM = [1, 2, 3, 4, 5, 6, 7]

# --- detection ---
MAX_LOOKS = 5              # keep looking until two detections agree within DETECT_AGREE
MAX_EMPTY_RETRIES = 8      # per look: retries on an empty read (viewpoint-dependent YOLO misses)
DETECT_AGREE = 0.03
PLAUSIBLE_X = (0.30, 0.85)
PLAUSIBLE_Y = (-0.50, 0.20)
PLAUSIBLE_Z = (0.15, 0.65)
MAX_FACE_YAW_DEG = 70.0    # beyond this the plane fit is more likely wrong than the door

# --- grasp (tuned on the arm, see NOTES.md) ---
GRIP_EXT = -0.010          # along the approach axis (negative = past the detected handle)
VERTICAL_CORR = 0.025      # world z
GRASP_X_ADJUST = 0.040     # along the approach axis
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
HINGE_RADIUS_RANGE = (0.25, 0.45)

# --- release ---
BACK_OFF_M = 0.20
PRE_PARK_X = 0.216


def _make_sim():
    scene = create_scene_description_from_config("src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
    return scene, FeedingDeploymentPyBulletSimulator(scene, use_gui=False).robot


def _swing_waypoints(start_pose, hinge, deg):
    """Door-arc waypoints [x, y, z, qx, qy, qz, qw] from `start_pose` about `hinge`."""
    radius = float(np.linalg.norm(np.asarray(start_pose.position) - hinge))
    wps = PerceptionInterface._generate_door_arc_waypoints(
        None, start_pose=start_pose, hinge_position=tuple(hinge), arc_length_m=radius * np.radians(deg),
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


def _transfer_hinge(prev, h, n):
    """Carry the last run's hinge over to this detection in the DOOR's frame (depth along the
    normal + distance along the face from the handle), so a moved or turned microwave is handled."""
    n0 = np.asarray(prev["closed_normal"], float)[:2]
    n0 /= np.linalg.norm(n0)
    t0 = np.array([-n0[1], n0[0]])
    d = np.asarray(prev["hinge"], float) - np.asarray(prev["closed_handle"], float)
    a, b = float(d[:2] @ n0), float(d[:2] @ t0)
    n1 = np.asarray(n, float)[:2] / np.linalg.norm(np.asarray(n, float)[:2])
    t1 = np.array([-n1[1], n1[0]])
    xy = np.asarray(h, float)[:2] + a * n1 + b * t1
    return np.array([xy[0], xy[1], float(h[2]) + float(d[2])])


def detect_handle():
    """Two agreeing looks. Returns (handle position, handle orientation, door normal, door_z, door_mid)."""
    rs = RealSenseROS2Interface()
    if not rs.wait_for_frames(30.0):
        sys.exit("No RGB-D frames")
    detector = HandleDetector()

    def look(tag):
        for attempt in range(1, MAX_EMPTY_RETRIES + 1):
            d = rs.get_camera_data()
            det = detector.detect(d["rgb_image"], d["camera_info"], d["depth_image"])
            if det is not None:
                break
            print(f"  no detection on look {tag} (retry {attempt}/{MAX_EMPTY_RETRIES})")
            time.sleep(0.5)
        else:
            sys.exit(f"NO DETECTION ({tag}) after {MAX_EMPTY_RETRIES} retries")
        print(f"  look {tag}: handle {np.round(det['handle'], 4)}  door normal {np.round(det['normal'], 3)}  "
              f"door z {np.round(det['door_z'], 3)}")
        return det["handle"], det["quat"], det["normal"], det["door_z"], det["door_mid"]

    looks, pair = [], None
    for i in range(1, MAX_LOOKS + 1):
        looks.append(look(str(i)))
        pair = next((looks[j] for j in range(len(looks) - 1)
                     if np.linalg.norm(looks[j][0] - looks[-1][0]) <= DETECT_AGREE), None)
        if pair is not None:
            break
    if pair is None:
        sys.exit(f"No two of {len(looks)} detections agreed within {DETECT_AGREE * 100:.0f} cm -- refusing.")
    (ha, _, na, zra, mida), (hb, orient, nb, zrb, midb) = pair, looks[-1]
    print(f"detections agree to {np.linalg.norm(ha - hb) * 100:.1f} cm after {len(looks)} look(s)")
    h = (ha + hb) / 2.0
    n = (na + nb) / np.linalg.norm(na + nb)
    door_z = np.mean([zra, zrb], axis=0)
    door_mid = np.mean([mida, midb], axis=0)
    if not (door_z[0] < h[2] < door_z[1] and 0.15 <= door_z[1] - door_z[0] <= 0.60):
        print(f"door z range {np.round(door_z, 3)} doesn't fit a door around the handle -- not saving it")
        door_z = None
    for nm, v, (lo, hi) in (("x", h[0], PLAUSIBLE_X), ("y", h[1], PLAUSIBLE_Y), ("z", h[2], PLAUSIBLE_Z)):
        if not lo <= v <= hi:
            sys.exit(f"handle {nm}={v:.3f} outside plausible {(lo, hi)} -- refusing.")
    print(f"handle (mean) {np.round(h, 4)}")
    return h, orient, n, door_z, door_mid


def grasp_pose(h, orient, n):
    """Grasp pose for handle position `h` (vertical-corrected here), squared to the door face."""
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
    quat = (R.from_euler("z", yaw) * quat).as_quat()
    approach = R.from_quat(quat).as_matrix()[:, 2]
    pos = h + approach * (GRASP_X_ADJUST - GRIP_EXT)
    print(f"grasp {np.round(pos, 4)}  range {np.linalg.norm(pos):.3f}")
    if np.linalg.norm(pos) > MAX_REACH or not MIN_Z <= pos[2] <= MAX_Z:
        sys.exit(f"GATE FAILED at grasp: range {np.linalg.norm(pos):.3f} (max {MAX_REACH}), "
                 f"z {pos[2]:.3f} (range {MIN_Z}-{MAX_Z})")
    return h, Pose(tuple(pos), tuple(quat))


def open_task(ai, execute):
    prev = json.loads(DOOR_FILE.read_text()) if DOOR_FILE.exists() else {}
    if not all(k in prev for k in ("hinge", "closed_handle", "closed_normal")):
        sys.exit(f"{DOOR_FILE} needs hinge/closed_handle/closed_normal from an earlier run (see NOTES.md).")
    scene, rb = _make_sim()

    st = ai.get_state()
    ee0, q0 = np.asarray(st["ee_pos"], dtype=float), np.asarray(st["position"], dtype=float)
    if float(st["gripper_pos"]) > 0.2:
        print("gripper is closed -- " + ("opening it first" if execute else "the real run opens it first"))
        if execute:
            ai.execute_command(OpenGripperCommand())
            time.sleep(1.5)
            if float(ai.get_state()["gripper_pos"]) > 0.2:
                sys.exit("Gripper still closed after opening -- refusing to grasp.")

    h, orient, n, door_z, door_mid = detect_handle()
    h, grasp = grasp_pose(h, orient, n)

    # --- plan everything before any motion ---
    g_p, g_q = np.asarray(grasp.position), np.asarray(grasp.orientation)
    leg = plan_cartesian(scene, rb, ee0[:3], ee0[3:7], g_p, g_q, q0, "to grasp")
    if leg is None:
        sys.exit(f"grasp path fails a gate -- refusing before any motion. Start joints {np.round(np.degrees(q0), 1)}: "
                 f"start from a view pose with |J4| well under {J4_GUARD_DEG:.0f}.")
    hinge = _transfer_hinge(prev, h, n)
    radius = float(np.linalg.norm(g_p - hinge))
    print(f"hinge (last hinge carried over in the door frame) {np.round(hinge, 4)}  radius {radius * 100:.1f} cm")
    if not HINGE_RADIUS_RANGE[0] <= radius <= HINGE_RADIUS_RANGE[1]:
        sys.exit(f"hinge radius {radius * 100:.1f} cm is implausible (door ~35 cm) -- refusing.")
    for deg in range(SWING_MAX_TRY_DEG, SWING_MIN_DEG - 1, -5):
        ok, _, msg = _check_arc(scene, rb, _swing_waypoints(grasp, hinge, deg), leg[-1])
        print(f"  swing {deg} deg: {'OK' if ok else 'fails -- ' + msg}")
        if ok:
            break
    else:
        sys.exit(f"no swing of {SWING_MIN_DEG}-{SWING_MAX_TRY_DEG} deg passes the sim gates -- refusing.")

    if not execute:
        print(f"\nDRY RUN -- would grasp, then swing {deg} deg. Nothing commanded.")
        return

    # --- grasp ---
    _, e, _ = run_cartesian_trajectory(ai, [(list(g_p), list(g_q))])
    print(f"grasp: tracking {e * 100:.1f} cm")
    if e > GRASP_TRACK_ABORT:
        sys.exit(f"Tracking {e * 100:.1f} cm at the grasp -- ABORT, gripper untouched.")
    # closed-door geometry for the close task (door_z/door_mid written even when None, so stale ones can't linger)
    save_door_geometry(closed_normal=n, closed_grasp_pos=g_p, closed_handle=h, door_z=door_z, door_mid=door_mid)
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
    save_door_geometry(hinge=hinge)
    sent = wps[SWING_SEND_EVERY - 1::SWING_SEND_EVERY]
    if sent[-1] is not wps[-1]:
        sent.append(wps[-1])
    print(f"swing {deg} deg: {len(wps)} waypoints checked, sending {len(sent)} ...")
    ok, err, _ = run_cartesian_trajectory(ai, [(w[:3], w[3:]) for w in sent])
    final = ai.get_state()
    print(f"SWING {'DONE' if ok else 'STOPPED SHORT'}: EE {np.round(final['ee_pos'][:3], 4)} "
          f"({err * 100:.1f} cm from the last waypoint)")
    record_door_angle(final["ee_pos"])


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--execute", action="store_true", help="command the arm; omit for a dry run")
    a.add_argument("--release", action="store_true",
                   help="after the swing: open the gripper, back off the handle, park")
    args = a.parse_args()
    ai = ArmInterfaceClient()
    if args.release:
        release_and_back_off(ai, BACK_OFF_M, args.execute, pre_park_x=PRE_PARK_X)
    else:
        open_task(ai, args.execute)


if __name__ == "__main__":
    main()
