"""One-shot: capture a single eye-in-hand calibration sample (board pose + arm pose)
and append it to a running JSON file. Invoked once per physical arm pose, on request
-- not interactive/looping, since it's launched non-interactively via `docker exec`.

Redo of the 2026-07-21 cv2.calibrateHandEye() calibration, fixing two things found
wrong with it (see TESTING_LOG.md, "depth stable, handle localization succeeded,
~20cm error found"):

  1. The old 17-sample set had almost no translation diversity (max pairwise distance
     between gripper positions was only 5cm), which starves cv2.calibrateHandEye()'s
     translation solve. This script prints running pairwise-translation-diversity
     stats after every capture.
  2. Captures the robot-pose half of every sample directly from a fresh
     arm.get_state()["ee_pos"] call at capture time (no separately-derived FK), to
     rule out a reference-frame mismatch in the old (no-longer-extant) script.

Board geometry (12-marker ArUco board, DICT_5X5 family, marker IDs 0-11, marker 0 as
the reference/origin marker, 5.08cm markers) is empirically-derived and reused as-is
from the old calibration's raw-sample dump.

Read-only w.r.t. the arm (get_state() only, no motion commands) -- the user moves the
arm by hand. No bulldog_bypass needed.
"""

import argparse
import itertools
import json
import os
from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)

OLD_SAMPLES_PATH = os.path.expanduser(
    "~/deployment_ws/pachirisu_wrist_camera_calib/handeye_raw_samples.json"
)
DEFAULT_OUTPUT = os.path.expanduser(
    "~/deployment_ws/pachirisu_wrist_camera_calib/handeye_raw_samples_v2.json"
)
ARUCO_DICT = aruco.DICT_5X5_250
MIN_MARKERS = 6


def load_board_geometry(path):
    with open(path) as f:
        d = json.load(f)
    board_local_points = {
        int(k): np.array(v, dtype=np.float64) for k, v in d["board_local_points"].items()
    }
    return board_local_points, d["reference_marker_id"], d["assumed_marker_len_m"]


def detect_board_pose(gray, camera_matrix, dist_coeffs, board_local_points):
    dictionary = aruco.getPredefinedDictionary(ARUCO_DICT)
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(dictionary, params)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return None, 0
    ids = ids.flatten().tolist()

    obj_pts, img_pts = [], []
    for marker_id, marker_corners in zip(ids, corners):
        if marker_id not in board_local_points:
            continue
        obj_pts.append(board_local_points[marker_id])
        img_pts.append(marker_corners.reshape(4, 2))
    n_used = len(obj_pts)
    if n_used < MIN_MARKERS:
        return None, n_used

    obj_pts = np.concatenate(obj_pts, axis=0).astype(np.float64)
    img_pts = np.concatenate(img_pts, axis=0).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts, img_pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        return None, n_used
    R_target2cam, _ = cv2.Rodrigues(rvec)
    return (R_target2cam, tvec.flatten()), n_used


def report_diversity(t_gripper2base_list):
    if len(t_gripper2base_list) < 2:
        return
    dists = [
        np.linalg.norm(np.array(a) - np.array(b))
        for a, b in itertools.combinations(t_gripper2base_list, 2)
    ]
    print(
        f"[diversity] n={len(t_gripper2base_list)} "
        f"max pairwise translation so far: {max(dists)*100:.1f}cm, "
        f"mean: {sum(dists)/len(dists)*100:.1f}cm "
        f"(old calibration topped out at 5.0cm max -- aim well past 20-30cm)"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    rospy.init_node("capture_calib_sample", anonymous=True, disable_signals=True)
    bridge = CvBridge()

    board_local_points, reference_marker_id, assumed_marker_len_m = load_board_geometry(
        OLD_SAMPLES_PATH
    )

    ArmManager.register("ArmInterface")
    mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    mg.connect()
    arm = mg.ArmInterface()

    camera_info_msg = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=10)
    camera_matrix = np.array(camera_info_msg.K, dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.array(camera_info_msg.D, dtype=np.float64)

    rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10)
    rgb = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

    board_pose, n_used = detect_board_pose(gray, camera_matrix, dist_coeffs, board_local_points)
    if board_pose is None:
        print(f"[capture] FAILED: only {n_used} markers matched (need >= {MIN_MARKERS}). Not saved.")
        rospy.signal_shutdown("capture failed")
        return
    R_target2cam, t_target2cam = board_pose

    ee_pos = np.asarray(arm.get_state()["ee_pos"], dtype=float)
    t_gripper2base = ee_pos[:3]
    R_gripper2base = Rotation.from_quat(ee_pos[3:7]).as_matrix()

    out_path = Path(args.output)
    if out_path.exists():
        with open(out_path) as f:
            data = json.load(f)
    else:
        data = {
            "R_gripper2base": [],
            "t_gripper2base": [],
            "R_target2cam": [],
            "t_target2cam": [],
            "n_markers": [],
            "reference_marker_id": reference_marker_id,
            "assumed_marker_len_m": assumed_marker_len_m,
            "board_local_points": {str(k): v.tolist() for k, v in board_local_points.items()},
            "pose_source": "arm.get_state()['ee_pos'] read fresh at each capture (direct, no separate FK)",
        }

    data["R_gripper2base"].append(R_gripper2base.tolist())
    data["t_gripper2base"].append(t_gripper2base.tolist())
    data["R_target2cam"].append(R_target2cam.tolist())
    data["t_target2cam"].append(t_target2cam.tolist())
    data["n_markers"].append(n_used)

    with open(out_path, "w") as f:
        json.dump(data, f, indent=2)

    print(f"[capture] #{len(data['t_gripper2base'])}: {n_used} markers, ee_pos={np.round(ee_pos[:3], 4)}")
    report_diversity(data["t_gripper2base"])

    rospy.signal_shutdown("capture complete")


if __name__ == "__main__":
    main()
