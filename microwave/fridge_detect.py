"""Fridge handle in arm_base_link: SAM 3 mask ("which pixels", colour-agnostic) + aligned depth ("how far").

The --fridge stand-in for `detect_handle()` in real_gen3_ros2_grasp_and_swing_microwave.py, returning the same
tuple. The model is the fridge branch's `Sam3HandleDetector` (scripts/detect_handle_sam3.py); the rest follows that
file's `detect_once`/`detect_until_agree`, with every constant taken from fridge_constants.py. Only the handle is
detected: the door normal is assumed and the hinge comes from a measured offset, so door_z, door_mid and the hinge
are returned as None.
"""
import sys
import time
from pathlib import Path

import numpy as np

import fridge_constants as F
from feeding_deployment.perception.tf_interface import TFInterface
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
from microwave_common import save_frame

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from detect_handle_sam3 import GRASP_QUAT, Sam3HandleDetector  # noqa: E402


def mask_to_camera_xyz(mask, depth_mm, camera_info):
    """Median camera-frame point (m) over the mask's valid-depth pixels, or None if too little has depth."""
    vs, us = np.where(mask)
    d = depth_mm[vs, us] / 1000.0
    ok = np.isfinite(d) & (d > F.DEPTH_RANGE_M[0]) & (d < F.DEPTH_RANGE_M[1])
    if ok.sum() == 0 or ok.sum() < F.MIN_VALID_DEPTH_FRAC * len(d):
        print(f"  only {int(ok.sum())}/{len(d)} handle pixels have valid depth")
        return None
    fx, fy, cx, cy = camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]
    z = d[ok]
    return np.median(np.stack([(us[ok] - cx) * z / fx, (vs[ok] - cy) * z / fy, z], axis=1), axis=0)


def detect_fridge_handle():
    """Two agreeing looks. Returns (handle, orientation, door normal, None, None, None) like detect_handle()."""
    rs = RealSenseROS2Interface(color_topic="/wrist_camera/color/image_raw",
                                camera_info_topic="/wrist_camera/color/camera_info",
                                depth_topic="/wrist_camera/aligned_depth_to_color/image_raw")
    if not rs.wait_for_frames(30.0):
        sys.exit("No RGB-D frames")
    tf = TFInterface()
    det = Sam3HandleDetector(prompt=F.SAM3_PROMPT, repo=F.SAM3_REPO, dtype=F.SAM3_DTYPE, thresh=F.SAM3_THRESHOLD)

    def look_once(d):
        seg = det.segment(np.ascontiguousarray(d["rgb_image"][:, :, ::-1]))   # bgr8 -> rgb
        if seg is None:
            print(f"  SAM 3 found nothing for {F.SAM3_PROMPT!r}")
            return None, None
        mask, score, box = seg
        if mask.sum() < F.MIN_MASK_PX:
            print(f"  mask only {int(mask.sum())} px")
            return None, None
        cam_xyz = mask_to_camera_xyz(mask, d["depth_image"], d["camera_info"])
        if cam_xyz is None:
            return None, None
        cam_xyz = cam_xyz + np.array([0.0, 0.0, F.HANDLE_DEPTH_CORR])   # calibration correction, optical axis
        t = tf.get_frame_to_frame_transform(d["camera_info"])
        if t is None:
            print("  no arm_base_link <- camera transform")
            return None, None
        base_T_cam = tf.make_homogeneous_transform(t)
        handle = (base_T_cam @ np.r_[cam_xyz, 1.0])[:3]
        return handle, {"handle": handle, "score": score, "box": box, "cam_xyz": cam_xyz, "base_T_cam": base_T_cam}

    def look(tag):
        for attempt in range(1, F.MAX_EMPTY_RETRIES + 1):
            d = rs.get_camera_data()
            handle, res = look_once(d)
            save_frame({"rgb": d["rgb_image"], "depth_mm": d["depth_image"],
                        "K": np.asarray(d["camera_info"].K, dtype=float).reshape(3, 3),
                        "base_T_cam": None if res is None else res["base_T_cam"]},
                       f"look{tag}_try{attempt}", F.DETECT_FRAMES_DIR, result=res)
            if handle is not None:
                print(f"  look {tag}: handle {np.round(handle, 4)}  score {res['score']:.2f}  "
                      f"cam z {res['cam_xyz'][2]:.3f} m")
                return handle
            print(f"  no detection on look {tag} (retry {attempt}/{F.MAX_EMPTY_RETRIES})")
            time.sleep(0.5)
        sys.exit(f"NO DETECTION ({tag}) after {F.MAX_EMPTY_RETRIES} retries")

    looks, pair = [], None
    for i in range(1, F.MAX_LOOKS + 1):
        looks.append(look(str(i)))
        pair = next((looks[j] for j in range(len(looks) - 1)
                     if np.linalg.norm(looks[j] - looks[-1]) <= F.DETECT_AGREE), None)
        if pair is not None:
            break
    if pair is None:
        sys.exit(f"No two of {len(looks)} detections agreed within {F.DETECT_AGREE * 100:.0f} cm -- refusing.")
    print(f"detections agree to {np.linalg.norm(pair - looks[-1]) * 100:.1f} cm after {len(looks)} look(s)")
    h = (pair + looks[-1]) / 2.0
    for nm, v, (lo, hi) in (("x", h[0], F.PLAUSIBLE_X), ("y", h[1], F.PLAUSIBLE_Y), ("z", h[2], F.PLAUSIBLE_Z)):
        if not lo <= v <= hi:
            sys.exit(f"handle {nm}={v:.3f} outside plausible {(lo, hi)} -- refusing.")
    print(f"handle (mean) {np.round(h, 4)}")
    return h, np.asarray(GRASP_QUAT), F.DOOR_NORMAL.copy(), None, None, None
