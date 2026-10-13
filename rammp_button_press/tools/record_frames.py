"""Record wrist-camera frames (colour + REAL aligned depth + intrinsics) for tuning the detector.
READ-ONLY: never touches the arm. Needs the camera up (sheppy arm-camera or camera-only).

    python3 record_frames.py cream_front            # 10 frames into frames/rec/<label>_<time>/
    python3 record_frames.py cream_side -n 20

Each frame: NNN_color.png, NNN_depth.npy (metres, float32), plus K.npy (fx fy cx cy).
Record a few views per spot (front ~30 cm, far ~50 cm, from a side, higher, lower), holding
the camera still for each. eval_frames.py-style tuning can then use real depth.
"""
import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rammp_button_press.perception import to_bgr_depth


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("label")
    ap.add_argument("-n", type=int, default=10)
    ap.add_argument("--camera-ns", default="/wrist_camera")
    a = ap.parse_args()
    out = Path(__file__).resolve().parent / "frames" / "rec" / f"{a.label}_{time.strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True)
    rclpy.init()
    node = rclpy.create_node("record_frames")
    br, st = CvBridge(), {}
    ns = a.camera_ns.rstrip("/")
    node.create_subscription(CameraInfo, f"{ns}/color/camera_info", lambda m: st.__setitem__("k", m.k), qos_profile_sensor_data)
    node.create_subscription(Image, f"{ns}/color/image_raw", lambda m: st.__setitem__("c", m), qos_profile_sensor_data)
    node.create_subscription(Image, f"{ns}/aligned_depth_to_color/image_raw", lambda m: st.__setitem__("d", m),
                             qos_profile_sensor_data)
    last, k, t0 = None, 0, time.time()
    while k < a.n and time.time() - t0 < 10 + a.n:
        rclpy.spin_once(node, timeout_sec=0.05)
        if {"k", "c", "d"} <= st.keys() and st["c"] is not last:
            last = st["c"]
            bgr, depth = to_bgr_depth(br, st["c"], st["d"])
            if k == 0:
                K = st["k"]
                np.save(out / "K.npy", np.array([K[0], K[4], K[2], K[5]]))
            cv2.imwrite(str(out / f"{k:03d}_color.png"), bgr)
            np.save(out / f"{k:03d}_depth.npy", depth.astype(np.float32))
            k += 1
    node.destroy_node()
    rclpy.shutdown()
    print(f"{k} frames -> {out}" if k else "no frames -- is the camera up (sheppy arm-camera or camera-only)?")


if __name__ == "__main__":
    main()
