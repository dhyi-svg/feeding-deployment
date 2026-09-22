"""STEP 1 -- does the phone-video reference match what the real camera sees?

Run this BEFORE writing any node or touching the arm. The reference was built
from 4K phone footage; the wrist camera is a lower-resolution RealSense with a
different sensor, different colour response and an upside-down mount. SIFT is
scale- and rotation-invariant so it may transfer, but that is an assumption,
not a measurement, and everything downstream depends on it.

Outcomes:
  MATCH    -> proceed to the node. The reference transfers.
  NO MATCH -> rebuild the reference from a camera frame (make_reference.py).
              This is expected-ish and takes ~15 minutes; it is not a failure
              of the approach.

No arm, no motion, no ROS required in the --image mode. Nothing here can move
anything.

Usage:
    # from a saved frame (safest -- grab one with rs-capture or ros2 topic echo)
    python3 check_reference_match.py --ref <ref_dir> --image frame.png

    # live from a ROS 2 image topic
    python3 check_reference_match.py --ref <ref_dir> --topic /camera/color/image_raw

    # live from librealsense directly, no ROS
    python3 check_reference_match.py --ref <ref_dir> --realsense
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


def load_detector(ref_dir):
    here = Path(__file__).resolve()
    for cand in (here.parents[3] / "src", here.parents[2] / "src"):
        if (cand / "feeding_deployment").exists():
            sys.path.insert(0, str(cand))
            break
    from feeding_deployment.perception.appliance_perception.reference_button_detector import (
        ReferenceButtonDetector,
    )
    return ReferenceButtonDetector(ref_dir)


def grab_ros(topic, timeout=10.0):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    rclpy.init()
    node = Node("button_ref_check")
    bridge = CvBridge()
    box = {}

    def cb(msg):
        box["img"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    node.create_subscription(Image, topic, cb, 10)
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while "img" not in box and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    if "img" not in box:
        sys.exit(f"no image on {topic} within {timeout}s -- is the camera up?")
    return box["img"]


def grab_realsense():
    import pyrealsense2 as rs

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe.start(cfg)
    try:
        for _ in range(15):          # let auto-exposure settle
            frames = pipe.wait_for_frames()
        return np.asanyarray(frames.get_color_frame().get_data())
    finally:
        pipe.stop()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True, help="reference directory (holds reference.json)")
    ap.add_argument("--image")
    ap.add_argument("--topic")
    ap.add_argument("--realsense", action="store_true")
    ap.add_argument("--out", default="ref_match_check.png")
    args = ap.parse_args()

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            sys.exit(f"could not read {args.image}")
    elif args.topic:
        img = grab_ros(args.topic)
    elif args.realsense:
        img = grab_realsense()
    else:
        sys.exit("give one of --image / --topic / --realsense")

    det = load_detector(args.ref)
    print(f"frame: {img.shape[1]}x{img.shape[0]}   reference views: {len(det.views)}")

    # Try the frame as-is and rotated 180: the wrist camera is mounted upside
    # down. The detector does not care (a homography absorbs the rotation), but
    # if ONLY the flipped one matches that is worth knowing explicitly.
    results = {}
    for label, frame in (("as-is", img), ("rotated 180", cv2.rotate(img, cv2.ROTATE_180))):
        r = det.detect(frame)
        results[label] = (r, frame)
        ok = r["center"] is not None
        print(f"  {label:12s}: {'MATCH  ' if ok else 'no match'} "
              f"inliers={r.get('inliers', 0):3d}  {r.get('reason', '')}")

    best = max(results.items(), key=lambda kv: (kv[1][0]["center"] is not None,
                                                kv[1][0].get("inliers", 0)))
    label, (r, frame) = best
    vis = frame.copy()
    if r["center"] is not None:
        q = r.get("quad")
        if q is not None:
            cv2.polylines(vis, [q.reshape(-1, 2).astype(np.int32)], True, (0, 255, 0), 2)
        p = (int(r["center"][0]), int(r["center"][1]))
        cv2.circle(vis, p, 14, (0, 0, 255), 3)
        cv2.drawMarker(vis, p, (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
        print(f"\nVERDICT: MATCH on the '{label}' orientation, {r['inliers']} inliers.")
        print("         Check the saved image -- the marker must be ON the +30SEC button,")
        print("         not merely somewhere on the panel. Inliers alone do not prove that.")
    else:
        print("\nVERDICT: NO MATCH. The phone-video reference does not transfer to this")
        print("         camera. Build one from a camera frame with make_reference.py.")
    cv2.imwrite(args.out, vis)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
