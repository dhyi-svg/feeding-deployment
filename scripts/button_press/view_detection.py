"""Live dome-layout detection viewer. READ-ONLY -- never touches the arm. Esc / q / Ctrl-C to close.

    python3 -u scripts/button_press/view_detection.py [--target timer_clock]

Magenta = the 5 domes the layout fit found (target ringed in green), with the measured dome
pitch (should read ~19 mm), mean fit error and range. "NO FIT" means dome_pattern.detect()
abstained on that frame (no 5-dome layout, wrong metric size, or closer than MIN_RANGE_M).
"""
import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from feeding_deployment.button_press import dome_pattern as dp
from feeding_deployment.button_press.perception import to_bgr_depth


def draw_domes(bgr, fit, target, dt_ms):
    vis = bgr.copy()
    if fit is None:
        cv2.putText(vis, "domes: NO FIT", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return vis
    cv2.polylines(vis, [fit.region.astype(np.int32)], True, (255, 200, 0), 1)
    r = max(4, int(0.3 * fit.s_px))
    for name in dp.NAMES:
        u, v = (int(round(c)) for c in fit.px[name])
        hit = name == target
        cv2.circle(vis, (u, v), r + (4 if hit else 0), (0, 255, 0) if hit else (255, 0, 255), 2)
        cv2.putText(vis, name, (u + r + 2, v - r), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 255, 0) if hit else (255, 255, 0), 1)
    cv2.putText(vis, f"domes: pitch {fit.s_mm:.1f} mm  err {fit.err:.2f}  z {fit.z*100:.0f} cm  {dt_ms:.0f} ms",
                (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    return vis


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", default="timer_clock")
    a = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("detection_viewer")
    bridge = CvBridge()
    got = {}
    node.create_subscription(Image, "/camera/color/image_raw", lambda m: got.__setitem__("c", m), 2)
    node.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", lambda m: got.__setitem__("d", m), 2)
    node.create_subscription(CameraInfo, "/camera/color/camera_info", lambda m: got.__setitem__("i", m), 2)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    win = "button detection (dome layout)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 640, 480)
    last = None
    try:
        while True:
            if all(k in got for k in ("c", "d", "i")) and got["c"] is not last:
                last = got["c"]
                bgr, depth = to_bgr_depth(bridge, last, got["d"])
                t = time.perf_counter()
                fit = dp.detect(bgr, depth, got["i"].k[0])
                cv2.imshow(win, draw_domes(bgr, fit, a.target, (time.perf_counter() - t) * 1000))
            k = cv2.waitKey(15) & 0xFF
            # (no WND_PROP_VISIBLE check: this OpenCV 4.5.4/GTK3 build reports 0 for a shown window)
            if k in (27, ord("q")):
                break
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    executor.shutdown()
    spin.join(timeout=2.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
