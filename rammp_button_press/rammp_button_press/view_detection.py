"""Live dome-layout detection viewer. READ-ONLY -- never touches the arm. Esc / q / Ctrl-C to close.

    ros2 run rammp_button_press view_detection [--target timer_clock] [--camera-ns /wrist_camera]

Magenta = the 5 domes the layout fit found (target ringed in green), with the measured dome
pitch (should read ~19 mm), mean fit error and range. When dome_pattern.detect() abstains the
reason is shown: "TOO CLOSE" / "TOO FAR" (layout found, outside MIN_RANGE_M..MAX_RANGE_M: move
the camera) or "no 5-dome layout".
"""
import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
import rclpy.executors
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image

from rammp_button_press import dome_pattern as dp
from rammp_button_press.perception import to_bgr_depth, why_no_fit


def draw_domes(bgr, fit, target, dt_ms, why="NO FIT"):
    vis = bgr.copy()
    if fit is None:
        cv2.putText(vis, f"domes: {why}", (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
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
    ap.add_argument("--camera-ns", default="/wrist_camera")
    a = ap.parse_args()
    ns = a.camera_ns.rstrip("/")

    rclpy.init()
    node = rclpy.create_node("detection_viewer")
    bridge = CvBridge()
    got = {}
    qos = qos_profile_sensor_data
    node.create_subscription(Image, f"{ns}/color/image_raw", lambda m: got.__setitem__("c", m), qos)
    node.create_subscription(Image, f"{ns}/aligned_depth_to_color/image_raw", lambda m: got.__setitem__("d", m), qos)
    node.create_subscription(CameraInfo, f"{ns}/color/camera_info", lambda m: got.__setitem__("i", m), qos)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    win = "button detection (dome layout)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 640, 480)
    last = prev = None
    try:
        while True:
            if all(k in got for k in ("c", "d", "i")) and got["c"] is not last:
                last = got["c"]
                bgr, depth = to_bgr_depth(bridge, last, got["d"])
                t = time.perf_counter()
                fit = dp.detect(bgr, depth, got["i"].k[0], hint=prev)
                if fit is None and prev is not None:
                    fit = dp.detect(bgr, depth, got["i"].k[0])
                prev = fit
                why = why_no_fit(bgr, depth, got["i"].k[0]) if fit is None else ""
                cv2.imshow(win, draw_domes(bgr, fit, a.target, (time.perf_counter() - t) * 1000, why))
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
