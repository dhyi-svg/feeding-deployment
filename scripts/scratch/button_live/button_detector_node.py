#!/usr/bin/env python3
"""ROS 2 node wrapping the reference-homography START/+30SEC button detector.

Publishes
    ~/button_pixel   geometry_msgs/PointStamped   pixel (x, y, 0) in the image frame
    ~/button_pose    geometry_msgs/PoseStamped    3D point in the camera optical frame
                                                  (only when depth + camera_info are available)
    ~/debug_image    sensor_msgs/Image            annotated frame, for eyeballing

Design rules, which matter more than the plumbing:

  * ABSTENTION PUBLISHES NOTHING. When the detector declines, this node emits no
    pose at all rather than a stale or low-confidence one. The detector's only
    real safety property is that it would rather say nothing than name the wrong
    button, and that property is destroyed if a consumer can latch onto an old
    value and treat it as current. Consumers must therefore treat "no message"
    as "do not act", and should check the message timestamp.

  * NO ARM MOTION LIVES HERE. This node only looks. Anything that moves the
    robot is someone else's node, downstream of a human deciding to trust this.

  * The 3D point is NOT VALIDATED. The detector was measured on phone video,
    which has no depth and no calibration -- 22/29 correct with zero wrong
    buttons, but that is the PIXEL only. Every line below that converts pixel to
    metres is unexercised. Treat the pose output as untested until somebody
    checks it against a touched ground truth on the real rig.

  * No upside-down flip is applied. The wrist camera is mounted inverted, but
    the button's identity comes from a mark on the reference carried through a
    homography, which absorbs the rotation. The bottom-row/rightmost backends
    need the flip; this one does not.
"""
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from feeding_deployment.perception.appliance_perception.reference_button_detector import (
    ReferenceButtonDetector,
)

# Depth samples outside this window are rejected outright (metres). Matches the
# sanity window the repo's existing pixel2World uses.
MIN_DEPTH_M, MAX_DEPTH_M = 0.05, 2.0
# Half-width of the patch median-filtered around the button pixel. A single
# depth pixel on a chrome dome is unreliable; chrome is exactly the surface a
# projected-light depth sensor does worst on.
DEPTH_PATCH = 4


class ButtonDetectorNode(Node):
    def __init__(self):
        super().__init__("button_detector")
        self.declare_parameter("reference_dir", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("rate_limit_hz", 5.0)

        ref = self.get_parameter("reference_dir").value
        if not ref:
            raise SystemExit(
                "reference_dir parameter is required -- it points at the directory "
                "holding reference.json for THIS appliance. There is one reference "
                "per microwave; a reference built on another unit will not work."
            )
        self.det = ReferenceButtonDetector(ref)
        self.get_logger().info(f"loaded reference from {ref} ({len(self.det.views)} view(s))")

        self.bridge = CvBridge()
        self.depth = None
        self.info = None
        self._last_run_ns = 0
        self._min_period_ns = int(1e9 / max(0.1, self.get_parameter("rate_limit_hz").value))

        self.pub_px = self.create_publisher(PointStamped, "~/button_pixel", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "~/button_pose", 10)
        self.pub_dbg = self.create_publisher(Image, "~/debug_image", 1)

        self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                 self._on_depth, 10)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self._on_info, 10)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self._on_image, 10)

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def _depth_at(self, u, v):
        """Median depth in metres over a small patch, or None."""
        if self.depth is None:
            return None
        d = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
        h, w = d.shape[:2]
        if not (0 <= u < w and 0 <= v < h):
            return None
        patch = d[max(0, v - DEPTH_PATCH):v + DEPTH_PATCH + 1,
                  max(0, u - DEPTH_PATCH):u + DEPTH_PATCH + 1].astype(np.float32)
        # RealSense 16UC1 is millimetres; 32FC1 is metres.
        if self.depth.encoding in ("16UC1", "mono16"):
            patch = patch / 1000.0
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch > MIN_DEPTH_M) & (patch < MAX_DEPTH_M)]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def _on_image(self, msg):
        now = self.get_clock().now().nanoseconds
        if now - self._last_run_ns < self._min_period_ns:
            return
        self._last_run_ns = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        res = self.det.detect(frame)
        center = res.get("center")

        if center is None:
            # Deliberately publishes nothing. See the module docstring.
            self.get_logger().debug(
                f"abstained: {res.get('reason', 'no fit')} (inliers={res.get('inliers', 0)})")
            if self.get_parameter("publish_debug_image").value:
                self._publish_debug(frame, msg.header, res)
            return

        u, v = int(round(center[0])), int(round(center[1]))
        px = PointStamped()
        px.header = msg.header
        px.point.x, px.point.y, px.point.z = float(u), float(v), 0.0
        self.pub_px.publish(px)

        depth_m = self._depth_at(u, v)
        if depth_m is not None and self.info is not None:
            fx, fy = self.info.k[0], self.info.k[4]
            cx, cy = self.info.k[2], self.info.k[5]
            if fx and fy:
                pose = PoseStamped()
                pose.header = msg.header
                pose.pose.position.x = (u - cx) * depth_m / fx
                pose.pose.position.y = (v - cy) * depth_m / fy
                pose.pose.position.z = depth_m
                pose.pose.orientation.w = 1.0   # position only; no orientation is claimed
                self.pub_pose.publish(pose)
        elif depth_m is None:
            # Common and benign on chrome, but say so: a consumer waiting on a
            # pose should know why none arrived.
            self.get_logger().debug("button found but no valid depth at that pixel")

        if self.get_parameter("publish_debug_image").value:
            self._publish_debug(frame, msg.header, res)

    def _publish_debug(self, frame, header, res):
        import cv2
        vis = frame.copy()
        c = res.get("center")
        if c is not None:
            q = res.get("quad")
            if q is not None:
                cv2.polylines(vis, [q.reshape(-1, 2).astype(np.int32)], True, (0, 255, 0), 2)
            p = (int(round(c[0])), int(round(c[1])))
            cv2.circle(vis, p, 14, (0, 0, 255), 3)
            cv2.drawMarker(vis, p, (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
            txt = f"LOCKED inliers={res.get('inliers')}"
            col = (0, 255, 0)
        else:
            txt = f"ABSTAIN ({res.get('reason', 'no fit')})"
            col = (0, 190, 255)
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(vis, txt, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        out = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        out.header = header
        self.pub_dbg.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ButtonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
