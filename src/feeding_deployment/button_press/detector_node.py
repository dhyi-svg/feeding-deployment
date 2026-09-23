#!/usr/bin/env python3
"""ROS 2 node wrapping the reference-homography START/+30SEC button detector.

Run (or via launch/ros2/button_press_bringup.launch.py)::

    python3 -u -m feeding_deployment.button_press.detector_node --ros-args \
        -p reference_dir:=$HOME/wrist_ref_red -p target_button:=timer_clock

Publishes
    ~/button_pixel   geometry_msgs/PointStamped   pixel (x, y, 0) in the image frame
    ~/claw_pixel     geometry_msgs/PointStamped   pixel of the LEFT gripper finger tip
                                                  (constant -- see LEFT_CLAW_PIXEL)
    ~/button_pose    geometry_msgs/PoseStamped    3D point in the camera optical frame
                                                  (only when depth + camera_info are available)
    ~/debug_image    sensor_msgs/Image            annotated frame, for eyeballing
    ~/panel_quad     geometry_msgs/PolygonStamped the reference crop's corners projected into
                                                  the frame (the fitted panel); only when locked
    ~/status         std_msgs/String              every processed frame: "locked <target>
                                                  inliers=N view=V" or "abstain <reason>" -- so a
                                                  consumer can tell "abstaining" from "not running"

Subscribes (optional, overlay only -- nothing here depends on them)
    /press_detector/force_dev   geometry_msgs/Vector3Stamped   baseline-subtracted tool
                                                               force dF from
                                                               press_detector.py
    /press_detector/pressed     std_msgs/Bool                  its press/release state
    These come from ``python3 -u -m feeding_deployment.button_press.press_detector --publish``
    (a separate, read-only Kortex session). The overlay shows |dF| and a CONTACT banner while pressed,
    holds the last press's peak for a moment after release, and falls back to "no data"
    once the feed goes quiet -- so a dead press detector never looks like "not pressed".

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
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point32, PointStamped, PolygonStamped, PoseStamped, Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

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
# Pixel of the LEFT gripper finger's tip (the highest point of its top edge)
# in the colour image. HARDCODED, not detected: the camera is mounted on the
# wrist, so the fingers sit at the same pixels in every frame regardless of
# where the arm is -- confirmed on two very different framings of the
# microwave on 2026-09-21 (rchi-cpu-5, D435i 640x480). Measured, not eyeballed:
# V<40 dark-blob top edge over 7 frames peaked over x=384..399 at y=394, so the
# centre of that flat is the tip. A live dark-blob detector was tried first and
# rejected because the shelf's shadow merges into the finger in some framings.
# It DOES move if the gripper opening changes; re-measure then. Override with
# the left_claw_pixel parameter. Both fingers are at the bottom of the frame:
# left one spans ~x 342-465, right one ~x 476-640.
# 2026-09-21 (later): re-measured on a live frame, the flat top now spans
# x=375..399 at y~393, so the centre moved ~6 px left of the first measurement
# (391 was sitting at the flat's right end). Shifted to 385, then to 377 on
# the user's visual check of the live overlay.
LEFT_CLAW_PIXEL = (377, 394)
# Press-detector feed is considered dead after this long without a message. The
# detector publishes every sample (~50 Hz), so half a second of silence is real.
FORCE_STALE_S = 0.5
# After a release, keep showing that press's peak for this long so a short press
# (~0.4 s by hand, see TESTING_LOG) survives the 5 Hz debug-image rate.
PRESS_HOLD_S = 1.5


class ButtonDetectorNode(Node):
    def __init__(self):
        super().__init__("button_detector")
        self.declare_parameter("reference_dir", "")
        # Which marked button to report: a key of the reference's per-view
        # "buttons" table ("" = the reference's default_button / legacy mark).
        self.declare_parameter("target_button", "")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("rate_limit_hz", 5.0)
        self.declare_parameter("left_claw_pixel", list(LEFT_CLAW_PIXEL))
        self.declare_parameter("force_topic", "/press_detector/force_dev")
        self.declare_parameter("pressed_topic", "/press_detector/pressed")

        ref = self.get_parameter("reference_dir").value
        if not ref:
            raise SystemExit(
                "reference_dir parameter is required -- it points at the directory "
                "holding reference.json for THIS appliance. There is one reference "
                "per microwave; a reference built on another unit will not work."
            )
        target = self.get_parameter("target_button").value or None
        self.det = ReferenceButtonDetector(ref, target=target)
        self.get_logger().info(
            f"loaded reference from {ref} ({len(self.det.views)} view(s)), "
            f"target button {self.det.target!r}")

        self.bridge = CvBridge()
        self.depth = None
        self.info = None
        self._last_run_ns = 0
        self._min_period_ns = int(1e9 / max(0.1, self.get_parameter("rate_limit_hz").value))

        self.pub_px = self.create_publisher(PointStamped, "~/button_pixel", 10)
        self.pub_claw = self.create_publisher(PointStamped, "~/claw_pixel", 10)
        cp = self.get_parameter("left_claw_pixel").value
        self.claw_px = (int(cp[0]), int(cp[1]))
        self.pub_pose = self.create_publisher(PoseStamped, "~/button_pose", 10)
        self.pub_dbg = self.create_publisher(Image, "~/debug_image", 1)
        self.pub_quad = self.create_publisher(PolygonStamped, "~/panel_quad", 10)
        self.pub_status = self.create_publisher(String, "~/status", 10)

        self.create_subscription(Image, self.get_parameter("depth_topic").value,
                                 self._on_depth, 10)
        self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                 self._on_info, 10)
        self.create_subscription(Image, self.get_parameter("image_topic").value,
                                 self._on_image, 10)

        # Press-detector overlay state. All wall-clock (time.monotonic) so it works
        # whether or not the publisher's stamps share this node's clock.
        self.force_dev = None       # np.ndarray(3) of the last dF, or None
        self.force_rx_t = None      # monotonic time of the last force message
        self.pressed = False
        self.press_peak = 0.0       # max |dF| during the current/last press
        self.release_t = None       # monotonic time of the last release, for the hold
        self.create_subscription(Vector3Stamped, self.get_parameter("force_topic").value,
                                 self._on_force, 10)
        self.create_subscription(Bool, self.get_parameter("pressed_topic").value,
                                 self._on_pressed, 10)

    def _on_depth(self, msg):
        self.depth = msg

    def _on_info(self, msg):
        self.info = msg

    def _on_force(self, msg):
        self.force_dev = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
        self.force_rx_t = time.monotonic()
        if self.pressed:
            self.press_peak = max(self.press_peak, float(np.linalg.norm(self.force_dev)))

    def _on_pressed(self, msg):
        if msg.data and not self.pressed:
            self.press_peak = 0.0 if self.force_dev is None else float(np.linalg.norm(self.force_dev))
        elif not msg.data and self.pressed:
            self.release_t = time.monotonic()
        self.pressed = bool(msg.data)

    def _force_status(self):
        """(text, colour_bgr, is_contact) for the bottom banner."""
        now = time.monotonic()
        if self.force_rx_t is None:
            return "FORCE: no data (start button_press.press_detector --publish)", (160, 160, 160), False
        if now - self.force_rx_t > FORCE_STALE_S:
            return f"FORCE: STALE {now - self.force_rx_t:.1f}s -- press detector stopped?", (160, 160, 160), False
        mag = float(np.linalg.norm(self.force_dev))
        if self.pressed:
            return f"CONTACT  |dF| {mag:5.1f} N   peak {self.press_peak:5.1f} N", (0, 0, 255), True
        if self.release_t is not None and now - self.release_t < PRESS_HOLD_S:
            return f"released  peak {self.press_peak:5.1f} N   |dF| {mag:4.1f} N", (0, 140, 255), False
        return f"|dF| {mag:4.2f} N", (0, 255, 0), False

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

        claw = PointStamped()
        claw.header = msg.header
        claw.point.x, claw.point.y, claw.point.z = float(self.claw_px[0]), float(self.claw_px[1]), 0.0
        self.pub_claw.publish(claw)

        if center is None:
            # Deliberately publishes no pixel/pose. See the module docstring. The
            # status string is the one exception: it says *that* we abstained, never where.
            self.get_logger().debug(
                f"abstained: {res.get('reason', 'no fit')} (inliers={res.get('inliers', 0)})")
            self.pub_status.publish(String(data=f"abstain {res.get('reason', 'no fit')}"))
            if self.get_parameter("publish_debug_image").value:
                self._publish_debug(frame, msg.header, res)
            return

        u, v = int(round(center[0])), int(round(center[1]))
        px = PointStamped()
        px.header = msg.header
        px.point.x, px.point.y, px.point.z = float(u), float(v), 0.0
        self.pub_px.publish(px)
        self.pub_status.publish(String(
            data=f"locked {self.det.target} inliers={res.get('inliers')} view={res.get('view')}"))
        quad = res.get("quad")
        if quad is not None:
            poly = PolygonStamped()
            poly.header = msg.header
            for qx, qy in quad.reshape(-1, 2):
                poly.polygon.points.append(Point32(x=float(qx), y=float(qy), z=0.0))
            self.pub_quad.publish(poly)

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
            txt = f"LOCKED {self.det.target} inliers={res.get('inliers')}"
            col = (0, 255, 0)
        else:
            txt = f"ABSTAIN ({res.get('reason', 'no fit')})"
            col = (0, 190, 255)
        # Left claw tip: same colour the older overlay scripts used for the
        # gripper point. When locked, also show the claw->button pixel offset.
        cv2.drawMarker(vis, self.claw_px, (255, 220, 0), cv2.MARKER_TILTED_CROSS, 20, 2)
        cv2.circle(vis, self.claw_px, 3, (255, 220, 0), -1)
        if c is not None:
            cv2.line(vis, self.claw_px, p, (255, 220, 0), 1, cv2.LINE_AA)
            txt += f"   claw->button dx={p[0] - self.claw_px[0]:+d} dy={p[1] - self.claw_px[1]:+d}px"
        cv2.rectangle(vis, (0, 0), (vis.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(vis, txt, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        # Bottom banner: tool-force contact from the press detector. Red frame border
        # while in contact so it is obvious even at rqt thumbnail size.
        ftxt, fcol, contact = self._force_status()
        h, w = vis.shape[:2]
        cv2.rectangle(vis, (0, h - 30), (w, h), (0, 0, 0), -1)
        cv2.putText(vis, ftxt, (10, h - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.6, fcol, 2, cv2.LINE_AA)
        if contact:
            cv2.rectangle(vis, (0, 0), (w - 1, h - 1), (0, 0, 255), 6)
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
