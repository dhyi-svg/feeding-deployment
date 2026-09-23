"""ROS 2 side of the button press: one rclpy node holding the latest of everything.

Subscribes to the button detector (``/button_detector/*``), the press detector
(``/press_detector/*``), the RealSense colour/aligned-depth/camera_info topics and tf2,
and spins itself on a daemon thread. The driver (``autonomous_press.Run``) reads from it;
nothing here moves the arm.
"""
from __future__ import annotations

import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PolygonStamped, Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from feeding_deployment.button_press import Abort
from feeding_deployment.button_press.geometry import PlaneFitError, fit_plane_inverse_depth, pixel_ray

# Frames to median-filter the button pixel over before steering on it. A single frame is
# not safe: on 2026-09-21 the pixel sat at (372.7, 399.5) +-(2.2, 0.5) px while the node
# matched reference view 3, but ~1 frame in 30 matched a DIFFERENT view and placed the
# button 29 px away in x. The servo took instantaneous samples, swallowed those outliers
# and lurched (commanded zero x correction, measured +9 px of x change), never settling
# under PX_TOL. A median rejects them; measured p90 error vs the long-run mean was
# 3.31 px for 1 frame, 2.18 for 4, 1.65 for 8, 1.31 for 12.
PX_SAMPLES = 9
PX_SAMPLES_MIN = 3           # accept a shorter median rather than failing outright
FORCE_STALE_S = 0.5          # press detector considered dead after this silence
FORCE_SETTLE_S = 0.3         # wait this long after a step before reading force
# ---- perception gates ------------------------------------------------------------------
MIN_INLIERS = 12             # to START a run (preflight); 6-8-inlier locks have picked the wrong dome
MIN_INLIERS_TRACK = 8        # to accept a re-servo correction mid-transit (a weak lock just skips it)
LOCK_HOLD_S = 2.0
FRESH_S = 1.0
PANEL_SAT_MIN = 100          # HSV saturation above which a pixel is red panel, not chrome
DEPTH_RANGE_M = (0.10, 1.5)
# (the plane-fit trims PLANE_Z_BAND_M / PLANE_TRIM_M live in geometry.py)


class Perception(Node):
    def __init__(self, ns: str, press_ns: str, arm_frame: str, cam_frame: str):
        super().__init__("press_button_autonomous")
        self.arm_frame, self.cam_frame = arm_frame, cam_frame
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest: dict[str, tuple[float, object]] = {}
        self.force_hist: deque[tuple[float, float]] = deque(maxlen=200)  # (t, |dF|)
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        def keep(name, conv=lambda m: m):
            def cb(msg):
                with self.lock:
                    self.latest[name] = (time.monotonic(), conv(msg))
            return cb

        self.create_subscription(PointStamped, f"{ns}/button_pixel",
                                 keep("button_px", lambda m: np.array([m.point.x, m.point.y])), 10)
        self.create_subscription(PointStamped, f"{ns}/claw_pixel",
                                 keep("claw_px", lambda m: np.array([m.point.x, m.point.y])), 10)
        self.create_subscription(String, f"{ns}/status", keep("status", lambda m: m.data), 10)
        self.create_subscription(PolygonStamped, f"{ns}/panel_quad",
                                 keep("quad", lambda m: np.array([[q.x, q.y] for q in m.polygon.points])), 10)
        self.create_subscription(CameraInfo, "/camera/color/camera_info", keep("info"), 10)
        self.create_subscription(Image, "/camera/color/image_raw", keep("color"), 1)
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", keep("depth"), 1)
        self.create_subscription(Bool, f"{press_ns}/pressed", keep("pressed", lambda m: bool(m.data)), 10)

        def on_force(msg):
            v = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
            with self.lock:
                self.latest["force"] = (time.monotonic(), v)
                self.force_hist.append((time.monotonic(), float(np.linalg.norm(v))))
        self.create_subscription(Vector3Stamped, f"{press_ns}/force_dev", on_force, 50)

        self._thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self._thread.start()

    # -- accessors ---------------------------------------------------------------------------
    def get(self, name, max_age=None):
        with self.lock:
            item = self.latest.get(name)
        if item is None:
            return None
        t, v = item
        if max_age is not None and time.monotonic() - t > max_age:
            return None
        return v

    def age(self, name):
        with self.lock:
            item = self.latest.get(name)
        return None if item is None else time.monotonic() - item[0]

    def wait_after(self, name, t_after, timeout):
        """Latest value of `name` received strictly after monotonic time t_after, or None.

        Needed after a move: the detector node runs at ~5 Hz and a value up to
        FRESH_S old can predate the motion, which made the servo act on pre-move
        pixels (each iteration then only removed ~half the error, 2026-09-21).
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                item = self.latest.get(name)
            if item is not None and item[0] > t_after:
                return item[1]
            time.sleep(0.05)
        return None

    def median_after(self, name, t_after, n_samples=PX_SAMPLES, timeout=4.0):
        """Median of up to `n_samples` DISTINCT values of `name` received after t_after.

        Outlier-rejecting counterpart to wait_after(): see PX_SAMPLES for why a single
        frame is not safe to steer on. Returns None if fewer than PX_SAMPLES_MIN arrive
        before the timeout.
        """
        deadline = time.monotonic() + timeout
        seen = []
        last_t = t_after
        while time.monotonic() < deadline and len(seen) < n_samples:
            with self.lock:
                item = self.latest.get(name)
            if item is not None and item[0] > last_t:
                last_t = item[0]
                seen.append(item[1])
            else:
                time.sleep(0.02)
        if len(seen) < PX_SAMPLES_MIN:
            return None
        return np.median(np.stack(seen), axis=0)

    def wait(self, name, timeout, max_age=FRESH_S):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            v = self.get(name, max_age)
            if v is not None:
                return v
            time.sleep(0.05)
        return None

    def locked(self, min_inliers=MIN_INLIERS) -> tuple[bool, str]:
        st = self.get("status", FRESH_S)
        if st is None:
            return False, "no status (node not running?)"
        if not st.startswith("locked"):
            return False, st
        try:
            inl = int(st.split("inliers=")[1].split()[0])
        except (IndexError, ValueError):
            inl = 0
        return inl >= min_inliers, st

    def lock_target(self) -> str | None:
        st = self.get("status", FRESH_S)
        return st.split()[1] if st and st.startswith("locked") and len(st.split()) > 1 else None

    def force_at_rest(self, window_s=FORCE_SETTLE_S) -> float:
        """Median |dF| over the last window; NaN if the feed is stale."""
        if (self.age("force") or 1e9) > FORCE_STALE_S:
            return float("nan")
        now = time.monotonic()
        with self.lock:
            vals = [m for t, m in self.force_hist if now - t <= window_s]
        return float(np.median(vals)) if vals else float("nan")

    def intrinsics(self):
        info = self.get("info")
        if info is None:
            raise Abort("no camera_info")
        return info.k[0], info.k[4], info.k[2], info.k[5]

    def cam_rotation_in_base(self) -> np.ndarray:
        """3x3 rotation taking camera-frame directions into arm-base directions."""
        from scipy.spatial.transform import Rotation as R  # noqa: PLC0415
        try:
            tr = self.tfbuf.lookup_transform(self.arm_frame, self.cam_frame, Time(),
                                             timeout=Duration(seconds=2.0))
        except Exception as e:  # noqa: BLE001
            raise Abort(f"tf {self.arm_frame} <- {self.cam_frame} unavailable: {e}\n"
                        "  /joint_states silent? restart joint_state_bridge; verify with\n"
                        f"  ros2 run tf2_ros tf2_echo {self.arm_frame} {self.cam_frame}")
        q = tr.transform.rotation
        return R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    def ray(self, px) -> np.ndarray:
        return pixel_ray(px, *self.intrinsics())

    def panel_plane(self):
        """Plane n.X = d (camera frame) fitted to the red panel inside the homography quad.

        Chrome domes (low saturation) are excluded: they are exactly where the depth
        sensor lies. Returns (n, d, median_depth, n_points).
        """
        quad = self.get("quad", FRESH_S)
        color = self.get("color", FRESH_S)
        depth = self.get("depth", FRESH_S)
        if quad is None or color is None or depth is None:
            raise Abort("panel plane: need fresh quad + color + aligned depth "
                        f"(quad {self.age('quad')}, color {self.age('color')}, depth {self.age('depth')})")
        bgr = self.bridge.imgmsg_to_cv2(color, "bgr8")
        d = self.bridge.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
        if depth.encoding in ("16UC1", "mono16"):
            d = d / 1000.0
        h, w = d.shape[:2]
        mask = np.zeros((h, w), np.uint8)
        cv2.fillConvexPoly(mask, quad.astype(np.int32), 255)
        sat = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[..., 1]
        ok = (mask > 0) & (sat > PANEL_SAT_MIN) & np.isfinite(d) & (d > DEPTH_RANGE_M[0]) & (d < DEPTH_RANGE_M[1])
        vs, us = np.where(ok)
        # OLS on inverse depth, not SVD -- see fit_plane_inverse_depth for why.
        try:
            return fit_plane_inverse_depth(us, vs, d[vs, us], *self.intrinsics())
        except PlaneFitError as e:
            raise Abort(f"panel plane: {e}") from e
