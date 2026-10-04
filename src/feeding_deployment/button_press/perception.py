"""ROS 2 side of the button press: one rclpy node holding the latest camera frames, plus tf2.

Subscribes to the RealSense colour / aligned-depth / camera_info topics and listens to tf2,
and spins itself on a daemon thread. ``press_button.Run`` reads from it; nothing here moves
the arm.
"""
from __future__ import annotations

import threading
import time

import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from feeding_deployment.button_press import Abort

FRESH_S = 1.0                # a frame older than this is stale


def to_bgr_depth(bridge: CvBridge, color_msg: Image, depth_msg: Image):
    """(bgr8 image, float32 depth in metres) from a colour + aligned-depth message pair."""
    bgr = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    depth = bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float32)
    if depth_msg.encoding in ("16UC1", "mono16"):   # the RealSense driver publishes millimetres
        depth /= 1000.0
    return bgr, depth


class Perception(Node):
    def __init__(self, arm_frame: str, cam_frame: str):
        super().__init__("press_button")
        self.arm_frame, self.cam_frame = arm_frame, cam_frame
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.latest: dict[str, tuple[float, object]] = {}
        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        def keep(name):
            def cb(msg):
                with self.lock:
                    self.latest[name] = (time.monotonic(), msg)
            return cb

        self.create_subscription(CameraInfo, "/camera/color/camera_info", keep("info"), 10)
        self.create_subscription(Image, "/camera/color/image_raw", keep("color"), 1)
        self.create_subscription(Image, "/camera/aligned_depth_to_color/image_raw", keep("depth"), 1)

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

    def wait(self, name, timeout, max_age=FRESH_S):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            v = self.get(name, max_age)
            if v is not None:
                return v
            time.sleep(0.05)
        return None

    def intrinsics(self):
        info = self.get("info")
        if info is None:
            raise Abort("no camera_info")
        return info.k[0], info.k[4], info.k[2], info.k[5]

    def _cam_in_base(self):
        try:
            return self.tfbuf.lookup_transform(self.arm_frame, self.cam_frame, Time(),
                                               timeout=Duration(seconds=2.0)).transform
        except Exception as e:  # noqa: BLE001
            raise Abort(f"tf {self.arm_frame} <- {self.cam_frame} unavailable: {e}\n"
                        "  /joint_states silent? restart joint_state_bridge; verify with\n"
                        f"  ros2 run tf2_ros tf2_echo {self.arm_frame} {self.cam_frame}")

    def cam_rotation_in_base(self) -> np.ndarray:
        """3x3 rotation taking camera-frame directions into arm-base directions."""
        from scipy.spatial.transform import Rotation as R  # noqa: PLC0415
        q = self._cam_in_base().rotation
        return R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

    def cam_position_in_base(self) -> np.ndarray:
        """Camera optical-frame origin in the arm base frame (tf2), metres."""
        t = self._cam_in_base().translation
        return np.array([t.x, t.y, t.z])
