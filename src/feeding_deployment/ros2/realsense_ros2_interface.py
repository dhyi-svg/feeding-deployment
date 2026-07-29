"""ROS 2 equivalent of :mod:`feeding_deployment.interfaces.realsense_interface`.

Subscribes to the ``realsense2_camera`` ROS 2 driver and exposes the exact same
``get_camera_data()`` contract the ROS 1 interface does, so every consumer
(``PerceptionInterface``, ``AppliancePerception``, ...) is unchanged::

    {"rgb_image": np.ndarray (bgr8),
     "camera_info": CameraInfoCompat,
     "depth_image": np.ndarray (float32, millimetres),
     "header": std_msgs/Header}

Two deliberate differences from the ROS 1 version:

* ``camera_info`` is wrapped in :class:`~feeding_deployment.ros2.compat.CameraInfoCompat`
  so downstream ``camera_info.K[0]`` keeps working -- ROS 2 renamed the field to
  lowercase ``k`` (see that module).
* The synchroniser is *approximate* rather than exact. The ROS 1 stack used
  ``TimeSynchronizer``; under the ROS 2 driver colour and aligned-depth stamps
  can differ by a frame interval, which an exact policy silently drops forever.

The driver must publish aligned depth (``align_depth.enable:=true``) -- the whole
handle pipeline assumes depth pixels line up with colour pixels.
"""

from __future__ import annotations

import time
from copy import deepcopy
from threading import Lock

import message_filters
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image

from feeding_deployment.ros2.compat import CameraInfoCompat
from feeding_deployment.ros2.node import get_node

DEFAULT_COLOR_TOPIC = "/camera/color/image_raw"
DEFAULT_CAMERA_INFO_TOPIC = "/camera/color/camera_info"
DEFAULT_DEPTH_TOPIC = "/camera/aligned_depth_to_color/image_raw"


class RealSenseROS2Interface:
    """Latest synchronised RGB-D frame from the ROS 2 RealSense driver."""

    def __init__(
        self,
        color_topic: str = DEFAULT_COLOR_TOPIC,
        camera_info_topic: str = DEFAULT_CAMERA_INFO_TOPIC,
        depth_topic: str = DEFAULT_DEPTH_TOPIC,
        queue_size: int = 10,
        slop_sec: float = 0.05,
        wait_for_first_frame_sec: float = 0.0,
    ) -> None:
        self.camera_lock = Lock()
        self.camera_header = None
        self.camera_color_data = None
        self.camera_info_data = None
        self.camera_depth_data = None

        self.bridge = CvBridge()
        self._node = get_node()

        # Images are published BEST_EFFORT by the RealSense driver; a RELIABLE
        # subscription would never match and the callback would never fire.
        from rclpy.qos import qos_profile_sensor_data

        self.color_image_sub = message_filters.Subscriber(
            self._node, Image, color_topic, qos_profile=qos_profile_sensor_data
        )
        self.camera_info_sub = message_filters.Subscriber(
            self._node, CameraInfo, camera_info_topic, qos_profile=qos_profile_sensor_data
        )
        self.depth_image_sub = message_filters.Subscriber(
            self._node, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_image_sub, self.camera_info_sub, self.depth_image_sub],
            queue_size=queue_size,
            slop=slop_sec,
        )
        self._sync.registerCallback(self.rgbdCallback)

        self._node.get_logger().info(
            f"RealSense (ROS 2) subscribed: {color_topic}, {camera_info_topic}, {depth_topic}"
        )

        if wait_for_first_frame_sec > 0:
            self.wait_for_frames(wait_for_first_frame_sec)

    def rgbdCallback(self, rgb_image_msg, camera_info_msg, depth_image_msg) -> None:
        """Store the latest synchronised triple (name kept from the ROS 1 class)."""
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
            # The driver publishes aligned depth as 16UC1 millimetres. Cast to
            # float32 WITHOUT rescaling: every consumer downstream divides by
            # 1000 itself (see AppliancePerception.pixel2World).
            depth_raw = self.bridge.imgmsg_to_cv2(depth_image_msg, "passthrough")
            depth_image = np.asarray(depth_raw, dtype=np.float32)
        except CvBridgeError as e:
            self._node.get_logger().warn(f"cv_bridge conversion failed: {e}")
            return

        with self.camera_lock:
            self.camera_color_data = rgb_image
            self.camera_info_data = CameraInfoCompat(camera_info_msg)
            self.camera_depth_data = depth_image
            self.camera_header = rgb_image_msg.header

    def get_camera_data(self) -> dict:
        """The latest frame, in the same shape the ROS 1 interface returns."""
        with self.camera_lock:
            return {
                "rgb_image": deepcopy(self.camera_color_data),
                # CameraInfoCompat wraps a live message; copying it would deep-copy
                # the ROS message for no benefit, and it is read-only anyway.
                "camera_info": self.camera_info_data,
                "depth_image": deepcopy(self.camera_depth_data),
                "header": deepcopy(self.camera_header),
            }

    def has_frames(self) -> bool:
        with self.camera_lock:
            return self.camera_color_data is not None and self.camera_depth_data is not None

    def wait_for_frames(self, timeout_sec: float = 10.0) -> bool:
        """Block until a synchronised RGB-D frame arrives. True if one did."""
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            if self.has_frames():
                return True
            time.sleep(0.05)
        self._node.get_logger().warn(
            f"No synchronised RGB-D frame within {timeout_sec:g}s. Is realsense2_camera "
            f"running with align_depth.enable:=true?"
        )
        return False
