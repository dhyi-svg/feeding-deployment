"""Live wrist-camera view: colour on the left, aligned depth (colour-mapped, 0.2-1.5 m) on the right.
Run as the sheppy node `camera_view`, or by hand after sourcing microwave/sheppy/env.sh:

    python3 microwave/tools/camera_view.py          # q / Esc closes the window
"""
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

NEAR_MM, FAR_MM = 200, 1500


class CameraView(Node):
    def __init__(self):
        super().__init__("microwave_camera_view")
        self.bridge, self.color, self.depth = CvBridge(), None, None
        self.create_subscription(Image, "/wrist_camera/color/image_raw", self._on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/wrist_camera/aligned_depth_to_color/image_raw", self._on_depth,
                                 qos_profile_sensor_data)

    def _on_color(self, m):
        self.color = self.bridge.imgmsg_to_cv2(m, "bgr8")      # the camera publishes rgb8

    def _on_depth(self, m):
        d = self.bridge.imgmsg_to_cv2(m).astype(np.float32)
        scaled = np.clip((d - NEAR_MM) / (FAR_MM - NEAR_MM), 0, 1)
        vis = cv2.applyColorMap((255 * (1 - scaled)).astype(np.uint8), cv2.COLORMAP_JET)
        vis[d == 0] = 0                                          # no depth -> black
        self.depth = vis


def main():
    rclpy.init()
    node = CameraView()
    cv2.namedWindow("wrist camera", cv2.WINDOW_NORMAL)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.03)
        if node.color is not None:
            frame = node.color if node.depth is None or node.depth.shape != node.color.shape \
                else np.hstack([node.color, node.depth])
            cv2.imshow("wrist camera", frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
