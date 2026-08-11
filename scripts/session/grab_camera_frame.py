"""READ-ONLY: save one RGB frame from the live ROS 2 camera, to check framing.

The wrist camera is eye-in-hand and no script repositions the arm, so you must
verify by eye that the microwave is actually in view before running detection.
See JETSON_TESTING.md for what has to be in frame.

Usage: grab_camera_frame.py [output.png]   (default /tmp/view.png)
"""
import sys

import cv2

from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface

out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/view.png"

rs = RealSenseROS2Interface()
if not rs.wait_for_frames(30.0):
    raise SystemExit("No RGB-D frames -- is microwave_bringup.launch.py running?")

image = rs.get_camera_data()["rgb_image"]
cv2.imwrite(out, image)
print(f"saved {out}  shape={image.shape}")
