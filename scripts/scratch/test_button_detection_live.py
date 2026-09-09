"""Live, read-only test of button-press detection against the real camera.

No arm motion -- this only runs perception (detect_start_button) and prints/saves
the result. Safe to run any time the camera is up.

Usage (inside the feed-noetic container, ros_env activated, roscore + camera
already running per PACHIRISU_SETUP.md):

    export PYTHONPATH=/opt/msgs_ws/devel/lib/python3.11/site-packages:$PYTHONPATH
    BUTTON_BACKEND=hough_circles python scripts/scratch/test_button_detection_live.py

BUTTON_BACKEND: molmo | grounding_dino | hough_circles | auto (default auto).
Ctrl-C to stop. Saves the last annotated frame to
scripts/scratch/test_button_detection_live_output.png each iteration.
"""
import time

import cv2
import rospy

from feeding_deployment.interfaces.realsense_interface import RealSenseInterface
from feeding_deployment.perception.appliance_perception.appliance_perception import (
    AppliancePerception,
)
from feeding_deployment.perception.grounded_sam import GroundedSAM

OUT_PATH = "scripts/scratch/test_button_detection_live_output.png"

rospy.init_node("test_button_detection_live")

print("Loading GroundedSAM (GroundingDINO)...")
grounded_sam = GroundedSAM()
appliance_perception = AppliancePerception(grounded_sam)
print(f"Button backend: {appliance_perception.button_backend}")

print("Waiting for camera data...")
realsense = RealSenseInterface()

camera_data = None
while not rospy.is_shutdown():
    camera_data = realsense.get_camera_data()
    if camera_data["rgb_image"] is not None:
        break
    time.sleep(0.1)

print("Got camera data. Running detect_start_button in a loop (Ctrl-C to stop)...")
while not rospy.is_shutdown():
    camera_data = realsense.get_camera_data()
    if camera_data["rgb_image"] is None:
        print("No camera data, waiting...")
        time.sleep(1.0)
        continue

    result = appliance_perception.detect_start_button(
        camera_data["rgb_image"],
        camera_data["camera_info"],
        camera_data["depth_image"],
    )
    print("Result (base_to_button pose):", result)

    vis = appliance_perception._last_images.get("rgb_button_pixel")
    if vis is not None:
        cv2.imwrite(OUT_PATH, vis)
        print(f"Saved annotated frame to {OUT_PATH}")

    time.sleep(1.0)
