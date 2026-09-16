"""Live button detection -> real 3D robot-base-frame pose. READ-ONLY: prints
the computed pose every loop, never calls move_to_ee_pose or anything else
that moves the arm.

This is the connect-the-two-halves step: our own classical-CV detector
(radius-outlier filtering, row-layout plausibility, temporal-consensus
stability -- see render_button_detection_overlay.py, which this file
reuses) feeding into the same pixel -> 3D -> robot-frame math the real
`AppliancePerception.detect_start_button` uses, so this prints exactly the
kind of pose `press_microwave_button.py` would receive.

Only the detection algorithm is ours; the coordinate-transform math
(pixel2World deprojection, tf2 lookup, homogeneous-transform composition)
is COPIED (not imported) from:
  - appliance_perception.py: pixel2World, the fixed button-orientation quat
  - tf_interface.py: get_frame_to_frame_transform, make_homogeneous_transform,
    matrix_to_pose
No feeding_deployment modules are imported for the math itself, per the
"copy-paste, don't touch repo code" rule for this scratch work. Camera and
ROS plumbing (RealSenseInterface, rospy, tf2_ros) IS imported -- that's
hardware/framework access, not detection logic, and reimplementing a camera
driver would be pointless.

Requires the real rig's ROS environment (RoboStack ros_env on Pachirisu, or
the ROS 2 bring-up on the Jetson) with roscore + the RealSense camera
already running -- see PACHIRISU_SETUP.md / JETSON_SETUP.md. Will not run
on a plain machine like this Mac; there's no camera, no calibration, no
arm_base_link tf tree here.

Usage (inside the ROS env, camera + tf tree already up):
    python scripts/scratch/detect_button_pose_live.py

Ctrl-C to stop.
"""
import math
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import rospy
import tf2_ros
from scipy.spatial.transform import Rotation

# Reuse our own detection algorithm rather than re-copying it a third time.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_button_detection_overlay import (  # noqa: E402
    StabilityTracker,
    detect_circles_multiscale,
    pick_start_button,
)

from feeding_deployment.interfaces.realsense_interface import RealSenseInterface  # noqa: E402


# -- copied from appliance_perception.py: pixel2World -----------------------
def pixel2world(camera_info, image_x, image_y, depth_image):
    """Deproject a pixel + its depth into a 3D point in the camera's own
    frame. Copied verbatim (trimmed of the surrounding-pixel fallback,
    which this read-only script doesn't need) from
    AppliancePerception.pixel2World."""
    if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
        return False, None

    depth = depth_image[image_y, image_x]
    depth = depth / 1000  # mm -> m
    if math.isnan(depth) or depth < 0.05 or depth > 2.0:
        return False, None

    fx = camera_info.K[0]
    fy = camera_info.K[4]
    cx = camera_info.K[2]
    cy = camera_info.K[5]

    world_x = (depth / fx) * (image_x - cx)
    world_y = (depth / fy) * (image_y - cy)
    world_z = depth
    return True, (world_x, world_y, world_z)


# -- copied from tf_interface.py: transform lookup + composition ------------
def make_homogeneous_transform(transform):
    a_to_b = np.zeros((4, 4))
    a_to_b[:3, :3] = Rotation.from_quat(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
    ).as_matrix()
    a_to_b[:3, 3] = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
    )
    a_to_b[3, 3] = 1
    return a_to_b


def matrix_to_pose(mat):
    position = mat[:3, 3]
    orientation = Rotation.from_matrix(mat[:3, :3]).as_quat()
    return position, orientation


# Fixed button-approach orientation, copied from
# AppliancePerception.detect_start_button (line ~455) -- rig-specific, not
# recomputed here, only carried through so the printed pose has the same
# shape a real caller (perceive_button_pressing_poses) would see.
BUTTON_ORIENTATION_QUAT = [-0.5, 0.5, 0.5, -0.5]


def flip_pixel(x, y, width, height):
    """The wrist camera is mounted upside down; our detector's row/corner
    logic assumes a right-side-up view (it was built against right-side-up
    phone footage). Flip the raw frame's pixel to the visually-upright
    frame before detecting, then flip the chosen pixel back before using it
    with the raw (un-flipped) depth image and intrinsics."""
    return (width - x, height - y)


def main():
    rospy.init_node("detect_button_pose_live")

    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    print("Waiting for camera data...")
    realsense = RealSenseInterface()
    camera_data = None
    while not rospy.is_shutdown():
        camera_data = realsense.get_camera_data()
        if camera_data["rgb_image"] is not None:
            break
        time.sleep(0.1)

    print("Got camera data. Detecting + computing pose (Ctrl-C to stop, READ-ONLY, no motion) ...")
    tracker = StabilityTracker()

    while not rospy.is_shutdown():
        camera_data = realsense.get_camera_data()
        rgb_image = camera_data["rgb_image"]
        depth_image = camera_data["depth_image"]
        camera_info = camera_data["camera_info"]
        if rgb_image is None or depth_image is None or camera_info is None:
            print("No camera data, waiting...")
            time.sleep(1.0)
            continue

        height, width = rgb_image.shape[:2]
        # Upright view for detection (camera is mounted upside down).
        upright = cv2.rotate(rgb_image, cv2.ROTATE_180)
        gray = cv2.cvtColor(upright, cv2.COLOR_BGR2GRAY)

        candidates, band = detect_circles_multiscale(gray)
        if candidates is None:
            print("No plausible button candidates this frame.")
            tracker.reset()
            time.sleep(0.5)
            continue

        chosen = pick_start_button(candidates)
        stable = tracker.update(chosen["center"])
        status = "STABLE" if stable else "unconfirmed"
        print(
            f"{len(candidates)} candidates (band {band}), picked "
            f"{chosen['center']} in the upright view [{status}]"
        )

        if not stable:
            time.sleep(0.5)
            continue

        # Upright pixel -> raw (un-flipped) pixel, to match depth_image/camera_info.
        raw_x, raw_y = flip_pixel(
            chosen["center"][0], chosen["center"][1], width, height
        )
        raw_x, raw_y = int(round(raw_x)), int(round(raw_y))

        ok, button_cam_xyz = pixel2world(camera_info, raw_x, raw_y, depth_image)
        if not ok:
            print(f"No valid depth at raw pixel ({raw_x}, {raw_y}); skipping.")
            time.sleep(0.5)
            continue

        try:
            transform = tf_buffer.lookup_transform(
                "arm_base_link", "camera_color_optical_frame", rospy.Time(0)
            )
        except Exception as e:  # noqa: BLE001 -- read-only diagnostic script
            print(f"tf lookup failed ({e}); is the calibration/tf tree up?")
            time.sleep(1.0)
            continue

        base_to_camera = make_homogeneous_transform(transform)
        camera_to_button = np.eye(4)
        camera_to_button[:3, 3] = button_cam_xyz
        base_to_button = base_to_camera @ camera_to_button
        base_to_button[:3, :3] = Rotation.from_quat(BUTTON_ORIENTATION_QUAT).as_matrix()

        position, orientation = matrix_to_pose(base_to_button)
        print(
            f"  -> button pose in arm_base_link: "
            f"position={np.round(position, 3).tolist()} "
            f"orientation={np.round(orientation, 3).tolist()}"
        )
        print("  (READ-ONLY: not moving the arm.)")

        time.sleep(1.0)


if __name__ == "__main__":
    main()
