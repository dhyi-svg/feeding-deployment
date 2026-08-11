"""READ-ONLY: is the tf2 camera pose consistent with the arm's real EE pose?

Replaces the "expect [0.255, 0.018, 0.565]" check in JETSON_SETUP.md / the runbook,
which is WRONG as an absolute: the camera is eye-in-hand, so
arm_base_link -> camera_color_optical_frame depends entirely on the current joint
configuration. There is no fixed expected value.

The real invariant is the EE -> camera offset, which must match the saved hand-eye
calibration (~7 cm). A broken joint bridge or calibration shows up as METRES of
disagreement, not centimetres. A few cm of residual is expected here: this compares
positions only and ignores the optical-frame rotation.

Needs arm_server.py and microwave_bringup.launch.py running. Goes through the arm
RPC, so it does not fight for the Kortex session the way arm_probe_state.py would.
"""
import os

import numpy as np
import yaml

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)
from feeding_deployment.perception.tf_interface import TFInterface
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface

CALIB_PATH = os.path.expanduser(
    "~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib"
)
TOLERANCE_M = 0.05

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
state = manager.ArmInterface().get_state()
ee = np.asarray(list(state["ee_pos"])[:3], dtype=float)
gripper = float(state.get("gripper_pos"))

print(f"arm EE (arm_base_link)   : {np.round(ee, 3)}")
print(f"joints (deg)             : {[round(float(np.degrees(v)), 1) for v in state['position']]}")
print(f"gripper                  : {gripper:.4f}  ({'open' if gripper < 0.2 else 'CLOSED'})")

rs = RealSenseROS2Interface()
if not rs.wait_for_frames(30.0):
    raise SystemExit("No RGB-D frames -- is microwave_bringup.launch.py running?")
camera_info = rs.get_camera_data()["camera_info"]

transform = TFInterface().get_frame_to_frame_transform(
    camera_info, "arm_base_link", "camera_color_optical_frame"
)
if transform is None:
    raise SystemExit("TF lookup returned None -- the tf tree is not up.")
t = transform.transform.translation
cam = np.array([t.x, t.y, t.z])
print(f"tf2 camera position      : {np.round(cam, 3)}")

calib = yaml.safe_load(open(CALIB_PATH))["transform"]["translation"]
expected = float(np.linalg.norm(np.array([calib["x"], calib["y"], calib["z"]])))
measured = float(np.linalg.norm(cam - ee))
print(f"EE -> camera distance    : {measured * 100:.1f} cm")
print(f"calibrated offset norm   : {expected * 100:.1f} cm")
print(
    "VERDICT                  :",
    "OK"
    if abs(measured - expected) < TOLERANCE_M
    else "SUSPECT -- check the joint bridge and calibration before any motion",
)
