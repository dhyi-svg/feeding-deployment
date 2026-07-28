"""One-off perception smoke test: run the repo's real, unmodified
AppliancePerception.detect_handle_and_placement against the live Pachirisu camera,
substituting a monkeypatched get_frame_to_frame_transform (built from the
2026-07-22 cv2.calibrateHandEye() result + a live arm.get_state() read) for the
tf2 lookup this repo normally expects, since nothing on this box publishes a TF
tree. No arm motion -- read-only get_state() only. See
~/.claude/plans/quiet-herding-puzzle.md for the full design.
"""

import json
import os
import types
from pathlib import Path

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)
from feeding_deployment.perception.appliance_perception.appliance_perception import (
    AppliancePerception,
)
from feeding_deployment.perception.grounded_sam import GroundedSAM

CALIB_PATH = os.path.expanduser(
    "~/deployment_ws/pachirisu_wrist_camera_calib/wrist_camera_calib_fullboard.json"
)


def _make_transform_fn(arm_interface, calib):
    """Returns a get_frame_to_frame_transform(camera_info, ...) replacement bound
    to a fresh arm.get_state() read each call, composed with the fixed eye-in-hand
    calibration -- same composition validated to 1.1mm on a real robot move."""
    R_cam2gripper = np.array(calib["R_cam2gripper"])
    t_cam2gripper = np.array(calib["t_cam2gripper"])

    override_norm = os.environ.get("CALIB_TCAM_NORM_OVERRIDE")
    if override_norm is not None:
        override_norm = float(override_norm)
        original_norm = np.linalg.norm(t_cam2gripper)
        t_cam2gripper = t_cam2gripper * (override_norm / original_norm)
        print(
            f"[detect-handle] CALIB_TCAM_NORM_OVERRIDE set: rescaling t_cam2gripper "
            f"norm {original_norm:.4f}m -> {override_norm:.4f}m (same direction)"
        )

    def get_frame_to_frame_transform(camera_info_data, frame_A=None, target_frame=None):
        ee_pos = np.asarray(arm_interface.get_state()["ee_pos"], dtype=float)
        t_gripper2base = ee_pos[:3]
        R_gripper2base = Rotation.from_quat(ee_pos[3:7]).as_matrix()

        R_base_to_camera = R_gripper2base @ R_cam2gripper
        t_base_to_camera = R_gripper2base @ t_cam2gripper + t_gripper2base
        quat = Rotation.from_matrix(R_base_to_camera).as_quat()  # xyzw

        transform = types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(
                    x=t_base_to_camera[0], y=t_base_to_camera[1], z=t_base_to_camera[2]
                ),
                rotation=types.SimpleNamespace(
                    x=quat[0], y=quat[1], z=quat[2], w=quat[3]
                ),
            )
        )
        return transform

    return get_frame_to_frame_transform


def _depth_to_mm(depth_msg, bridge):
    encoding = depth_msg.encoding
    print(f"[detect-handle] depth encoding: {encoding}")
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    if encoding == "16UC1":
        return depth.astype(np.float32)  # already mm
    if encoding == "32FC1":
        return (depth.astype(np.float32)) * 1000.0  # m -> mm
    raise ValueError(f"Unexpected depth encoding: {encoding}")


def main():
    rospy.init_node("test_detect_handle_and_placement", anonymous=True, disable_signals=True)

    print("[detect-handle] connecting to arm_server (read-only get_state only)...")
    ArmManager.register("ArmInterface")
    mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    mg.connect()
    arm_interface = mg.ArmInterface()
    ee_pos = arm_interface.get_state()["ee_pos"]
    print(f"[detect-handle] arm connected, ee_pos={list(np.round(ee_pos, 4))}")

    print("[detect-handle] grabbing one camera frame...")
    bridge = CvBridge()
    rgb_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10)
    camera_info_msg = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=10)
    depth_msg = rospy.wait_for_message(
        "/camera/aligned_depth_to_color/image_raw", Image, timeout=10
    )
    rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    depth_mm = _depth_to_mm(depth_msg, bridge)
    print(f"[detect-handle] frame grabbed: rgb {rgb_image.shape}, depth {depth_mm.shape}")

    with open(CALIB_PATH) as f:
        calib = json.load(f)
    print(f"[detect-handle] loaded calibration from {CALIB_PATH} ({calib['n_samples']} samples)")

    print("[detect-handle] loading GroundedSAM (GroundingDINO Swin-B)...")
    gsam = GroundedSAM()
    # Stub out the lazy SAM predictor: AppliancePerception.__init__ eagerly reads
    # grounded_sam.sam_predictor (a post-merge regression -- see chat), which would
    # otherwise force-load ViT-H; the SAM checkpoint isn't even present on this box,
    # and the appliance/handle detection path never touches SAM. Scratch-script-only
    # workaround, not a repo change.
    gsam._sam_predictor = object()
    appliance_perception = AppliancePerception(gsam)
    appliance_perception.get_frame_to_frame_transform = _make_transform_fn(
        arm_interface, calib
    )

    log_dir = Path(__file__).parent / "log" / "test_detect_handle_and_placement"
    log_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(log_dir / "input_rgb.png"), rgb_image)

    for handle_type in ("microwave handle", "microwave"):
        print(f"\n[detect-handle] === trying handle_type={handle_type!r} ===")
        result = appliance_perception.detect_handle_and_placement(
            handle_type, rgb_image, camera_info_msg, depth_mm
        )
        handle_pose, hinge_pose, placement_pose, top_pose = result
        if handle_pose is not None:
            print(f"[detect-handle] SUCCESS with handle_type={handle_type!r}")
            print(f"  handle_pose:    {handle_pose}")
            print(f"  hinge_pose:     {hinge_pose}")
            print(f"  placement_pose: {placement_pose}")
            print(f"  top_pose:       {top_pose}")
            break
        print(f"[detect-handle] handle_type={handle_type!r} produced no pose (see diagnostics above)")
    else:
        print("\n[detect-handle] both handle_type attempts failed -- see per-stage diagnostics above")

    for name, image in appliance_perception._last_images.items():
        out_path = log_dir / f"{name}.png"
        cv2.imwrite(str(out_path), image)
        print(f"[detect-handle] saved {out_path}")

    rospy.signal_shutdown("test_detect_handle_and_placement complete")


if __name__ == "__main__":
    main()
