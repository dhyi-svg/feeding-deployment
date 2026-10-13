"""Publish the wrist camera's easy_handeye2 hand-eye calibration as tf.

RAMMP publishes no calibrated wrist-camera tf (its manifests run the camera and the arm driver
only), so this node completes ``base_link -> ... -> end_effector_link`` (robot_state_publisher on
the driver's URDF, sheppy node ``arm_tf``) with ``end_effector_link -> camera_color_optical_frame``.
Port of feeding_deployment/ros2/calibration_tf.py without the repo dependency.

An eye-in-hand ``.calib`` stores ``robot_effector_frame -> tracking_base_frame``. Sent once on
``/tf_static`` AND re-sent on ``/tf`` at REPEAT_HZ with fresh stamps: on 2026-07-29 a listener
with several static publishers got only some of them ("two or more unconnected trees") and a
detection died mid-run; the repeat is what actually guarantees delivery.

    ros2 run rammp_button_press calibration_tf --calib ~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

DEFAULT_CALIB = Path.home() / ".ros2" / "easy_handeye2" / "calibrations" / "wrist_camera_calib.calib"
REPEAT_HZ = 20.0


def load_calibration(path) -> dict:
    """(parent, child, xyz, quat xyzw) of an eye-in-hand easy_handeye2 ``.calib`` file."""
    path = Path(path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"no hand-eye calibration at {path} (pass --calib)")
    data = yaml.safe_load(path.read_text())
    params, tf = data["parameters"], data["transform"]
    if params.get("calibration_type", "eye_in_hand") != "eye_in_hand":
        raise ValueError(f"{path}: expected an eye_in_hand calibration, got {params.get('calibration_type')!r}")
    t, r = tf["translation"], tf["rotation"]
    return {"parent": params["robot_effector_frame"], "child": params["tracking_base_frame"],
            "xyz": (float(t["x"]), float(t["y"]), float(t["z"])),
            "quat": (float(r["x"]), float(r["y"]), float(r["z"]), float(r["w"])), "path": str(path)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=str(DEFAULT_CALIB))
    a, _ = ap.parse_known_args(argv)
    calib = load_calibration(a.calib)

    rclpy.init()
    node = rclpy.create_node("wrist_camera_calibration_tf")

    def make():
        m = TransformStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = calib["parent"], calib["child"]
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = calib["xyz"]
        (m.transform.rotation.x, m.transform.rotation.y,
         m.transform.rotation.z, m.transform.rotation.w) = calib["quat"]
        return m

    static, dynamic = StaticTransformBroadcaster(node), TransformBroadcaster(node)
    static.sendTransform(make())
    node.create_timer(1.0 / REPEAT_HZ, lambda: dynamic.sendTransform(make()))
    node.get_logger().info(f"{calib['parent']} -> {calib['child']} from {calib['path']} "
                           f"(static, and repeated on /tf at {REPEAT_HZ:g} Hz)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
