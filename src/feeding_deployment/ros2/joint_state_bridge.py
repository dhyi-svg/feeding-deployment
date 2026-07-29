"""Publish the arm's live joint state into ROS 2, from the repo's own arm RPC.

The lab runs the Kinova through a ROS driver, so ``/joint_states`` exists and
``robot_state_publisher`` can build the TF tree. This deployment talks to the arm
over the repo's own RPC (``arm_server.py`` / :class:`ArmInterfaceClient`) and has
no ROS driver at all -- so nothing publishes joint states, and without them there
is no ``base_link`` -> ``end_effector_link`` transform for tf2 to look up.

This bridge closes that gap: it polls ``ArmInterfaceClient.get_state()`` and
republishes it as ``sensor_msgs/JointState``. Point ``robot_state_publisher`` at
the Kinova ``gen3.xacro`` description and the whole arm TF chain comes up, which
is what the hand-eye calibration (``end_effector_link`` ->
``camera_color_optical_frame``) hangs off.

Read-only: it never commands the arm.

Run standalone::

    ARM_RPC_HOST=127.0.0.1 python -m feeding_deployment.ros2.joint_state_bridge
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

from feeding_deployment.ros2.node import get_node

# Actuated arm joints of the 7-DOF Gen3, in the order get_state() returns them
# and named as kortex_description's gen3.xacro emits them with no prefix.
GEN3_JOINT_NAMES = [f"joint_{i}" for i in range(1, 8)]

# The Robotiq 2F-85's driven joint. get_state()["gripper_pos"] is a 0..1 (or
# 0..100) opening fraction; the knuckle joint spans 0..0.8 rad.
GRIPPER_JOINT_NAME = "robotiq_85_left_knuckle_joint"
GRIPPER_JOINT_MAX_RAD = 0.8

DEFAULT_RATE_HZ = 50.0


def _gripper_position_rad(gripper_pos) -> float | None:
    """Convert the arm's gripper reading to the knuckle joint angle in radians.

    Accepts either a 0..1 fraction or a 0..100 percentage; anything unusable
    returns None so the caller simply omits the gripper joint.
    """
    if gripper_pos is None:
        return None
    try:
        value = float(np.asarray(gripper_pos).reshape(-1)[0])
    except (TypeError, ValueError, IndexError):
        return None
    if not np.isfinite(value):
        return None
    if value > 1.0:  # reported as a percentage
        value /= 100.0
    return float(np.clip(value, 0.0, 1.0) * GRIPPER_JOINT_MAX_RAD)


class JointStateBridge:
    """Polls the arm RPC and publishes ``sensor_msgs/JointState``."""

    def __init__(
        self,
        robot_interface,
        topic: str = "/joint_states",
        rate_hz: float = DEFAULT_RATE_HZ,
        publish_gripper: bool = True,
    ) -> None:
        self.robot_interface = robot_interface
        self.publish_gripper = publish_gripper
        self._node = get_node()
        self._pub = self._node.create_publisher(JointState, topic, 10)
        self._timer = self._node.create_timer(1.0 / rate_hz, self._tick)
        self._warned = False
        self._node.get_logger().info(
            f"Joint-state bridge publishing {topic} at {rate_hz:g} Hz"
        )

    def _tick(self) -> None:
        try:
            state = self.robot_interface.get_state()
        except Exception as e:  # noqa: BLE001 -- a transient RPC blip must not kill the timer
            if not self._warned:
                self._node.get_logger().warn(f"get_state() failed: {e}")
                self._warned = True
            return
        self._warned = False

        positions = np.asarray(state["position"], dtype=float).reshape(-1)
        if positions.size < len(GEN3_JOINT_NAMES):
            self._node.get_logger().warn(
                f"get_state() returned {positions.size} joints, "
                f"expected {len(GEN3_JOINT_NAMES)}"
            )
            return

        msg = JointState()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.name = list(GEN3_JOINT_NAMES)
        msg.position = [float(v) for v in positions[: len(GEN3_JOINT_NAMES)]]

        velocity = np.asarray(state.get("velocity", []), dtype=float).reshape(-1)
        if velocity.size >= len(GEN3_JOINT_NAMES):
            msg.velocity = [float(v) for v in velocity[: len(GEN3_JOINT_NAMES)]]
        effort = np.asarray(state.get("effort", []), dtype=float).reshape(-1)
        if effort.size >= len(GEN3_JOINT_NAMES):
            msg.effort = [float(v) for v in effort[: len(GEN3_JOINT_NAMES)]]

        if self.publish_gripper:
            finger = _gripper_position_rad(state.get("gripper_pos"))
            if finger is not None:
                msg.name.append(GRIPPER_JOINT_NAME)
                msg.position.append(finger)
                # Only positions stay aligned with `name` once the gripper is
                # appended; drop the arm-only velocity/effort rather than
                # publish a ragged message.
                msg.velocity = []
                msg.effort = []

        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--rate", type=float, default=DEFAULT_RATE_HZ)
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="publish the 7 arm joints only",
    )
    args = parser.parse_args()

    # Imported here so `--help` works without the arm RPC stack present.
    from feeding_deployment.control.robot_controller.arm_client import (
        ArmInterfaceClient,
    )

    robot_interface = ArmInterfaceClient()
    node = get_node()
    JointStateBridge(
        robot_interface,
        topic=args.topic,
        rate_hz=args.rate,
        publish_gripper=not args.no_gripper,
    )
    node.get_logger().info("Joint-state bridge running; Ctrl-C to stop.")
    # get_node() already spins this node on its own executor thread, so just
    # park the main thread -- calling spin_once() here would double-spin it.
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
