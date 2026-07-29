"""ROS 2 (Humble) support for the feeding-deployment stack.

The repo targets ROS 1 Noetic. This package holds the ROS 2 equivalents used by
the single-machine Jetson deployment: a shared rclpy node, ROS1<->ROS2 message
compat adapters, a joint-state bridge over the repo's own arm RPC, a RealSense
topic consumer, and the hand-eye calibration static publisher.

Nothing here depends on any other ROS 2 workspace -- only on rclpy, tf2_ros,
sensor_msgs, cv_bridge and the repo's own interfaces.
"""
