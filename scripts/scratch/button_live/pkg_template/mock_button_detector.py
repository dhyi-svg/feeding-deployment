#!/usr/bin/env python3
"""Mock alternative for the button detector.

RAMMP's pre-PR checklist requires "a mock alternative next to the real one", and
it is useful well before that: it lets the rest of the stack run with no camera
and no microwave in front of the robot.

It publishes a FIXED pixel and pose on the same topics as the real node, at the
same rate. It deliberately does NOT pretend to abstain or to vary -- a mock that
imitates failure modes invites someone to test failure handling against
fiction. This one is obviously, constantly fake.
"""
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node


class MockButtonDetector(Node):
    def __init__(self):
        super().__init__("button_detector")
        self.declare_parameter("pixel_x", 320.0)
        self.declare_parameter("pixel_y", 240.0)
        self.declare_parameter("pose_xyz", [0.0, 0.0, 0.45])
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("rate_hz", 5.0)

        self.pub_px = self.create_publisher(PointStamped, "~/button_pixel", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "~/button_pose", 10)
        hz = self.get_parameter("rate_hz").value
        self.create_timer(1.0 / max(0.1, hz), self._tick)
        self.get_logger().warn("MOCK button detector -- publishing a FIXED pose, not perceiving anything")

    def _tick(self):
        now = self.get_clock().now().to_msg()
        frame = self.get_parameter("frame_id").value

        px = PointStamped()
        px.header.stamp, px.header.frame_id = now, frame
        px.point.x = float(self.get_parameter("pixel_x").value)
        px.point.y = float(self.get_parameter("pixel_y").value)
        self.pub_px.publish(px)

        xyz = self.get_parameter("pose_xyz").value
        pose = PoseStamped()
        pose.header.stamp, pose.header.frame_id = now, frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, xyz)
        pose.pose.orientation.w = 1.0
        self.pub_pose.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = MockButtonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
