#!/usr/bin/env python3
"""
shared_autonomy_teleop.py

Xbox-controller teleop for the Vention base, for use in SHARED AUTONOMY.

Unlike vention_teleop_controller.py (which opens the Arduino serial port directly
and therefore cannot coexist with autonomous navigation), this node does NOT
touch the motors. It only:

  * reads the Xbox controller (via pygame, same as the standalone script), and
  * publishes geometry_msgs/Twist to /cmd_vel ONLY while the deadman is held, and
  * emits two high-level intents to the shared_autonomy_manager:
        - /shared_autonomy/takeover (std_msgs/Empty) on the rising edge of the
          deadman button  -> "human is taking over, cancel the autopilot"
        - /shared_autonomy/done     (std_msgs/Empty) on the rising edge of the
          done button     -> "human has parked the robot, report success"

Because it publishes Twist (instead of grabbing serial), it can run side-by-side
with autonomous navigation. When the deadman is not held it publishes nothing,
so it has zero effect on the robot.

Safety:
  * Deadman: motion is published only while the deadman button is held. On
    release we publish a single zero Twist, then go silent; the bridge watchdog
    keeps the base stopped.
  * Velocity is clamped to the autonomy's limits (max_vel_x / max_vel_theta) so a
    human cannot out-drive the safe envelope.
  * A missing/disconnected controller never blocks autonomy: the node stays
    dormant and keeps trying to (re)connect.
"""

from feeding_deployment.ros2_utils import node_handle
from feeding_deployment.ros2_utils import rospy_compat as rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import pygame


def apply_deadband(value: float, deadband: float) -> float:
    if abs(value) < deadband:
        return 0.0
    return value


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SharedAutonomyTeleop:
    def __init__(self) -> None:
        # ---- Topics ----
        # TODO(ros2): "~name" (rospy private param) has no exact rclpy
        # equivalent; declared as plain parameter names on this node instead.
        _node = node_handle.get_node()
        self.cmd_vel_topic = _node.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self.takeover_topic = _node.declare_parameter(
            "takeover_topic", "/shared_autonomy/takeover"
        ).value
        self.done_topic = _node.declare_parameter(
            "done_topic", "/shared_autonomy/done"
        ).value

        # ---- Controller mapping (pygame indices; Xbox defaults) ----
        # Left stick: axis 1 = forward/back (up is negative), axis 0 = left/right.
        self.axis_linear = int(_node.declare_parameter("axis_linear", 1).value)
        self.axis_angular = int(_node.declare_parameter("axis_angular", 0).value)
        # RB (right bumper) is commonly button 5; Start/Menu is commonly button 7.
        self.deadman_button = int(_node.declare_parameter("deadman_button", 5).value)
        self.done_button = int(_node.declare_parameter("done_button", 7).value)

        # ---- Velocity limits ----
        # Required (no defaults): the launch file is the single source of truth
        # for teleop speed, which is intentionally faster than the autonomy's
        # TEB limits. Missing params fail the node loudly at startup.
        # TODO(ros2): rospy.get_param(name) with no default raised if missing;
        # declare_parameter requires SOME default/type in rclpy unless
        # ParameterDescriptor(dynamic_typing=True) is used -- declaring with
        # float("nan") as a sentinel and asserting it was actually overridden.
        self.max_vel_x = float(_node.declare_parameter("max_vel_x", float("nan")).value)
        assert self.max_vel_x == self.max_vel_x, "required parameter 'max_vel_x' not set"
        self.max_vel_theta = float(_node.declare_parameter("max_vel_theta", float("nan")).value)
        assert self.max_vel_theta == self.max_vel_theta, "required parameter 'max_vel_theta' not set"
        self.deadband = float(_node.declare_parameter("deadband", 0.12).value)

        # Flip these if a stick drives the robot the wrong way on real hardware.
        self.invert_linear = bool(_node.declare_parameter("invert_linear", True).value)
        self.invert_angular = bool(_node.declare_parameter("invert_angular", True).value)

        self.rate_hz = float(_node.declare_parameter("rate", 20.0).value)
        self.joystick_id = int(_node.declare_parameter("joystick_id", 0).value)

        # ---- Publishers ----
        self.cmd_pub = _node.create_publisher(Twist, self.cmd_vel_topic, 1)
        self.takeover_pub = _node.create_publisher(Empty, self.takeover_topic, 1)
        self.done_pub = _node.create_publisher(Empty, self.done_topic, 1)

        # ---- State ----
        self.joystick = None
        self.prev_deadman = False
        self.prev_done = False

        pygame.init()
        pygame.joystick.init()
        self._try_open_joystick()

    # ------------------------------------------------------------------ #
    # Joystick connection handling
    # ------------------------------------------------------------------ #
    def _try_open_joystick(self) -> bool:
        """(Re)open the controller. Returns True if a controller is available."""
        try:
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() <= self.joystick_id:
                self.joystick = None
                return False
            self.joystick = pygame.joystick.Joystick(self.joystick_id)
            self.joystick.init()
            rospy.loginfo(
                "shared_autonomy_teleop: controller connected: %s",
                self.joystick.get_name(),
            )
            return True
        except pygame.error as exc:
            rospy.logwarn_throttle(
                5.0, "shared_autonomy_teleop: joystick init failed: %s", exc
            )
            self.joystick = None
            return False

    # ------------------------------------------------------------------ #
    # Stick mixing
    # ------------------------------------------------------------------ #
    def _compute_twist(self) -> Twist:
        x = apply_deadband(self.joystick.get_axis(self.axis_angular), self.deadband)
        y = apply_deadband(self.joystick.get_axis(self.axis_linear), self.deadband)

        # Stick up (negative axis) -> forward.
        lin = -y if self.invert_linear else y
        # Stick left (negative axis) -> turn left (positive angular.z, REP-103).
        ang = -x if self.invert_angular else x

        twist = Twist()
        twist.linear.x = clamp(lin * self.max_vel_x, -self.max_vel_x, self.max_vel_x)
        twist.angular.z = clamp(
            ang * self.max_vel_theta, -self.max_vel_theta, self.max_vel_theta
        )
        return twist

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #
    def spin(self) -> None:
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo(
            "shared_autonomy_teleop running. Hold button %d (deadman) to take over "
            "and drive; press button %d (done) to report goal reached.",
            self.deadman_button,
            self.done_button,
        )
        while not rospy.is_shutdown():
            # No controller: stay dormant, keep trying to (re)connect.
            if self.joystick is None:
                self._try_open_joystick()
                rate.sleep()
                continue

            try:
                pygame.event.pump()
                deadman = bool(self.joystick.get_button(self.deadman_button))
                done = bool(self.joystick.get_button(self.done_button))
            except pygame.error as exc:
                rospy.logwarn("shared_autonomy_teleop: controller read failed: %s", exc)
                self.joystick = None
                # Make sure we don't leave the base creeping if it dropped mid-drive.
                if self.prev_deadman:
                    self.cmd_pub.publish(Twist())
                self.prev_deadman = False
                self.prev_done = False
                rate.sleep()
                continue

            # Rising edge of deadman = the takeover request.
            if deadman and not self.prev_deadman:
                self.takeover_pub.publish(Empty())
                rospy.loginfo("shared_autonomy_teleop: takeover requested.")

            # Rising edge of done = goal-reached request.
            if done and not self.prev_done:
                self.done_pub.publish(Empty())
                rospy.loginfo("shared_autonomy_teleop: done (goal reached) requested.")

            if deadman:
                # Drive while held.
                self.cmd_pub.publish(self._compute_twist())
            elif self.prev_deadman:
                # Falling edge: publish a single stop, then stay silent.
                self.cmd_pub.publish(Twist())

            self.prev_deadman = deadman
            self.prev_done = done
            rate.sleep()


def main() -> None:
    node_handle.init_node("shared_autonomy_teleop")
    SharedAutonomyTeleop().spin()


if __name__ == "__main__":
    main()
