#!/usr/bin/env python3
"""
cmd_vel_bridge_basicmicro.py

ROS1 node that converts geometry_msgs/Twist on /cmd_vel into differential-drive
left/right motor commands for the Vention base (BasicMicro + Arduino bridge).

The Arduino lives on the NUC; this node runs on the compute box (where move_base /
teleop publish /cmd_vel) and is a pure /cmd_vel -> RPC translator. It sends the
mixed left/right speeds to BaseInterface on the NUC via BaseInterfaceClient. It does
NOT own a lost-command watchdog: the bridge only emits set_speeds in response to
/cmd_vel, so every failure mode (publisher death, this node crashing, compute hang,
network drop) shows up at the NUC as "set_speeds stopped arriving" and is caught by
BaseInterface's authoritative timeout there. A watchdog here could not cover a
compute hang anyway (its timer would freeze with the box).

Used by BOTH autonomous nav (move_base/TEB -> /cmd_vel) and the shared-autonomy
Xbox teleop, so it must faithfully reproduce whatever (v, w) it is handed.

Keeps combined translate+turn intact by:
  1) Mixing linear+angular into left/right, then clamping each wheel symmetrically
     to +/-max_speed_units.
  2) Angular deadband near zero to avoid sign-flip jitter.
  3) Overcoming motor stiction with a RATIO-PRESERVING floor: when the dominant
     wheel is below min_move_units, both wheels are scaled up by the same factor.
     This keeps the robot moving at slow speeds (important for autonomous nav /
     localization) WITHOUT distorting the curvature. NOTE: the previous per-wheel
     minimum did distort it -- it snapped any gentle arc to pure-straight or
     pure-spin, so the base could only ever translate OR rotate, never both.
"""

import math
import os
import sys
import traceback

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

def add_ros_vention_src_to_path():
    try:
        import rospkg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("feeding_deployment")
        src_path = os.path.join(pkg_path, "src")
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        return True
    except Exception as e:
        rospy.logwarn(f"Failed to add feeding_deployment/src to PYTHONPATH via rospkg: {e}")
        return False
# The base driver now lives on the NUC behind an RPC server; this node talks to it
# via BaseInterfaceClient. Make sure feeding_deployment/src is on PYTHONPATH (handled
# by add_ros_vention_src_to_path / your catkin workspace).

class CmdVelBridgeBasicmicro:
    def __init__(self):
        add_ros_vention_src_to_path()
        try:
            from feeding_deployment.control.base_controller.base_client import BaseInterfaceClient
            self.base = BaseInterfaceClient()
        except Exception:
            rospy.logerr("Failed to connect to base RPC server (is base_server.py running on the NUC?):\n" + traceback.format_exc())
            raise
        # ---- ROS params ----
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        # Convert m/s and rad/s into "speed units" used by your motor controller.
        # Start with these, then tune:
        # - If robot is too slow: increase linear_scale
        # - If steering is too weak/strong: adjust angular_scale
        self.linear_scale = float(rospy.get_param("~linear_scale", 1000.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 800.0))

        # Deadband on angular velocity (rad/s) to avoid sign-flip jitter.
        self.w_deadband = float(rospy.get_param("~w_deadband", 0.03))

        # Clamp output units.
        # max_speed_units should match what your controller expects safely.
        self.max_speed_units = int(rospy.get_param("~max_speed_units", 800))

        # Ratio-preserving stiction floor (units). When a command is nonzero but
        # the dominant wheel falls below this, BOTH wheels are scaled up by the
        # same factor so the robot moves while the left/right ratio (hence the
        # commanded curvature) is preserved. Set to 0 to disable.
        # IMPORTANT: do NOT reintroduce a per-wheel minimum here -- forcing each
        # wheel up independently snaps gentle arcs to pure-straight/pure-spin,
        # which is the "translate OR rotate, never both" bug this replaced.
        self.min_move_units = int(rospy.get_param("~min_move_units", 250))

        # If your wiring is flipped, you can invert left/right or swap outputs:
        self.invert_left = bool(rospy.get_param("~invert_left", False))
        self.invert_right = bool(rospy.get_param("~invert_right", False))
        self.swap_left_right = bool(rospy.get_param("~swap_left_right", False))

        # If your robot turns the wrong way for positive angular.z, flip rotation sign:
        self.flip_angular = bool(rospy.get_param("~flip_angular", False))

        # NOTE: the lost-command stop lives on the NUC (BaseInterface), not here.
        # This node only translates /cmd_vel into set_speeds RPC calls.

        # ---- Safety hold (ZED-divergence interlock) ----
        # zed_health_monitor asserts /nav_safety_hold when ZED tracking diverges
        # (odom becomes garbage). While held, we command zero regardless of
        # /cmd_vel, so the base stops until the ZED recovers. Gating here -- the
        # single point to the motors -- guarantees the stop for both nav and
        # teleop. Fail-safe: once we've ever heard the monitor, a stale flag
        # (monitor died) also holds; if the monitor was never launched, we never
        # hold (backward compatible).
        self.hold_topic = rospy.get_param("~safety_hold_topic", "/nav_safety_hold")
        self.hold_stale_s = float(rospy.get_param("~safety_hold_stale_s", 1.0))
        self.safety_hold = False
        self.hold_last_msg = None

        # ---- ROS wiring ----
        self.sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cb, queue_size=10)
        self.hold_sub = rospy.Subscriber(self.hold_topic, Bool, self.cb_hold, queue_size=5)

        # Diagnostics echo of the effectively-applied command (after the
        # stiction floor and clamp), converted back to m/s / rad/s so it can
        # be overlaid against the incoming /cmd_vel.
        self.applied_pub = rospy.Publisher("~applied", Twist, queue_size=10)

        rospy.loginfo("cmd_vel bridge running. Waiting for %s...", self.cmd_vel_topic)

    @staticmethod
    def _clamp(x: int, max_abs: int) -> int:
        """Symmetric clamp of integer x to [-max_abs, max_abs]."""
        if max_abs <= 0:
            return x
        return max(-max_abs, min(max_abs, x))

    def _apply_output_mapping(self, right: int, left: int):
        """Optional swapping/inverting to match your motor wiring."""
        if self.swap_left_right:
            right, left = left, right
        if self.invert_right:
            right = -right
        if self.invert_left:
            left = -left
        return right, left

    def cb_hold(self, msg: Bool):
        self.safety_hold = bool(msg.data)
        self.hold_last_msg = rospy.Time.now()

    def _held(self) -> bool:
        """True if the ZED-divergence interlock says stop. Fail-safe: if we have
        ever heard the monitor but its flag is stale, hold. Never heard it -> no
        hold (backward compatible with setups that don't launch the monitor)."""
        if self.hold_last_msg is None:
            return False
        if self.safety_hold:
            return True
        return (rospy.Time.now() - self.hold_last_msg).to_sec() > self.hold_stale_s

    def cb(self, msg: Twist):
        # Safety interlock: ZED tracking diverged -> command zero, ignore /cmd_vel.
        if self._held():
            try:
                self.base.set_speeds(0, 0)
            except Exception:
                rospy.logerr("Motor command failed (safety hold):\n%s", traceback.format_exc())
            rospy.logwarn_throttle(2.0, "cmd_vel bridge: safety HOLD active "
                                   "(ZED tracking) -- commanding zero")
            return

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        if self.flip_angular:
            w = -w

        # Deadband on angular velocity to prevent jitter around zero
        if abs(w) < self.w_deadband:
            w = 0.0

        # Convert to controller "units"
        lin_units = int(v * self.linear_scale)
        rot_units = int(w * self.angular_scale)

        # Differential drive mix: right = lin + rot, left = lin - rot
        right = lin_units + rot_units
        left = lin_units - rot_units

        # Ratio-preserving stiction floor: if the command is nonzero but the
        # dominant wheel is below min_move_units, scale BOTH wheels up by the
        # same factor. This keeps the robot moving at slow speeds without
        # distorting the curvature (the left/right ratio is preserved), unlike a
        # per-wheel minimum which would snap gentle arcs to pure-straight/spin.
        peak = max(abs(right), abs(left))
        if self.min_move_units > 0 and 0 < peak < self.min_move_units:
            scale = self.min_move_units / float(peak)
            right = int(round(right * scale))
            left = int(round(left * scale))

        right = self._clamp(right, self.max_speed_units)
        left = self._clamp(left, self.max_speed_units)

        # Echo applied command (pre-wiring-mapping, in the /cmd_vel convention).
        applied = Twist()
        applied.linear.x = (right + left) / 2.0 / self.linear_scale
        w_applied = (right - left) / 2.0 / self.angular_scale
        applied.angular.z = -w_applied if self.flip_angular else w_applied
        self.applied_pub.publish(applied)

        right, left = self._apply_output_mapping(right, left)

        try:
            # NOTE: BaseInterface.set_speeds(speed_a, speed_b) over RPC.
            # If speed_a maps to right and speed_b maps to left in your hardware, this is correct.
            # If reversed, set ~swap_left_right:=true.
            self.base.set_speeds(right, left)
        except Exception:
            rospy.logerr("Motor command failed:\n%s", traceback.format_exc())


def main():
    rospy.init_node("cmd_vel_bridge_basicmicro", anonymous=False)
    _ = CmdVelBridgeBasicmicro()
    rospy.spin()


if __name__ == "__main__":
    main()