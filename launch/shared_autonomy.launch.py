"""ROS2 port of shared_autonomy.launch.

Shared-autonomy bring-up.

Prerequisites (start these first, as usual):
  - Localization: cartographer_localization.launch.py (map->odom) and the
    fused IMU+wheel EKF from sensors.launch.py (odom->vention_base_link).
    These MUST stay running during a human takeover so TF stays valid.
  - Sensors: lidar topics.

This launch starts:
  - navigation.launch.py: TODO(ros2) -- currently only starts the cmd_vel
    bridge, NOT move_base (no ROS2 equivalent yet; see that file).
  - shared_autonomy_manager: serves the "navigate" action, forwards to
    move_base (TODO(ros2): forwards to whatever the eventual Nav2 nav
    action ends up being -- "navigate_to_pose" is Nav2's usual
    action name, NOT "move_base"; the shared_autonomy_manager.py source
    itself needs a human decision here once Nav2 is wired up, tracked
    separately from this launch-file conversion), handles takeover/done.
  - shared_autonomy_teleop: Xbox controller -> /cmd_vel (deadman-gated) +
    takeover/done intents.

NavigateHLA connects to "navigate" by default, so it is routed through the
manager automatically. (Set FEEDING_NAV_ACTION=move_base to bypass --
TODO(ros2): that bypass env var's value itself references the now-defunct
move_base action name; flagged for the same human decision as above.)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    navigation_launch = os.path.join(
        get_package_share_directory("feeding_deployment"),
        "launch",
        "navigation.launch.py",
    )

    return LaunchDescription(
        [
            # move_base + cmd_vel bridge. (The ZED-divergence interlock and
            # its zed_interlock/NO_ZED_INTERLOCK flag were unplumbed
            # 2026-07-13, see navigation.launch.py; nav rides the fused
            # IMU+wheel EKF now.)
            IncludeLaunchDescription(PythonLaunchDescriptionSource(navigation_launch)),
            # Manager: navigate action server -> move_base passthrough
            # (TODO(ros2): move_base passthrough target needs updating
            # once Nav2 lands, see module docstring).
            Node(
                package="feeding_deployment",
                executable="shared_autonomy_manager.py",
                name="shared_autonomy_manager",
                output="screen",
                parameters=[
                    {"navigate_action": "navigate", "move_base_action": "move_base"}
                ],
            ),
            # Teleop: Xbox controller -> Twist on /cmd_vel (deadman-gated).
            Node(
                package="feeding_deployment",
                executable="shared_autonomy_teleop.py",
                name="shared_autonomy_teleop",
                output="screen",
                parameters=[
                    {
                        # Controller mapping (pygame indices; confirm on
                        # the real pad).
                        "deadman_button": 2,  # X: hold to take over + drive
                        "done_button": 3,  # Y: press = goal reached
                        "axis_linear": 1,
                        "axis_angular": 0,
                        # Teleop publishes on its own topic; the cmd_vel
                        # bridge executes it with priority over /cmd_vel
                        # and exempt from any /nav_safety_hold gate (the
                        # human is supervising). Autonomous nav stays on
                        # /cmd_vel.
                        "cmd_vel_topic": "/cmd_vel_teleop",
                        # Teleop cap 0.10 m/s / 0.1667 rad/s (2026-07-10,
                        # 5/3 ratio). Kept above the autonomy's TEB limits
                        # (see teb_local_planner.yaml); deadman-gated,
                        # supervised.
                        "max_vel_x": 0.10,
                        "max_vel_theta": 0.1667,
                        "deadband": 0.12,
                    }
                ],
            ),
        ]
    )
