"""ROS2 port of zed_drift_test.launch.

Drift test: map-anchored live traces of wheel odom vs the fused EKF
(/odometry/fused_imu_wheel) vs live Cartographer, driven by Xbox teleop.
(ZED VIO traces retired 2026-07-15 with IMU-only ZED.)

Bring-up order:
  1. NUC: base_server up (tmux 'robot', launch_base)
  2. compute: sensors.launch.py (REQUIRED: it owns wheel_odom_publisher,
     gyro_bias_estimator, and ekf_fused_imu_wheel, the odom->base TF
     source and cartographer's odom topic since Jul 2026)
  3. compute: THIS file
  4. your terminal: ros2 run feeding_deployment drift_lock.py (TODO(ros2):
     scripts/drift_lock.py is out of this migration's 52-file scope, see
     ROS2_MIGRATION_NOTES.md -- not yet an installed console entry point)
     -> let localization settle, press ENTER to lock the anchor, then
        drive with the X-deadman.

MUTUALLY EXCLUSIVE with navigation.launch.py and shared_autonomy.launch.py:
they declare the same node names (cmd_vel_bridge_basicmicro,
shared_autonomy_teleop) and ROS2 launch will similarly conflict/duplicate
if both are started against the same node names. If cartographer
localization is ALREADY running (tmux pane), pass carto:=false.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("feeding_deployment")
    default_log_dir = os.path.join(
        share, "src", "feeding_deployment", "integration", "log", "system_logs"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "load_state_filename",
                default_value=os.path.join(share, "maps", "aimee-7-3.pbstream"),
            ),
            DeclareLaunchArgument("carto", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument("log_dir", default_value=default_log_dir),
            # Cartographer localization on the known map (map->odom).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "cartographer_localization.launch.py")
                ),
                launch_arguments={
                    "load_state_filename": LaunchConfiguration("load_state_filename")
                }.items(),
                condition=IfCondition(LaunchConfiguration("carto")),
            ),
            # Teleop: Xbox controller -> Twist on /cmd_vel_teleop
            # (deadman-gated). Verbatim from shared_autonomy.launch.py;
            # max_vel_x/max_vel_theta are REQUIRED (the node has no code
            # defaults).
            Node(
                package="feeding_deployment",
                executable="shared_autonomy_teleop.py",
                name="shared_autonomy_teleop",
                output="screen",
                parameters=[
                    {
                        "deadman_button": 2,  # X: hold to drive
                        "done_button": 3,  # Y (unused here)
                        "axis_linear": 1,
                        "axis_angular": 0,
                        "cmd_vel_topic": "/cmd_vel_teleop",
                        "max_vel_x": 0.10,
                        "max_vel_theta": 0.1667,
                        "deadband": 0.12,
                    }
                ],
            ),
            # cmd_vel bridge: /cmd_vel_teleop -> set_speeds RPC to the NUC.
            # Verbatim params from navigation.launch.py. Teleop is executed
            # with priority and without the ZED-divergence hold gate.
            Node(
                package="feeding_deployment",
                executable="cmd_vel_bridge_basicmicro.py",
                name="cmd_vel_bridge_basicmicro",
                output="screen",
                parameters=[
                    {
                        "linear_scale": 4874.0,
                        "angular_scale": 2600.0,
                        "max_speed_units": 2500,
                        "min_move_units": 100,
                    }
                ],
            ),
            # The tracer (+ optional rviz/rosbag) lives in
            # drift_traces.launch.py so it can also run alongside the full
            # nav stack; /wheel_odom comes from sensors.launch.py. This
            # test passes its own rviz/record args through.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "drift_traces.launch.py")
                ),
                launch_arguments={
                    "rviz": LaunchConfiguration("rviz"),
                    "record": LaunchConfiguration("record"),
                    "log_dir": LaunchConfiguration("log_dir"),
                }.items(),
            ),
            # zed_health_monitor DELETED [2026-07-15]: it watched ZED VIO
            # odom/status, which no longer exist under IMU-only ZED
            # (unplumbed from navigation.launch.py 2026-07-13; script
            # removed from the tree with the VIO tooling sweep).
        ]
    )
