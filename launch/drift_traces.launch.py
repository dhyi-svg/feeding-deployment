"""ROS2 port of drift_traces.launch.

Drift-trace OBSERVER only: the map-anchored trace comparator (+ optional
dedicated RViz window and rosbag). Safe to run ALONGSIDE the full
production stack: no node names or topics overlap, these nodes only
observe. Lock/re-lock the anchor anytime with drift_lock.py (scripts/, out
of this migration's scope -- see ROS2_MIGRATION_NOTES.md).

Conversion notes:
  - if="$(arg ...)" -> launch.conditions.IfCondition(LaunchConfiguration(...)).
  - <node pkg="rosbag" type="record" .../> -> ROS2 has no `rosbag` package
    or in-graph "record node" the same way; `ros2 bag record` is a CLI
    verb. Ported to launch.actions.ExecuteProcess running that CLI, which
    is the standard ROS2 launch idiom for this (see e.g. nav2_bringup's
    own launch files for the same pattern).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_log_dir = os.path.join(
        get_package_share_directory("feeding_deployment"),
        "src",
        "feeding_deployment",
        "integration",
        "log",
        "system_logs",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("record", default_value="false"),
            DeclareLaunchArgument("log_dir", default_value=default_log_dir),
            # The tracer: /drift_test/{carto,wheel,fused_imu_wheel}_path +
            # anchor. Draws nothing until /drift_test/lock (see
            # drift_lock.py).
            Node(
                package="feeding_deployment",
                executable="drift_trace_compare.py",
                name="drift_trace_compare",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="drift_test_rviz",
                arguments=[
                    "-d",
                    os.path.join(
                        get_package_share_directory("feeding_deployment"),
                        "rviz",
                        "drift_test.rviz",
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            # Optional bag for post-analysis (no images, no Path topics;
            # traces are reconstructable from the odoms + tf + anchor).
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    [LaunchConfiguration("log_dir"), "/drift_test"],
                    "/tf",
                    "/tf_static",
                    "/cmd_vel_teleop",
                    "/odometry/fused_imu_wheel",
                    "/wheel_odom",
                    "/wheel_odom/counts",
                    "/wheel_odom/side_disagreement",
                    "/drift_test/anchor",
                    "/lidar_r/scan",
                    "/lidar_l/scan",
                ],
                name="drift_test_record",
                condition=IfCondition(LaunchConfiguration("record")),
            ),
        ]
    )
