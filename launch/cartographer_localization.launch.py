"""ROS2 port of cartographer_localization.launch.

Localization against a prebuilt Cartographer state (.pbstream). See
cartographer_mapping.launch.py for the general cartographer_ros conversion
notes (confirmed available for Jazzy, same node/executable names and
Lua-config CLI style).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir_arg = DeclareLaunchArgument(
        "config_dir",
        default_value=os.path.join(
            get_package_share_directory("feeding_deployment"), "config"
        ),
    )

    return LaunchDescription(
        [
            config_dir_arg,
            DeclareLaunchArgument(
                "config_basename", default_value="vention_2lidar_localization.lua"
            ),
            DeclareLaunchArgument(
                "load_state_filename", default_value="/tmp/vention_map.pbstream"
            ),
            DeclareLaunchArgument("load_frozen_state", default_value="true"),
            # Coverage floor for the scan gate (occupied 0.2 m cells).
            # Healthy scans measure 36-114 across the jul14_2 trigger bags;
            # feeding-occlusion sits around 10-20 per Cartographer's node
            # point counts. Retune from the scan_gate stats lines after the
            # first gated table session.
            DeclareLaunchArgument("gate_min_cells", default_value="25"),
            # Scan gate: drops BOTH lidars' scans whenever either is too
            # occluded to localize on (person/arm at the table gutted
            # scans -> Cartographer matched them 4-7 m off at 90% score and
            # yanked map->odom 2-7 m every optimization; both table stays,
            # session jul14_2). Starved, Cartographer freezes map->odom on
            # the last good pose and the robot rides wheel+IMU odom until
            # the view clears. Costmaps/diagnostics keep the raw topics;
            # only Cartographer subscribes to the gated ones.
            #
            # TODO(ros2): scripts/scan_gate.py is a scripts/-only file, out
            # of this migration's confirmed 52-file scope (see
            # ROS2_MIGRATION_NOTES.md) -- this Node action's structure is
            # preserved but the script itself has NOT been ported to
            # rclpy.
            Node(
                package="feeding_deployment",
                executable="scan_gate.py",
                name="scan_gate",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[{"min_cells": LaunchConfiguration("gate_min_cells")}],
            ),
            # Cartographer node in localization mode.
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                arguments=[
                    "-configuration_directory", LaunchConfiguration("config_dir"),
                    "-configuration_basename", LaunchConfiguration("config_basename"),
                    "-load_state_filename", LaunchConfiguration("load_state_filename"),
                    "-load_frozen_state", LaunchConfiguration("load_frozen_state"),
                ],
                remappings=[
                    ("scan_2", "/lidar_l/scan_gated"),
                    ("scan_1", "/lidar_r/scan_gated"),
                    # Fused wheel+IMU odometry (ekf_fused_imu_wheel in
                    # sensors.launch.py, which also owns the
                    # odom->vention_base_link TF). child_frame_id equals
                    # the tracking_frame, so no TF hop. Was the sanitized
                    # ZED VIO (/zed_mini/zed_node/odom_sanitized) until Jul
                    # 2026; VIO jumps kept dragging localization (e.g.
                    # 31.5 m excursion, session jul10-6).
                    ("odom", "/odometry/fused_imu_wheel"),
                ],
            ),
            # Publish occupancy grid as /map.
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="occupancy_grid_node",
                output="screen",
                parameters=[{"resolution": 0.05, "publish_period_sec": 1.0}],
            ),
        ]
    )
