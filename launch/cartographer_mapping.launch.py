"""ROS2 port of cartographer_mapping.launch.

cartographer_ros is confirmed available on this box for Jazzy
(apt-cache search -> ros-jazzy-cartographer-ros); its ROS2 port keeps the
same node executable names (cartographer_node,
cartographer_occupancy_grid_node) and the same Lua-config-file CLI-argument
style (-configuration_directory/-configuration_basename) rather than
ROS-parameter based config, so this port is a much more direct translation
than navigation.launch.py's move_base (which has no ROS2 equivalent at
all -- see that file). The .lua config files themselves
(config/vention_2lidar_2d.lua etc.) were NOT modified -- cartographer's Lua
configuration format is unchanged between the ROS1 and ROS2 releases.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_dir",
                default_value="/home/isacc/deployment_ws/src/feeding-deployment/config",
            ),
            DeclareLaunchArgument("config_basename", default_value="vention_2lidar_2d.lua"),
            Node(
                package="cartographer_ros",
                executable="cartographer_node",
                name="cartographer_node",
                output="screen",
                arguments=[
                    "-configuration_directory", LaunchConfiguration("config_dir"),
                    "-configuration_basename", LaunchConfiguration("config_basename"),
                ],
                remappings=[
                    # Fused wheel+IMU odometry (ekf_fused_imu_wheel in
                    # sensors.launch.py). Must match the odom->base TF
                    # owner; mixing sources corrupts the extrapolator. Was
                    # raw ZED VIO until Jul 2026.
                    ("scan_2", "/lidar_l/scan"),
                    ("scan_1", "/lidar_r/scan"),
                    ("odom", "/odometry/fused_imu_wheel"),
                ],
            ),
            Node(
                package="cartographer_ros",
                executable="cartographer_occupancy_grid_node",
                name="occupancy_grid_node",
                output="screen",
                parameters=[{"resolution": 0.05, "publish_period_sec": 1.0}],
            ),
        ]
    )
