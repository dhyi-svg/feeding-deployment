"""ROS2 port of base_sensors.launch.

LEGACY standalone bringup (superseded by sensors.launch.py): still assumes
ZED-owned odom (vention.urdf inverted joint + publish_tf=true). NEVER run
alongside sensors.launch.py (two-parent TF tree).

Conversion notes:
  - zed_wrapper's ROS1 zedm.launch include: TODO(ros2) -- the ROS2 ZED
    wrapper is a different package (stereolabs/zed-ros2-wrapper,
    "zed_wrapper" still as the package name but a different launch file
    layout/API, typically zed_camera.launch.py with a `camera_model`
    launch arg) and was NOT verified installed/available on this box
    (unlike rviz2/rplidar_ros/rosbridge_suite, `apt-cache search zed` was
    not run against a confirmed-present source in this session). Left as
    an IncludeLaunchDescription pointing at the expected ROS2 path with a
    loud TODO rather than guessing at parameter names -- a human with a
    real ZED wrapper install needs to verify/replace this before use.
  - rviz -> rviz2 (confirmed available for Jazzy).
  - feeding_deployment's own zed_pose_to_odom_feedback.py is a
    scripts/-only file (not migrated in this pass -- see
    ROS2_MIGRATION_NOTES.md's explicit non-goals; scripts/ was out of the
    confirmed 52-file rospy migration scope). The Node action below still
    references it structurally (package executable) so the launch file's
    *shape* is preserved, but running it requires that script to be
    ROS2-ported and installed as a console entry point first.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("feeding_deployment"), "urdf", "vention.urdf"
    )
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("feeding_deployment"), "rviz", "vention.rviz"
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port1",
                default_value="/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.4.4:1.0-port0",
            ),
            DeclareLaunchArgument(
                "port2",
                default_value="/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.4.3:1.0-port0",
            ),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("base_frame", default_value="vention_base_link"),
            DeclareLaunchArgument("lidar_r_x", default_value="0.2575"),
            DeclareLaunchArgument("lidar_r_y", default_value="-0.135"),
            DeclareLaunchArgument("lidar_r_z", default_value="0.415"),
            DeclareLaunchArgument("lidar_r_rr", default_value="3.14159"),
            DeclareLaunchArgument("lidar_r_rp", default_value="0.0"),
            DeclareLaunchArgument("lidar_r_ry", default_value="0.0"),
            DeclareLaunchArgument("lidar_l_x", default_value="0.2575"),
            DeclareLaunchArgument("lidar_l_y", default_value="0.135"),
            DeclareLaunchArgument("lidar_l_z", default_value="0.415"),
            DeclareLaunchArgument("lidar_l_rr", default_value="3.14159"),
            DeclareLaunchArgument("lidar_l_rp", default_value="0.0"),
            DeclareLaunchArgument("lidar_l_ry", default_value="0.0"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("camera_name", default_value="zed_mini"),
            DeclareLaunchArgument("camera_base_frame", default_value="zed_mini_base_link"),
            rviz_config_arg,
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_lidar_r",
                arguments=[
                    "--x", LaunchConfiguration("lidar_r_x"),
                    "--y", LaunchConfiguration("lidar_r_y"),
                    "--z", LaunchConfiguration("lidar_r_z"),
                    "--roll", LaunchConfiguration("lidar_r_rr"),
                    "--pitch", LaunchConfiguration("lidar_r_rp"),
                    "--yaw", LaunchConfiguration("lidar_r_ry"),
                    "--frame-id", LaunchConfiguration("base_frame"),
                    "--child-frame-id", "lidar_r",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_to_lidar_l",
                arguments=[
                    "--x", LaunchConfiguration("lidar_l_x"),
                    "--y", LaunchConfiguration("lidar_l_y"),
                    "--z", LaunchConfiguration("lidar_l_z"),
                    "--roll", LaunchConfiguration("lidar_l_rr"),
                    "--pitch", LaunchConfiguration("lidar_l_rp"),
                    "--yaw", LaunchConfiguration("lidar_l_ry"),
                    "--frame-id", LaunchConfiguration("base_frame"),
                    "--child-frame-id", "lidar_l",
                ],
            ),
            Node(
                package="rplidar_ros",
                executable="rplidarNode",
                name="rplidarNode",
                namespace="lidar_r",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("port1"),
                        "serial_baudrate": LaunchConfiguration("baud"),
                        "frame_id": "lidar_r",
                        "angle_compensate": True,
                        "inverted": False,
                    }
                ],
            ),
            Node(
                package="rplidar_ros",
                executable="rplidarNode",
                name="rplidarNode",
                namespace="lidar_l",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("port2"),
                        "serial_baudrate": LaunchConfiguration("baud"),
                        "frame_id": "lidar_l",
                        "angle_compensate": True,
                        "inverted": False,
                    }
                ],
            ),
            # TODO(ros2): verify the zed-ros2-wrapper package is actually
            # installed on the target box and that this include path/args
            # match its real launch API before relying on this.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            get_package_share_directory("zed_wrapper")
                            if _zed_wrapper_available()
                            else "TODO_ROS2_ZED_WRAPPER_NOT_VERIFIED",
                            "launch",
                            "zed_camera.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "camera_model": "zedm",
                }.items(),
            ),
            Node(
                package="feeding_deployment",
                executable="zed_pose_to_odom_feedback.py",
                name="zed_pose_to_odom_feedback",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": [LaunchConfiguration("camera_name"), "/zed_node/odom"],
                        "output_odom_topic": "/move_base/odom_feedback",
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )


def _zed_wrapper_available() -> bool:
    try:
        get_package_share_directory("zed_wrapper")
        return True
    except Exception:  # pylint: disable=broad-except
        return False
