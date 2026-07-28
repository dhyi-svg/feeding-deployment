"""ROS2 port of rplidar_a1.launch.

Conversion notes:
  - rplidar_ros keeps the SAME ROS package name in ROS2 (confirmed via
    `apt-cache search rplidar` on this box -> ros-jazzy-rplidar-ros). The
    ROS1 node type "rplidarNode" is a plain executable name in ROS2 too;
    left as-is, but NOT independently confirmed against the ROS2 release's
    actual executable name (the ROS2 rplidar_ros driver historically also
    ships as "rplidar_node" -- check `ros2 pkg executables rplidar_ros`
    before relying on this; flagged in ROS2_MIGRATION_NOTES.md).
  - <group ns="..."> -> launch_ros.actions.Node(namespace=...); ROS2 has no
    exact equivalent of wrapping multiple unrelated actions in one XML
    <group>, but here each group has exactly one node so `namespace=` on
    the Node itself is the direct, simpler equivalent (no PushRosNamespace
    needed).
  - <node pkg="tf2_ros" type="static_transform_publisher" args="x y z roll
    pitch yaw parent child"/> (old ROS1 6-number RPY form) -> ROS2's
    tf2_ros static_transform_publisher takes named flags. Translated
    explicitly below (--roll/--pitch/--yaw, not quaternion, to match the
    original RPY args one-to-one without doing our own quaternion math).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port1",
                default_value="/dev/serial/by-path/pci-0000:00:14.0-usb-0:13.4.4:1.0-port0",
            ),
            DeclareLaunchArgument(
                "port2",
                default_value="/dev/serial/by-path/pci-0000:00:14.0-usb-0:13.4.3:1.0-port0",
            ),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("base_frame", default_value="vention_base_link"),
            # Lidar Right pose w.r.t. vention_base_link
            DeclareLaunchArgument("lidar_r_x", default_value="0.2575"),
            DeclareLaunchArgument("lidar_r_y", default_value="-0.135"),  # -0.3377
            DeclareLaunchArgument("lidar_r_z", default_value="0.415"),  # 0.09
            DeclareLaunchArgument("lidar_r_rr", default_value="3.14159"),  # roll
            DeclareLaunchArgument("lidar_r_rp", default_value="0.0"),  # pitch
            DeclareLaunchArgument("lidar_r_ry", default_value="0.0"),  # yaw
            # Lidar Left pose w.r.t. vention_base_link
            DeclareLaunchArgument("lidar_l_x", default_value="0.2575"),
            DeclareLaunchArgument("lidar_l_y", default_value="0.135"),  # 0.3377
            DeclareLaunchArgument("lidar_l_z", default_value="0.415"),  # 0.09
            DeclareLaunchArgument("lidar_l_rr", default_value="3.14159"),  # roll
            DeclareLaunchArgument("lidar_l_rp", default_value="0.0"),  # pitch
            DeclareLaunchArgument("lidar_l_ry", default_value="0.0"),  # yaw
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
        ]
    )
