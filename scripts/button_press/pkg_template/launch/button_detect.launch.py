"""Launch the button detector against the wrist camera.

reference_dir has no default on purpose. There is one reference per appliance;
silently falling back to some other microwave's reference would produce
confident, wrong answers, which is the one failure mode this detector exists to
avoid. Pass it explicitly.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("reference_dir",
                              description="directory holding reference.json for THIS microwave"),
        DeclareLaunchArgument("image_topic", default_value="/camera/color/image_raw"),
        DeclareLaunchArgument("depth_topic",
                              default_value="/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/color/camera_info"),
        DeclareLaunchArgument("rate_limit_hz", default_value="5.0"),
    ]
    node = Node(
        package="rammp_button_detect",
        executable="button_detector_node",
        name="button_detector",
        output="screen",
        parameters=[{
            "reference_dir": LaunchConfiguration("reference_dir"),
            "image_topic": LaunchConfiguration("image_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "rate_limit_hz": LaunchConfiguration("rate_limit_hz"),
        }],
    )
    return LaunchDescription(args + [node])
