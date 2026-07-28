"""ROS2 port of zed.launch.

LEGACY standalone ZED bringup: still sets pos_tracking/publish_tf=true
(ZED-owned odom). NEVER run alongside sensors.launch.py, where the fused
EKF owns odom->base since Jul 2026.

TODO(ros2): see base_sensors.launch.py's conversion notes for the
zed_wrapper package-availability caveat (not verified installed on this
box, ROS2 zed-ros2-wrapper has a different launch API than ROS1's
zedm.launch). Additionally: ROS1's pattern of setting bare
`/$(arg camera_name)/zed_node/pos_tracking/publish_tf` etc. as top-level
<param> tags AFTER the <include> works because roslaunch pre-seeds the
parameter server before any node starts, regardless of declaration order.
ROS2 has no global parameter server -- there is no way to "reach into"
an included node's parameters from outside; these have to be passed as
`parameters=[{...}]` (or a params-override YAML) on the zed node action
itself, which means editing whatever the real zed_camera.launch.py
include's own launch-argument surface exposes. Left as a documented dict
below rather than guessed at, since the ROS2 zed wrapper's parameter
names/paths were not verified against a real install this session.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

# TF ownership (original ROS1 intent, preserved for whoever finishes the
# ROS2 zed_wrapper integration): ZED owns odom -> zed_mini_base_link, URDF
# owns vention_base_link <-> zed_mini_base_link, Cartographer owns
# map -> odom.
ORIGINAL_ROS1_ZED_PARAMS = {
    "pos_tracking.publish_tf": True,
    "pos_tracking.publish_map_tf": False,
    "pos_tracking.odometry_frame": "odom",  # LaunchConfiguration("odom_frame")
    "pos_tracking.map_frame": "map",  # LaunchConfiguration("map_frame")
    "pos_tracking.base_frame": "zed_mini_base_link",  # LaunchConfiguration("camera_base_frame")
}


def generate_launch_description():
    actions = [
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("camera_name", default_value="zed_mini"),
        DeclareLaunchArgument("camera_base_frame", default_value="zed_mini_base_link"),
    ]

    try:
        zed_share = get_package_share_directory("zed_wrapper")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([zed_share, "launch", "zed_camera.launch.py"])
                ),
                launch_arguments={
                    "camera_name": LaunchConfiguration("camera_name"),
                    "camera_model": "zedm",
                }.items(),
            )
        )
        actions.append(
            LogInfo(
                msg=(
                    "TODO(ros2): ORIGINAL_ROS1_ZED_PARAMS (publish_tf,"
                    " odometry_frame, map_frame, base_frame) were NOT"
                    " re-applied -- ROS2 has no global param server to set"
                    " them post-include. Pass them as launch_arguments or a"
                    " params-override YAML to the real zed_camera.launch.py"
                    " once its ROS2 argument surface is confirmed."
                )
            )
        )
    except Exception:  # pylint: disable=broad-except
        actions.append(
            LogInfo(
                msg=(
                    "TODO(ros2): zed_wrapper package share directory not"
                    " found -- ZED include was skipped. Verify a ROS2"
                    " zed-ros2-wrapper install is present."
                )
            )
        )

    return LaunchDescription(actions)
