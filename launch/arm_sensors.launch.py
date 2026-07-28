"""ROS2 port of arm_sensors.launch.

Conversion notes:
  - realsense2_camera's ROS1 rs_camera.launch include -> ROS2's
    realsense2_camera ships rs_launch.py with the same launch args
    (align_depth, enable_sync, filters, etc.), confirmed as the standard
    ROS2 realsense2_camera launch entry point name; NOT independently
    verified installed on this box (no camera-facing package check was
    run this session -- flagged in ROS2_MIGRATION_NOTES.md). Do not treat
    as tested.
  - netft_rdt_driver's ROS1 ft_sensor.launch include: TODO(ros2) -- per
    CLAUDE.md this driver has "no public distribution at all" even for
    ROS1; there is no known ROS2 port to include here. Left as an explicit
    TODO block (not a guessed IncludeLaunchDescription) since guessing a
    package name here would be worse than flagging it.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    actions = []

    try:
        realsense_share = get_package_share_directory("realsense2_camera")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([realsense_share, "launch", "rs_launch.py"])
                ),
                launch_arguments={
                    "align_depth.enable": "true",
                    "enable_sync": "true",
                    "pointcloud.enable": "true",
                }.items(),
            )
        )
    except Exception:  # pylint: disable=broad-except
        actions.append(
            LogInfo(
                msg=(
                    "TODO(ros2): realsense2_camera package share directory not"
                    " found -- rs_launch.py include was skipped. Verify"
                    " ros-jazzy-realsense2-camera is installed."
                )
            )
        )

    # TODO(ros2): netft_rdt_driver has no known ROS2 (or even public ROS1)
    # distribution -- see CLAUDE.md. Nothing to include here yet; the FT
    # sensor bringup needs a real driver source before this launch file can
    # be considered complete.
    actions.append(
        LogInfo(
            msg=(
                "TODO(ros2): netft_rdt_driver has no known ROS2 port -- FT"
                " sensor is not brought up by this launch file. See"
                " CLAUDE.md / ROS2_MIGRATION_NOTES.md."
            )
        )
    )

    return LaunchDescription(actions)
