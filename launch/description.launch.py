"""ROS2 port of description.launch.

LEGACY standalone bringup: vention.urdf still assumes ZED-owned odom
(inverted joint). NEVER run alongside sensors.launch.py (two-parent TF).

Conversion notes (see ROS2_MIGRATION_NOTES.md for the general pattern used
across every launch file in this directory):
  - <param name="robot_description" textfile="..."/> -> read the URDF file
    at launch-description-construction time and pass its contents as a
    string parameter to robot_state_publisher (ROS2's robot_state_publisher
    takes the URDF as the "robot_description" parameter value directly,
    same as ROS1, just no more <param textfile=.../> shorthand).
  - joint_state_publisher / robot_state_publisher package+executable names
    are unchanged between ROS1 and ROS2.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("feeding_deployment"), "urdf", "vention.urdf"
    )
    with open(urdf_path, "r", encoding="utf-8") as f:
        robot_description = f.read()

    return LaunchDescription(
        [
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
        ]
    )
