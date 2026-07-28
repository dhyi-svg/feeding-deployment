"""ROS2 port of arm.launch.

Conversion notes:
  - NOTE (found during migration, not a ROS2-specific issue): the original
    XML's camera_link_broadcaster args string was
    "-0.04611404 0.0837761 0.11 0.7071068, 0, 0, 0.7071068 end_effector_link
    camera_link" -- it contains stray commas glued onto the "0.7071068,"
    and "0," tokens. roslaunch's whitespace-only tokenizer would have
    passed "0.7071068," (with a trailing comma) as one argv token straight
    to argparse/C++ arg parsing; whether the ROS1 tf2_ros
    static_transform_publisher's parser silently stripped/ignored that or
    actually errored at runtime was NOT verified here (nothing in this
    migration touches real hardware). Converting to ROS2's discrete
    `--qx/--qy/--qz/--qw` flags below naturally drops the commas since
    each value is now its own separate list element -- flagging this as a
    pre-existing-repo finding worth a human's eyes, not something silently
    "fixed" as part of the migration.
  - Old `pkg="tf"` static_transform_publisher nodes (tool base frames,
    st_map2world) -> ROS2 has no `tf` package at all (fully replaced by
    tf2_ros); ported to `tf2_ros`'s static_transform_publisher with named
    flags. The original quaternion-form args ("x y z qx qy qz qw parent
    child period_ms") drop the trailing period_ms -- ROS2's static
    publisher has no periodic-republish concept, it publishes once on
    /tf_static with a transient-local QoS instead.
  - RViz node: was already commented out in the ROS1 source (rviz2 binary
    kept commented here too, left as a TODO rather than silently deleted).
  - realsense2_camera / netft_rdt_driver: see arm_sensors.launch.py's
    conversion notes -- same caveats apply, not repeated here.
  - <group ns="sim"> -> launch_ros doesn't have a direct multi-node
    "apply this param + these nodes under a namespace" one-liner the way
    XML <group> does; used GroupAction + PushRosNamespace, which is the
    idiomatic ROS2 equivalent.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _xacro_robot_description():
    return ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " --inorder ",
                PathJoinSubstitution(
                    [
                        get_package_share_directory("kortex_description"),
                        "robots",
                        "gen3_robotiq_2f_85.xacro",
                    ]
                ),
                " dof:=7 vision:=false tool:=all",
            ]
        ),
        value_type=str,
    )


def generate_launch_description():
    robot_description = {"robot_description": _xacro_robot_description()}

    actions = [
        DeclareLaunchArgument(
            "base_path",
            default_value=get_package_share_directory("feeding_deployment"),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="rob_st_pub",
            parameters=[robot_description, {"ignore_timestamp": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_link_broadcaster",
            arguments=[
                "--x", "-0.04611404",
                "--y", "0.0837761",
                "--z", "0.11",
                "--qx", "0.7071068",
                "--qy", "0",
                "--qz", "0",
                "--qw", "0.7071068",
                "--frame-id", "end_effector_link",
                "--child-frame-id", "camera_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_feeding_tool",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "finger_tip", "--child-frame-id", "forkbase",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_drinking_tool",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "finger_tip", "--child-frame-id", "drinkbase",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_wiping_tool",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "finger_tip", "--child-frame-id", "wipebase",
            ],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                {
                    "use_gui": False,
                    "source_list": ["robot_joint_states", "wrist_joint_states"],
                    "rate": 100,
                }
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="st_map2world",
            output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "map", "--child-frame-id", "world",
            ],
        ),
        # Rosbridge server to communicate with web app.
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
        ),
        # Simulated Robot, namespaced under "sim" (was <group ns="sim">).
        GroupAction(
            [
                PushRosNamespace("sim"),
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    name="joint_state_publisher",
                    parameters=[
                        {
                            "use_gui": False,
                            "source_list": ["robot_joint_states", "wrist_joint_states"],
                            "rate": 100,
                        }
                    ],
                ),
                Node(
                    package="robot_state_publisher",
                    executable="robot_state_publisher",
                    name="rob_st_pub",
                    parameters=[
                        robot_description,
                        {"ignore_timestamp": True, "tf_prefix": "sim"},
                    ],
                ),
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="st_map2world",
                    output="screen",
                    arguments=[
                        "--x", "0", "--y", "0", "--z", "0",
                        "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                        "--frame-id", "map", "--child-frame-id", "sim/world",
                    ],
                ),
            ]
        ),
        LogInfo(
            msg=(
                "TODO(ros2): realsense2_camera and netft_rdt_driver includes"
                " were intentionally omitted here -- see"
                " arm_sensors.launch.py for the same bringup with"
                " conversion notes; include it alongside this file if both"
                " are needed together."
            )
        ),
        # RViz was already commented out in the ROS1 source
        # ("-d $(arg base_path)/config/real.rviz"); left disabled here too.
    ]
    return LaunchDescription(actions)
