"""ROS2 port of sim.launch.

Conversion notes: same static_transform_publisher / xacro / group-namespace
patterns as arm.launch.py -- see that file's header comment for the
detailed reasoning, not repeated here.

  - <node ... required="true"/> (RViz) -> ROS2 launch's `Node` has an
    `on_exit` hook rather than a `required` flag; the closest built-in
    equivalent is emitting a shutdown event when this node exits. Used
    `launch.actions.Shutdown` via `on_exit=` to mirror "required" (roslaunch
    tears down the whole launch if a required node dies).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, Shutdown
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

    base_path_arg = DeclareLaunchArgument(
        "base_path", default_value=get_package_share_directory("feeding_deployment")
    )

    return LaunchDescription(
        [
            base_path_arg,
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
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="rob_st_pub",
                parameters=[robot_description, {"ignore_timestamp": True}],
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
            # TODO(ros2): wrist_driver_ros node -- was already commented out
            # in the ROS1 source ("Access does not work through roslaunch
            # even after an immense number of tries"), preserved disabled.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_link_broadcaster",
                arguments=[
                    "--x", "-0.04611404",
                    "--y", "0.0837761",
                    "--z", "0.0729979",
                    "--qx", "-0.70180948",
                    "--qy", "-0.00506307",
                    "--qz", "-0.000348",
                    "--qw", "0.71234662",
                    "--frame-id", "end_effector_link",
                    "--child-frame-id", "camera_link",
                ],
            ),
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
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=["-d", [LaunchConfiguration("base_path"), "/config/real.rviz"]],
                on_exit=Shutdown(),
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
            ),
        ]
    )
