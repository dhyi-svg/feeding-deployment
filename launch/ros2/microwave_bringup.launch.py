"""ROS 2 bring-up for the microwave task on the single-machine (Jetson) rig.

The ROS 1 stack got its TF tree from the lab's Kinova ROS driver. This rig talks
to the arm over the repo's own RPC (``arm_server.py``) and has no ROS arm driver,
so this launch assembles the equivalent tree from the pieces that do exist:

    base_link --(robot_state_publisher + joint_state_bridge)--> end_effector_link
    end_effector_link --(easy_handeye2 calibration, static)--> camera_color_optical_frame
    base_link --(identity, static)--> arm_base_link      # the name repo code uses

With that up, ``PerceptionInterface``'s tf2 lookups resolve and
``OpenDoorHLA.open_microwave()`` runs unmodified.

Prerequisites (not started here -- they own the arm and must be brought up and
checked by hand first, in this order):

  1. ``arm_server.py``            connects to the arm, clears faults, holds position
  2. ``scripts/stub_base_server.py``  no-op base so bulldog's handshake passes
  3. ``scripts/bulldog_bypass.py``    flips bulldog_ready and heartbeats

Then::

    ros2 launch launch/ros2/microwave_bringup.launch.py

Check the result before moving the arm::

    ros2 run tf2_ros tf2_echo arm_base_link camera_color_optical_frame

Note ``align_depth.enable:=true``: the whole handle pipeline assumes depth
pixels line up with colour pixels.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    calib_file_arg = DeclareLaunchArgument(
        "calib_file",
        default_value=os.path.expanduser(
            "~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib"
        ),
        description="easy_handeye2 hand-eye calibration to publish as static TF.",
    )
    launch_camera_arg = DeclareLaunchArgument(
        "launch_camera",
        default_value="true",
        description="Start realsense2_camera. Set false if the camera is already up.",
    )
    arm_rpc_host_arg = DeclareLaunchArgument(
        "arm_rpc_host",
        default_value="127.0.0.1",
        description="Host running arm_server.py (ARM_RPC_HOST).",
    )
    # feeding_deployment is a pip/catkin package, not an ament one, so its ROS 2
    # helpers are run as plain modules out of the project venv rather than as
    # installed ROS 2 executables.
    python_arg = DeclareLaunchArgument(
        "python_executable",
        default_value=os.path.expanduser("~/feeding-deployment/.venv/bin/python"),
        description="Interpreter with the project (and rclpy) importable.",
    )
    repo_src_arg = DeclareLaunchArgument(
        "repo_src",
        default_value=os.path.expanduser("~/feeding-deployment/src"),
        description="Path added to PYTHONPATH so feeding_deployment imports.",
    )

    # The Kinova description, matching this rig: 7-DOF Gen3 + Robotiq 2F-85.
    # No prefix, so the frames come out as base_link / end_effector_link -- the
    # exact names the saved hand-eye calibration refers to.
    robot_description = Command(
        [
            "xacro ",
            PathJoinSubstitution(
                [FindPackageShare("kortex_description"), "robots", "gen3.xacro"]
            ),
            " dof:=7",
            " gripper:=robotiq_2f_85",
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # PYTHONPATH must be prepended, not replaced: rclpy and the ROS 2 message
    # packages already live on it.
    pythonpath = [LaunchConfiguration("repo_src"), ":", os.environ.get("PYTHONPATH", "")]

    # Feeds robot_state_publisher from the repo's arm RPC. Read-only: it polls
    # get_state() and never commands the arm.
    joint_state_bridge = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m",
            "feeding_deployment.ros2.joint_state_bridge",
        ],
        name="joint_state_bridge",
        output="screen",
        additional_env={
            "ARM_RPC_HOST": LaunchConfiguration("arm_rpc_host"),
            "PYTHONPATH": pythonpath,
        },
    )

    calibration_tf = ExecuteProcess(
        cmd=[
            LaunchConfiguration("python_executable"),
            "-m",
            "feeding_deployment.ros2.calibration_tf",
            "--calib",
            LaunchConfiguration("calib_file"),
        ],
        name="hand_eye_calibration_tf",
        output="screen",
        additional_env={"PYTHONPATH": pythonpath},
    )

    color_profile_arg = DeclareLaunchArgument(
        "color_profile", default_value="640,480,15",
        description="RealSense colour profile W,H,FPS. Kept low: the default 1280x720 "
                    "stalls the colour stream on this rig (see below).")
    depth_profile_arg = DeclareLaunchArgument(
        "depth_profile", default_value="640,480,15",
        description="RealSense depth profile W,H,FPS.")

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        condition=IfCondition(LaunchConfiguration("launch_camera")),
        launch_arguments={
            # The handle pipeline reads depth at colour pixel coordinates.
            "align_depth.enable": "true",
            "camera_name": "camera",
            "camera_namespace": "",
            # Bandwidth, not preference. At the driver's default (1280x720 colour +
            # 848x480 depth) the COLOUR stream stalls on this rig within minutes
            # while depth keeps running -- so the RGB-D synchroniser never fires and
            # detection fails with "no synchronised frames" even though the camera
            # node is alive and depth is healthy (observed repeatedly 2026-07-29).
            # 640x480x15 on both streams cuts the USB load several-fold.
            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
            "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
        }.items(),
    )

    return LaunchDescription(
        [
            calib_file_arg,
            launch_camera_arg,
            arm_rpc_host_arg,
            python_arg,
            repo_src_arg,
            color_profile_arg,
            depth_profile_arg,
            robot_state_publisher,
            joint_state_bridge,
            calibration_tf,
            realsense,
        ]
    )
