"""ROS 2 perception bring-up for the autonomous microwave button press.

Starts the two perception processes the press driver consumes:

  * ``feeding_deployment.button_press.detector_node``   -> /button_detector/*
  * ``feeding_deployment.button_press.press_detector --publish`` -> /press_detector/*

It does NOT start the camera/tf chain (use ``microwave_bringup.launch.py``) or the arm
servers, and it never starts the press driver itself -- every motion run is a deliberate,
separate invocation with a human at the e-stop (docs/button_press_runbook.md).

The press detector takes its rest baseline in the first ~2 s, so the arm must be PARKED
and untouched when this launches. It opens its own read-only Kortex session: start it
once, AFTER arm_server + joint_state_bridge are up, and never restart it while the arm
stack runs (new sessions have evicted arm_server's; re-baseline with SIGUSR1 instead).

    ros2 launch launch/ros2/button_press_bringup.launch.py \\
        reference_dir:=$HOME/wrist_ref_red target_button:=timer_clock

Run ``ros2 launch`` with system python3 (it needs ``lark``); the nodes run with the venv.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "reference_dir", default_value=os.path.expanduser("~/wrist_ref_red"),
            description="Directory holding reference.json for THIS microwave."),
        DeclareLaunchArgument(
            "target_button", default_value="timer_clock",
            description="Named button to report (start_30s / timer_clock / '' = default)."),
        DeclareLaunchArgument(
            "force_threshold", default_value="8",
            description="press_detector --threshold (N); 8 is what the 2026-09-21 runs used."),
        DeclareLaunchArgument(
            "force_log", default_value=os.path.expanduser("~/press_logs/force.jsonl"),
            description="press_detector --log (append); replayable with --replay."),
        DeclareLaunchArgument(
            "launch_press_detector", default_value="true",
            description="Set false if a press detector is already running (do not open a "
                        "second Kortex session)."),
        DeclareLaunchArgument(
            "arm_ip", default_value="192.168.1.10", description="Kinova arm IP."),
        DeclareLaunchArgument(
            "python_executable",
            default_value=os.path.expanduser("~/feeding-deployment/.venv/bin/python"),
            description="Interpreter with the project (and rclpy) importable."),
        DeclareLaunchArgument(
            "repo_src", default_value=os.path.expanduser("~/feeding-deployment/src"),
            description="Path prepended to PYTHONPATH so feeding_deployment imports."),
    ]
    py = LaunchConfiguration("python_executable")
    # Prepend, never replace: rclpy and the message packages already live on PYTHONPATH.
    env = {"PYTHONPATH": [LaunchConfiguration("repo_src"), ":", os.environ.get("PYTHONPATH", "")]}

    detector = ExecuteProcess(
        cmd=[py, "-u", "-m", "feeding_deployment.button_press.detector_node", "--ros-args",
             "-p", ["reference_dir:=", LaunchConfiguration("reference_dir")],
             "-p", ["target_button:=", LaunchConfiguration("target_button")]],
        name="button_detector", output="screen", additional_env=env)

    press_detector = ExecuteProcess(
        cmd=[py, "-u", "-m", "feeding_deployment.button_press.press_detector",
             "--publish", "--print-hz", "0",
             "--ip", LaunchConfiguration("arm_ip"),
             "--threshold", LaunchConfiguration("force_threshold"),
             "--log", LaunchConfiguration("force_log")],
        name="press_detector", output="screen", additional_env=env,
        condition=IfCondition(LaunchConfiguration("launch_press_detector")))

    return LaunchDescription(args + [detector, press_detector])
