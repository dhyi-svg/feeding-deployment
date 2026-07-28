"""ROS2 port of sensors.launch.

The full sensor/localization bring-up: combined-URDF joint/robot state
publishing, lidar TFs + drivers, ZED passive witness (IMU-only, see
comments below), the authoritative fused wheel+IMU EKF, arm-specific TFs,
the arm RealSense camera, and the rosbridge server for the webapp.

Conversion notes specific to this file (see individual Node comments below
for the ROS1 tribal-knowledge context, preserved verbatim -- do not lose
this, it documents real hardware incidents):
  - robot_localization: `ekf_localization_node` (ROS1 executable name) was
    RENAMED to `ekf_node` in the package's ROS2 port. This is documented
    in robot_localization's own ROS2 README; NOT independently confirmed
    against a real installed binary on this box (the deb package itself
    isn't installed here, only found via `apt-cache search`) -- verify
    with `ros2 pkg executables robot_localization` before relying on this.
  - The ROS1 file set a handful of `/$(arg camera_name)/zed_node/...`
    params as bare top-level <param> tags AFTER the zed_wrapper <include>;
    this works in ROS1 because roslaunch pre-seeds the whole parameter
    server before any node starts. ROS2 has no global parameter server --
    seev zed.launch.py's docstring for the same issue; all of those
    params are preserved in ORIGINAL_ROS1_ZED_PARAMS below rather than
    silently dropped, but are NOT wired into the (also TODO/unverified)
    zed_wrapper include here.
  - zed_svo_recorder.py / wheel_odom_publisher.py / gyro_bias_estimator.py:
    wheel_odom_publisher.py IS in the confirmed 52-file rospy migration
    scope (control/base_controller/wheel_odom_publisher.py); the other two
    are scripts/-only and out of scope (see ROS2_MIGRATION_NOTES.md).
  - realsense2_camera / netft_rdt_driver: see arm_sensors.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Original ROS1 zed_node params that were set as post-include top-level
# <param> tags -- see this file's module docstring for why they are not
# mechanically portable. Preserved here, not applied.
ORIGINAL_ROS1_ZED_PARAMS = {
    "pos_tracking.publish_tf": False,
    "pos_tracking.publish_map_tf": False,
    "pos_tracking.odometry_frame": "odom",
    "pos_tracking.map_frame": "map",
    "general.base_frame": "zed_mini_base_link",
    "pos_tracking.area_memory": False,
    "pos_tracking.save_area_memory_db_on_exit": False,
    "pos_tracking.area_memory_db_path": "",
    "pos_tracking.floor_alignment": False,
    "pos_tracking.two_d_mode": True,
    "pos_tracking.fixed_z_value": 0.0,
    "depth.depth_mode": "NONE",
    "pos_tracking.pos_tracking_enabled": False,
    "general.pub_frame_rate": 30.0,
}


def _combined_urdf_robot_description():
    return ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " --inorder ",
                PathJoinSubstitution(
                    [
                        get_package_share_directory("feeding_deployment"),
                        "urdf",
                        "combined.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )


def generate_launch_description():
    robot_description = {"robot_description": _combined_urdf_robot_description()}

    declares = [
        DeclareLaunchArgument(
            "port1",
            default_value="/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.4.4:1.0-port0",
        ),
        DeclareLaunchArgument(
            "port2",
            default_value="/dev/serial/by-path/pci-0000:00:14.0-usb-0:4.4.3:1.0-port0",
        ),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("camera_name", default_value="zed_mini"),
        DeclareLaunchArgument("camera_base_frame", default_value="zed_mini_base_link"),
        # Record raw ZED stereo+IMU to SVO for the navigation dataset (see
        # the IMU-only ZED block below).
        DeclareLaunchArgument("zed_svo_record", default_value="true"),
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
    ]

    nodes = [
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            parameters=[
                {
                    "source_list": ["robot_joint_states", "wrist_joint_states"],
                    "rate": 100,
                }
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description, {"ignore_timestamp": True}],
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
                "--frame-id", "vention_base_link",
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
                "--frame-id", "vention_base_link",
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
        LogInfo(
            msg=(
                "TODO(ros2): ZED Mini passive-witness include skipped --"
                " see this file's docstring + zed.launch.py /"
                " base_sensors.launch.py for the zed_wrapper"
                " availability/param caveats. ORIGINAL_ROS1_ZED_PARAMS"
                " documents what needs re-applying once a real ROS2 zed"
                " wrapper include is wired up."
            )
        ),
        Node(
            package="feeding_deployment",
            executable="zed_svo_recorder.py",
            name="zed_svo_recorder",
            output="screen",
            condition=IfCondition(LaunchConfiguration("zed_svo_record")),
            parameters=[
                {
                    "camera_name": LaunchConfiguration("camera_name"),
                    # Dataset disk (ext4 NVMe, label "robot-data"), NOT the
                    # near-full OS drive; sibling of the rosbag output
                    # (see scripts/record_meal.sh).
                    "output_dir": "/data/feeding_dataset/svo",
                }
            ],
        ),
        # ===== Fused wheel+IMU odometry (AUTHORITATIVE odom->base) =====
        # Encoder odometry from the NUC base RPC. Blocks retrying the RPC
        # at startup if base_server isn't up yet; respawn covers crashes
        # only.
        Node(
            package="feeding_deployment",
            executable="wheel_odom_publisher.py",
            name="wheel_odom_publisher",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        # Stillness-calibrated debiasing of the ZED gyro (the raw gyro
        # carries a per-boot bias lottery, measured -131 deg/h on Jul 10).
        # Wait for its "first calibration complete" log (needs >=15 s of
        # stillness with fresh /wheel_odom) before trusting yaw.
        Node(
            package="feeding_deployment",
            executable="gyro_bias_estimator.py",
            name="gyro_bias_estimator",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
        ),
        # The authoritative EKF: wheel vx (+advisory wheel vyaw) + debiased
        # gyro yaw-rate. Owns odom->vention_base_link (publish_tf override
        # below); the ZED VIO is excluded by pointing odom0 at a topic
        # nobody publishes. Cartographer's odom topic and TEB's odom
        # feedback both consume /odometry/fused_imu_wheel.
        # Deliberately NO respawn: an auto-restarted EKF resumes at the
        # odom origin, a silent unsanitized teleport into Cartographer's
        # extrapolator (jul02 corruption class). If it ever dies, the
        # stack goes loudly dead (carto collator stalls, TEB loses odom);
        # restart sensors (pane 2), then prefix+r. The wheel/gyro nodes
        # above DO respawn safely: the EKF fuses only the wheel TWIST
        # (reset-proof) and the estimator simply re-calibrates at the next
        # stillness.
        Node(
            package="robot_localization",
            executable="ekf_node",  # ROS1 name was ekf_localization_node
            name="ekf_fused_imu_wheel",
            output="screen",
            parameters=[
                os.path.join(
                    get_package_share_directory("feeding_deployment"),
                    "config",
                    "nav",
                    "ekf_zed_wheel.yaml",
                ),
                {
                    "publish_tf": True,
                    "odom0": "/odometry/_zed_vio_disabled",
                    "imu0": [LaunchConfiguration("camera_name"), "/zed_node/imu/data_debiased"],
                    "imu0_config": [
                        False, False, False,
                        False, False, False,
                        False, False, False,
                        False, False, True,
                        False, False, False,
                    ],
                    "imu0_differential": False,
                    "imu0_relative": False,
                    "imu0_queue_size": 20,
                },
            ],
            remappings=[("odometry/filtered", "/odometry/fused_imu_wheel")],
        ),
        # ===== Arm-specific TFs (from robot.launch, with arm_ prefix) =====
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
                "--frame-id", "arm_end_effector_link",
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
                "--frame-id", "arm_finger_tip", "--child-frame-id", "arm_forkbase",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_drinking_tool",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "arm_finger_tip", "--child-frame-id", "arm_drinkbase",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_wiping_tool",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                "--frame-id", "arm_finger_tip", "--child-frame-id", "arm_wipebase",
            ],
        ),
        # ===== Arm RealSense camera driver (from robot.launch) =====
        # D435: run color and depth at 1280x720 @ 30 Hz (depth's
        # max-at-30Hz), so both streams are at native res for depth
        # alignment / pointcloud.
        # DISABLED [2026-07-10]: turned off to test hypothesis that
        # RealSense and ZED compete for USB bandwidth/resources, causing
        # the ZED to die. If confirmed, the fix is to move them to
        # separate USB controllers rather than leave this off.
        # NOTE: when this is disabled, the camera frequency + resolution
        # checks in watchdog.py MUST also be disabled or the watchdog
        # E-stops the arm immediately.
        LogInfo(
            msg=(
                "TODO(ros2): arm RealSense include intentionally left"
                " disabled here, matching the ROS1 source's"
                " '[2026-07-10] DISABLED' state -- see"
                " arm_sensors.launch.py for the (also unverified) ROS2"
                " realsense2_camera include if/when re-enabling."
            )
        ),
        LogInfo(
            msg=(
                "TODO(ros2): netft_rdt_driver has no known ROS2 port -- FT"
                " sensor is not brought up by this launch file."
            )
        ),
        # ===== ROS bridge for web app (from robot.launch) =====
        # Bohan [Jun 18, 2026]: to allow rosbridge serves TLS
        # automatically when certfile + keyfile are present.
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[
                {
                    "certfile": "/home/isacc/certs/192.168.1.2.pem",
                    "keyfile": "/home/isacc/certs/192.168.1.2-key.pem",
                    "port": 9090,
                }
            ],
        ),
    ]

    return LaunchDescription(declares + nodes)
