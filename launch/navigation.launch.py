"""ROS2 port of navigation.launch.

TODO(ros2) -- NOT a mechanical port, needs a human decision and real nav
expertise:
`move_base` (ROS1's monolithic navigation node: one process owning a
global planner plugin, local planner plugin, global/local costmaps, and
the recovery-behavior state machine) has NO ROS2 equivalent package at
all -- confirmed via `apt-cache search move-base` / `move_base` on this
box (zero hits, vs. real hits for `ros-jazzy-nav2-*` packages). ROS2's
navigation stack is Nav2: a completely different architecture, splitting
move_base's responsibilities across several lifecycle-managed nodes
(bt_navigator, controller_server, planner_server, smoother_server,
behavior_server, ...) orchestrated by a behavior tree XML instead of a
flat recovery chain, each with its own YAML params file under a
`ros__parameters:` key. This is a real navigation-stack migration
project on its own, not a syntax conversion -- global_planner/GlobalPlanner
and teb_local_planner/TebLocalPlannerROS both have Nav2-plugin
counterparts (nav2 has bundled planner plugins; teb_local_planner has a
maintained Nav2 controller-server plugin port), but wiring them up
correctly (BT XML, lifecycle bringup, costmap plugin YAML in the new
schema) needs a human who owns the nav stack, not a guess made here.

What IS ported below: feeding_deployment's own cmd_vel_bridge_basicmicro
node (a real rospy->rclpy migration target, tracked in the 52-file scope;
see control/base_controller/cmd_vel_bridge_basicmicro.py for the Python
side). The move_base block is left as an explicit, loud placeholder with
every original tuned parameter preserved in a comment/dict so nothing gets
silently lost, but it launches nothing.

zed_health_monitor: was already noted UNPLUMBED/DELETED in the ROS1
source (see original comment, preserved below) -- nothing to port.
"""
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node

# Original ROS1 move_base params, preserved for whoever builds the Nav2
# replacement. NOT reformatted into Nav2's YAML schema (different plugin
# list format, BT-based recovery instead of a flat chain) -- that
# reformatting IS the actual migration work, not something to guess here.
ORIGINAL_ROS1_MOVE_BASE_PARAMS = {
    # Global planner: switched off the move_base default (navfn/NavfnROS)
    # to global_planner/GlobalPlanner (2026-07-13).
    "base_global_planner": "global_planner/GlobalPlanner",
    # dwa_local_planner/DWAPlannerROS was tried and superseded by TEB.
    "base_local_planner": "teb_local_planner/TebLocalPlannerROS",
    "rosparam_files": [
        "config/nav/costmap_common.yaml (ns: global_costmap AND local_costmap)",
        "config/nav/global_costmap.yaml",
        "config/nav/local_costmap.yaml",
        "config/nav/global_planner.yaml",
        "config/nav/teb_local_planner.yaml",
    ],
    "planner_patience": 15.0,
    "controller_patience": 15.0,
    # Drop rotate_recovery from the default recovery chain (leaves
    # conservative_reset -> aggressive_reset -> abort). Its speed limits
    # come from ~/TrajectoryPlannerROS, which doesn't exist under TEB, so
    # it spins at the 0.4-1.0 rad/s defaults, ~8x TEB's max_vel_theta now
    # that angular_scale is calibrated (2026-07-09) and a fast in-place
    # spin is the prime trigger for the 180-deg table alias.
    "clearing_rotation_allowed": False,
    # Belt-and-suspenders for the line above: rotate_recovery reads its
    # speed limits from ~/TrajectoryPlannerROS regardless of the active
    # local planner, so if it is ever re-enabled it would again default to
    # 0.4-1.0 rad/s. Pin that namespace to the supervised teleop caps
    # (shared_autonomy.launch.py: 0.10 m/s / 0.1667 rad/s) so no recovery
    # rotation can out-drive a human operator; acc matches TEB's
    # acc_lim_theta.
    "TrajectoryPlannerROS/max_rotational_vel": 0.1667,
    "TrajectoryPlannerROS/min_in_place_rotational_vel": 0.05,
    "TrajectoryPlannerROS/acc_lim_th": 1.5,
    # 0.2 Hz: global replans re-inject Cartographer yaw jitter into TEB via
    # the plan; at 1.0 Hz this drove ~1 Hz left/right hunting
    # (navlog_20260706_131121). TEB still avoids obstacles locally between
    # replans.
    "planner_frequency": 1,
    "controller_frequency": 10.0,
}


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "TODO(ros2): move_base has NOT been ported -- no ROS2"
                    " equivalent package exists. See this file's module"
                    " docstring, ORIGINAL_ROS1_MOVE_BASE_PARAMS, and"
                    " ROS2_MIGRATION_NOTES.md. Nav2 migration is a"
                    " separate project."
                )
            ),
            # cmd_vel bridge: /cmd_vel -> set_speeds RPC to BaseInterface on
            # the NUC. Runs on the compute box; the Arduino is on the NUC
            # behind base_server.py. The lost-command stop lives on the
            # NUC (BaseInterface), not here.
            Node(
                package="feeding_deployment",
                executable="cmd_vel_bridge_basicmicro.py",
                name="cmd_vel_bridge_basicmicro",
                output="screen",
                # Physically calibrated 2026-07-09 (direct-serial sweeps):
                # linear_scale == counts_per_meter (4874) so commanded m/s
                # == actual; angular_scale (2600) from measured rotation
                # (250 counts/s -> 0.096 rad/s). Speed is governed by TEB
                # max_vel_*, NOT by detuning these. WARNING: ~4-6x higher
                # than before, so the base now runs the full
                # TEB-commanded speed (0.4 m/s / 0.5 rad/s), for teleop
                # AND autonomy; lower TEB max_vel_* before autonomous
                # runs. angular_scale measured on a slip-prone patch;
                # verify/re-tune on the deployment floor.
                parameters=[
                    {
                        "linear_scale": 4874.0,
                        "angular_scale": 2600.0,
                        # Per-wheel ceiling = teleop worst case:
                        # max_vel_x*linear_scale + max_vel_theta*angular_scale
                        # = 0.10*4874 + 0.1667*2600 = 920 units
                        # (shared_autonomy.launch.py caps). Everything
                        # legitimate fits under it (TEB 0.075/0.125 mixes
                        # to 691), but a rogue full-rate command (like
                        # rotate_recovery's 1.0 rad/s spin on Jul 13,
                        # which the old 2500 passed through at
                        # 0.96 rad/s) is now clipped at the choke point to
                        # teleop's own wheel speed. Raise together with
                        # teleop/TEB max_vel_*.
                        "max_speed_units": 920,
                        # Ratio-preserving stiction floor (units); scales
                        # both wheels together so slow commands still move
                        # without flattening arcs. 0 disables.
                        "min_move_units": 100,
                    }
                ],
            ),
        ]
    )
