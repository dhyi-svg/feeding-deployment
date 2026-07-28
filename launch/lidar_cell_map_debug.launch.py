"""ROS2 port of lidar_cell_map_debug.launch.

TODO(ros2) -- NOT a mechanical port, needs a human decision:
ROS1's `costmap_2d` package shipped a standalone `costmap_2d_node`
executable that could run one obstacle-layer costmap completely outside
move_base, which is exactly what this debug-only visualization aid used.
ROS2's `nav2_costmap_2d` has no equivalent standalone/bare-node executable
-- a Costmap2DROS is normally constructed and lifecycle-managed from
inside a Nav2 server (controller_server/planner_server) or a small custom
composed-node wrapper a human would need to write. This file preserves the
original intent (debug-only lidar occupancy overlay, no static map, no
inflation, no interaction with real navigation) and the exact tuned
parameter values as a plain dict below for whoever builds the ROS2
replacement, but does NOT actually launch a working costmap node -- doing
so would require either standing up a minimal Nav2 controller_server just
to host one costmap (heavyweight for a debug tool) or hand-writing a
lifecycle-node wrapper around nav2_costmap_2d.Costmap2DROS, and guessing at
either felt worse than leaving this explicitly unfinished.
"""
from launch import LaunchDescription
from launch.actions import LogInfo

# Original ROS1 costmap_2d_node params, preserved verbatim for whoever
# implements the ROS2 replacement. Nav2's costmap YAML schema differs (its
# plugin list format changed from "{name: x, type: 'ns::Class'}" dicts to
# a flat "plugin_names"/"plugin_types" pair, and every param needs the
# node's `ros__parameters:` wrapper) -- NOT reformatted here, see the
# TODO(ros2) note in ROS2_MIGRATION_NOTES.md about nav-stack YAML files.
ORIGINAL_ROS1_COSTMAP_PARAMS = {
    "global_frame": "map",
    "robot_base_frame": "vention_base_link",
    "update_frequency": 10.0,
    "publish_frequency": 5.0,
    "transform_tolerance": 0.5,
    "rolling_window": True,
    "width": 12.0,
    "height": 12.0,
    "resolution": 0.05,
    "track_unknown_space": False,
    "footprint": [[0.37, 0.295], [0.37, -0.295], [-0.37, -0.295], [-0.37, 0.295]],
    "plugins": [{"name": "obstacle_layer", "type": "costmap_2d::ObstacleLayer"}],
    "obstacle_layer": {
        "observation_sources": "lidar_l lidar_r",
        "lidar_l": {
            "data_type": "LaserScan",
            "topic": "/lidar_l/scan",
            "marking": True,
            "clearing": True,
            "obstacle_range": 8.0,
            "raytrace_range": 10.0,
            "min_obstacle_height": -2.0,
            "max_obstacle_height": 2.0,
        },
        "lidar_r": {
            "data_type": "LaserScan",
            "topic": "/lidar_r/scan",
            "marking": True,
            "clearing": True,
            "obstacle_range": 8.0,
            "raytrace_range": 10.0,
            "min_obstacle_height": -2.0,
            "max_obstacle_height": 2.0,
        },
    },
}


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(
                msg=(
                    "TODO(ros2): lidar_cell_map_debug has NOT been ported --"
                    " costmap_2d_node has no ROS2 standalone equivalent."
                    " See this file's module docstring and"
                    " ROS2_MIGRATION_NOTES.md."
                )
            )
        ]
    )
