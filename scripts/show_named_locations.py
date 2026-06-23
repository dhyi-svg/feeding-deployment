#!/usr/bin/env python3
"""Visualize the named navigation poses from nav_named_locations.yaml in RViz.

Publishes a latched visualization_msgs/MarkerArray on /named_locations: one arrow
per location (position + heading) and a text label with the location name. Add a
"MarkerArray" display in RViz subscribed to /named_locations, with Fixed Frame set
to "map" (or whatever frame_id the file uses).

    rosrun feeding_deployment show_named_locations.py
    rosrun feeding_deployment show_named_locations.py --file /path/to/locations.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any, Dict, Optional

import yaml

try:
    import rospy
    from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
    from std_msgs.msg import ColorRGBA
    from visualization_msgs.msg import Marker, MarkerArray

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False


# Distinct colors cycled across locations so neighbors are easy to tell apart.
_PALETTE = [
    (0.20, 0.80, 0.20),  # green
    (0.20, 0.55, 1.00),  # blue
    (1.00, 0.55, 0.10),  # orange
    (0.85, 0.20, 0.85),  # magenta
    (0.10, 0.80, 0.80),  # cyan
    (0.90, 0.80, 0.15),  # yellow
]


def _find_locations_file() -> Optional[Path]:
    """Walk up from this script looking for config/nav_named_locations.yaml.

    Robust to source-vs-installed layout differences (capture_named_locations.py
    hardcodes a parents[N] that only resolves when installed)."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "config" / "nav_named_locations.yaml"
        if candidate.exists():
            return candidate
    return None


def _load_locations(filepath: Path) -> Dict[str, Any]:
    with open(filepath, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data


def _make_arrow(name: str, loc: Dict[str, Any], default_frame: str,
                marker_id: int, color: tuple, stamp) -> "Marker":
    m = Marker()
    m.header.frame_id = loc.get("frame_id", default_frame)
    m.header.stamp = stamp
    m.ns = "named_locations"
    m.id = marker_id
    m.type = Marker.ARROW
    m.action = Marker.ADD
    p = loc["position"]
    o = loc["orientation"]
    m.pose = Pose(
        position=Point(x=float(p["x"]), y=float(p["y"]), z=float(p["z"])),
        orientation=Quaternion(
            x=float(o["x"]), y=float(o["y"]), z=float(o["z"]), w=float(o["w"])
        ),
    )
    # scale.x = shaft length, scale.y = shaft diameter, scale.z = head diameter.
    m.scale = Vector3(x=0.5, y=0.08, z=0.14)
    m.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.0)
    return m


def _make_label(name: str, loc: Dict[str, Any], default_frame: str,
                marker_id: int, stamp) -> "Marker":
    m = Marker()
    m.header.frame_id = loc.get("frame_id", default_frame)
    m.header.stamp = stamp
    m.ns = "named_locations_labels"
    m.id = marker_id
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    p = loc["position"]
    m.pose.position = Point(x=float(p["x"]), y=float(p["y"]), z=float(p["z"]) + 0.35)
    m.pose.orientation.w = 1.0
    m.scale.z = 0.22  # text height (m)
    m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    m.text = name
    return m


def _build_marker_array(data: Dict[str, Any], stamp) -> "MarkerArray":
    default_frame = data.get("frame_id", "map")
    locations = data.get("locations", {}) or {}
    arr = MarkerArray()
    # Clear any stale markers first so removed/renamed locations don't linger.
    clear = Marker()
    clear.action = Marker.DELETEALL
    arr.markers.append(clear)
    for i, (name, loc) in enumerate(sorted(locations.items())):
        color = _PALETTE[i % len(_PALETTE)]
        arr.markers.append(_make_arrow(name, loc, default_frame, i, color, stamp))
        arr.markers.append(_make_label(name, loc, default_frame, i, stamp))
    return arr


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--file", type=str, default=None,
        help="Path to nav_named_locations.yaml (default: auto-discovered).",
    )
    parser.add_argument(
        "--topic", type=str, default="/named_locations",
        help="MarkerArray topic to publish on.",
    )
    parser.add_argument(
        "--rate", type=float, default=1.0,
        help="Re-publish rate (Hz) so late RViz subscribers still get markers.",
    )
    args = parser.parse_args()

    if not ROSPY_IMPORTED:
        raise SystemExit("rospy / ROS message packages not available — source your ROS env.")

    filepath = Path(args.file).expanduser() if args.file else _find_locations_file()
    if filepath is None or not filepath.exists():
        raise SystemExit(
            "Could not find nav_named_locations.yaml. Pass --file <path> explicitly."
        )

    data = _load_locations(filepath)
    locations = data.get("locations", {}) or {}

    rospy.init_node("show_named_locations", anonymous=True)
    pub = rospy.Publisher(args.topic, MarkerArray, queue_size=1, latch=True)

    rospy.loginfo("Loaded %d named locations from %s", len(locations), filepath)
    for name, loc in sorted(locations.items()):
        p = loc["position"]
        rospy.loginfo(
            "  %-12s frame=%s  x=%.3f y=%.3f z=%.3f",
            name, loc.get("frame_id", data.get("frame_id", "map")),
            float(p["x"]), float(p["y"]), float(p["z"]),
        )
    rospy.loginfo("Publishing MarkerArray on %s (latched). Add it in RViz; Fixed Frame=map.",
                  args.topic)

    rate = rospy.Rate(max(args.rate, 0.1))
    while not rospy.is_shutdown():
        pub.publish(_build_marker_array(data, rospy.Time.now()))
        rate.sleep()


if __name__ == "__main__":
    main()
