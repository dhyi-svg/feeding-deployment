#!/usr/bin/env python3
"""Measure the live navigation residual (map-frame pose error) vs a goal pose.

Mirrors the _residual() computation in the refinement window of
feeding_deployment/actions/navigate.py: looks up the latest map ->
vention_base_link transform on TF and prints the signed error
(current - goal) as dx, dy in cm and dyaw in deg, plus |xy|.

Pass the goal pose directly (yaw in radians, or --yaw-deg, or a quaternion):

    rosrun feeding_deployment nav_residual.py --x 1.23 --y -0.45 --yaw-deg 90
    rosrun feeding_deployment nav_residual.py --x 1.23 --y -0.45 --quat 0 0 0.707 0.707

or by named location (same file navigate.py resolves, honoring
FEEDING_NAV_LOCATIONS_FILE):

    rosrun feeding_deployment nav_residual.py --location sink

navigate.py measures its residual against the OFFSET-ADJUSTED goal (the
learned per-user [dx, dy, dyaw] composed onto the nominal pose). To compare
against the same target, pass that offset along:

    rosrun feeding_deployment nav_residual.py --location sink --offset 0.05 -0.02 0.1

Sample continuously while nudging the base with --watch (Ctrl-C to stop):

    rosrun feeding_deployment nav_residual.py --location sink --watch
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Optional

import yaml

try:
    import rospy
    import tf2_ros

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False


# Same clamp bounds as NavigateHLA._MAX_OFFSET_* — keep in sync with navigate.py.
MAX_OFFSET_XY_M = 0.5
MAX_OFFSET_YAW_RAD = math.radians(45.0)


def _wrap(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def _se2_compose(x: float, y: float, yaw: float,
                 dx: float, dy: float, dyaw: float) -> tuple:
    """(x,y,yaw) o (dx,dy,dyaw): apply a local-frame offset to a map-frame pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    return x + dx * c - dy * s, y + dx * s + dy * c, _wrap(yaw + dyaw)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _find_locations_file() -> Optional[Path]:
    """Walk up from this script looking for config/nav_named_locations.yaml,
    honoring FEEDING_NAV_LOCATIONS_FILE the way navigate.py does."""
    user_path = os.environ.get("FEEDING_NAV_LOCATIONS_FILE", "").strip()
    if user_path:
        return Path(user_path).expanduser().resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "config" / "nav_named_locations.yaml"
        if candidate.exists():
            return candidate
    return None


def _load_location(name: str, filepath: Optional[Path]) -> tuple:
    """Return (x, y, yaw) for a named location from nav_named_locations.yaml."""
    if filepath is None or not filepath.exists():
        raise SystemExit(
            "Could not find nav_named_locations.yaml. Pass --file <path> explicitly."
        )
    with open(filepath, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    locations = data.get("locations", {}) or {}
    if name not in locations:
        raise SystemExit(
            f"Missing location '{name}' in {filepath}. "
            f"Available locations: {sorted(locations)}"
        )
    loc = locations[name]
    p = loc["position"]
    o = loc["orientation"]
    return (
        float(p["x"]),
        float(p["y"]),
        _yaw_from_quat(float(o["x"]), float(o["y"]), float(o["z"]), float(o["w"])),
    )


def _resolve_goal(args: argparse.Namespace) -> tuple:
    """Resolve the goal (x, y, yaw) from --location or --x/--y + heading args."""
    if args.location:
        gx, gy, gyaw = _load_location(
            args.location, Path(args.file).expanduser() if args.file else _find_locations_file()
        )
    else:
        if args.x is None or args.y is None:
            raise SystemExit("Pass either --location NAME or --x and --y (plus a heading).")
        gx, gy = args.x, args.y
        if args.quat is not None:
            gyaw = _yaw_from_quat(*args.quat)
        elif args.yaw_deg is not None:
            gyaw = math.radians(args.yaw_deg)
        elif args.yaw is not None:
            gyaw = args.yaw
        else:
            raise SystemExit("Pass a heading: --yaw (rad), --yaw-deg, or --quat qx qy qz qw.")

    if args.offset is not None:
        # Same composition + clamping as NavigateHLA._apply_offset_to_pose.
        dx = _clamp(float(args.offset[0]), MAX_OFFSET_XY_M)
        dy = _clamp(float(args.offset[1]), MAX_OFFSET_XY_M)
        dyaw = _clamp(float(args.offset[2]), MAX_OFFSET_YAW_RAD)
        gx, gy, gyaw = _se2_compose(gx, gy, gyaw, dx, dy, dyaw)
        print(
            f"Applied offset to nominal goal: dx={dx:+.3f}m dy={dy:+.3f}m "
            f"dyaw={math.degrees(dyaw):+.1f}deg"
        )
    return gx, gy, gyaw


def _print_residual(tf_buffer: "tf2_ros.Buffer", args: argparse.Namespace,
                    gx: float, gy: float, gyaw: float) -> bool:
    """Look up map->base and print (current - goal); False if TF lookup failed."""
    try:
        tfm = tf_buffer.lookup_transform(
            args.map_frame, args.base_frame, rospy.Time(0), rospy.Duration(1.0)
        )
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
        tf2_ros.TimeoutException,
    ) as exc:
        print(f"TF lookup {args.map_frame}->{args.base_frame} failed: {exc}")
        return False
    tt = tfm.transform.translation
    qq = tfm.transform.rotation
    yaw = _yaw_from_quat(qq.x, qq.y, qq.z, qq.w)
    dx = tt.x - gx
    dy = tt.y - gy
    dyaw = _wrap(yaw - gyaw)
    print(
        f"Residual error vs goal: dx={dx * 100:+.1f}cm dy={dy * 100:+.1f}cm "
        f"dyaw={math.degrees(dyaw):+.2f}deg (|xy|={math.hypot(dx, dy) * 100:.1f}cm)"
    )
    return True


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    goal = parser.add_argument_group("goal pose")
    goal.add_argument("--location", type=str, default=None,
                      help="Named location from nav_named_locations.yaml.")
    goal.add_argument("--file", type=str, default=None,
                      help="Locations yaml path (default: auto-discovered / env var).")
    goal.add_argument("--x", type=float, default=None, help="Goal x in map frame (m).")
    goal.add_argument("--y", type=float, default=None, help="Goal y in map frame (m).")
    goal.add_argument("--yaw", type=float, default=None, help="Goal yaw (rad).")
    goal.add_argument("--yaw-deg", type=float, default=None, help="Goal yaw (deg).")
    goal.add_argument("--quat", type=float, nargs=4, default=None,
                      metavar=("QX", "QY", "QZ", "QW"), help="Goal orientation quaternion.")
    goal.add_argument("--offset", type=float, nargs=3, default=None,
                      metavar=("DX", "DY", "DYAW"),
                      help="Learned local-frame offset [m, m, rad] composed onto the "
                           "goal, as navigate.py does before sending it.")
    parser.add_argument("--map-frame", type=str, default="map")
    parser.add_argument("--base-frame", type=str, default="vention_base_link")
    parser.add_argument("--watch", action="store_true",
                        help="Keep printing the residual until Ctrl-C.")
    parser.add_argument("--rate", type=float, default=2.0,
                        help="Sampling rate in --watch mode (Hz).")
    args = parser.parse_args()

    if not ROSPY_IMPORTED:
        raise SystemExit("rospy / tf2_ros not available — source your ROS env.")

    gx, gy, gyaw = _resolve_goal(args)
    print(f"Goal (map frame): x={gx:.3f}m y={gy:.3f}m yaw={math.degrees(gyaw):+.2f}deg")

    rospy.init_node("nav_residual", anonymous=True)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
    _listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(0.5)  # let the listener fill before the first lookup

    if args.watch:
        rate = rospy.Rate(max(args.rate, 0.1))
        try:
            while not rospy.is_shutdown():
                _print_residual(tf_buffer, args, gx, gy, gyaw)
                rate.sleep()
        except (KeyboardInterrupt, rospy.ROSInterruptException):
            pass
    else:
        if not _print_residual(tf_buffer, args, gx, gy, gyaw):
            raise SystemExit(1)


if __name__ == "__main__":
    main()
