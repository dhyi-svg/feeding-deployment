#!/usr/bin/env python3
"""
teb_shape_test.py — drive TEB along a FIXED shape and log plan-tracking + hunting.

Isolates TEB tuning from the task behavior trees: it sends a single move_base goal
that induces a pure straight line, an in-place rotation, or an arc (relative to the
robot's current map pose), then records the global plan vs the executed trajectory
and the /cmd_vel it produced, so you can tune weights against a repeatable target
instead of a full fridge->microwave leg.

Metrics on completion:
  * cross-track error   (base distance to the nearest global-plan pose): max, RMS
  * heading error to goal at the end
  * hunting             (/cmd_vel angular.z sign flips, and per-minute rate)
  * duration, terminal state, final residual vs goal

Shapes (relative to current pose):
  line    --forward <m>
  rotate  --turn <deg>
  arc     --forward <m> --turn <deg>          (end pose forward + rotated)
  goal    --x --y (--yaw|--yaw-deg) | --location NAME    (absolute, map frame)

By default it targets the raw `move_base` action (bypassing shared_autonomy_manager)
so the takeover/hold state machine can't interfere with a controller test; use
--action navigate to go through the manager.

Examples:
  rosrun feeding_deployment teb_shape_test.py line   --forward 1.5
  rosrun feeding_deployment teb_shape_test.py rotate --turn 90
  rosrun feeding_deployment teb_shape_test.py arc    --forward 1.5 --turn 90
  rosrun feeding_deployment teb_shape_test.py goal   --location table
"""

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path


def _parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="shape", required=True)

    ln = sub.add_parser("line", help="straight line forward")
    ln.add_argument("--forward", type=float, required=True, help="distance ahead (m, signed)")

    ro = sub.add_parser("rotate", help="in-place rotation")
    ro.add_argument("--turn", type=float, required=True, help="heading change (deg, signed)")

    ar = sub.add_parser("arc", help="forward + rotate")
    ar.add_argument("--forward", type=float, required=True, help="distance ahead (m)")
    ar.add_argument("--turn", type=float, required=True, help="heading change (deg)")

    go = sub.add_parser("goal", help="absolute map-frame goal")
    go.add_argument("--x", type=float)
    go.add_argument("--y", type=float)
    go.add_argument("--yaw", type=float, help="goal yaw (rad)")
    go.add_argument("--yaw-deg", type=float, help="goal yaw (deg)")
    go.add_argument("--location", help="named location from nav_named_locations.yaml")

    for sp in (ln, ro, ar, go):
        sp.add_argument("--action", default="move_base",
                        help="action server (default move_base; use 'navigate' for the manager)")
        sp.add_argument("--rate", type=float, default=10.0, help="sample rate Hz")
        sp.add_argument("--timeout", type=float, default=120.0, help="goal timeout (s)")
        sp.add_argument("--plan-topic", default=None,
                        help="global plan topic (default: auto TEB then NavfnROS)")
        sp.add_argument("--map-frame", default="map")
        sp.add_argument("--base-frame", default="vention_base_link")
        sp.add_argument("--locations-file", default=None)
        sp.add_argument("--out-dir", default=None, help="CSV dir (default <repo>/nav_tuning_logs)")
        sp.add_argument("--label", default=None)
    return p.parse_args()


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _load_location(path, name):
    import yaml
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    locs = data.get("locations", {})
    if name not in locs:
        raise KeyError(f"location '{name}' not in {path}; have {sorted(locs)}")
    t = locs[name]
    pos, ori = t.get("position", {}), t.get("orientation", {})
    return (float(pos["x"]), float(pos["y"]),
            _yaw_from_quat(type("Q", (), {"x": float(ori["x"]), "y": float(ori["y"]),
                                          "z": float(ori["z"]), "w": float(ori["w"])})))


def main():
    args = _parse_args()

    try:
        import rospy
        import actionlib
        import tf2_ros
        from actionlib_msgs.msg import GoalStatus
        from geometry_msgs.msg import Twist
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from nav_msgs.msg import Path as PathMsg  # avoid shadowing pathlib.Path
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"ROS import failed (run inside your ROS env): {e}\n")
        return 2

    rospy.init_node("teb_shape_test", anonymous=True, disable_signals=True)

    repo_root = Path(__file__).resolve().parents[1]
    out_dir = Path(args.out_dir) if args.out_dir else repo_root / "nav_tuning_logs"
    out_dir.mkdir(parents=True, exist_ok=True)

    tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buf)

    def read_map_base():
        try:
            tf = tf_buf.lookup_transform(args.map_frame, args.base_frame,
                                         rospy.Time(0), rospy.Duration(0.2))
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        return t.x, t.y, _yaw_from_quat(tf.transform.rotation)

    # ---- resolve the goal (absolute map frame) ----
    # ~5 s budget to acquire the first pose (returns instantly when Cartographer
    # is up; fails fast rather than hanging when TF is absent).
    cur = None
    for _ in range(25):
        cur = read_map_base()
        if cur is not None:
            break
        rospy.sleep(0.05)
    if cur is None and args.shape != "goal":
        sys.stderr.write("No map->base TF; is Cartographer up? Relative shapes need it.\n")
        return 3

    if args.shape == "line":
        cx, cy, cyaw = cur
        gx = cx + args.forward * math.cos(cyaw)
        gy = cy + args.forward * math.sin(cyaw)
        gyaw = cyaw
    elif args.shape == "rotate":
        cx, cy, cyaw = cur
        gx, gy, gyaw = cx, cy, _wrap(cyaw + math.radians(args.turn))
    elif args.shape == "arc":
        cx, cy, cyaw = cur
        gyaw = _wrap(cyaw + math.radians(args.turn))
        gx = cx + args.forward * math.cos(gyaw)
        gy = cy + args.forward * math.sin(gyaw)
    else:  # goal
        if args.location:
            loc_file = args.locations_file or os.environ.get("FEEDING_NAV_LOCATIONS_FILE") \
                or str(repo_root / "config" / "nav_named_locations.yaml")
            gx, gy, gyaw = _load_location(loc_file, args.location)
        else:
            if args.x is None or args.y is None:
                sys.stderr.write("goal needs --location or --x/--y (+ optional --yaw).\n")
                return 3
            gx, gy = args.x, args.y
            gyaw = math.radians(args.yaw_deg) if args.yaw_deg is not None else (args.yaw or 0.0)

    # ---- state from topics ----
    state = {"plan": [], "cmd_w": float("nan"), "cmd_v": float("nan")}

    def _on_plan(msg):
        state["plan"] = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]

    def _on_cmd(msg):
        state["cmd_v"] = msg.linear.x
        state["cmd_w"] = msg.angular.z

    plan_topic = args.plan_topic or "/move_base/TebLocalPlannerROS/global_plan"
    rospy.Subscriber(plan_topic, PathMsg, _on_plan, queue_size=1)
    # fallback so we still get a plan if TEB's transformed-plan topic is absent
    if args.plan_topic is None:
        rospy.Subscriber("/move_base/NavfnROS/plan", PathMsg, _on_plan, queue_size=1)
    rospy.Subscriber("/cmd_vel", Twist, _on_cmd, queue_size=10)

    client = actionlib.SimpleActionClient(args.action, MoveBaseAction)
    rospy.loginfo("teb_shape_test: waiting for action '%s'...", args.action)
    if not client.wait_for_server(rospy.Duration(15.0)):
        sys.stderr.write(f"action server '{args.action}' not available\n")
        return 3

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = args.map_frame
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = gx
    goal.target_pose.pose.position.y = gy
    qx, qy, qz, qw = _quat_from_yaw(gyaw)
    goal.target_pose.pose.orientation.x = qx
    goal.target_pose.pose.orientation.y = qy
    goal.target_pose.pose.orientation.z = qz
    goal.target_pose.pose.orientation.w = qw

    rospy.loginfo("teb_shape_test: shape=%s goal=(%.3f, %.3f, %.1fdeg) via %s",
                  args.shape, gx, gy, math.degrees(gyaw), args.action)
    client.send_goal(goal)

    label = args.label or args.shape
    csv_path = out_dir / f"tebshape_{label}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    rate = rospy.Rate(args.rate)
    rows = []
    flips = 0
    last_wsign = 0
    start = rospy.Time.now()

    def cross_track(px, py):
        if not state["plan"]:
            return float("nan")
        return min(math.hypot(px - qx_, py - qy_) for qx_, qy_ in state["plan"])

    try:
        while not rospy.is_shutdown():
            st = client.get_state()
            if st not in (GoalStatus.ACTIVE, GoalStatus.PENDING):
                break
            if (rospy.Time.now() - start).to_sec() > args.timeout:
                client.cancel_goal()
                rospy.logwarn("teb_shape_test: timeout, cancelled")
                break
            mb = read_map_base()
            xt = cross_track(mb[0], mb[1]) if mb else float("nan")
            w = state["cmd_w"]
            if not math.isnan(w) and abs(w) > 0.05:
                s = 1 if w > 0 else -1
                if last_wsign != 0 and s != last_wsign:
                    flips += 1
                last_wsign = s
            rows.append({
                "t": round((rospy.Time.now() - start).to_sec(), 4),
                "mb_x": mb[0] if mb else float("nan"),
                "mb_y": mb[1] if mb else float("nan"),
                "mb_yaw": mb[2] if mb else float("nan"),
                "cross_track_m": xt,
                "cmd_v": state["cmd_v"], "cmd_w": state["cmd_w"],
                "plan_len": len(state["plan"]),
            })
            rate.sleep()
    finally:
        pass

    term = client.get_state()
    fields = ["t", "mb_x", "mb_y", "mb_yaw", "cross_track_m", "cmd_v", "cmd_w", "plan_len"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    xts = [r["cross_track_m"] for r in rows if not math.isnan(r["cross_track_m"])]
    dur = rows[-1]["t"] if rows else 0.0
    mb = read_map_base()
    res_xy = math.hypot(mb[0] - gx, mb[1] - gy) if mb else float("nan")
    res_yaw = math.degrees(abs(_wrap(mb[2] - gyaw))) if mb else float("nan")
    max_xt = max(xts) if xts else float("nan")
    rms_xt = math.sqrt(sum(v * v for v in xts) / len(xts)) if xts else float("nan")
    flip_rate = flips / (dur / 60.0) if dur > 0 else 0.0

    names = {GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED",
             GoalStatus.PREEMPTED: "PREEMPTED", GoalStatus.REJECTED: "REJECTED"}
    summary = (f"state={names.get(term, term)}  dur={dur:.1f}s  "
               f"cross-track max={max_xt*100:.1f}cm rms={rms_xt*100:.1f}cm  "
               f"hunting={flips} flips ({flip_rate:.1f}/min)  "
               f"final residual |xy|={res_xy*100:.1f}cm yaw={res_yaw:.1f}deg")
    rospy.loginfo("teb_shape_test DONE. %s", summary)
    print(f"[teb_shape_test] wrote {len(rows)} rows -> {csv_path}")
    print(f"[teb_shape_test] {summary}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
