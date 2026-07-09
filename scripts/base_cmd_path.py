#!/usr/bin/env python3
"""
base_cmd_path.py — repeatable ACTUATION-level path driver + logger.

Isolates the base + cmd_vel bridge (stiction floor / relay / latency) from TEB and
localization. It publishes a scripted Twist on /cmd_vel_teleop (the hold-EXEMPT
stream, so the ZED-divergence interlock cannot silently zero the test), drives one
of a few fixed shapes open-loop, then stops and records the settle so you can see
overshoot directly.

It logs, per tick:
  * commanded v / w  (what we published)
  * applied  v / w  (the bridge echo on /cmd_vel_bridge_basicmicro/applied, i.e.
                     AFTER the min_move_units stiction floor + clamp)
  * wheel_odom pose  (advisory dead-reckoning, /wheel_odom)
  * map->base pose   (closed-loop truth from TF, if Cartographer is up)

Shapes (direction from the mode name; --v/--w are positive magnitudes):
  straight      --v <m/s>   --distance <m>              forward
  reverse       --v <m/s>   --distance <m>              backward
  rotate_left   --w <rad/s> --angle <deg>               CCW / left
  rotate_right  --w <rad/s> --angle <deg>               CW / right
  arc           --v <m/s>   --w <rad/s>  --duration <s> general curve (signed v/w)

Safety:
  * velocities are clamped to --max-v / --max-w
  * the drive phase stops at whichever comes first: the nominal time, the measured
    progress target, or --max-duration
  * zeros are published on every exit path (convergence, timeout, Ctrl-C, error)
  * --dry-run logs and computes without publishing any motion

Why /cmd_vel_teleop and not /cmd_vel: the autonomous stream is muted while a human
is "driving" and is gated by /nav_safety_hold; teleop is priority + hold-exempt,
which is exactly what a scripted actuation test wants. NOTE: a nonzero teleop
command mutes autonomous /cmd_vel for ~teleop_mute_s — fine here (no autonomy runs
during the test), but don't run this against a live move_base goal.

Examples:
  rosrun feeding_deployment base_cmd_path.py straight     --v 0.2 --distance 1.0
  rosrun feeding_deployment base_cmd_path.py reverse      --v 0.2 --distance 1.0
  rosrun feeding_deployment base_cmd_path.py rotate_left  --w 0.4 --angle 90
  rosrun feeding_deployment base_cmd_path.py rotate_right --w 0.4 --angle 90
  rosrun feeding_deployment base_cmd_path.py arc          --v 0.2 --w 0.3 --duration 5
"""

import argparse
import csv
import math
import sys
import time
from pathlib import Path


def _parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="mode", required=True)

    # Direction comes from the mode name (like calibrate_wheel_odom); --v/--w are
    # positive magnitudes in these four. `arc` stays the general signed case.
    st = sub.add_parser("straight", help="drive forward a fixed distance")
    st.add_argument("--v", type=float, required=True, help="linear speed magnitude (m/s)")
    st.add_argument("--distance", type=float, required=True, help="target distance (m, >0)")

    rv = sub.add_parser("reverse", help="drive backward a fixed distance")
    rv.add_argument("--v", type=float, required=True, help="linear speed magnitude (m/s)")
    rv.add_argument("--distance", type=float, required=True, help="target distance (m, >0)")

    rl = sub.add_parser("rotate_left", help="rotate in place CCW / left a fixed angle")
    rl.add_argument("--w", type=float, required=True, help="angular speed magnitude (rad/s)")
    rl.add_argument("--angle", type=float, required=True, help="target angle (deg, >0)")

    rr = sub.add_parser("rotate_right", help="rotate in place CW / right a fixed angle")
    rr.add_argument("--w", type=float, required=True, help="angular speed magnitude (rad/s)")
    rr.add_argument("--angle", type=float, required=True, help="target angle (deg, >0)")

    a = sub.add_parser("arc", help="drive an arc for a fixed duration (signed --v/--w)")
    a.add_argument("--v", type=float, required=True, help="linear vel (m/s, signed)")
    a.add_argument("--w", type=float, required=True, help="angular vel (rad/s, signed)")
    a.add_argument("--duration", type=float, required=True, help="drive duration (s, >0)")

    for sp in (st, rv, rl, rr, a):
        sp.add_argument("--rate", type=float, default=10.0, help="publish rate Hz (default 10, match controller_frequency)")
        sp.add_argument("--topic", default="/cmd_vel_teleop", help="Twist topic (default /cmd_vel_teleop, hold-exempt)")
        sp.add_argument("--settle", type=float, default=3.0, help="post-stop observation window (s)")
        sp.add_argument("--max-v", type=float, default=0.4, help="linear vel clamp (m/s)")
        sp.add_argument("--max-w", type=float, default=0.6, help="angular vel clamp (rad/s)")
        sp.add_argument("--max-duration", type=float, default=30.0, help="hard drive-phase cap (s)")
        sp.add_argument("--map-frame", default="map")
        sp.add_argument("--base-frame", default="vention_base_link")
        sp.add_argument("--out-dir", default=None, help="CSV dir (default <repo>/nav_tuning_logs)")
        sp.add_argument("--label", default=None, help="filename label")
        sp.add_argument("--dry-run", action="store_true", help="log + compute without publishing motion")
    return p.parse_args()


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    args = _parse_args()

    # Import ROS only after arg parsing so `--help` works without a ROS env.
    try:
        import rospy
        import tf2_ros
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
    except Exception as e:  # noqa: BLE001
        sys.stderr.write(f"ROS import failed (run inside your ROS env): {e}\n")
        return 2

    rospy.init_node("base_cmd_path", anonymous=True, disable_signals=True)

    repo_root = Path(__file__).resolve().parents[1]
    out_dir = Path(args.out_dir) if args.out_dir else repo_root / "nav_tuning_logs"
    out_dir.mkdir(parents=True, exist_ok=True)
    label = args.label or args.mode
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = out_dir / f"cmdpath_{label}_{stamp}.csv"

    # ---- state populated by callbacks ----
    state = {"applied_v": float("nan"), "applied_w": float("nan"),
             "wo_x": float("nan"), "wo_y": float("nan"), "wo_yaw": float("nan")}

    def _on_applied(msg):
        state["applied_v"] = msg.linear.x
        state["applied_w"] = msg.angular.z

    def _on_wheel(msg):
        state["wo_x"] = msg.pose.pose.position.x
        state["wo_y"] = msg.pose.pose.position.y
        state["wo_yaw"] = _yaw_from_quat(msg.pose.pose.orientation)

    pub = rospy.Publisher(args.topic, Twist, queue_size=1)
    rospy.Subscriber("/cmd_vel_bridge_basicmicro/applied", Twist, _on_applied, queue_size=10)
    rospy.Subscriber("/wheel_odom", Odometry, _on_wheel, queue_size=10)

    tf_buf = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf2_ros.TransformListener(tf_buf)

    def read_map_base():
        """(x, y, yaw) of map->base or None if TF unavailable."""
        try:
            tf = tf_buf.lookup_transform(args.map_frame, args.base_frame,
                                         rospy.Time(0), rospy.Duration(0.05))
        except Exception:  # noqa: BLE001
            return None
        t = tf.transform.translation
        return t.x, t.y, _yaw_from_quat(tf.transform.rotation)

    # ---- build the commanded twist + progress target ----
    # --v/--w are positive magnitudes for the directional modes; sign = mode.
    if args.mode in ("straight", "reverse", "rotate_left", "rotate_right") and (
            getattr(args, "v", 0.0) < 0 or getattr(args, "w", 0.0) < 0):
        sys.stderr.write("--v/--w are magnitudes for this mode; pick direction via the "
                         "mode (reverse / rotate_right), not a negative value.\n")
        return 3
    if args.mode in ("straight", "reverse"):
        speed = _clamp(args.v, 0.0, args.max_v)
        cmd_v = speed if args.mode == "straight" else -speed
        cmd_w = 0.0
        target = abs(args.distance)
        nominal_t = target / max(abs(cmd_v), 1e-6)
        kind = "dist"
    elif args.mode in ("rotate_left", "rotate_right"):
        rate = _clamp(args.w, 0.0, args.max_w)
        cmd_v = 0.0
        cmd_w = rate if args.mode == "rotate_left" else -rate
        target = math.radians(abs(args.angle))
        nominal_t = target / max(abs(cmd_w), 1e-6)
        kind = "yaw"
    else:  # arc (signed)
        cmd_v = _clamp(args.v, -args.max_v, args.max_v)
        cmd_w = _clamp(args.w, -args.max_w, args.max_w)
        target = None
        nominal_t = abs(args.duration)
        kind = "time"

    drive_t = min(nominal_t, args.max_duration)
    rospy.loginfo("base_cmd_path: mode=%s cmd_v=%.3f cmd_w=%.3f drive=%.2fs (nominal %.2fs) "
                  "target=%s dry_run=%s -> %s",
                  args.mode, cmd_v, cmd_w, drive_t, nominal_t,
                  f"{target:.3f} {kind}" if target is not None else kind,
                  args.dry_run, csv_path)

    rate = rospy.Rate(args.rate)
    rows = []
    start = rospy.Time.now()
    origin = None  # (x,y,yaw) at motion start, from map->base if available else wheel_odom

    def sample(phase):
        now = (rospy.Time.now() - start).to_sec()
        mb = read_map_base()
        mb_x, mb_y, mb_yaw = mb if mb else (float("nan"),) * 3
        rows.append({
            "t": round(now, 4), "phase": phase,
            "cmd_v": cmd_v if phase == "drive" else 0.0,
            "cmd_w": cmd_w if phase == "drive" else 0.0,
            "applied_v": state["applied_v"], "applied_w": state["applied_w"],
            "wo_x": state["wo_x"], "wo_y": state["wo_y"], "wo_yaw": state["wo_yaw"],
            "mb_x": mb_x, "mb_y": mb_y, "mb_yaw": mb_yaw,
        })
        return mb

    def progress(mb):
        """Signed progress toward target (m or rad) using map->base if available,
        else wheel_odom. Returns None if no reference yet."""
        ref = None
        if mb is not None:
            ref = mb
        elif not math.isnan(state["wo_x"]):
            ref = (state["wo_x"], state["wo_y"], state["wo_yaw"])
        if ref is None or origin is None:
            return None
        if kind == "dist":
            return math.hypot(ref[0] - origin[0], ref[1] - origin[1])
        if kind == "yaw":
            return abs(_wrap(ref[2] - origin[2]))
        return None

    def publish(v, w):
        if args.dry_run:
            return
        tw = Twist()
        tw.linear.x = v
        tw.angular.z = w
        pub.publish(tw)

    def stop_base():
        for _ in range(5):
            publish(0.0, 0.0)
            rospy.sleep(0.02)

    try:
        # settle publisher/TF connections
        rospy.sleep(0.3)
        mb0 = read_map_base()
        if mb0 is not None:
            origin = mb0
        elif not math.isnan(state["wo_x"]):
            origin = (state["wo_x"], state["wo_y"], state["wo_yaw"])

        # ---- DRIVE ----
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= drive_t:
                break
            publish(cmd_v, cmd_w)
            mb = sample("drive")
            if origin is None:  # late TF/odom — latch origin as soon as we have it
                if mb is not None:
                    origin = mb
                elif not math.isnan(state["wo_x"]):
                    origin = (state["wo_x"], state["wo_y"], state["wo_yaw"])
            prog = progress(mb)
            if target is not None and prog is not None and prog >= target:
                rospy.loginfo("base_cmd_path: reached target (%.3f >= %.3f) at %.2fs",
                              prog, target, elapsed)
                break
            rate.sleep()

        # ---- STOP + SETTLE (this is where overshoot shows up) ----
        stop_base()
        settle_start = rospy.Time.now()
        while not rospy.is_shutdown() and (rospy.Time.now() - settle_start).to_sec() < args.settle:
            sample("settle")
            rate.sleep()
    finally:
        stop_base()

    # ---- write CSV ----
    fields = ["t", "phase", "cmd_v", "cmd_w", "applied_v", "applied_w",
              "wo_x", "wo_y", "wo_yaw", "mb_x", "mb_y", "mb_yaw"]
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    # ---- summary (overshoot) ----
    def _summ():
        if target is None or origin is None:
            return "no closed-loop target/reference — inspect the CSV time series."
        prog_series = []
        for row in rows:
            ref = None
            if not math.isnan(row["mb_x"]):
                ref = (row["mb_x"], row["mb_y"], row["mb_yaw"])
            elif not math.isnan(row["wo_x"]):
                ref = (row["wo_x"], row["wo_y"], row["wo_yaw"])
            if ref is None:
                continue
            if kind == "dist":
                prog_series.append(math.hypot(ref[0] - origin[0], ref[1] - origin[1]))
            else:
                prog_series.append(abs(_wrap(ref[2] - origin[2])))
        if not prog_series:
            return "no map->base or wheel_odom samples captured."
        peak = max(prog_series)
        final = prog_series[-1]
        unit = "m" if kind == "dist" else "deg"
        scale = 1.0 if kind == "dist" else 180.0 / math.pi
        tgt = target * scale
        return (f"target={tgt:.3f}{unit}  peak={peak * scale:.3f}{unit}  "
                f"final={final * scale:.3f}{unit}  overshoot={(peak - target) * scale:+.3f}{unit}  "
                f"residual={(final - target) * scale:+.3f}{unit}")

    applied_ws = [abs(r["applied_w"]) for r in rows
                  if r["phase"] == "drive" and not math.isnan(r["applied_w"]) and r["applied_w"] != 0.0]
    relay_note = ""
    if applied_ws:
        med = sorted(applied_ws)[len(applied_ws) // 2]
        relay_note = f"  |applied w| median during drive = {med:.4f} rad/s (stiction floor ~0.417)"

    rospy.loginfo("base_cmd_path DONE. %s%s", _summ(), relay_note)
    print(f"[base_cmd_path] wrote {len(rows)} rows -> {csv_path}")
    print(f"[base_cmd_path] {_summ()}{relay_note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
