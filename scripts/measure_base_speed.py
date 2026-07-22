#!/usr/bin/env python3
"""
measure_base_speed.py -- report the current speed of the robot base, live.

Default: reads the twist off a nav_msgs/Odometry topic (default /wheel_odom,
the wheel-encoder odometry). Forward is +x, reverse is -x. This is the
*commanded-through-the-wheels* speed: counts * counts_per_meter / dt. It is
blind to slip -- if the wheels spin without traction, or the robot is pushed,
it lies.

--compare: also derives the *localized* ground speed by numerically
differentiating the map -> base TF (ZED VIO corrected by Cartographer -- the
robot's true pose in the map). Prints wheel speed, localized speed, and their
difference. That difference is your slip / drift estimate. The localized signal
is jumpy (Cartographer corrects in discrete steps), so use --window to smooth.

Examples:
  rosrun feeding_deployment measure_base_speed.py
  rosrun feeding_deployment measure_base_speed.py --window 0.5
  rosrun feeding_deployment measure_base_speed.py --compare --window 0.5
  rosrun feeding_deployment measure_base_speed.py --topic /odometry/fused_imu_wheel
"""

import argparse
import math
from collections import deque

import rospy
import tf2_ros
from nav_msgs.msg import Odometry


class Channel:
    """Smoothing window + run stats for one signed forward-speed signal."""

    def __init__(self, window):
        self.window = window        # moving-average window (s); 0 = instantaneous
        self.buf = deque()          # (stamp, vx, wz, ground)
        self.max_fwd = 0.0
        self.max_rev = 0.0
        self.max_ground = 0.0
        self.sum_ground = 0.0
        self.n = 0
        self.have = False

    def add(self, stamp, vx, wz, ground):
        self.buf.append((stamp, vx, wz, ground))
        cutoff = stamp - self.window
        while len(self.buf) > 1 and self.buf[0][0] < cutoff:
            self.buf.popleft()
        self.max_fwd = max(self.max_fwd, vx)
        self.max_rev = min(self.max_rev, vx)
        self.max_ground = max(self.max_ground, ground)
        self.sum_ground += ground
        self.n += 1
        self.have = True

    def value(self):
        n = len(self.buf)
        if not n:
            return 0.0, 0.0, 0.0
        return (sum(b[1] for b in self.buf) / n,
                sum(b[2] for b in self.buf) / n,
                sum(b[3] for b in self.buf) / n)

    def summary(self, name):
        if not self.n:
            return f"{name}: no messages received"
        return (f"{name}: samples={self.n}  "
                f"max_fwd={self.max_fwd:+.3f}  max_rev={self.max_rev:+.3f}  "
                f"max_ground={self.max_ground:.3f}  "
                f"mean_ground={self.sum_ground / self.n:.3f}  (m/s)")


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _norm_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/wheel_odom",
                    help="nav_msgs/Odometry topic for wheel speed (default: %(default)s)")
    ap.add_argument("--window", type=float, default=0.0,
                    help="moving-average window in seconds (default: 0 = raw)")
    ap.add_argument("--compare", action="store_true",
                    help="also show localized speed from d/dt of map->base TF")
    ap.add_argument("--map-frame", default="map", help="(default: %(default)s)")
    ap.add_argument("--base-frame", default="vention_base_link",
                    help="(default: %(default)s)")
    ap.add_argument("--tf-rate", type=float, default=20.0,
                    help="Hz to sample map->base TF in --compare (default: %(default)s)")
    args = ap.parse_args()

    rospy.init_node("measure_base_speed", anonymous=True)
    wheel = Channel(args.window)
    loc = Channel(args.window) if args.compare else None

    def render():
        wvx, wwz, wg = wheel.value()
        arrow = "->" if wvx >= 0 else "<-"
        if not args.compare:
            line = (f"wheel {wvx:+.3f} m/s {arrow}  yaw {wwz:+.3f} rad/s  "
                    f"ground {wg:.3f}  ({args.topic})")
        else:
            lvx, lwz, lg = loc.value()
            delta = wvx - lvx
            loc_txt = f"{lvx:+.3f}" if loc.have else "  ...  "
            d_txt = f"{delta:+.3f}" if loc.have else "  ... "
            line = (f"wheel {wvx:+.3f}  map {loc_txt}  "
                    f"delta(slip) {d_txt} m/s  yaw {wwz:+.3f} rad/s")
        print("\r " + line + "    ", end="", flush=True)

    def wheel_cb(msg):
        t = msg.header.stamp.to_sec() or rospy.get_time()
        lin, ang = msg.twist.twist.linear, msg.twist.twist.angular
        wheel.add(t, lin.x, ang.z, math.hypot(lin.x, lin.y))
        render()

    rospy.Subscriber(args.topic, Odometry, wheel_cb, queue_size=50)

    if args.compare:
        buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(buf)
        state = {"t": None, "x": None, "y": None, "yaw": None}

        def tf_cb(_evt):
            try:
                tf = buf.lookup_transform(args.map_frame, args.base_frame,
                                          rospy.Time(0), rospy.Duration(0.05))
            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException):
                return
            t = tf.header.stamp.to_sec()
            x, y = tf.transform.translation.x, tf.transform.translation.y
            yaw = _yaw_from_quat(tf.transform.rotation)
            p = state
            if p["t"] is not None and t > p["t"]:
                dt = t - p["t"]
                dx, dy = x - p["x"], y - p["y"]
                # signed forward speed: displacement projected onto heading
                fwd = (dx * math.cos(p["yaw"]) + dy * math.sin(p["yaw"])) / dt
                wz = _norm_angle(yaw - p["yaw"]) / dt
                loc.add(t, fwd, wz, math.hypot(dx, dy) / dt)
                render()
            if p["t"] is None or t > p["t"]:
                p.update(t=t, x=x, y=y, yaw=yaw)

        rospy.Timer(rospy.Duration(1.0 / args.tf_rate), tf_cb)

    mode = "raw" if args.window <= 0 else f"{args.window}s avg"
    src = (f"{args.topic}  +  d/dt {args.map_frame}->{args.base_frame}"
           if args.compare else args.topic)
    print(f"Listening: {src}  ({mode}) -- Ctrl-C to stop\n")

    try:
        rospy.spin()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        print("\n")
        print(wheel.summary("wheel"))
        if args.compare:
            print(loc.summary("map  "))


if __name__ == "__main__":
    main()
