#!/usr/bin/env python3
"""
print_zed_covariance.py -- print the ZED odom covariance diagonal, throttled.

Shows what the ZED SDK actually reports for pose (x, y, yaw) and twist
(vx, vy, vyaw) uncertainty -- the numbers zed_feedback_cov_relay.py propagates
into the fused-odom EKF. Prints the wheel odom's hand-set covariance alongside
so you can judge the relative trust (whichever variance is smaller wins that
field in the fusion).

  rosrun feeding_deployment print_zed_covariance.py
  rosrun feeding_deployment print_zed_covariance.py --rate 2
  rosrun feeding_deployment print_zed_covariance.py --topic /zed_mini/zed_node/odom
"""

import argparse
import math

import rospy
from nav_msgs.msg import Odometry

# 6x6 row-major diagonal indices: [x/vx, y/vy, z/vz, roll, pitch, yaw/vyaw]
DIAG = [0, 7, 14, 21, 28, 35]

# What wheel_odom_publisher.py stamps on /wheel_odom twist (for comparison).
WHEEL = {"vx": 1e-4, "vyaw": 0.1}


def fmt(var, unit):
    if var <= 0:
        return f"var={var:.1e}  (zero/negative)"
    std = math.sqrt(var)
    extra = f" = {math.degrees(std):.2f} deg" if unit.startswith("rad") else ""
    return f"var={var:.2e}  std={std:.3g} {unit}{extra}"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/zed_mini/zed_node/odom")
    ap.add_argument("--rate", type=float, default=1.0, help="print rate Hz (default 1)")
    args = ap.parse_args()

    rospy.init_node("print_zed_covariance", anonymous=True)
    latest = {"msg": None}
    rospy.Subscriber(args.topic, Odometry, lambda m: latest.__setitem__("msg", m),
                     queue_size=5)

    print(f"listening on {args.topic} (Ctrl-C to stop)")
    print(f"wheel odom stamps (reference):  vx {fmt(WHEEL['vx'], 'm/s')}  |  "
          f"vyaw {fmt(WHEEL['vyaw'], 'rad/s')}\n")

    rate = rospy.Rate(args.rate)
    while not rospy.is_shutdown():
        msg = latest["msg"]
        if msg is None:
            rospy.logwarn_throttle(3.0, f"no messages on {args.topic} yet...")
            rate.sleep()
            continue
        p = [msg.pose.covariance[i] for i in DIAG]
        t = [msg.twist.covariance[i] for i in DIAG]
        print(f"[t={msg.header.stamp.to_sec():.1f}] pose cov diag:")
        print(f"    x    {fmt(p[0], 'm')}")
        print(f"    y    {fmt(p[1], 'm')}")
        print(f"    yaw  {fmt(p[5], 'rad')}")
        print(f"    (z / roll / pitch var: {p[2]:.1e} / {p[3]:.1e} / {p[4]:.1e})")
        if all(v == 0 for v in t):
            print("  twist cov diag: ALL ZERO "
                  "(wrapper doesn't populate it -> relay uses the pose cov above)")
        else:
            print("  twist cov diag:")
            print(f"    vx   {fmt(t[0], 'm/s')}")
            print(f"    vy   {fmt(t[1], 'm/s')}")
            print(f"    vyaw {fmt(t[5], 'rad/s')}")
        print()
        rate.sleep()


if __name__ == "__main__":
    main()
