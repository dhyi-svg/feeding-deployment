#!/usr/bin/env python3
"""
drift_lock.py -- press-ENTER anchor lock for the ZED drift test.

Run this in your own terminal alongside `roslaunch feeding_deployment
zed_drift_test.launch` (roslaunch doesn't hand nodes an interactive stdin,
hence a separate foreground tool):

  1. It prints the live Cartographer pose (map->vention_base_link) twice a
     second while you let localization settle / nudge the base until the laser
     scan hugs the map walls in RViz.
  2. Press ENTER -> calls /drift_test/lock: the anchor freezes, the four
     traces start from that map point.
  3. It keeps running: press ENTER again anytime to RE-lock (clears traces,
     fresh anchor -- e.g. after a ZED restart or to start a new experiment).
     Ctrl-C to exit.
"""

import math
import sys
import threading

import rospy
import tf2_ros
from std_srvs.srv import Trigger

POSE_PERIOD = 0.5


def main():
    rospy.init_node("drift_lock", anonymous=True, disable_signals=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    stop = threading.Event()
    locked_once = [False]

    def pose_printer():
        while not stop.is_set() and not rospy.is_shutdown():
            try:
                tr = buf.lookup_transform("map", "vention_base_link",
                                          rospy.Time(0), rospy.Duration(0.4))
                t = tr.transform.translation
                q = tr.transform.rotation
                yaw = math.degrees(math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
                state = "RE-lock available" if locked_once[0] else "still localizing"
                sys.stdout.write(
                    f"\r  map pose: x={t.x:+7.3f}  y={t.y:+7.3f}  "
                    f"yaw={yaw:+7.1f} deg   [{state} -- ENTER to lock]   ")
                sys.stdout.flush()
            except Exception:
                sys.stdout.write(
                    "\r  waiting for map->vention_base_link TF "
                    "(cartographer + sensors up?)                    ")
                sys.stdout.flush()
            stop.wait(POSE_PERIOD)

    print(__doc__)
    print("Waiting for the /drift_test/lock service...")
    try:
        rospy.wait_for_service("/drift_test/lock", timeout=30.0)
    except rospy.ROSException:
        print("ERROR: /drift_test/lock not up -- is zed_drift_test.launch running?")
        return 1
    lock_srv = rospy.ServiceProxy("/drift_test/lock", Trigger)

    th = threading.Thread(target=pose_printer, daemon=True)
    th.start()
    try:
        while not rospy.is_shutdown():
            input()  # ENTER
            try:
                resp = lock_srv()
            except rospy.ServiceException as e:
                # Tracer down / launch being restarted: keep the helper alive
                # (the docstring promises re-lock works across restarts).
                print(f"\n  -> FAILED: /drift_test/lock unreachable ({e}). "
                      "Is zed_drift_test.launch running? Press ENTER to retry.\n")
                continue
            print(f"\n  -> {'LOCKED' if resp.success else 'FAILED'}: {resp.message}\n")
            if resp.success:
                locked_once[0] = True
    except (KeyboardInterrupt, EOFError):
        print("\nbye")
    finally:
        stop.set()
    return 0


if __name__ == "__main__":
    sys.exit(main())
