#!/usr/bin/env python3
"""
hold_reason_watch.py -- human-friendly tail of the ZED health monitor.

`rostopic echo /nav_safety_hold_reason` drowns you in empty strings (the
monitor republishes state at 10 Hz). This prints ONLY transitions, wall-clock
timestamped:

    14:02:11.482  HOLD    implied jump 3.21 m/s / ... -- VIO jump signature
    14:02:13.107  CHANGE  implied jump ... + map->odom yank 1.80 m / ...
    14:02:19.964  CLEAR   (held 8.5 s)

Run anywhere the ROS master is reachable:
    rosrun feeding_deployment hold_reason_watch.py
"""

import datetime

import rospy
from std_msgs.msg import String


def stamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


class HoldReasonWatch:
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/nav_safety_hold_reason")
        self.last = None          # None until the first (latched) message
        self.hold_started = None
        rospy.Subscriber(self.topic, String, self.cb, queue_size=10)
        print(f"watching {self.topic} -- transitions only, Ctrl-C to quit")

    def cb(self, msg):
        reason = msg.data.strip()
        if self.last is None:
            # Baseline from the latched message so you know the current state.
            if reason:
                self.hold_started = datetime.datetime.now()
                print(f"{stamp()}  HOLD    {reason}   (already held at startup)",
                      flush=True)
            else:
                print(f"{stamp()}  clear (baseline)", flush=True)
            self.last = reason
            return
        if reason == self.last:
            return  # the 10 Hz republish spam this script exists to hide
        if reason and not self.last:
            self.hold_started = datetime.datetime.now()
            print(f"{stamp()}  HOLD    {reason}", flush=True)
        elif reason and self.last:
            print(f"{stamp()}  CHANGE  {reason}", flush=True)
        else:
            held = ""
            if self.hold_started is not None:
                dt = (datetime.datetime.now() - self.hold_started).total_seconds()
                held = f"   (held {dt:.1f} s)"
                self.hold_started = None
            print(f"{stamp()}  CLEAR{held}", flush=True)
        self.last = reason


def main():
    rospy.init_node("hold_reason_watch", anonymous=True)
    HoldReasonWatch()
    rospy.spin()


if __name__ == "__main__":
    main()
