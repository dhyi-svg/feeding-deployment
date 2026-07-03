#!/usr/bin/env python3
"""ZED VIO health interlock.

The ZED positional tracking can diverge ("Positional tracking has diverged --
Re-initializing odometry"), usually triggered by corrupted frames / low visual
features. When it does, odom→base jumps wildly (measured up to ~4 m/s implied on
a stationary robot) and every downstream consumer -- Cartographer, move_base --
is fed garbage. This node detects that condition and asserts a safety HOLD so the
base is stopped until the ZED recovers, then releases it.

Detection (belt-and-suspenders, either trips a HOLD):
  1. ZED tracking status != OK  (zed_interfaces/PosTrackStatus on odom/pose status
     topics; SEARCHING/OFF mean tracking lost / re-initializing).
  2. Raw odom implies a physically-impossible velocity (the divergence jump),
     independent of the status topic in case it publishes on-change and we miss it.

Release: only after status is OK *and* odom has been stable (no implied-velocity
spikes) for `recovery_stable_s` continuous seconds, so we don't resume mid re-init.

Output: /nav_safety_hold (std_msgs/Bool), published continuously (latched). The
cmd_vel bridge consumes it and commands zero while held.
"""

import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

try:
    from zed_interfaces.msg import PosTrackStatus
    _HAVE_STATUS = True
except Exception:
    _HAVE_STATUS = False

OK = 1  # PosTrackStatus.OK; anything else = not cleanly tracking


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


class ZedHealthMonitor:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/zed_mini/zed_node/odom")
        self.hold_topic = rospy.get_param("~hold_topic", "/nav_safety_hold")
        # Same physical-plausibility gate as the sanitizer; a divergence blows past it.
        self.max_lin = rospy.get_param("~max_lin_vel", 0.5)
        self.max_ang = rospy.get_param("~max_ang_vel", 1.5)
        self.odom_gap_s = rospy.get_param("~odom_gap_s", 0.3)
        # How long status must be OK + odom quiet before releasing the hold.
        self.recovery_stable_s = rospy.get_param("~recovery_stable_s", 2.0)
        # Statuses that force a hold (SEARCHING=0, OFF=2). FPS_TOO_LOW is left out
        # to avoid nuisance holds on transient dips; the odom-jump check still
        # catches an actual divergence.
        self.bad_statuses = set(rospy.get_param("~bad_statuses", [0, 2]))

        self.pub = rospy.Publisher(self.hold_topic, Bool, queue_size=1, latch=True)

        self.status_ok = True          # last known tracking status (assume OK until told)
        self.last_status_stamp = None
        self.prev_odom = None          # (stamp, pos, yaw)
        self.last_bad_time = None      # last time anything looked wrong
        self.bad_reason = "unknown"    # WHICH check tripped, for the HOLD log line
        self.held = False
        self.held_since = None

        rospy.Subscriber(self.odom_topic, Odometry, self.cb_odom, queue_size=50)
        if _HAVE_STATUS:
            rospy.Subscriber("/zed_mini/zed_node/odom/status", PosTrackStatus,
                             self.cb_status, queue_size=5)
            rospy.Subscriber("/zed_mini/zed_node/pose/status", PosTrackStatus,
                             self.cb_status, queue_size=5)
        else:
            rospy.logwarn("zed_health_monitor: zed_interfaces not importable; "
                          "relying on odom-jump detection only")

        # Publish/evaluate the hold at a steady rate so a late-starting bridge and
        # a stale-monitor fail-safe both work.
        rospy.Timer(rospy.Duration(0.1), self.tick)
        rospy.on_shutdown(lambda: self.pub.publish(Bool(data=False)))
        self.pub.publish(Bool(data=False))
        rospy.loginfo("zed_health_monitor: publishing %s (gate %.2f m/s / %.2f rad/s, "
                      "recovery %.1fs, status=%s)", self.hold_topic, self.max_lin,
                      self.max_ang, self.recovery_stable_s, _HAVE_STATUS)

    def cb_status(self, msg):
        # Hold only on the configured bad statuses (SEARCHING/OFF by default) --
        # not on every non-OK value; FPS_TOO_LOW etc. are deliberately excluded
        # (see ~bad_statuses above). Was `!= OK`, which contradicted that.
        self.status_ok = msg.status not in self.bad_statuses
        self.last_status_stamp = rospy.Time.now()
        if not self.status_ok:
            self.last_bad_time = rospy.Time.now()
            self.bad_reason = (
                f"tracking status={msg.status} (SEARCHING=0, OFF=2)"
            )

    def cb_odom(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        pos = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.prev_odom is not None:
            pt, pp, pyaw = self.prev_odom
            dt = (stamp - pt).to_sec()
            if dt > self.odom_gap_s:
                self.last_bad_time = rospy.Time.now()
                self.bad_reason = (
                    f"odom stamp gap {dt:.2f}s (limit {self.odom_gap_s:.2f}s)"
                    " -- SDK/driver stall signature"
                )
            elif dt > 0.0:
                dist = math.sqrt((pos.x - pp.x) ** 2 + (pos.y - pp.y) ** 2 +
                                 (pos.z - pp.z) ** 2)
                dyaw = abs(angle_diff(yaw, pyaw))
                if dist / dt > self.max_lin or dyaw / dt > self.max_ang:
                    self.last_bad_time = rospy.Time.now()
                    self.bad_reason = (
                        f"implied jump {dist / dt:.2f} m/s / {dyaw / dt:.2f} rad/s"
                        f" (gates {self.max_lin:.2f}/{self.max_ang:.2f})"
                        " -- VIO jump signature"
                    )
        self.prev_odom = (stamp, pos, yaw)

    def tick(self, _):
        now = rospy.Time.now()
        recent_bad = (self.last_bad_time is not None and
                      (now - self.last_bad_time).to_sec() < self.recovery_stable_s)
        # Hold while tracking is bad, or while we're still inside the post-bad
        # settle window, or if the status is currently not OK.
        should_hold = recent_bad or (not self.status_ok)

        if should_hold and not self.held:
            self.held = True
            self.held_since = now
            rospy.logwarn("zed_health_monitor: HOLD asserted [%s] -- stopping "
                          "base until recovery", self.bad_reason)
        elif not should_hold and self.held:
            self.held = False
            dur = (now - self.held_since).to_sec() if self.held_since else -1.0
            rospy.logwarn("zed_health_monitor: HOLD released after %.1fs -- "
                          "ZED recovered (was: %s)", dur, self.bad_reason)
        self.pub.publish(Bool(data=self.held))


def main():
    rospy.init_node("zed_health_monitor")
    ZedHealthMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
