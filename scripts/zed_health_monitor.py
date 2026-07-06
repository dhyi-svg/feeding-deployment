#!/usr/bin/env python3
"""ZED VIO health interlock.

The ZED positional tracking can diverge ("Positional tracking has diverged --
Re-initializing odometry"), usually triggered by corrupted frames / low visual
features. When it does, odom→base jumps wildly (measured up to ~4 m/s implied on
a stationary robot) and every downstream consumer -- Cartographer, move_base --
is fed garbage. This node detects that condition and asserts a safety HOLD so the
base is stopped until the ZED recovers, then releases it.

Detection (belt-and-suspenders, any trips a HOLD):
  1. ZED tracking status != OK  (zed_interfaces/PosTrackStatus on odom/pose status
     topics; SEARCHING/OFF mean tracking lost / re-initializing).
  2. Raw odom implies a physically-impossible velocity (the divergence jump),
     independent of the status topic in case it publishes on-change and we miss it.
  3. Odom SILENCE: no odom message for ~silence_hold_s. The other checks are all
     message-driven, so a ZED that dies without saying so was invisible while
     silent (seen twice in real runs: an unrecovered stop, and a 9.5 s outage on
     2026-07-06). This one runs off the tick timer and holds DURING the outage.
  4. map->odom YANK: Cartographer relocalized/aliased (measured 0.75-6.6 m jumps,
     incl. a 180-deg table alias on 2026-07-06 that ran a whole leg on a pose
     ~8 m wrong). Not a ZED fault -- raw odom stays clean -- but the pose is
     untrustworthy and every costmap obstacle mark painted before the jump is
     misplaced by it, so this trips the same hold AND clears the move_base
     costmaps. Settles longer than the ZED channels (~yank_settle_s) because
     pose-graph snaps ping-pong.

Release: only after status is OK *and* odom has been stable (no implied-velocity
spikes) for `recovery_stable_s` continuous seconds (yank channel: ~yank_settle_s),
AND map->odom yaw is quiet (< ~release_mo_rate_rad_s over ~release_rate_window_s).
The last condition exists because channels can settle while Cartographer's
estimate is still visibly rotating under the held, stationary robot (measured
0.07-0.22 rad/s episodes vs a 0.002 rad/s healthy-stationary median) -- releasing
then hands move_base a moving pose and TEB "corrects" a fictitious heading error.
~max_hold_s caps the extension (wander episodes self-terminate in tens of
seconds); a timeout release is logged distinctly.

Output: /nav_safety_hold (std_msgs/Bool), published continuously (latched). The
cmd_vel bridge consumes it and commands zero while held. /nav_safety_hold_reason
(std_msgs/String, latched) carries WHY: every detector channel currently
asserting, joined with " + " (empty string while clear), so consumers and the
run logs can record which condition(s) tripped, not just that one did.
"""

import math
import threading
from collections import deque

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty

try:
    from zed_interfaces.msg import PosTrackStatus
    _HAVE_STATUS = True
except Exception:
    _HAVE_STATUS = False

# Detector channels, in report order: "status" = ZED's own tracking status,
# "silence" = no odom arriving at all (dead/stalled ZED, detected live),
# "gap" = odom stamp gap (stall, detected on resume), "jump" = implied
# velocity (VIO teleport), "yank" = map->odom jump (Cartographer
# relocalization/alias -- not a ZED fault, but pose + costmaps untrustworthy).
CHANNELS = ("status", "silence", "gap", "jump", "yank")


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
        # Liveness: hold if no odom message has ARRIVED for this long (wall
        # clock, evaluated on the tick timer -- works while ZED is silent,
        # unlike the message-driven checks). ~7 missed frames at 15 Hz.
        self.silence_hold_s = rospy.get_param("~silence_hold_s", 0.5)
        # How long status must be OK + odom quiet before releasing the hold.
        self.recovery_stable_s = rospy.get_param("~recovery_stable_s", 2.0)
        # Statuses that force a hold (SEARCHING=0, OFF=2). FPS_TOO_LOW is left out
        # to avoid nuisance holds on transient dips; the odom-jump check still
        # catches an actual divergence.
        self.bad_statuses = set(rospy.get_param("~bad_statuses", [0, 2]))

        # map->odom yank channel (Cartographer relocalization/alias). Gates sit
        # far above normal correction jitter (~0.13 m between 100 ms samples,
        # navlog_20260706_131121) and below the observed events (0.75-6.6 m).
        self.enable_yank = bool(rospy.get_param("~enable_yank_channel", True))
        self.yank_lin = rospy.get_param("~yank_lin_m", 0.5)
        self.yank_ang = rospy.get_param("~yank_ang_rad", 0.4)
        # Pose-graph snaps ping-pong (three within 90 s on 2026-07-06), so the
        # yank channel settles longer than the ZED channels before release.
        self.yank_settle_s = rospy.get_param("~yank_settle_s", 5.0)
        self.clear_srv = rospy.get_param("~clear_costmaps_service",
                                         "/move_base/clear_costmaps")
        self.clear_min_interval_s = rospy.get_param("~clear_min_interval_s", 5.0)

        # Release hardening: gate release on map->odom yaw actually being quiet.
        # Healthy-stationary rate is ~0.002 rad/s median / 0.005 p95; wander
        # episodes run 0.07-0.22 (navlog_20260706_131121) -- 0.03 sits in the
        # empty band with ~6x margin both ways. 98.4% of releases are unaffected.
        self.release_mo_rate = rospy.get_param("~release_mo_rate_rad_s", 0.03)
        self.release_rate_window_s = rospy.get_param("~release_rate_window_s", 3.0)
        self.max_hold_s = rospy.get_param("~max_hold_s", 45.0)

        self.reason_topic = rospy.get_param("~reason_topic",
                                            self.hold_topic + "_reason")

        self.pub = rospy.Publisher(self.hold_topic, Bool, queue_size=1, latch=True)
        self.reason_pub = rospy.Publisher(self.reason_topic, String,
                                          queue_size=1, latch=True)

        self.status_ok = True          # last known tracking status (assume OK until told)
        self.last_status_stamp = None
        self.prev_odom = None          # (stamp, pos, yaw)
        # Arrival wall-time of the last odom msg. Seeded at startup so a ZED
        # that never comes up (or isn't up yet) holds rather than passes.
        self.last_odom_walltime = rospy.Time.now()
        # Per-channel bad state: channel -> (last bad rospy.Time, reason string).
        # Kept per channel (not last-writer-wins) so coincident triggers are all
        # reported, not just whichever callback ran last.
        self.bad = {}
        self.prev_active = set()       # channels active last tick (mid-hold re-log)
        self.hold_reasons = {}         # every channel that fired during this hold
        self.held = False
        self.held_since = None
        self.prev_mo = None            # last (x, y, yaw) sample of map->odom
        self.mo_hist = deque()         # (time, yaw) samples for the release gate
        self.last_clear = None         # rospy.Time of last clear_costmaps call
        if self.enable_yank:
            self.tf_buf = tf2_ros.Buffer()
            tf2_ros.TransformListener(self.tf_buf)

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
        rospy.on_shutdown(lambda: self._publish(False, ""))
        self._publish(False, "")
        rospy.loginfo("zed_health_monitor: publishing %s (gate %.2f m/s / %.2f rad/s, "
                      "recovery %.1fs, status=%s, yank=%s %.2fm/%.2frad settle %.1fs)",
                      self.hold_topic, self.max_lin, self.max_ang,
                      self.recovery_stable_s, _HAVE_STATUS, self.enable_yank,
                      self.yank_lin, self.yank_ang, self.yank_settle_s)

    def cb_status(self, msg):
        # Hold only on the configured bad statuses (SEARCHING/OFF by default) --
        # not on every non-OK value; FPS_TOO_LOW etc. are deliberately excluded
        # (see ~bad_statuses above). Was `!= OK`, which contradicted that.
        self.status_ok = msg.status not in self.bad_statuses
        self.last_status_stamp = rospy.Time.now()
        if not self.status_ok:
            self._mark_bad("status",
                           f"tracking status={msg.status} (SEARCHING=0, OFF=2)")

    def cb_odom(self, msg):
        self.last_odom_walltime = rospy.Time.now()
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        pos = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.prev_odom is not None:
            pt, pp, pyaw = self.prev_odom
            dt = (stamp - pt).to_sec()
            if dt > self.odom_gap_s:
                self._mark_bad("gap",
                               f"odom stamp gap {dt:.2f}s (limit {self.odom_gap_s:.2f}s)"
                               " -- SDK/driver stall signature")
            elif dt > 0.0:
                dist = math.sqrt((pos.x - pp.x) ** 2 + (pos.y - pp.y) ** 2 +
                                 (pos.z - pp.z) ** 2)
                dyaw = abs(angle_diff(yaw, pyaw))
                if dist / dt > self.max_lin or dyaw / dt > self.max_ang:
                    self._mark_bad("jump",
                                   f"implied jump {dist / dt:.2f} m/s / {dyaw / dt:.2f} rad/s"
                                   f" (gates {self.max_lin:.2f}/{self.max_ang:.2f})"
                                   " -- VIO jump signature")
        self.prev_odom = (stamp, pos, yaw)

    def _mark_bad(self, channel, reason):
        self.bad[channel] = (rospy.Time.now(), reason)

    def _settle_s(self, channel):
        return self.yank_settle_s if channel == "yank" else self.recovery_stable_s

    def _check_yank(self, now):
        """Sample map->odom; a big jump = Cartographer relocalized/aliased."""
        try:
            tr = self.tf_buf.lookup_transform("map", "odom", rospy.Time(0))
        except Exception:
            return  # cartographer not up yet / TF momentarily unavailable
        t = tr.transform.translation
        cur = (t.x, t.y, yaw_from_quat(tr.transform.rotation))
        # Sliding yaw window for the release gate (_mo_settled).
        self.mo_hist.append((now, cur[2]))
        while self.mo_hist and (now - self.mo_hist[0][0]).to_sec() > self.release_rate_window_s:
            self.mo_hist.popleft()
        prev, self.prev_mo = self.prev_mo, cur
        if prev is None:
            return  # first sample after startup: seed only
        d = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        dyaw = abs(angle_diff(cur[2], prev[2]))
        if d < self.yank_lin and dyaw < self.yank_ang:
            return
        self._mark_bad("yank",
                       f"map->odom yank {d:.2f} m / {dyaw:.2f} rad"
                       f" (gates {self.yank_lin:.2f}/{self.yank_ang:.2f})"
                       " -- localization relocalized/aliased")
        # Every obstacle mark painted before the jump is now misplaced by it;
        # scrub so both costmaps rebuild from fresh scans (static layer is
        # untouched). Off-thread so a slow/absent service can't stall the tick.
        if (self.last_clear is None or
                (now - self.last_clear).to_sec() > self.clear_min_interval_s):
            self.last_clear = now
            threading.Thread(target=self._clear_costmaps, daemon=True).start()

    def _clear_costmaps(self):
        try:
            rospy.wait_for_service(self.clear_srv, timeout=2.0)
            rospy.ServiceProxy(self.clear_srv, Empty)()
            rospy.logwarn("zed_health_monitor: cleared costmaps after map->odom "
                          "yank (stale obstacle marks)")
        except Exception as e:
            rospy.logwarn("zed_health_monitor: clear_costmaps failed: %s", e)

    def _mo_settled(self, now):
        """True when map->odom yaw is quiet over the sliding window.

        Insufficient or stale TF data counts as settled -- there is nothing to
        judge against, and the ZED channels still gate the release."""
        if not self.enable_yank or len(self.mo_hist) < 5:
            return True
        if (now - self.mo_hist[-1][0]).to_sec() > 1.0:
            return True  # cartographer/TF stopped mid-hold; don't wedge on old data
        span = (self.mo_hist[-1][0] - self.mo_hist[0][0]).to_sec()
        if span <= 0.5:
            return True
        yaws = [y for _, y in self.mo_hist]
        total = sum(abs(angle_diff(b, a)) for a, b in zip(yaws, yaws[1:]))
        return total / span < self.release_mo_rate

    def _publish(self, held, reason):
        self.pub.publish(Bool(data=held))
        self.reason_pub.publish(String(data=reason))

    def tick(self, _):
        now = rospy.Time.now()
        # Liveness (timer-driven, unlike the message-driven callbacks): while
        # odom is silent this re-stamps every tick, so the hold persists for
        # the whole outage and releases via the normal settle window after the
        # stream resumes.
        silent = (now - self.last_odom_walltime).to_sec()
        if silent > self.silence_hold_s:
            self._mark_bad("silence",
                           f"no odom for {silent:.1f}s (limit "
                           f"{self.silence_hold_s:.1f}s) -- ZED down/stalled")
        # map->odom yank (timer-driven, like silence): Cartographer
        # relocalized/aliased -- stop, scrub costmaps, let the pose graph
        # settle; the manager then re-sends the goal from the corrected pose.
        if self.enable_yank:
            self._check_yank(now)
        # Every channel still inside its post-bad settle window keeps the hold
        # asserted, and ALL of them are reported (coincident triggers used to
        # be last-writer-wins in a single bad_reason string).
        active = {ch: reason for ch, (t, reason) in self.bad.items()
                  if (now - t).to_sec() < self._settle_s(ch)}
        # A currently-bad status holds even after its settle window ages out:
        # the status topic may publish on-change only, so a persisting
        # SEARCHING/OFF produces no fresh message to re-stamp the channel.
        if not self.status_ok and "status" not in active:
            active["status"] = self.bad.get(
                "status", (None, "tracking status not OK"))[1]
        should_hold = bool(active)
        reason = " + ".join(active[ch] for ch in CHANNELS if ch in active)

        if should_hold and not self.held:
            self.held = True
            self.held_since = now
            self.hold_reasons = dict(active)
            rospy.logwarn("zed_health_monitor: HOLD asserted [%s] -- stopping "
                          "base until recovery", reason)
        elif should_hold and self.held:
            # Track everything that fires during this hold (latest text per
            # channel) and re-log when the active set changes, so a mid-hold
            # escalation (say status -> status+jump) is not swallowed.
            self.hold_reasons.update(active)
            if set(active) != self.prev_active:
                rospy.logwarn("zed_health_monitor: HOLD now [%s] (%.1fs in)",
                              reason, (now - self.held_since).to_sec())
        elif not should_hold and self.held:
            held_for = (now - self.held_since).to_sec() if self.held_since else 0.0
            settled = self._mo_settled(now)
            if not settled and held_for < self.max_hold_s:
                # All channels have settled, but Cartographer's estimate is
                # still rotating under the stationary, held robot -- releasing
                # now hands move_base a moving pose. Extend until it converges
                # (wander episodes self-terminate) or max_hold_s.
                delay_reason = (f"channels settled but map->odom still moving "
                                f"(> {self.release_mo_rate:.3f} rad/s)")
                rospy.logwarn_throttle(5.0, "zed_health_monitor: delaying HOLD "
                                       "release -- %s (%.1fs in)",
                                       delay_reason, held_for)
                self.prev_active = set(active)
                self._publish(True, delay_reason)
                return
            self.held = False
            fired = " + ".join(self.hold_reasons[ch] for ch in CHANNELS
                               if ch in self.hold_reasons)
            if not settled:
                rospy.logwarn("zed_health_monitor: HOLD released on TIMEOUT "
                              "after %.1fs -- map->odom never settled "
                              "(fired: %s)", held_for, fired)
            else:
                rospy.logwarn("zed_health_monitor: HOLD released after %.1fs -- "
                              "recovered (fired: %s)", held_for, fired)
            self.hold_reasons = {}
        self.prev_active = set(active)
        self._publish(self.held, reason if self.held else "")


def main():
    rospy.init_node("zed_health_monitor")
    ZedHealthMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
