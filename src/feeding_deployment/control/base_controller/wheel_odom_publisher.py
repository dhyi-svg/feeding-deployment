#!/usr/bin/env python3
"""
wheel_odom_publisher.py

ROS1 node (runs on the compute box) publishing wheel-encoder odometry for the
Vention base. Polls BaseInterface.get_encoders() on the NUC over RPC -- the
Arduino firmware (v7+) streams RoboClaw encoder counts as "E ..." lines and
the NUC caches the latest snapshot -- and integrates a differential-drive
SE(2) pose.

Design notes (see docs/wheel_odom_bringup.md):
- NO TF is broadcast: the ZED owns odom->zed_mini_base_link and Cartographer
  owns map->odom. This node publishes a standalone nav_msgs/Odometry on
  /wheel_odom with frame_id "wheel_odom" (its own dead-reckoning frame,
  origin = wherever the robot was at the first valid sample / last reset).
- Skid steer: 4 fixed grippy wheels scrub in turns, so use an EFFECTIVE track
  width (calibrated, surface-dependent -- commonly 1.3-2x geometric) and treat
  yaw as advisory. Distance / linear velocity is the trustworthy channel; the
  published covariance says so.
- dt comes from FIRMWARE millis deltas, never receipt time: RPC + poll jitter
  (tens of ms on a 100 ms period) would otherwise alias into velocity.
  Dedup compares millis for inequality (not >): millis wraps at ~49.7 days.
- Arduino reset (DTR on reconnect, power cycle) restarts counts and millis at
  zero; integrating across it would emit a huge fake motion. The bridge counts
  resets (banner + millis regressions); on any change, or a local millis
  regression, we re-baseline and drop that interval.
- Sides with ok=0 (RoboClaw read failed / backed off / motor power off) leave
  the baseline untouched; counts are absolute, so the next fully-valid sample
  integrates cleanly across the gap.

Units: counts -> meters via ~counts_per_meter. Expected from the drivetrain:
28 counts/motor-rev x 71.2:1 gearbox = 1993.6 counts/wheel-rev over a 96 mm
wheel (0.30159 m/rev) with 1:1 miter gears => ~6610 counts/m. Calibrate with a
tape-measured straight drive.
"""

import math
import os
import sys
import traceback

import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray


def add_ros_vention_src_to_path():
    try:
        import rospkg
        rp = rospkg.RosPack()
        pkg_path = rp.get_path("feeding_deployment")
        src_path = os.path.join(pkg_path, "src")
        if src_path not in sys.path:
            sys.path.insert(0, src_path)
        return True
    except Exception as e:
        rospy.logwarn(f"Failed to add feeding_deployment/src to PYTHONPATH via rospkg: {e}")
        return False


def _wrap_delta_u32(new: int, old: int) -> int:
    """Signed delta between two uint32 counters, wrap-aware (RoboClaw encoder
    registers are uint32 two's-complement of the signed count)."""
    d = (int(new) - int(old)) & 0xFFFFFFFF
    return d - 0x100000000 if d >= 0x80000000 else d


def _norm_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class WheelOdomPublisher:
    def __init__(self):
        add_ros_vention_src_to_path()

        # ---- params ----
        self.poll_rate_hz = float(rospy.get_param("~poll_rate_hz", 20.0))
        # counts -> meters (see module docstring; calibrate on the robot).
        self.counts_per_meter = float(rospy.get_param("~counts_per_meter", 6610.0))
        # EFFECTIVE track width for skid-steer yaw (calibrate by rotating
        # 2x360 deg in place on the target floor surface). Geometric spacing
        # is a placeholder start value only.
        self.track_width_m = float(rospy.get_param("~track_width_m", 0.55))
        # Expected +1/+1 (closed-loop drive would run away if encoder polarity
        # opposed command polarity), but keep the knobs for the bench check.
        self.side_a_sign = float(rospy.get_param("~side_a_sign", 1.0))  # A = right pair
        self.side_b_sign = float(rospy.get_param("~side_b_sign", 1.0))  # B = left pair
        self.odom_topic = rospy.get_param("~odom_topic", "/wheel_odom")
        self.odom_frame = rospy.get_param("~odom_frame", "wheel_odom")
        self.base_frame = rospy.get_param("~base_frame", "vention_base_link")
        # Front/rear wheels of one side are commanded identically; persistent
        # disagreement means slip or a failing encoder.
        self.disagree_warn_m = float(rospy.get_param("~disagree_warn_m", 0.02))
        # Plausibility gate: a per-side implied speed above this can only be a
        # count glitch or a RoboClaw power-cycle (volatile encoder registers
        # zero on motor-power loss WITHOUT rebooting the Arduino), not real
        # base motion -- re-baseline instead of integrating the jump. Base tops
        # out ~0.6 m/s under teleop; 2.0 leaves generous margin.
        self.max_plausible_speed_mps = float(rospy.get_param("~max_plausible_speed_mps", 2.0))

        # ---- pubs ----
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        # Raw per-motor counts for logging/debug:
        # [millis, a1, a2, b1, b2, ok_a, ok_b, resets] (Float64 is exact for uint32).
        self.counts_pub = rospy.Publisher(self.odom_topic + "/counts",
                                          Float64MultiArray, queue_size=10)
        # Per-tick max |front-rear| distance disagreement within a side (m).
        self.disagree_pub = rospy.Publisher(self.odom_topic + "/side_disagreement",
                                            Float64, queue_size=10)

        # ---- state ----
        self.client = None
        self.baseline = None  # last fully-valid snapshot we integrated from
        # True after any interval we refuse to integrate across (ok=0 gap, or a
        # rejected implausible jump): the next valid sample re-baselines rather
        # than differencing against a pre-gap count, so a RoboClaw power-cycle
        # (counts back to 0) can't teleport the pose.
        self._need_rebaseline = False
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self._connect_client(block=True)
        rospy.Timer(rospy.Duration(1.0 / self.poll_rate_hz), self._tick)
        rospy.loginfo(
            "wheel_odom_publisher: polling get_encoders() at %.0f Hz -> %s "
            "(counts_per_meter=%.1f, track_width_m=%.3f, signs A=%+.0f B=%+.0f)",
            self.poll_rate_hz, self.odom_topic,
            self.counts_per_meter, self.track_width_m,
            self.side_a_sign, self.side_b_sign,
        )

    # ---- RPC plumbing ----
    def _connect_client(self, block: bool = False):
        from feeding_deployment.control.base_controller.base_client import BaseInterfaceClient
        while not rospy.is_shutdown():
            try:
                self.client = BaseInterfaceClient()
                rospy.loginfo("wheel_odom_publisher: connected to base RPC server")
                return
            except Exception:
                self.client = None
                if not block:
                    return
                rospy.logwarn_throttle(
                    10.0, "wheel_odom_publisher: base RPC server unreachable "
                          "(is base_server.py running on the NUC?), retrying...")
                rospy.sleep(2.0)

    def _get_snapshot(self):
        if self.client is None:
            self._connect_client(block=False)
            if self.client is None:
                rospy.logwarn_throttle(10.0, "wheel_odom_publisher: no RPC connection")
                return None
        try:
            return self.client.get_encoders()
        except AttributeError:
            # Server predates get_encoders (old base_server still running).
            rospy.logwarn_throttle(
                10.0, "wheel_odom_publisher: base_server has no get_encoders() "
                      "-- restart it with the updated code")
            return None
        except (EOFError, BrokenPipeError, ConnectionError, OSError):
            rospy.logwarn_throttle(
                5.0, "wheel_odom_publisher: lost RPC connection, reconnecting...")
            self.client = None
            return None
        except Exception:
            rospy.logwarn_throttle(
                5.0, "wheel_odom_publisher: get_encoders failed:\n%s" % traceback.format_exc())
            return None

    # ---- main loop ----
    def _tick(self, _evt):
        snap = self._get_snapshot()
        if snap is None:
            # None also covers stale (>1 s) and v6 firmware (no E lines).
            rospy.logwarn_throttle(
                10.0, "wheel_odom_publisher: no fresh encoder data "
                      "(v6 firmware? serial stalled?)")
            return

        self._publish_counts(snap)

        if not (snap["ok_a"] and snap["ok_b"]):
            # A side failed/backed off (e.g. motor power off). Do NOT trust the
            # next sample to span the gap: on motor-power loss the RoboClaw's
            # volatile encoder counts reset to 0 while the Arduino keeps
            # running, so differencing across the gap would fabricate a huge
            # displacement. Force a re-baseline when valid data returns.
            self._need_rebaseline = True
            rospy.logwarn_throttle(
                10.0, "wheel_odom_publisher: encoder read invalid "
                      "(ok_a=%s ok_b=%s) -- motor power off?" %
                      (snap["ok_a"], snap["ok_b"]))
            return

        base = self.baseline
        if (
            base is None
            or self._need_rebaseline                 # recovering from a gap
            or snap["resets"] != base["resets"]      # Arduino rebooted
            or snap["millis"] < base["millis"]       # belt-and-suspenders
        ):
            if base is not None and snap["resets"] != base["resets"]:
                rospy.logwarn("wheel_odom_publisher: Arduino reset detected "
                              "(resets %s -> %s) -- re-baselining, pose held",
                              base["resets"], snap["resets"])
            self.baseline = snap
            self._need_rebaseline = False
            return
        if snap["millis"] == base["millis"]:
            return  # duplicate poll of the same firmware sample

        dt = (snap["millis"] - base["millis"]) / 1000.0

        # Per-motor deltas (counts) -> per-side mean distance (m).
        da1 = _wrap_delta_u32(snap["a1"], base["a1"])
        da2 = _wrap_delta_u32(snap["a2"], base["a2"])
        db1 = _wrap_delta_u32(snap["b1"], base["b1"])
        db2 = _wrap_delta_u32(snap["b2"], base["b2"])

        d_right = self.side_a_sign * (da1 + da2) / 2.0 / self.counts_per_meter
        d_left = self.side_b_sign * (db1 + db2) / 2.0 / self.counts_per_meter

        # Plausibility gate: a per-side speed beyond physical limits is a count
        # glitch or a brownout-without-ok=0-gap RoboClaw reset -- drop this
        # interval and re-baseline rather than teleport the pose.
        if dt > 0 and max(abs(d_right), abs(d_left)) / dt > self.max_plausible_speed_mps:
            rospy.logwarn(
                "wheel_odom_publisher: implausible per-side speed "
                "(R=%.1f L=%.1f m/s over %.3f s) -- likely encoder reset, "
                "re-baselining, pose held", d_right / dt, d_left / dt, dt)
            self.baseline = snap
            return

        self.baseline = snap

        disagree = max(abs(da1 - da2), abs(db1 - db2)) / self.counts_per_meter
        self.disagree_pub.publish(Float64(disagree))
        if disagree > self.disagree_warn_m:
            rospy.logwarn_throttle(
                5.0, "wheel_odom_publisher: front/rear encoder disagreement "
                     "%.3f m in one tick (slip or failing encoder?)" % disagree)

        ds = 0.5 * (d_right + d_left)
        dyaw = (d_right - d_left) / self.track_width_m

        # Midpoint (arc) integration.
        mid_yaw = self.yaw + 0.5 * dyaw
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw = _norm_angle(self.yaw + dyaw)

        self._publish_odom(ds / dt, dyaw / dt, snap.get("age_s", 0.0))

    # ---- publishing ----
    def _publish_counts(self, snap):
        msg = Float64MultiArray()
        msg.data = [
            float(snap["millis"]),
            float(snap["a1"]), float(snap["a2"]),
            float(snap["b1"]), float(snap["b2"]),
            1.0 if snap["ok_a"] else 0.0,
            1.0 if snap["ok_b"] else 0.0,
            float(snap["resets"]),
        ]
        self.counts_pub.publish(msg)

    def _publish_odom(self, vx: float, wz: float, age_s: float = 0.0):
        odom = Odometry()
        # Stamp at the MEASUREMENT time, not receipt: the E-line sample is
        # age_s old (NUC-measured, ~50-100 ms typical) by the time it reaches
        # us over RPC. age_s is a pure duration on the NUC clock, so
        # subtracting it introduces no cross-machine clock skew. Correct stamps
        # matter for time-aligning /wheel_odom against ZED odom (slip
        # detection, calibration, fusion).
        odom.header.stamp = rospy.Time.now() - rospy.Duration.from_sec(max(0.0, age_s))
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = Quaternion(
            0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))

        # Explicitly non-zero covariance (all-zeros reads as "perfect" to
        # consumers like robot_localization). x/y modest; yaw HIGH: skid-steer
        # yaw through an effective track width is advisory only. z/roll/pitch
        # unobservable -> huge.
        HUGE = 1e6
        odom.pose.covariance = [
            1e-3, 0, 0, 0, 0, 0,
            0, 1e-3, 0, 0, 0, 0,
            0, 0, HUGE, 0, 0, 0,
            0, 0, 0, HUGE, 0, 0,
            0, 0, 0, 0, HUGE, 0,
            0, 0, 0, 0, 0, 0.5,
        ]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = [
            1e-4, 0, 0, 0, 0, 0,
            0, HUGE, 0, 0, 0, 0,
            0, 0, HUGE, 0, 0, 0,
            0, 0, 0, HUGE, 0, 0,
            0, 0, 0, 0, HUGE, 0,
            0, 0, 0, 0, 0, 0.1,
        ]
        self.odom_pub.publish(odom)


def main():
    rospy.init_node("wheel_odom_publisher", anonymous=False)
    _ = WheelOdomPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
