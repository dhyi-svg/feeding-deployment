#!/usr/bin/env python3
"""
gyro_bias_estimator.py -- stillness-calibrated debiasing of the ZED IMU gyro.

The ZED Mini's gyro carries a large but SLOWLY-WANDERING z bias (measured
2026-07-10: -0.036 deg/s ~ -131 deg/h, wandering -0.033 -> -0.041 deg/s over a
14-min run). robot_localization does NOT estimate IMU bias, so that bias
integrates straight into fused yaw: the jul10 trace run ended -34 deg / 0.57 m
off over 16 m, of which ~-31 deg was this one constant. Counterfactual replay
of that bag with the bias subtracted: -6 deg / 0.44 m.

This node republishes the IMU with the gyro bias subtracted:

  /zed_mini/zed_node/imu/data  ->  /zed_mini/zed_node/imu/data_debiased

Bias estimation (all three gyro axes, z is the one that matters in 2D):
  - Stillness = |wheel vx| < vx_still AND |gyro z| < wz_still, continuously for
    still_window_s, both streams fresh -- the same dual criterion as
    zupt_publisher.py (wheels can differential-cancel during an in-place spin;
    the gyro catches that; wz_still is ~15x any plausible bias, so a biased
    gyro still passes when truly still).
  - While confirmed still, the estimate tracks the gyro mean with an EMA
    (time constant bias_tau_s of ACCUMULATED still time), so it follows the
    thermal wander across a run. While moving or stale it freezes.
  - Every incoming Imu is republished immediately with the CURRENT estimate
    subtracted (zero added latency; passthrough with bias 0 until the first
    stillness -- in practice bringup idle time calibrates it before the first
    motion, which matters: an uncalibrated first 2 min costs ~5 deg).

Diagnostics: /gyro_bias_estimate (Vector3Stamped, deg/s) at ~1 Hz + a one-time
log line when the first calibration completes.

Consumer: the LIVE ekf_fused_imu_wheel (sensors.launch) -- the authoritative
odom->base source since Jul 2026 -- reads the debiased topic. Calibration
quality directly affects localization; wait for the "first calibration
complete" log before driving.
"""

import math
import threading

import rospy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class BiasCore:
    """Pure estimator logic (no ROS) so it can be replay-tested offline.

    Feed every gyro sample via update(); wheel stillness via set_wheel().
    bias[] is the current (x, y, z) estimate in rad/s.
    """

    def __init__(self, vx_still=0.01, wz_still=0.01, still_window_s=1.0,
                 fresh_timeout_s=0.3, bias_tau_s=30.0, cal_min_s=10.0):
        self.vx_still = vx_still
        self.wz_still = wz_still
        self.still_window_s = still_window_s
        self.fresh_timeout_s = fresh_timeout_s
        self.bias_tau_s = bias_tau_s
        self.cal_min_s = cal_min_s        # still time before 'calibrated'
        self.bias = [0.0, 0.0, 0.0]
        self.calibrated = False          # True once one full window absorbed
        self.still_accum_s = 0.0         # accumulated still time (diagnostic)
        self._last_wheel = None          # (t, vx)
        self._still_since = None
        self._last_t = None

    def set_wheel(self, t, vx):
        self._last_wheel = (t, vx)

    def _wheel_still(self, t):
        if self._last_wheel is None:
            return False
        wt, vx = self._last_wheel
        return (t - wt) < self.fresh_timeout_s and abs(vx) < self.vx_still

    def update(self, t, gx, gy, gz):
        """One gyro sample. Returns True if this sample updated the bias."""
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        # Gyro-side stillness uses the RAW reading: threshold >> any plausible
        # bias, so this never wedges (a biased-but-still gyro passes).
        if not (self._wheel_still(t) and abs(gz) < self.wz_still):
            self._still_since = None
            return False
        if self._still_since is None:
            self._still_since = t
        if (t - self._still_since) < self.still_window_s:
            return False                  # not settled long enough yet
        if dt <= 0.0 or dt > self.fresh_timeout_s:
            return False                  # stream gap: don't jump the EMA
        # Running mean over the first bias_tau_s of still time (exact, fast
        # cold-start convergence), then an EMA with that tau so the estimate
        # keeps tracking the slow thermal wander.
        self.still_accum_s += dt
        alpha = min(1.0, dt / min(self.still_accum_s, self.bias_tau_s))
        for i, g in enumerate((gx, gy, gz)):
            self.bias[i] += alpha * (g - self.bias[i])
        if not self.calibrated and self.still_accum_s >= self.cal_min_s:
            self.calibrated = True
        return True


class GyroBiasEstimator:
    def __init__(self):
        imu_topic = rospy.get_param("~imu_topic", "/zed_mini/zed_node/imu/data")
        out_topic = rospy.get_param(
            "~output_topic", "/zed_mini/zed_node/imu/data_debiased")
        wheel_topic = rospy.get_param("~wheel_odom_topic", "/wheel_odom")
        self.core = BiasCore(
            vx_still=float(rospy.get_param("~vx_still", 0.01)),
            wz_still=float(rospy.get_param("~wz_still", 0.01)),
            still_window_s=float(rospy.get_param("~still_window_s", 1.0)),
            fresh_timeout_s=float(rospy.get_param("~fresh_timeout_s", 0.3)),
            bias_tau_s=float(rospy.get_param("~bias_tau_s", 30.0)))
        self._lock = threading.Lock()
        self._announced = False

        self.pub = rospy.Publisher(out_topic, Imu, queue_size=50)
        self.bias_pub = rospy.Publisher(
            "/gyro_bias_estimate", Vector3Stamped, queue_size=2)
        rospy.Subscriber(wheel_topic, Odometry, self._cb_wheel, queue_size=20)
        rospy.Subscriber(imu_topic, Imu, self._cb_imu, queue_size=100)
        rospy.Timer(rospy.Duration(1.0), self._tick_diag)
        rospy.loginfo("gyro_bias_estimator: %s -> %s (wheel stillness from %s, "
                      "EMA tau %.0fs)", imu_topic, out_topic, wheel_topic,
                      self.core.bias_tau_s)

    def _cb_wheel(self, msg):
        with self._lock:
            self.core.set_wheel(rospy.Time.now().to_sec(),
                                msg.twist.twist.linear.x)

    def _cb_imu(self, msg):
        av = msg.angular_velocity
        with self._lock:
            self.core.update(rospy.Time.now().to_sec(), av.x, av.y, av.z)
            bx, by, bz = self.core.bias
            if self.core.calibrated and not self._announced:
                self._announced = True
                rospy.loginfo(
                    "gyro_bias_estimator: first calibration complete -- "
                    "bias z = %+.4f deg/s (%+.1f deg/h)",
                    math.degrees(bz), math.degrees(bz) * 3600.0)
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = av.x - bx
        out.angular_velocity.y = av.y - by
        out.angular_velocity.z = av.z - bz
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        self.pub.publish(out)

    def _tick_diag(self, _evt):
        with self._lock:
            bx, by, bz = self.core.bias
        v = Vector3Stamped()
        v.header.stamp = rospy.Time.now()
        v.vector.x = math.degrees(bx)
        v.vector.y = math.degrees(by)
        v.vector.z = math.degrees(bz)
        self.bias_pub.publish(v)


def main():
    rospy.init_node("gyro_bias_estimator")
    GyroBiasEstimator()
    rospy.spin()


if __name__ == "__main__":
    main()
