#!/usr/bin/env python3
"""
drift_trace_compare.py

Map-anchored live comparison of independent motion witnesses, for localizing
WHERE the ZED VIO misbehaves in the home. Four traces, all in `map`:

  /drift_test/carto_path          green   live map->vention_base_link (lidar-
                                          corrected reference, closed loop)
  /drift_test/zed_path            red     RAW ZED odom, OPEN LOOP after lock
  /drift_test/zed_sanitized_path  orange  sanitized ZED odom, open loop
  /drift_test/wheel_path          blue    /wheel_odom, open loop

Nothing is published until the anchor is LOCKED via the std_srvs/Trigger
service /drift_test/lock (use scripts/drift_lock.py for press-Enter UX; call
again anytime to re-lock and clear the traces). Until then Cartographer keeps
relocalizing freely.

At lock we latch M0 = map->odom, T0 = map->vention_base_link and W0 = the
current wheel-odom pose. Open-loop samples are then baked into map AT CAPTURE
TIME:

    ZED (raw/sanitized):  P(t) = M0 . Z(t) . E
    wheel:                P(t) = T0 . W0^-1 . W(t)

where Z(t) is the odom->zed_mini_base_link pose in the ZED message and E is
the static zed_mini_base_link->vention_base_link URDF transform. Baking with
the FROZEN M0 is the point: a post-lock Cartographer yank must not
retroactively bend the history of an open-loop trace (RViz would re-transform
an odom-frame Path with the live map->odom every frame).

Reading the picture: red/orange peeling away from green = VIO drift, located
on the map; blue peeling from green = wheel slip / execution error; red vs
blue disagreement is arbitrated by green.
"""

import math
import threading

import numpy as np
import rospy
import tf2_ros
from tf import transformations as tft

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_srvs.srv import Trigger, TriggerResponse


def pose_to_mat(pose):
    """geometry_msgs/Pose -> 4x4 homogeneous matrix."""
    q = pose.orientation
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    m[0, 3] = pose.position.x
    m[1, 3] = pose.position.y
    m[2, 3] = pose.position.z
    return m


def transform_to_mat(tr):
    """geometry_msgs/Transform -> 4x4 homogeneous matrix."""
    q = tr.rotation
    m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
    m[0, 3] = tr.translation.x
    m[1, 3] = tr.translation.y
    m[2, 3] = tr.translation.z
    return m


def mat_to_pose_stamped(m, stamp, frame_id="map"):
    ps = PoseStamped()
    ps.header.stamp = stamp
    ps.header.frame_id = frame_id
    q = tft.quaternion_from_matrix(m)
    ps.pose.position.x = m[0, 3]
    ps.pose.position.y = m[1, 3]
    ps.pose.position.z = m[2, 3]
    ps.pose.orientation.x = q[0]
    ps.pose.orientation.y = q[1]
    ps.pose.orientation.z = q[2]
    ps.pose.orientation.w = q[3]
    return ps


def mat_xy_yaw(m):
    return m[0, 3], m[1, 3], math.atan2(m[1, 0], m[0, 0])


class Trace:
    """A decimated, capped list of map-frame PoseStamped."""

    def __init__(self, min_step_m, min_step_rad, max_poses):
        self.min_step_m = min_step_m
        self.min_step_rad = min_step_rad
        self.max_poses = max_poses
        self.poses = []

    def clear(self):
        self.poses = []

    def append(self, mat, stamp):
        if self.poses:
            lx, ly, lyaw = self._last_xyyaw
            x, y, yaw = mat_xy_yaw(mat)
            if (math.hypot(x - lx, y - ly) < self.min_step_m
                    and abs(math.atan2(math.sin(yaw - lyaw),
                                       math.cos(yaw - lyaw))) < self.min_step_rad):
                return
        self.poses.append(mat_to_pose_stamped(mat, stamp))
        self._last_xyyaw = mat_xy_yaw(mat)
        if len(self.poses) > self.max_poses:
            # Thin the older half (keep endpoints dense where it matters: now).
            half = len(self.poses) // 2
            self.poses = self.poses[:half:2] + self.poses[half:]


class DriftTraceCompare:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "vention_base_link")
        self.zed_frame = rospy.get_param("~zed_frame", "zed_mini_base_link")
        self.raw_topic = rospy.get_param("~zed_raw_topic", "/zed_mini/zed_node/odom")
        self.san_topic = rospy.get_param(
            "~zed_sanitized_topic", "/zed_mini/zed_node/odom_sanitized")
        self.wheel_topic = rospy.get_param("~wheel_odom_topic", "/wheel_odom")
        min_step_m = float(rospy.get_param("~min_step_m", 0.02))
        min_step_rad = float(rospy.get_param("~min_step_rad", 0.05))
        max_poses = int(rospy.get_param("~max_poses", 4000))
        self.carto_rate = float(rospy.get_param("~carto_sample_hz", 10.0))
        self.path_pub_rate = float(rospy.get_param("~path_pub_hz", 3.0))
        # A single-sample wheel jump beyond this is a wheel_odom NODE restart
        # (integrated pose re-zeroed): freeze the wheel trace, advise re-lock.
        # (An Arduino/RoboClaw reset does NOT do this -- the publisher holds
        # pose across those; only a node restart re-zeroes.)
        self.wheel_jump_m = float(rospy.get_param("~wheel_jump_m", 1.0))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.lock = threading.Lock()
        self.locked = False
        self.M0 = None            # map->odom at lock
        self.T0 = None            # map->base at lock (published anchor)
        self.W0_inv = None        # inverse of wheel pose at wheel-anchor latch
        self.wheel_T0 = None      # map->base at wheel-anchor latch (see _cb_wheel)
        self.E = None             # zed_mini_base_link->vention_base_link (static)
        self.last_wheel_mat = None    # latest wheel_odom pose (pre-lock cache)
        self.last_wheel_baked = None  # last baked wheel point (jump detection)
        self.wheel_frozen = False

        self.traces = {
            "carto": Trace(min_step_m, min_step_rad, max_poses),
            "zed": Trace(min_step_m, min_step_rad, max_poses),
            "zed_sanitized": Trace(min_step_m, min_step_rad, max_poses),
            "wheel": Trace(min_step_m, min_step_rad, max_poses),
        }
        self.path_pubs = {
            "carto": rospy.Publisher("/drift_test/carto_path", Path, queue_size=2),
            "zed": rospy.Publisher("/drift_test/zed_path", Path, queue_size=2),
            "zed_sanitized": rospy.Publisher("/drift_test/zed_sanitized_path",
                                             Path, queue_size=2),
            "wheel": rospy.Publisher("/drift_test/wheel_path", Path, queue_size=2),
        }
        self.anchor_pub = rospy.Publisher("/drift_test/anchor", PoseStamped,
                                          queue_size=1, latch=True)

        rospy.Subscriber(self.raw_topic, Odometry, self._cb_zed_raw, queue_size=20)
        rospy.Subscriber(self.san_topic, Odometry, self._cb_zed_san, queue_size=20)
        rospy.Subscriber(self.wheel_topic, Odometry, self._cb_wheel, queue_size=20)
        rospy.Service("/drift_test/lock", Trigger, self._srv_lock)

        rospy.Timer(rospy.Duration(1.0 / self.carto_rate), self._tick_carto)
        rospy.Timer(rospy.Duration(1.0 / self.path_pub_rate), self._tick_publish)
        rospy.loginfo("drift_trace_compare: waiting for /drift_test/lock "
                      "(run drift_lock.py and press ENTER when localized). "
                      "No traces are drawn until then.")

    # ---- static transform ----
    def _get_E(self):
        """zed_mini_base_link->vention_base_link, looked up once."""
        if self.E is not None:
            return self.E
        try:
            tr = self.tf_buffer.lookup_transform(
                self.zed_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.5))
            self.E = transform_to_mat(tr.transform)
            return self.E
        except Exception:
            return None

    # ---- lock service ----
    def _srv_lock(self, _req):
        try:
            m0 = transform_to_mat(self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rospy.Time(0),
                rospy.Duration(1.0)).transform)
            t0 = transform_to_mat(self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0),
                rospy.Duration(1.0)).transform)
        except Exception as e:
            return TriggerResponse(success=False,
                                   message=f"TF not available (cartographer up?): {e}")
        if self._get_E() is None:
            return TriggerResponse(success=False,
                                   message="static zed->base TF missing (sensors up?)")
        with self.lock:
            if self.last_wheel_mat is None:
                return TriggerResponse(
                    success=False,
                    message="no /wheel_odom yet (wheel_odom_publisher up? "
                            "base_server on the NUC?)")
            self.M0 = m0
            self.T0 = t0
            # Wheel anchor is deliberately NOT latched here: the cached wheel
            # sample can be ~100-200 ms staler than the TF snapshot (10 Hz +
            # RPC age), which would bake a constant offset into the whole blue
            # trace if the operator locks while moving. Instead the first
            # post-lock wheel message latches (fresh TF, fresh wheel pose) at
            # the same instant -- see _cb_wheel.
            self.W0_inv = None
            self.wheel_T0 = None
            for t in self.traces.values():
                t.clear()
            self.wheel_frozen = False
            self.last_wheel_baked = None
            self.locked = True
        stamp = rospy.Time.now()
        self.anchor_pub.publish(mat_to_pose_stamped(t0, stamp))
        x, y, yaw = mat_xy_yaw(t0)
        msg = f"anchor locked at x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg"
        rospy.loginfo("drift_trace_compare: %s", msg)
        return TriggerResponse(success=True, message=msg)

    # ---- subscribers ----
    def _cb_zed_raw(self, msg):
        self._bake_zed(msg, "zed")

    def _cb_zed_san(self, msg):
        self._bake_zed(msg, "zed_sanitized")

    def _bake_zed(self, msg, key):
        E = self._get_E()
        with self.lock:
            if not self.locked or E is None:
                return
            z = pose_to_mat(msg.pose.pose)
            baked = self.M0 @ z @ E
            self.traces[key].append(baked, msg.header.stamp)

    def _cb_wheel(self, msg):
        w = pose_to_mat(msg.pose.pose)
        with self.lock:
            self.last_wheel_mat = w
            if not self.locked or self.wheel_frozen:
                return
            if self.W0_inv is None:
                # First post-lock wheel message: latch the wheel anchor NOW,
                # pairing this wheel pose with the CURRENT carto pose. Both
                # are fresh, so no stale-sample bias; the first baked point is
                # the anchor itself, so the jump check below is meaningful
                # from the second sample onward.
                try:
                    tr = self.tf_buffer.lookup_transform(
                        self.map_frame, self.base_frame, rospy.Time(0))
                except Exception:
                    return  # TF hiccup; try again on the next wheel message
                self.wheel_T0 = transform_to_mat(tr.transform)
                self.W0_inv = np.linalg.inv(w)
            baked = self.wheel_T0 @ self.W0_inv @ w
            if self.last_wheel_baked is not None:
                px, py, _ = mat_xy_yaw(self.last_wheel_baked)
                x, y, _ = mat_xy_yaw(baked)
                if math.hypot(x - px, y - py) > self.wheel_jump_m:
                    self.wheel_frozen = True
                    rospy.logerr(
                        "drift_trace_compare: wheel trace jumped %.1f m in one "
                        "sample -- wheel_odom_publisher restarted (pose "
                        "re-zeroed)? Wheel trace FROZEN; re-lock "
                        "(/drift_test/lock) to resume.",
                        math.hypot(x - px, y - py))
                    return
            self.last_wheel_baked = baked
            self.traces["wheel"].append(baked, msg.header.stamp)

    # ---- timers ----
    def _tick_carto(self, _evt):
        with self.lock:
            if not self.locked:
                return
        try:
            tr = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0))
        except Exception:
            return
        m = transform_to_mat(tr.transform)
        with self.lock:
            if self.locked:
                self.traces["carto"].append(m, tr.header.stamp)

    def _tick_publish(self, _evt):
        with self.lock:
            if not self.locked:
                return
            snap = {k: list(t.poses) for k, t in self.traces.items()}
        stamp = rospy.Time.now()
        for k, poses in snap.items():
            path = Path()
            path.header.stamp = stamp
            path.header.frame_id = self.map_frame
            path.poses = poses
            self.path_pubs[k].publish(path)


def main():
    rospy.init_node("drift_trace_compare", anonymous=False)
    _ = DriftTraceCompare()
    rospy.spin()


if __name__ == "__main__":
    main()
