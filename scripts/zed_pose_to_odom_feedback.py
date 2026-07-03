#!/usr/bin/env python3

import math
from collections import deque

import rospy
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
	siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
	cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
	return math.atan2(siny_cosp, cosy_cosp)


def angle_diff(a, b):
	return math.atan2(math.sin(a - b), math.cos(a - b))


class OdomSanitizerDifferentiator:
	"""Two jobs on the ZED odom stream:

	1) SANITIZE the pose: drop single-frame VIO teleports (the fridge->microwave
	   failure was one frame of 6.74 m / ~101 m/s that corrupted Cartographer's
	   extrapolator for 86 s). Republishes a clean full-pose Odometry that
	   Cartographer consumes (point its `odom` remap at ~sanitized_odom_topic).
	   A physically-impossible frame-to-frame jump is HELD at the last good pose;
	   a *sustained* new pose (genuine ZED relocalization/restart) is adopted
	   after ~jump_accept_frames consistent raw frames so we never reject forever.

	2) DIFFERENTIATE the (sanitized) pose over a fixed time window into a twist
	   for move_base's /odom_feedback (windowed to keep velocity noise low at the
	   full ZED rate). Feeding it sanitized poses also keeps a teleport from
	   spiking TEB's initial-velocity estimate.

	NOTE: this owns neither the odom->base_link TF nor the raw topic -- the ZED
	still publishes both, so the raw topic remains available to diagnostics and a
	teleport still causes a transient one-frame TF blip (self-healing, unlike the
	measurement-path corruption this prevents).
	"""

	def __init__(self):
		self.input_topic = rospy.get_param("~input_odom_topic", "/zed_mini/zed_node/odom")
		self.output_topic = rospy.get_param("~output_odom_topic", "/move_base/odom_feedback")
		# Sanitized full-pose odom for Cartographer (repoint its `odom` remap here).
		self.sanitized_topic = rospy.get_param(
			"~sanitized_odom_topic", "/zed_mini/zed_node/odom_sanitized")

		# Ablation switches (config/nav/odom_pipeline.yaml). Either false ==
		# original behavior for that stage.
		self.enable_sanitizer = bool(rospy.get_param("~enable_sanitizer", True))
		self.enable_windowed_diff = bool(rospy.get_param("~enable_windowed_diff", True))

		# Windowed differentiation: velocity noise = pose jitter / dt. Differencing
		# over a fixed window (not consecutive frames) keeps the noise floor low
		# regardless of ZED rate.
		self.diff_window = rospy.get_param("~vel_diff_window", 0.08)

		# Physical-plausibility gate. Robot caps are 0.2 m/s / 0.5 rad/s; these
		# ~2.5x thresholds sit far above real motion and far below a VIO teleport.
		self.max_lin = rospy.get_param("~max_lin_vel", 0.5)
		self.max_ang = rospy.get_param("~max_ang_vel", 1.5)
		# Consecutive self-consistent raw frames before ACCEPTING a sustained
		# shift as a genuine relocalization (so a real restart isn't rejected
		# forever). At 15 Hz, 5 frames ~= 0.33 s.
		self.accept_frames = int(rospy.get_param("~jump_accept_frames", 5))

		self.fb_pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
		self.san_pub = rospy.Publisher(self.sanitized_topic, Odometry, queue_size=20)
		self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb, queue_size=50)

		# sanitizer state
		self.good_pose = None    # last accepted geometry_msgs/Pose
		self.good_yaw = None
		self.good_stamp = None
		self.last_raw = None     # (position, yaw, stamp) of previous raw frame
		self.jump_streak = 0
		self.reject_count = 0

		# differentiator state, fed SANITIZED poses
		self.hist = deque()        # windowed-diff samples
		self.prev = None           # consecutive-diff previous sample

		rospy.loginfo(
			"odom pipeline: sanitizer=%s windowed_diff=%s | in=%s sanitized=%s "
			"feedback=%s gate=%.2f m/s,%.2f rad/s window=%.3fs accept_frames=%d",
			self.enable_sanitizer, self.enable_windowed_diff,
			self.input_topic, self.sanitized_topic, self.output_topic,
			self.max_lin, self.max_ang, self.diff_window, self.accept_frames)

	def cb(self, msg):
		stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
		pos = msg.pose.pose.position
		yaw = yaw_from_quat(msg.pose.pose.orientation)

		pose = self._sanitize(msg, stamp, pos, yaw)

		# sanitized full-pose odom (Cartographer consumes this)
		san = Odometry()
		san.header.stamp = stamp
		san.header.frame_id = msg.header.frame_id
		san.child_frame_id = msg.child_frame_id
		san.pose.pose = pose
		self.san_pub.publish(san)

		# windowed twist on the sanitized pose (move_base odom_feedback)
		self._publish_feedback(msg, stamp, pose)

	def _sanitize(self, msg, stamp, pos, yaw):
		"""Return the Pose to trust for this frame (incoming, adopted, or held)."""
		if not self.enable_sanitizer:  # ablation off == raw passthrough (original)
			return msg.pose.pose
		if self.good_pose is None:  # bootstrap
			self.good_pose, self.good_yaw, self.good_stamp = msg.pose.pose, yaw, stamp
			self.last_raw = (pos, yaw, stamp)
			return msg.pose.pose

		dt = (stamp - self.good_stamp).to_sec()
		if dt <= 0.0:  # out-of-order / duplicate stamp
			return self.good_pose

		dist = math.sqrt((pos.x - self.good_pose.position.x) ** 2 +
			(pos.y - self.good_pose.position.y) ** 2 +
			(pos.z - self.good_pose.position.z) ** 2)
		dyaw = abs(angle_diff(yaw, self.good_yaw))
		in_sync = (dist / dt <= self.max_lin) and (dyaw / dt <= self.max_ang)

		result = self.good_pose  # default: hold last good
		held = False
		if in_sync:
			self.good_pose, self.good_yaw, self.good_stamp = msg.pose.pose, yaw, stamp
			self.jump_streak = 0
			result = msg.pose.pose
		else:
			# Diverged from our trusted pose. Is the RAW stream itself coherent
			# frame-to-frame? If so it's a sustained shift (relocalization); if
			# not, this frame is the teleport spike.
			raw_ok = False
			if self.last_raw is not None:
				rp, ryaw, rstamp = self.last_raw
				rdt = (stamp - rstamp).to_sec()
				if rdt > 0.0:
					rdist = math.sqrt((pos.x - rp.x) ** 2 + (pos.y - rp.y) ** 2 +
						(pos.z - rp.z) ** 2)
					rdyaw = abs(angle_diff(yaw, ryaw))
					raw_ok = (rdist / rdt <= self.max_lin) and (rdyaw / rdt <= self.max_ang)
			if raw_ok:
				self.jump_streak += 1
				if self.jump_streak >= self.accept_frames:
					rospy.logwarn("odom sanitizer: adopting sustained %.2f m shift "
						"as relocalization", dist)
					self.good_pose, self.good_yaw, self.good_stamp = msg.pose.pose, yaw, stamp
					self.jump_streak = 0
					result = msg.pose.pose
				else:
					held = True
			else:
				self.jump_streak = 0
				held = True
			if held:
				self.reject_count += 1
				rospy.logwarn_throttle(2.0, "odom sanitizer: rejected %.2f m jump "
					"(%.0f m/s implied), holding last good [%d total]",
					dist, dist / dt, self.reject_count)

		self.last_raw = (pos, yaw, stamp)
		return result

	def _publish_feedback(self, msg, stamp, pose):
		yaw = yaw_from_quat(pose.orientation)

		# Select the differencing base: a fixed time window, or (ablation off)
		# the immediately previous frame == original consecutive-frame diff.
		base = None  # (base_p, base_yaw, dt)
		if self.enable_windowed_diff:
			if self.hist and (stamp - self.hist[-1][0]).to_sec() <= 0.0:
				return
			self.hist.append((stamp, pose.position, yaw))
			while len(self.hist) >= 2 and (stamp - self.hist[1][0]).to_sec() >= self.diff_window:
				self.hist.popleft()
			b_stamp, b_p, b_yaw = self.hist[0]
			dt = (stamp - b_stamp).to_sec()
			if dt > 0.0:
				base = (b_p, b_yaw, dt)
		else:
			if self.prev is not None:
				b_stamp, b_p, b_yaw = self.prev
				dt = (stamp - b_stamp).to_sec()
				if dt <= 0.0:
					return
				base = (b_p, b_yaw, dt)
			self.prev = (stamp, pose.position, yaw)

		out = Odometry()
		out.header.stamp = stamp
		out.header.frame_id = msg.header.frame_id
		out.child_frame_id = msg.child_frame_id
		out.pose.pose = pose
		if base is not None:
			b_p, b_yaw, dt = base
			vx_odom = (pose.position.x - b_p.x) / dt
			vy_odom = (pose.position.y - b_p.y) / dt
			vz_odom = (pose.position.z - b_p.z) / dt
			wz = angle_diff(yaw, b_yaw) / dt
			c = math.cos(yaw)
			s = math.sin(yaw)
			out.twist.twist.linear.x = c * vx_odom + s * vy_odom
			out.twist.twist.linear.y = -s * vx_odom + c * vy_odom
			out.twist.twist.linear.z = vz_odom
			out.twist.twist.angular.z = wz
		self.fb_pub.publish(out)


def main():
	rospy.init_node("zed_pose_to_odom_feedback")
	OdomSanitizerDifferentiator()
	rospy.spin()


if __name__ == "__main__":
	main()
