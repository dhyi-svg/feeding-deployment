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


class OdomDifferentiator:
	def __init__(self):
		self.input_topic = rospy.get_param("~input_odom_topic", "/zed_mini/zed_node/odom")
		self.output_topic = rospy.get_param("~output_odom_topic", "/move_base/odom_feedback")

		# Differentiate over a fixed time window, not consecutive frames:
		# velocity noise = pose jitter / dt, and dt per frame shrank from
		# 65 ms to ~19 ms when the ZED went 15 -> 60 Hz (spikes reached
		# 0.12 m/s on a stationary robot, destabilizing TEB's feasibility
		# check). The window restores the old noise floor at full rate.
		self.diff_window = rospy.get_param("~vel_diff_window", 0.08)

		self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=20)
		self.sub = rospy.Subscriber(self.input_topic, Odometry, self.cb, queue_size=50)

		self.hist = deque()

		rospy.loginfo("zed_pose_to_odom_feedback input=%s output=%s vel_diff_window=%.3fs",
			self.input_topic, self.output_topic, self.diff_window)

	def cb(self, msg):
		stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
		yaw = yaw_from_quat(msg.pose.pose.orientation)

		if self.hist and (stamp - self.hist[-1][0]).to_sec() <= 0.0:
			return
		self.hist.append((stamp, msg.pose.pose.position, yaw))

		# Diff base = newest sample at least diff_window old, so dt spans
		# [diff_window, diff_window + one frame period).
		while len(self.hist) >= 2 and (stamp - self.hist[1][0]).to_sec() >= self.diff_window:
			self.hist.popleft()

		base_stamp, base_p, base_yaw = self.hist[0]
		dt = (stamp - base_stamp).to_sec()

		out = Odometry()
		out.header = msg.header
		out.child_frame_id = msg.child_frame_id
		out.pose = msg.pose

		if dt > 0.0:
			curr_p = msg.pose.pose.position

			vx_odom = (curr_p.x - base_p.x) / dt
			vy_odom = (curr_p.y - base_p.y) / dt
			vz_odom = (curr_p.z - base_p.z) / dt
			wz = angle_diff(yaw, base_yaw) / dt

			# Put translational velocity in base frame.
			c = math.cos(yaw)
			s = math.sin(yaw)
			out.twist.twist.linear.x = c * vx_odom + s * vy_odom
			out.twist.twist.linear.y = -s * vx_odom + c * vy_odom
			out.twist.twist.linear.z = vz_odom
			out.twist.twist.angular.z = wz

		self.pub.publish(out)


def main():
	rospy.init_node("zed_pose_to_odom_feedback")
	OdomDifferentiator()
	rospy.spin()


if __name__ == "__main__":
	main()
