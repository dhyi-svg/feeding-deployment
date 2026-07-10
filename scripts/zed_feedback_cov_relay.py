#!/usr/bin/env python3
"""
zed_feedback_cov_relay.py -- propagate the raw ZED SDK covariance onto the
sanitized ZED velocity for the fused-odom EKF.

/move_base/odom_feedback (the sanitized velocity we fuse) carries a ZERO twist
covariance -- the sanitizer drops the ZED's confidence. The raw ZED odom
(/zed_mini/zed_node/odom) carries the SDK's VIO pose covariance, which grows as
tracking degrades. This node stamps that covariance onto the feedback velocity
and republishes to /move_base/odom_feedback_cov (-> EKF odom0), so the EKF trusts
ZED less exactly when the SDK says VIO is uncertain.

The SDK's confidence lives in the POSE covariance, so its (x, y, yaw) diagonal
is mapped onto the output TWIST (vx, vy, vyaw); non-fused axes stay large.
"""

import rospy
from nav_msgs.msg import Odometry

HUGE = 1e6  # non-fused twist axes (vz, vroll, vpitch)


class ZedFeedbackCovRelay:
    def __init__(self):
        self.out_topic = rospy.get_param(
            "~output_topic", "/move_base/odom_feedback_cov")
        raw_topic = rospy.get_param("~raw_zed_topic", "/zed_mini/zed_node/odom")
        fb_topic = rospy.get_param("~feedback_topic", "/move_base/odom_feedback")

        self.cov = None  # latest raw ZED (x, y, yaw) variances

        self.pub = rospy.Publisher(self.out_topic, Odometry, queue_size=20)
        rospy.Subscriber(raw_topic, Odometry, self._cb_raw, queue_size=20)
        rospy.Subscriber(fb_topic, Odometry, self._cb_fb, queue_size=20)
        rospy.loginfo("zed_feedback_cov_relay: %s pose covariance -> %s twist "
                      "covariance", raw_topic, self.out_topic)

    def _cb_raw(self, msg):
        p = msg.pose.covariance  # 6x6 row-major: x=0, y=7, yaw=35
        self.cov = (p[0], p[7], p[35])

    def _cb_fb(self, msg):
        if self.cov is None:
            return  # no raw ZED covariance seen yet
        xv, yv, yawv = self.cov
        msg.twist.covariance = [
            xv, 0, 0, 0, 0, 0,
            0, yv, 0, 0, 0, 0,
            0, 0, HUGE, 0, 0, 0,
            0, 0, 0, HUGE, 0, 0,
            0, 0, 0, 0, HUGE, 0,
            0, 0, 0, 0, 0, yawv,
        ]
        self.pub.publish(msg)


def main():
    rospy.init_node("zed_feedback_cov_relay")
    ZedFeedbackCovRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
