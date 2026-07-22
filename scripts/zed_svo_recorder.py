#!/usr/bin/env python3
"""
zed_svo_recorder.py -- start one raw ZED SVO recording per bringup.

Since 2026-07-15 the ZED runs IMU-only (depth_mode NONE, positional tracking
off; see the block in launch/sensors.launch): the SDK's tracking/depth
internals segfaulted 4x since Jun 21 and nothing live consumes the visual
products anyway. The navigation dataset (color/depth for offline method
comparison) is instead recorded RAW here: SVO2 holds the compressed stereo
stream + full-rate IMU, and an offline SDK replay can regenerate depth (any
quality) and VIO -- or feed identical frames to competing methods --
reproducibly, without the live GPU-contention confound.

Behavior: wait for /<camera>/zed_node/start_svo_recording, then request one
recording named zed_<YYYYmmdd_HHMMSS>.svo2 under ~output_dir, retrying (with a
fresh timestamp) until the camera accepts -- the service errors while the
camera is still opening. After success, idle; best-effort stop_svo_recording
on shutdown (the wrapper also finalizes the file itself on a clean close).

Deliberately NO crash recovery: zed_node is required=true, so if the ZED dies
the whole launch exits loudly and the next bringup starts a fresh file (RJ
rule, Jul 15). Recording keeps the wrapper's grab loop alive independently of
depth/tracking (mGrabActive = mRecording || ...).

Output lands OUTSIDE the pruned system_logs session bundles on purpose --
dataset files must survive SESSION_KEEP rotation. Collect at teardown, never
over the robot WiFi mid-run. Expect ~2-4 GB/h at HD720@30 H.265.
"""

import os
import shutil
import time

import rospy
from zed_interfaces.srv import start_svo_recording, stop_svo_recording


def main():
    rospy.init_node("zed_svo_recorder")

    camera_name = rospy.get_param("~camera_name", "zed_mini")
    output_dir = rospy.get_param(
        "~output_dir",
        os.path.expanduser(
            "~/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/log/svo"))
    retry_period = float(rospy.get_param("~retry_period_s", 5.0))

    start_srv_name = "/%s/zed_node/start_svo_recording" % camera_name
    stop_srv_name = "/%s/zed_node/stop_svo_recording" % camera_name

    os.makedirs(output_dir, exist_ok=True)
    free_gb = shutil.disk_usage(output_dir).free / 2**30
    rospy.loginfo("zed_svo_recorder: output dir %s (%.0f GB free; budget ~2-4 GB/h)",
                  output_dir, free_gb)

    rospy.loginfo("zed_svo_recorder: waiting for %s", start_srv_name)
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(start_srv_name, timeout=5.0)
            break
        except rospy.ROSException:
            continue
    if rospy.is_shutdown():
        return

    start_recording = rospy.ServiceProxy(start_srv_name, start_svo_recording)
    svo_path = None
    while not rospy.is_shutdown():
        # Fresh timestamp per attempt so a late-accepted file is named by when
        # recording actually began, not by bringup time.
        candidate = os.path.join(
            output_dir, "zed_%s.svo2" % time.strftime("%Y%m%d_%H%M%S"))
        try:
            resp = start_recording(candidate)
        except rospy.ServiceException as exc:
            # The wrapper returns handler-false while the camera is still
            # opening (and for "Recording was already active"), which rospy
            # surfaces as an exception. Both are retry/ignore cases.
            rospy.logwarn("zed_svo_recorder: start attempt failed (%s); retrying in %.0fs",
                          exc, retry_period)
            rospy.sleep(retry_period)
            continue
        if resp.result:
            svo_path = candidate
            rospy.loginfo("zed_svo_recorder: RECORDING -> %s", svo_path)
            break
        rospy.logwarn("zed_svo_recorder: camera refused (%s); retrying in %.0fs",
                      resp.info, retry_period)
        rospy.sleep(retry_period)

    def stop_recording():
        # Best-effort: at teardown the zed node may already be gone, and its
        # own clean close finalizes the file anyway.
        try:
            rospy.ServiceProxy(stop_srv_name, stop_svo_recording)()
            rospy.loginfo("zed_svo_recorder: recording stopped (%s)", svo_path)
        except Exception:
            pass

    if svo_path is not None:
        rospy.on_shutdown(stop_recording)
        rospy.spin()


if __name__ == "__main__":
    main()
