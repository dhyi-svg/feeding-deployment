'''
Runs a client-side (run on compute machine) watchdog for the robot's intended functionality. 
It validates the following:
1. All sensors are streaming correctly.
2. All sensor outputs are within the expected range.
    a. The ft sensor is not exceeding the threshold.
    b. The camera perception outputs are within the expected range (ToDo)
    c. The robot's current state is not near collision.
If any of the above is not true, the watchdog will return the corresponding AnomalyStatus.
'''

import rospy
import numpy as np
import time
from enum import Enum
import queue
import signal
import sys

from sensor_msgs.msg import CameraInfo, Image, LaserScan, JointState
from geometry_msgs.msg import WrenchStamped, Pose
from std_msgs.msg import Bool, Float32MultiArray

import threading
import time
import numpy as np
from pathlib import Path

import rospy
from std_msgs.msg import Bool
from netft_rdt_driver.srv import String_cmd

from feeding_deployment.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY


CAMERA_FREQUENCY_THRESHOLD = 2 # expected is 30 Hz
# Depth is a separate USB endpoint from color: it can stall (uvc streamer watchdog)
# while color + camera_info stay at 30 Hz. RGBD perception needs all three synced, so
# monitor depth independently -- a color-only check would miss a depth-only stall.
CAMERA_DEPTH_FREQUENCY_THRESHOLD = 2 # expected is 30 Hz (aligned_depth_to_color)
EXPECTED_CAMERA_RESOLUTION = (1280, 720) # (width, height); D435 color stream at 30 Hz
FT_FREQUENCY_THRESHOLD = 300 # expected is 1000 Hz
FT_THRESHOLD = [40.0, 40.0, 40.0, 2.0, 2.0, 2.0]
COLLISION_FREE_FREQUENCY_THRESHOLD = 100 # expected is 350 Hz (empirical)
LIDAR_FREQUENCY_THRESHOLD = 2 # expected is ~5-10 Hz (RPLIDAR A1)
ROBOT_JOINT_STATES_FREQUENCY_THRESHOLD = 2 # expected high (tight publish loop)
ROBOT_CARTESIAN_STATE_FREQUENCY_THRESHOLD = 2 # expected high (tight publish loop)

WATCHDOG_RUN_FREQUENCY = 1000

from feeding_deployment.safety.utils import PeekableQueue, AnomalyStatus

class WatchDog:
    def __init__(self):

        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()

        # bias FT sensor
        bias = rospy.ServiceProxy('/forque/bias_cmd', String_cmd)
        bias('bias')
        time.sleep(2.0) # wait for bias to complete

        queue_size = 1000
        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.cameraCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.camera_timestamps = PeekableQueue()

        # Depth stream is a distinct USB endpoint from color and can stall on its own;
        # RGBD perception (RealSenseInterface's exact-time sync) needs it, so monitor it.
        self.camera_depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.cameraDepthCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.camera_depth_timestamps = PeekableQueue()

        # One-time camera resolution check at launch (not monitored every loop).
        self.camera_resolution_unexpected = self._check_camera_resolution()

        self.camera_unexpected_sub = rospy.Subscriber("/head_perception/unexpected", Bool, self.cameraUnexpectedCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.camera_unexpected = False 
        
        self.ft_sub = rospy.Subscriber('/forque/forqueSensor', WrenchStamped, self.ftCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.ft_timestamps = PeekableQueue()
        self.ft_unexpected = False

        self.collision_free_sub = rospy.Subscriber('/collision_free', Bool, self.collisionFreeCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.collision_free_timestamps = PeekableQueue()
        self.collision_free_unexpected = False

        # Collision force/threshold for the status panel, published by collision_sensor.py
        # as [current_max_error, peak_last_10s, threshold]. None until first message.
        self.collision_force_sub = rospy.Subscriber('/collision_force', Float32MultiArray, self.collisionForceCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.collision_force = None

        self.lidar_l_sub = rospy.Subscriber('/lidar_l/scan', LaserScan, self.lidarLeftCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.lidar_l_timestamps = PeekableQueue()

        self.lidar_r_sub = rospy.Subscriber('/lidar_r/scan', LaserScan, self.lidarRightCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.lidar_r_timestamps = PeekableQueue()

        # ZED odom subscriber removed [2026-07-15]: the ZED runs IMU-only
        # (no VIO -> /zed_node/odom never publishes), and any odom subscriber
        # makes the wrapper WARN "Cannot start Positional Tracking" every grab
        # cycle. Its frequency check had long been commented out below.

        self.robot_joint_states_sub = rospy.Subscriber('/robot_joint_states', JointState, self.robotJointStatesCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.robot_joint_states_timestamps = PeekableQueue()

        self.robot_cartesian_state_sub = rospy.Subscriber('/robot_cartesian_state', Pose, self.robotCartesianStateCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.robot_cartesian_state_timestamps = PeekableQueue()

        self.watchdog_status_pub = rospy.Publisher("/watchdog_status", Bool, queue_size=1)

        self.execution_log_path = Path(__file__).parent.parent / "integration" / "log" / "execution_log.txt"

        self.disable_collision_sensor_pub = rospy.Publisher("/disable_collision_sensor", Bool, queue_size=1)

        self.second_counter = 0
        # Status-panel rendering state: number of lines drawn last refresh, so we can
        # move the cursor up and overwrite in place (only when writing to a real TTY).
        self._panel_lines = 0
        self._is_tty = sys.stdout.isatty()
        time.sleep(5.0) # Wait for all queues to fill up / collision monitor to start
        
        # make sure collision is enabled
        self.disable_collision_sensor_pub.publish(Bool(data=False))
        print("Initialized.")

    def _check_camera_resolution(self):
        """Read a single CameraInfo at launch and verify the stream resolution.
        Returns True if the resolution is not as expected. Frequency monitoring
        (via cameraCallback) handles the camera-not-streaming case separately."""
        try:
            msg = rospy.wait_for_message("/camera/color/camera_info", CameraInfo, timeout=10.0)
        except rospy.ROSException:
            print("Could not read camera resolution at launch (no CameraInfo received).")
            rospy.loginfo("Could not read camera resolution at launch (no CameraInfo received).")
            return False
        resolution = (msg.width, msg.height)
        if resolution != EXPECTED_CAMERA_RESOLUTION:
            print(f"Unexpected camera resolution: {resolution}, expected {EXPECTED_CAMERA_RESOLUTION}")
            rospy.loginfo(f"Unexpected camera resolution: {resolution}, expected {EXPECTED_CAMERA_RESOLUTION}")
            return True
        return False

    def cameraCallback(self, msg):
        self.camera_timestamps.put(time.time())

    def cameraDepthCallback(self, msg):
        self.camera_depth_timestamps.put(time.time())

    def cameraUnexpectedCallback(self, msg):
        # self.camera_unexpected = msg.data
        pass

    def ftCallback(self, msg):

        self.ft_timestamps.put(time.time())
        ft = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z]
        if not self.ft_unexpected:
            for i in range(6):
                if abs(ft[i]) > FT_THRESHOLD[i]:
                    print("FT threshold exceeded with magnitude: ", ft)
                    self.ft_unexpected = True
                    break

    def collisionFreeCallback(self, msg):

        self.collision_free_timestamps.put(time.time())
        if not msg.data:
            self.collision_free_unexpected = True

    def collisionForceCallback(self, msg):
        # [current_max_error, peak_last_10s, threshold]
        if len(msg.data) >= 3:
            self.collision_force = (msg.data[0], msg.data[1], msg.data[2])

    def lidarLeftCallback(self, msg):
        self.lidar_l_timestamps.put(time.time())

    def lidarRightCallback(self, msg):
        self.lidar_r_timestamps.put(time.time())

    def robotJointStatesCallback(self, msg):
        self.robot_joint_states_timestamps.put(time.time())

    def robotCartesianStateCallback(self, msg):
        self.robot_cartesian_state_timestamps.put(time.time())

    def _render_panel(self, frequencies):
        """Render a status panel (sensor frequencies + collision force) that updates
        in place on a TTY, so the shared watchdog terminal looks static with only the
        numbers changing. Falls back to plain prints when stdout is not a terminal."""
        name_width = max((len(name) for name, _ in frequencies), default=0)
        lines = []
        lines.append("================ Watchdog ================")
        lines.append("Frequency (msgs in last 1s):")
        for name, freq in frequencies:
            lines.append(f"  {name:<{name_width}} : {freq}")
        lines.append("Collision force:")
        if self.collision_force is not None:
            current, peak, threshold = self.collision_force
            label_width = len("Peak (last 10s)")
            lines.append(f"  {'Current':<{label_width}} : {current:8.3f}")
            lines.append(f"  {'Peak (last 10s)':<{label_width}} : {peak:8.3f}")
            lines.append(f"  {'Threshold':<{label_width}} : {threshold:8.3f}")
        else:
            lines.append("  (waiting for /collision_force ...)")
        lines.append("==========================================")

        text = "\n".join(lines)
        if self._is_tty:
            # Move cursor to the top of the previous panel and clear to end of screen,
            # then redraw in place.
            if self._panel_lines:
                sys.stdout.write(f"\033[{self._panel_lines}F\033[J")
            sys.stdout.write(text + "\n")
            sys.stdout.flush()
            self._panel_lines = len(lines)
        else:
            print(text)

    def check_status(self):
        self.second_counter += 1
        self._arm_interface.is_alive()
        anomaly = AnomalyStatus.NO_ANOMALY
        start_time = time.time()
        frequencies = []
        for _queue, _threshold, _anomaly in [(self.ft_timestamps, FT_FREQUENCY_THRESHOLD, AnomalyStatus.FT_FREQUENCY),
                                            (self.camera_timestamps, CAMERA_FREQUENCY_THRESHOLD, AnomalyStatus.CAMERA_FREQUENCY),
                                            (self.camera_depth_timestamps, CAMERA_DEPTH_FREQUENCY_THRESHOLD, AnomalyStatus.CAMERA_DEPTH_FREQUENCY),
                                            (self.collision_free_timestamps, COLLISION_FREE_FREQUENCY_THRESHOLD, AnomalyStatus.COLLISION_FREE_FREQUENCY),
                                            (self.lidar_l_timestamps, LIDAR_FREQUENCY_THRESHOLD, AnomalyStatus.LIDAR_L_FREQUENCY),
                                            (self.lidar_r_timestamps, LIDAR_FREQUENCY_THRESHOLD, AnomalyStatus.LIDAR_R_FREQUENCY),
                                            # zed odom check retired [2026-07-15]: IMU-only ZED, no VIO odom (subscriber removed above)
                                            (self.robot_joint_states_timestamps, ROBOT_JOINT_STATES_FREQUENCY_THRESHOLD, AnomalyStatus.ROBOT_JOINT_STATES_FREQUENCY),
                                            (self.robot_cartesian_state_timestamps, ROBOT_CARTESIAN_STATE_FREQUENCY_THRESHOLD, AnomalyStatus.ROBOT_CARTESIAN_STATE_FREQUENCY)]:
            while _queue.peek() < start_time - 1.0:
                _queue.get()
            queue_size = _queue.qsize()
            if queue_size < _threshold:
                print(f"Frequency: {queue_size} for {_anomaly}")
                rospy.loginfo(f"Frequency: {queue_size} for {_anomaly}")
                anomaly = _anomaly
                break   
            frequencies.append((_anomaly.name, queue_size))

        if self.second_counter == WATCHDOG_RUN_FREQUENCY:
            self._render_panel(frequencies)
            self.second_counter = 0

        for _unexpected, _anomaly in [
                                    (self.camera_unexpected, AnomalyStatus.CAMERA_UNEXPECTED),
                                    (self.camera_resolution_unexpected, AnomalyStatus.CAMERA_RESOLUTION),
                                    (self.ft_unexpected, AnomalyStatus.FT_UNEXPECTED),
                                    (self.collision_free_unexpected, AnomalyStatus.COLLISION_FREE_UNEXPECTED)]:
            if _unexpected:
                print(f"Unexpected: {_anomaly}")
                rospy.loginfo(f"Unexpected: {_anomaly}")
                anomaly = _anomaly
                break

        if anomaly != AnomalyStatus.NO_ANOMALY:
            self._arm_interface.emergency_stop()
            print(f"AnomalyStatus detected: {anomaly}")
            rospy.loginfo(f"AnomalyStatus detected: {anomaly}")
            with open(self.execution_log_path, 'a') as f:
                f.write(f"Anomaly Detected: {AnomalyStatus.get_error_message(anomaly)}\n") 

        self.watchdog_status_pub.publish(Bool(data=anomaly == AnomalyStatus.NO_ANOMALY))
        return anomaly
    
    def run(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            status = self.check_status()
            if status != AnomalyStatus.NO_ANOMALY:
                break
            end_time = time.time()
            # print(f"Time taken: {end_time - start_time}")
            time.sleep(max(0, 1.0/WATCHDOG_RUN_FREQUENCY - (end_time - start_time)))

if __name__ == '__main__':

    rospy.init_node('WatchDog', anonymous=True)
    
    watchdog = WatchDog()
    watchdog.run()
    