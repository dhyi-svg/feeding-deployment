'''
Runs a server-side (run on NUC) watchdog to ensure robot is not in a state of emergency stop (from the experimentor emergency stop button).
'''

import rospy
import numpy as np
import time
from enum import Enum
import queue
import signal
import sys
import os
import paramiko

import threading
import time
import numpy as np
from pathlib import Path

import rospy
from std_msgs.msg import Bool

from feeding_deployment.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY
from feeding_deployment.control.base_controller.base_interface import BaseManager, BASE_RPC_PORT

# Min packets/sec on /experimentor_estop (counted over the trailing 1s window).
# Nominal is ~82 Hz from the Mac sender; 30 leaves wide margin above normal
# jitter (81-84) while still flagging real degradation early.
EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD = 30

# Debounce for the frequency check: the rate must stay below the threshold
# CONTINUOUSLY for this long before we trip. This is the link-liveness heartbeat
# (NOT a button press, which is latched on its own immediate path), so a transient
# WiFi blackout of up to ~1s should be ridden out rather than cause a false stop.
# A genuine link death still trips within ~ESTOP_FREQ_GRACE_S of the rate dipping.
# If ~1s WiFi blackouts still slip through, raise this toward 1.2-1.5s.
ESTOP_FREQ_GRACE_S = 1.0

BULLDOG_RUN_FREQUENCY = 1000

from feeding_deployment.safety.utils import PeekableQueue, AnomalyStatus

class BullDog:
    def __init__(self):
        print("BullDog awakening...")
        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()
        
        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()

        # Base RPC client setup (symmetric with the arm). Required: if the base
        # server is down, connect() raises and bulldog refuses to start, exactly
        # like the arm today.
        BaseManager.register("BaseInterface")
        self.base_manager = BaseManager(address=(NUC_HOSTNAME, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
        self.base_manager.connect()
        self._base_interface = self.base_manager.BaseInterface()

        queue_size = 1000
        self.experimentor_emergency_stop_sub = rospy.Subscriber('/experimentor_estop', Bool, self.experimentorEmergencyStopCallback, queue_size = queue_size, buff_size = 65536*queue_size)
        self.experimentor_emergency_stop_timestamps = PeekableQueue()
        self.experimentor_emergency_stop_pressed = False
        # Debounce state: wall-clock time the rate first dipped below threshold in
        # the current low spell, or None when the rate is healthy. See check_status.
        self.experimentor_estop_low_freq_since = None
        # Lowest rate seen during the current low spell, for the near-miss log.
        self.experimentor_estop_low_freq_min = None

        self.bulldog_status_pub = rospy.Publisher('/bulldog_status', Bool, queue_size=1)

        # Path is hardcoded because emprise uses two machines, 
        # isacc for compute and nuc for robot control, 
        # and we need to transmit logs from nuc (where bulldog runs) to isacc
        self.remote_execution_log_path = "/home/isacc/deployment_ws/src/feeding-deployment/src/feeding_deployment/integration/log/nuc_execution_log.txt"
        hostname = "192.168.1.2"
        username = "isacc"

        # Get the password from the environment variable
        password = os.getenv('ISACC_PASSWORD')
        if not password:
            print("Error: The environment variable 'ISACC_PASSWORD' must be set.")
            sys.exit(1)

        try:
            # Initialize SSH client
            self.client = paramiko.SSHClient()
            self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())

            # Connect to the server using the password from the environment variable
            self.client.connect(hostname, username=username, password=password)
        except paramiko.AuthenticationException:
            print("Authentication failed. Check your username and password.")
        except paramiko.SSHException as e:
            print(f"SSH error: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

        self.second_counter = 0
        time.sleep(1.0)
        self._arm_interface.register_bulldog()
        self._base_interface.register_bulldog()
        print("BullDog is guarding the robot.")

    def write_to_remote(self, anomaly_message):

        # Open SFTP session
        sftp = self.client.open_sftp()
        with sftp.file(self.remote_execution_log_path, 'a') as f:
            f.write(anomaly_message + '\n')

        sftp.close()

    def experimentorEmergencyStopCallback(self, msg):

        self.experimentor_emergency_stop_timestamps.put(time.time())
        if msg.data:
            self.experimentor_emergency_stop_pressed = True

    def check_status(self):
        self.second_counter += 1
        self._arm_interface.is_alive()
        self._base_interface.is_alive()
        anomaly = AnomalyStatus.NO_ANOMALY
        start_time = time.time()
        frequencies = []
        for _queue, _threshold, _anomaly in [(self.experimentor_emergency_stop_timestamps, EXPERIMENTOR_ESTOP_FREQUENCY_THRESHOLD, AnomalyStatus.EXPERIMENTOR_ESTOP_FREQUENCY)]:
            while _queue.peek() < start_time - 1.0:
                _queue.get()
            queue_size = _queue.qsize()
            frequencies.append(queue_size)
            if queue_size < _threshold:
                # Below threshold. Debounce it: only trip once the rate has been
                # low CONTINUOUSLY for ESTOP_FREQ_GRACE_S, so a transient WiFi
                # blackout does not cause a false stop. A genuine link death stays
                # low and trips after the grace window.
                if self.experimentor_estop_low_freq_since is None:
                    self.experimentor_estop_low_freq_since = start_time
                    self.experimentor_estop_low_freq_min = queue_size
                else:
                    self.experimentor_estop_low_freq_min = min(
                        self.experimentor_estop_low_freq_min, queue_size)
                low_for = start_time - self.experimentor_estop_low_freq_since
                if low_for >= ESTOP_FREQ_GRACE_S:
                    print(f"Frequency: {queue_size} for {_anomaly} (low for {low_for:.2f}s)")
                    rospy.loginfo(f"Frequency: {queue_size} for {_anomaly} (low for {low_for:.2f}s)")
                    anomaly = _anomaly
                    break
            else:
                # Healthy reading clears the low spell. If we were in a dip that the
                # debounce swallowed (no trip), log it as a near-miss so a degrading
                # WiFi link is visible BEFORE a blackout finally exceeds the grace.
                if self.experimentor_estop_low_freq_since is not None:
                    dip_for = start_time - self.experimentor_estop_low_freq_since
                    if dip_for >= 0.1:  # ignore single-reading blips
                        msg = (f"experimentor estop heartbeat dipped to "
                               f"{self.experimentor_estop_low_freq_min} Hz for {dip_for:.2f}s "
                               f"(threshold {_threshold}, grace {ESTOP_FREQ_GRACE_S:.1f}s) -- "
                               f"recovered, no stop")
                        print(f"[near-miss] {msg}")
                        rospy.logwarn(msg)
                self.experimentor_estop_low_freq_since = None
                self.experimentor_estop_low_freq_min = None

        if self.second_counter == BULLDOG_RUN_FREQUENCY:
            print("Bulldog running at expected frequency.")
            if frequencies:
                print(f"Frequencies: Experimentor EStop: {frequencies[0]}")
            self.second_counter = 0

        for _unexpected, _anomaly in [(self.experimentor_emergency_stop_pressed, AnomalyStatus.EXPERIMENTOR_ESTOP_PRESSED)]:
            if _unexpected:
                print(f"Unexpected: {_anomaly}")
                rospy.loginfo(f"Unexpected: {_anomaly}")
                anomaly = _anomaly
                break

        if anomaly != AnomalyStatus.NO_ANOMALY:
            self._arm_interface.emergency_stop()
            self._base_interface.emergency_stop()  # idempotent; safe if heartbeat monitor also fired
            print(f"AnomalyStatus detected: {anomaly}")
            rospy.loginfo(f"AnomalyStatus detected: {anomaly}")
            self.write_to_remote(f"Anomaly Detected: {AnomalyStatus.get_error_message(anomaly)}")
            # with open(self.execution_log_path, 'a') as f:
                # f.write(f"Anomaly Detected: {AnomalyStatus.get_error_message(anomaly)}\n") 

        self.bulldog_status_pub.publish(Bool(data=anomaly == AnomalyStatus.NO_ANOMALY))
        return anomaly
    
    def run(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            status = self.check_status()
            if status != AnomalyStatus.NO_ANOMALY:
                break
            end_time = time.time()
            # print(f"Time taken: {end_time - start_time}")
            time.sleep(max(0, 1.0/BULLDOG_RUN_FREQUENCY - (end_time - start_time)))

if __name__ == '__main__':

    rospy.init_node('BullDog', anonymous=True)
    bulldog = BullDog()
    
    bulldog.run()
    