# This RPC server allows other processes to communicate with the Kinova arm
# low-level controller, which runs in its own, dedicated real-time process.
#
# Note: Operations that are not time-sensitive should be run in a separate,
# non-real-time process to avoid interfering with the real-time low-level
# control and causing latency spikes.

import os
import queue
import time
import threading
from pathlib import Path

import numpy as np
from multiprocess.managers import BaseManager as MPBaseManager

RPC_AUTHKEY = b"secret-key"
# Lab default is the NUC running arm_server.py; override for single-machine
# rigs where the client and server share a box (e.g. ARM_RPC_HOST=127.0.0.1).
NUC_HOSTNAME = os.environ.get("ARM_RPC_HOST", "192.168.1.3")
ARM_RPC_PORT = 5000
BULLDOG_HEARTBEAT_TIMEOUT = 1.0  # seconds

class ArmInterface:
    def __init__(self, arm_instance):
        self.arm = arm_instance
        # self.arm.set_joint_limits(speed_limits=(7 * (30,)), acceleration_limits=(7 * (80,)))
        
        self.command_queue = queue.Queue(1)
        self.gravity_compensation_external_event = threading.Event()
        self.gravity_compensation_internal_event = threading.Event()
        self.print_debug_once = True
        self.in_compliant_mode = False

        self.emergency_stop_active = False
        self.controller = None
        self.bulldog_ready = False
        self.last_bulldog_heartbeat = None
        self._bulldog_monitor_thread = None

        # log file
        self.log_file = Path(__file__).parent / "safety_log" / "arm_commands_log.txt"
        # clear log file and set time stamp
        with open(self.log_file, "w") as f:
            f.write(f"Log file created at {time.strftime('%Y-%m-%d %H:%M:%S')}\n")

        # Lock to handle a corner case where the gravity compensation event is set by self.emergency_stop(),
        # but cleared by self.switch_out_of_compliant_mode().
        # Also handles the corner case where emergency stop is pressed right when it is switching to compliant mode
        self.gravity_compensation_external_event_lock = threading.Lock()  

    def is_alive(self):
        self.last_bulldog_heartbeat = time.time()
        return True

    def register_bulldog(self):
        self.bulldog_ready = True
        self.last_bulldog_heartbeat = time.time()
        self._bulldog_monitor_thread = threading.Thread(target=self._bulldog_monitor, daemon=True)
        self._bulldog_monitor_thread.start()
        print("Bulldog registered — arm commands unlocked.")

    def _require_bulldog(self):
        assert self.bulldog_ready, "Bulldog is not running — arm commands are locked"

    def _log_command(self, message: str):
        # One line per command, full precision, timestamped so entries
        # cross-correlate with the tmux pane logs (and failed trajectories can
        # be replayed exactly from this file alone).
        with open(self.log_file, "a") as f:
            f.write(f"{time.strftime('%Y-%m-%dT%H:%M:%S')} {message}\n")

    def _bulldog_monitor(self):
        while not self.emergency_stop_active:
            time.sleep(0.2)
            if self.last_bulldog_heartbeat is not None:
                if time.time() - self.last_bulldog_heartbeat > BULLDOG_HEARTBEAT_TIMEOUT:
                    print("ERROR: Bulldog heartbeat lost — triggering emergency stop.")
                    if not self.emergency_stop_active:
                        self.emergency_stop()
                    break

    def get_arm_state(self):
        """Diagnostic, read-only: raw Kortex ARMSTATE_* (int + name)."""
        from kortex_api.autogen.messages import Base_pb2
        val = self.arm.base.GetArmState().active_state
        name = next((n for n in dir(Base_pb2) if n.startswith("ARMSTATE") and getattr(Base_pb2, n) == val), str(val))
        return {"active_state": val, "name": name}

    def get_state(self):
        try:
            current_state = self.arm.get_state()
        except Exception as e:
            print(f"Error in get_state: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in get_state: {str(e)}") from None # suppress original exception

        # also check if gravity compensation has been set by the controller
        if self.gravity_compensation_internal_event.is_set():
            if self.print_debug_once:
                print("Emergency stop (gravity compensation) activated by controller, will not take any more commands")
                self.print_debug_once = False
            self.emergency_stop_active = True
            if self.in_compliant_mode:
                self.in_compliant_mode = False

        return current_state

    def reset(self):
        self._require_bulldog()
        # Go to home position
        print("Moving to home position")
        try:
            self.arm.home()
        except Exception as e:
            print(f"Error in reset: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in reset: {str(e)}") from None # suppress original exception

    def set_tool(self, tool: str):
        self._require_bulldog()
        print(f"Setting tool to {tool}")
        try:
            self.arm.set_tool(tool)
        except Exception as e:
            print(f"Error in set_tool: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_tool: {str(e)}") from None # suppress original exception

    def set_speed(self, speed: str):
        """ speed: "low", "medium", "high" """
        self._require_bulldog()
        assert speed in ["low", "medium", "high"], "Invalid speed"
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "Cannot set speed while in compliant mode"
        
        self._log_command(f"set_speed: {speed}")

        print(f"Setting speed to {speed}")
        try:
            self.arm.choose_from_speed_presets(speed)
        except Exception as e:
            print(f"Error in choose_from_speed_presets: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in choose_from_speed_presets: {str(e)}") from None
        
    def get_speed(self):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "Cannot get speed while in compliant mode"

        try:
            arm_speed = self.arm.get_speed_preset()
        except Exception as e:
            print(f"Error in get_speed: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in get_speed: {str(e)}") from None

        return arm_speed


    def switch_to_task_compliant_mode(self):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "Already in compliant mode"

        self._log_command("switch_to_task_compliant_mode")

        # clear command queue
        print("Clearing command queue")
        while not self.command_queue.empty():
            self.command_queue.get()

        # switch to joint compliant mode
        print("Switching to joint compliant mode")
        with self.gravity_compensation_external_event_lock:
            try:
                self.arm.switch_to_task_compliant_mode(self.command_queue, self.gravity_compensation_external_event, self.gravity_compensation_internal_event)
            except Exception as e:
                print(f"Error in switch_to_task_compliant_mode: {e}")
                # Re-raise a simplified exception to avoid pickling issues
                raise Exception(f"Error in switch_to_task_compliant_mode: {str(e)}") from None # suppress original exception
            self.in_compliant_mode = True

    def switch_to_joint_compliant_mode(self):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "Already in compliant mode"

        self._log_command("switch_to_joint_compliant_mode")

        # clear command queue
        print("Clearing command queue")
        while not self.command_queue.empty():
            self.command_queue.get()

        # switch to joint compliant mode
        print("Switching to joint compliant mode")

        with self.gravity_compensation_external_event_lock:
            try:
                self.arm.switch_to_joint_compliant_mode(self.command_queue, self.gravity_compensation_external_event, self.gravity_compensation_internal_event)
            except Exception as e:
                print(f"Error in switch_to_joint_compliant_mode: {e}")
                # Re-raise a simplified exception to avoid pickling issues
                raise Exception(f"Error in switch_to_joint_compliant_mode: {str(e)}") from None
            self.in_compliant_mode = True

    def switch_out_of_compliant_mode(self):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert self.in_compliant_mode, "Not in compliant mode"

        self._log_command("switch_out_of_compliant_mode")

        # first move to gravity compensation 
        print("Moving to gravity compensation")
        self.gravity_compensation_external_event.set()
        time.sleep(1.0) # Wait for the arm to settle

        with self.gravity_compensation_external_event_lock:

            # switch out of joint compliant mode
            if self.emergency_stop_active:
                print("Cannot switch out of compliant mode due to emergency stop")
                return
    
            print("Switching out of joint compliant mode")
            try:
                self.arm.switch_out_of_compliant_mode()
            except Exception as e:
                print(f"Error in switch_out_of_compliant_mode: {e}")
                # Re-raise a simplified exception to avoid pickling issues
                raise Exception(f"Error in switch_out_of_compliant_mode: {str(e)}") from None # suppress original exception
            self.in_compliant_mode = False

            self.gravity_compensation_external_event.clear()

    def compliant_set_joint_position(self, command_pos):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert self.in_compliant_mode, "Not in compliant mode"

        self._log_command(f"compliant_set_joint_position: {np.asarray(command_pos).tolist()}")

        # print(f"Received compliant joint pos command: {command_pos}")
        gripper_pos = 0
        self.command_queue.put((command_pos, gripper_pos))

        return True

    def compliant_set_ee_pose(self, xyz, xyz_quat):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert self.in_compliant_mode, "Not in compliant mode"

        self._log_command(f"compliant_set_ee_pose: {np.asarray(xyz).tolist()}, {np.asarray(xyz_quat).tolist()}")

        command_pose = np.zeros(7)
        command_pose[:3] = xyz
        command_pose[3:] = xyz_quat

        # print(f"Received compliant cartesian pose command: {xyz}, {xyz_quat}")
        gripper_pos = 0
        self.command_queue.put((command_pose, gripper_pos))

        return True

    def set_joint_position(self, command_pos):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        self._log_command(f"set_joint_position: {np.asarray(command_pos).tolist()}")

        print(f"Received joint pos command: {command_pos}")

        try:
            success = self.arm.move_angular(command_pos)
        except Exception as e:
            print(f"Error in set_joint_position: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_joint_position: {str(e)}") from None # suppress original exception
        
        return success

    def set_joint_trajectory(self, trajectory_command):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        traj_str = "; ".join(str(np.asarray(w).tolist()) for w in trajectory_command)
        self._log_command(f"set_joint_trajectory ({len(trajectory_command)} waypoints): {traj_str}")

        print(
            f"Received joint trajectory command with {len(trajectory_command)} waypoints"
        )

        try:
            success = self.arm.move_angular_trajectory(trajectory_command)
        except Exception as e:
            print(f"Error in set_joint_trajectory: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_joint_trajectory: {str(e)}") from None # suppress original exception
        return success

    def set_ee_pose(self, xyz, xyz_quat, soft_stop=False):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        self._log_command(f"set_ee_pose: {np.asarray(xyz).tolist()}, {np.asarray(xyz_quat).tolist()}")

        print(f"Received cartesian pose command: {xyz}, {xyz_quat}")

        try:
            success = self.arm.move_cartesian(xyz, xyz_quat, soft_stop=soft_stop)
        except Exception as e:
            print(f"Error in set_ee_pose: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_ee_pose: {str(e)}") from None # suppress original exception
        return success
        
    def set_cartesian_trajectory(self, trajectory_command):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        traj_str = "; ".join(
            f"{np.asarray(pos).tolist()} {np.asarray(quat).tolist()}"
            for pos, quat in trajectory_command
        )
        self._log_command(f"set_cartesian_trajectory ({len(trajectory_command)} waypoints): {traj_str}")

        print(
            f"Received cartesian trajectory command with {len(trajectory_command)} waypoints"
        )

        try:
            success = self.arm.move_cartesian_trajectory(trajectory_command)
        except Exception as e:
            print(f"Error in set_cartesian_trajectory: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_cartesian_trajectory: {str(e)}") from None # suppress original exception
        return success

    def set_gripper(self, gripper_pos):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        self._log_command(f"set_gripper: {gripper_pos}")

        print(f"Received gripper pos command: {gripper_pos}")

        try:
            self.arm._gripper_position_command(gripper_pos)
        except Exception as e:
            print(f"Error in set_gripper: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in set_gripper: {str(e)}") from None # suppress original exception

        return True

    def open_gripper(self):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        self._log_command("open_gripper")

        print("Received open gripper command")

        try:
            self.arm.open_gripper()
        except Exception as e:
            print(f"Error in open_gripper: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in open_gripper: {str(e)}") from None # suppress original exception

        return True

    def close_gripper(self):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        self._log_command("close_gripper")

        print("Received close gripper command")

        try:
            self.arm.close_gripper()
        except Exception as e:
            print(f"Error in close_gripper: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in close_gripper: {str(e)}") from None # suppress original exception

        return True

    def close(self):
        print("Close arm command received")
        if self.in_compliant_mode:
            print("Switching out of compliant mode through emergency stop")
            self.emergency_stop()
            time.sleep(1.0) # Wait for the arm to settle

        try:
            self.arm.stop() # Exit low level servoing mode incase it was in compliant mode, otherwise stop arm
            print("Arm stopped")
            self.arm.disconnect()
            print("Arm disconnected")
        except Exception as e:
            print(f"Error in close: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in close: {str(e)}") from None # suppress original exception

    def retract(self):
        self._require_bulldog()
        assert not self.emergency_stop_active, "Emergency stop is active"
        assert not self.in_compliant_mode, "In compliant mode"

        print("Received retract command")

        try:
            self.arm.retract()
        except Exception as e:
            print(f"Error in retract: {e}")
            # Re-raise a simplified exception to avoid pickling issues
            raise Exception(f"Error in retract: {str(e)}") from None # suppress original exception

    def stop_action(self):
        """Abort the action currently executing without latching emergency stop.

        Unlike emergency_stop(), this does NOT set emergency_stop_active, so the
        arm keeps accepting commands afterward. Used to preempt the (long) move
        to the default pose from the manual teleop recovery screen.
        """
        self._log_command("stop_action")

        if self.in_compliant_mode:
            print("stop_action ignored: arm is in compliant mode")
            return
        try:
            self.arm.stop_action()
        except Exception as e:
            print(f"Error in stop_action: {e}")
            raise Exception(f"Error in stop_action: {str(e)}") from None

    def emergency_stop(self):
        assert not self.emergency_stop_active, "Emergency stop is already active"

        self._log_command("emergency_stop")

        with self.gravity_compensation_external_event_lock:
            self.emergency_stop_active = True
            if self.in_compliant_mode:
                self.in_compliant_mode = False
                self.gravity_compensation_external_event.set()
            else: # If not in compliant mode, stop arm (otherwise, arm is already stopped)
                try:
                    self.arm.stop()
                except Exception as e:
                    print(f"Error in emergency_stop: {e}")
                    # Re-raise a simplified exception to avoid pickling issues
                    raise Exception(f"Error in emergency_stop: {str(e)}") from None # suppress original exception

            print("Emergency stop activated by user, will not take any more commands")

class ArmManager(MPBaseManager):
    pass
