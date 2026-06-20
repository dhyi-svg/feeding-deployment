'''
Entrypoint for controlling the robot arm on compute machine. Additionally runs two important threads:
1. A thread that checks no safety anomalies have occurred using the watchdog
2. A thread that publishes joint states to ROS
'''

import threading
import time
import types
import numpy as np
import yaml
from pathlib import Path

try:
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool
    from geometry_msgs.msg import Pose
    # from netft_rdt_driver.srv import String_cmd
    ROSPY_IMPORTED = True
except ModuleNotFoundError as e:
    # print(f"ROS not imported: {e}")
    ROSPY_IMPORTED = False

def load_robot_config(config_path: str) -> types.SimpleNamespace:
    with open(config_path, "r") as f:
        raw = yaml.safe_load(f)
    parsed = {}
    for key, entry in raw.items():
        if isinstance(entry, dict):
            parsed[key] = entry["values"]
    return types.SimpleNamespace(**parsed)

from feeding_deployment.control.robot_controller.arm_interface import ArmInterface, ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY
from feeding_deployment.control.robot_controller.command_interface import KinovaCommand, JointTrajectoryCommand, CartesianTrajectoryCommand, JointCommand, CartesianCommand, OpenGripperCommand, CloseGripperCommand
# from feeding_deployment.safety.watchdog import WATCHDOG_MONITOR_FREQUENCY, PeekableQueue

class ArmInterfaceClient:
    def __init__(self):

        assert ROSPY_IMPORTED, "ROS is required to run on the real robot"

        # make sure watchdog is running
        print("Waiting for Watchdog status...")
        rospy.wait_for_message("/watchdog_status", Bool)
        print("Watchdog is running, continuing...")

        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()
        self.in_compliant_mode = False

    def switch_to_task_compliant_mode(self):
        assert not self.in_compliant_mode, "Already in compliant mode"
        self._arm_interface.switch_to_task_compliant_mode()
        self.in_compliant_mode = True

    def switch_to_joint_compliant_mode(self):
        assert not self.in_compliant_mode, "Already in compliant mode"
        self._arm_interface.switch_to_joint_compliant_mode()
        self.in_compliant_mode = True

    def switch_out_of_compliant_mode(self):
        assert self.in_compliant_mode, "Not in compliant mode"
        # time.sleep(2.0) # Wait for the arm to settle
        self._arm_interface.switch_out_of_compliant_mode()
        self.in_compliant_mode = False

    def get_state(self):
        return self._arm_interface.get_state()

    def stop_action(self):
        """Abort the current arm action without latching emergency stop.

        NOTE: this is issued on the same RPC connection as execute_command. If a
        blocking move is in flight on this connection, the manager may serialize
        this call behind it. See TELEOP_INTEGRATION.md (Default -> Stop).
        """
        return self._arm_interface.stop_action()

    def get_speed(self):
        return self._arm_interface.get_speed()
    
    def set_speed(self, speed: str):
        assert not self.in_compliant_mode, "Cannot set speed in compliant mode"
        assert speed in ["low", "medium", "high"], "Speed must be one of 'low', 'medium', 'high'"
        self._arm_interface.set_speed(speed)
        time.sleep(1.0) # Make sure the arm has time to change speed

    def set_tool(self, tool: str):
        assert not self.in_compliant_mode, "Cannot set tool in compliant mode"
        self._arm_interface.set_tool(tool)

    def execute_command(self, cmd: KinovaCommand) -> None:

        # if not self.in_compliant_mode:
            # input("Press enter to execute command...")

        if cmd.__class__.__name__ == "JointTrajectoryCommand":
            return self._arm_interface.set_joint_trajectory(cmd.traj)
        
        if cmd.__class__.__name__ == "CartesianTrajectoryCommand":
            return self._arm_interface.set_cartesian_trajectory(cmd.traj)

        if cmd.__class__.__name__ == "JointCommand":
            if self.in_compliant_mode:
                return self._arm_interface.compliant_set_joint_position(cmd.pos)
            else:
                joint_command_pos = cmd.pos
                if isinstance(joint_command_pos, np.ndarray):
                    joint_command_pos = joint_command_pos.tolist()  # Convert to a list if it's a NumPy array
                return self._arm_interface.set_joint_position(joint_command_pos)

        if cmd.__class__.__name__ == "CartesianCommand":
            if self.in_compliant_mode:
                return self._arm_interface.compliant_set_ee_pose(cmd.pos, cmd.quat)
            else:
                return self._arm_interface.set_ee_pose(cmd.pos, cmd.quat)

        if cmd.__class__.__name__ == "OpenGripperCommand":
            return self._arm_interface.open_gripper()

        if cmd.__class__.__name__ == "CloseGripperCommand":
            return self._arm_interface.close_gripper()

        raise NotImplementedError(f"Unrecognized command: {cmd}")

if __name__ == "__main__":

    rospy.init_node("arm_interface_client", anonymous=True)
    arm_client_interface = ArmInterfaceClient()

    _config_path = Path(__file__).parent.parent.parent / "simulation" / "configs" / "vention.yaml"
    config = load_robot_config(str(_config_path))

    run_commands = input("Press 'y' to run commands")

    if run_commands != "y":
        exit()

    # arm_client_interface.execute_command(JointCommand(config.left_retract_pos))
    # arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))

    def pick_plate_from_holder():
        arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))
        arm_client_interface.execute_command(JointCommand(config.behind_intermediate_pos))
        arm_client_interface.execute_command(JointCommand(config.above_plate_holder_pos))
        arm_client_interface.execute_command(CloseGripperCommand())
        arm_client_interface.execute_command(CartesianCommand(config.inside_plate_holder_pose[:3], config.inside_plate_holder_pose[3:]))
        arm_client_interface.execute_command(OpenGripperCommand())
        arm_client_interface.execute_command(CartesianCommand(config.above_plate_holder_pose[:3], config.above_plate_holder_pose[3:]))
        arm_client_interface.execute_command(CartesianCommand(config.intermediate_plate_holder_pose[:3], config.intermediate_plate_holder_pose[3:]))
        arm_client_interface.execute_command(JointCommand(config.behind_intermediate_pos))
        arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))

    def place_plate_in_holder():
        arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))
        arm_client_interface.execute_command(JointCommand(config.behind_intermediate_pos))
        arm_client_interface.execute_command(JointCommand(config.intermediate_plate_holder_pos))
        arm_client_interface.execute_command(JointCommand(config.above_plate_holder_pos))
        arm_client_interface.execute_command(CartesianCommand(config.inside_plate_holder_pose[:3], config.inside_plate_holder_pose[3:]))
        arm_client_interface.execute_command(CloseGripperCommand())
        arm_client_interface.execute_command(CartesianCommand(config.above_plate_holder_pose[:3], config.above_plate_holder_pose[3:]))
        arm_client_interface.execute_command(JointCommand(config.behind_intermediate_pos))
        arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))

    # test_pos = [3.10744480314447, -1.8572371452122223, 1.1964974245416935, -1.594397260375179, -0.30501716596895534, -1.4795676131253597, -0.3509103557698543]
    # test_pose = [0.08398756384849548, -0.26479199528694153, 0.041124988347291946, -0.5071205909168355, 0.4950622048133263, 0.5030556345347846, -0.49464850974842006]

    # test_pos = [3.1058546296575837, -1.8652210358441703, 1.1922856352777331, -1.585627470905548, -0.3061271715113003, -1.4878527072765966, -0.35709528108545463]
    # test_pose = [0.08472398668527603, -0.2637099027633667, 0.038250695914030075, -0.5053791193878973, 0.49333470475951946, 0.5047943064799332, -0.4963824361437406]


    # arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))
    # arm_client_interface.execute_command(JointCommand(config.behind_intermediate_pos))
    # arm_client_interface.execute_command(JointCommand(config.above_plate_holder_pos))
    # arm_client_interface.execute_command(CloseGripperCommand())
    # arm_client_interface.execute_command(CartesianCommand(config.inside_plate_holder_pose[:3], config.inside_plate_holder_pose[3:]))

    # print current state
    state = arm_client_interface.get_state()
    print("Current joint positions:", state["position"])
    print("Current end-effector pose:", state["ee_pos"])
    ee_pose = state["ee_pos"]
    joint_positions = state["position"]

    def pick_plate_from_fridge():
        arm_client_interface.execute_command(JointCommand(config.left_back_retract_pos))
        arm_client_interface.execute_command(JointCommand(config.fridge_contents_gaze_pos))
        arm_client_interface.execute_command(JointCommand(config.left_back_retract_pos))
        arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))
        arm_client_interface.execute_command(JointCommand(config.fridge_inside_intermediate_pos))
        arm_client_interface.execute_command(CartesianCommand(config.fridge_inside_intermediate_pose[:3], config.fridge_inside_intermediate_pose[3:]))
        arm_client_interface.execute_command(CartesianCommand(config.fridge_above_intermediate_pose[:3], config.fridge_above_intermediate_pose[3:]))
        arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))

    # joint_positions = [2.45761645, -1.51664894, -1.86802135, -2.26435991, 0.66374883, -0.25456571, 2.22414458]
    # arm_client_interface.execute_command(JointCommand(config.fridge_inside_intermediate_pos))
    # arm_client_interface.execute_command(CartesianCommand(ee_pose[:3], ee_pose[3:]))
    # arm_client_interface.execute_command(CartesianCommand(config.fridge_above_intermediate_pose[:3], config.fridge_above_intermediate_pose[3:]))
    # arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))
    # arm_client_interface.execute_command(JointCommand(joint_positions))
    # arm_client_interface.execute_command(JointCommand(config.behind_back_retract_pos))
    # pick_plate_from_fridge()

    def pick_plate_from_table():
        arm_client_interface.execute_command(JointCommand(config.back_retract_pos))
        arm_client_interface.execute_command(JointCommand(config.table_gaze_pos))
        arm_client_interface.execute_command(JointCommand(config.table_plate_staging_pos))

    # pick_plate_from_table()

    arm_client_interface.execute_command(JointCommand(config.table_gaze_pos))