'''
Entrypoint for controlling the robot arm on compute machine. Additionally runs two important threads:
1. A thread that checks no safety anomalies have occurred using the watchdog
2. A thread that publishes joint states to ROS
'''

import threading
import time
import numpy as np

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

    run_commands = input("Press 'y' to run commands")

    if run_commands != "y":
        exit()

    left_retract_pos = [-1.57, -0.34903602299465675, -3.141591055693139, -2.0, 0.0, -0.872688061814757, 1.57075917569769]
    arm_client_interface.execute_command(JointCommand(left_retract_pos))

    # fridge_door_gaze_pos = [-0.980015584273823, 0.47420615883552164, -3.015619807706865, -2.207007668035523, 0.7286555882107023, 1.2561937782724737, 1.2283014269116377]
    # arm_client_interface.execute_command(JointCommand(fridge_door_gaze_pos))

    behind_back_retract_pos = [3.141592653589793, -1.8338532592607812, 3.1415681525077646, -2.5482659290666034, 1.0329455279146852e-05, -0.8727280092311087, 1.570780081512247]
    arm_client_interface.execute_command(JointCommand(behind_back_retract_pos))

    # left_retract_pos = [-1.57, -0.34903602299465675, -3.141591055693139, -2.0, 0.0, -0.872688061814757, 1.57075917569769]
    # arm_client_interface.execute_command(JointCommand(left_retract_pos))

    # fridge_contents_gaze_pos = [-2.1760412298284084, 0.563137862775351, 2.71337286190381, -2.4594899867610303, -0.9522223025055379, 1.6603493795065551, 1.836221075785971]
    # arm_client_interface.execute_command(JointCommand(fridge_contents_gaze_pos))

    # left_retract_pos = [-1.57, -0.34903602299465675, -3.141591055693139, -2.0, 0.0, -0.872688061814757, 1.57075917569769]
    # arm_client_interface.execute_command(JointCommand(left_retract_pos))

    # behind_back_retract_pos = [-3.1399148621030433, -1.6540834940938165, -3.1407652094391056, -2.4261370899010957, -0.002468750330502978, -0.8149885462693183, 1.57272552068855]
    # arm_client_interface.execute_command(JointCommand(behind_back_retract_pos))

    # fridge_inside_pos = [2.300764254211763, -1.6941960264325902, -1.93227570366684, -1.8381923476248474, 0.6430000051753475, -0.6125052127972861, 2.4240739390046113]
    # # fridge_inside_pos = [2.2972291741807913, -1.5456267913325572, -2.0362172831159784, -1.765400431865821, 0.5576307785362035, -0.6607787359804984, 2.336141952448484]
    # arm_client_interface.execute_command(JointCommand(fridge_inside_pos))

    # fridge_inside_pose = [-0.26278427243232727, 0.4587034285068512, 0.4376140534877777, 0.4996538765705326, -0.5012942110358767, -0.500239433155195, 0.49880920914366256]
    # arm_client_interface.execute_command(CartesianCommand(fridge_inside_pose[:3], fridge_inside_pose[3:]))

    # behind_back_retract_pose = [-0.08530566096305847, 0.000523008406162262, 0.4735228419303894, -0.5053744744642887, 0.49691096451778954, 0.5021637159852019, -0.4954873724424042]
    # arm_client_interface.execute_command(CartesianCommand(behind_back_retract_pose[:3], behind_back_retract_pose[3:]))

    # microwave_config = [3.129084851214127, -0.8821358920979225, -3.1265742892545094, -2.5615186175991647, -0.006664826944032143, 0.10794081516859698, 1.5825896030500946]
    # arm_client_interface.execute_command(JointCommand(microwave_config))

    # inside_plate_pos = [3.1062098953470003, -1.8586057436964136, 1.195002325905723, -1.6045593504627034, -0.3068414313156591, -1.4701101954622704, -0.34915320208262024]
    # arm_client_interface.execute_command(JointCommand(inside_plate_pos))

    # behind_back_retract_pos = [3.141592653589793, -1.8338532592607812, 3.1415681525077646, -2.5482659290666034, 1.0329455279146852e-05, -0.8727280092311087, 1.570780081512247]
    # inside_plate_pose = [0.08079531788825989, -0.264194130897522, 0.04117746278643608, -0.5071274014359088, 0.49509496077110954, 0.5030325925271225, -0.4946321758512664]
    # above_inside_plate_pos = [3.082130391718831, -1.7038792801561184, 1.3132477435696772, -1.74837857144247, -0.16963377405812263, -1.4230169852741819, -0.22686830167723393]
    # above_inside_plate_pose = inside_plate_pose.copy()
    # above_inside_plate_pose[2] += 0.1
    # microwave_config = [3.129084851214127, -0.8821358920979225, -3.1265742892545094, -2.5615186175991647, -0.006664826944032143, 0.10794081516859698, 1.5825896030500946]
    # intermediate_retract_pos = [-3.1387180375091615, -1.9098678662549133, 3.141273873207311, -2.0296824184331417, -0.012072641853547061, -1.4674305227734328, 1.5728473603084212]

    # arm_client_interface.execute_command(JointCommand(behind_back_retract_pos))
    # arm_client_interface.execute_command(JointCommand(intermediate_retract_pos))
    # arm_client_interface.execute_command(JointCommand(above_inside_plate_pos))
    # arm_client_interface.execute_command(CartesianCommand(inside_plate_pose[:3], inside_plate_pose[3:]))
    # arm_client_interface.execute_command(CartesianCommand(above_inside_plate_pose[:3], above_inside_plate_pose[3:]))
    # arm_client_interface.execute_command(JointCommand(intermediate_retract_pos))
    # arm_client_interface.execute_command(JointCommand(behind_back_retract_pos))
    # arm_client_interface.execute_command(JointCommand(microwave_config))

    # for i in range(5):
    #     input("Press enter to execute inside plate position...")
    #     inside_plate_pose = [0.08079531788825989, -0.264194130897522, 0.04117746278643608, -0.5071274014359088, 0.49509496077110954, 0.5030325925271225, -0.4946321758512664]
    #     arm_client_interface.execute_command(CartesianCommand(inside_plate_pose[:3], inside_plate_pose[3:]))

    #     arm_client_interface.execute_command(CloseGripperCommand())

    #     above_inside_plate_pose = inside_plate_pose.copy()
    #     above_inside_plate_pose[2] += 0.1
    #     arm_client_interface.execute_command(CartesianCommand(above_inside_plate_pose[:3], above_inside_plate_pose[3:]))

    #     behind_back_retract_pos = [3.141592653589793, -1.8338532592607812, 3.1415681525077646, -2.5482659290666034, 1.0329455279146852e-05, -0.8727280092311087, 1.570780081512247]
    #     arm_client_interface.execute_command(JointCommand(behind_back_retract_pos))

    #     above_inside_plate_pos = [3.082130391718831, -1.7038792801561184, 1.3132477435696772, -1.74837857144247, -0.16963377405812263, -1.4230169852741819, -0.22686830167723393]
    #     arm_client_interface.execute_command(JointCommand(above_inside_plate_pos))

    #     arm_client_interface.execute_command(CartesianCommand(inside_plate_pose[:3], inside_plate_pose[3:]))

    #     arm_client_interface.execute_command(OpenGripperCommand())

    #     arm_client_interface.execute_command(CartesianCommand(above_inside_plate_pose[:3], above_inside_plate_pose[3:]))

    #     # behind_back_retract_pos = [3.141592653589793, -1.8338532592607812, 3.1415681525077646, -2.5482659290666034, 1.0329455279146852e-05, -0.8727280092311087, 1.570780081512247]
    #     arm_client_interface.execute_command(JointCommand(behind_back_retract_pos))

    #     # above_inside_plate_pos = [3.082130391718831, -1.7038792801561184, 1.3132477435696772, -1.74837857144247, -0.16963377405812263, -1.4230169852741819, -0.22686830167723393]
    #     arm_client_interface.execute_command(JointCommand(above_inside_plate_pos))

    # behind_back_retract_pos = [3.141592653589793, -1.8338532592607812, 3.1415681525077646, -2.5482659290666034, 1.0329455279146852e-05, -0.8727280092311087, 1.570780081512247]
    # arm_client_interface.execute_command(JointCommand(behind_back_retract_pos))

    # back_home_pos = [3.141592653589793, 0.26193837151853794, -3.1415766746232525, -2.2690119171669654, 4.621619302645493e-06, 0.9598732314221993, 1.5708048489103847]
    # arm_client_interface.execute_command(JointCommand(back_home_pos))

    # left_back_retract_pos = [-1.57, -1.8338532592607812, 3.1415681525077646, -2.5482659290666034, 1.0329455279146852e-05, -0.8727280092311087, 1.570780081512247]
    # arm_client_interface.execute_command(JointCommand(left_back_retract_pos))

    # staging_pos = [3.141592653589793, 0.26193837151853794, -3.1415766746232525, -2.2690119171669654, 4.621619302645493e-06, 0.9598732314221993, 1.5708048489103847]
    # arm_client_interface.execute_command(JointCommand(staging_pos))

    # back_retract_pos = [-2.6098978682220775e-05, -1.8305038015577884, 3.1415657556627834, -2.5482749838143097, 4.621619302645493e-06, -0.8727317376566344, 1.5708057810167664]
    # arm_client_interface.execute_command(JointCommand(back_retract_pos))
    
    # retract_pos = [0.0, -0.34903602299465675, -3.141591055693139, -2.0, 0.0, -0.872688061814757, 1.57075917569769]
    # arm_client_interface.execute_command(JointCommand(retract_pos))

    # midpoint_pos = [2.2912525080624357, 0.730991513381838, 2.0830126187361424, -2.1737367965371632, 0.28532185799581516, -0.4648462461578422, -0.29495787389950756]
    # arm_client_interface.execute_command(JointCommand(midpoint_pos))

    # before_transfer_pos = [-2.86554642, -1.61951779, -2.60986085, -1.37302839, 1.11779249, -1.18028264, 2.05515862]
    # arm_client_interface.execute_command(JointCommand(before_transfer_pos))

    # drink_gaze_pos = [-0.004187021865822871, 0.6034579885210962, -3.1259047705564633, -2.3538005746884725, 0.01149092320739253, 1.3411586039000891, 1.6825233913747728]
    # arm_client_interface.execute_command(JointCommand(drink_gaze_pos))

    # input("Press enter to execute home position...")
    # home_pos = [-2.8762139772986473e-05, 0.26193837151853794, -3.1415766746232525, -2.2690119171669654, 4.621619302645493e-06, 0.9598732314221993, 1.5708048489103847]
    # arm_client_interface.execute_command(JointCommand(home_pos))

    # input("Press enter to execute plate position...")
    # plate_pose = [0.219, -0.264, -0.001, 0.489, -0.491, -0.496, 0.524]
    # arm_client_interface.execute_command(CartesianCommand(plate_pose[:3], plate_pose[3:]))

    # input("Press enter to execute outside plate position...")
    # outside_plate_pose = plate_pose.copy()
    # outside_plate_pose[0] += 0.1
    # arm_client_interface.execute_command(CartesianCommand(outside_plate_pose[:3], outside_plate_pose[3:]))

    # input("Press enter to execute above plate position...")
    # above_plate_pose = plate_pose.copy()
    # above_plate_pose[2] += 0.1
    # arm_client_interface.execute_command(CartesianCommand(above_plate_pose[:3], above_plate_pose[3:]))