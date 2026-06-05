"""An interface for perception (robot joints, human head poses, etc.)."""

import threading
import time
from pathlib import Path
import numpy as np
from pybullet_helpers.geometry import Pose, Pose3D
from pybullet_helpers.joint import JointPositions
from scipy.spatial.transform import Rotation as R
import json
import pickle
import serial
import copy

LED_SERIAL_PORT = '/dev/ttyACM0'
LED_BAUD_RATE = 115200

try:
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, Bool
    import tf2_ros
    from geometry_msgs.msg import WrenchStamped, Point, Pose as PoseMsg
    from netft_rdt_driver.srv import String_cmd

    from feeding_deployment.perception.head_perception.ros_wrapper import HeadPerceptionROSWrapper
    from feeding_deployment.perception.drink_perception.drink_perception import DrinkPerception
    from feeding_deployment.perception.handle_perception.handle_perception import HandlePerception
except ModuleNotFoundError:
    ROSPY_IMPORTED = False

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.utils.camera_utils import CustomCameraInfo

class PerceptionInterface:
    """An interface for perception (robot joints, human head poses, etc.)."""

    def __init__(self, robot_interface: ArmInterfaceClient | None, record_goal_pose: bool = False, simulate_head_perception: bool = False, log_dir: str | None = None) -> None:
        self.robot_interface = robot_interface
        self._simulate_head_perception = simulate_head_perception
        self.log_dir = log_dir

        # run head perception
        if self.robot_interface is None:
            self.simulation = True
            self._head_perception = None
            self._drink_perception = None
            self._handle_perception = None
        else:
            self.simulation = False
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)

            print("Initializing head perception ROS wrapper ...")
            self._head_perception = HeadPerceptionROSWrapper(record_goal_pose)
            print("Head perception ROS wrapper initialized")
            
            # warm start head perception only if we're not recording the goal pose
            if not record_goal_pose:
                print("Setting tool to fork")
                self._head_perception.set_tool("fork")
                for _ in range(10):
                    print("Warming up head perception ...")
                    self._head_perception.run_head_perception()

            print("Initializing drink perception ...")
            # Rajat ToDo: pass perception queues to all perception classes instead of having them use ros subscribers which spawn threads
            self._drink_perception = DrinkPerception()
            print("Initializing handle perception ...")
            self._handle_perception = HandlePerception()
            print("Perception interface initialized")

            self.speak_pub = rospy.Publisher('/speak', String, queue_size=1)

            self.transfer_button = False
            self.transfer_button_sub = rospy.Subscriber('/transfer_button', Bool, self.transfer_button_callback)
            
            self.ft_threshold_exceeded = False
            self.ft_sensor_sub = rospy.Subscriber('/forque/forqueSensor', WrenchStamped, self.ft_callback)

        self.head_perception_data_lock = threading.Lock()
        # this term is updated in the run_head_perception method and read in the get_tool_tip_pose method
        self.head_perception_data = None

        # Head perception thread setup
        self.head_perception_thread = None
        self.kill_the_thread = False
        self.head_perception_running = False

        # Head perception data logging setup
        self.log_head_perception_data = []
        self.log_head_perception_start_time = None
        self.log_head_perception = False

        self.last_plate_poses = None
        self.last_drink_poses = None

        # set led brightness
        # self.set_led_brightness()

    def zero_ft_sensor(self):
        print("Zeroing FT sensor")
        # return # not using FT for now
        if self.simulation:
            return
        bias = rospy.ServiceProxy('/forque/bias_cmd', String_cmd)
        bias('bias')
        
    def speak(self, text):
        print("Speaking: ", text)
        if self.simulation:
            return
        self.speak_pub.publish(String(data=text))

    def set_led_brightness(self, brightness: float = 0.2):
        print("Setting LED Brightness")
        if self.simulation:
            return
        with serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            # Convert brightness to string, encode to bytes, and concatenate
            command = f"BRIGHTNESS {brightness}\r\n".encode()
            ser.write(command)

    def turn_on_led(self):
        with serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1) as ser:
            ser.reset_input_buffer()  # Clear input buffer
            ser.reset_output_buffer()  # Clear output buffer
            ser.write(b"ON\r\n")  # Send the command

    def turn_off_led(self):
        with serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(b"OFF\r\n")

    def detect_button_press(self):
        print("Waiting for button press")
        if self.simulation:
            return True
        
        self.transfer_button = False
        # wait for button press
        while not rospy.is_shutdown() and not self.transfer_button:
            time.sleep(0.05)
        self.transfer_button = False
        return True
    
    def detect_force_trigger(self):
        print("Waiting for force torque threshold to be exceeded")
        if self.simulation:
            return True
        
        self.ft_threshold_exceeded = False
        # wait for force torque threshold to be exceeded
        while not rospy.is_shutdown() and not self.ft_threshold_exceeded:
            time.sleep(0.05)
        self.ft_threshold_exceeded = False
        return True

    def ft_callback(self, msg):

        ft_reading = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

        down_torque = ft_reading[3]
        if np.abs(down_torque) > 0.1:
            self.ft_threshold_exceeded = True

    def transfer_button_callback(self, msg):
        print("Transfer button pressed")
        self.transfer_button = True
        
    def get_robot_joints(self) -> "JointState":
        """Get the current robot joint state."""
        joint_state_msg = rospy.wait_for_message("/robot_joint_states", JointState)
        q = np.array(joint_state_msg.position[:7])
        gripper_position = joint_state_msg.position[7]
        
        joint_state = q.tolist() + [
            gripper_position,
            gripper_position,
            gripper_position,
            gripper_position,
            -gripper_position,
            -gripper_position,
        ]
        return joint_state

    def get_camera_data(self):  # Rajat ToDo: Add return type
        camera_color_data, camera_info_data, camera_depth_data, _ = self._head_perception.get_camera_data()
        camera_info = CustomCameraInfo(fx=camera_info_data.K[0], fy=camera_info_data.K[4], cx=camera_info_data.K[2], cy=camera_info_data.K[5])
        return camera_color_data, camera_info, camera_depth_data
    
    def set_head_perception_tool(self, tool: str) -> None:
        """Set the tool for head perception."""
        self.tool = tool
        if self._head_perception is not None:
            self._head_perception.set_tool(tool)

    def head_perception_thread_is_running(self) -> bool:
        return self.head_perception_running

    def start_head_perception_thread(self):
        assert not self.head_perception_running, "Head perception thread is already running" 

        # Start head perception thread
        self.kill_the_thread = False
        self.head_perception_thread = threading.Thread(
            target=self.run_head_perception_thread, args=(), daemon=True
        )
        self.head_perception_thread.start()
        print("Head perception thread started")

    def run_head_perception_thread(self):
        self.head_perception_running = True

        t_init = time.time()
        while not self.kill_the_thread:
            t_now = time.time()
            step_time = t_now - t_init
            if step_time >= 0.02:  # 50 Hz
                if self._head_perception is not None and not self._simulate_head_perception:
                    head_perception_data = self._head_perception.run_head_perception()
                    # print("Head pose: ", head_perception_data["head_pose"])
                else:
                    try:
                        # read from logged data
                        with open(self.log_dir / f'head_perception_data_{self.tool}.pkl', 'rb') as f:
                            head_perception_data = pickle.load(f)
                    except FileNotFoundError:
                        raise FileNotFoundError("No transfer logged data found for tool: ", self.tool)
                    time.sleep(0.1) # Maintain 10 Hz rate that real perception would have
                if self.log_head_perception:
                    self.log_head_perception_data.append((time.time(), head_perception_data))
                with self.head_perception_data_lock:
                    self.head_perception_data = head_perception_data
        self.head_perception_running = False

    def stop_head_perception_thread(self):
        if self.head_perception_running:
            self.kill_the_thread = True
            self.head_perception_thread.join()
            print("Head perception thread stopped")
        else:
            print("Head perception thread is not running")

    def get_head_perception_data(self) -> dict:
        """Get head perception data (head pose, face keypoints, tool tip target pose)."""

        with self.head_perception_data_lock:
            head_perception_data = self.head_perception_data

        # # Just for testing
        # benjamin_tool_tip_target_pose = np.eye(4)
        # benjamin_tool_tip_target_pose[:3, 3] = [-0.282, 0.540, 0.619]
        # benjamin_tool_tip_target_pose[:3, :3] = R.from_quat([-0.490, 0.510, 0.511, -0.489]).as_matrix()
        # head_perception_data["tool_tip_target_pose"] = benjamin_tool_tip_target_pose

        # save them in a pickle file
        if self.robot_interface is not None and self.log_dir is not None and self._simulate_head_perception == False:
            with open(self.log_dir / f'head_perception_data_{self.tool}.pkl', 'wb') as f:
                pickle.dump(head_perception_data, f)
            
        return head_perception_data
        
    def get_tool_tip_pose(self) -> np.ndarray:

        current_state = self.robot_interface.get_state()
        ee_pose = current_state["ee_pos"]

        tool_tip_pose = np.eye(4)
        tool_tip_pose[:3, 3] = ee_pose[:3]
        tool_tip_pose[:3, :3] = R.from_quat(ee_pose[3:]).as_matrix()

        return tool_tip_pose
    
    def get_tool_tip_pose_at_staging(self) -> np.ndarray:

        tool_tip_staging_pose = np.eye(4)

        # Rajat ToDo: Fix these hardcoded values
        if self.tool == "fork":
            tool_tip_staging_pose[:3, 3] = [0.250, 0.272, 0.518]
            tool_tip_staging_pose[:3, :3] = R.from_quat([0.523, -0.503, -0.469, 0.503]).as_matrix()
        elif self.tool == "drink":
            tool_tip_staging_pose[:3, 3] = [0.289, 0.315, 0.587]
            tool_tip_staging_pose[:3, :3] = R.from_quat([0.523, -0.503, -0.469, 0.503]).as_matrix()
        elif self.tool == "wipe":
            tool_tip_staging_pose[:3, 3] = [0.367, 0.277, 0.506]
            tool_tip_staging_pose[:3, :3] = R.from_quat([0.523, -0.503, -0.469, 0.503]).as_matrix()

        return tool_tip_staging_pose

    def getTransformationFromTF(self, source_frame, target_frame):

        while not rospy.is_shutdown():
            try:
                # print("Looking for transform")
                transform = self.tfBuffer.lookup_transform(source_frame, target_frame, rospy.Time())
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.control_rate.sleep()
                continue

        T = np.zeros((4,4))
        T[:3,:3] = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]).as_matrix()
        T[:3,3] = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]).reshape(1,3)
        T[3,3] = 1

        return T
    
    def _generate_door_arc_waypoints(
        self,
        start_pose: Pose,
        hinge_position: Pose3D,
        arc_length_m: float,
        waypoint_spacing_m: float,
        direction: int = 1,
        rotate_orientation: bool = True,
    ) -> list[Pose]:
        """Generate end-effector waypoints along a door-opening arc.

        Assumptions:
        - Door rotates in the xy plane.
        - Hinge axis is vertical (z-axis).
        - Arc is centered at hinge_position[:2].
        """
        if arc_length_m <= 0:
            return []
        if waypoint_spacing_m <= 0:
            raise ValueError("waypoint_spacing_m must be > 0")
        if direction not in (-1, 1):
            raise ValueError("direction must be either +1 or -1")

        hx, hy, hz = start_pose.position
        cx, cy, _ = hinge_position

        radius_vec = np.array([hx - cx, hy - cy], dtype=float)
        radius = np.linalg.norm(radius_vec)
        if radius < 1e-8:
            raise ValueError("Handle pose is too close to hinge pose; radius is ~0.")

        start_theta = np.arctan2(radius_vec[1], radius_vec[0])
        total_angle = arc_length_m / radius
        num_segments = max(1, int(np.ceil(arc_length_m / waypoint_spacing_m)))

        start_rot = R.from_quat(start_pose.orientation)
        waypoints: list[Pose] = []

        for i in range(1, num_segments + 1):
            frac = i / num_segments
            delta_angle = direction * frac * total_angle
            theta = start_theta + delta_angle

            x = cx + radius * np.cos(theta)
            y = cy + radius * np.sin(theta)
            z = hz

            if rotate_orientation:
                yaw_rot = R.from_euler("z", delta_angle)
                orientation = (yaw_rot * start_rot).as_quat()
            else:
                orientation = start_pose.orientation

            waypoints.append(
                Pose(
                    position=(x, y, z),
                    orientation=orientation,
                )
            )

        return waypoints
    
    def perceive_plate_poses(self):

        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'button_pressing_pose.pkl', 'rb') as f:
                button_pressing_pose = pickle.load(f)
            handle_poses = button_pressing_pose["last_button_pressing_poses"]

        else:
            self._handle_perception.turn_on("microwave") # microwave and fridge have the same button, so we can just turn on microwave perception
            # Rajat Hack: Wait three seconds
            time.sleep(3)

            # handle_pose_msg = rospy.wait_for_message("/handle_pose", PoseMsg)
            # hinge_pose_msg = rospy.wait_for_message("/hinge_pose", PoseMsg)
            button_pose_msg = rospy.wait_for_message("/button_pose", PoseMsg)
            self._handle_perception.turn_off()

            button_pose = Pose(
                position=(button_pose_msg.position.x, button_pose_msg.position.y, button_pose_msg.position.z),
                orientation=(button_pose_msg.orientation.x, button_pose_msg.orientation.y, button_pose_msg.orientation.z, button_pose_msg.orientation.w)
            )

            button_transform = self.pose_to_matrix(button_pose)
            offset = np.eye(4)
            offset[:3, 3] = np.array([0.005, -0.027, -0.042]) # x axis is left, y axis is up, z axis is forward. 
            press_pose = self.matrix_to_pose(button_transform @ offset)

            pre_press_offset = np.eye(4)
            pre_press_offset[:3, 3] = np.array([0.005, -0.027, -0.12])
            pre_press_pose = self.matrix_to_pose(button_transform @ pre_press_offset)

            intermediate_offset = np.eye(4)
            intermediate_offset[:3, 3] = np.array([0.005, -0.027, -0.08])
            intermediate_pose = self.matrix_to_pose(button_transform @ intermediate_offset)
            

            return {
                "press_pose": press_pose,
                "pre_press_pose": pre_press_pose,
                "intermediate_pose": intermediate_pose,
            }
    
    def perceive_button_pressing_poses(self):

        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'button_pressing_pose.pkl', 'rb') as f:
                button_pressing_pose = pickle.load(f)
            handle_poses = button_pressing_pose["last_button_pressing_poses"]

        else:
            self._handle_perception.turn_on("microwave") # microwave and fridge have the same button, so we can just turn on microwave perception
            # Rajat Hack: Wait three seconds
            time.sleep(3)

            # handle_pose_msg = rospy.wait_for_message("/handle_pose", PoseMsg)
            # hinge_pose_msg = rospy.wait_for_message("/hinge_pose", PoseMsg)
            button_pose_msg = rospy.wait_for_message("/button_pose", PoseMsg)
            self._handle_perception.turn_off()

            button_pose = Pose(
                position=(button_pose_msg.position.x, button_pose_msg.position.y, button_pose_msg.position.z),
                orientation=(button_pose_msg.orientation.x, button_pose_msg.orientation.y, button_pose_msg.orientation.z, button_pose_msg.orientation.w)
            )

            button_transform = self.pose_to_matrix(button_pose)
            offset = np.eye(4)
            offset[:3, 3] = np.array([0.005, -0.027, -0.042]) # x axis is left, y axis is up, z axis is forward. 
            press_pose = self.matrix_to_pose(button_transform @ offset)

            pre_press_offset = np.eye(4)
            pre_press_offset[:3, 3] = np.array([0.005, -0.027, -0.12])
            pre_press_pose = self.matrix_to_pose(button_transform @ pre_press_offset)

            intermediate_offset = np.eye(4)
            intermediate_offset[:3, 3] = np.array([0.005, -0.027, -0.08])
            intermediate_pose = self.matrix_to_pose(button_transform @ intermediate_offset)
            

            return {
                "press_pose": press_pose,
                "pre_press_pose": pre_press_pose,
                "intermediate_pose": intermediate_pose,
            }



    def perceive_handle_opening_poses(self, handle_type: str):

        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'handle_opening_pos.pkl', 'rb') as f:
                handle_opening_pos = pickle.load(f)
            handle_poses = handle_opening_pos["last_handle_poses"]

        else:
            self._handle_perception.turn_on(handle_type)
            # Rajat Hack: Wait three seconds
            time.sleep(3)

            handle_pose_msg = rospy.wait_for_message("/handle_pose", PoseMsg)
            hinge_pose_msg = rospy.wait_for_message("/hinge_pose", PoseMsg)
            self._handle_perception.turn_off()

            # hack add 0.01 to y as the perception is slightly off

            handle_pose = Pose(
                position=(handle_pose_msg.position.x, handle_pose_msg.position.y, handle_pose_msg.position.z),
                orientation=(handle_pose_msg.orientation.x, handle_pose_msg.orientation.y, handle_pose_msg.orientation.z, handle_pose_msg.orientation.w)
            )

            handle_transform = self.pose_to_matrix(handle_pose)
            offset = np.eye(4)
            offset[:3, 3] = np.array([0.0, 0.0, -0.04]) # x axis is left, y axis is up, z axis is forward. 
            grasp_pose = self.matrix_to_pose(handle_transform @ offset)

            pre_grasp_offset = np.eye(4)
            pre_grasp_offset[:3, 3] = np.array([0.0, 0.0, -0.12])
            pre_grasp_pose = self.matrix_to_pose(handle_transform @ pre_grasp_offset)

            hinge_pose = Pose(
                position=(hinge_pose_msg.position.x, hinge_pose_msg.position.y, hinge_pose_msg.position.z),
                orientation=(hinge_pose_msg.orientation.x, hinge_pose_msg.orientation.y, hinge_pose_msg.orientation.z, hinge_pose_msg.orientation.w)
            )

            opening_waypoints = self._generate_door_arc_waypoints(
                start_pose=grasp_pose,
                hinge_position=hinge_pose.position,
                arc_length_m=0.55,
                waypoint_spacing_m=0.05,
                direction=1 if handle_type == "white fridge door" else -1, # microwave is left hinged
                rotate_orientation=True,
            )

            post_release_pose = copy.deepcopy(opening_waypoints[-1])
            offset = np.eye(4)
            offset[:3, 3] = np.array([0, 0.15, 0])
            post_release_pose_mat = self.pose_to_matrix(post_release_pose)
            post_release_pose_mat = post_release_pose_mat @ offset
            post_release_pose = self.matrix_to_pose(post_release_pose_mat)

            # rotate the sixth-to-last (assuming thickness is 35cm) opening waypoint by 180 degrees so that the gripper can push the door open instead of pulling it
            push_pose = copy.deepcopy(opening_waypoints[-6])
            push_pose_mat = self.pose_to_matrix(push_pose)
            if handle_type == "white fridge door":
                push_pose_mat[:3, :3] = push_pose_mat[:3, :3] @ R.from_euler("y", -np.pi/2).as_matrix()
            else:
                push_pose_mat[:3, :3] = push_pose_mat[:3, :3] @ R.from_euler("y", np.pi/2).as_matrix()
            push_pose = self.matrix_to_pose(push_pose_mat)
            
            second_waypoints = self._generate_door_arc_waypoints(
                start_pose=push_pose,
                hinge_position=hinge_pose.position,
                arc_length_m=0.5 if handle_type == "microwave" else 0.85, # the microwave is already partially open at the push waypoint
                waypoint_spacing_m=0.05,
                direction=1 if handle_type == "white fridge door" else -1, # microwave is left hinged
                rotate_orientation=True,
            )
            print("Number of second waypoints: ", len(second_waypoints))
            push_waypoints = second_waypoints[:-6]
            len_push_waypoints = len(push_waypoints)
            print("Number of push waypoints: ", len_push_waypoints)

            pre_push_offset = np.eye(4)
            pre_push_offset[:3, 3] = np.array([0, 0.15, 0])
            pre_push_pose_mat = self.pose_to_matrix(push_pose) @ pre_push_offset
            pre_push_pose = self.matrix_to_pose(pre_push_pose_mat)

            closing_waypoints = copy.deepcopy(second_waypoints)
            print("Number of closing waypoints: ", len(closing_waypoints))
            closing_waypoints.reverse()

            closing_waypoint = closing_waypoints[0]

            last_push = push_waypoints[-1]
            last_push = self.pose_to_matrix(last_push)
            offset = np.eye(4)
            offset[:3, 3] = np.array([0, 0.15, 0])
            last_push = last_push @ offset
            last_push = self.matrix_to_pose(last_push)

            above_closing_waypoint = closing_waypoint
            above_closing_waypoint_mat = self.pose_to_matrix(above_closing_waypoint)
            offset = np.eye(4)
            offset[:3, 3] = np.array([0, 0.15, 0])
            above_closing_waypoint_mat = above_closing_waypoint_mat @ offset
            above_closing_waypoint = self.matrix_to_pose(above_closing_waypoint_mat)

            more_closing_waypoints = copy.deepcopy(opening_waypoints[:-2])
            more_closing_waypoints.reverse()
            more_closing_waypoints.append(copy.deepcopy(grasp_pose))

            offset_closing_waypoints = []
            offset = np.eye(4)
            # offset[:3, 3] = np.array([0, 0.0, 0.0])
            offset[:3, 3] = np.array([0, 0.0, -0.025])
            for waypoint in more_closing_waypoints:
                waypoint_mat = self.pose_to_matrix(waypoint)
                offset_closing_waypoints.append(self.matrix_to_pose(waypoint_mat @ offset))

            # beginning_closing_waypoint = offset_closing_waypoints[0]
            # beginning_closing_waypoint_mat = self.pose_to_matrix(beginning_closing_waypoint)
            # offset = np.eye(4)
            # offset[:3, 3] = np.array([0, 0.0, -0.05])
            # beginning_closing_waypoint_mat = beginning_closing_waypoint_mat @ offset
            # beginning_closing_waypoint = self.matrix_to_pose(beginning_closing_waypoint_mat)

            handle_poses = {
                "pre_grasp_pose": pre_grasp_pose,
                "grasp_pose": grasp_pose,
                "opening_waypoints": opening_waypoints,
                "post_release_pose": post_release_pose,
                "pre_push_pose": pre_push_pose,
                "push_pose": push_pose,
                "push_waypoints": push_waypoints,
                "before_above_closing_waypoint": last_push,
                "above_closing_waypoint": above_closing_waypoint,
                "closing_waypoint": closing_waypoint,
                "closing_waypoints": closing_waypoints,
                # "beginning_closing_waypoint": beginning_closing_waypoint,
                "offset_closing_waypoints": offset_closing_waypoints,
            }

        self.last_handle_poses = handle_poses
        self.sync_rviz()

        return handle_poses

    def perceive_handle_closing_poses(self, handle_type: str):
        assert handle_type in ["white fridge door", "microwave"]
        return self.last_handle_poses

    def perceive_drink_pickup_poses(self):

        def get_drink_transform():
            tf = np.zeros((4, 4))
            tf[:3, :3] = R.from_euler("xyz", [0, 0, np.pi / 2]).as_matrix()
            tf[:3, 3] = np.array([0.0, 0.0, 0.0]) 
            tf[3, 3] = 1
            return tf

        def get_pre_grasp_transform():
            tf = np.zeros((4, 4))
            tf[:3, :3] = R.from_euler("xyz", [np.pi, 0, np.pi / 2]).as_matrix()
            tf[:3, 3] = np.array([0.09, -0.02, 0.1]) 
            tf[3, 3] = 1
            return tf

        def get_inside_bottom_transform():
            tf = get_pre_grasp_transform()
            tf[2, 3] = 0.017
            return tf

        def get_inside_top_transform():
            tf = get_inside_bottom_transform()
            tf[0, 3] = 0.14
            return tf
        
        def get_post_grasp_pose():
            tf = get_inside_top_transform()
            tf[0, 3] = 0.32
            return tf
        
        def get_place_inside_bottom_transform():
            tf = get_inside_bottom_transform()
            # tf[1, 3] = 0.0
            return tf

        def get_place_pre_grasp_transform():
            tf = get_pre_grasp_transform()
            # tf[2, 3] = 0.25
            # tf[1, 3] = 0.0
            return tf
                
        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'drink_pickup_pos.pkl', 'rb') as f:
                drink_pickup_pos = pickle.load(f)
            drink_poses = drink_pickup_pos["last_drink_poses"]

        else:
            self._drink_perception.turn_on()
            # Rajat Hack: Wait one second for the aruco mean to be correct, does this actually help though?
            time.sleep(3)

            aruco_pose_msg = rospy.wait_for_message("/aruco_pose_0", PoseMsg)
            position = (aruco_pose_msg.position.x, aruco_pose_msg.position.y, aruco_pose_msg.position.z)
            orientation = (aruco_pose_msg.orientation.x, aruco_pose_msg.orientation.y, aruco_pose_msg.orientation.z, aruco_pose_msg.orientation.w)
            self.aruco_pose = (position, orientation)
            self._drink_perception.turn_off()

            drink_poses  = {}
            drink_poses['drink_pose'] = self.get_aruco_relative_pose(get_drink_transform(), "drink")
            drink_poses['pre_grasp_pose'] = self.get_aruco_relative_pose(get_pre_grasp_transform(), "drink")
            drink_poses['inside_bottom_pose'] = self.get_aruco_relative_pose(get_inside_bottom_transform(), "drink")
            drink_poses['inside_top_pose'] = self.get_aruco_relative_pose(get_inside_top_transform(), "drink")
            drink_poses['post_grasp_pose'] = self.get_aruco_relative_pose(get_post_grasp_pose(), "drink")
            drink_poses['place_inside_bottom_pose'] = self.get_aruco_relative_pose(get_place_inside_bottom_transform(), "drink")
            drink_poses['place_pre_grasp_pose'] = self.get_aruco_relative_pose(get_place_pre_grasp_transform(), "drink")

        self.last_drink_poses = drink_poses
        self.sync_rviz()

        return drink_poses
    
    def record_drink_pickup_joint_pos(self):
        if self.simulation:
            return
        
        self.drink_pickup_joint_pos = self.get_robot_joints()[:7]
        # save them in a pickle file
        drink_pickup_pos = {
            "last_drink_poses": self.last_drink_poses,
            "drink_pickup_joint_pos": self.drink_pickup_joint_pos
        }
        with open(self.log_dir / 'drink_pickup_pos.pkl', 'wb') as f:
            pickle.dump(drink_pickup_pos, f)
        print("Drink pickup poses recorded")

    def get_last_drink_pickup_configs(self, study_poses = False):
        assert not study_poses
        last_drink_poses = self.last_drink_poses
        try:
            drink_pickup_joint_pos = self.drink_pickup_joint_pos
        except Exception as e:
            drink_pickup_joint_pos = None
        
        return last_drink_poses, drink_pickup_joint_pos

    def get_aruco_relative_pose(self, transform, override_angles = ""):
        aruco_pos_mat = self.pose_to_matrix(self.aruco_pose)
        goal_frame = np.dot(aruco_pos_mat, transform)
        goal_pose = self.matrix_to_pose(goal_frame)

        # If true, use 2 hardcoded angle values.
        if override_angles == "drink":
            rot = R.from_quat(goal_pose[1])
            roll = np.pi / 2
            pitch = 0
            _, _, yaw = rot.as_euler("xyz")
            new_rot = R.from_euler("xyz", [roll, pitch, yaw])
            goal_pose = Pose(goal_pose[0], new_rot.as_quat())
        elif override_angles == "plate":
            rot = R.from_quat(goal_pose[1])
            roll = np.pi
            pitch = 0
            _, _, yaw = rot.as_euler("xyz")
            new_rot = R.from_euler("xyz", [roll, pitch, yaw])
            goal_pose = Pose(goal_pose[0], new_rot.as_quat())
        elif override_angles == "plate-pose":
            rot = R.from_quat(goal_pose[1])
            roll = 0
            pitch = 0
            _, _, yaw = rot.as_euler("xyz")
            new_rot = R.from_euler("xyz", [roll, pitch, yaw + np.pi])
            goal_pose = Pose(goal_pose[0], new_rot.as_quat())
        
        return goal_pose

    def pose_to_matrix(self, pose):
        position = pose[0]
        orientation = pose[1]
        pose_matrix = np.zeros((4, 4))
        pose_matrix[:3, 3] = position
        pose_matrix[:3, :3] = R.from_quat(orientation).as_matrix()
        pose_matrix[3, 3] = 1
        return pose_matrix
    
    def matrix_to_pose(self, mat):
        position = mat[:3, 3]
        orientation = R.from_matrix(mat[:3, :3]).as_quat()
        return Pose(position, orientation) 
    
    def start_logging_head_perception(self):
        assert self.head_perception_running, "Head perception thread should be running to start logging"
        self.log_head_perception_data = []
        self.log_head_perception_start_time = time.time()
        self.log_head_perception = True
        return self.log_head_perception_start_time

    def stop_logging_head_perception(self):
        assert self.head_perception_running, "Head perception thread should be running to stop logging"
        self.log_head_perception = False

    def delete_logged_head_perception_data(self):
        import gc
        self.log_head_perception_data = []
        self.log_head_perception_start_time = None
        gc.collect()
    
    def extract_from_logged_head_perception_data(self, timestamp):

        # add three second to timestamp[0]
        print("data segment true start time: ", timestamp[0])
        print("data segment true end time: ", timestamp[1])
        start_time = timestamp[0] + 3
        end_time = timestamp[1] - 3
        
        data_segment = {
            "head_pose": [],
            "face_keypoints": [],
            "tool_tip_target_pose": [],
            "timestamp": []
        }

        video_segment = []

        for (timestamp, head_perception_data) in self.log_head_perception_data:
            if timestamp >= start_time and timestamp <= end_time:
                if head_perception_data is None:
                    continue
                data_segment["head_pose"].append(head_perception_data["head_pose"])
                data_segment["face_keypoints"].append(head_perception_data["face_keypoints"])
                data_segment["tool_tip_target_pose"].append(head_perception_data["tool_tip_target_pose"])
                data_segment["timestamp"].append(timestamp)
                video_segment.append(head_perception_data["camera_color_data"])

        return data_segment, video_segment
    
    def perceive_plate_pickup_poses(self):

        def get_plate_transform():
            tf = np.zeros((4, 4))
            tf[:3, :3] = R.from_euler("xyz", [0, 0, np.pi]).as_matrix()
            tf[:3, 3] = np.array([0.05, 0.15, 0.0]) 
            tf[3, 3] = 1
            return tf

        def get_pre_grasp_transform():
            tf = np.zeros((4, 4))
            tf[:3, :3] = R.from_euler("xyz", [np.pi, 0, np.pi]).as_matrix()
            tf[:3, 3] = np.array([0.07, -0.01, 0.1]) 
            tf[3, 3] = 1
            return tf

        def get_inside_bottom_transform():
            tf = get_pre_grasp_transform()
            tf[2, 3] = 0.025
            return tf

        def get_inside_top_transform():
            tf = get_inside_bottom_transform()
            tf[1, 3] = -0.025
            return tf
        
        def get_post_grasp_pose():
            tf = get_inside_top_transform()
            tf[2, 3] = 0.05
            return tf
        
        def get_place_inside_bottom_transform():
            tf = get_inside_bottom_transform()
            # tf[1, 3] = 0.0
            return tf

        def get_place_pre_grasp_transform():
            tf = get_pre_grasp_transform()
            # tf[1, 3] = 0.0
            return tf
        
        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'plate_pickup_pos.pkl', 'rb') as f:
                plate_pickup_pos = pickle.load(f)
            plate_poses = plate_pickup_pos["last_plate_poses"]

        else:
            # Rajat Hack: Wait one second for the aruco mean to be correct, does this actually help though?
            time.sleep(3)

            aruco_pose_msg = rospy.wait_for_message("/aruco_pose_1", PoseMsg)
            # save in pickle file
            with open(self.log_dir / 'aruco_pose.pkl', 'wb') as f:
                pickle.dump(aruco_pose_msg, f)
            
            position = (aruco_pose_msg.position.x, aruco_pose_msg.position.y, aruco_pose_msg.position.z)
            orientation = (aruco_pose_msg.orientation.x, aruco_pose_msg.orientation.y, aruco_pose_msg.orientation.z, aruco_pose_msg.orientation.w)
            self.aruco_pose = (position, orientation)

            plate_poses  = {}
            plate_poses['plate_pose'] = self.get_aruco_relative_pose(get_plate_transform(), override_angles="plate-pose")
            plate_poses['pre_grasp_pose'] = self.get_aruco_relative_pose(get_pre_grasp_transform(), "plate")
            plate_poses['inside_bottom_pose'] = self.get_aruco_relative_pose(get_inside_bottom_transform(), "plate")
            plate_poses['inside_top_pose'] = self.get_aruco_relative_pose(get_inside_top_transform(), "plate")
            plate_poses['post_grasp_pose'] = self.get_aruco_relative_pose(get_post_grasp_pose(), "plate")
            plate_poses['place_inside_bottom_pose'] = self.get_aruco_relative_pose(get_place_inside_bottom_transform(), "plate")
            plate_poses['place_pre_grasp_pose'] = self.get_aruco_relative_pose(get_place_pre_grasp_transform(), "plate")

        self.last_plate_poses = plate_poses
        self.sync_rviz()

        return plate_poses
    
    def record_plate_pickup_joint_pos(self):
        if self.simulation:
            return
        
        self.plate_pickup_joint_pos = self.get_robot_joints()[:7]
        # save them in a pickle file
        plate_pickup_pos = {
            "last_plate_poses": self.last_plate_poses,
            "plate_pickup_joint_pos": self.plate_pickup_joint_pos
        }
        with open(self.log_dir / 'plate_pickup_pos.pkl', 'wb') as f:
            pickle.dump(plate_pickup_pos, f)
        print("Plate pickup poses recorded")

    def get_last_plate_pickup_configs(self, study_poses = False):
        if study_poses:
            with open(Path(__file__).parent.parent / 'integration' / 'log' / 'study_pickup_pickup_pos.pkl', 'rb') as f:
                plate_pickup_pos = pickle.load(f)
            last_plate_poses = plate_pickup_pos["last_plate_poses"]
            # plate_pickup_joint_pos = plate_pickup_pos["plate_pickup_joint_pos"]
        else:
            last_plate_poses = self.last_plate_poses
        
        return last_plate_poses

    def sync_rviz(self):
        if self.last_plate_poses:
            plate_poses = self.last_plate_poses
            self._drink_perception.updateTF("arm_base_link", "plate", self.pose_to_matrix(plate_poses['plate_pose']))
            self._drink_perception.updateTF("arm_base_link", "plate_pre", self.pose_to_matrix(plate_poses['pre_grasp_pose']))
        if self.last_drink_poses:
            drink_poses = self.last_drink_poses
            self._drink_perception.updateTF("arm_base_link", "drink", self.pose_to_matrix(drink_poses['drink_pose']))
            self._drink_perception.updateTF("arm_base_link", "drink_pre", self.pose_to_matrix(drink_poses['pre_grasp_pose']))
