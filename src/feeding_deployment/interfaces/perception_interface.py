"""An interface for perception (robot joints, human head poses, etc.)."""

import os
import inspect
import threading
import time
from pathlib import Path
import cv2
import numpy as np
from pybullet_helpers.geometry import Pose, Pose3D
from pybullet_helpers.joint import JointPositions
from scipy.spatial.transform import Rotation as R
import json
import pickle
import serial
import copy

LED_SERIAL_PORT = '/dev/serial/by-id/usb-UnexpectedMaker_FeatherS2_Neo_84722E753121-if00'
LED_BAUD_RATE = 115200

try:
    import rospy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String, Bool
    import tf2_ros
    from geometry_msgs.msg import WrenchStamped, Point, Pose as PoseMsg
    from netft_rdt_driver.srv import String_cmd

    from feeding_deployment.interfaces.realsense_interface import RealSenseInterface
    from feeding_deployment.perception.grounded_sam import GroundedSAM
    from feeding_deployment.perception.head_perception.ros_wrapper import HeadPerceptionROSWrapper
    from feeding_deployment.perception.drink_perception.drink_perception import DrinkPerception
    from feeding_deployment.perception.appliance_perception.appliance_perception import AppliancePerception
    from feeding_deployment.perception.attachment_perception.attachment_perception import AttachmentPerception
except ModuleNotFoundError:
    ROSPY_IMPORTED = False

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.utils.camera_utils import CustomCameraInfo

class PerceptionInterface:
    """An interface for perception (robot joints, human head poses, etc.)."""

    def __init__(self, robot_interface: ArmInterfaceClient | None, record_goal_pose: bool = False, simulate_head_perception: bool = False, data_logger=None) -> None:
        self.robot_interface = robot_interface
        self._simulate_head_perception = simulate_head_perception
        # Single logs handle: `.state_dir` is the shared user log directory (used
        # for the perception pose pickles) and the logger captures release images
        # (no-op when disabled). May be None for entry points that don't log.
        self.data_logger = data_logger
        self.log_dir = data_logger.state_dir if data_logger is not None else None

        # run head perception
        if self.robot_interface is None:
            self.simulation = True
            self._head_perception = None
            self._drink_perception = None
            self._appliance_perception = None
            self._attachment_perception = None
            self._realsense = None
            self._grounded_sam = None
        else:
            self.simulation = False
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)

            print("Initializing RealSense interface ...")
            self._realsense = RealSenseInterface(record_goal_pose=record_goal_pose)
            print("RealSense interface initialized")

            print("Initializing shared GroundedSAM ...")
            self._grounded_sam = GroundedSAM()
            print("GroundedSAM initialized")

            print("Initializing head perception ROS wrapper ...")
            self._head_perception = HeadPerceptionROSWrapper(record_goal_pose)
            print("Head perception ROS wrapper initialized")

            print("Initializing drink perception ...")
            self._drink_perception = DrinkPerception()
            print("Initializing handle perception ...")
            self._appliance_perception = AppliancePerception(self._grounded_sam, data_logger=self.data_logger)
            print("Initializing attachment perception ...")
            self._attachment_perception = AttachmentPerception(data_logger=self.data_logger)
            print("Perception interface initialized")

            self._head_perception_warm_started = False

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

        # Rajat ToDo: Support LED
        self.set_led_brightness()

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
        try:
            with serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1) as ser:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                # Convert brightness to string, encode to bytes, and concatenate
                command = f"BRIGHTNESS {brightness}\r\n".encode()
                ser.write(command)
        except (serial.SerialException, OSError) as e:
            print(f"[LED] set_led_brightness failed on {LED_SERIAL_PORT}: {e}")

    def turn_on_led(self):
        if self.simulation:
            return
        try:
            with serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1) as ser:
                ser.reset_input_buffer()  # Clear input buffer
                ser.reset_output_buffer()  # Clear output buffer
                ser.write(b"ON\r\n")  # Send the command
        except (serial.SerialException, OSError) as e:
            print(f"[LED] turn_on_led failed on {LED_SERIAL_PORT}: {e}")

    def turn_off_led(self):
        if self.simulation:
            return
        try:
            with serial.Serial(LED_SERIAL_PORT, LED_BAUD_RATE, timeout=1) as ser:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                ser.write(b"OFF\r\n")
        except (serial.SerialException, OSError) as e:
            print(f"[LED] turn_off_led failed on {LED_SERIAL_PORT}: {e}")

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
        cam_data = self._realsense.get_camera_data()
        camera_color_data = cam_data["rgb_image"]
        camera_info_data = cam_data["camera_info"]
        camera_depth_data = cam_data["depth_image"]
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
                    cam_data = self._realsense.get_camera_data()
                    rgb_image = cam_data["rgb_image"]
                    camera_info = cam_data["camera_info"]
                    depth_image = cam_data["depth_image"]

                    if rgb_image is not None and camera_info is not None and depth_image is not None:
                        transform = self._head_perception.get_base_to_camera_transform(camera_info)
                        if transform is not None:
                            base_to_camera = np.zeros((4, 4))
                            base_to_camera[:3, :3] = R.from_quat([
                                transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w,
                            ]).as_matrix()
                            base_to_camera[:3, 3] = np.array([
                                transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z,
                            ]).reshape(1, 3)
                            base_to_camera[3, 3] = 1

                            if not self._head_perception_warm_started:
                                print("Warming up head perception ...")
                                self._head_perception.set_tool(self.tool if hasattr(self, 'tool') else "fork")
                                for _ in range(10):
                                    self._head_perception.run_head_perception(rgb_image, camera_info, depth_image, base_to_camera, visualize=False)
                                self._head_perception_warm_started = True

                            head_perception_data = self._head_perception.run_head_perception(rgb_image, camera_info, depth_image, base_to_camera)
                        else:
                            head_perception_data = None
                    else:
                        head_perception_data = None
                else:
                    try:
                        with open(self.log_dir / f'head_perception_data_{self.tool}.pkl', 'rb') as f:
                            head_perception_data = pickle.load(f)
                    except FileNotFoundError:
                        raise FileNotFoundError("No transfer logged data found for tool: ", self.tool)
                    time.sleep(0.1)
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
    
    def _terminal_confirmation(self, detection_type: str, vis_image=None) -> bool:
        if vis_image is not None:
            display = vis_image.copy()
            cv2.putText(display, "Press 'y' to confirm, any other key to redo",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow(f"{detection_type} detection", display)
            key = cv2.waitKey(0) & 0xFF
            cv2.destroyAllWindows()
            return key == ord('y')
        response = input(f"Is the {detection_type} detection correct? [y/N]: ").strip().lower()
        return response == "y"

    def perceive_button_pressing_poses(self, web_interface=None):

        if self.simulation:
            with open(self.log_dir / 'button_pressing_pose.pkl', 'rb') as f:
                button_pressing_pose = pickle.load(f)
            handle_poses = button_pressing_pose["last_button_pressing_poses"]

        else:
            while True:
                button_pose = None
                for _ in range(20):
                    cam_data = self._realsense.get_camera_data()
                    rgb_image = cam_data["rgb_image"]
                    camera_info = cam_data["camera_info"]
                    depth_image = cam_data["depth_image"]

                    if rgb_image is not None and camera_info is not None and depth_image is not None:
                        button_pose = self._appliance_perception.detect_start_button(rgb_image, camera_info, depth_image)
                        if button_pose is not None:
                            break
                    time.sleep(0.1)

                if button_pose is None:
                    raise RuntimeError("Could not detect button pressing pose")

                vis_image = self._appliance_perception._last_images.get("rgb_button_pixel")
                if web_interface is None:
                    confirmed = self._terminal_confirmation("button", vis_image)
                else:
                    confirmed = web_interface.get_detection_confirmation("button", vis_image)
                if confirmed:
                    break
                print("Button detection rejected by user. Re-running button perception ...")

            button_transform = self.pose_to_matrix(button_pose)
            offset = np.eye(4)
            offset[:3, 3] = np.array([0.005, 0.0, -0.051])
            press_pose = self.matrix_to_pose(button_transform @ offset)

            pre_press_offset = np.eye(4)
            pre_press_offset[:3, 3] = np.array([0.005, 0.0, -0.12])
            pre_press_pose = self.matrix_to_pose(button_transform @ pre_press_offset)

            intermediate_offset = np.eye(4)
            intermediate_offset[:3, 3] = np.array([0.005, 0.0, -0.08])
            intermediate_pose = self.matrix_to_pose(button_transform @ intermediate_offset)

            return {
                "press_pose": press_pose,
                "pre_press_pose": pre_press_pose,
                "intermediate_pose": intermediate_pose,
            }



    def perceive_handle_opening_poses(self, handle_type: str, web_interface=None):

        # if self.last_handle_poses is not None:
        #     print("Using last handle opening poses from perception cache")
        #     return self.last_handle_poses

        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'handle_opening_pos.pkl', 'rb') as f:
                handle_opening_pos = pickle.load(f)
            handle_poses = handle_opening_pos["last_handle_poses"]

        else:
            while True:
                handle_pose = None
                for _ in range(20):
                    cam_data = self._realsense.get_camera_data()
                    rgb_image = cam_data["rgb_image"]
                    camera_info = cam_data["camera_info"]
                    depth_image = cam_data["depth_image"]

                    if rgb_image is not None and camera_info is not None and depth_image is not None:
                        handle_pose, hinge_pose, placement_pose, top_of_appliance_pose = self._appliance_perception.detect_handle_and_placement(handle_type, rgb_image, camera_info, depth_image)
                        if handle_pose is not None:
                            break
                    time.sleep(0.1)

                if handle_pose is None:
                    raise RuntimeError(f"Could not detect handle opening poses for {handle_type}")

                vis_image = self._appliance_perception._last_images.get("handle_hinge_pixels")
                # Orientation is handled centrally in WebInterface._send_image (the
                # camera is upside down); do not rotate here or it would double-flip.
                if web_interface is None:
                    confirmed = self._terminal_confirmation("handle", vis_image)
                else:
                    confirmed = web_interface.get_detection_confirmation("handle", vis_image)
                if confirmed:
                    break
                print("Handle detection rejected by user. Re-running handle perception ...")

            if handle_type == "microwave":
                top_offset = 0.03
            elif handle_type == "bottom textured fridge door":
                top_offset = 0.05

            offset = np.eye(4)
            offset[:3, 3] = np.array([0, -0.055, 0.0]) # x axis is left, y axis is up, z axis is forward.
            placement_pose = self.matrix_to_pose(self.pose_to_matrix(placement_pose) @ offset)

            # behind placement pose
            offset = np.eye(4)
            offset[:3, 3] = np.array([0, 0, -0.05]) # x axis is left, y axis is up, z axis is forward.
            behind_placement_pose = self.matrix_to_pose(self.pose_to_matrix(placement_pose) @ offset)

            handle_transform = self.pose_to_matrix(handle_pose)
            offset = np.eye(4)
            if handle_type == "bottom textured fridge door":
                offset[:3, 3] = np.array([0.01, 0.0, -0.035]) # x axis is left, y axis is up, z axis is forward. 
            else:
                offset[:3, 3] = np.array([0.0, 0.0, -0.045]) # x axis is left, y axis is up, z axis is forward. 
            grasp_pose = self.matrix_to_pose(handle_transform @ offset)

            pre_grasp_offset = np.eye(4)
            if handle_type == "bottom textured fridge door":
                pre_grasp_offset[:3, 3] = np.array([0.01, 0.0, -0.12]) # x axis is left, y axis is up, z axis is forward. 
            else:
                pre_grasp_offset[:3, 3] = np.array([0.0, 0.0, -0.12])
            pre_grasp_pose = self.matrix_to_pose(handle_transform @ pre_grasp_offset)

            opening_waypoints = self._generate_door_arc_waypoints(
                start_pose=grasp_pose,
                hinge_position=hinge_pose.position,
                arc_length_m=0.55 if handle_type == "microwave" else 0.35,
                waypoint_spacing_m=0.05,
                direction=1 if handle_type == "bottom textured fridge door" else -1, # microwave is left hinged
                rotate_orientation=True,
            )

            will_be_used_for_closing_waypoints = self._generate_door_arc_waypoints(
                start_pose=grasp_pose,
                hinge_position=hinge_pose.position,
                arc_length_m=0.65,
                waypoint_spacing_m=0.05,
                direction=1, 
                rotate_orientation=True,
            )

            # set z of post_release_pose to top_of_appliance to avoid collision with the microwave handle
            post_release_pose = Pose(
                position=(opening_waypoints[-1].position[0], opening_waypoints[-1].position[1], top_of_appliance_pose.position[2] + 0.05),
                orientation=opening_waypoints[-1].orientation,
            )
            # copy.deepcopy(opening_waypoints[-1])
            # offset = np.eye(4)
            # offset[:3, 3] = np.array([0, 0.15, 0])
            # post_release_pose_mat = self.pose_to_matrix(post_release_pose)
            # post_release_pose_mat = post_release_pose_mat @ offset
            # post_release_pose = self.matrix_to_pose(post_release_pose_mat)

            # rotate the sixth-to-last (assuming thickness is 35cm) opening waypoint by 180 degrees so that the gripper can push the door open instead of pulling it
            push_pose = copy.deepcopy(opening_waypoints[-6])
            push_pose_mat = self.pose_to_matrix(push_pose)
            if handle_type == "bottom textured fridge door":
                push_pose_mat[:3, :3] = push_pose_mat[:3, :3] @ R.from_euler("y", -np.pi/2).as_matrix()
            else:
                push_pose_mat[:3, :3] = push_pose_mat[:3, :3] @ R.from_euler("y", np.pi/2).as_matrix()
            push_pose = self.matrix_to_pose(push_pose_mat)

            push_pose = Pose(
                position=(push_pose.position[0], push_pose.position[1], top_of_appliance_pose.position[2]  - top_offset),  
                orientation=push_pose.orientation,
            )
            
            second_waypoints = self._generate_door_arc_waypoints(
                start_pose=push_pose,
                hinge_position=hinge_pose.position,
                arc_length_m=0.5 if handle_type == "microwave" else 0.85, # the microwave is already partially open at the push waypoint
                waypoint_spacing_m=0.05,
                direction=1 if handle_type == "bottom textured fridge door" else -1, # microwave is left hinged
                rotate_orientation=True,
            )
            print("Number of second waypoints: ", len(second_waypoints))
            if handle_type == "microwave":
                push_waypoints = second_waypoints[:-4]
            else:
                push_waypoints = second_waypoints[:-6]
            len_push_waypoints = len(push_waypoints)
            print("Number of push waypoints: ", len_push_waypoints)

            # set z of push_waypoints to top_of_appliance_pose.position[2]  - top_offset 
            for i in range(len(push_waypoints)):
                push_waypoints[i] = Pose(
                    position=(push_waypoints[i].position[0], push_waypoints[i].position[1], top_of_appliance_pose.position[2]  - top_offset),
                    orientation=push_waypoints[i].orientation,
                )

            # pre_push_offset = np.eye(4)
            # pre_push_offset[:3, 3] = np.array([0, 0.15, 0])
            # pre_push_pose_mat = self.pose_to_matrix(push_pose) @ pre_push_offset
            # pre_push_pose = self.matrix_to_pose(pre_push_pose_mat)
            pre_push_pose = Pose(
                position=(push_pose.position[0], push_pose.position[1], top_of_appliance_pose.position[2] + 0.05),
                orientation=push_pose.orientation,
            )

            closing_waypoints = copy.deepcopy(second_waypoints)
            print("Number of closing waypoints: ", len(closing_waypoints))
            closing_waypoints.reverse()

            for i in range(len(closing_waypoints)):
                closing_waypoints[i] = Pose(
                    position=(closing_waypoints[i].position[0], closing_waypoints[i].position[1], top_of_appliance_pose.position[2]  - top_offset),
                    orientation=closing_waypoints[i].orientation,
                )

            closing_waypoint = closing_waypoints[0]

            last_push = Pose(
                position=(push_waypoints[-1].position[0], push_waypoints[-1].position[1], top_of_appliance_pose.position[2] + 0.05),
                orientation=push_waypoints[-1].orientation,
            )
            # last_push = push_waypoints[-1]
            # last_push = self.pose_to_matrix(last_push)
            # offset = np.eye(4)
            # offset[:3, 3] = np.array([0, 0.15, 0])
            # last_push = last_push @ offset
            # last_push = self.matrix_to_pose(last_push)

            above_closing_waypoint = Pose(
                position=(closing_waypoint.position[0], closing_waypoint.position[1], top_of_appliance_pose.position[2] + 0.05),
                orientation=closing_waypoint.orientation,
            )
            # above_closing_waypoint = closing_waypoint
            # above_closing_waypoint_mat = self.pose_to_matrix(above_closing_waypoint)
            # offset = np.eye(4)
            # offset[:3, 3] = np.array([0, 0.15, 0])
            # above_closing_waypoint_mat = above_closing_waypoint_mat @ offset
            # above_closing_waypoint = self.matrix_to_pose(above_closing_waypoint_mat)

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

            temp = copy.deepcopy(second_waypoints[:-3])
            temp.reverse()
            pull_closing_waypoints = []
            for waypoint in temp[:8]:
                waypoint_mat = self.pose_to_matrix(waypoint)
                waypoint_mat[:3, :3] = waypoint_mat[:3, :3] @ R.from_euler("y", -np.pi/2).as_matrix()
                offset = np.eye(4)
                offset[:3, 3] = np.array([0.03, -0.09, -0.14])
                pull_closing_waypoints.append(self.matrix_to_pose(waypoint_mat @ offset))

            pull_closing_waypoint = pull_closing_waypoints[0]

            pre_pull_offset = np.eye(4)
            pre_pull_offset[:3, 3] = np.array([0, 0.0, -0.1])
            pre_pull_pose_mat = self.pose_to_matrix(pull_closing_waypoint) @ pre_pull_offset
            pre_pull_pose = self.matrix_to_pose(pre_pull_pose_mat)

            behind_pull_closing_waypoint = pull_closing_waypoints[-1]
            offset = np.eye(4)
            offset[:3, 3] = np.array([0, 0.0, -0.03])
            behind_pull_closing_waypoint_mat = self.pose_to_matrix(behind_pull_closing_waypoint) @ offset
            behind_pull_closing_waypoint = self.matrix_to_pose(behind_pull_closing_waypoint_mat)

            above_pull_closing_waypoint = pull_closing_waypoints[-1] # last 
            offset = np.eye(4)
            offset[:3, 3] = np.array([0, 0.24, -0.02])
            above_pull_closing_waypoint_mat = self.pose_to_matrix(above_pull_closing_waypoint) @ offset
            above_pull_closing_waypoint = self.matrix_to_pose(above_pull_closing_waypoint_mat)

            push_closing_waypoints = []
            for waypoint in will_be_used_for_closing_waypoints:
                waypoint_mat = self.pose_to_matrix(waypoint)
                waypoint_mat[:3, :3] = waypoint_mat[:3, :3] @ R.from_euler("y", -np.pi/2).as_matrix()
                offset = np.eye(4)
                offset[:3, 3] = np.array([0.02, 0.0, 0.0])
                push_closing_waypoints.append(self.matrix_to_pose(waypoint_mat @ offset))
            push_closing_waypoints.reverse()

            above_push_closing_waypoint = Pose(
                position=(push_closing_waypoints[0].position[0], push_closing_waypoints[0].position[1], top_of_appliance_pose.position[2] + 0.05),
                orientation=push_closing_waypoints[0].orientation,
            )
            # above_push_closing_waypoint = push_closing_waypoints[0]
            # offset = np.eye(4)
            # offset[:3, 3] = np.array([0, 0.15, 0])
            # above_push_closing_waypoint_mat = self.pose_to_matrix(above_push_closing_waypoint) @ offset
            # above_push_closing_waypoint = self.matrix_to_pose(above_push_closing_waypoint_mat)

            # beginning_closing_waypoint = offset_closing_waypoints[0]
            # beginning_closing_waypoint_mat = self.pose_to_matrix(beginning_closing_waypoint)
            # offset = np.eye(4)
            # offset[:3, 3] = np.array([0, 0.0, -0.05])
            # beginning_closing_waypoint_mat = beginning_closing_waypoint_mat @ offset
            # beginning_closing_waypoint = self.matrix_to_pose(beginning_closing_waypoint_mat)

            handle_poses = {
                "placement_pose": placement_pose,
                "behind_placement_pose": behind_placement_pose,
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
                # "pull_pose": pull_pose,
                # "pre_pull_pose": pre_pull_pose,
                "pull_closing_waypoints": pull_closing_waypoints,
                "pull_closing_waypoint": pull_closing_waypoint,
                "pre_pull_pose": pre_pull_pose,
                "behind_pull_closing_waypoint": behind_pull_closing_waypoint,
                "above_pull_closing_waypoint": above_pull_closing_waypoint,
                "above_push_closing_waypoint": above_push_closing_waypoint,
                "push_closing_waypoints": push_closing_waypoints,
            }

        # self.last_handle_poses = handle_poses
        # Save in temp pickle file just for testing, we can remove this later if we don't need it
        if self.log_dir is not None:
            with open(self.log_dir / 'handle_opening_pos.pkl', 'wb') as f:
                pickle.dump({"last_handle_poses": handle_poses}, f)
        self.sync_rviz()

        return handle_poses

    def get_perceived_poses(self):
        if self.log_dir is not None:
            with open(self.log_dir / 'handle_opening_pos.pkl', 'rb') as f:
                handle_opening_pos = pickle.load(f)
            handle_poses = handle_opening_pos["last_handle_poses"]
        else:
            raise ValueError("No log directory provided, cannot load handle opening poses to compute closing poses. Please provide a log directory or run perceive_handle_opening_poses first to save the opening poses.")
        return handle_poses

    def perceive_handle_closing_poses(self, handle_type: str):
        assert handle_type in ["bottom textured fridge door", "microwave"]
        if self.log_dir is not None:
            with open(self.log_dir / 'handle_opening_pos.pkl', 'rb') as f:
                handle_opening_pos = pickle.load(f)
            handle_poses = handle_opening_pos["last_handle_poses"]
        else:
            raise ValueError("No log directory provided, cannot load handle opening poses to compute closing poses. Please provide a log directory or run perceive_handle_opening_poses first to save the opening poses.")
        return handle_poses
        # return self.last_handle_poses

    @staticmethod
    def _rgb_to_hsv_color(r: int, g: int, b: int) -> list:
        """Convert RGB (0-255) to OpenCV HSV [H:0-179, S:0-255, V:0-255]."""
        bgr = np.array([[[b, g, r]]], dtype=np.uint8)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        return hsv[0, 0].tolist()

    def _interactive_color_correction(
        self,
        web_interface,
        rgb_image,
        camera_info,
        depth_image,
        initial_color,
        initial_range: float,
        handle_orientation: str,
        initial_attachment_pose,
        flip=True,
    ):
        """Run the interactive color correction loop on the web interface.

        Returns (confirmed, color, color_range, attachment_pose).
        confirmed=False means the user pressed Back; caller should redo detection.
        flip: pass False when the camera is already upright for this capture (see
        WebInterface._send_image), e.g. the microwave plate pickup.
        """
        current_color = list(initial_color) if hasattr(initial_color, '__iter__') else initial_color
        current_range = float(initial_range)
        last_attachment_pose = initial_attachment_pose

        # Grab a fresh frame at entry for the pick image instead of the (now stale)
        # frame the caller detected on. The RealSense auto-exposure keeps adjusting,
        # so picking on a current, settled frame -- the same regime later Rerun
        # frames come from -- keeps the picked color consistent with detection.
        entry_cam = self._realsense.get_camera_data()
        pick_rgb = entry_cam.get("rgb_image")
        if pick_rgb is None:
            pick_rgb = rgb_image

        # Land the user directly on pixel selection -- no initial detection/result is
        # pre-populated, so they pick a color first instead of having to Reset away
        # from a prediction.
        # Orientation is handled centrally in WebInterface._send_image (the camera
        # is upside down), unless flip=False (camera already upright, e.g. microwave
        # pickup); pass the raw frame through here. The picker returns an RGB color,
        # not a pixel coordinate, so the displayed orientation never affected
        # correctness anyway.
        web_interface.start_color_correction(
            pick_rgb,
            initial_vis_image=None,
            initial_color_range=current_range,
            flip=flip,
        )

        while True:
            msg = web_interface.wait_for_color_correction_message()
            if msg is None:
                return False, initial_color, initial_range, initial_attachment_pose

            status = msg.get("status", "")

            if status in ("rerun", "confirm"):
                r = int(msg.get("r", 0))
                g = int(msg.get("g", 0))
                b = int(msg.get("b", 0))
                current_color = self._rgb_to_hsv_color(r, g, b)
                current_range = float(msg.get("color_range", current_range))

                # Use a fresh camera frame so the TF lookup timestamp is current.
                # Reusing the original camera_info.header.stamp causes TF
                # extrapolation errors once the user has spent >10 s on the page.
                fresh_cam = self._realsense.get_camera_data()
                fresh_rgb   = fresh_cam.get("rgb_image")
                fresh_info  = fresh_cam.get("camera_info")
                fresh_depth = fresh_cam.get("depth_image")
                if fresh_rgb is None or fresh_info is None or fresh_depth is None:
                    fresh_rgb, fresh_info, fresh_depth = rgb_image, camera_info, depth_image

                new_pose = self._attachment_perception.detect_attachment(
                    fresh_rgb, fresh_info, fresh_depth, handle_orientation,
                    handle_color=current_color, color_range=current_range,
                )
                if new_pose is not None:
                    last_attachment_pose = new_pose

                result_vis = self._attachment_perception._last_images.get("attachment_corners")
                # Orientation is handled centrally in WebInterface._send_image (the
                # camera is upside down), unless flip=False (camera already upright);
                # do not rotate here or it would double-flip.
                web_interface.send_color_correction_result(result_vis, new_pose is not None, flip=flip)

                # Refresh the pick image to the frame we just detected on, so the
                # next pick happens on the most recent frame (matching the result the
                # user is looking at) rather than the stale entry frame. Do this on
                # failure too so the user re-picks on the latest frame.
                if fresh_rgb is not None:
                    web_interface.update_color_correction_pick_image(fresh_rgb, flip=flip)

                if status == "confirm":
                    if last_attachment_pose is not None:
                        web_interface.switch_to_explanation_page()
                        return True, current_color, current_range, last_attachment_pose
                    # No pose yet (rerun failed) — stay in loop so user can adjust
                    print("Confirm pressed but no valid detection — ask user to adjust and rerun.")

            elif status == "back":
                return False, initial_color, initial_range, initial_attachment_pose

    def perceive_attachment_poses(self, handle_type: str, handle_color, color_range, handle_orientation: str = "front", web_interface=None, camera_flipped=False):

        # camera_flipped=True when the robot physically flipped its (upside-down) camera
        # for this capture (microwave plate pickup), making the frame already upright --
        # so the user-facing images must NOT get the central 180 deg flip.
        flip = not camera_flipped

        if self.simulation:
            with open(self.log_dir / 'attachment_poses.pkl', 'rb') as f:
                attachment_poses = pickle.load(f)
            # Ensure keys are present for callers that read them.
            attachment_poses.setdefault(
                "handle_color",
                list(handle_color) if hasattr(handle_color, '__iter__') else handle_color,
            )
            attachment_poses.setdefault("color_range", float(color_range))
            return attachment_poses

        current_color = list(handle_color) if hasattr(handle_color, '__iter__') else handle_color
        current_range = float(color_range)

        # Let the RealSense auto-exposure / auto-white-balance settle on this scene
        # before the first detection. The robot has just moved to the detection pose,
        # so the first frames are often mis-lit; detecting on them gives a dark frame
        # that also throws off color picking. This waits once per call (i.e. once per
        # microwave/fridge/table pickup) -- redo retries re-enter the loop below
        # without re-waiting, and color-correction reruns are a separate method.
        CAMERA_SETTLE_SECONDS = 5.0
        time.sleep(CAMERA_SETTLE_SECONDS)

        while True:
            attachment_pose = None
            last_rgb_image = None
            last_camera_info = None
            last_depth_image = None
            for _ in range(20):
                cam_data = self._realsense.get_camera_data()
                rgb_image = cam_data["rgb_image"]
                camera_info = cam_data["camera_info"]
                depth_image = cam_data["depth_image"]

                if rgb_image is not None and camera_info is not None and depth_image is not None:
                    attachment_pose = self._attachment_perception.detect_attachment(
                        rgb_image, camera_info, depth_image, handle_orientation,
                        handle_color=current_color, color_range=current_range,
                    )
                    if attachment_pose is not None:
                        last_rgb_image = rgb_image
                        last_camera_info = camera_info
                        last_depth_image = depth_image
                        break
                time.sleep(0.1)

            if attachment_pose is None:
                raise RuntimeError("Could not detect attachment pose")

            vis_image = self._attachment_perception._last_images.get("attachment_corners")
            # Orientation is handled centrally in WebInterface._send_image (the
            # camera is upside down); do not rotate here or it would double-flip.

            if web_interface is None:
                confirmed = self._terminal_confirmation("attachment", vis_image)
                if confirmed:
                    break
                print("Attachment detection rejected by user. Re-running attachment perception ...")
                continue

            action = web_interface.get_attachment_detection_action("attachment", vis_image, flip=flip)

            if action == "confirm":
                web_interface.switch_to_explanation_page()
                break
            elif action == "redo":
                print("Attachment detection rejected by user. Re-running attachment perception ...")
                continue
            elif action == "correct_color":
                confirmed, current_color, current_range, attachment_pose = self._interactive_color_correction(
                    web_interface,
                    last_rgb_image,
                    last_camera_info,
                    last_depth_image,
                    current_color,
                    current_range,
                    handle_orientation,
                    attachment_pose,
                    flip=flip,
                )
                if confirmed:
                    break
                print("Color correction cancelled. Re-running attachment perception ...")

        offset = np.eye(4)
        if handle_type == "microwave":
            offset[:3, 3] = np.array([0, 0.009, -0.01])
        elif handle_type == "bottom textured fridge door":
            offset[:3, 3] = np.array([0, -0.012, 0.0])
        elif handle_type == "table":
            offset[:3, 3] = np.array([0, -0.008, 0.0])
        else:
            raise ValueError(f"Unknown handle type: {handle_type}")
        pickup_pose = self.matrix_to_pose(self.pose_to_matrix(attachment_pose) @ offset)

        if handle_type == "microwave":
            offset[:3, 3] = np.array([0, 0.009, -0.11])
        elif handle_type == "bottom textured fridge door":
            offset[:3, 3] = np.array([0, -0.012, -0.11])
        elif handle_type == "table":
            offset[:3, 3] = np.array([0, -0.008, -0.11])
        else:
            raise ValueError(f"Unknown handle type: {handle_type}")

        pre_pickup_pose = self.matrix_to_pose(self.pose_to_matrix(attachment_pose) @ offset)

        offset_for_above = np.eye(4)
        offset_for_above[:3, 3] = np.array([0, 0.1, 0.0])
        above_pickup_pose = self.matrix_to_pose(self.pose_to_matrix(pickup_pose) @ offset_for_above)

        attachment_poses = {
            "pickup_pose": pickup_pose,
            "pre_pickup_pose": pre_pickup_pose,
            "above_pickup_pose": above_pickup_pose,
            "handle_color": current_color,
            "color_range": current_range,
        }
        with open(self.log_dir / 'attachment_poses.pkl', 'wb') as f:
            pickle.dump(attachment_poses, f)

        return attachment_poses

    def perceive_sink_placement_poses(self, web_interface=None):

        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'sink_placement_poses.pkl', 'rb') as f:
                sink_placement_poses = pickle.load(f)

        else:
            while True:
                sink_placement_pose = None
                for _ in range(20):
                    cam_data = self._realsense.get_camera_data()
                    rgb_image = cam_data["rgb_image"]
                    camera_info = cam_data["camera_info"]
                    depth_image = cam_data["depth_image"]

                    if rgb_image is not None and camera_info is not None and depth_image is not None:
                        sink_placement_pose = self._appliance_perception.detect_sink_placement(rgb_image, camera_info, depth_image)
                        if sink_placement_pose is not None:
                            break
                    time.sleep(0.1)

                if sink_placement_pose is None:
                    raise RuntimeError("Could not detect sink placement pose")

                vis_image = self._appliance_perception._last_images.get("sink_back_pixel")
                if web_interface is None:
                    confirmed = self._terminal_confirmation("sink", vis_image)
                else:
                    confirmed = web_interface.get_detection_confirmation("sink", vis_image)
                if confirmed:
                    break
                print("Sink placement detection rejected by user. Re-running sink placement perception ...")

            offset = np.eye(4)
            offset[:3, 3] = np.array([0.0, 0.15, -0.45])
            sink_placement_pose = self.matrix_to_pose(self.pose_to_matrix(sink_placement_pose) @ offset)

            sink_placement_poses = {
                "sink_placement_pose": sink_placement_pose,
            }

            with open(self.log_dir / 'sink_placement_poses.pkl', 'wb') as f:
                pickle.dump(sink_placement_poses, f)

        return sink_placement_poses

    def perceive_table_placement_poses(self, web_interface=None):

        if self.simulation:
            # load them from a pickle file
            with open(self.log_dir / 'table_placement_poses.pkl', 'rb') as f:
                table_placement_poses = pickle.load(f)

        else:
            while True:
                table_placement_pose = None
                for _ in range(20):
                    cam_data = self._realsense.get_camera_data()
                    rgb_image = cam_data["rgb_image"]
                    camera_info = cam_data["camera_info"]
                    depth_image = cam_data["depth_image"]

                    if rgb_image is not None and camera_info is not None and depth_image is not None:
                        table_placement_pose = self._appliance_perception.detect_table_placement(rgb_image, camera_info, depth_image)
                        if table_placement_pose is not None:
                            break
                    time.sleep(0.1)

                if table_placement_pose is None:
                    raise RuntimeError("Could not detect table placement pose")

                # detect_table_placement logs the annotated frame (red dot = placement
                # center, blue = surrounding pixels used for depth); read it from memory.
                vis_image = self._appliance_perception._last_images.get("table_placement_pixel")
                # Orientation is handled centrally in WebInterface._send_image (the
                # camera is upside down); do not rotate here or it would double-flip.
                if web_interface is None:
                    confirmed = self._terminal_confirmation("plate", vis_image)
                else:
                    confirmed = web_interface.get_detection_confirmation("plate", vis_image)
                if confirmed:
                    break
                print("Table placement detection rejected by user. Re-running table placement perception ...")

            offset = np.eye(4)
            offset[:3, 3] = np.array([0.0, 0.075, -0.23])
            table_placement_pose = self.matrix_to_pose(self.pose_to_matrix(table_placement_pose) @ offset)

            offset_for_pre_place = np.eye(4)
            offset_for_pre_place[:3, 3] = np.array([0.0, 0.1, 0.0])
            pre_table_placement_pose = self.matrix_to_pose(self.pose_to_matrix(table_placement_pose) @ offset_for_pre_place)

            # behind table placement pose
            offset_for_behind = np.eye(4)
            offset_for_behind[:3, 3] = np.array([0.0, 0.0, -0.1])
            behind_table_placement_pose = self.matrix_to_pose(self.pose_to_matrix(table_placement_pose) @ offset_for_behind)

            table_placement_poses = {
                "table_placement_pose": table_placement_pose,
                "pre_table_placement_pose": pre_table_placement_pose,
                "behind_table_placement_pose": behind_table_placement_pose
            }

            self.last_table_placement_poses = table_placement_poses

            with open(self.log_dir / 'table_placement_poses.pkl', 'wb') as f:
                pickle.dump(table_placement_poses, f)

        return table_placement_poses

    def get_perceived_table_placement_poses(self):
        if self.log_dir is not None:
            with open(self.log_dir / 'table_placement_poses.pkl', 'rb') as f:
                table_placement_poses = pickle.load(f)
            return table_placement_poses
        else:
            raise ValueError("No log directory provided, cannot load table placement poses. Please provide a log directory or run perceive_table_placement_poses first to save the poses.")

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
            aruco_pose_msg = None
            for _ in range(100):
                cam_data = self._realsense.get_camera_data()
                rgb_image = cam_data["rgb_image"]
                camera_info = cam_data["camera_info"]
                depth_image = cam_data["depth_image"]

                if rgb_image is not None and camera_info is not None and depth_image is not None:
                    self._drink_perception.update(rgb_image, camera_info, depth_image)

                try:
                    aruco_pose_msg = rospy.wait_for_message("/aruco_pose_0", PoseMsg, timeout=0.1)
                    break
                except rospy.ROSException:
                    pass

            if aruco_pose_msg is None:
                raise RuntimeError("Could not detect drink pickup pose")

            position = (aruco_pose_msg.position.x, aruco_pose_msg.position.y, aruco_pose_msg.position.z)
            orientation = (aruco_pose_msg.orientation.x, aruco_pose_msg.orientation.y, aruco_pose_msg.orientation.z, aruco_pose_msg.orientation.w)
            self.aruco_pose = (position, orientation)

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
