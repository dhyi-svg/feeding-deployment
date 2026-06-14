# Description: This script is used to detect ArUco markers and estimate their pose.

# python imports
import os, sys
import cv2
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation

# ros imports
import rospy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import MarkerArray, Marker

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import CartesianCommand, JointCommand, CloseGripperCommand, OpenGripperCommand
from collections import deque

from geometry_msgs.msg import Pose as pose_msg

from feeding_deployment.perception.tf_interface import TFInterface


class DrinkPerception(TFInterface):
    def __init__(self, num_perception_samples=25):
        super().__init__()

        self.num_perception_samples = num_perception_samples
        self.aruco_pose_queues = {0: deque(maxlen=num_perception_samples), 1: deque(maxlen=num_perception_samples)}
        self.aruco_pose_publisher =  rospy.Publisher("/aruco_pose", Pose, queue_size=10)
        self.aruco_pose_publisher_0 = rospy.Publisher("/aruco_pose_0", Pose, queue_size=10)
        self.aruco_pose_publisher_1 = rospy.Publisher("/aruco_pose_1", Pose, queue_size=10)

    def update(self, rgb_image, camera_info_msg, depth_image):
        """Process images and detect ArUco markers. Publishes results on ROS topics."""
        if rgb_image is None or camera_info_msg is None or depth_image is None:
            return

        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        (corners, ids, rejected) = detector.detectMarkers(rgb_image)

        if ids is not None:
            for i in range(len(corners)):
                self._process_aruco_marker(corners[i], ids[i], camera_info_msg, depth_image)
        
    def _process_aruco_marker(self, markerCorner, markerID, camera_info_msg, depth_image):

        assert len(markerID) == 1
        markerID = markerID[0]
        if markerID not in [0, 1]:
            return
        markerCorner = markerCorner.reshape((4, 2))

        landmarks = markerCorner
        landmarks_model = np.array([[-0.04,0.04,0],[0.04,0.04,0],[0.04,-0.04,0],[-0.04,-0.04,0]])

        # convert  2d landmarks to 3d world points 
        valid_landmarks_model = []
        valid_landmarks_world = []
        for i in range(landmarks.shape[0]):
            validity, point = self.pixel2World(camera_info_msg, landmarks[i,0].astype(int), landmarks[i,1].astype(int), depth_image)
            if validity:
                valid_landmarks_model.append(landmarks_model[i])
                valid_landmarks_world.append(point)

        if len(valid_landmarks_world) < 4:
            # print("Not enough landmarks to fit model.")
            return

        valid_landmarks_model = np.array(valid_landmarks_model)
        valid_landmarks_world = np.array(valid_landmarks_world)

        scale_fixed = 1.0
        s, ret_R, ret_t = self.kabschUmeyama(valid_landmarks_world, valid_landmarks_model, scale_fixed)

        # print("landmarks_selected_model[:,:,np.newaxis].shape: ",landmarks_model[:,:,np.newaxis].shape)
        # print("ret_R.shape: ",ret_R.shape)
        landmarks_model_camera_frame = ret_t.reshape(3,1) + s * (ret_R @ landmarks_model[:,:,np.newaxis])
        landmarks_model_camera_frame = np.squeeze(landmarks_model_camera_frame)

        # get things into world frame

        tag_pos = np.append(np.mean(valid_landmarks_world, axis=0), 1) # pad with 1 for homogeneous coordinate

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        if transform is not None:   
            base_to_camera = self.make_homogeneous_transform(transform)

            # cam to tag homogeneous transform
            camera_to_tag = np.zeros((4, 4))
            camera_to_tag[:3, :3] = ret_R
            camera_to_tag[:3, 3] = np.array([ tag_pos[0], tag_pos[1], tag_pos[2] ]).reshape(1, 3)
            camera_to_tag[3, 3] = 1 

            # base to tag homogeneous transform and update tf
            base_to_tag = np.dot(base_to_camera, camera_to_tag)
            self.updateTF("arm_base_link", f"AR_tag_{markerID}", base_to_tag)
            self.update_aruco_pose(base_to_tag, markerID)

    def update_aruco_pose(self, aruco_pose_mat, markerID):
        aruco_pose = self.matrix_to_pose(aruco_pose_mat)
        marker_pub = {
            0: self.aruco_pose_publisher_0,
            1: self.aruco_pose_publisher_1,
        }[markerID]
        aruco_pose_queue = self.aruco_pose_queues[markerID]
        aruco_pose_queue.append(aruco_pose)
        # Wait to update until we have at least num_perception_samples samples.
        if len(aruco_pose_queue) >= self.num_perception_samples:
            running_average_position = np.mean([pose[0] for pose in aruco_pose_queue], axis=0)
            running_average_orientation = np.mean([pose[1] for pose in aruco_pose_queue], axis=0)
            p = Pose()
            p.position.x = running_average_position[0]
            p.position.y = running_average_position[1]
            p.position.z = running_average_position[2]
            p.orientation.x = running_average_orientation[0]
            p.orientation.y = running_average_orientation[1]
            p.orientation.z = running_average_orientation[2]
            p.orientation.w = running_average_orientation[3]
            for publisher in [self.aruco_pose_publisher, marker_pub]:
                publisher.publish(p)

    def pixel2World(self, camera_info, image_x, image_y, depth_image):

        # print("(image_y,image_x): ",image_y,image_x)
        # print("depth image: ", depth_image.shape[0], depth_image.shape[1])

        if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
            return False, None

        depth = depth_image[image_y, image_x]

        if math.isnan(depth) or depth < 0.05 or depth > 1.0:

            depth = []
            for i in range(-2,2):
                for j in range(-2,2):
                    if image_y+i >= depth_image.shape[0] or image_x+j >= depth_image.shape[1]:
                        return False, None
                    pixel_depth = depth_image[image_y+i, image_x+j]
                    if not (math.isnan(pixel_depth) or pixel_depth < 50 or pixel_depth > 1000):
                        depth += [pixel_depth]

            if len(depth) == 0:
                return False, None

            depth = np.mean(np.array(depth))

        depth = depth/1000.0 # Convert from mm to m

        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]  

        # Convert to world space
        world_x = (depth / fx) * (image_x - cx)
        world_y = (depth / fy) * (image_y - cy)
        world_z = depth

        return True, (world_x, world_y, world_z)

    def kabschUmeyama(self, A, B, scale):

        assert A.shape == B.shape

        # print("A Shape:", A.shape)
        # print("B Shape:", B.shape)

        # Calculate scaled B
        scaled_B = B*scale

        # Calculate translation using centroids
        A_centered = A - np.mean(A, axis=0)
        B_centered = scaled_B - np.mean(scaled_B, axis=0)

        # Calculate rotation using scipy

        R, rmsd = Rotation.align_vectors(A_centered, B_centered)

        # print("R: ",R.as_matrix().shape)
        # print("Scaled B: ",np.mean(scaled_B, axis=0).shape)

        t = np.mean(A, axis=0) - R.as_matrix()@np.mean(scaled_B, axis=0)

        return scale, R.as_matrix(), t
    
if __name__ == '__main__':
    rospy.init_node('DrinkPerception')
    drink_perception = DrinkPerception()
    rospy.spin()