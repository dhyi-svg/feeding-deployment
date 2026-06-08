# Description: This script is used to detect ArUco markers and estimate their pose in the camera frame.

# python imports
import os, sys
import cv2
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN   # <-- ADDED
from feeding_deployment.actions.flair.inference_class import FOOD_MODELS_IMPORTS
import open3d as o3d
from pybullet_helpers.geometry import Pose, Pose3D
from copy import deepcopy
from threading import Lock

# ros imports
import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import CartesianCommand, JointCommand, CloseGripperCommand, OpenGripperCommand
from geometry_msgs.msg import TransformStamped
from collections import deque

from feeding_deployment.perception.appliance_perception.remote_molmo import RemoteMolmo

from geometry_msgs.msg import Pose as pose_msg

import supervision as sv
import torch
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms
from groundingdino.util.inference import Model
from segment_anything import sam_model_registry, SamPredictor
PATH_TO_GROUNDED_SAM = '/home/isacc/Grounded-Segment-Anything'

class TFInterface:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()  # Using default cache time of 10 secs
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        time.sleep(1.0)

    def updateTF(self, source_frame, target_frame, pose):

        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = source_frame
        t.child_frame_id = target_frame

        t.transform.translation.x = pose[0][3]
        t.transform.translation.y = pose[1][3]
        t.transform.translation.z = pose[2][3]

        R = Rotation.from_matrix(pose[:3, :3]).as_quat()
        t.transform.rotation.x = R[0]
        t.transform.rotation.y = R[1]
        t.transform.rotation.z = R[2]
        t.transform.rotation.w = R[3]

        self.broadcaster.sendTransform(t)

    def get_frame_to_frame_transform(self, camera_info_data, frame_A = "arm_base_link", target_frame = "camera_color_optical_frame"):
        stamp = camera_info_data.header.stamp
        try:
            transform = self.tfBuffer.lookup_transform(
                frame_A,
                target_frame,
                rospy.Time(secs=stamp.secs, nsecs=stamp.nsecs),
            )
            return transform
        except Exception as e:
            print("Exception finding transform between arm_base_link and", target_frame)
            print("Error:", e)

            return None

    def make_homogeneous_transform(self, transform):
        A_to_B = np.zeros((4, 4))
        A_to_B[:3, :3] = Rotation.from_quat(
            [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
        ).as_matrix()
        A_to_B[:3, 3] = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ]
        ).reshape(1, 3)
        A_to_B[3, 3] = 1

        return A_to_B

    def pose_to_matrix(self, pose):
        position = pose[0]
        orientation = pose[1]
        pose_matrix = np.zeros((4, 4))
        pose_matrix[:3, 3] = position
        pose_matrix[:3, :3] = Rotation.from_quat(orientation).as_matrix()
        pose_matrix[3, 3] = 1
        return pose_matrix
    
    def matrix_to_pose(self, mat):
        position = mat[:3, 3]
        orientation = Rotation.from_matrix(mat[:3, :3]).as_quat()
        return Pose(position, orientation) 


class AppliancePerception(TFInterface):
    def __init__(self, num_perception_samples=25):

        self.DEVICE = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # GroundingDINO config and checkpoint
        self.GROUNDING_DINO_CONFIG_PATH = PATH_TO_GROUNDED_SAM + "/GroundingDINO/groundingdino/config/GroundingDINO_SwinT_OGC.py"
        self.GROUNDING_DINO_CHECKPOINT_PATH = PATH_TO_GROUNDED_SAM + "/groundingdino_swint_ogc.pth"

        print("Initializing Grounding Dino")        
        # Building GroundingDINO inference model
        self.grounding_dino_model = Model(model_config_path=self.GROUNDING_DINO_CONFIG_PATH, model_checkpoint_path=self.GROUNDING_DINO_CHECKPOINT_PATH)

        self.BOX_THRESHOLD = 0.3
        self.TEXT_THRESHOLD = 0.3
        self.NMS_THRESHOLD = 0.4

        # Segment-Anything checkpoint
        SAM_ENCODER_VERSION = "vit_h"
        SAM_CHECKPOINT_PATH = PATH_TO_GROUNDED_SAM + "/sam_vit_h_4b8939.pth"

        print("Initializing SAM")
        # Building SAM Model and SAM Predictor
        sam = sam_model_registry[SAM_ENCODER_VERSION](checkpoint=SAM_CHECKPOINT_PATH)
        sam.to(device=self.DEVICE)
        self.sam_predictor = SamPredictor(sam)

        print("Initializing molmo")
        self.molmo = RemoteMolmo(
            ssh_host="rj277@bhattacharjee-compute-02.coecis.cornell.edu",
            remote_dir="/home/rj277/molmo/sensor_msgs",
        )

        self.turned_on = False
        self.handle_type = None # "bottom white fridge door" or "microwave"
        self.num_perception_samples = num_perception_samples
        self.bridge = CvBridge()

        self.color_image_sub = message_filters.Subscriber(
            '/camera/color/image_raw', Image)
        self.camera_info_sub = message_filters.Subscriber(
            '/camera/color/camera_info', CameraInfo)
        self.depth_image_sub = message_filters.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', Image)
        
        self.handle_points_pub = rospy.Publisher("/handle_points", Marker, queue_size=1)
        self.handle_center_pub = rospy.Publisher("/handle_center", Marker, queue_size=1)

        ts = message_filters.TimeSynchronizer(
            [self.color_image_sub,
             self.camera_info_sub,
             self.depth_image_sub], 1)
        ts.registerCallback(self.rgbdCallback)

        self.camera_lock = Lock()
        self.camera_header = None
        self.camera_color_data = None
        self.camera_info_data = None
        self.camera_depth_data = None
        self.camera_transform = None

        super().__init__()

    def turn_on(self, handle_type: str):
        self.handle_type = handle_type
        self.turned_on = True

    def turn_off(self):
        self.turned_on = False

    def rgbdCallback(self, rgb_image_msg, camera_info_msg, depth_image_msg):

        # if hasattr(self, "saved") and self.saved:
            # return

        if not self.turned_on or self.handle_type is None:
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(
                rgb_image_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                depth_image_msg, "32FC1")
        except CvBridgeError as e:
            print(e)
            return
        
        transform = self.get_frame_to_frame_transform(camera_info_msg)

        with self.camera_lock:
            self.camera_color_data = rgb_image
            self.camera_info_data = camera_info_msg
            self.camera_depth_data = depth_image
            self.camera_header = rgb_image_msg.header
            self.camera_transform = transform

    def get_camera_data(self):
        with self.camera_lock:
            return (
                deepcopy(self.camera_color_data),
                deepcopy(self.camera_info_data),
                deepcopy(self.camera_depth_data),
                deepcopy(self.camera_header),
                deepcopy(self.camera_transform),
            )

    def detect_start_button(self):

        rgb_image, camera_info_msg, depth_image, header, transform = self.get_camera_data()

        file_path = os.path.dirname(__file__)
        print("Got images")
        cv2.imwrite(file_path + "/rgb.png", rgb_image)
        depth_mm = (depth_image * 1000.0).astype("uint16")
        cv2.imwrite(file_path + "/depth.png", depth_mm)

        rgb_image_flipped = cv2.flip(rgb_image.copy(), -1)
        cv2.imwrite(file_path + "/rgb_flipped.png", rgb_image_flipped)

        vis_image, pixel_coords, response = self.molmo.query(
            image_path=file_path + "/rgb_flipped.png",
            prompt="Point to the center of the start / 30 secs button which has a triangle symbol on it.",
            save_response_image_to=file_path + "/rgb_keypoint.png",
        )

        print("Pixel coords from molmo:", pixel_coords)
        # Flip pixel coords back since we flipped the image before sending to molmo
        button_pixel = (rgb_image.shape[1] - pixel_coords[0][0], rgb_image.shape[0] - pixel_coords[0][1])

        # visualize button pixel on original rgb image
        vis_image = rgb_image.copy()
        cv2.circle(vis_image, button_pixel, 10, (0, 0, 255), -1)
        cv2.imwrite(file_path + "/rgb_button_pixel.png", vis_image)

        ok, button_3d = self.pixel2World(camera_info_msg, button_pixel[0], button_pixel[1], depth_image)

        if not ok:
            print("Could not get valid 3D point for button")
            return

        if transform is not None:   
            print("Got transform between arm_base_link and camera_color_optical_frame")
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_button = np.eye(4)
            camera_to_button[:3, 3] = button_3d
            camera_to_button[3, 3] = 1 
            base_to_button = np.dot(base_to_camera, camera_to_button)
            base_to_button[:3, :3] = Rotation.from_quat([-0.5, 0.5, 0.5, -0.5]).as_matrix()
            return self.matrix_to_pose(base_to_button)

        print("Could not get transform between arm_base_link and camera_color_optical_frame")
        return None

    def detect_handle_and_placement(self):

        rgb_image, camera_info_msg, depth_image, header, transform = self.get_camera_data()

        if rgb_image is None:
            print("No camera data yet")
            return None, None, None

        file_path = os.path.dirname(__file__)
        print("Got images")
        cv2.imwrite(file_path + "/rgb.png", rgb_image)
        depth_mm = (depth_image * 1000.0).astype("uint16")
        cv2.imwrite(file_path + "/depth.png", depth_mm)

        detection = self.detect_items(rgb_image, [self.handle_type])

        if detection is None:
            print("No detection")
            return None, None, None

        # create mask using detection
        x1, y1, x2, y2 = detection.astype(int)
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        cv2.imwrite("detection_mask.png", mask)

        center_pixel = ((x1 + x2) // 2, (y1 + y2) // 2)

        # -----------------------------
        # Extract ALL 3D points from mask
        # -----------------------------
        bounding_box_points_3d = []
        pixels = []

        ys, xs = np.where(mask > 0)
        for u, v in zip(xs, ys):
            ok, p = self.pixel2World(
                camera_info_msg, u, v, depth_image)
            if ok:
                bounding_box_points_3d.append(p)
                pixels.append((u, v))

        if len(bounding_box_points_3d) == 0:
            print("-------------- ERROR: No valid 3D points from mask.")
            # rospy.logwarn("No valid 3D points from mask.")
            return

        bounding_box_points_3d = np.array(bounding_box_points_3d)
        pixels = np.array(pixels)

        # Fit a plane to the 3D points to find the main planar surface (fridge door)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(bounding_box_points_3d)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.02,  # 1 cm threshold for inliers
            ransac_n=3,
            num_iterations=500
        )
        outliers = np.setdiff1d(np.arange(len(bounding_box_points_3d)), inliers)

        plane_cloud = pcd.select_by_index(inliers)
        possible_handle_cloud = pcd.select_by_index(inliers, invert=True)

        # visulize plane_cloud on image

        vis = rgb_image.copy()
        for u, v in pixels[inliers]:
            vis[v, u] = (255, 0, 0)
        cv2.imwrite("plane_pixels.png", vis)

        vis = rgb_image.copy()
        for u, v in pixels[outliers]:
            vis[v, u] = (0, 0, 255)
        cv2.imwrite("possible_handle_pixels.png", vis)

        plane_depth = np.median(np.asarray(plane_cloud.points)[:, 2])
        print("Plane depth in m:", plane_depth)

        # remove points from possible_handle_cloud which are behind the plane
        possible_handle_points = np.asarray(possible_handle_cloud.points)
        handle_points = []
        handle_pixels = []
        for i, p in enumerate(possible_handle_points):
            if p[2] < plane_depth and p[2] > plane_depth - 0.07:  # within a reasonable distance infront of the door
                handle_points.append(p)
                handle_pixels.append(pixels[outliers[i]])

        # visualize handle_pixels on image
        vis = rgb_image.copy()
        for u, v in handle_pixels:
            vis[v, u] = (0, 255, 0)
        cv2.imwrite("handle_pixels.png", vis)

        # -----------------------------
        # DBSCAN clustering (7 cm)
        # -----------------------------
        clustering = DBSCAN(
            eps=0.02,  # 30 cm to allow for fridge handles which can be quite large
            min_samples=50
        ).fit(handle_points)

        # print("Ran DBSCAN")

        labels = clustering.labels_
        valid = labels >= 0

        if not np.any(valid):
            # rospy.logwarn("DBSCAN found no clusters.")
            return

        unique, counts = np.unique(labels[valid], return_counts=True)
        main_label = unique[np.argmax(counts)]

        handle_pixels = np.array(handle_pixels)
        handle_points = np.array(handle_points)
        cluster_pixels = handle_pixels[labels == main_label]
        cluster_points_3d = handle_points[labels == main_label]

        top_most_y = np.max(cluster_points_3d[:, 1])

        # for handle_centroid take median in x, y and z to be more robust to outliers

        handle_centroid = np.median(cluster_points_3d, axis=0)
        handle_centroid[1] = top_most_y - 0.04
        # handle_centroid[1] = top_most_y - 0.02 # 2 cm below the top most point in the cluster, which should be close to the center of the handle

        handle_centroid_3d = handle_centroid
        handle_centroid_pixel = self.world2Pixel(camera_info_msg, handle_centroid[0], handle_centroid[1], handle_centroid[2])

        vis = rgb_image.copy()
        for u, v in cluster_pixels:
            vis[v, u] = (0, 255, 255)
        # draw large green circle at handle_centroid_pixel
        print("Handle centroid pixel:", handle_centroid_pixel)
        cv2.circle(vis, (handle_centroid_pixel[0], handle_centroid_pixel[1]), 10, (0, 255, 0), -1)

        # left most point in bounding_box_points_3d with the same y value as handle_centroid_3d is likely the hinge
        # same_y = np.isclose(bounding_box_points_3d[:, 1], handle_centroid_3d[1], atol=0.02)
        
        if self.handle_type == "bottom white fridge door":
            # take a strip of leftmost points on the plane_cloud (0.02 m wide)
            print("Finding strip anchor point for fridge door handle")
            strip_anchor_point = np.min(plane_cloud.points, axis=0)
        else:
            print("Finding strip anchor point for microwave handle")
            strip_anchor_point = np.max(plane_cloud.points, axis=0)

        hinge_strip_points = []
        for p in plane_cloud.points:
            if np.abs(p[0] - strip_anchor_point[0]) < 0.02:
                hinge_strip_points.append(p)
        hinge_strip_points = np.array(hinge_strip_points)
        hinge_idx = np.argmin(np.linalg.norm(hinge_strip_points - handle_centroid_3d, axis=1))
        hinge_3d = hinge_strip_points[hinge_idx]

        # Hack: hinge y is the same as handle_centroid y
        hinge_3d[1] = handle_centroid_3d[1]

        hinge_pixel = self.world2Pixel(camera_info_msg, hinge_3d[0], hinge_3d[1], hinge_3d[2])
        cv2.circle(vis, (hinge_pixel[0], hinge_pixel[1]), 10, (255, 0, 0), -1)
        
        # update center pixel x is average of handle_centroid_pixel and hinge_pixel
        center_pixel = ((handle_centroid_pixel[0] + hinge_pixel[0]) // 2, center_pixel[1])
        cv2.circle(vis, (center_pixel[0], center_pixel[1]), 10, (0, 0, 255), -1)
        cv2.imwrite("handle_hinge_pixels.png", vis)

        ok, center_3d = self.pixel2World(camera_info_msg, center_pixel[0], center_pixel[1], depth_image, depth=plane_depth)
        if not ok:
            print("Could not get valid 3D point for center pixel")
            return

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        if transform is not None:   
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_handle = np.eye(4)
            camera_to_handle[:3, 3] = handle_centroid_3d
            camera_to_handle[3, 3] = 1 
            base_to_handle = np.dot(base_to_camera, camera_to_handle)
            base_to_handle[:3, :3] = Rotation.from_quat([-0.5, 0.5, 0.5, -0.5]).as_matrix()

            camera_to_hinge = np.eye(4)
            camera_to_hinge[:3, 3] = hinge_3d
            camera_to_hinge[3, 3] = 1
            base_to_hinge = np.dot(base_to_camera, camera_to_hinge)
            base_to_hinge[:3, :3] = Rotation.from_quat([-0.5, 0.5, 0.5, -0.5]).as_matrix()

            camera_to_placement = np.eye(4)
            camera_to_placement[:3, 3] = center_3d
            camera_to_placement[3, 3] = 1
            base_to_placement = np.dot(base_to_camera, camera_to_placement)
            base_to_placement[:3, :3] = Rotation.from_quat([-0.5, 0.5, 0.5, -0.5]).as_matrix()

            return self.matrix_to_pose(base_to_handle), self.matrix_to_pose(base_to_hinge), self.matrix_to_pose(base_to_placement)
        
        print("Could not get transform between arm_base_link and camera_color_optical_frame")
        return None, None, None
            
    def detect_items(self, input_image, classes_being_detected, log_path = None):

        # flip image because camera is mounted upside down
        image = cv2.flip(input_image.copy(), -1)
        
        # detect objects
        detections = self.grounding_dino_model.predict_with_classes(
            image=image,
            classes=classes_being_detected,
            box_threshold=self.BOX_THRESHOLD,
            text_threshold=self.TEXT_THRESHOLD
        )
        
        # annotate image with detections
        box_annotator = sv.BoxAnnotator()
        labels = [
            f"{classes_being_detected[class_id]} {confidence:0.2f}" 
            for _, _, confidence, class_id, _, _
            in detections]

        # NMS post process
        #print(f"Before NMS: {len(detections.xyxy)} boxes")
        nms_idx = torchvision.ops.nms(
            torch.from_numpy(detections.xyxy), 
            torch.from_numpy(detections.confidence), 
            self.NMS_THRESHOLD
        ).numpy().tolist()

        # remove boxes which are union of two boxes
        
        detections.xyxy = detections.xyxy[nms_idx]
        detections.confidence = detections.confidence[nms_idx]
        detections.class_id = detections.class_id[nms_idx]
        #print(f"After NMS: {len(detections.xyxy)} boxes")

        annotated_frame = box_annotator.annotate(scene=image.copy(), detections=detections, labels=labels)
        cv2.imwrite("detections_flipped.png", annotated_frame)

        print("Image size:", image.shape)
        print("Detections (flipped):")
        for _, _, confidence, class_id, _, _ in detections:
            print(f"  {classes_being_detected[class_id]}: {confidence:0.2f}")
        print(detections.xyxy)

        # flip back the image and detections to original orientation
        image = cv2.flip(image, -1)
        for i in range(len(detections.xyxy)):
            x1, y1, x2, y2 = detections.xyxy[i]
            detections.xyxy[i] = [image.shape[1] - x2, image.shape[0] - y2, image.shape[1] - x1, image.shape[0] - y1]

        annotated_frame = box_annotator.annotate(scene=image.copy(), detections=detections, labels=labels)
        cv2.imwrite("detections_original.png", annotated_frame)

        print("Image size:", image.shape)
        print("Detections:")
        for _, _, confidence, class_id, _, _ in detections:
            print(f"  {classes_being_detected[class_id]}: {confidence:0.2f}")
        print(detections.xyxy)

        return detections.xyxy[0] if len(detections.xyxy) > 0 else None

    def detect_handle_color(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower = np.array([60, 50, 50])
        upper = np.array([95, 180, 200])
        return cv2.inRange(hsv, lower, upper)

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def pixel2World(self, camera_info, image_x, image_y, depth_image, depth=None):

        # print("Image pixels: ", image_x, image_y)
        # print("Depth shape: ", depth_image.shape)

        if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
            return False, None

        if depth is  None:
            depth = depth_image[image_y, image_x]
            depth = depth / 1000 # convert from mm to m
            # print("Depth: ", depth)

        if math.isnan(depth) or depth < 0.05 or depth > 2.0:
            return False, None

        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5]

        world_x = (depth / fx) * (image_x - cx)
        world_y = (depth / fy) * (image_y - cy)
        world_z = depth

        # print("3D Pixel: ", world_x, world_y, world_z)

        return True, (world_x, world_y, world_z)
    
    def world2Pixel(self, camera_info, world_x, world_y, world_z):

        fx = camera_info.K[0]
        fy = camera_info.K[4]
        cx = camera_info.K[2]
        cy = camera_info.K[5] 

        image_x = world_x * (fx / world_z) + cx
        image_y = world_y * (fy / world_z) + cy

        return int(image_x), int(image_y)


if __name__ == '__main__':
    rospy.init_node('AppliancePerception')
    appliance_perception = AppliancePerception()
    # appliance_perception.turn_on("Start / 30 SEC button") # "bottom white fridge door" or "microwave"
    appliance_perception.turn_on("bottom white fridge door") # "bottom white fridge door" or "microwave"
    while True:
        poses = appliance_perception.detect_handle_and_placement()
        print("Handle pose:", poses[0])
        print("Hinge pose:", poses[1])
        print("Placement pose:", poses[2])
    rospy.spin()
