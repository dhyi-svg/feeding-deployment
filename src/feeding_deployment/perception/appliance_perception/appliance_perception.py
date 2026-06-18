# Description: This script is used to detect appliance handles and placement poses.

# python imports
import os, sys
import cv2
import numpy as np
import time
import math
import requests
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN
import open3d as o3d
from pybullet_helpers.geometry import Pose, Pose3D

# ros imports
import rospy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import CartesianCommand, JointCommand, CloseGripperCommand, OpenGripperCommand
from collections import deque

from feeding_deployment.perception.tf_interface import TFInterface
from feeding_deployment.perception.grounded_sam import GroundedSAM

from geometry_msgs.msg import Pose as pose_msg

import supervision as sv
import torch
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms 


class AppliancePerception(TFInterface):
    def __init__(self, grounded_sam: GroundedSAM, num_perception_samples=25):
        super().__init__()

        self.grounding_dino_model = grounded_sam.grounding_dino_model
        self.sam_predictor = grounded_sam.sam_predictor

        self.BOX_THRESHOLD = 0.3
        self.TEXT_THRESHOLD = 0.3
        self.NMS_THRESHOLD = 0.4

        self.molmo_url = "https://ace7-128-84-97-177.ngrok-free.app/predict"

        self.handle_type = None
        self.num_perception_samples = num_perception_samples

        self.handle_points_pub = rospy.Publisher("/handle_points", Marker, queue_size=1)
        self.handle_center_pub = rospy.Publisher("/handle_center", Marker, queue_size=1)

    def detect_start_button(self, rgb_image, camera_info_msg, depth_image):
        transform = self.get_frame_to_frame_transform(camera_info_msg)

        file_path = os.path.dirname(__file__)
        print("Got images")
        cv2.imwrite(file_path + "/rgb.png", rgb_image)
        depth_mm = (depth_image * 1000.0).astype("uint16")
        cv2.imwrite(file_path + "/depth.png", depth_mm)

        rgb_image_flipped = cv2.flip(rgb_image.copy(), -1)
        cv2.imwrite(file_path + "/rgb_flipped.png", rgb_image_flipped)

        with open(file_path + "/rgb_flipped.png", "rb") as img_file:
            http_response = requests.post(
                self.molmo_url,
                files={"image": img_file},
                data={"prompt": "Point to the center of the start / 30 secs button. Right one out of two rectangular buttons at the bottom row of the microwave control panel."},
            )
        http_response.raise_for_status()
        response = http_response.json()
        print("Molmo HTTP response:", response)
        pixel_coords = response.get("pixel_coords", [])

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

    def detect_handle_and_placement(self, handle_type, rgb_image, camera_info_msg, depth_image):
        if rgb_image is None:
            print("No camera data provided")
            return None, None, None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        file_path = os.path.dirname(__file__)
        print("Got images")
        cv2.imwrite(file_path + "/rgb.png", rgb_image)
        depth_mm = (depth_image * 1000.0).astype("uint16")
        cv2.imwrite(file_path + "/depth.png", depth_mm)

        detection = self.detect_items(rgb_image, [handle_type])

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
        if handle_type == "bottom fridge door":
            handle_centroid[1] = top_most_y - 0.07
        else:
            handle_centroid[1] = top_most_y - 0.04
        # handle_centroid[1] = top_most_y - 0.02 # 2 cm below the top most point in the cluster, which should be close to the center of the handle

        # find top of the plane_cloud (not handle) just above handle
        top_of_appliance = handle_centroid.copy()
        top_of_appliance[1] = np.max(np.asarray(plane_cloud.points)[:, 1])
        top_of_appliance_pixel = self.world2Pixel(camera_info_msg, top_of_appliance[0], top_of_appliance[1], top_of_appliance[2])

        handle_centroid_3d = handle_centroid
        handle_centroid_pixel = self.world2Pixel(camera_info_msg, handle_centroid[0], handle_centroid[1], handle_centroid[2])

        vis = rgb_image.copy()
        # for u, v in cluster_pixels:
        #     vis[v, u] = (0, 255, 255)

        # draw large green circle at handle_centroid_pixel
        print("Handle centroid pixel:", handle_centroid_pixel)
        cv2.circle(vis, (handle_centroid_pixel[0], handle_centroid_pixel[1]), 10, (0, 255, 0), -1)

        # draw large orange circle at top_of_appliance_pixel
        print("Top of plane pixel:", top_of_appliance_pixel)
        cv2.circle(vis, (top_of_appliance_pixel[0], top_of_appliance_pixel[1]), 10, (0, 165, 255), -1)

        if handle_type == "bottom fridge door":
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

            camera_to_top_of_appliance = np.eye(4)
            camera_to_top_of_appliance[:3, 3] = top_of_appliance
            camera_to_top_of_appliance[3, 3] = 1
            base_to_top_of_appliance = np.dot(base_to_camera, camera_to_top_of_appliance)
            base_to_top_of_appliance[:3, :3] = Rotation.from_quat([-0.5, 0.5, 0.5, -0.5]).as_matrix()

            return self.matrix_to_pose(base_to_handle), self.matrix_to_pose(base_to_hinge), self.matrix_to_pose(base_to_placement), self.matrix_to_pose(base_to_top_of_appliance)
        
        print("Could not get transform between arm_base_link and camera_color_optical_frame")
        return None, None, None, None

    def detect_sink_placement(self, rgb_image, camera_info_msg, depth_image):
        if rgb_image is None:
            print("No camera data provided")
            return None, None, None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        file_path = os.path.dirname(__file__)
        print("Got images")
        cv2.imwrite(file_path + "/rgb.png", rgb_image)
        depth_mm = (depth_image * 1000.0).astype("uint16")
        cv2.imwrite(file_path + "/depth.png", depth_mm)

        detection = self.detect_items(rgb_image, ["sink basin tap"])

        if detection is None:
            print("No detection")
            return None, None, None

        # create mask using detection
        x1, y1, x2, y2 = detection.astype(int)
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        cv2.imwrite("detection_mask.png", mask)

        # Hack, take a point with x as center of bounding box and y as 40 pixels above the top of the bounding box 
        center_pixel = ((x1 + x2) // 2 + 140, y1 - 50)
        cv2.circle(rgb_image, center_pixel, 10, (0, 0, 255), -1)
        cv2.imwrite("sink_back_pixel.png", cv2.rotate(rgb_image, cv2.ROTATE_180))

        ok, center_3d = self.pixel2World(camera_info_msg, center_pixel[0], center_pixel[1], depth_image)
        if not ok:
            print("Could not get valid 3D point for sink placement")

        if transform is not None:   
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_sink = np.eye(4)
            camera_to_sink[:3, 3] = center_3d
            camera_to_sink[3, 3] = 1
            base_to_sink = np.dot(base_to_camera, camera_to_sink)
            base_to_sink[:3, :3] = Rotation.from_quat([0.5, 0.5, 0.5, 0.5]).as_matrix()
            return self.matrix_to_pose(base_to_sink)
        
        print("Could not get transform between arm_base_link and camera_color_optical_frame")
        return None

    def detect_table_placement(self, rgb_image, camera_info_msg, depth_image):
        if rgb_image is None:
            print("No camera data provided")
            return None, None, None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        file_path = os.path.dirname(__file__)
        print("Got images")
        cv2.imwrite(file_path + "/rgb.png", rgb_image)
        depth_mm = (depth_image * 1000.0).astype("uint16")
        cv2.imwrite(file_path + "/depth.png", depth_mm)

        detection = self.detect_items(rgb_image, ["white circle on table"])

        if detection is None:
            print("No detection")
            return None, None, None

        # create mask using detection
        x1, y1, x2, y2 = detection.astype(int)
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        cv2.imwrite("detection_mask.png", mask)

        center_pixel = ((x1 + x2) // 2, (y1 + y2) // 2)

        # mark all "surrounding pixels" which will be used for depth estimation as well
        pixel_range = 70
        for dy in range(-pixel_range, pixel_range + 1):
                for dx in range(-pixel_range, pixel_range + 1):
                    new_y = center_pixel[1] + dy
                    new_x = center_pixel[0] + dx
                    if 0 <= new_x < rgb_image.shape[1] and 0 <= new_y < rgb_image.shape[0]:
                        cv2.circle(rgb_image, (new_x, new_y), 2, (255, 0, 0), -1)

        cv2.circle(rgb_image, center_pixel, 10, (0, 0, 255), -1)
        cv2.imwrite("table_placement_pixel.png", rgb_image)

        ok, center_3d = self.pixel2World(camera_info_msg, center_pixel[0], center_pixel[1], depth_image, use_surrounding_pixels=True)
        if not ok:
            print("Could not get valid 3D point for table placement")

        if transform is not None:   
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_center = np.eye(4)
            camera_to_center[:3, 3] = center_3d
            camera_to_center[3, 3] = 1
            base_to_center = np.dot(base_to_camera, camera_to_center)
            base_to_center[:3, :3] = Rotation.from_quat([0.0, 0.707, 0.707, 0.0]).as_matrix()
            return self.matrix_to_pose(base_to_center)
        
        print("Could not get transform between arm_base_link and camera_color_optical_frame")
        return None
            
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
        labels = [labels[i] for i in nms_idx]
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

    def pixel2World(self, camera_info, image_x, image_y, depth_image, depth=None, use_surrounding_pixels=False):

        # print("Image pixels: ", image_x, image_y)
        # print("Depth shape: ", depth_image.shape)

        if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
            return False, None

        if depth is None:
            depth = depth_image[image_y, image_x]
            depth = depth / 1000 # convert from mm to m
            # print("Depth: ", depth)

        if math.isnan(depth) or depth < 0.05 or depth > 2.0:
            if use_surrounding_pixels:
                pixel_range = 70
                depth_values = []
                for dy in range(-pixel_range, pixel_range + 1):
                    for dx in range(-pixel_range, pixel_range + 1):
                        new_y = image_y + dy
                        new_x = image_x + dx
                        if new_y >= 0 and new_y < depth_image.shape[0] and new_x >= 0 and new_x < depth_image.shape[1]:
                            d = depth_image[new_y, new_x] / 1000.0
                            if not math.isnan(d) and d >= 0.05 and d <= 2.0:
                                depth_values.append(d)
                if len(depth_values) > 0:
                    depth = np.median(depth_values)
                    print(f"Using surrounding pixels to get depth: {depth} m")
                else:
                    print("No valid depth values in surrounding pixels")
                    return False, None
            else:
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
    import argparse
    from feeding_deployment.interfaces.realsense_interface import RealSenseInterface

    parser = argparse.ArgumentParser()
    parser.add_argument('--handle_type', type=str, default='microwave handle',
                        help='Handle type to detect (e.g. "microwave handle", "bottom fridge door")')
    args = parser.parse_args()

    rospy.init_node('AppliancePerception')
    grounded_sam = GroundedSAM()
    appliance_perception = AppliancePerception(grounded_sam)

    print("Waiting for camera data...")
    realsense = RealSenseInterface()

    camera_data = None
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera_data = realsense.get_camera_data()
        if camera_data['rgb_image'] is not None:
            break
        rate.sleep()

    print(f"Running detect_handle_and_placement loop with handle_type='{args.handle_type}' (Ctrl-C to stop)")
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        camera_data = realsense.get_camera_data()
        if camera_data['rgb_image'] is None:
            print("No camera data, waiting...")
            rate.sleep()
            continue
        result = appliance_perception.detect_handle_and_placement(
            args.handle_type,
            camera_data['rgb_image'],
            camera_data['camera_info'],
            camera_data['depth_image'],
        )
        print("Result:", result)
        rate.sleep()
