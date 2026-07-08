# Description: This script is used to detect appliance handles and placement poses.

# python imports
import math
import os
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
import requests

# ros imports
import rospy
import supervision as sv
import torch
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose as pose_msg
from pybullet_helpers.geometry import Pose, Pose3D
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import (
    CartesianCommand,
    CloseGripperCommand,
    JointCommand,
    OpenGripperCommand,
)
from feeding_deployment.perception.grounded_sam import GroundedSAM
from feeding_deployment.perception.tf_interface import TFInterface

# --- Handle detection-confirmation overlay styling (presentation only) ---
# BGR colors (the webapp legend chips mirror these).
# Unified scheme across all detection screens:
#   amber = detected-region box (recording brackets, + shaded fill where useful)
#   red   = the point the robot acts on (grasp/press/place): red dot + red box
#   blue  = hinge pivot edge/axis (handle screens only)
_OVERLAY_HINGE = (219, 152, 52)  # blue
_OVERLAY_ACCENT = (
    60,
    190,
    255,
)  # amber  -- detected-region box (all types) + swing arrow
_OVERLAY_RED = (0, 0, 255)  # red    -- action/grasp point
_OVERLAY_WHITE = (255, 255, 255)
# Overlay element sizes below are tuned for a reference 1280-wide frame and scaled
# by AppliancePerception._scale() to the actual image width. Cameras differ in
# resolution (e.g. the 640x480 microwave cam vs the 1280x720 head cam), and the
# iPad renders every frame letterboxed (object-fit: contain) into the same box --
# so scaling keeps the markers a consistent on-screen size across all screens.
_OVERLAY_REF_W = 1280
# Swing-out arrow: wide, shallow elliptical arc about the hinge sweeping to the
# foreground (foreshortened projection of the door's out-of-plane swing).
_SWING_SWEEP_DEG = 43.0
_SWING_KY = {
    "bottom textured fridge door": 0.37,  # up-left (unchanged)
    "microwave": -0.22,  # mirror vertical so the left-hinged door sweeps up-right
}
_SWING_KY_DEFAULT = 0.22  # others (kept positive / upward)
_SWING_DASH_ON, _SWING_DASH_OFF, _SWING_NSEG = 7, 6, 60


class AppliancePerception(TFInterface):
    def __init__(
        self, grounded_sam: GroundedSAM, num_perception_samples=25, data_logger=None
    ):
        super().__init__()

        self.grounding_dino_model = grounded_sam.grounding_dino_model
        # NOTE: the appliance/handle path uses GroundingDINO boxes only and never
        # segments with SAM, so we deliberately do not touch grounded_sam.sam_predictor
        # here -- that keeps ViT-H unloaded (see GroundedSAM's lazy sam_predictor).

        self.BOX_THRESHOLD = 0.3
        self.TEXT_THRESHOLD = 0.3
        self.NMS_THRESHOLD = 0.4

        self.molmo_url = "https://c0fd-128-84-97-177.ngrok-free.app/predict"

        self.handle_type = None
        self.num_perception_samples = num_perception_samples

        self.handle_points_pub = rospy.Publisher("/handle_points", Marker, queue_size=1)
        self.handle_center_pub = rospy.Publisher("/handle_center", Marker, queue_size=1)

        # All detection images flow through the per-day data logger into the
        # active skill's folder (images/<skill>/<run>_<name>.png). We also keep the
        # most recent frame per name in memory so PerceptionInterface can relay the
        # confirmation vis to the iPad without re-reading it from disk.
        self._data_logger = data_logger
        self._last_images = {}

    def _log_image(self, name, image):
        """Log a detection image by semantic name (e.g. "rgb",
        "handle_pixels").

        Routes to the data logger (per-skill, ordered) and caches the
        frame for the UI relay. Nothing is written to the source tree or
        cwd.
        """
        self._last_images[name] = image
        if self._data_logger is not None:
            self._data_logger.log_image(name, image)

    # ------------------------------------------------------------------
    # Handle detection-confirmation overlay (what the user approves before
    # the robot opens a door). Presentation only -- the pixels/poses fed in
    # are unchanged. Drawn in raw (upside-down) image coords; the camera is
    # mounted upside down and WebInterface._send_image flips 180 deg centrally
    # for display, so we must NOT pre-rotate here. The shapes are symmetric and
    # carry no text, so they read correctly after that flip. The legend (which
    # color means what) lives in the webapp page, not baked into the image.
    # ------------------------------------------------------------------
    def _draw_handle_overlay(self, rgb_image, box, handle_px, hinge_px, handle_type):
        vis = rgb_image.copy()
        h, w = vis.shape[:2]
        x1, y1, x2, y2 = (int(v) for v in box)
        x1, x2 = max(0, min(x1, x2)), min(w, max(x1, x2))
        y1, y2 = max(0, min(y1, y2)), min(h, max(y1, y2))
        clamped = (x1, y1, x2, y2)

        self._draw_spotlight(vis, clamped)  # dim outside the detected door
        self._draw_corner_brackets(
            vis, clamped, length=self._scale(vis, 46)
        )  # amber door box
        self._draw_hinge_axis(vis, hinge_px, clamped)  # vertical pivot axis
        ky = _SWING_KY.get(handle_type, _SWING_KY_DEFAULT)
        self._draw_swing_arrow(
            vis, handle_px, hinge_px, ky
        )  # door swings out toward robot
        self._draw_halo_marker(vis, hinge_px, _OVERLAY_HINGE)  # blue pivot point
        self._draw_action_point(vis, handle_px)  # red grasp point (dot + box)
        return vis

    def _draw_button_overlay(self, rgb_image, button_px):
        """Microwave start button (a single detected point, no box): subtle
        spotlight + red recording box & dot at the press point.

        Uses the shared action-point sizes -- scaled to the frame -- so
        it matches the other screens despite the lower-resolution
        microwave camera.
        """
        vis = rgb_image.copy()
        self._draw_spotlight(vis, self._point_box(button_px, vis.shape, half=70))
        self._draw_action_point(vis, button_px)
        return vis

    def _draw_sink_overlay(self, rgb_image, box, used_px):
        """Sink: amber detected-region box (recording brackets + shaded fill)
        and a red recording box & dot at the pixel actually used (offset above
        the basin).

        Subtle spotlight focuses the sink area.
        """
        vis = rgb_image.copy()
        clamped = self._clamp_box(box, vis.shape)
        self._draw_spotlight(vis, clamped)  # subtle dim outside sink
        self._draw_shaded_box(vis, clamped, _OVERLAY_ACCENT)  # amber shaded fill
        self._draw_corner_brackets(
            vis, clamped, length=self._bracket_length(clamped)
        )  # amber recording corners
        self._draw_action_point(vis, used_px)  # red placement point
        return vis

    def _draw_plate_overlay(self, rgb_image, box, center_px):
        """Table placement: amber detected-marker box + red recording box & dot
        at the placement point.

        Subtle spotlight focuses the marker.
        """
        vis = rgb_image.copy()
        clamped = self._clamp_box(box, vis.shape)
        self._draw_spotlight(vis, clamped)
        self._draw_corner_brackets(vis, clamped, length=self._bracket_length(clamped))
        self._draw_action_point(vis, center_px)
        return vis

    @staticmethod
    def _scale(vis, px):
        """Scale a reference (1280-wide) size to this frame's width, so overlay
        elements stay a consistent on-screen size across camera resolutions."""
        return max(1, int(round(px * vis.shape[1] / _OVERLAY_REF_W)))

    @staticmethod
    def _clamp_box(box, shape):
        h, w = shape[:2]
        x1, y1, x2, y2 = (int(v) for v in box)
        x1, x2 = max(0, min(x1, x2)), min(w, max(x1, x2))
        y1, y2 = max(0, min(y1, y2)), min(h, max(y1, y2))
        return (x1, y1, x2, y2)

    @staticmethod
    def _point_box(pt, shape, half=55):
        """Small square box centred on a single detected pixel (half scaled to
        res)."""
        h, w = shape[:2]
        half = max(1, int(round(half * w / _OVERLAY_REF_W)))
        x, y = int(pt[0]), int(pt[1])
        return (max(0, x - half), max(0, y - half), min(w, x + half), min(h, y + half))

    @staticmethod
    def _bracket_length(box, frac=0.28, lo=18, hi=46):
        """Bracket length scaled to the box so corners never meet on small
        boxes."""
        x1, y1, x2, y2 = box
        return int(max(lo, min(hi, frac * min(x2 - x1, y2 - y1))))

    @staticmethod
    def _draw_shaded_box(vis, box, color, alpha=0.22):
        """Translucent fill only -- corners are drawn separately as
        brackets."""
        x1, y1, x2, y2 = box
        overlay = vis.copy()
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
        cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, dst=vis)

    def _draw_action_point(self, vis, pt, half=45, length=24, radius=20):
        """The point the robot acts on (grasp/press/place): a red recording box +
        red dot. Consistent 'action = red' across screens; sizes scale down for
        small targets (e.g. a microwave button)."""
        box = self._point_box(pt, vis.shape, half=half)
        self._draw_corner_brackets(
            vis, box, color=_OVERLAY_RED, length=self._scale(vis, length)
        )
        self._draw_halo_marker(vis, pt, _OVERLAY_RED, radius=radius)

    @staticmethod
    def _draw_spotlight(vis, box, dim=0.6):
        x1, y1, x2, y2 = box
        keep = vis[y1:y2, x1:x2].copy()
        vis[:] = (vis.astype(np.float32) * dim).astype(np.uint8)
        vis[y1:y2, x1:x2] = keep

    @staticmethod
    def _draw_corner_brackets(vis, box, color=_OVERLAY_ACCENT, length=46, thickness=5):
        # length arrives in image px (callers scale fixed lengths / pass box-relative
        # ones); thickness is a fixed reference size -> scale it to the frame.
        t = AppliancePerception._scale(vis, thickness)
        x1, y1, x2, y2 = box
        for cx, cy, sx, sy in (
            (x1, y1, 1, 1),
            (x2, y1, -1, 1),
            (x1, y2, 1, -1),
            (x2, y2, -1, -1),
        ):
            cv2.line(vis, (cx, cy), (cx + sx * length, cy), color, t, cv2.LINE_AA)
            cv2.line(vis, (cx, cy), (cx, cy + sy * length), color, t, cv2.LINE_AA)

    @staticmethod
    def _draw_hinge_axis(vis, hinge_px, box, thickness=5):
        _, y1, _, y2 = box
        off = AppliancePerception._scale(vis, 8)
        x = int(hinge_px[0])
        cv2.line(
            vis,
            (x, y1 + off),
            (x, y2 - off),
            _OVERLAY_HINGE,
            AppliancePerception._scale(vis, thickness),
            cv2.LINE_AA,
        )

    @staticmethod
    def _draw_halo_marker(vis, pt, color, radius=20):
        p = (int(pt[0]), int(pt[1]))
        r = AppliancePerception._scale(vis, radius)
        cv2.circle(
            vis,
            p,
            r + AppliancePerception._scale(vis, 6),
            _OVERLAY_WHITE,
            -1,
            cv2.LINE_AA,
        )
        cv2.circle(vis, p, r, color, -1, cv2.LINE_AA)

    @staticmethod
    def _draw_swing_arrow(vis, handle_px, hinge_px, ky):
        # Elliptical arc about the hinge: full horizontal radius, vertical squashed by
        # ky (foreshortened out-of-plane swing). Starts at the handle and sweeps toward the
        # top of the displayed frame, stopping at the straight-up point so it never dips back down.
        # The arc is mirrored across the horizontal axis: in the image the user sees, the
        # door opens toward the top: sweep toward -y (target = -pi/2).
        # Handedness is inherent from the handle/hinge positions.
        cx, cy = float(hinge_px[0]), float(hinge_px[1])
        vx, vy = handle_px[0] - cx, handle_px[1] - cy
        R = math.hypot(vx, vy)
        if R < 1e-3:
            return
        a0 = math.atan2(vy, vx)
        target = -math.pi / 2.0  # toward -y (top) in the displayed frame
        d = 1.0 if (target - a0) > 0 else -1.0
        sweep = min(math.radians(_SWING_SWEEP_DEG), abs(target - a0))
        pts = [
            (
                cx + R * math.cos(a0 + d * sweep * i / _SWING_NSEG),
                cy + R * ky * math.sin(a0 + d * sweep * i / _SWING_NSEG),
            )
            for i in range(_SWING_NSEG + 1)
        ]
        ox, oy = (
            handle_px[0] - pts[0][0],
            handle_px[1] - pts[0][1],
        )  # anchor start on handle
        pts = [(int(round(px + ox)), int(round(py + oy))) for px, py in pts]
        period = _SWING_DASH_ON + _SWING_DASH_OFF
        for i in range(len(pts) - 1):
            if (i % period) < _SWING_DASH_ON:  # coarse dashes
                t = AppliancePerception._scale(
                    vis, 8 + 6 * i / len(pts)
                )  # taper thicker toward tip
                cv2.line(vis, pts[i], pts[i + 1], _OVERLAY_ACCENT, t, cv2.LINE_AA)
        cv2.arrowedLine(
            vis,
            pts[-3],
            pts[-1],
            _OVERLAY_ACCENT,
            AppliancePerception._scale(vis, 14),
            tipLength=0.5,
            line_type=cv2.LINE_AA,
        )

    def detect_start_button(self, rgb_image, camera_info_msg, depth_image):
        transform = self.get_frame_to_frame_transform(camera_info_msg)

        print("Got images")
        self._log_image("rgb", rgb_image)
        # depth_image is already in millimeters (RealSense 32FC1; pixel2World divides
        # by 1000). Cast directly -- the old *1000 wrapped uint16 and logged garbage.
        depth_mm = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0).astype(
            "uint16"
        )
        self._log_image("depth", depth_mm)

        # Log the inputs (intrinsics + base<-camera transform) for offline replay.
        self._log_detection_inputs("detect_start_button", camera_info_msg, transform)

        rgb_image_flipped = cv2.flip(rgb_image.copy(), -1)
        self._log_image("rgb_flipped", rgb_image_flipped)

        # Encode in memory for the molmo POST -- no scratch file on disk.
        ok_enc, flipped_buf = cv2.imencode(".png", rgb_image_flipped)
        if not ok_enc:
            print("Failed to encode flipped image for molmo request")
            return None
        http_response = requests.post(
            self.molmo_url,
            files={"image": ("rgb_flipped.png", flipped_buf.tobytes(), "image/png")},
            data={
                "prompt": "Point to the center of the start / 30 secs white button. Right one out of two rectangular buttons at the bottom row of the microwave control panel."
            },
        )
        http_response.raise_for_status()
        response = http_response.json()
        print("Molmo HTTP response:", response)
        pixel_coords = response.get("pixel_coords", [])

        print("Pixel coords from molmo:", pixel_coords)
        # Flip pixel coords back since we flipped the image before sending to molmo
        button_pixel = (
            rgb_image.shape[1] - pixel_coords[0][0],
            rgb_image.shape[0] - pixel_coords[0][1],
        )

        # User-facing overlay: red recording-style box + red dot on the button.
        # Drawn raw; WebInterface._send_image applies the central 180 deg flip.
        vis_image = self._draw_button_overlay(rgb_image, button_pixel)
        self._log_image("rgb_button_pixel", vis_image)

        ok, button_3d = self.pixel2World(
            camera_info_msg, button_pixel[0], button_pixel[1], depth_image
        )

        if not ok:
            print("Could not get valid 3D point for button")
            return None

        if transform is not None:
            print("Got transform between arm_base_link and camera_color_optical_frame")
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_button = np.eye(4)
            camera_to_button[:3, 3] = button_3d
            camera_to_button[3, 3] = 1
            base_to_button = np.dot(base_to_camera, camera_to_button)
            base_to_button[:3, :3] = Rotation.from_quat(
                [-0.5, 0.5, 0.5, -0.5]
            ).as_matrix()
            return self.matrix_to_pose(base_to_button)

        print(
            "Could not get transform between arm_base_link and camera_color_optical_frame"
        )
        return None

    def detect_handle_and_placement(
        self, handle_type, rgb_image, camera_info_msg, depth_image
    ):
        if rgb_image is None:
            print("No camera data provided")
            return None, None, None, None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        print("Got images")
        self._log_image("rgb", rgb_image)
        # depth_image is already in millimeters (RealSense 32FC1; pixel2World divides
        # by 1000). Cast directly -- the old *1000 wrapped uint16 and logged garbage.
        depth_mm = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0).astype(
            "uint16"
        )
        self._log_image("depth", depth_mm)

        # Log the inputs (intrinsics + base<-camera transform + handle type) for offline replay.
        self._log_detection_inputs(
            "detect_handle_and_placement",
            camera_info_msg,
            transform,
            handle_type=handle_type,
        )

        detection = self.detect_items(rgb_image, [handle_type])

        if detection is None:
            print("No detection")
            return None, None, None, None

        # create mask using detection
        x1, y1, x2, y2 = detection.astype(int)
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        self._log_image("detection_mask", mask)

        center_pixel = ((x1 + x2) // 2, (y1 + y2) // 2)

        # -----------------------------
        # Extract ALL 3D points from mask
        # -----------------------------
        bounding_box_points_3d = []
        pixels = []

        ys, xs = np.where(mask > 0)
        for u, v in zip(xs, ys):
            ok, p = self.pixel2World(camera_info_msg, u, v, depth_image)
            if ok:
                bounding_box_points_3d.append(p)
                pixels.append((u, v))

        if len(bounding_box_points_3d) == 0:
            print("-------------- ERROR: No valid 3D points from mask.")
            # rospy.logwarn("No valid 3D points from mask.")
            return None, None, None, None

        bounding_box_points_3d = np.array(bounding_box_points_3d)
        pixels = np.array(pixels)

        # Fit a plane to the 3D points to find the main planar surface (fridge door)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(bounding_box_points_3d)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.02,  # 1 cm threshold for inliers
            ransac_n=3,
            num_iterations=500,
        )
        outliers = np.setdiff1d(np.arange(len(bounding_box_points_3d)), inliers)

        plane_cloud = pcd.select_by_index(inliers)
        possible_handle_cloud = pcd.select_by_index(inliers, invert=True)

        # visulize plane_cloud on image

        vis = rgb_image.copy()
        for u, v in pixels[inliers]:
            vis[v, u] = (255, 0, 0)
        self._log_image("plane_pixels", vis)

        vis = rgb_image.copy()
        for u, v in pixels[outliers]:
            vis[v, u] = (0, 0, 255)
        self._log_image("possible_handle_pixels", vis)

        plane_depth = np.median(np.asarray(plane_cloud.points)[:, 2])
        print("Plane depth in m:", plane_depth)

        # remove points from possible_handle_cloud which are behind the plane
        possible_handle_points = np.asarray(possible_handle_cloud.points)
        handle_points = []
        handle_pixels = []
        for i, p in enumerate(possible_handle_points):
            if (
                p[2] < plane_depth and p[2] > plane_depth - 0.07
            ):  # within a reasonable distance infront of the door
                handle_points.append(p)
                handle_pixels.append(pixels[outliers[i]])

        # visualize handle_pixels on image
        vis = rgb_image.copy()
        for u, v in handle_pixels:
            vis[v, u] = (0, 255, 0)
        self._log_image("handle_pixels", vis)

        # -----------------------------
        # DBSCAN clustering (7 cm)
        # -----------------------------
        clustering = DBSCAN(
            eps=0.02,  # 30 cm to allow for fridge handles which can be quite large
            min_samples=50,
        ).fit(handle_points)

        # print("Ran DBSCAN")

        labels = clustering.labels_
        valid = labels >= 0

        if not np.any(valid):
            # rospy.logwarn("DBSCAN found no clusters.")
            return None, None, None, None

        unique, counts = np.unique(labels[valid], return_counts=True)
        main_label = unique[np.argmax(counts)]

        handle_pixels = np.array(handle_pixels)
        handle_points = np.array(handle_points)
        cluster_pixels = handle_pixels[labels == main_label]
        cluster_points_3d = handle_points[labels == main_label]

        top_most_y = np.max(cluster_points_3d[:, 1])

        # for handle_centroid take median in x, y and z to be more robust to outliers

        handle_centroid = np.median(cluster_points_3d, axis=0)
        if handle_type == "bottom textured fridge door":
            handle_centroid[1] = top_most_y - 0.07
        else:
            handle_centroid[1] = top_most_y - 0.04
        # handle_centroid[1] = top_most_y - 0.02 # 2 cm below the top most point in the cluster, which should be close to the center of the handle

        # find top of the plane_cloud (not handle) just above handle
        top_of_appliance = handle_centroid.copy()
        top_of_appliance[1] = np.max(np.asarray(plane_cloud.points)[:, 1])
        top_of_appliance_pixel = self.world2Pixel(
            camera_info_msg,
            top_of_appliance[0],
            top_of_appliance[1],
            top_of_appliance[2],
        )

        handle_centroid_3d = handle_centroid
        handle_centroid_pixel = self.world2Pixel(
            camera_info_msg, handle_centroid[0], handle_centroid[1], handle_centroid[2]
        )

        print("Handle centroid pixel:", handle_centroid_pixel)
        print("Top of plane pixel:", top_of_appliance_pixel)

        if handle_type == "bottom textured fridge door":
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
        hinge_idx = np.argmin(
            np.linalg.norm(hinge_strip_points - handle_centroid_3d, axis=1)
        )
        hinge_3d = hinge_strip_points[hinge_idx]

        # Hack: hinge y is the same as handle_centroid y
        hinge_3d[1] = handle_centroid_3d[1]

        hinge_pixel = self.world2Pixel(
            camera_info_msg, hinge_3d[0], hinge_3d[1], hinge_3d[2]
        )

        # update center pixel x is average of handle_centroid_pixel and hinge_pixel
        center_pixel = (
            (handle_centroid_pixel[0] + hinge_pixel[0]) // 2,
            center_pixel[1],
        )

        # Build the user-facing detection-confirmation overlay (presentation only --
        # handle/hinge pixels and the box convey what the robot will do).
        vis = self._draw_handle_overlay(
            rgb_image,
            (x1, y1, x2, y2),
            handle_centroid_pixel,
            hinge_pixel,
            handle_type,
        )
        self._log_image("handle_hinge_pixels", vis)

        ok, center_3d = self.pixel2World(
            camera_info_msg,
            center_pixel[0],
            center_pixel[1],
            depth_image,
            depth=plane_depth,
        )
        if not ok:
            print("Could not get valid 3D point for center pixel")
            return None, None, None, None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        if transform is not None:
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_handle = np.eye(4)
            camera_to_handle[:3, 3] = handle_centroid_3d
            camera_to_handle[3, 3] = 1
            base_to_handle = np.dot(base_to_camera, camera_to_handle)
            base_to_handle[:3, :3] = Rotation.from_quat(
                [-0.5, 0.5, 0.5, -0.5]
            ).as_matrix()

            camera_to_hinge = np.eye(4)
            camera_to_hinge[:3, 3] = hinge_3d
            camera_to_hinge[3, 3] = 1
            base_to_hinge = np.dot(base_to_camera, camera_to_hinge)
            base_to_hinge[:3, :3] = Rotation.from_quat(
                [-0.5, 0.5, 0.5, -0.5]
            ).as_matrix()

            camera_to_placement = np.eye(4)
            camera_to_placement[:3, 3] = center_3d
            camera_to_placement[3, 3] = 1
            base_to_placement = np.dot(base_to_camera, camera_to_placement)
            base_to_placement[:3, :3] = Rotation.from_quat(
                [-0.5, 0.5, 0.5, -0.5]
            ).as_matrix()

            camera_to_top_of_appliance = np.eye(4)
            camera_to_top_of_appliance[:3, 3] = top_of_appliance
            camera_to_top_of_appliance[3, 3] = 1
            base_to_top_of_appliance = np.dot(
                base_to_camera, camera_to_top_of_appliance
            )
            base_to_top_of_appliance[:3, :3] = Rotation.from_quat(
                [-0.5, 0.5, 0.5, -0.5]
            ).as_matrix()

            return (
                self.matrix_to_pose(base_to_handle),
                self.matrix_to_pose(base_to_hinge),
                self.matrix_to_pose(base_to_placement),
                self.matrix_to_pose(base_to_top_of_appliance),
            )

        print(
            "Could not get transform between arm_base_link and camera_color_optical_frame"
        )
        return None, None, None, None

    def detect_sink_placement(self, rgb_image, camera_info_msg, depth_image):
        if rgb_image is None:
            print("No camera data provided")
            return None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        print("Got images")
        self._log_image("rgb", rgb_image)
        # depth_image is already in millimeters (RealSense 32FC1; pixel2World divides
        # by 1000). Cast directly -- the old *1000 wrapped uint16 and logged garbage.
        depth_mm = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0).astype(
            "uint16"
        )
        self._log_image("depth", depth_mm)

        # Log the inputs (intrinsics + base<-camera transform) for offline replay.
        self._log_detection_inputs("detect_sink_placement", camera_info_msg, transform)

        detection = self.detect_items(rgb_image, ["sink basin tap"])

        if detection is None:
            print("No detection")
            return None

        # create mask using detection
        x1, y1, x2, y2 = detection.astype(int)
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        self._log_image("detection_mask", mask)

        # Hack, take a point with x as center of bounding box and y as 40 pixels above the top of the bounding box
        center_pixel = ((x1 + x2) // 2 + 140, y1 - 50)
        # User-facing overlay: yellow shaded sink-detection box + a small red
        # recording-style box & dot at the pixel actually used. Drawn raw; the
        # camera-orientation flip is applied centrally in WebInterface._send_image,
        # so rotating here would double-flip and show an upside-down image.
        vis_image = self._draw_sink_overlay(rgb_image, (x1, y1, x2, y2), center_pixel)
        self._log_image("sink_back_pixel", vis_image)

        ok, center_3d = self.pixel2World(
            camera_info_msg,
            center_pixel[0],
            center_pixel[1],
            depth_image,
            use_surrounding_pixels=True,
        )
        if not ok:
            print("Could not get valid 3D point for sink placement")
            return None

        if transform is not None:
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_sink = np.eye(4)
            camera_to_sink[:3, 3] = center_3d
            camera_to_sink[3, 3] = 1
            base_to_sink = np.dot(base_to_camera, camera_to_sink)
            base_to_sink[:3, :3] = Rotation.from_quat([0.5, 0.5, 0.5, 0.5]).as_matrix()
            return self.matrix_to_pose(base_to_sink)

        print(
            "Could not get transform between arm_base_link and camera_color_optical_frame"
        )
        return None

    def detect_table_placement(self, rgb_image, camera_info_msg, depth_image):
        if rgb_image is None:
            print("No camera data provided")
            return None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        print("Got images")
        self._log_image("rgb", rgb_image)
        # depth_image is already in millimeters (RealSense 32FC1; pixel2World divides
        # by 1000). Cast directly -- the old *1000 wrapped uint16 and logged garbage.
        depth_mm = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0).astype(
            "uint16"
        )
        self._log_image("depth", depth_mm)

        # Log the inputs (intrinsics + base<-camera transform) for offline replay.
        self._log_detection_inputs("detect_table_placement", camera_info_msg, transform)

        detection = self.detect_items(rgb_image, ["blue square on table"])

        if detection is None:
            print("No detection")
            return None

        # create mask using detection
        x1, y1, x2, y2 = detection.astype(int)
        mask = np.zeros(rgb_image.shape[:2], dtype=np.uint8)
        mask[y1:y2, x1:x2] = 255
        self._log_image("detection_mask", mask)

        center_pixel = ((x1 + x2) // 2, (y1 + y2) // 2)

        # mark all "surrounding pixels" which will be used for depth estimation as well
        # pixel_range = 70
        # for dy in range(-pixel_range, pixel_range + 1):
        #         for dx in range(-pixel_range, pixel_range + 1):
        #             new_y = center_pixel[1] + dy
        #             new_x = center_pixel[0] + dx
        #             if 0 <= new_x < rgb_image.shape[1] and 0 <= new_y < rgb_image.shape[0]:
        #                 cv2.circle(rgb_image, (new_x, new_y), 2, (255, 0, 0), -1)

        # User-facing overlay: red recording-style detection box + red center dot.
        vis_image = self._draw_plate_overlay(rgb_image, (x1, y1, x2, y2), center_pixel)
        self._log_image("table_placement_pixel", vis_image)

        ok, center_3d = self.pixel2World(
            camera_info_msg,
            center_pixel[0],
            center_pixel[1],
            depth_image,
            use_surrounding_pixels=True,
        )
        if not ok:
            print("Could not get valid 3D point for table placement")
            return None

        if transform is not None:
            base_to_camera = self.make_homogeneous_transform(transform)

            camera_to_center = np.eye(4)
            camera_to_center[:3, 3] = center_3d
            camera_to_center[3, 3] = 1
            base_to_center = np.dot(base_to_camera, camera_to_center)
            base_to_center[:3, :3] = Rotation.from_quat(
                [0.0, 0.707, 0.707, 0.0]
            ).as_matrix()
            return self.matrix_to_pose(base_to_center)

        print(
            "Could not get transform between arm_base_link and camera_color_optical_frame"
        )
        return None

    def detect_items(self, input_image, classes_being_detected, log_path=None):

        # flip image because camera is mounted upside down
        image = cv2.flip(input_image.copy(), -1)

        # detect objects
        detections = self.grounding_dino_model.predict_with_classes(
            image=image,
            classes=classes_being_detected,
            box_threshold=self.BOX_THRESHOLD,
            text_threshold=self.TEXT_THRESHOLD,
        )

        # annotate image with detections
        box_annotator = sv.BoxAnnotator()
        labels = [
            f"{classes_being_detected[class_id]} {confidence:0.2f}"
            for _, _, confidence, class_id, _, _ in detections
        ]

        # NMS post process
        # print(f"Before NMS: {len(detections.xyxy)} boxes")
        nms_idx = (
            torchvision.ops.nms(
                torch.from_numpy(detections.xyxy),
                torch.from_numpy(detections.confidence),
                self.NMS_THRESHOLD,
            )
            .numpy()
            .tolist()
        )

        # remove boxes which are union of two boxes

        detections.xyxy = detections.xyxy[nms_idx]
        detections.confidence = detections.confidence[nms_idx]
        detections.class_id = detections.class_id[nms_idx]
        labels = [labels[i] for i in nms_idx]
        # print(f"After NMS: {len(detections.xyxy)} boxes")

        annotated_frame = box_annotator.annotate(
            scene=image.copy(), detections=detections, labels=labels
        )
        self._log_image("detections_flipped", annotated_frame)

        print("Image size:", image.shape)
        print("Detections (flipped):")
        for _, _, confidence, class_id, _, _ in detections:
            print(f"  {classes_being_detected[class_id]}: {confidence:0.2f}")
        print(detections.xyxy)

        # flip back the image and detections to original orientation
        image = cv2.flip(image, -1)
        for i in range(len(detections.xyxy)):
            x1, y1, x2, y2 = detections.xyxy[i]
            detections.xyxy[i] = [
                image.shape[1] - x2,
                image.shape[0] - y2,
                image.shape[1] - x1,
                image.shape[0] - y1,
            ]

        annotated_frame = box_annotator.annotate(
            scene=image.copy(), detections=detections, labels=labels
        )
        self._log_image("detections_original", annotated_frame)

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

    def pixel2World(
        self,
        camera_info,
        image_x,
        image_y,
        depth_image,
        depth=None,
        use_surrounding_pixels=False,
    ):

        # print("Image pixels: ", image_x, image_y)
        # print("Depth shape: ", depth_image.shape)

        if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
            return False, None

        if depth is None:
            depth = depth_image[image_y, image_x]
            depth = depth / 1000  # convert from mm to m
            # print("Depth: ", depth)

        if math.isnan(depth) or depth < 0.05 or depth > 2.0:
            if use_surrounding_pixels:
                pixel_range = 25
                depth_values = []
                for dy in range(-pixel_range, pixel_range + 1):
                    for dx in range(-pixel_range, pixel_range + 1):
                        new_y = image_y + dy
                        new_x = image_x + dx
                        if (
                            new_y >= 0
                            and new_y < depth_image.shape[0]
                            and new_x >= 0
                            and new_x < depth_image.shape[1]
                        ):
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


if __name__ == "__main__":
    import argparse

    from feeding_deployment.interfaces.realsense_interface import RealSenseInterface

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--handle_type",
        type=str,
        default="microwave handle",
        help='Handle type to detect (e.g. "microwave handle", "bottom textured fridge door")',
    )
    args = parser.parse_args()

    rospy.init_node("AppliancePerception")
    grounded_sam = GroundedSAM()
    appliance_perception = AppliancePerception(grounded_sam)

    print("Waiting for camera data...")
    realsense = RealSenseInterface()

    camera_data = None
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        camera_data = realsense.get_camera_data()
        if camera_data["rgb_image"] is not None:
            break
        rate.sleep()

    print(
        f"Running detect_handle_and_placement loop with handle_type='{args.handle_type}' (Ctrl-C to stop)"
    )
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        camera_data = realsense.get_camera_data()
        if camera_data["rgb_image"] is None:
            print("No camera data, waiting...")
            rate.sleep()
            continue
        result = appliance_perception.detect_handle_and_placement(
            args.handle_type,
            camera_data["rgb_image"],
            camera_data["camera_info"],
            camera_data["depth_image"],
        )
        print("Result:", result)
        rate.sleep()
