# Description: This script is used to detect tool attachments.

# python imports
import os, sys
import cv2
import numpy as np
import time
import math
from scipy.spatial.transform import Rotation
from sklearn.cluster import DBSCAN
import open3d as o3d
from pybullet_helpers.geometry import Pose, Pose3D

# ros imports
import rospy
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker

from collections import deque

from geometry_msgs.msg import Pose as pose_msg

from feeding_deployment.perception.tf_interface import TFInterface 


class AttachmentPerception(TFInterface):
    def __init__(self, num_perception_samples=25, data_logger=None):
        super().__init__()

        self.num_perception_samples = num_perception_samples

        self.attachment_points_pub = rospy.Publisher("/attachment_points", Marker, queue_size=1)
        self.attachment_center_pub = rospy.Publisher("/attachment_center", Marker, queue_size=1)

        # Route detection images through the per-day data logger (into the active
        # skill's folder) and cache the latest frame per name so PerceptionInterface
        # can relay the confirmation vis to the iPad without re-reading from disk.
        self._data_logger = data_logger
        self._last_images = {}

    def _log_image(self, name, image):
        """Log a detection image by semantic name; no write to the source tree/cwd."""
        self._last_images[name] = image
        if self._data_logger is not None:
            self._data_logger.log_image(name, image)

    def detect_attachment(self, rgb_image, camera_info_msg, depth_image, handle_orientation="front", handle_color=None, color_range=0.1, perceive_yaw=True, max_yaw_deg=60.0):
        if rgb_image is None:
            print("No camera data provided.")
            return None

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        print("Got images")
        self._log_image("rgb", rgb_image)
        # depth_image is already in millimeters (RealSense 32FC1; pixel2World divides
        # by 1000). Cast directly -- the old *1000 wrapped uint16 and logged garbage.
        depth_mm = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0).astype("uint16")
        self._log_image("depth", depth_mm)

        # Log everything needed to re-run this exact detection offline (intrinsics,
        # base<-camera transform, and the color/orientation knobs) as a sidecar next
        # to the rgb/depth frames above.
        self._log_detection_inputs(
            "detect_attachment", camera_info_msg, transform,
            handle_orientation=handle_orientation,
            handle_color=handle_color,
            color_range=color_range,
            perceive_yaw=perceive_yaw,
            max_yaw_deg=max_yaw_deg,
        )

        # -----------------------------
        # Color mask
        # -----------------------------
        mask = self.detect_attachment_color(rgb_image, handle_color=handle_color, color_range=color_range)
        mask = self.clean_mask(mask)

        # -----------------------------
        # Extract ALL 3D points from mask
        # -----------------------------
        points_3d = []
        pixels = []

        ys, xs = np.where(mask > 0)
        print(f"Found {len(xs)} pixels in mask")
        for u, v in zip(xs, ys):
            ok, p = self.pixel2World(
                camera_info_msg, u, v, depth_image)
            if ok:
                points_3d.append(p)
                pixels.append((u, v))

        if len(points_3d) == 0:
            print("No valid 3D points from mask.")
            # rospy.logwarn("No valid 3D points from mask.")
            return

        points_3d = np.array(points_3d)
        pixels = np.array(pixels)        # print("Found all pixels")

        # -----------------------------
        # DBSCOkay,AN clustering (7 cm)
        # -----------------------------
        clustering = DBSCAN(
            eps=0.07,
            min_samples=25
        ).fit(points_3d)

        # print("Ran DBSCAN")

        labels = clustering.labels_
        valid = labels >= 0

        if not np.any(valid):
            print("DBSCAN found no clusters.")
            # rospy.logwarn("DBSCAN found no clusters.")
            return

        unique, counts = np.unique(labels[valid], return_counts=True)
        main_label = unique[np.argmax(counts)]

        cluster_pixels = pixels[labels == main_label]
        cluster_points_3d = points_3d[labels == main_label]

        # print("Found cluster")

        # -----------------------------
        # Project cluster back to image
        # -----------------------------
        cluster_mask = np.zeros(mask.shape, dtype=np.uint8)
        for u, v in cluster_pixels:
            cluster_mask[v, u] = 255

        cluster_mask = cv2.dilate(
            cluster_mask, np.ones((3, 3), np.uint8), iterations=1)

        vis = rgb_image.copy()
        vis[cluster_mask > 0] = (0, 0, 255)

        self._log_image("attachment_mask", vis)
        # rospy.loginfo(
            # f"Saved attachment_mask.png with {cluster_pixels.shape[0]} pixels")
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cluster_points_3d)

        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.003,
            ransac_n=3,
            num_iterations=500
        )

        plane_cloud = pcd.select_by_index(inliers)
        non_plane_cloud = pcd.select_by_index(inliers, invert=True)

        pts3d_planar = np.asarray(plane_cloud.points)
        pts3d_attachment = np.asarray(non_plane_cloud.points)
        self.visualizeattachment(pts3d_planar)

        # Plane normal
        a, b, c, d = plane_model
        n = np.array([a, b, c])
        n = n / np.linalg.norm(n)

        # Build plane basis (camera is level)
        up = np.array([0, 0, 1])
        u = np.cross(up, n)
        u = u / np.linalg.norm(u)
        v = np.cross(n, u)

        # Project 3D points to 2D plane coordinates
        P0 = pts3d_planar.mean(axis=0)
        P = pts3d_planar - P0
        x = P @ u
        y = P @ v
        P2 = np.stack([x, y], axis=1).astype(np.float32)

        # Fit minimum-area rectangle
        rect = cv2.minAreaRect(P2)
        (center_2d, _, _) = rect

        # Back-project center to 3D
        center_3d = P0 + center_2d[0] * u + center_2d[1] * v

        # Get 4 rectangle corners in 2D (plane coordinates)
        box_2d = cv2.boxPoints(rect)  # shape (4,2)

        # -----------------------------
        # User-facing visualization: show the color-detected pixels (all raw
        # matches faint, the kept cluster bold) plus viewfinder brackets at the
        # four detected corners, so a non-technical user can judge the detection
        # -- and whether to hit "Correct Color". Drawn in raw orientation;
        # WebInterface._send_image applies the central 180 deg flip before display.
        # -----------------------------
        corner_vis = self._overlay_detected_color(rgb_image.copy(), mask, cluster_mask)

        # Back-project the four corners to 3D, collecting their pixels.
        corners_3d = []
        corner_uvs = []
        for x2d, y2d in box_2d:
            p3d = P0 + x2d * u + y2d * v
            corners_3d.append(p3d)
            uv = self.world2Pixel(camera_info_msg, world_x=p3d[0], world_y=p3d[1], world_z=p3d[2])
            corner_uvs.append((int(uv[0]), int(uv[1])))

        # Viewfinder brackets at the corners + center dot (amber, matching). The pose
        # gizmo (showing the *applied* orientation incl. the perceived yaw) is drawn on
        # top and logged once the final rotation is known, further below.
        self._draw_corner_brackets(corner_vis, corner_uvs)
        uv_center = self.world2Pixel(camera_info_msg, world_x=center_3d[0], world_y=center_3d[1], world_z=center_3d[2])
        self._halo_marker(corner_vis, (int(uv_center[0]), int(uv_center[1])), self._OVERLAY_ACCENT)

        corners_3d = np.array(corners_3d)

        points_to_show = np.vstack([center_3d.reshape(1, 3), corners_3d])  # (5,3)
        self.visualizeattachmentCorners(points_to_show)

        # Sort corners by image-space Y (top vs bottom)
        # Smaller Y = higher in image (top)
        ys = corners_3d[:, 1]
        top_idx = np.argsort(ys)[:2]
        bottom_idx = np.argsort(ys)[2:]

        top_pts = corners_3d[top_idx]
        bottom_pts = corners_3d[bottom_idx]

        # Sort left/right within top and bottom using X
        top_left, top_right = top_pts[np.argsort(top_pts[:, 0])]
        bottom_left, bottom_right = bottom_pts[np.argsort(bottom_pts[:, 0])]

        # X-axis: bottom → top
        x_axis = ((top_left + top_right) / 2.0) - ((bottom_left + bottom_right) / 2.0)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # Y-axis: right → left
        y_axis = ((top_left + bottom_left) / 2.0) - ((top_right + bottom_right) / 2.0)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Z-axis: towards camera (right-handed)
        z_axis = np.cross(x_axis, y_axis)
        z_axis = z_axis / np.linalg.norm(z_axis)

        # Re-orthogonalize Y to avoid drift
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Rotation matrix (columns are axes)
        R_mat = np.column_stack((x_axis, y_axis, z_axis))

        transform = self.get_frame_to_frame_transform(camera_info_msg)

        if transform is not None:   
            base_to_camera = self.make_homogeneous_transform(transform)

            # cam to tag homogeneous transform
            camera_to_tag = np.zeros((4, 4))
            camera_to_tag[:3, :3] = R_mat
            camera_to_tag[:3, 3] = center_3d
            camera_to_tag[3, 3] = 1 

            # base to tag homogeneous transform and update tf
            base_to_tag = np.dot(base_to_camera, camera_to_tag)

            # The holder is mounted perpendicular to the plate, so its flat face
            # points back at the camera: roll/pitch are fixed by the mounting and only
            # the yaw (how the plate is rotated about the vertical axis) varies between
            # pickups. Keep the hand-tuned nominal orientation -- which bakes in the
            # correct roll/pitch and the gripper approach direction -- and rotate it
            # about base +Z by the yaw read off the detected face plane. This is far
            # more robust than trusting the full RANSAC/rectangle rotation, whose
            # roll/pitch are noisy on a small, near-planar patch. The pickup offsets
            # downstream are applied in this frame, so the approach automatically
            # follows the perceived yaw while "up" (+Y) stays vertical.
            nominal_quat = self._NOMINAL_HANDLE_QUAT.get(handle_orientation)
            if nominal_quat is not None:
                nominal_R = Rotation.from_quat(nominal_quat).as_matrix()
                dyaw = self._perceived_face_yaw(
                    n, base_to_camera[:3, :3], nominal_R, max_yaw_deg
                ) if perceive_yaw else 0.0
                yaw_R = Rotation.from_rotvec([0.0, 0.0, dyaw]).as_matrix()
                base_to_tag[:3, :3] = yaw_R @ nominal_R
            # else: unknown orientation -> keep the fully perceived rotation.

            # Draw the *applied* orientation as an axis gizmo at the detected center so
            # the operator can sanity-check the pose (esp. the perceived yaw: the
            # horizontal X/Z axes swing with it while Y/up stays vertical) before hitting
            # Confirm. Project it via the camera-frame rotation of the final pose.
            R_cam_final = base_to_camera[:3, :3].T @ base_to_tag[:3, :3]
            self._draw_pose_axes(corner_vis, camera_info_msg, center_3d, R_cam_final)
            self._log_image("attachment_corners", corner_vis)

            self.updateTF("arm_base_link", "attachment", base_to_tag)
            return self.matrix_to_pose(base_to_tag)

        # No transform: still surface the detection (brackets only) for the UI relay.
        self._log_image("attachment_corners", corner_vis)
        print("Could not find transform between arm_base_link and camera_color_optical_frame.")
        return None

    # Nominal base-frame orientation of the attachment per mounting, with the correct
    # roll/pitch (flat face toward the camera) and gripper approach direction baked in
    # -- (x, y, z, w). Their common local +Y is base +Z (up); their local +Z is the
    # (horizontal) approach axis. We keep these fixed and add only the perceived yaw.
    _NOMINAL_HANDLE_QUAT = {
        "front": [-0.5, 0.5, 0.5, -0.5],
        "left": [0.0, 0.7071, 0.7071, 0.0],  # convention (x, y, z, w)
    }

    @staticmethod
    def _perceived_face_yaw(n_cam, base_R_camera, nominal_R, max_yaw_deg=60.0, min_horiz=0.3):
        """Yaw offset (radians, about base +Z) of the detected holder face relative to
        the nominal orientation.

        The holder's flat face points back at the camera, so the detected plane
        normal's heading in the horizontal plane *is* the holder yaw. We compare it
        against the nominal orientation's approach axis (its local +Z, column 2) and
        return the signed rotation about vertical that lines them up.

        Returns 0.0 when the detection is unreliable -- the face normal is too close
        to vertical (holder not actually facing the camera, or a bad plane fit). The
        offset is clamped to +/-``max_yaw_deg`` so a stray detection can only nudge,
        not flip, the hand-tuned nominal orientation.
        """
        # Face normal in the base frame; yaw is its heading in the horizontal plane.
        n_base = base_R_camera @ np.asarray(n_cam, dtype=float)
        n_h = n_base[:2]
        if np.linalg.norm(n_h) < min_horiz:
            print("[attachment] face normal near-vertical; skipping perceived yaw.")
            return 0.0

        # Nominal approach axis (local +Z), projected to horizontal.
        z_nom_h = nominal_R[:2, 2]

        # A plane normal's sign is arbitrary; flip it into the same half-plane as the
        # nominal approach axis so a well-aligned holder reads as ~0 yaw (this assumes
        # the true yaw stays within +/-90 deg of nominal, which the clamp enforces).
        if np.dot(n_h, z_nom_h) < 0:
            n_h = -n_h

        dtheta = math.atan2(n_h[1], n_h[0]) - math.atan2(z_nom_h[1], z_nom_h[0])
        dyaw = math.atan2(math.sin(dtheta), math.cos(dtheta))  # wrap to [-pi, pi]

        max_yaw = math.radians(max_yaw_deg)
        if abs(dyaw) > max_yaw:
            print(f"[attachment] perceived yaw {math.degrees(dyaw):.1f} deg exceeds "
                  f"+/-{max_yaw_deg:.0f} deg; clamping.")
            dyaw = max(-max_yaw, min(max_yaw, dyaw))
        else:
            print(f"[attachment] perceived yaw offset: {math.degrees(dyaw):.1f} deg")
        return dyaw

    # BGR constants for the user-facing detection overlay.
    _OVERLAY_HL = (0, 0, 255)         # red:  color pixels kept (used to fit the attachment)
    _OVERLAY_REJECT = (255, 255, 0)   # cyan: color matches discarded by clustering
    _OVERLAY_ACCENT = (40, 190, 255)  # amber: corner brackets + center dot

    @staticmethod
    def _overlay_detected_color(vis, raw_mask, cluster_mask):
        """Tint the color-filtered pixels onto ``vis``.

        Pixels that pass the color threshold but are discarded by DBSCAN clustering
        are painted white (rejected); the kept cluster -- the pixels actually used
        to fit the attachment -- is painted bold red. So the user sees what the
        color filter caught versus what was actually used.
        """
        # Solid colors, no alpha -- rejected matches = white, selected cluster = red.
        if raw_mask is not None and cluster_mask is not None:
            rejected = (raw_mask > 0) & (cluster_mask == 0)
            vis[rejected] = (255, 255, 255)      # solid white
        if cluster_mask is not None:
            vis[cluster_mask > 0] = (0, 0, 255)  # solid red
        return vis

    @staticmethod
    def _draw_corner_brackets(vis, corner_uvs, frac=0.32, thickness=3):
        """Draw viewfinder-style L brackets at each corner, along the rectangle
        edges. ``corner_uvs`` are in ``cv2.boxPoints`` order, so consecutive
        points are adjacent corners.
        """
        accent = AttachmentPerception._OVERLAY_ACCENT
        pts = [np.array(p, dtype=float) for p in corner_uvs]
        n = len(pts)
        for i in range(n):
            p = pts[i]
            for nb in (pts[(i - 1) % n], pts[(i + 1) % n]):
                end = p + (nb - p) * frac
                cv2.line(vis, tuple(p.astype(int)), tuple(end.astype(int)), accent, thickness, cv2.LINE_AA)

    @staticmethod
    def _halo_marker(vis, pt, color, radius=6):
        """Filled dot with a white ring so it reads on any background."""
        cv2.circle(vis, pt, radius + 4, (255, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(vis, pt, radius, color, -1, cv2.LINE_AA)

    def _draw_pose_axes(self, vis, camera_info, origin_3d_cam, R_cam, length=0.06, thickness=3):
        """Project a right-handed axis gizmo of the attachment's *applied* orientation
        onto the image, so the operator can eyeball the pose before confirming.

        ``origin_3d_cam`` is the attachment center and ``R_cam`` its rotation, both in
        the camera frame; each column of ``R_cam`` is drawn as an arrow (X red, Y green,
        Z blue -- the RViz convention). Y is the holder's "up" and stays vertical; the
        perceived yaw shows up as the X/Z arrows swinging in the horizontal plane. No
        text is baked in, so the gizmo survives the central 180 deg display flip.
        """
        o = np.asarray(origin_3d_cam, dtype=float)
        if o[2] <= 1e-6:
            return  # center behind the camera; nothing sensible to project
        ou = self.world2Pixel(camera_info, world_x=o[0], world_y=o[1], world_z=o[2])
        axis_bgr = [(0, 0, 255), (0, 255, 0), (255, 0, 0)]  # X red, Y green, Z blue
        # Draw the (mostly out-of-plane, foreshortened) approach/normal axis first so the
        # in-plane axes render on top.
        for i in (2, 0, 1):
            tip = o + length * R_cam[:, i]
            if tip[2] <= 1e-6:
                continue
            tu = self.world2Pixel(camera_info, world_x=tip[0], world_y=tip[1], world_z=tip[2])
            cv2.arrowedLine(vis, (int(ou[0]), int(ou[1])), (int(tu[0]), int(tu[1])),
                            axis_bgr[i], thickness, cv2.LINE_AA, tipLength=0.25)

    def detect_attachment_color(self, bgr_image, handle_color=None, color_range=0.1):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        if handle_color is None:
            handle_color = np.array([12, 223, 169])
        h, s, v = np.asarray(handle_color, dtype=int).tolist()

        # Hue is the discriminative channel, so keep its tolerance tight while
        # letting saturation/value be looser. Scaling all three uniformly by
        # color_range made the hue band ~70 deg wide at range 0.2, which swept
        # in unrelated colors and let DBSCAN lock onto the wrong (larger) blob.
        cr = float(color_range)
        h_tol = int(cr * 90)   # half the previous hue width
        s_tol = int(cr * 255)
        v_tol = int(cr * 255)

        s_lo, s_hi = int(np.clip(s - s_tol, 0, 255)), int(np.clip(s + s_tol, 0, 255))
        v_lo, v_hi = int(np.clip(v - v_tol, 0, 255)), int(np.clip(v + v_tol, 0, 255))

        # Hue is circular (0-179 in OpenCV). When the band straddles the 0/179
        # wrap point, clipping to [0, 179] would silently drop half of it (e.g.
        # for red handles), so split it into two inRange calls and OR them.
        if h_tol >= 90:
            # Tolerance spans the whole hue circle; match on S/V only.
            return cv2.inRange(hsv,
                               np.array([0, s_lo, v_lo], dtype=np.uint8),
                               np.array([179, s_hi, v_hi], dtype=np.uint8))

        lo_h = (h - h_tol) % 180
        hi_h = (h + h_tol) % 180
        if lo_h <= hi_h:
            return cv2.inRange(hsv,
                               np.array([lo_h, s_lo, v_lo], dtype=np.uint8),
                               np.array([hi_h, s_hi, v_hi], dtype=np.uint8))
        mask_lo = cv2.inRange(hsv,
                              np.array([0, s_lo, v_lo], dtype=np.uint8),
                              np.array([hi_h, s_hi, v_hi], dtype=np.uint8))
        mask_hi = cv2.inRange(hsv,
                              np.array([lo_h, s_lo, v_lo], dtype=np.uint8),
                              np.array([179, s_hi, v_hi], dtype=np.uint8))
        return cv2.bitwise_or(mask_lo, mask_hi)

    def clean_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask
    
    def visualizeattachment(self, points):

        marker = Marker()
        marker.header.frame_id = "camera_color_optical_frame"  # IMPORTANT: match your camera TF
        marker.header.stamp = rospy.Time.now()

        marker.ns = "attachment_points"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Point size (meters)
        marker.scale.x = 0.005
        marker.scale.y = 0.005

        # Color (red)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Lifetime (0 = forever)
        marker.lifetime = rospy.Duration(0)

        # Fill points
        for x, y, z in points:
            p = Point()
            p.x = x
            p.y = y
            p.z = z
            marker.points.append(p)

        self.attachment_points_pub.publish(marker)

    def visualizeattachmentCorners(self, points_3d):
        """
        points_3d: Nx3 numpy array.
        First point is the center (green, larger).
        Remaining points are corners (blue, smaller).
        """

        # --- Center marker (green sphere) ---
        corner_marker = Marker()
        corner_marker.header.frame_id = "camera_color_optical_frame"
        corner_marker.header.stamp = rospy.Time.now()

        corner_marker.ns = "attachment_corners"
        corner_marker.id = 1
        corner_marker.type = Marker.SPHERE_LIST
        corner_marker.action = Marker.ADD

        corner_marker.scale.x = 0.015
        corner_marker.scale.y = 0.015
        corner_marker.scale.z = 0.015

        corner_marker.color.r = 0.0
        corner_marker.color.g = 0.0
        corner_marker.color.b = 1.0
        corner_marker.color.a = 1.0

        corner_marker.lifetime = rospy.Duration(0)

        for x, y, z in points_3d:
            p = Point()
            p.x = float(x)
            p.y = float(y)
            p.z = float(z)
            corner_marker.points.append(p)

        self.attachment_center_pub.publish(corner_marker)

    def pixel2World(self, camera_info, image_x, image_y, depth_image):

        # print("Image pixels: ", image_x, image_y)
        # print("Depth shape: ", depth_image.shape)

        if image_y >= depth_image.shape[0] or image_x >= depth_image.shape[1]:
            return False, None

        depth = depth_image[image_y, image_x]
        depth = depth / 1000 # convert from mm to m
        # print("Depth: ", depth)

        if math.isnan(depth) or depth < 0.05 or depth > 1.0:
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
    file_path = os.path.dirname(__file__)
    rgb_path = os.path.join(file_path, "rgb.png")
    bgr = cv2.imread(rgb_path)
    if bgr is None:
        print(f"Could not load {rgb_path}")
        sys.exit(1)

    ap = AttachmentPerception.__new__(AttachmentPerception)
    mask = ap.detect_attachment_color(bgr)

    vis = bgr.copy()
    vis[mask > 0] = (0, 0, 255)
    out_path = os.path.join(file_path, "color_test.png")
    cv2.imwrite(out_path, vis)
    print(f"Mask pixels: {np.count_nonzero(mask)}  →  saved {out_path}")