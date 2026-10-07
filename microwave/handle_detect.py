"""Microwave handle + door face from one RGB-D frame, in arm_base_link.

The handle-finding half of `AppliancePerception.detect_handle_and_placement`, copied as-is
(same thresholds, same order of operations) so the grasp constants tuned against it still hold:

1. YOLO box around the microwave (COCO 'microwave', else the classes it confuses it with).
2. Every box pixel with valid depth -> 3D point (camera frame).
3. RANSAC plane = the door face; reject it if its normal isn't horizontal (table/floor).
4. Points 0-7 cm in front of the plane -> DBSCAN -> the most vertical, elongated cluster.
5. Handle = the cluster's median, with camera y set 4 cm in from its extreme.
6. Hinge = the door-face point on the edge farthest from the handle, nearest the handle's height.
   Reject if that edge is not clearly on the far side (the plane pulled in background).
7. Into arm_base_link via tf2 (TFInterface, shared with the rest of the repo).

Dropped vs the original: the hinge/placement/top-of-appliance poses, overlays, RViz markers,
image logging, the camera flip (this rig's camera is upright) and HANDLE_DEPTH_CORR (0 here).
"""
import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from ultralytics import YOLO

from feeding_deployment.perception.tf_interface import TFInterface

COCO_MICROWAVE_CLASS_ID = 68
YOLO_MODEL = "yolo26s.pt"
# COCO classes YOLO confuses this microwave with (bus 5, train 6, suitcase 28, oven 69,
# refrigerator 72), used only when it finds no 'microwave' at all.
YOLO_FALLBACK_CLASS_IDS = [5, 6, 28, 69, 72]
BOX_THRESHOLD = 0.3

DEPTH_RANGE_M = (0.05, 2.0)
PLANE_DIST_M = 0.02
HANDLE_PROTRUSION_MAX_M = 0.07     # how far in front of the door plane a handle can sit
DBSCAN_EPS_M, DBSCAN_MIN_SAMPLES = 0.02, 50
HANDLE_MIN_CLUSTER_FRACTION = 0.25  # clusters below this fraction of the biggest are noise
HANDLE_ELONGATION_CAP = 10.0        # so a very thin sliver can't dominate the score
HANDLE_Y_FROM_TOP_M = 0.04          # handle point: this far in from the cluster's max camera y
MAX_DOOR_NORMAL_VERTICAL = 0.5      # door is vertical, so its normal must be ~horizontal
# fixed handle orientation the detector has always returned (the grasp yaws it to the face)
HANDLE_QUAT = np.array([-0.5, 0.5, 0.5, -0.5])


class HandleDetector(TFInterface):
    def __init__(self):
        super().__init__()
        self._yolo = YOLO(YOLO_MODEL)

    def _box(self, rgb):
        """Highest-confidence microwave box (x1, y1, x2, y2), or None."""
        res = self._yolo.predict(rgb, classes=[COCO_MICROWAVE_CLASS_ID], conf=BOX_THRESHOLD, verbose=False)[0]
        if len(res.boxes) == 0:
            res = self._yolo.predict(rgb, classes=YOLO_FALLBACK_CLASS_IDS, conf=BOX_THRESHOLD, verbose=False)[0]
            if len(res.boxes) == 0:
                return None
            best = int(res.boxes.conf.argmax())
            print(f"  (YOLO fallback: no 'microwave'; using '{res.names[int(res.boxes.cls[best])]}' "
                  f"{float(res.boxes.conf[best]):.2f})")
        else:
            best = int(res.boxes.conf.argmax())
            print(f"  microwave {float(res.boxes.conf[best]):.2f}")
        return res.boxes.xyxy[best].cpu().numpy().astype(int)

    def detect(self, rgb, camera_info, depth):
        """Returns dict(handle, quat, normal, hinge, door_z, door_mid) in arm_base_link, or None.
        normal: horizontal door-face normal, out of the door toward the camera.
        hinge: door-face point on the edge farthest from the handle, nearest the handle's height
          (the swing pivots about the vertical line through it).
        door_z: (low, high) of the door face (1st/99th percentile height).
        door_mid: door-face point at the middle of its horizontal span."""
        transform = self.get_frame_to_frame_transform(camera_info)
        if transform is None:
            print("no arm_base_link <- camera transform")
            return None
        base_T_cam = self.make_homogeneous_transform(transform)
        up_cam = base_T_cam[2, :3]   # base +z in the camera frame

        box = self._box(rgb)
        if box is None:
            print("  no microwave box")
            return None
        x1, y1, x2, y2 = box

        # box pixels -> camera-frame points (row-major, like the original's np.where on the mask)
        mask = np.zeros(rgb.shape[:2], dtype=bool)
        mask[y1:y2, x1:x2] = True
        vs, us = np.where(mask)
        d = depth[vs, us] / 1000.0
        ok = ~np.isnan(d) & (d >= DEPTH_RANGE_M[0]) & (d <= DEPTH_RANGE_M[1])
        us, vs, d = us[ok], vs[ok], d[ok]
        if len(d) == 0:
            print("  no valid depth in the box")
            return None
        fx, fy, cx, cy = camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]
        pts = np.stack([d / fx * (us - cx), d / fy * (vs - cy), d], axis=1)

        # door plane
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        plane_model, inliers = pcd.segment_plane(distance_threshold=PLANE_DIST_M, ransac_n=3, num_iterations=500)
        plane_pts = pts[inliers]
        off_plane = pts[np.setdiff1d(np.arange(len(pts)), inliers)]
        normal, offset = np.asarray(plane_model[:3], float), float(plane_model[3])
        nn = np.linalg.norm(normal)
        normal, offset = normal / nn, offset / nn
        if np.dot(normal, plane_pts.mean(axis=0)) > 0:   # point toward the camera: "in front" is +
            normal, offset = -normal, -offset
        if abs(float(np.dot(normal, up_cam))) > MAX_DOOR_NORMAL_VERTICAL:
            print("  plane is not a door (normal not horizontal -- table/floor?)")
            return None

        # protruding points -> clusters -> the vertical, elongated one
        signed = off_plane @ normal + offset
        handle_pts = off_plane[(signed > 0.0) & (signed < HANDLE_PROTRUSION_MAX_M)]
        if len(handle_pts) == 0:
            print("  nothing protruding in front of the door plane")
            return None
        labels = DBSCAN(eps=DBSCAN_EPS_M, min_samples=DBSCAN_MIN_SAMPLES).fit(handle_pts).labels_
        if not np.any(labels >= 0):
            print("  DBSCAN found no clusters")
            return None
        unique, counts = np.unique(labels[labels >= 0], return_counts=True)
        best_label, best_score = None, -1.0
        for label in unique[counts >= HANDLE_MIN_CLUSTER_FRACTION * counts.max()]:
            cluster = handle_pts[labels == label]
            _, s, vt = np.linalg.svd(cluster - cluster.mean(axis=0), full_matrices=False)
            score = abs(float(vt[0] @ up_cam)) * min(float(s[0] / max(s[1], 1e-9)), HANDLE_ELONGATION_CAP)
            if score > best_score:
                best_label, best_score = label, score
        cluster = handle_pts[labels == best_label]
        handle_cam = np.median(cluster, axis=0)
        handle_cam[1] = np.max(cluster[:, 1]) - HANDLE_Y_FROM_TOP_M

        # door edges along the plane's own horizontal axis (percentiles: RANSAC's edge points drift)
        axis = np.cross(normal, up_cam)
        axis /= np.linalg.norm(axis)
        proj = plane_pts @ axis
        h_proj = float(handle_cam @ axis)
        lo, hi = float(np.percentile(proj, 1)), float(np.percentile(proj, 99))
        edge = lo if abs(h_proj - lo) > abs(h_proj - hi) else hi
        strip = plane_pts[np.abs(proj - edge) < 0.02]
        hinge_cam = strip[int(np.argmin(np.linalg.norm(strip - handle_cam, axis=1)))]
        if abs(float(hinge_cam @ axis) - h_proj) < 0.5 * max(abs(h_proj - lo), abs(h_proj - hi)):
            print("  far door edge is on the handle's side -- plane includes background")
            return None

        # to arm_base_link
        Rb, tb = base_T_cam[:3, :3], base_T_cam[:3, 3]
        n_base = Rb @ normal
        n_base[2] = 0.0
        plane_base = plane_pts @ Rb.T + tb
        return {
            "handle": Rb @ handle_cam + tb,
            "quat": HANDLE_QUAT,
            "normal": n_base / np.linalg.norm(n_base),
            "hinge": Rb @ hinge_cam + tb,
            "door_z": (float(np.percentile(plane_base[:, 2], 1)), float(np.percentile(plane_base[:, 2], 99))),
            "door_mid": plane_base[int(np.argmin(np.abs(proj - 0.5 * (lo + hi))))].copy(),
        }
