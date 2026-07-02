import time
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from pybullet_helpers.geometry import Pose


class TFInterface:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
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

    def get_frame_to_frame_transform(self, camera_info_data, frame_A="arm_base_link", target_frame="camera_color_optical_frame"):
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

    # -- detection-input logging (offline replay) ---------------------------
    # Enough to re-run any detection later without the robot: the camera intrinsics,
    # the base<-camera transform(s), and the scalar knobs (color/range/orientation).
    # Paired with the rgb/depth images the same detection already logs.

    @staticmethod
    def _jsonable(value):
        """Coerce numpy scalars/arrays (and containers of them) to plain Python so the
        logged JSON stays numerically faithful -- json's ``default=str`` fallback would
        otherwise stringify numpy floats and break offline replay."""
        if value is None or isinstance(value, (bool, int, float, str)):
            return value
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, dict):
            return {k: TFInterface._jsonable(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [TFInterface._jsonable(v) for v in value]
        return value

    @staticmethod
    def camera_info_to_dict(camera_info):
        """Everything needed to reproduce the pinhole projection (pixel<->world) offline."""
        if camera_info is None:
            return None
        header = getattr(camera_info, "header", None)
        stamp = getattr(header, "stamp", None)
        info = {
            "frame_id": getattr(header, "frame_id", None),
            "width": int(getattr(camera_info, "width", 0)),
            "height": int(getattr(camera_info, "height", 0)),
            "distortion_model": getattr(camera_info, "distortion_model", ""),
            "K": [float(x) for x in getattr(camera_info, "K", [])],
            "D": [float(x) for x in getattr(camera_info, "D", [])],
            "R": [float(x) for x in getattr(camera_info, "R", [])],
            "P": [float(x) for x in getattr(camera_info, "P", [])],
        }
        if stamp is not None:
            info["stamp"] = {"secs": int(stamp.secs), "nsecs": int(stamp.nsecs)}
        return info

    def transform_to_dict(self, transform):
        """A ``TransformStamped`` as translation+quaternion and a 4x4 homogeneous matrix
        (parent_from_child), so offline code can use it directly."""
        if transform is None:
            return None
        t = transform.transform.translation
        q = transform.transform.rotation
        stamp = transform.header.stamp
        return {
            "parent": transform.header.frame_id,
            "child": transform.child_frame_id,
            "stamp": {"secs": int(stamp.secs), "nsecs": int(stamp.nsecs)},
            "translation": [float(t.x), float(t.y), float(t.z)],
            "quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
            "matrix": self.make_homogeneous_transform(transform).tolist(),
        }

    def _log_detection_inputs(self, detector, camera_info=None, transform=None,
                              extra_transforms=None, **params):
        """Log every non-image input to a detection as a JSON sidecar next to its
        rgb/depth frames, so the detection can be re-run offline.

        ``transform`` / ``extra_transforms`` are ``TransformStamped`` (the base<-camera
        lookups); ``params`` are the detector's scalar knobs. Best-effort and silent on
        failure -- logging must never disturb a live detection. No-op when the class has
        no data logger attached.
        """
        logger = getattr(self, "_data_logger", None)
        if logger is None or not hasattr(logger, "log_json"):
            return
        try:
            tfs = {}
            for tr in ([transform] if transform is not None else []) + list(extra_transforms or []):
                d = self.transform_to_dict(tr)
                if d is not None:
                    tfs[f"{d['parent']}__from__{d['child']}"] = d
            payload = {
                "detector": detector,
                "camera_info": self.camera_info_to_dict(camera_info),
                "transforms": tfs,
                "params": {k: self._jsonable(v) for k, v in params.items()},
            }
            logger.log_json("detection_inputs", payload)
        except Exception as e:  # noqa: BLE001 -- never let logging break detection
            print(f"[tf_interface] Failed to log detection inputs for {detector}: {e}")
