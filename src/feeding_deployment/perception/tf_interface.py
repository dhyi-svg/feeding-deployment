import os
import time
import numpy as np
from scipy.spatial.transform import Rotation
from pybullet_helpers.geometry import Pose

# This class runs on both ROS distros. The lab stack is ROS 1 Noetic; the
# single-machine Jetson deployment is ROS 2 Humble. Only __init__, updateTF and
# get_frame_to_frame_transform actually touch ROS -- everything below them is
# pure numpy and works either way.
ROS_VERSION = None
try:
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped

    ROS_VERSION = 1
except ModuleNotFoundError:
    try:
        import rclpy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time as _RclpyTime

        ROS_VERSION = 2
    except ModuleNotFoundError:
        pass

ROS_AVAILABLE = ROS_VERSION is not None


class TFInterface:
    # How long a ROS 2 transform lookup keeps retrying before giving up. Covers
    # latched /tf_static arriving after the dynamic tree (see
    # get_frame_to_frame_transform). Detection is not latency-critical, so a
    # generous default costs nothing when the tree is already complete.
    TF_LOOKUP_TIMEOUT_SEC = 10.0

    def __init__(self):
        self.tfBuffer = None
        self.listener = None
        self.broadcaster = None
        self._node = None
        self.tf_lookup_timeout_sec = float(
            os.environ.get("TF_LOOKUP_TIMEOUT_SEC", self.TF_LOOKUP_TIMEOUT_SEC)
        )
        if not ROS_AVAILABLE:
            return

        if ROS_VERSION == 1:
            self.tfBuffer = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.tfBuffer)
            self.broadcaster = tf2_ros.TransformBroadcaster()
            time.sleep(1.0)
            return

        # ROS 2: every tf2 object needs an explicit node, and something must
        # spin it -- get_node() owns that shared node and its executor thread.
        from feeding_deployment.ros2.node import get_node

        self._node = get_node()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self._node)
        self.broadcaster = tf2_ros.TransformBroadcaster(self._node)
        # Give the listener a moment to accumulate the tree before first lookup.
        time.sleep(1.0)

    def updateTF(self, source_frame, target_frame, pose):
        if not ROS_AVAILABLE:
            return
        t = TransformStamped()

        if ROS_VERSION == 1:
            t.header.stamp = rospy.Time.now()
        else:
            t.header.stamp = self._node.get_clock().now().to_msg()
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
        if not ROS_AVAILABLE:
            print("No ROS available; cannot look up", frame_A, "->", target_frame)
            return None

        stamp = camera_info_data.header.stamp
        try:
            if ROS_VERSION == 1:
                return self.tfBuffer.lookup_transform(
                    frame_A,
                    target_frame,
                    rospy.Time(secs=stamp.secs, nsecs=stamp.nsecs),
                )

            # ROS 2 renamed the stamp fields (secs/nsecs -> sec/nanosec);
            # CameraInfoCompat answers to both, a raw ROS 2 message only to the
            # new names.
            from feeding_deployment.ros2.compat import stamp_to_sec_nanosec

            sec, nanosec = stamp_to_sec_nanosec(stamp)
            return self.tfBuffer.lookup_transform(
                frame_A,
                target_frame,
                _RclpyTime(seconds=sec, nanoseconds=nanosec),
            )
        except Exception as e:
            # Falling back to the latest available transform: the arm is static
            # or near-static during a detection, so the newest TF is as good as
            # the frame-stamped one, and this is the difference between a usable
            # detection and none at all when the camera clock lags the tf tree.
            #
            # The retry matters as much as the fallback. /tf_static is latched
            # (TRANSIENT_LOCAL), so a listener that has only just come up can
            # have the dynamic tree from robot_state_publisher but not yet the
            # static calibration -- tf2 then reports "two or more unconnected
            # trees" and a single immediate retry fails too. Waiting a few
            # seconds is the difference between a working detection and none.
            if ROS_VERSION == 2:
                deadline = time.time() + self.tf_lookup_timeout_sec
                last_error = e
                warned = False
                while time.time() < deadline:
                    try:
                        transform = self.tfBuffer.lookup_transform(
                            frame_A, target_frame, _RclpyTime()
                        )
                        print(
                            f"tf2 lookup at the frame stamp failed ({e}); "
                            f"using the latest {frame_A} <- {target_frame} instead."
                        )
                        return transform
                    except Exception as e2:
                        last_error = e2
                        if not warned:
                            print(
                                f"Waiting up to {self.tf_lookup_timeout_sec:g}s for "
                                f"{frame_A} <- {target_frame} (tf tree still incomplete) ..."
                            )
                            warned = True
                        time.sleep(0.1)
                e = last_error
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
        # ROS 1 spells these secs/nsecs, ROS 2 sec/nanosec.
        secs = getattr(stamp, "secs", None)
        nsecs = getattr(stamp, "nsecs", None)
        if secs is None:
            secs, nsecs = int(stamp.sec), int(stamp.nanosec)
        return {
            "parent": transform.header.frame_id,
            "child": transform.child_frame_id,
            "stamp": {"secs": int(secs), "nsecs": int(nsecs)},
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
