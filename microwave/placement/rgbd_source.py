"""Camera acquisition for placement perception: one calibrated RGB-D frame, whatever the camera.

An `RGBDFrame` is everything the rest of the pipeline may use about a camera: the colour image,
a depth image ALIGNED to it (metres), its 3x3 intrinsics K, the 4x4 `base_T_cam`
(arm_base_link <- the camera's optical frame) and that frame's name. Nothing downstream knows
where the camera is mounted -- a wrist camera and a fixed scene camera differ only in the
`base_T_cam` they deliver (a wrist camera's changes with the arm; a fixed camera's is constant).

Sources:
  RosRGBDSource     live: any ROS 2 driver publishing colour + aligned depth + CameraInfo; the
                    transform is looked up in tf2 from CameraInfo.header.frame_id, so a different
                    camera only needs its topics and a calibration on tf. Default topics = the sheppy
                    wrist camera (microwave/sheppy/sheppy-manifest.yaml: realsense2_camera, aligned
                    depth, 640x480x15, calibration_tf -> camera_color_optical_frame).
  load_recorded     a saved frame: `placement_workflow.Frame.save` .npz (bgr, depth_mm, K, base_T_cam),
                    `microwave_common.save_frame` .npz (+ its _rgb.jpg), or a teleop-capture frame dir
                    (frame_rgb.jpg, frame_depth.png, frame_detection_inputs.json).

Depth is in metres here (the RealSense driver publishes uint16 millimetres; converted on the way in).
Distortion is not applied: the RealSense colour stream is published rectified (D = 0 on this rig) --
a camera with real distortion would have to be undistorted in its source.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

BASE_FRAME = "arm_base_link"      # = Part 1's base_link (calibration_tf publishes the alias)


class AcquisitionError(RuntimeError):
    """No usable frame (message says why)."""


@dataclass
class CameraConfig:
    color_topic: str = "/wrist_camera/color/image_raw"
    info_topic: str = "/wrist_camera/color/camera_info"
    depth_topic: str = "/wrist_camera/aligned_depth_to_color/image_raw"
    base_frame: str = BASE_FRAME
    first_frame_timeout_s: float = 30.0
    max_frame_age_s: float = 1.0       # an older frame means the stream stalled


@dataclass
class RGBDFrame:
    bgr: np.ndarray                    # HxWx3 uint8
    depth_m: np.ndarray                # HxW float32, metres, aligned to bgr; 0 / nan = no depth
    K: np.ndarray                      # 3x3
    base_T_cam: np.ndarray             # 4x4, base_frame <- camera optical frame
    frame_id: str = "camera_color_optical_frame"
    base_frame: str = BASE_FRAME
    stamp: float = 0.0
    source: str = ""
    extra: dict = field(default_factory=dict)

    def __post_init__(self):
        self.K = np.asarray(self.K, float).reshape(3, 3)
        self.base_T_cam = np.asarray(self.base_T_cam, float).reshape(4, 4)
        self.depth_m = np.asarray(self.depth_m, np.float32)
        if self.bgr.shape[:2] != self.depth_m.shape[:2]:
            raise AcquisitionError(f"colour {self.bgr.shape[:2]} and depth {self.depth_m.shape[:2]} differ in size "
                                   "-- depth must be aligned to colour")
        Rm = self.base_T_cam[:3, :3]
        if not np.allclose(Rm @ Rm.T, np.eye(3), atol=1e-3):
            raise AcquisitionError("base_T_cam rotation is not orthonormal")

    @property
    def cam_pos(self):
        return self.base_T_cam[:3, 3]

    def save(self, path):
        """Same keys as placement_workflow.Frame (depth in mm) + frame names: loadable by both."""
        np.savez_compressed(path, bgr=self.bgr, depth_mm=self.depth_m * 1000.0, K=self.K.reshape(-1),
                            base_T_cam=self.base_T_cam, stamp=self.stamp, frame_id=self.frame_id,
                            base_frame=self.base_frame)


# ---------------------------------------------------------------------------
# recorded frames
# ---------------------------------------------------------------------------
def load_recorded(path):
    """One RGBDFrame from a saved file or capture directory (formats in the module docstring)."""
    import cv2
    p = Path(path).expanduser()
    if p.is_dir():
        inputs = p / "frame_detection_inputs.json"
        if not inputs.exists():
            raise AcquisitionError(f"{p}: no frame_detection_inputs.json (not a capture frame dir)")
        meta = json.loads(inputs.read_text())
        ci = meta["camera_info"]
        tr = next((v for k, v in meta.get("transforms", {}).items() if v.get("child") == ci["frame_id"]), None)
        if tr is None:
            raise AcquisitionError(f"{p}: no transform to {ci['frame_id']} recorded")
        bgr = cv2.imread(str(p / "frame_rgb.jpg"))
        depth = cv2.imread(str(p / "frame_depth.png"), cv2.IMREAD_UNCHANGED)
        if bgr is None or depth is None:
            raise AcquisitionError(f"{p}: frame_rgb.jpg / frame_depth.png missing")
        st = ci.get("stamp", {})
        return RGBDFrame(bgr, depth.astype(np.float32) / 1000.0, ci["K"], tr["matrix"], ci["frame_id"],
                         tr.get("parent", BASE_FRAME), st.get("secs", 0) + st.get("nsecs", 0) * 1e-9, str(p))
    if p.suffix != ".npz" or not p.exists():
        raise AcquisitionError(f"{p}: not an .npz frame or a capture frame dir")
    d = np.load(p, allow_pickle=False)
    if "base_T_cam" not in d:
        raise AcquisitionError(f"{p}: no base_T_cam saved -- the frame cannot be put in the arm's frame")
    if "bgr" in d:
        bgr = d["bgr"]
    else:
        jpg = p.with_name(p.stem + "_rgb.jpg")
        bgr = cv2.imread(str(jpg))
        if bgr is None:
            raise AcquisitionError(f"{p}: no colour image (bgr key or {jpg.name})")
    stamp = float(d["stamp"]) if "stamp" in d else 0.0
    frame_id = str(d["frame_id"]) if "frame_id" in d else "camera_color_optical_frame"
    base = str(d["base_frame"]) if "base_frame" in d else BASE_FRAME
    return RGBDFrame(bgr, d["depth_mm"].astype(np.float32) / 1000.0, d["K"], d["base_T_cam"], frame_id, base,
                     stamp, str(p))


class RecordedSource:
    """Saved frames, cycled. `get()` matches RosRGBDSource."""

    def __init__(self, paths):
        self.frames = [load_recorded(x) for x in paths]
        if not self.frames:
            raise AcquisitionError("no recorded frames given")
        self.i = 0

    def get(self):
        f = self.frames[self.i % len(self.frames)]
        self.i += 1
        return f


# ---------------------------------------------------------------------------
# live ROS 2
# ---------------------------------------------------------------------------
class RosRGBDSource:
    """Latest synchronised colour + aligned depth + CameraInfo, and tf2's base_frame <- CameraInfo's
    frame. Reads topics only."""

    def __init__(self, cfg: CameraConfig = None):
        from feeding_deployment.perception.tf_interface import TFInterface
        from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
        self.cfg = cfg or CameraConfig()
        self.rs = RealSenseROS2Interface(color_topic=self.cfg.color_topic, camera_info_topic=self.cfg.info_topic,
                                         depth_topic=self.cfg.depth_topic)
        if not self.rs.wait_for_frames(self.cfg.first_frame_timeout_s):
            raise AcquisitionError(f"no RGB-D frames on {self.cfg.color_topic} / {self.cfg.depth_topic} within "
                                   f"{self.cfg.first_frame_timeout_s:.0f} s (camera up, aligned depth on?)")
        self.tf = TFInterface()

    def get(self):
        from feeding_deployment.ros2.compat import stamp_to_sec_nanosec
        d = self.rs.get_camera_data()
        bgr, depth, cam = d["rgb_image"], d["depth_image"], d["camera_info"]
        if bgr is None or depth is None or cam is None:
            raise AcquisitionError("no camera frame")
        sec, nsec = stamp_to_sec_nanosec(cam.header.stamp)
        stamp = sec + nsec * 1e-9
        if time.time() - stamp > self.cfg.max_frame_age_s:
            raise AcquisitionError(f"camera frame is {time.time() - stamp:.1f} s old -- the stream has stalled")
        frame_id = cam.header.frame_id
        tr = self.tf.get_frame_to_frame_transform(cam, self.cfg.base_frame, frame_id)
        if tr is None:
            raise AcquisitionError(f"no {self.cfg.base_frame} <- {frame_id} transform on tf (calibration published?)")
        return RGBDFrame(bgr, np.asarray(depth, np.float32) / 1000.0, np.asarray(cam.K, float).reshape(-1),
                         self.tf.make_homogeneous_transform(tr), frame_id, self.cfg.base_frame, stamp, "ros")
