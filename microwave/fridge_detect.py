"""Fridge handle detectors for real_gen3_ros2_grasp_and_swing_microwave.py --fridge (`HANDLE_DETECTOR` in
fridge_constants.py picks one). Both have HandleDetector's interface -- `detect(rgb, camera_info, depth)` -> dict or
None -- so the script's own detect_handle() runs them (two agreeing looks, plausibility, frame saving).

"depth" (default): `FridgeDepthDetector`, the microwave's handle_detect.HandleDetector unchanged (door plane by RANSAC,
the vertical cluster sticking out of it = handle), searching the WHOLE image instead of a YOLO microwave box: at grasp
range the door fills the frame, and on the 10-21 Jetson frames of this fridge YOLO gave 'cell phone' on the handle and
'chair' on the door. Colour-blind. Found the handle in 4/4 of those frames.

"sam3": `FridgeSam3Detector`, scripts/detect_handle_sam3.py's SAM 3 model and mask -> point, with its own defaults.
It sees only the handle: the door normal is assumed and there is no hinge or door extent (None).
"""
import sys
from pathlib import Path

import numpy as np

import fridge_constants as F
import handle_detect
from feeding_deployment.perception.tf_interface import TFInterface


class FridgeDepthDetector(handle_detect.HandleDetector):
    """handle_detect.HandleDetector over the whole image. No YOLO."""

    def __init__(self):
        TFInterface.__init__(self)                  # skips HandleDetector's YOLO load: the region is the whole image

    def _box(self, rgb):
        return np.array([0, 0, rgb.shape[1], rgb.shape[0]])


class FridgeSam3Detector(TFInterface):
    """scripts/detect_handle_sam3.py's model and mask -> point, with HandleDetector's detect() interface."""

    def __init__(self):
        super().__init__()
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
        import detect_handle_sam3                   # torch + transformers, only when asked for
        self._s3 = detect_handle_sam3
        self._model = detect_handle_sam3.Sam3HandleDetector()

    def detect(self, rgb, camera_info, depth):
        seg = self._model.segment(np.ascontiguousarray(rgb[:, :, ::-1]))   # bgr8 -> rgb
        if seg is None or seg[0].sum() < self._s3.MIN_MASK_PX:
            print(f"  SAM 3: no handle mask for {self._model.prompt!r}")
            return None
        try:
            cam_xyz, _, _ = self._s3.mask_to_camera_xyz(seg[0], depth, camera_info)
        except RuntimeError as e:
            print(f"  SAM 3: {e}")
            return None
        transform = self.get_frame_to_frame_transform(camera_info)
        if transform is None:
            print("no arm_base_link <- camera transform")
            return None
        cam_xyz = cam_xyz + np.array([0.0, 0.0, self._s3.HANDLE_DEPTH_CORR_M])   # its calibration correction
        print(f"  SAM 3 mask score {seg[1]:.2f}")
        return {"handle": (self.make_homogeneous_transform(transform) @ np.r_[cam_xyz, 1.0])[:3],
                "quat": np.asarray(self._s3.GRASP_QUAT), "normal": F.SAM3_DOOR_NORMAL.copy(),
                "hinge": None, "door_z": None, "door_mid": None}


DETECTORS = {"depth": FridgeDepthDetector, "sam3": FridgeSam3Detector}
