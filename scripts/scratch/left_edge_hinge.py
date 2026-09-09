"""Standalone, detect-only: estimate the hinge as the LEFT edge of the raw YOLO
handle bounding box, deprojected via depth + the same camera->arm_base_link
transform AppliancePerception uses internally (get_frame_to_frame_transform +
make_homogeneous_transform, same pattern as _apply_handle_corrections /
detect_start_button in appliance_perception.py). No motion, no grasp --
geometry only, for comparison against the door-plane 'farthest edge' hinge
estimate (~35cm) and the stale hand-measured radius (~12cm, per the user:
not confirmed correct).
"""
import numpy as np
import supervision as sv
from ultralytics import YOLO

from feeding_deployment.perception.appliance_perception.appliance_perception import AppliancePerception
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface

COCO_MICROWAVE_CLASS_ID = 68
YOLO_MODEL = "yolo26s.pt"


class _YoloGroundingDinoAdapter:
    def __init__(self, model_name=YOLO_MODEL):
        self._model = YOLO(model_name)

    def predict_with_classes(self, image, classes, box_threshold, text_threshold):
        del classes, text_threshold
        results = self._model.predict(
            image, classes=[COCO_MICROWAVE_CLASS_ID], conf=box_threshold, verbose=False,
        )
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            return sv.Detections(
                xyxy=np.zeros((0, 4), dtype=float),
                confidence=np.zeros((0,), dtype=float),
                class_id=np.zeros((0,), dtype=int),
            )
        return sv.Detections(
            xyxy=boxes.xyxy.cpu().numpy(),
            confidence=boxes.conf.cpu().numpy(),
            class_id=np.zeros(len(boxes), dtype=int),
        )


class _YoloGroundedSamShim:
    def __init__(self):
        self.grounding_dino_model = _YoloGroundingDinoAdapter()


rs = RealSenseROS2Interface()
if not rs.wait_for_frames(30.0):
    raise SystemExit("No RGB-D frames")

apc = AppliancePerception(_YoloGroundedSamShim())

d = rs.get_camera_data()
rgb, depth, cam_info = d["rgb_image"], d["depth_image"], d["camera_info"]

hh, _, _, top = apc.detect_handle_and_placement("microwave handle", rgb, cam_info, depth)
if hh is None:
    raise SystemExit("NO DETECTION")
handle_pos = np.asarray(hh.position)
print(f"handle (existing pipeline): {np.round(handle_pos, 4)}")

# Raw YOLO box on the SAME frame, to get the left edge in pixel space.
# CAMERA_UPSIDE_DOWN is false on this rig (set via env for the main script;
# read directly here since detect_items applies the flip internally and we
# want the box in the SAME orientation detect_handle_and_placement used).
adapter = apc.grounding_dino_model
det = adapter.predict_with_classes(rgb, ["microwave handle"], apc.BOX_THRESHOLD, apc.TEXT_THRESHOLD)
if len(det.xyxy) == 0:
    raise SystemExit("NO RAW BOX DETECTION")
box = det.xyxy[np.argmax(det.confidence)]
x1, y1, x2, y2 = box
left_x = int(round(x1))
mid_y = int(round((y1 + y2) / 2))
print(f"raw box: {np.round(box, 1)}  left edge px ({left_x}, {mid_y})")

ok, hinge_cam = apc.pixel2World(cam_info, left_x, mid_y, depth, use_surrounding_pixels=True)
if not ok:
    raise SystemExit("Could not get valid depth at the left-edge pixel")

transform = apc.get_frame_to_frame_transform(cam_info)
if transform is None:
    raise SystemExit("No transform arm_base_link <-> camera_color_optical_frame")
base_to_camera = apc.make_homogeneous_transform(transform)
camera_to_point = np.eye(4)
camera_to_point[:3, 3] = hinge_cam
hinge_base = (base_to_camera @ camera_to_point)[:3, 3]

radius = float(np.linalg.norm(hinge_base - handle_pos))
print(f"hinge (left-edge-based): {np.round(hinge_base, 4)}")
print(f"radius (handle -> left-edge hinge): {radius*100:.1f} cm")
print("DETECT ONLY -- nothing commanded.")
