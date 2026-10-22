"""Every fridge-specific constant for `real_gen3_ros2_grasp_and_swing_microwave.py --fridge`.

The values come from the fridge branch (`origin/detect-fridge-handle`, scripts/real_gen3_ros2_sam3_grasp_fridge.py
and scripts/detect_handle_sam3.py at 351c9231), tuned on rchi-cpu-5 (old arm_server stack, RTX 4070 Ti) in 09-20..09-25.
None has been re-checked on the Jetson + sheppy rig: the microwave's GRIP_EXT moved 9.8 cm between those two rigs,
so touch-check GRIP_EXT and HANDLE_DEPTH_CORR before trusting a grasp.

`OVERRIDES` (bottom) replaces the microwave script's module constants of the same name; the rest are used by
fridge_detect.py and the --fridge branches of the script.
"""
from pathlib import Path

import numpy as np

# --- detection: SAM 3 segments the handle from colour, depth gives its distance (fridge_detect.py) ---
# Why not the microwave's YOLO + door plane: at grasp range the fridge door fills the frame (no appliance box), and
# the handle sits ~11 mm off the door, inside the plane fit's 20 mm band, so the plane swallows it.
SAM3_PROMPT = "handle"
SAM3_REPO = "MTerryJack/sam3"     # ungated mirror of facebook/sam3 (safetensors only)
SAM3_DTYPE = "bf16"               # falls back to fp32 on CPU
SAM3_THRESHOLD = 0.5
MIN_MASK_PX = 300                 # smaller than this is noise, not a handle
MIN_VALID_DEPTH_FRAC = 0.30       # refuse a mask whose depth is mostly holes
DEPTH_RANGE_M = (0.05, 2.0)
# Added along the camera's optical axis before the transform. 09-20 touch check on rchi-cpu-5 (fixed offset, repeatable
# to 0.2 cm at 13 and 19 cm). Calibration-specific: re-measure on this rig.
HANDLE_DEPTH_CORR = 0.051
# Nothing detects the door, so its normal is assumed: facing the arm head-on, out of the door toward the arm (-x).
# With the detector's fixed handle orientation this gives a 0-deg yaw in grasp_pose, i.e. the fridge branch's grasp.
DOOR_NORMAL = np.array([-1.0, 0.0, 0.0])
MAX_LOOKS = 5                     # keep looking until two detections agree within DETECT_AGREE
MAX_EMPTY_RETRIES = 8             # per look: retries on a frame with no usable mask
DETECT_AGREE = 0.03
PLAUSIBLE_X = (0.30, 0.90)
PLAUSIBLE_Y = (-0.60, 0.60)
PLAUSIBLE_Z = (0.25, 0.75)
DETECT_FRAMES_DIR = Path.home() / "fridge_detections"

# --- grasp ---
# Along the approach axis, positive = short of the detected handle front face. Walked -0.020 -> +0.037 over ~10 grasps
# on rchi-cpu-5 without converging: suspect the fingers nudging the door, not the constant.
GRIP_EXT = 0.037
GRASP_X_ADJUST = 0.0              # the fridge branch had no separate adjust; GRIP_EXT is the whole offset
VERTICAL_CORR = 0.0               # validated 09-20: no height bias
PREGRASP_STANDOFFS = (0.12, 0.10) # fridge branch PRE_STANDOFF 0.12
GRASP_PITCHES_DEG = (0,)          # level only: the fridge handle is high; the nose-down tilts are for a low microwave
MAX_REACH, MIN_Z, MAX_Z = 0.94, 0.25, 0.75
GRASP_FORCE = None                # SheppyArm default (0.5). The microwave's looser 0.25 stops it tipping; the fridge
                                  # door's seal needs a firm hold (the fridge branch closed at full force)

# --- hinge: measured, relative to the grasp point (the fridge gets moved between runs) ---
HINGE_ALONG_DOOR_M = 14.5 * 0.0254          # 36.8 cm left of the handle middle (+y, image-left), user measurement 09-20
HANDLE_CENTRE_TO_FACE_M = 0.020             # half the ~4 cm handle depth
PIN_BEHIND_FACE_M = 1.25 * 0.0254 + 0.015   # 1.25 in door thickness + 1.5 cm empirical (hinge side crept forward)
HINGE_OFFSET_FROM_GRASP = np.array([HANDLE_CENTRE_TO_FACE_M + PIN_BEHIND_FACE_M, HINGE_ALONG_DOOR_M, 0.0])
HINGE_RADIUS_RANGE = (0.30, 0.45)
DOOR_FILE = Path.home() / ".fridge_door.json"   # hinge + closed-door grasp saved here (never the microwave's file)

# --- swing: one wide stiff swing, no push-open ---
SWING_DIRECTION = -1              # pulls the handle toward the arm (sim-verified 09-20)
SWING_MAX_TRY_DEG = 90            # fridge branch reached ~79-100 deg; tries this down to SWING_MIN_DEG in 5-deg steps
SWING_MIN_DEG = 20
SWING_MAX_REACH = 0.94

OVERRIDES = dict(
    GRIP_EXT=GRIP_EXT, GRASP_X_ADJUST=GRASP_X_ADJUST, VERTICAL_CORR=VERTICAL_CORR,
    PREGRASP_STANDOFFS=PREGRASP_STANDOFFS, GRASP_PITCHES_DEG=GRASP_PITCHES_DEG,
    MAX_REACH=MAX_REACH, MIN_Z=MIN_Z, MAX_Z=MAX_Z, GRASP_FORCE=GRASP_FORCE,
    HINGE_OFFSET_FROM_GRASP=HINGE_OFFSET_FROM_GRASP, HINGE_RADIUS_RANGE=HINGE_RADIUS_RANGE, DOOR_SAVE_FILE=DOOR_FILE,
    SWING_DIRECTION=SWING_DIRECTION, SWING_MAX_TRY_DEG=SWING_MAX_TRY_DEG, SWING_MIN_DEG=SWING_MIN_DEG,
    SWING_MAX_REACH=SWING_MAX_REACH,
)
