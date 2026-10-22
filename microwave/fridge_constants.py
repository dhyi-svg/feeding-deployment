"""Fridge constants for `real_gen3_ros2_grasp_and_swing_microwave.py --fridge`: only where the fridge needs a value the
microwave script doesn't have or can't use. Everything else is the microwave's own (detection, approach, tilts, swing
checks, VERTICAL_CORR, and GRIP_EXT / GRASP_X_ADJUST with the depth detector). Values are the fridge branch's
(`origin/detect-fridge-handle` at 351c9231, rchi-cpu-5, 09-20..09-25) unless noted.

`OVERRIDES` (bottom) replaces the microwave script's constants of the same name for a --fridge run.
"""
from pathlib import Path

import numpy as np

HANDLE_DETECTOR = "depth"         # "depth": the microwave's depth detector over the whole image; "sam3": SAM 3 mask +
                                  # depth (scripts/detect_handle_sam3.py, needs `transformers`). See fridge_detect.py

# --- handle position gates: the microwave's (y >= -0.50, z <= 0.65) would refuse the fridge handles of 09-20/10-21
PLAUSIBLE_Y = (-0.60, 0.60)
PLAUSIBLE_Z = (0.25, 0.75)

# --- reach: the fridge branch's 09-20 grasps were 0.87-0.92 m out (microwave limits 0.915 grasp / 0.90 swing)
MAX_REACH = 0.94
SWING_MAX_REACH = 0.94

# --- grip: a normal close (the fridge branch's CloseGripperCommand = SheppyArm's default force). The microwave's 0.25
# is loosened on purpose for the microwave (lets the bar turn instead of tipping it)
GRASP_FORCE = 0.5

# --- hinge: no detected hinge; the fridge branch's measured offset from the closed-door grasp point, in the door's
# frame (into the door, along it toward the hinge, up). Its last 09-20 runs swung ~100 deg about this hinge.
# MEASURE all three on the fridge being tested (see FRIDGE_OPEN_TESTING.md).
HANDLE_CENTRE_TO_FACE_M = 0.053             # middle of the handle bar (where the fingers close) -> door's front surface,
                                            # perpendicular to the door, gap behind the bar included (measured 10-22;
                                            # was 0.020, estimated by eye)
PIN_BEHIND_FACE_M = 1.25 * 0.0254 + 0.015   # door's front surface -> hinge pin axis: door thickness 1.25 in (measured
                                            # 09-20, pin assumed at the back face) + 1.5 cm (hinge side crept forward).
                                            # If the pin depth is measured directly, use it alone.
HINGE_ALONG_DOOR_M = 14.5 * 0.0254          # handle bar centre line -> hinge pin axis, along the door at handle
                                            # height (measured 09-20)
HINGE_OFFSET_FROM_GRASP = np.array([HANDLE_CENTRE_TO_FACE_M + PIN_BEHIND_FACE_M, HINGE_ALONG_DOOR_M, 0.0])

# --- swing
SWING_DEG = 90                    # swing when no --swing-deg is given (--swing-deg N for any other angle). Exactly this
                                  # swing or the run refuses -- no smaller fallback
DOOR_FILE = Path.home() / ".fridge_door.json"   # hinge + closed-door grasp go here, never into the microwave's file

# --- release: let go, then back straight out of the handle. The microwave's 0.20 failed the sim checks from 4 of the
# 5 real 09-20 swing endings (J6 / IK); 0.12 passed from all 5 (and is what worked on the microwave on 10-21)
BACK_OFF_M = 0.12
LAST_GRASP_FILE = Path.home() / ".fridge_last_grasp.json"   # the release's grasp record, never the microwave's

# --- "sam3" option only: it sees no door, and its point is the handle's visible pixels, not the microwave detector's
SAM3_DOOR_NORMAL = np.array([-1.0, 0.0, 0.0])   # assumed: the door faces the arm head-on
SAM3_GRIP_EXT, SAM3_GRASP_X_ADJUST = 0.037, 0.0

OVERRIDES = dict(
    FRIDGE_HANDLE_DETECTOR=HANDLE_DETECTOR, PLAUSIBLE_Y=PLAUSIBLE_Y, PLAUSIBLE_Z=PLAUSIBLE_Z,
    MAX_REACH=MAX_REACH, SWING_MAX_REACH=SWING_MAX_REACH, GRASP_FORCE=GRASP_FORCE,
    FRIDGE_HINGE_OFFSET_FROM_GRASP=HINGE_OFFSET_FROM_GRASP, DOOR_SAVE_FILE=DOOR_FILE, FRIDGE_LAST_GRASP_FILE=LAST_GRASP_FILE,
    BACK_OFF_M=BACK_OFF_M,
)
if HANDLE_DETECTOR == "sam3":
    OVERRIDES.update(GRIP_EXT=SAM3_GRIP_EXT, GRASP_X_ADJUST=SAM3_GRASP_X_ADJUST)
