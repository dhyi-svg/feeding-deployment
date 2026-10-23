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
# MEASURE all three on the fridge being tested (see FRIDGE_TESTING.md).
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

# --- door model: the open writes these into DOOR_FILE, and every fridge plan near the door (push-open, close) checks
# clearance against a box door + cabinet built from them (microwave_common.add_door_model; the microwave's door file has
# none of these keys and keeps its own constants). MEASURE on the fridge being tested (see FRIDGE_TESTING.md).
DOOR_WIDTH_M = 0.45               # PLACEHOLDER (not measured): the door's full width, hinge-side edge to free edge. The
                                  # free edge = this past the hinge pin, i.e. DOOR_WIDTH_M - HINGE_ALONG_DOOR_M past the
                                  # bar (8 cm here; the 10-22 depth detection suggests ~4-6). Too small = the plans pass
                                  # closer to the real edge than they think; err big
DOOR_DEPTH_M = 1.25 * 0.0254      # PLACEHOLDER (not measured): the door's depth, front face to the innermost point of
                                  # its inside (bins / shelves included). Modelled as a slab this deep across the whole
                                  # door. Now just its thickness at the edge (1.25 in, 09-20): if the inside has bins,
                                  # the real door reaches much further into the gap the push-open goes into
DOOR_ABOVE_HANDLE_M = 0.50        # door top / bottom above / below the middle of the handle bar. Generous (not measured):
DOOR_BELOW_HANDLE_M = 0.50        #   the fridge's door is taller than the arm reaches around, and too tall is only safer
BODY_DEPTH_M = 0.50               # cabinet behind the door
DOOR_MODEL = dict(face_behind_grasp=HANDLE_CENTRE_TO_FACE_M, door_t=DOOR_DEPTH_M,
                  door_past_handle=DOOR_WIDTH_M - HINGE_ALONG_DOOR_M, body_depth=BODY_DEPTH_M)

# --- --push-open (fridge_push.push_open): the microwave's open -- pull a little holding the handle, let go, replay the
# user's push-open demo relative to the door -- with the demo's up-and-over-the-door start replaced by a route round
# the door's free edge at hand height (a fridge door is too tall to go over)
PUSH_OPEN_PULL_DEG = 35           # the pull (--swing-deg N for another). Not the microwave's 50: in an offline sim check
                                  # (10-22 detection, door width/depth guessed) 35 came closest to clearing the door going
                                  # behind it -- smaller leaves the gap too narrow for the fingers, larger turns the door
                                  # into the wrist's path. None of 20-50 cleared the 2 cm yet
PUSH_OPEN_OFF_HANDLE_M = BACK_OFF_M   # let go, then straight out off the handle this far (the release's 12 cm) ...
PUSH_OPEN_EDGE_CLEAR_M = 0.16     # ... along the open door to this far past its free edge (tool point), round it ...
PUSH_OPEN_DEMO = Path(__file__).resolve().parent / "demos" / "push_open_2026-10-07.door.json"   # ... then the
PUSH_OPEN_DEMO_FROM = 4           # microwave demo from this keypoint on: "4" (into the gap behind the door) and "5"
                                  # (the push). 1-3 are its up, over, down-beside-the-microwave

# --- close (`real_gen3_ros2_close_microwave.py --phase push-close --fridge`, fridge_push.close): the microwave's
# push-close swing, but reached by backing out in front of the open door's free edge and going left behind it (no
# going over the door), from further back, and only until the door is at CLOSE_TO_DEG (it swings shut by itself)
CHAIR_BACK_M = 0.30               # the user moves the wheelchair back this far first (prompted) ...
CHAIR_BACK_DIR = (1.0, 0.0)       # ... which moves the recorded fridge this way in the arm's frame (arm-base x = the
                                  # chair's forward). The plan uses the door from the open, shifted by this
CLOSE_TO_DEG = 45                 # swing the door shut until it is open this far, then back out (no torque push)
CLOSE_ARG_DEFAULTS = dict(        # replace the microwave close's argparse defaults for --fridge (any can still be given)
    push_radius=0.30, push_radius_end=0.28,     # the hand's distance from the hinge during the swing (handle at ~37 cm)
    push_stop_short=0.0,          # the swing ends AT the CLOSE_TO_DEG door model ...
    push_max=0.0,                 # ... and there is no final push
    keep_orientation=True)        # holding a container (as the microwave container close)

# --- "sam3" option only: it sees no door, and its point is the handle's visible pixels, not the microwave detector's
SAM3_DOOR_NORMAL = np.array([-1.0, 0.0, 0.0])   # assumed: the door faces the arm head-on
SAM3_GRIP_EXT, SAM3_GRASP_X_ADJUST = 0.037, 0.0

OVERRIDES = dict(
    FRIDGE_HANDLE_DETECTOR=HANDLE_DETECTOR, PLAUSIBLE_Y=PLAUSIBLE_Y, PLAUSIBLE_Z=PLAUSIBLE_Z,
    MAX_REACH=MAX_REACH, SWING_MAX_REACH=SWING_MAX_REACH, GRASP_FORCE=GRASP_FORCE,
    FRIDGE_HINGE_OFFSET_FROM_GRASP=HINGE_OFFSET_FROM_GRASP, DOOR_SAVE_FILE=DOOR_FILE, FRIDGE_LAST_GRASP_FILE=LAST_GRASP_FILE,
    BACK_OFF_M=BACK_OFF_M, FRIDGE_DOOR_MODEL=DOOR_MODEL, FRIDGE_DOOR_Z_FROM_HANDLE=(-DOOR_BELOW_HANDLE_M, DOOR_ABOVE_HANDLE_M),
)
if HANDLE_DETECTOR == "sam3":
    OVERRIDES.update(GRIP_EXT=SAM3_GRIP_EXT, GRASP_X_ADJUST=SAM3_GRASP_X_ADJUST)
