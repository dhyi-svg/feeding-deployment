"""Which roll correction does the CURRENTLY published calibration need? Read-only, no motion.

    python3 microwave/placement/tools/check_camera_roll.py                 # live camera + tf
    python3 microwave/placement/tools/check_camera_roll.py --frame ~/microwave_place_logs/<run>/frame_1.npz

Point the wrist camera at the counter/microwave (some level surface below the camera in view).
For each extra roll (0, +90, -90, 180) it counts the level, upward-facing surface points the
frame would have. Prints the recommendation: 0 = the published calibration is right; anything
else = write that roll with rotate_camera_calibration.py and restart calibration_tf with it.
"""
import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from camera_roll import recommend, roll_scores  # noqa: E402
from placement_config import load_config  # noqa: E402

a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
a.add_argument("--frame", help="a frame_*.npz saved by the placement (default: live)")
args = a.parse_args()
cfg = load_config()
if args.frame:
    import placement_workflow as wf
    fr = wf.Frame.load(args.frame)
else:
    import placement_workflow as wf
    from feeding_deployment.perception.tf_interface import TFInterface
    from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface
    rs = RealSenseROS2Interface()
    if not rs.wait_for_frames(30.0):
        sys.exit("no RGB-D frames")
    fr = wf.RosFrameSource(rs, TFInterface(), 2.0).get()
scores = roll_scores(fr.depth_mm, fr.K, fr.base_T_cam, cfg.cavity, cfg.cloud)
for r, n in scores.items():
    print(f"  extra roll {r:+6.0f} deg: {n:6d} level upward-facing points below the camera")
rec = recommend(scores)
if rec is None:
    print("NO CLEAR ANSWER -- get more of the counter/floor in view and re-run")
elif rec == 0:
    print("OK: the published calibration is consistent with gravity")
else:
    print(f"RECOMMEND: rotate_camera_calibration.py --roll-deg {rec:+.0f}, then restart calibration_tf with it")
sys.stdout.flush()
if not args.frame:
    from feeding_deployment.ros2.node import shutdown
    shutdown()
os._exit(0)
