"""Re-run the REAL handle detection offline, from captures, with no arm and no camera.

Why this exists: a live dry run costs ~85 s of inference on top of ~274 s of model
loading, needs the whole stack up, and -- with DETECTION_LOG_DIR unset -- throws away
every diagnostic image it built. So the only way to investigate a bad detection was to
reposition and re-run blind, at ~6 minutes a look.

This loads GroundingDINO **once** and replays N saved captures through the genuine
``AppliancePerception.detect_handle_and_placement``. Nothing is stubbed except the tf2
lookup, which is replaced by the exact 4x4 base<-camera matrix recorded at capture time
(``_log_detection_inputs`` saves it precisely so this is possible). Same detector, same
plane fit, same DBSCAN, same corrections.

What it buys you:
  * measure detection spread across viewpoints without touching the robot
  * iterate on HANDLE_DEPTH_CORR / prompts / thresholds against FIXED data, so a change
    in the answer means a change in the code -- not a different frame
  * the overlays land next to each capture, so you can SEE which pixels were clustered
    as "handle" instead of inferring it from a centroid coordinate

Capture first, by running the normal dry run with logging on:

    DETECTION_LOG_DIR=~/captures/pose1 $PY -u scripts/real_gen3_ros2_approach_microwave.py
    # reposition the arm, then repeat into pose2, pose3, ...

Then replay them all in one process:

    $PY -u scripts/session/replay_detection.py ~/captures/pose*

Handle poses are in arm_base_link, a WORLD-FIXED frame -- so captures from different
viewpoints must agree. Disagreement is the finding, not an artifact of moving the arm.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

from feeding_deployment.perception.appliance_perception.appliance_perception import (
    AppliancePerception,
)
from feeding_deployment.perception.grounded_sam import GroundedSAM

TF_KEY = "arm_base_link__from__camera_color_optical_frame"


class _CameraInfoShim:
    """Enough of a CameraInfo for the detection path, rebuilt from the JSON sidecar."""

    class _Stamp:
        def __init__(self, secs=0, nsecs=0):
            self.secs, self.nsecs = secs, nsecs
            self.sec, self.nanosec = secs, nsecs

    class _Header:
        def __init__(self, frame_id, stamp):
            self.frame_id, self.stamp = frame_id, stamp

    def __init__(self, d):
        stamp = d.get("stamp") or {}
        self.header = self._Header(
            d.get("frame_id"), self._Stamp(stamp.get("secs", 0), stamp.get("nsecs", 0))
        )
        self.width, self.height = d["width"], d["height"]
        self.distortion_model = d.get("distortion_model", "")
        self.K, self.D, self.R, self.P = d["K"], d["D"], d["R"], d["P"]
        self.k, self.d, self.r, self.p = d["K"], d["D"], d["R"], d["P"]
        self.fx, self.fy = d["K"][0], d["K"][4]
        self.cx, self.cy = d["K"][2], d["K"][5]


def find_capture(directory: Path):
    """Locate one capture's rgb / depth / inputs-json inside a DETECTION_LOG_DIR tree."""
    js = sorted(directory.rglob("*_detection_inputs.json"))
    if not js:
        return None
    sidecar = js[0]
    stem = sidecar.name.replace("_detection_inputs.json", "")
    rgb = next(iter(sorted(sidecar.parent.glob(f"{stem}_rgb.png"))), None)
    depth = next(iter(sorted(sidecar.parent.glob(f"{stem}_depth.png"))), None)
    if rgb is None or depth is None:
        return None
    return sidecar, rgb, depth


parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("captures", nargs="+", type=Path,
                    help="DETECTION_LOG_DIR directories written by a logged dry run")
parser.add_argument("--handle-type", default="microwave handle")
args = parser.parse_args()

found = []
for directory in args.captures:
    hit = find_capture(directory)
    if hit is None:
        print(f"  SKIP {directory}: no *_detection_inputs.json + rgb/depth found")
        continue
    found.append((directory, *hit))
if not found:
    sys.exit("No usable captures. Run a dry run with DETECTION_LOG_DIR set first.")

print(f"replaying {len(found)} capture(s) -- loading GroundingDINO once ...")
apc = AppliancePerception(GroundedSAM())
print(f"corrections: depth={apc.handle_depth_corr} lat={apc.handle_lat_corr}\n")

results = []
for directory, sidecar, rgb_path, depth_path in found:
    payload = json.loads(sidecar.read_text())
    tf = payload.get("transforms", {}).get(TF_KEY)
    if tf is None:
        print(f"  SKIP {directory.name}: sidecar has no {TF_KEY}")
        continue
    base_from_cam = np.array(tf["matrix"], dtype=float)

    rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
    # Saved as uint16 millimetres; pixel2World divides by 1000, so feed it back as-is.
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    info = _CameraInfoShim(payload["camera_info"])

    # Replace only the tf2 lookup: hand back the matrix recorded at capture time.
    # Overriding make_homogeneous_transform (rather than faking a TransformStamped)
    # keeps this robust to the message type differing between ROS 1 and ROS 2.
    apc.get_frame_to_frame_transform = lambda *a, **k: "REPLAY"
    apc.make_homogeneous_transform = lambda _tr: base_from_cam

    handle, hinge, placement, top = apc.detect_handle_and_placement(
        args.handle_type, rgb, info, depth
    )
    if handle is None:
        print(f"  {directory.name}: NO DETECTION")
        continue

    for name, image in apc._last_images.items():
        cv2.imwrite(str(directory / f"replay_{name}.png"), image)

    pos = np.asarray(handle.position)
    results.append((directory.name, pos))
    print(f"  {directory.name}: handle {np.round(pos, 4)}   "
          f"overlays -> {directory}/replay_*.png")

if len(results) < 2:
    sys.exit(0)

print("\nAgreement across viewpoints (arm_base_link is world-fixed -- these MUST match):")
positions = np.array([p for _, p in results])
centre = positions.mean(axis=0)
for name, pos in results:
    print(f"  {name:24s} {np.round(pos, 4)}   {np.linalg.norm(pos - centre) * 100:5.1f} cm from mean")
worst = max(
    float(np.linalg.norm(a - b))
    for i, a in enumerate(positions)
    for b in positions[i + 1:]
)
print(f"\nworst pairwise disagreement: {worst * 100:.1f} cm")
print("  < 3 cm  -- consistent; the grasp script's agreement gate would pass")
print("  > 3 cm  -- the grasp script would REFUSE. Compare the replay_handle_pixels.png")
print("             overlays to see whether the cluster moved to a different object.")
