"""Replay the REAL 3D pipeline on saved captures, with no model load. Seconds, not minutes.

Runs the genuine ``AppliancePerception.detect_handle_and_placement`` -- real plane fit,
real hinge selection, real corrections, real transforms -- with only ``detect_items``
stubbed, replaying the bounding box saved in the capture's ``detection_mask.png``. So it
exercises the code that was actually edited, unlike an inline reimplementation.

Use it after ANY change to the geometry in appliance_perception.py:

    python scripts/session/replay_geometry.py ~/captures/*/
    python scripts/session/replay_geometry.py --roll 90 ~/captures/grasp_exec

``--roll`` re-runs each capture with the wrist camera rotated about its optical axis, to
check the result does not depend on camera orientation. A rule keyed on camera-frame axes
changes its answer under roll; one keyed on the door plane does not.

Complements scripts/session/replay_detection.py, which loads GroundingDINO and replays
detection *including* the neural net. Use that when the question is "what did the detector
see"; use this when the question is "is the geometry right".
"""
import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from feeding_deployment.perception.appliance_perception.appliance_perception import (
    AppliancePerception,
)

TF_KEY = "arm_base_link__from__camera_color_optical_frame"


class _Stamp:
    secs = nsecs = sec = nanosec = 0


class _Header:
    stamp = _Stamp()
    frame_id = "camera_color_optical_frame"


class _CameraInfo:
    def __init__(self, d):
        self.header = _Header()
        self.width, self.height = d["width"], d["height"]
        self.distortion_model = d.get("distortion_model", "")
        self.K = self.k = d["K"]
        self.D = self.d = d["D"]
        self.R = self.r = d["R"]
        self.P = self.p = d["P"]
        self.fx, self.fy, self.cx, self.cy = d["K"][0], d["K"][4], d["K"][2], d["K"][5]


def load(sidecar: Path):
    stem = str(sidecar).replace("_detection_inputs.json", "")
    payload = json.loads(sidecar.read_text())
    rgb = cv2.imread(stem + "_rgb.png", cv2.IMREAD_COLOR)
    depth = cv2.imread(stem + "_depth.png", cv2.IMREAD_UNCHANGED)
    mask = cv2.imread(stem + "_detection_mask.png", cv2.IMREAD_UNCHANGED)
    if rgb is None or depth is None or mask is None:
        return None
    ys, xs = np.where(mask > 0)
    box = np.array([xs.min(), ys.min(), xs.max(), ys.max()], dtype=float)
    return payload, rgb, depth, box


def make_perception(matrix, box, corr_depth, corr_lat):
    """A real AppliancePerception with only the detector and tf lookup replaced."""
    apc = AppliancePerception.__new__(AppliancePerception)
    apc._last_images = {}
    apc._data_logger = None
    apc.handle_depth_corr = corr_depth
    apc.handle_lat_corr = corr_lat
    apc.points_publisher = None
    apc.center_publisher = None
    apc.detect_items = lambda *a, **k: box
    apc.get_frame_to_frame_transform = lambda *a, **k: "REPLAY"
    apc.make_homogeneous_transform = lambda _tr: matrix
    return apc


parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("captures", nargs="+", help="capture dirs written with DETECTION_LOG_DIR")
parser.add_argument("--handle-type", default="microwave handle")
parser.add_argument("--roll", type=float, nargs="*", default=[0.0],
                    help="simulated wrist-camera rolls in degrees (default: 0)")
parser.add_argument("--depth-corr", type=float, default=0.094)
parser.add_argument("--lat-corr", type=float, default=0.0)
args = parser.parse_args()

sidecars = sorted(
    Path(p) for c in args.captures
    for p in glob.glob(str(Path(c) / "**" / "*_detection_inputs.json"), recursive=True)
)
if not sidecars:
    raise SystemExit("No captures found. Run a task script with DETECTION_LOG_DIR set.")

print(f"{len(sidecars)} capture(s), rolls {args.roll} deg, "
      f"corrections depth={args.depth_corr} lat={args.lat_corr}\n")
print(f"{'capture':>24} {'roll':>5} {'handle (base)':>26} {'hinge (base)':>26} {'radius':>8}")

rows = []
for sc in sidecars:
    loaded = load(sc)
    if loaded is None:
        print(f"{sc.parent.name:>24}  SKIP (missing rgb/depth/mask)")
        continue
    payload, rgb, depth, box = loaded
    base_matrix = np.array(payload["transforms"][TF_KEY]["matrix"])
    name = f"{sc.parts[-5]}/{sc.name[:3]}"

    for roll in args.roll:
        m = base_matrix.copy()
        rot = R.from_euler("z", np.radians(roll)).as_matrix()
        m[:3, :3] = m[:3, :3] @ rot
        # Points are expressed in the rolled camera frame, so roll them too.
        depth_used = depth if roll == 0 else depth
        apc = make_perception(m, box, args.depth_corr, args.lat_corr)
        if roll != 0:
            inv = rot.T
            orig = apc.pixel2World

            def rolled(ci, u, v, d_img, depth=None, use_surrounding_pixels=False, _o=orig, _i=inv):
                ok, p = _o(ci, u, v, d_img, depth, use_surrounding_pixels)
                return (ok, _i @ np.asarray(p)) if ok else (ok, p)

            apc.pixel2World = rolled

        handle, hinge, _placement, _top = apc.detect_handle_and_placement(
            args.handle_type, rgb, _CameraInfo(payload["camera_info"]), depth_used)
        if handle is None:
            print(f"{name:>24} {roll:5.0f} {'REFUSED / no detection':>26}")
            continue
        h = np.asarray(handle.position)
        g = np.asarray(hinge.position)
        radius = float(np.linalg.norm((g - h)[:2]))
        rows.append((name, roll, h, g, radius))
        print(f"{name:>24} {roll:5.0f} {str(np.round(h,3)):>26} {str(np.round(g,3)):>26} "
              f"{radius*100:7.1f}cm")

if len(rows) > 1:
    print()
    hinges = np.array([r[3] for r in rows])
    radii = np.array([r[4] for r in rows])
    print(f"hinge spread across all rows : {np.ptp(hinges, axis=0)[:2].max()*100:.1f} cm")
    print(f"radius  min/max              : {radii.min()*100:.1f} / {radii.max()*100:.1f} cm")
    if len(args.roll) > 1:
        per = {}
        for name, roll, _h, g, _r in rows:
            per.setdefault(name, []).append(g)
        worst = max(np.ptp(np.array(v), axis=0)[:2].max() for v in per.values() if len(v) > 1)
        verdict = "ROLL-INDEPENDENT" if worst < 0.02 else "*** DEPENDS ON CAMERA ROLL ***"
        print(f"same capture across rolls    : {worst*100:.1f} cm  -> {verdict}")
