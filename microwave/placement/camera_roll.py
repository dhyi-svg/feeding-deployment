"""The wrist camera was remounted rotated 90 deg (CCW) about its optical axis after the
2026-09-07 hand-eye calibration. Two pieces handle that:

* `rotate_calibration` -- the easy_handeye2 calibration with an extra rotation about the
  camera's own optical (+z) axis, optionally about a pivot that is not the optical centre.
  Publishing that file with calibration_tf makes the whole TF chain (and every consumer --
  this task, the open task's detector, RViz) right, instead of patching points per script.
* `roll_scores` -- which extra roll makes the live scene's level surfaces level: for each
  candidate (0, +90, -90, 180) the frame's points are re-projected with that correction and
  the level, upward-facing area below the camera (counter, microwave floor) is measured. The
  right correction has a big level area; a wrong one has none (the counter turns into a wall,
  or ends up above the camera). This decides the sign of "CCW" from data, not from a convention.

Convention: roll_deg is a right-handed rotation about the optical +z (pointing out of the lens).
Seen from BEHIND the camera (looking where it looks) +90 turns the camera clockwise; seen from
the FRONT (facing the lens) it is counter-clockwise.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

import cavity_perception as cp
import point_cloud as pcm

CANDIDATES = (0.0, 90.0, -90.0, 180.0)


def roll_transform(roll_deg, pivot=(0.0, 0.0, 0.0)):
    """4x4 old_camera <- new_camera: the camera turned roll_deg about +z through `pivot`
    (a point in the old camera frame, m)."""
    Rz = R.from_euler("z", roll_deg, degrees=True).as_matrix()
    pv = np.asarray(pivot, float)
    T = np.eye(4)
    T[:3, :3] = Rz
    T[:3, 3] = pv - Rz @ pv
    return T


def rotate_calibration(calib_yaml, roll_deg, pivot=(0.0, 0.0, 0.0)):
    """easy_handeye2 calib (parsed YAML dict, effector -> camera) with the camera rolled. Returns a new dict."""
    t = calib_yaml["transform"]
    T = np.eye(4)
    T[:3, :3] = R.from_quat([t["rotation"][k] for k in "xyzw"]).as_matrix()
    T[:3, 3] = [t["translation"][k] for k in "xyz"]
    Tn = T @ roll_transform(roll_deg, pivot)
    qn = R.from_matrix(Tn[:3, :3]).as_quat()
    out = {"parameters": dict(calib_yaml["parameters"]),
           "transform": {"translation": dict(zip("xyz", map(float, Tn[:3, 3]))),
                         "rotation": dict(zip("xyzw", map(float, qn)))}}
    return out


def write_calibration(path, data, note):
    path = Path(path).expanduser()
    text = f"# {note}\n" + yaml.safe_dump(data, sort_keys=False, default_flow_style=None)
    path.write_text(text)
    return path


def roll_scores(depth_mm, K, base_T_cam, cav_cfg, cloud_cfg, candidates=CANDIDATES):
    """{roll_deg: (level area points, tilt of the biggest level-ish plane deg)} for each extra roll."""
    pts, _, _ = pcm.deproject(depth_mm, K, stride=3, depth_range=(cloud_cfg.depth_min_m, cloud_cfg.depth_max_m))
    out = {}
    for r in candidates:
        T = base_T_cam @ roll_transform(r)
        cloud = pcm.voxel_downsample(pcm.transform_points(T, pts), 0.01)
        sheets, _ = cp.horizontal_sheets(cloud, T[:3, 3], cav_cfg)
        level_up = [s for s in sheets if s.kind == "up" and np.median(s.points[:, 2]) < T[2, 3] - 0.05]
        out[r] = sum(len(s.points) for s in level_up)
    return out


def recommend(scores, margin=2.0):
    """The candidate with the most level area, if it beats the runner-up by `margin`x; else None."""
    ranked = sorted(scores.items(), key=lambda kv: -kv[1])
    best, second = ranked[0], ranked[1]
    if best[1] > 0 and best[1] >= margin * max(second[1], 1):
        return best[0]
    return None
