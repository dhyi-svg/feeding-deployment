"""Run cv2.calibrateHandEye() on a captured samples file (see capture_calib_sample.py)
and save the resulting R_cam2gripper/t_cam2gripper, with diagnostics: rotation
diversity between samples, and comparison of the resulting t_cam2gripper norm against
the user's physically-measured wrist-to-camera distance.
"""

import argparse
import itertools
import json

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--method", default="TSAI", choices=["TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"])
    args = parser.parse_args()

    with open(args.input) as f:
        d = json.load(f)

    R_gripper2base = [np.array(r) for r in d["R_gripper2base"]]
    t_gripper2base = [np.array(t) for t in d["t_gripper2base"]]
    R_target2cam = [np.array(r) for r in d["R_target2cam"]]
    t_target2cam = [np.array(t) for t in d["t_target2cam"]]
    n = len(R_gripper2base)
    print(f"[hand-eye] {n} samples loaded from {args.input}")

    # Rotation diversity: pairwise relative-rotation angles between gripper poses.
    angles = []
    for i, j in itertools.combinations(range(n), 2):
        rel = R_gripper2base[i].T @ R_gripper2base[j]
        angle = np.degrees(np.arccos(np.clip((np.trace(rel) - 1) / 2, -1, 1)))
        angles.append(angle)
    print(f"[hand-eye] pairwise rotation diversity: max={max(angles):.1f}deg, mean={sum(angles)/len(angles):.1f}deg")

    # Translation diversity (recap).
    dists = [np.linalg.norm(t_gripper2base[i] - t_gripper2base[j]) for i, j in itertools.combinations(range(n), 2)]
    print(f"[hand-eye] pairwise translation diversity: max={max(dists)*100:.1f}cm, mean={sum(dists)/len(dists)*100:.1f}cm")

    method_map = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
        "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
        "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
    }
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=method_map[args.method]
    )
    t_cam2gripper = t_cam2gripper.flatten()
    quat = Rotation.from_matrix(R_cam2gripper).as_quat()

    norm = np.linalg.norm(t_cam2gripper)
    print(f"\n[hand-eye] method={args.method}")
    print(f"[hand-eye] R_cam2gripper:\n{R_cam2gripper}")
    print(f"[hand-eye] t_cam2gripper: {t_cam2gripper} (norm={norm*100:.1f}cm)")
    print(f"[hand-eye] user's physically-measured wrist-to-camera distance was ~5cm -- "
          f"{'CONSISTENT' if abs(norm*100 - 5) < 3 else 'STILL OFF' if abs(norm*100-5) >= 3 else ''}")

    # Self-consistency check: for each sample, compute base->camera and see spread
    # (mirrors the old calibration's "consistency_std_m" diagnostic).
    t_base2cam_all = []
    for i in range(n):
        R_g2b = R_gripper2base[i]
        t_g2b = t_gripper2base[i]
        t_b2c = R_g2b @ t_cam2gripper + t_g2b
        t_base2cam_all.append(t_b2c)
    t_base2cam_all = np.array(t_base2cam_all)
    std = t_base2cam_all.std(axis=0)
    spread = t_base2cam_all.max(axis=0) - t_base2cam_all.min(axis=0)
    print(f"[hand-eye] self-consistency (base->camera translation across samples): std={std}, spread={spread}")

    out = {
        "R_cam2gripper": R_cam2gripper.tolist(),
        "t_cam2gripper": t_cam2gripper.tolist(),
        "quat_xyzw": quat.tolist(),
        "n_samples": n,
        "min_markers_filter": 6,
        "reference_marker_id": d["reference_marker_id"],
        "assumed_marker_len_m": d["assumed_marker_len_m"],
        "method": f"cv2.CALIB_HAND_EYE_{args.method}",
        "consistency_std_m": std.tolist(),
        "consistency_max_spread_m": spread.tolist(),
        "pose_source": d.get("pose_source"),
        "rotation_diversity_deg": {"max": max(angles), "mean": sum(angles) / len(angles)},
        "translation_diversity_m": {"max": max(dists), "mean": sum(dists) / len(dists)},
    }
    with open(args.output, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\n[hand-eye] saved to {args.output}")


if __name__ == "__main__":
    main()
