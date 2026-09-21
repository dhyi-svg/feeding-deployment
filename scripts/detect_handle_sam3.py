"""Handle detection = SAM 3 (RGB, "which pixels") + RealSense depth ("how far").

Importable module for the fridge motion scripts, and a CLI for a one-off look.

The split is deliberate and is the whole design:

  SAM 3      text-prompted ("handle") instance segmentation on the COLOUR image.
             Open-vocabulary and colour-agnostic -- it learned what a handle IS,
             not what colour this one happens to be -- so a future black handle
             on a black door needs no retuning. Outputs a pixel mask. Knows
             nothing about distance.
  RealSense  aligned depth at exactly those mask pixels. Knows nothing about
             handles. On this rig depth is good (1.8 mm plane residual measured
             2026-09-20); the earlier failures were never depth QUALITY.

What this replaces, and why (2026-09-20):

  * YOLO appliance box -> AppliancePerception.detect_handle_and_placement
    (detect_appliance_handle.py). Two independent failures on the fridge:
    (1) at grasp range the door fills the frame with no visible edges, so YOLO
    cannot recognise an appliance at all and the pipeline never starts;
    (2) when it does start, the handle sits only ~11 mm off the door plane --
    INSIDE RANSAC's 20 mm inlier band -- so the plane fit absorbs the handle
    into the door and the protrusion test never proposes it. Not fixable by
    denoising: clean depth that says "flush" is still "flush".
  * HSV colour threshold (detect_handle_classical.py, deleted). Worked on the
    white handle, but hardcoded white -- dead on arrival for a black handle.

Neither the appliance nor the door is detected any more. Only the handle.

Depth -> arm frame: every mask pixel with valid depth is deprojected with the
camera intrinsics (vectorised, not the per-pixel loop the old path used), the
MEDIAN 3D point is taken (robust to the few bad-depth pixels at the mask edge),
and that point is moved into arm_base_link with the same TFInterface lookup
the rest of the repo uses. Orientation is the same fixed GRASP_QUAT the old
path stamped on every handle -- vision never derived orientation there either.

Weights: facebook/sam3 is gated, so the default repo is a complete ungated
mirror. Safetensors only; the bundled .pt is an untrusted pickle.

    python3 scripts/detect_handle_sam3.py                 # detect once, save overlay
    python3 scripts/detect_handle_sam3.py --prompt "door handle" --max-detects 3
"""
import argparse
import sys
import os
import time
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from pybullet_helpers.geometry import Pose

from feeding_deployment.perception.tf_interface import TFInterface
from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface

# The repo's detect_handle_and_placement stamps every handle with this fixed
# quaternion; the motion scripts apply their own rig-specific fix on top.
GRASP_QUAT = (-0.5, 0.5, 0.5, -0.5)

DEFAULT_PROMPT = "handle"
DEFAULT_REPO = "MTerryJack/sam3"    # ungated mirror of facebook/sam3, identical files
DETECT_AGREE = 0.03                 # two looks must agree within 3 cm
MIN_MASK_PX = 300                   # smaller than this is noise, not a handle
MIN_VALID_DEPTH_FRAC = 0.30         # refuse a mask whose depth is mostly holes

# Calibrated depth correction along the CAMERA'S OPTICAL AXIS, applied to the
# handle point in the camera frame before it is transformed into arm_base_link.
# This is the same idea as AppliancePerception's HANDLE_DEPTH_CORR (the Jetson
# rig needed +0.094 m of it) -- a hand-eye calibration's translation along the
# look direction is its least-constrained component when the calibration board
# was always seen face-on, and it shows up as a constant range-independent
# error along that one axis.
#
# Measured 2026-09-20 on rchi-cpu-5 against a touched ground truth (user
# hand-placed the open gripper around the fridge handle; EE snapshot compared
# to the detection): truth - detected = dx +7.1, dy +0.3, dz +0.0 cm. Of that,
# 2.0 cm is the handle's half-depth (GRIP_EXT covers it); the remaining
# +0.051 m is this correction. Detection itself was repeatable to 0.2 cm and
# identical from 13 cm and 19 cm away, so it is a fixed offset, not noise.
# Re-measure after any recalibration or camera remount.
HANDLE_DEPTH_CORR_M = float(os.environ.get("SAM3_HANDLE_DEPTH_CORR", "0.051"))

# Loose absolute sanity bounds in arm_base_link. Z upper bound matches the
# motion scripts' own MAX_Z gate (0.75) -- the old 0.65 ceiling was tighter
# than the safety gate it sat in front of and blocked real fridge detections
# at 0.66-0.68 m for no safety reason.
PLAUSIBLE_X = (0.30, 0.90)
PLAUSIBLE_Y = (-0.60, 0.60)   # widened 2026-09-20: fridge legitimately sat at y=-0.46 after being nudged by grasps
PLAUSIBLE_Z = (0.25, 0.75)


class Sam3HandleDetector:
    """Wraps Sam3Model for one-shot text-prompted segmentation of a frame."""

    def __init__(self, prompt=DEFAULT_PROMPT, repo=DEFAULT_REPO, dtype="bf16",
                 thresh=0.5):
        from transformers import Sam3Model, Sam3Processor

        self.prompt = prompt
        self.thresh = thresh
        self.dev = "cuda" if torch.cuda.is_available() else "cpu"
        want = {"fp32": torch.float32, "fp16": torch.float16, "bf16": torch.bfloat16}[dtype]
        # Half precision is a tensor-core win on the GPU and a slowdown on CPU.
        self.dtype = want if self.dev == "cuda" else torch.float32
        t0 = time.time()
        self.model = Sam3Model.from_pretrained(
            repo, use_safetensors=True, dtype=self.dtype).eval().to(self.dev)
        self.processor = Sam3Processor.from_pretrained(repo)
        print(f"SAM 3 loaded from {repo} in {time.time()-t0:.1f}s on "
              f"{self.dev.upper()} ({'fp32' if self.dtype is torch.float32 else dtype})")

    def segment(self, rgb):
        """Best handle instance in an RGB frame.

        Returns (mask[H,W] bool, score, box_xyxy) or None if nothing passes
        thresh. Highest score wins; SAM 3 sometimes also returns a nested
        sub-part (the grip channel), which always scores lower.
        """
        image = Image.fromarray(np.ascontiguousarray(rgb))
        inputs = self.processor(images=image, text=self.prompt,
                                return_tensors="pt").to(self.dev)
        if self.dtype is not torch.float32:
            inputs["pixel_values"] = inputs["pixel_values"].to(self.dtype)
        with torch.no_grad():
            out = self.model(**inputs)
        res = self.processor.post_process_instance_segmentation(
            out, threshold=self.thresh, mask_threshold=0.5,
            target_sizes=inputs.get("original_sizes").tolist())[0]
        if len(res["masks"]) == 0:
            return None
        scores = [float(s) for s in res["scores"]]
        i = int(np.argmax(scores))
        # post-processed masks/boxes come back on the model's device
        m = res["masks"][i].detach().cpu().numpy().astype(bool)
        if m.ndim == 3:
            m = m[0]
        box = [int(v) for v in res["boxes"][i].detach().float().cpu().numpy().ravel()[:4]]
        return m, scores[i], box


def mask_to_camera_xyz(mask, depth_mm, camera_info):
    """Median 3D point (camera frame, metres) over the mask's valid-depth pixels.

    Vectorised pinhole deprojection with the same K the old pixel2World used.
    Returns (xyz, n_valid, n_mask). Raises RuntimeError if too little of the
    mask has depth -- a glossy-surface dropout is a real failure, not a point.
    """
    ys, xs = np.where(mask)
    d = depth_mm[ys, xs] / 1000.0
    ok = np.isfinite(d) & (d > 0.05) & (d < 2.0)
    n_valid = int(ok.sum())
    if n_valid == 0 or n_valid < MIN_VALID_DEPTH_FRAC * len(d):
        raise RuntimeError(
            f"only {n_valid}/{len(d)} handle pixels have valid depth -- refusing.")
    fx, fy, cx, cy = camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]
    z = d[ok]
    x = (xs[ok] - cx) * z / fx
    y = (ys[ok] - cy) * z / fy
    return np.median(np.stack([x, y, z], axis=1), axis=0), n_valid, len(d)


def build_detector(prompt=DEFAULT_PROMPT, repo=DEFAULT_REPO, dtype="bf16",
                   log_dir=None):
    """Camera + TF + SAM 3. No arm connection. Returns (det, rs, tf, log_dir).

    Raises RuntimeError if the camera never produces frames. TF failures are
    reported per-detection (the tree may still be filling at startup).
    """
    rs = RealSenseROS2Interface()
    if not rs.wait_for_frames(30.0):
        raise RuntimeError("No RGB-D frames -- is realsense2_camera up with "
                           "align_depth.enable:=true?")
    tf = TFInterface()
    det = Sam3HandleDetector(prompt=prompt, repo=repo, dtype=dtype)
    if log_dir:
        log_dir = Path(log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        print(f"overlays -> {log_dir}")
    return det, rs, tf, log_dir


def detect_once(det, rs, tf, tag="", log_dir=None):
    """One look. Returns (Pose in arm_base_link, xyz, info dict).

    Raises RuntimeError at whichever stage fails: no mask, bad depth, no TF.
    """
    d = rs.get_camera_data()
    bgr, depth, cam = d["rgb_image"], d["depth_image"], d["camera_info"]
    rgb = np.ascontiguousarray(bgr[:, :, ::-1])

    t0 = time.time()
    seg = det.segment(rgb)
    if seg is None:
        raise RuntimeError(f"NO HANDLE ({tag}): SAM 3 found nothing for {det.prompt!r}")
    mask, score, box = seg
    if mask.sum() < MIN_MASK_PX:
        raise RuntimeError(f"NO HANDLE ({tag}): mask only {int(mask.sum())} px")

    cam_xyz, n_valid, n_mask = mask_to_camera_xyz(mask, depth, cam)
    # calibration correction along the optical axis (see HANDLE_DEPTH_CORR_M)
    cam_xyz = cam_xyz + np.array([0.0, 0.0, HANDLE_DEPTH_CORR_M])

    transform = tf.get_frame_to_frame_transform(cam)
    if transform is None:
        raise RuntimeError("No arm_base_link<-camera transform (is calibration_tf up?)")
    base_xyz = (tf.make_homogeneous_transform(transform) @ np.array([*cam_xyz, 1.0]))[:3]

    print(f"  detect {tag}: score {score:.2f}  mask {n_mask} px ({n_valid} with depth)  "
          f"cam z {cam_xyz[2]:.3f} m  ->  base {np.round(base_xyz, 4)}  "
          f"[{time.time()-t0:.2f}s]")

    if log_dir is not None:
        vis = bgr.copy()
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours, -1, (0, 255, 0), 2)
        cv2.rectangle(vis, tuple(box[:2]), tuple(box[2:]), (0, 255, 0), 1)
        cv2.putText(vis, f"{score:.2f}  {cam_xyz[2]:.3f}m", (box[0], max(16, box[1] - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        cv2.imwrite(str(Path(log_dir) / f"detect_{tag or 'x'}.png"), vis)

    info = {"score": score, "box": box, "n_mask": n_mask, "n_valid": n_valid,
            "cam_xyz": cam_xyz}
    return Pose(tuple(base_xyz), GRASP_QUAT), np.asarray(base_xyz), info


def detect_until_agree(det, rs, tf, max_detects=5, agree_m=DETECT_AGREE, log_dir=None):
    """Keep looking until two independent looks agree within agree_m.

    Returns (Pose, mean_xyz). Raises RuntimeError if no two of max_detects
    agree. Same self-consistency gate the old detector used -- a single look
    that happens to be wrong must not drive the arm.
    """
    dets, pair = [], None
    for i in range(1, max_detects + 1):
        dets.append(detect_once(det, rs, tf, str(i), log_dir))
        for j in range(len(dets) - 1):
            spread = float(np.linalg.norm(dets[j][1] - dets[-1][1]))
            if spread <= agree_m:
                pair = (dets[j], dets[-1], spread)
                break
        if pair:
            break
        if i > 1:
            print(f"  no pair within {agree_m*100:.0f} cm yet after {i} looks; re-detecting")
    if pair is None:
        raise RuntimeError(
            f"No two of {len(dets)} detections agreed within {agree_m*100:.0f} cm -- refusing.")
    (_, ha, _), (pose_b, hb, _), spread = pair
    print(f"detections agree to {spread*100:.1f} cm (limit {agree_m*100:.0f}), "
          f"used {len(dets)} look(s)")
    h = (ha + hb) / 2.0
    return Pose(tuple(h), pose_b.orientation), h


def check_plausible(h):
    """Loose absolute sanity check in arm_base_link. Raises RuntimeError."""
    for nm, v, lo_hi in (("x", h[0], PLAUSIBLE_X), ("y", h[1], PLAUSIBLE_Y),
                         ("z", h[2], PLAUSIBLE_Z)):
        if not (lo_hi[0] <= v <= lo_hi[1]):
            raise RuntimeError(f"handle {nm}={v:.3f} outside plausible {lo_hi} -- refusing.")


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--prompt", default=DEFAULT_PROMPT)
    a.add_argument("--repo", default=DEFAULT_REPO)
    a.add_argument("--dtype", default="bf16", choices=["fp32", "fp16", "bf16"])
    a.add_argument("--max-detects", type=int, default=5)
    a.add_argument("--log-dir", default=None,
                   help="overlay PNGs go here (default /tmp/detect_handle_sam3/<timestamp>)")
    args = a.parse_args()
    log_dir = args.log_dir or f"/tmp/detect_handle_sam3/{time.strftime('%Y%m%d_%H%M%S')}"

    det, rs, tf, log_dir = build_detector(args.prompt, args.repo, args.dtype, log_dir)
    try:
        _, h = detect_until_agree(det, rs, tf, args.max_detects, log_dir=log_dir)
        check_plausible(h)
    except RuntimeError as e:
        raise SystemExit(str(e)) from None
    print(f"\nhandle (arm_base_link, mean of agreeing pair): {np.round(h, 4)}")
    print(f"overlays in {log_dir}")


def _clean_exit(code):
    """Exit without the interpreter's own teardown.

    The shared rclpy executor runs in a daemon thread; at interpreter exit that
    thread is killed mid-call inside rclpy's C++ and the process aborts with
    "terminate called without an active exception" / core dump -- AFTER all the
    work is done, so it is harmless but alarming. Shut the node down explicitly
    first, then os._exit so the C++ side is never torn down underneath it.
    """
    sys.stdout.flush(); sys.stderr.flush()
    try:
        from feeding_deployment.ros2.node import shutdown
        shutdown()
    except Exception:
        pass
    os._exit(code)


if __name__ == "__main__":
    _code = 0
    try:
        main()
    except SystemExit as e:  # sys.exit("message") is how every gate reports a refusal
        if isinstance(e.code, int) or e.code is None:
            _code = e.code or 0
        else:
            print(e.code, file=sys.stderr); _code = 1
    _clean_exit(_code)
