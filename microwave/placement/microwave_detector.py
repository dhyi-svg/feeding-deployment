"""Real-time microwave detector for the placement task (replaces SAM 3 there), plus the
stabiliser that turns a stream of per-frame boxes into one trusted region of interest.

Detector: YOLO26s, COCO 'microwave' (class 68) -- the same weights and fallback classes as the
open task's `handle_detect.py`, ~10-20 ms per frame on the RTX 4070 Ti (SAM 3 was ~1 s). It only
has to find the microwave: the interior (cavity walls, floor) is measured from depth by
`cavity_perception.py`, not segmented from colour. The image is rotated upright first (see
`point_cloud.upright_k`), because the camera is mounted rotated and COCO models are not
rotation invariant; boxes are returned in RAW image pixels.

`DetectionStabilizer` accepts a box only once `min_consistent` (>= 3) recent detections agree
with their median box (IoU and centre shift), rejecting the outliers; `wait_for_stable` gives up
after `timeout_s` -- a refusal, never a guess.
"""
from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

import numpy as np

from point_cloud import rot_box_to_raw, upright_k


@dataclass
class Detection:
    box: tuple          # (x1, y1, x2, y2) raw image pixels
    conf: float
    label: str
    stamp: float        # time.time() of the frame
    fallback: bool = False   # True = a confused class (bus/train/...), not 'microwave'
    k_upright: int = 0       # quarter turns applied before detection


class YoloMicrowaveDetector:
    def __init__(self, cfg):
        from ultralytics import YOLO  # heavy import, only where a detector is built
        self.cfg = cfg
        self.model = YOLO(cfg.model)
        self.names = self.model.names

    def _predict(self, img, classes):
        kw = {"classes": list(classes), "conf": self.cfg.conf, "verbose": False}
        if self.cfg.device:
            kw["device"] = self.cfg.device
        res = self.model.predict(img, **kw)[0]
        if res.boxes is None or len(res.boxes) == 0:
            return None
        best = int(res.boxes.conf.argmax())
        return (res.boxes.xyxy[best].cpu().numpy().astype(float), float(res.boxes.conf[best]),
                self.names[int(res.boxes.cls[best])])

    def detect(self, bgr, base_T_cam=None, stamp=None):
        """Best microwave box in one BGR frame, or None. `base_T_cam` (if given and
        cfg.upright_from_tf) rotates the image upright first."""
        k = upright_k(base_T_cam) if (base_T_cam is not None and self.cfg.upright_from_tf) else 0
        img = np.ascontiguousarray(np.rot90(bgr, k)) if k else bgr
        hit, fallback = self._predict(img, self.cfg.class_ids), False
        if hit is None and self.cfg.fallback_class_ids:
            hit, fallback = self._predict(img, self.cfg.fallback_class_ids), True
        if hit is None:
            return None
        box, conf, label = hit
        return Detection(rot_box_to_raw(box, k, bgr.shape), conf, label,
                         time.time() if stamp is None else stamp, fallback, k)


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _center(b):
    return np.array([(b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0])


class DetectionStabilizer:
    """Keeps the newest `window` detections; `stable()` returns (median box, inliers) once at
    least `min_consistent` of them agree with the median box, else (None, reason)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.dets = deque(maxlen=cfg.window)

    def reset(self):
        self.dets.clear()

    def add(self, det):
        if det is not None:
            self.dets.append(det)

    def stable(self):
        n = len(self.dets)
        if n < self.cfg.min_consistent:
            return None, f"{n}/{self.cfg.min_consistent} detections so far"
        boxes = np.array([d.box for d in self.dets])
        med = np.median(boxes, axis=0)
        ok = [d for d in self.dets
              if iou(d.box, med) >= self.cfg.min_iou
              and np.linalg.norm(_center(d.box) - _center(med)) <= self.cfg.max_center_shift_px]
        if len(ok) < self.cfg.min_consistent:
            return None, (f"only {len(ok)} of {n} detections agree with their median box "
                          f"(need {self.cfg.min_consistent}; IoU >= {self.cfg.min_iou})")
        box = tuple(np.median(np.array([d.box for d in ok]), axis=0).tolist())
        return box, ok


def wait_for_stable(get_detection, cfg, log=print, sleep=time.sleep, clock=time.time):
    """Call `get_detection()` (-> Detection or None) until the stabiliser agrees or cfg.timeout_s
    passes. Returns (box, inlier detections). Raises TimeoutError with the last reason."""
    stab = DetectionStabilizer(cfg)
    t0, reason, frames = clock(), "no frames", 0
    while clock() - t0 < cfg.timeout_s:
        det = get_detection()
        frames += 1
        stab.add(det)
        box, info = stab.stable()
        if box is not None:
            log(f"  detector: stable after {frames} frames ({len(info)} agree, "
                f"conf {min(d.conf for d in info):.2f}-{max(d.conf for d in info):.2f}"
                f"{', FALLBACK class ' + info[-1].label if any(d.fallback for d in info) else ''})")
            return box, info
        reason = info
        sleep(cfg.frame_period_s)
    raise TimeoutError(f"no stable microwave detection within {cfg.timeout_s:.0f} s "
                       f"({frames} frames): {reason}")
