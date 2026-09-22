"""Crop a labeling view from the ORIGINAL video frame at native resolution.

The work frames are downscaled to a 960px long edge, which is fine for
running detectors but throws away the detail needed to hand-place a label on
a distant panel (~25px wide at work resolution, ~100px in the 4K source).
Labeling from the source and converting back keeps ground truth trustworthy
on exactly the frames that are hardest -- and those are the frames that
decide whether a detector is allowed to claim a pick at all.

Usage:
    source_zoom.py <stem> <work_cx> <work_cy> [half_extent_work_px]

Coordinates in and out are WORK-FRAME pixels; the source crop is an
implementation detail. Grid labels are drawn in work-frame coordinates so a
center read off this view can be written straight into labels.json.
"""
import json
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
PHOTOS = ROOT.parent / "button_photos"


def main(stem, wcx, wcy, half=40):
    manifest = {m["id"]: m for m in json.loads((ROOT / "manifest.json").read_text())}
    if stem not in manifest:
        raise SystemExit(f"{stem} not in manifest")
    m = manifest[stem]
    cap = cv2.VideoCapture(str(PHOTOS / m["clip"]))
    cap.set(cv2.CAP_PROP_POS_FRAMES, m["frame"])
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise SystemExit("read failed")

    s = m["scale"]               # source -> work
    inv = 1.0 / s                # work -> source
    sx0 = int((wcx - half) * inv)
    sy0 = int((wcy - half) * inv)
    sx1 = int((wcx + half) * inv)
    sy1 = int((wcy + half) * inv)
    H, W = frame.shape[:2]
    sx0, sy0 = max(0, sx0), max(0, sy0)
    sx1, sy1 = min(W, sx1), min(H, sy1)
    crop = frame[sy0:sy1, sx0:sx1]
    if crop.size == 0:
        raise SystemExit("empty crop")

    factor = max(1, int(900 / max(crop.shape[:2])))
    big = cv2.resize(crop, None, fx=factor, fy=factor, interpolation=cv2.INTER_CUBIC)

    # Grid step scales with the window: a fixed 5px step over a 300px-wide
    # window draws 60 lines and buries the image it is meant to help read.
    step_w = 5 if half <= 40 else (10 if half <= 80 else 25)
    w0 = int((wcx - half) // step_w * step_w)
    w1 = int(wcx + half)
    for wx in range(w0, w1 + step_w, step_w):
        px = int((wx * inv - sx0) * factor)
        if 0 <= px < big.shape[1]:
            cv2.line(big, (px, 0), (px, big.shape[0]), (0, 255, 255), 1)
            cv2.putText(big, str(wx), (px + 2, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (0, 255, 255), 1, cv2.LINE_AA)
    h0 = int((wcy - half) // step_w * step_w)
    h1 = int(wcy + half)
    for wy in range(h0, h1 + step_w, step_w):
        py = int((wy * inv - sy0) * factor)
        if 0 <= py < big.shape[0]:
            cv2.line(big, (0, py), (big.shape[1], py), (0, 255, 255), 1)
            cv2.putText(big, str(wy), (2, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        (0, 255, 255), 1, cv2.LINE_AA)

    tag = f"f{stem.split('_f')[-1]}"
    cv2.rectangle(big, (0, big.shape[0] - 26), (150, big.shape[0]), (0, 0, 0), -1)
    cv2.putText(big, f"{tag} SRC", (5, big.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA)
    out = ROOT / "labeling" / f"{stem}_srczoom.png"
    cv2.imwrite(str(out), big)
    print(f"{out}  src_crop=({sx0},{sy0})-({sx1},{sy1}) x{factor} "
          f"work_window=({wcx-half},{wcy-half})-({wcx+half},{wcy+half})")


if __name__ == "__main__":
    a = sys.argv[1:]
    main(a[0], float(a[1]), float(a[2]), float(a[3]) if len(a) > 3 else 40)
