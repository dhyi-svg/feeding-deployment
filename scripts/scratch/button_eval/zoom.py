"""Zoom helper for hand-labeling: crop a region of an eval frame and upscale
it with a fine coordinate grid whose tick labels are in ORIGINAL work-frame
pixels, so a button center read off the zoom can be recorded directly."""
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent


def zoom(stem, x0, y0, x1, y1, out=None, factor=None):
    img = cv2.imread(str(ROOT / "frames" / f"{stem}.png"))
    if img is None:
        raise SystemExit(f"no such frame: {stem}")
    h, w = img.shape[:2]
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)
    crop = img[y0:y1, x0:x1]
    if crop.size == 0:
        raise SystemExit("empty crop")
    if factor is None:
        factor = max(1, int(900 / max(1, max(crop.shape[:2]))))
    big = cv2.resize(crop, None, fx=factor, fy=factor, interpolation=cv2.INTER_NEAREST)

    step = 5 if factor >= 8 else 10
    for x in range(x0 - x0 % step, x1, step):
        px = (x - x0) * factor
        cv2.line(big, (px, 0), (px, big.shape[0]), (0, 255, 255), 1)
        cv2.putText(big, str(x), (px + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 255, 255), 1, cv2.LINE_AA)
    for y in range(y0 - y0 % step, y1, step):
        py = (y - y0) * factor
        cv2.line(big, (0, py), (big.shape[1], py), (0, 255, 255), 1)
        cv2.putText(big, str(y), (2, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    (0, 255, 255), 1, cv2.LINE_AA)

    out = out or str(ROOT / "labeling" / f"{stem}_zoom.png")
    cv2.imwrite(out, big)
    print(f"{out}  crop=({x0},{y0})-({x1},{y1}) factor={factor} size={big.shape[1]}x{big.shape[0]}")


if __name__ == "__main__":
    a = sys.argv[1:]
    zoom(a[0], int(a[1]), int(a[2]), int(a[3]), int(a[4]))
