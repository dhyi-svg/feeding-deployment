"""Generate several labeling zooms at once and tile them into a single image,
so a batch of eval frames can be hand-labeled from one look. Each tile keeps
its own coordinate grid in that frame's work-frame pixels."""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


def make_zoom(stem, x0, y0, x1, y1, target=620):
    img = cv2.imread(str(ROOT / "frames" / f"{stem}.png"))
    if img is None:
        return None
    h, w = img.shape[:2]
    x0, y0 = max(0, int(x0)), max(0, int(y0))
    x1, y1 = min(w, int(x1)), min(h, int(y1))
    crop = img[y0:y1, x0:x1]
    if crop.size == 0:
        return None
    factor = max(2, int(target / max(1, max(crop.shape[:2]))))
    big = cv2.resize(crop, None, fx=factor, fy=factor, interpolation=cv2.INTER_NEAREST)
    step = 10 if factor >= 5 else 20
    for x in range(x0 - x0 % step, x1, step):
        px = (x - x0) * factor
        cv2.line(big, (px, 0), (px, big.shape[0]), (0, 255, 255), 1)
        cv2.putText(big, str(x), (px + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 255), 1, cv2.LINE_AA)
    for y in range(y0 - y0 % step, y1, step):
        py = (y - y0) * factor
        cv2.line(big, (0, py), (big.shape[1], py), (0, 255, 255), 1)
        cv2.putText(big, str(y), (2, py - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 255), 1, cv2.LINE_AA)
    tag = stem.split("_f")[-1]
    cv2.rectangle(big, (0, big.shape[0] - 24), (90, big.shape[0]), (0, 0, 0), -1)
    cv2.putText(big, f"f{tag}", (4, big.shape[0] - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
    return big


def main(argv):
    """argv: out_name stem:x0,y0,x1,y1 [stem:x0,y0,x1,y1 ...]"""
    out_name = argv[0]
    tiles = []
    for spec in argv[1:]:
        stem, box = spec.split(":")
        x0, y0, x1, y1 = (int(v) for v in box.split(","))
        t = make_zoom(stem, x0, y0, x1, y1)
        if t is not None:
            tiles.append(t)
    if not tiles:
        raise SystemExit("no tiles")
    ch = max(t.shape[0] for t in tiles)
    padded = [cv2.copyMakeBorder(t, 0, ch - t.shape[0], 0, 6, cv2.BORDER_CONSTANT, value=(40, 40, 40)) for t in tiles]
    out = str(ROOT / "labeling" / f"{out_name}.png")
    cv2.imwrite(out, np.hstack(padded))
    print(out)


if __name__ == "__main__":
    main(sys.argv[1:])
