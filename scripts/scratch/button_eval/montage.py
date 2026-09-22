"""Coarse montage of eval frames (with a grid in ORIGINAL work-frame
coordinates) used to locate the button panel roughly before zooming in on
each frame to label it precisely."""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


def main(prefix, cols=4, cell_w=470):
    stems = sorted(p.stem for p in (ROOT / "frames").glob(f"{prefix}*.png"))
    if not stems:
        raise SystemExit(f"no frames matching {prefix}")
    tiles = []
    for s in stems:
        img = cv2.imread(str(ROOT / "frames" / f"{s}.png"))
        h, w = img.shape[:2]
        f = cell_w / float(w)
        small = cv2.resize(img, (cell_w, int(round(h * f))))
        for x in range(0, w, 100):
            cv2.line(small, (int(x * f), 0), (int(x * f), small.shape[0]), (0, 255, 255), 1)
            cv2.putText(small, str(x), (int(x * f) + 2, 12), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (0, 255, 255), 1, cv2.LINE_AA)
        for y in range(0, h, 100):
            cv2.line(small, (0, int(y * f)), (small.shape[1], int(y * f)), (0, 255, 255), 1)
            cv2.putText(small, str(y), (2, int(y * f) - 3), cv2.FONT_HERSHEY_SIMPLEX,
                        0.35, (0, 255, 255), 1, cv2.LINE_AA)
        tag = s.split("_f")[-1]
        cv2.rectangle(small, (0, small.shape[0] - 22), (70, small.shape[0]), (0, 0, 0), -1)
        cv2.putText(small, f"f{tag}", (4, small.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 2, cv2.LINE_AA)
        tiles.append(small)

    ch = max(t.shape[0] for t in tiles)
    rows = []
    for i in range(0, len(tiles), cols):
        chunk = tiles[i:i + cols]
        padded = [cv2.copyMakeBorder(t, 0, ch - t.shape[0], 0, 0, cv2.BORDER_CONSTANT, value=(40, 40, 40)) for t in chunk]
        while len(padded) < cols:
            padded.append(np.full((ch, cell_w, 3), 40, np.uint8))
        rows.append(np.hstack(padded))
    out = str(ROOT / "labeling" / f"montage_{prefix.strip('_')[:12]}.png")
    cv2.imwrite(out, np.vstack(rows))
    print(out, "frames:", ", ".join(s.split("_f")[-1] for s in stems))


if __name__ == "__main__":
    main(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 4)
