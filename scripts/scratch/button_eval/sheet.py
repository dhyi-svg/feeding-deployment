"""Near-full-resolution grid sheet for a named list of eval frames, so
several can be hand-labeled from a single look without losing the pixel
precision the small-button frames need."""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


def main(out_name, stems, cols=2, step=50):
    tiles = []
    for s in stems:
        img = cv2.imread(str(ROOT / "frames" / f"{s}.png"))
        if img is None:
            print("missing", s)
            continue
        vis = img.copy()
        h, w = vis.shape[:2]
        for x in range(0, w, step):
            hv = 2 if x % 100 == 0 else 1
            cv2.line(vis, (x, 0), (x, h), (0, 255, 255), hv)
            if x % 50 == 0:
                cv2.putText(vis, str(x), (x + 2, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
        for y in range(0, h, step):
            hv = 2 if y % 100 == 0 else 1
            cv2.line(vis, (0, y), (w, y), (0, 255, 255), hv)
            if y % 50 == 0:
                cv2.putText(vis, str(y), (2, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)
        tag = f"f{s.split('_f')[-1]}"
        cv2.rectangle(vis, (0, h - 26), (110, h), (0, 0, 0), -1)
        cv2.putText(vis, tag, (5, h - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        tiles.append(vis)
    if not tiles:
        raise SystemExit("no tiles")
    ch, cw = max(t.shape[0] for t in tiles), max(t.shape[1] for t in tiles)
    rows = []
    for i in range(0, len(tiles), cols):
        chunk = [cv2.copyMakeBorder(t, 0, ch - t.shape[0], 0, cw - t.shape[1] + 5,
                                    cv2.BORDER_CONSTANT, value=(40, 40, 40)) for t in tiles[i:i + cols]]
        while len(chunk) < cols:
            chunk.append(np.full((ch, cw + 5, 3), 40, np.uint8))
        rows.append(np.hstack(chunk))
    out = ROOT / "labeling" / f"{out_name}.png"
    cv2.imwrite(str(out), np.vstack(rows))
    print(out)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
