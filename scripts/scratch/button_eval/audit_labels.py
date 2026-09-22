"""Overlay every ground-truth label on its frame at high magnification, tiled
for review.

This exists because two labels were already found wrong by exactly this check:
one was placed on the STOP/ECO button (one button left of the target) and was
being scored as a 20px detector failure when the detector was correct. Labels
placed from a wide view are not trustworthy; a label is only trustworthy once
it has been seen sitting on the right button at a magnification where the
neighbouring button is clearly distinguishable.

Green circle = the +30SEC label. Magenta square = the STOP/ECO neighbour, which
is the thing a mislabel most often slides onto, so showing both makes an
off-by-one-button error obvious.

Usage: audit_labels.py [split]     (split: tune | holdout | all)
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent


def main(split="all", cols=4):
    labels = json.loads((ROOT / "labels.json").read_text())["labels"]
    manifest = {m["id"]: m for m in json.loads((ROOT / "manifest.json").read_text())}

    tiles = []
    for stem, lab in sorted(labels.items()):
        sp = manifest.get(stem, {}).get("split", "?")
        if split != "all" and sp != split:
            continue
        img = cv2.imread(str(ROOT / "frames" / f"{stem}.png"))
        if img is None or "center" not in lab:
            continue
        cx, cy = lab["center"]
        r = max(5, int(lab.get("radius", 8)))
        half = max(40, r * 5)
        x0, y0 = max(0, cx - half), max(0, cy - half)
        x1, y1 = min(img.shape[1], cx + half), min(img.shape[0], cy + half)
        crop = img[y0:y1, x0:x1]
        if crop.size == 0:
            continue
        f = max(2, int(430 / max(crop.shape[:2])))
        big = cv2.resize(crop, None, fx=f, fy=f, interpolation=cv2.INTER_NEAREST)
        cv2.circle(big, ((cx - x0) * f, (cy - y0) * f), r * f, (0, 255, 0), 2)
        cv2.drawMarker(big, ((cx - x0) * f, (cy - y0) * f), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)
        nb = lab.get("neighbor_xy")
        if nb and x0 <= nb[0] < x1 and y0 <= nb[1] < y1:
            cv2.drawMarker(big, ((nb[0] - x0) * f, (nb[1] - y0) * f), (255, 0, 255),
                           cv2.MARKER_SQUARE, 20, 2)
        tag = f"f{stem.split('_f')[-1]} [{sp[:1].upper()}] {lab.get('status','')[:4]}"
        cv2.rectangle(big, (0, big.shape[0] - 22), (230, big.shape[0]), (0, 0, 0), -1)
        cv2.putText(big, tag, (4, big.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (255, 255, 255), 1, cv2.LINE_AA)
        tiles.append(big)

    if not tiles:
        raise SystemExit(f"no labeled frames for split={split}")
    ch, cw = max(t.shape[0] for t in tiles), max(t.shape[1] for t in tiles)
    rows = []
    for i in range(0, len(tiles), cols):
        chunk = [cv2.copyMakeBorder(t, 0, ch - t.shape[0], 0, cw - t.shape[1] + 5,
                                    cv2.BORDER_CONSTANT, value=(40, 40, 40))
                 for t in tiles[i:i + cols]]
        while len(chunk) < cols:
            chunk.append(np.full((ch, cw + 5, 3), 40, np.uint8))
        rows.append(np.hstack(chunk))
    out = ROOT / "labeling" / f"audit_{split}.png"
    cv2.imwrite(str(out), np.vstack(rows))
    print(f"{out}  ({len(tiles)} labels)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "all")
