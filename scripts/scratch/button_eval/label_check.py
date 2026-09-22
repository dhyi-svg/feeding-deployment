"""Render each label ON TOP of a high-magnification view of its own source
frame, so a wrong label is visible as a marker sitting off the button rather
than having to be inferred from a coordinate readout.

Reading centers off a grid by eye turned out to be the single largest source
of error in this harness (4 wrong labels in the first ~20, two of which were
reported as detector failures). Drawing the label back onto the image at the
magnification it was placed at closes that loop: it removes the mental step of
converting between grid ticks and pixels, which is where the mistakes happened.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
PHOTOS = ROOT.parent / "button_photos"


def tile(stem, lab, man, box=46):
    cap = cv2.VideoCapture(str(PHOTOS / man["clip"]))
    cap.set(cv2.CAP_PROP_POS_FRAMES, man["frame"])
    ok, frame = cap.read()
    cap.release()
    if not ok:
        return None
    inv = 1.0 / man["scale"]
    cx, cy = lab["center"]
    half = max(box, int(lab.get("radius", 10) * 3.5))
    sx0, sy0 = int((cx - half) * inv), int((cy - half) * inv)
    sx1, sy1 = int((cx + half) * inv), int((cy + half) * inv)
    H, W = frame.shape[:2]
    sx0, sy0 = max(0, sx0), max(0, sy0)
    sx1, sy1 = min(W, sx1), min(H, sy1)
    crop = frame[sy0:sy1, sx0:sx1]
    if crop.size == 0:
        return None
    f = max(2, int(420 / max(crop.shape[:2])))
    big = cv2.resize(crop, None, fx=f, fy=f, interpolation=cv2.INTER_CUBIC)

    def to_px(p):
        return (int((p[0] * inv - sx0) * f), int((p[1] * inv - sy0) * f))

    r_px = int(lab.get("radius", 10) * inv * f)
    cv2.circle(big, to_px(lab["center"]), r_px, (0, 255, 0), 2)
    cv2.drawMarker(big, to_px(lab["center"]), (0, 255, 0), cv2.MARKER_CROSS, 26, 2)
    nb = lab.get("neighbor_xy")
    if nb:
        cv2.drawMarker(big, to_px(nb), (255, 0, 255), cv2.MARKER_SQUARE, 20, 2)
    tag = f"f{stem.split('_f')[-1]} {tuple(lab['center'])}"
    cv2.rectangle(big, (0, big.shape[0] - 22), (210, big.shape[0]), (0, 0, 0), -1)
    cv2.putText(big, tag, (4, big.shape[0] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return big


def main(split, cols=4):
    labels = json.loads((ROOT / "labels.json").read_text())["labels"]
    man = {m["id"]: m for m in json.loads((ROOT / "manifest.json").read_text())}
    tiles = []
    for stem, lab in sorted(labels.items()):
        if man.get(stem, {}).get("split") != split or "center" not in lab:
            continue
        t = tile(stem, lab, man[stem])
        if t is not None:
            tiles.append(t)
    ch, cw = max(t.shape[0] for t in tiles), max(t.shape[1] for t in tiles)
    rows = []
    for i in range(0, len(tiles), cols):
        ck = [cv2.copyMakeBorder(t, 0, ch - t.shape[0], 0, cw - t.shape[1] + 5,
                                 cv2.BORDER_CONSTANT, value=(40, 40, 40)) for t in tiles[i:i + cols]]
        while len(ck) < cols:
            ck.append(np.full((ch, cw + 5, 3), 40, np.uint8))
        rows.append(np.hstack(ck))
    out = ROOT / "labeling" / f"labelcheck_{split}.png"
    cv2.imwrite(str(out), np.vstack(rows))
    print(f"{out} ({len(tiles)} labels)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "fresh")
