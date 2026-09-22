"""Extract a fixed, reproducible set of evaluation frames from the button
footage, downscaled to a working resolution, plus grid-overlaid copies used
only for hand-labeling.

Frames are chosen to span the full range of apparent button scale in each
clip (wide establishing shots through panel close-ups), because scale is
where the current HoughCircles detector fails. The reference frames used to
build the homography detector are deliberately EXCLUDED so the eval set
never contains the image the detector was built from.
"""
import json
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
PHOTOS = ROOT.parent / "button_photos"

# Working resolution for evaluation. The 4K source is 2-8s/frame through the
# multi-scale Hough sweep; everything downstream runs on this instead. Long
# edge, aspect preserved.
WORK_LONG_EDGE = 960

COMFEE = "2C3103B0-5173-4F73-BFE2-3E059285BFF7.MP4"

# Reference window (sharp, whole panel visible) -- held out of the eval set.
REFERENCE_FRAMES = range(195, 216)

# Only the Comfee clip. The older IMG_38xx footage is a DIFFERENT microwave
# (red, different panel finish and button labelling) and is out of scope --
# this detector targets the appliance in the clip the work is actually for.
#
# Because there is no second appliance to hold out, the holdout is temporal
# instead: TUNE_FRAMES are the frames parameters may be fitted against, and
# HOLDOUT_FRAMES are never looked at while tuning. Weaker evidence than a
# different appliance would be (same lighting, same session, same operator),
# but it still catches parameters fitted to specific frames.
TUNE_FRAMES = [0, 80, 160, 240, 320, 400, 480, 560, 640, 700, 760, 820, 874]
HOLDOUT_FRAMES = [40, 120, 280, 360, 440, 520, 600, 660, 730, 790, 850]

# A SECOND, later holdout. The first one stopped being clean once it was used to
# diagnose a design decision (the affine-fallback experiment was analysed on its
# failures). These indices have never been extracted, looked at or scored, and
# are kept aside so there is still one set whose number means something.
# None of them fall within 15 frames of the reference window, so none is a
# near-duplicate of the image the detector was built from.
FRESH_FRAMES = [20, 100, 140, 260, 340, 420, 500, 580, 680, 805]

CLIPS = {COMFEE: TUNE_FRAMES + HOLDOUT_FRAMES + FRESH_FRAMES}


def work_size(w, h):
    scale = WORK_LONG_EDGE / float(max(w, h))
    return int(round(w * scale)), int(round(h * scale)), scale


def draw_grid(img, step=50):
    """Coordinate grid so a frame can be hand-labeled by reading off ticks."""
    vis = img.copy()
    h, w = vis.shape[:2]
    for x in range(0, w, step):
        cv2.line(vis, (x, 0), (x, h), (0, 255, 255), 1)
        cv2.putText(vis, str(x), (x + 2, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 255), 1, cv2.LINE_AA)
    for y in range(0, h, step):
        cv2.line(vis, (0, y), (w, y), (0, 255, 255), 1)
        cv2.putText(vis, str(y), (2, y - 3), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (0, 255, 255), 1, cv2.LINE_AA)
    return vis


def main():
    manifest = []
    for clip, idxs in CLIPS.items():
        path = PHOTOS / clip
        if not path.exists():
            print(f"SKIP missing {clip}")
            continue
        cap = cv2.VideoCapture(str(path))
        for i in idxs:
            if clip == COMFEE and i in REFERENCE_FRAMES:
                print(f"SKIP {clip} frame {i}: reference window")
                continue
            cap.set(cv2.CAP_PROP_POS_FRAMES, i)
            ok, frame = cap.read()
            if not ok:
                print(f"SKIP {clip} frame {i}: read failed")
                continue
            h, w = frame.shape[:2]
            nw, nh, scale = work_size(w, h)
            small = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_AREA)
            stem = f"{Path(clip).stem}_f{i:04d}"
            cv2.imwrite(str(ROOT / "frames" / f"{stem}.png"), small)
            cv2.imwrite(str(ROOT / "labeling" / f"{stem}_grid.png"), draw_grid(small))
            manifest.append({
                "id": stem, "clip": clip, "frame": i,
                "src_size": [w, h], "work_size": [nw, nh],
                "scale": scale,
                "split": ("tune" if i in TUNE_FRAMES
                          else "holdout" if i in HOLDOUT_FRAMES else "fresh"),
            })
        cap.release()
    (ROOT / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(f"Extracted {len(manifest)} eval frames -> {ROOT/'frames'}")


if __name__ == "__main__":
    main()
