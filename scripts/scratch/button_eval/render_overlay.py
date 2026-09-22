"""Render an annotated copy of the microwave clip showing what the
reference-homography detector does on every frame.

Drawn per frame:
  - the projected panel quad (green) -- where the detector thinks the control
    panel is, i.e. the geometry the whole answer rests on
  - the +30SEC button (red circle + crosshair) with its pixel coordinate
  - a status line: inlier count, and on an abstention the reason it declined

Abstentions are drawn as prominently as detections, in amber. That is
deliberate: this detector's value is that it declines rather than guessing, so
a viewer should be able to see it declining, not just see nothing happen.

Runs at the working resolution the detector was measured at (960px long edge),
not the 4K source -- nothing downstream uses that detail and 4K would be
seconds per frame.
"""
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[2] / "src"))
from feeding_deployment.perception.appliance_perception.reference_button_detector import (  # noqa: E402
    ReferenceButtonDetector,
)

WORK_LONG_EDGE = 960


def main():
    src = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        ROOT.parent / "button_photos" / "2C3103B0-5173-4F73-BFE2-3E059285BFF7.MP4")
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else (
        src.with_name(src.stem + "_reference_overlay.mp4"))

    det = ReferenceButtonDetector(ROOT / "reference")
    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        sys.exit(f"could not open {src}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 24.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w0 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h0 = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    s = WORK_LONG_EDGE / float(max(w0, h0))
    W, H = int(round(w0 * s)), int(round(h0 * s))

    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"avc1"), fps, (W, H))
    if not writer.isOpened():
        sys.exit(f"could not open writer for {out} (no H.264 support?)")

    found = 0
    for i in range(n):
        ok, frame = cap.read()
        if not ok:
            break
        small = cv2.resize(frame, (W, H), interpolation=cv2.INTER_AREA)
        res = det.detect(small)
        vis = small.copy()

        center = res.get("center")
        if center is not None:
            found += 1
            quad = res.get("quad")
            if quad is not None:
                cv2.polylines(vis, [quad.reshape(-1, 2).astype(np.int32)], True,
                              (0, 255, 0), 2, cv2.LINE_AA)
            px = (int(round(center[0])), int(round(center[1])))
            cv2.circle(vis, px, 16, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.drawMarker(vis, px, (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
            txt = f"START +30SEC  ({px[0]}, {px[1]})"
            for col, th in (((0, 0, 0), 4), ((0, 0, 255), 2)):
                cv2.putText(vis, txt, (px[0] + 22, px[1] - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, th, cv2.LINE_AA)
            status = f"LOCKED   inliers={res.get('inliers')}"
            colour = (0, 255, 0)
        else:
            status = f"ABSTAIN  ({res.get('reason', 'no fit')})"
            colour = (0, 190, 255)

        cv2.rectangle(vis, (0, 0), (W, 40), (0, 0, 0), -1)
        cv2.putText(vis, f"frame {i:4d}   {status}", (14, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, colour, 2, cv2.LINE_AA)
        writer.write(vis)
        if i % 150 == 0:
            print(f"  {i}/{n}", flush=True)

    cap.release()
    writer.release()
    print(f"wrote {out}\n{found}/{n} frames produced a button "
          f"({found/max(n,1):.0%}); the rest abstained")


if __name__ == "__main__":
    main()
