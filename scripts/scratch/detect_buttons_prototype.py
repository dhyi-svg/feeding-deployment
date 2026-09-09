"""Standalone prototype (no repo deps): can classical CV find the microwave's
5 round chrome push-buttons using cv2.HoughCircles?

Not integrated with the real repo yet -- this only answers the yes/no "does this
approach work on this microwave" question, using the reference photo.

Usage: python3 detect_buttons_prototype.py <input_photo> <output_annotated_photo>
"""
import sys

import cv2
import numpy as np

in_path = sys.argv[1]
out_path = sys.argv[2]

img = cv2.imread(in_path)
if img is None:
    raise SystemExit(f"Could not read image: {in_path}")

gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.medianBlur(gray, 5)

# Radius range is a rough guess from the reference photo (buttons look ~40-70px
# across here) -- this will need real recalibration once run against the wrist
# camera's actual working-distance footage. This script exists to check the
# *approach*, not to produce final tuned parameters.
circles = cv2.HoughCircles(
    blurred,
    cv2.HOUGH_GRADIENT,
    dp=1.0,
    minDist=40,
    param1=100,
    param2=30,
    minRadius=15,
    maxRadius=27,
)

vis = img.copy()
if circles is None:
    print("No circles found at all -- params need loosening.")
else:
    circles = np.round(circles[0]).astype(int)
    print(f"Found {len(circles)} circle candidates:")
    for x, y, r in circles:
        print(f"  center=({x},{y}) radius={r}")
        cv2.circle(vis, (x, y), r, (0, 255, 0), 3)
        cv2.circle(vis, (x, y), 2, (0, 0, 255), 3)

cv2.imwrite(out_path, vis)
print(f"Annotated image saved to {out_path}")
