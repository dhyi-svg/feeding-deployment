"""Render an annotated copy of a microwave-button video: draws every detected
button circle (green) and highlights whichever one the selection rule picks
as the START/+30s button (red when temporally stable, orange when only a
single frame's guess), plus a frame-number/circle-count/stability readout.

Standalone (no repo deps) -- same detection logic as
AppliancePerception.detect_start_button_pixel_hough, but run directly on
phone-video footage that is already right-side-up (no upside-down-camera
flip, unlike the real wrist camera).

Multi-scale: a single fixed radius band (15-27px, tuned for a close-up
photo) missed the buttons in far-away frames of IMG_3842.MOV, where they're
only ~5-12px. Tries progressively smaller radius bands and keeps the first
whose candidates pass the plausibility checks below -- this fixed most (not
all -- some failures are motion blur, not scale) of the far-frame misses
without needing depth data, which this phone footage doesn't have anyway.

Stability hardening (on top of the original exactly-5-circles version):
  - radius-outlier filtering: a circle whose radius is far from the group's
    median is more likely a reflection/scratch than a real button (this
    microwave's finish is heavily scratched and glare-prone).
  - relaxed candidate count (3-5, not just exactly 5): a real button panel
    partially out of frame or with one occluded button should still be
    usable, rather than discarded outright.
  - row-layout plausibility: reject candidate sets that don't split into
    a small number of horizontally-banded rows -- guards against a
    coincidental handful of circles (background clutter, reflections) that
    happen to number 3-5 but aren't actually a button panel.
  - temporal consensus: a pick is only trusted once the last STABLE_FRAMES
    frames agree within STABLE_PIXEL_TOL pixels. A single bad frame no
    longer produces an actionable coordinate on its own.

None of this makes single-frame classical CV detection safe to fire a real
button press off of -- see the caveats in the conversation this script came
from. It reduces (not eliminates) the odds a bad single frame reaches the
"stable" state at all.

Usage:
    python3 scripts/scratch/render_button_detection_overlay.py <input_video> [output_video]

If output_video is omitted, writes "<input>_detection_overlay.mp4" next to
the input. Requires H.264 support in the local OpenCV/ffmpeg build (avc1
fourcc) for the output to be playable in QuickTime/Preview -- the mp4v
fourcc writes a container many mac video players refuse to open.
"""
import statistics
import sys
from collections import deque
from pathlib import Path

import cv2
import numpy as np

# Progressively smaller radius bands -- tried in order, first band whose
# candidates pass the plausibility checks wins. (15,27) alone is what the
# real detect_start_button_pixel_hough uses today (tuned for a close-up
# photo); the smaller bands are what recovers far-away frames.
RADIUS_BANDS = [(15, 27), (8, 20), (5, 12)]

# Accept a partial panel (occlusion / edge-of-frame) instead of requiring
# all 5 buttons, but not so few that "candidates" stops meaning "a button
# panel".
MIN_CANDIDATES = 3
MAX_CANDIDATES = 5

# A circle whose radius differs from the group median by more than this
# fraction of the median is treated as a mis-detection, not a button.
RADIUS_OUTLIER_TOL_FRAC = 0.35

# Row-split: sorted by y, a gap this many times the mean of the other gaps
# is treated as the boundary between rows (not noise within one row).
ROW_GAP_RATIO = 1.5

# Temporal consensus: how many consecutive frames' picks must agree, and
# how close (pixels) they must be, before a pick counts as "stable" rather
# than a single frame's unconfirmed guess.
STABLE_FRAMES = 5
STABLE_PIXEL_TOL = 12


def _filter_radius_outliers(candidates):
    """Drop circles whose radius is far from the group's median -- glare
    and scratches on this microwave's finish occasionally Hough-match as a
    circle a very different size from the real buttons."""
    radii = [c["r"] for c in candidates]
    median_r = statistics.median(radii)
    if median_r <= 0:
        return candidates
    return [
        c
        for c in candidates
        if abs(c["r"] - median_r) <= RADIUS_OUTLIER_TOL_FRAC * median_r
    ]


def _split_into_rows(candidates):
    """Sort by y and split at the single largest gap, if that gap clearly
    stands out from the others -- a cheap way to test "does this look like
    a small number of horizontal rows" without assuming exactly 2 rows or
    a fixed pixel tolerance (which is what broke under perspective before).
    """
    by_y = sorted(candidates, key=lambda c: c["center"][1])
    if len(by_y) <= 2:
        return [by_y]

    gaps = [by_y[i + 1]["center"][1] - by_y[i]["center"][1] for i in range(len(by_y) - 1)]
    max_gap = max(gaps)
    max_idx = gaps.index(max_gap)
    other_gaps = [g for i, g in enumerate(gaps) if i != max_idx]
    mean_other = statistics.mean(other_gaps) if other_gaps else 0.0

    if mean_other > 0 and max_gap > ROW_GAP_RATIO * mean_other:
        return [by_y[: max_idx + 1], by_y[max_idx + 1 :]]
    return [by_y]


def _plausible_layout(candidates):
    """Reject candidate sets that don't look like a real button panel: too
    many rows for a 5-button, <=3-row panel, or a row so tall it can't be
    one physical row of buttons."""
    rows = _split_into_rows(candidates)
    if len(rows) > 3:
        return False
    for row in rows:
        if len(row) <= 1:
            continue
        row_span = max(c["center"][1] for c in row) - min(c["center"][1] for c in row)
        row_width = max(c["center"][0] for c in row) - min(c["center"][0] for c in row)
        # A real row is wide relative to how tall it is; a vertically
        # spread-out "row" is more likely leftover mis-clustering.
        if row_width > 0 and row_span > 0.6 * row_width:
            return False
    return True


def pick_start_button(candidates):
    """Closest to the bottom-right corner of the candidates' own bounding
    box. Replaces an earlier "bottom row (fixed pixel tolerance), then
    rightmost" rule, which broke under perspective: at oblique/far
    viewing angles the top-right button's y can land within that fixed
    tolerance of the true bottom row, so it got swept in and won on x.
    Scoring by distance to (max_x, max_y) instead weighs x and y jointly,
    so no single hard row cutoff has to hold across viewing angles.
    Callers whose footage is already right-side-up (like this script's
    phone-video input) pass raw (x, y) centers directly; the real wrist
    camera needs the upside-down flip applied first (see
    appliance_perception.py)."""
    corner_x = max(c["center"][0] for c in candidates)
    corner_y = max(c["center"][1] for c in candidates)

    def dist_to_corner(c):
        dx = c["center"][0] - corner_x
        dy = c["center"][1] - corner_y
        return dx * dx + dy * dy

    return min(candidates, key=dist_to_corner)


def detect_circles_multiscale(gray):
    """Try each radius band in turn; keep the first whose circle count (3-5),
    radius consistency, and row layout all look like a real button panel.
    Returns (candidates, band_used) or (None, None)."""
    blurred = cv2.medianBlur(gray, 5)
    for min_r, max_r in RADIUS_BANDS:
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=1.0,
            minDist=int(min_r * 2.5),
            param1=100,
            param2=30,
            minRadius=min_r,
            maxRadius=max_r,
        )
        if circles is None:
            continue

        circ = np.round(circles[0]).astype(int)
        candidates = [{"center": (int(x), int(y)), "r": int(r)} for x, y, r in circ]
        candidates = _filter_radius_outliers(candidates)
        if not (MIN_CANDIDATES <= len(candidates) <= MAX_CANDIDATES):
            continue
        if not _plausible_layout(candidates):
            continue
        return candidates, (min_r, max_r)
    return None, None


class StabilityTracker:
    """Tracks whether the last STABLE_FRAMES picks agree within
    STABLE_PIXEL_TOL pixels of each other. A single frame's pick is never
    trusted on its own -- see the module docstring."""

    def __init__(self):
        self.history = deque(maxlen=STABLE_FRAMES)

    def update(self, center):
        self.history.append(center)
        if len(self.history) < STABLE_FRAMES:
            return False
        cx = statistics.mean(p[0] for p in self.history)
        cy = statistics.mean(p[1] for p in self.history)
        return all(
            ((p[0] - cx) ** 2 + (p[1] - cy) ** 2) ** 0.5 <= STABLE_PIXEL_TOL
            for p in self.history
        )

    def reset(self):
        self.history.clear()


def main():
    if len(sys.argv) < 2:
        sys.exit(f"Usage: {sys.argv[0]} <input_video> [output_video]")

    in_path = Path(sys.argv[1])
    out_path = Path(sys.argv[2]) if len(sys.argv) > 2 else (
        in_path.with_name(in_path.stem + "_detection_overlay.mp4")
    )

    cap = cv2.VideoCapture(str(in_path))
    if not cap.isOpened():
        sys.exit(f"Could not open {in_path}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    fourcc = cv2.VideoWriter_fourcc(*"avc1")
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (w, h))
    if not writer.isOpened():
        sys.exit(f"Could not open VideoWriter for {out_path}")

    tracker = StabilityTracker()
    count_found = 0
    count_stable = 0
    for i in range(n):
        ok, frame = cap.read()
        if not ok:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        candidates, band = detect_circles_multiscale(gray)

        vis = frame.copy()
        n_circles = 0
        stable = False
        if candidates is not None:
            n_circles = len(candidates)
            count_found += 1
            chosen = pick_start_button(candidates)
            stable = tracker.update(chosen["center"])
            if stable:
                count_stable += 1

            for c in candidates:
                if c is chosen:
                    continue
                cv2.circle(vis, c["center"], c["r"], (0, 255, 0), 3)
                cv2.circle(vis, c["center"], 3, (0, 255, 0), 4)
            # Drawn last (on top) so the chosen button always stands out.
            # Red = temporally stable (last STABLE_FRAMES picks agree);
            # orange = this frame's unconfirmed guess only.
            chosen_color = (0, 0, 255) if stable else (0, 140, 255)
            cv2.circle(vis, chosen["center"], chosen["r"] + 4, chosen_color, 4)
            cv2.circle(vis, chosen["center"], 3, chosen_color, 4)
            # Pixel coordinates only -- this is plain phone video with no
            # depth channel and no camera/robot calibration, so there is no
            # real-world x/y/z to show here. That conversion (pixel -> 3D ->
            # arm_base_link) only exists in detect_button_pose_live.py,
            # which needs the real depth camera + calibration to run.
            coord_text = f"({chosen['center'][0]}, {chosen['center'][1]})px"
            text_pos = (chosen["center"][0] + 15, chosen["center"][1] - 15)
            cv2.putText(
                vis, coord_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 0), 4, cv2.LINE_AA,
            )
            cv2.putText(
                vis, coord_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                chosen_color, 2, cv2.LINE_AA,
            )
        else:
            tracker.reset()

        band_text = f"band {band[0]}-{band[1]}px" if band else "no plausible band"
        status_text = "STABLE" if stable else "unconfirmed"
        cv2.putText(
            vis,
            f"frame {i} | {n_circles} candidates | {band_text} | {status_text}",
            (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.2,
            (255, 255, 255),
            3,
            cv2.LINE_AA,
        )
        writer.write(vis)

    cap.release()
    writer.release()
    print(
        f"Wrote {out_path}: {count_found}/{n} frames had a plausible pick, "
        f"{count_stable}/{n} were temporally stable"
    )


if __name__ == "__main__":
    main()
