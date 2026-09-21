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
import os
import statistics
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

# Set BUTTON_DEBUG_TIMING=1 to print a per-stage breakdown from
# detect_circles_multiscale (which estimator/band was tried, how long each
# HoughCircles call took) -- diagnosing why detection glitches under motion.
_DEBUG_TIMING = os.environ.get("BUTTON_DEBUG_TIMING") == "1"

# The body-scale/depth dynamic-radius estimators (see
# estimate_expected_radius_px_from_body / estimate_expected_radius_px) need
# a real, stable target to lock onto. Against the cluttered photo-on-a-
# monitor test rig they instead lock onto whichever background rectangle
# won that frame, jumping between ~12-19px estimates frame to frame and
# making the drawn overlay flicker between different noise each frame.
# Off by default until validated against the real physical microwave; set
# BUTTON_DYNAMIC_RADIUS=1 to re-enable.
_DYNAMIC_RADIUS_ENABLED = os.environ.get("BUTTON_DYNAMIC_RADIUS") == "1"

# Scale-invariant detection: rather than guessing which of a few hand-picked
# pixel-radius buckets the buttons will fall into (the old RADIUS_BANDS list
# -- (15,27) tuned for a close-up photo, (40,100) for arm's-length against
# the real wrist camera, etc., each missing whatever size fell between/
# outside them), generate a geometric sequence of overlapping bands that
# covers the whole plausible range continuously (see _generate_radius_bands)
# and try each in turn.
#
# A single HoughCircles call across that *whole* range at once was tried
# first and rejected: with minRadius/maxRadius spanning 3-240px on a 640x480
# frame it returned 300+ raw hits (dial rim, text/reflection edges, screws,
# ...) at every scale simultaneously, and no per-candidate filter could
# reliably separate the 5 real buttons' cluster from that soup. Narrow bands
# keep each HoughCircles call's own tuned minDist working near its intended
# scale, so the relative filters below (radius-outlier-vs-median,
# exact-candidate-count, row-layout) only ever have to disambiguate within
# one scale at a time -- same as the old fixed-band approach, just with
# bands generated to cover any size instead of a few preset ones.
MIN_RADIUS_PX = 3
# A button can't be bigger than the frame; bounding the top of the band
# sequence by the image itself (see detect_circles_multiscale) keeps it from
# being unbounded while still not baking in any particular working distance.
MAX_RADIUS_FRAC_OF_FRAME = 0.5
# Each band's upper bound is this many times its lower bound...
RADIUS_BAND_GROWTH = 1.6
# ...and each next band starts this fraction of the way back into the
# previous one, so a real button size sitting near a band boundary still
# falls solidly inside at least one band instead of being split across two.
RADIUS_BAND_OVERLAP_FRAC = 0.3
# A single physical button can produce more than one HoughCircles hit at
# slightly different centers/radii within the same band (the same edge fits
# a couple of nearby radii almost as well). Two hits whose centers are
# closer than this fraction of their combined radius are treated as the
# same button, not two -- see _dedupe_by_overlap.
DEDUPE_OVERLAP_FRAC = 0.6

# Require the full panel. A partial catch (e.g. only the top row) used to
# be accepted for genuine occlusion/edge-of-frame cases, but pick_start_button
# has no way to know it's only seeing part of the panel -- it confidently
# labels "closest to the bottom-right corner of whatever WAS detected" as
# START, which silently mislabels a real button whenever Hough misses one
# (observed: catching only the top 3 of 5 got the top-right button labeled
# START instead of the real bottom-right START button). Rejecting anything
# short of all 5 trades some "no candidates" frames for never mislabeling.
MIN_CANDIDATES = 5
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


# Real microwave buttons are roughly this size -- used to convert a depth
# reading into an expected pixel radius (r_px = fx * REAL_BUTTON_RADIUS_M /
# depth_m) instead of guessing a fixed pixel-radius band. Not precisely
# measured against this rig's actual buttons; if the dynamic band keeps
# missing high or low, adjust this first.
REAL_BUTTON_RADIUS_M = 0.01

# Approximate width of a compact countertop microwave (the long side of its
# front face) -- used as an in-frame scale reference instead of an absolute
# depth reading. Not measured against this rig's actual unit; verify if the
# body-scale estimate is consistently off.
REAL_MICROWAVE_WIDTH_M = 0.45


def detect_microwave_body_width_px(gray, frame_area_frac_min=0.15, aspect_range=(1.0, 3.0)):
    """Largest appliance-shaped rectangle in frame, by contour area --
    the long side of its minAreaRect, in pixels.

    Unlike a depth reading, this doesn't assume the camera is looking at a
    real 3D object at some distance -- it derives scale from the ratio of
    two things both visible in the same 2D frame (the appliance's known
    real width vs. its apparent width here), so it holds up whether the
    "microwave" is the real 3D unit or a photo of one on a screen, and
    re-derives itself every frame rather than trusting one absolute number.

    Caveat: this picks the *largest* appliance-shaped rectangle in frame.
    If something else large and similarly-shaped is also in view (e.g. a
    monitor bezel showing a photo of the microwave), it can lock onto that
    instead -- there's no semantic check that the rectangle found is
    actually the microwave."""
    edges = cv2.Canny(gray, 50, 150)
    edges = cv2.dilate(edges, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    frame_area = gray.shape[0] * gray.shape[1]
    best_width, best_area = None, 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < frame_area_frac_min * frame_area or area <= best_area:
            continue
        (rw, rh) = cv2.minAreaRect(c)[1]
        if rw <= 0 or rh <= 0:
            continue
        aspect = max(rw, rh) / min(rw, rh)
        if not (aspect_range[0] <= aspect <= aspect_range[1]):
            continue
        best_area = area
        best_width = max(rw, rh)
    return best_width


def estimate_expected_radius_px_from_body(gray, real_radius_m=REAL_BUTTON_RADIUS_M):
    """Body-scale version of estimate_expected_radius_px -- tried first
    (see detect_circles_multiscale) since it doesn't need depth at all."""
    body_px = detect_microwave_body_width_px(gray)
    if body_px is None:
        return None
    pixels_per_meter = body_px / REAL_MICROWAVE_WIDTH_M
    return pixels_per_meter * real_radius_m


def estimate_expected_radius_px(depth_image, camera_info, real_radius_m=REAL_BUTTON_RADIUS_M, roi_frac=0.2):
    """Median depth over a small central ROI -> expected on-screen button
    radius via pinhole projection. Returns None if the ROI has no valid
    depth (out of range / a hole), so callers can fall back to the static
    RADIUS_BANDS -- this is the same 0.05-2.0m sanity window
    detect_button_pose_live.py's pixel2world uses.

    Assumes the button panel is roughly centered and depth-visible, which
    holds once the arm is approaching it (not for arbitrary framing)."""
    h, w = depth_image.shape[:2]
    cy, cx = h // 2, w // 2
    half_h, half_w = int(h * roi_frac / 2), int(w * roi_frac / 2)
    roi = depth_image[cy - half_h : cy + half_h, cx - half_w : cx + half_w]
    valid = roi[(roi > 50) & (roi < 2000)]  # mm
    if valid.size == 0:
        return None

    depth_m = float(np.median(valid)) / 1000.0
    fx = camera_info.K[0]
    return fx * real_radius_m / depth_m


def _dedupe_by_overlap(candidates, overlap_frac=DEDUPE_OVERLAP_FRAC):
    """Collapse multiple Hough hits on the same physical button (found at
    several nearby radii) into one. cv2.HoughCircles returns candidates
    strongest-first, so keep the first hit in each cluster and drop later
    ones whose center falls within overlap_frac of the pair's combined
    radius -- real, distinct buttons don't sit that close together."""
    kept = []
    for c in candidates:
        cx, cy = c["center"]
        if any(
            ((cx - k["center"][0]) ** 2 + (cy - k["center"][1]) ** 2) ** 0.5
            < overlap_frac * (c["r"] + k["r"])
            for k in kept
        ):
            continue
        kept.append(c)
    return kept


def _generate_radius_bands(min_r, max_r, growth=RADIUS_BAND_GROWTH, overlap_frac=RADIUS_BAND_OVERLAP_FRAC):
    """Geometric sequence of overlapping (lo, hi) radius bands covering
    [min_r, max_r] continuously, so any real button size falls solidly
    inside at least one band -- see the module-level comment above
    MIN_RADIUS_PX for why this replaces both a single wide-range Hough call
    and a short hand-picked band list."""
    bands = []
    lo = min_r
    while lo < max_r:
        hi = min(int(lo * growth) + 2, max_r)
        bands.append((lo, hi))
        if hi >= max_r:
            break
        lo = max(min_r, int(hi * (1 - overlap_frac)))
    return bands


# HoughCircles' own edge-strength threshold. Lower than a typical single-band
# default (was 30) because within a *narrow* band that's tuned to the right
# scale, a stricter threshold was found (on real glare-prone/scratched-finish
# footage) to miss real buttons that are just slightly softer-edged than the
# others in the same frame -- see the conversation this constant came from.
# The exact-candidate-count + outlier + layout checks below are what keep
# this leniency from letting non-button circles through.
HOUGH_PARAM2 = 20


def _try_radius_band(blurred, min_r, max_r):
    """One HoughCircles pass + de-dup + the same plausibility checks every
    band goes through. Returns candidates or None."""
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.0,
        minDist=max(int(min_r * 2.0), 8),
        param1=100,
        param2=HOUGH_PARAM2,
        minRadius=min_r,
        maxRadius=max_r,
    )
    if circles is None:
        return None

    circ = np.round(circles[0]).astype(int)
    candidates = [{"center": (int(x), int(y)), "r": int(r)} for x, y, r in circ]
    candidates = _dedupe_by_overlap(candidates)
    candidates = _filter_radius_outliers(candidates)
    if not (MIN_CANDIDATES <= len(candidates) <= MAX_CANDIDATES):
        return None
    if not _plausible_layout(candidates):
        return None
    return candidates


def detect_circles_multiscale(gray, depth_image=None, camera_info=None):
    """Try, in order: (1) the microwave-body-scale estimate (no depth
    needed, re-derived every frame from the appliance's own known real
    width -- see estimate_expected_radius_px_from_body), (2) a depth-derived
    radius band (if depth_image/camera_info are given and depth is valid
    where the panel should be), (3) a geometric sequence of bands covering
    the whole plausible radius range (see _generate_radius_bands). (1) and
    (2) are a fast path when depth/body-scale is available and trustworthy;
    (3) is what actually holds at any distance/zoom otherwise -- replaces
    the old fixed-bucket RADIUS_BANDS list, which only found buttons whose
    apparent size happened to fall inside one of a few preset ranges.
    Returns (candidates, band_used) or (None, None)."""
    _t0 = time.monotonic()
    blurred = cv2.medianBlur(gray, 5)
    if _DEBUG_TIMING:
        print(f"[detect_circles_multiscale] medianBlur: {(time.monotonic()-_t0)*1000:.1f}ms")

    if _DYNAMIC_RADIUS_ENABLED:
        _t0 = time.monotonic()
        r_px = estimate_expected_radius_px_from_body(gray)
        if _DEBUG_TIMING:
            print(f"[detect_circles_multiscale] body-scale estimate: {(time.monotonic()-_t0)*1000:.1f}ms -> r_px={r_px}")
        if r_px is None and depth_image is not None and camera_info is not None:
            r_px = estimate_expected_radius_px(depth_image, camera_info)
        if r_px is not None:
            min_r, max_r = int(r_px * 0.6), int(r_px * 1.4)
            _t0 = time.monotonic()
            candidates = _try_radius_band(blurred, min_r, max_r)
            if _DEBUG_TIMING:
                print(f"[detect_circles_multiscale] dynamic band ({min_r},{max_r}): {(time.monotonic()-_t0)*1000:.1f}ms -> {None if candidates is None else len(candidates)} candidates")
            if candidates is not None:
                return candidates, (min_r, max_r)

    max_r = max(MIN_RADIUS_PX + 1, int(min(gray.shape[:2]) * MAX_RADIUS_FRAC_OF_FRAME))
    for min_r, band_max_r in _generate_radius_bands(MIN_RADIUS_PX, max_r):
        _t0 = time.monotonic()
        candidates = _try_radius_band(blurred, min_r, band_max_r)
        if _DEBUG_TIMING:
            print(f"[detect_circles_multiscale] band ({min_r},{band_max_r}): {(time.monotonic()-_t0)*1000:.1f}ms -> {None if candidates is None else len(candidates)} candidates")
        if candidates is not None:
            return candidates, (min_r, band_max_r)
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
