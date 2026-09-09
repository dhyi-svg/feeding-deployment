"""Stage 0 (vision-only, zero hardware risk): live overlay showing the detected
microwave start-button pixel and the tracked gripper-marker (teal foam pad) pixel,
with a line between them and the live pixel distance -- this is the "two dots"
visualization for the planned visual-servo button press.

No `arm_interface` import, no arm connection of any kind. Pure perception smoke test
per the plan at ~/.claude/plans/ok-lets-work-together-kind-acorn.md, Stage 0.

Button detection: HSV saturation/value threshold (chrome buttons are low-saturation
against the high-saturation red panel) -> morphological clean -> contour + circularity
filter -> minEnclosingCircle. Mirrors the sanity-filtering shape of the real
detect_start_button_pixel_local (appliance_perception.py:507-537) closely enough that
swapping in the real GroundingDINO backend later is a drop-in change.

Marker detection: HSV hue threshold tuned for the teal foam pad (sampled empirically
this session: H~73-84, S~65-98, V~97-198 -- distinct from both the red panel, H~8,
and the chrome buttons, which are low-saturation) -> same clean/contour pipeline ->
largest-area surviving blob (the pad is a large, distinctly-colored patch, so area
is more robust here than circularity).

Trackbars let both thresholds be tuned live instead of guessing from a static image.
Press 'f' to toggle the upside-down-camera flip, 'q'/Esc to quit.
"""

import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs

WINDOW = "Button + claw-marker detection (q/Esc quit, f flip)"
ROW_TOL_FRAC = 0.06

BUTTON_MIN_AREA = 100
BUTTON_MAX_AREA = 900
BUTTON_MIN_CIRCULARITY = 0.5

MARKER_MIN_AREA = 150

_last_marker_seen_iter = None
_missing_marker_streak = 0


def nothing(_):
    pass


def build_trackbars():
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("BtnSatMax", WINDOW, 90, 255, nothing)
    cv2.createTrackbar("BtnValMin", WINDOW, 90, 255, nothing)
    cv2.createTrackbar("MarkerHueMin", WINDOW, 60, 179, nothing)
    cv2.createTrackbar("MarkerHueMax", WINDOW, 100, 179, nothing)
    cv2.createTrackbar("MarkerSatMin", WINDOW, 50, 255, nothing)
    cv2.createTrackbar("MarkerValMin", WINDOW, 60, 255, nothing)


def _clean_mask(mask):
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def detect_buttons(hsv, height):
    sat_max = cv2.getTrackbarPos("BtnSatMax", WINDOW)
    val_min = cv2.getTrackbarPos("BtnValMin", WINDOW)
    mask = cv2.inRange(hsv, (0, 0, val_min), (180, sat_max, 255))
    mask = _clean_mask(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < BUTTON_MIN_AREA or area > BUTTON_MAX_AREA:
            continue
        perim = cv2.arcLength(c, True)
        if perim <= 0:
            continue
        circularity = 4 * np.pi * area / (perim * perim)
        if circularity < BUTTON_MIN_CIRCULARITY:
            continue
        (x, y), r = cv2.minEnclosingCircle(c)
        candidates.append({"center": (float(x), float(y)), "radius": float(r)})

    chosen = None
    if candidates:
        lowest_y = max(c["center"][1] for c in candidates)
        row_tol = ROW_TOL_FRAC * height
        bottom_row = [c for c in candidates if lowest_y - c["center"][1] <= row_tol]
        chosen = max(bottom_row, key=lambda c: c["center"][0])

    return mask, candidates, chosen


def detect_marker(hsv, target_pixel=None):
    """Find the claw marker blob, then report its EDGE point facing the target --
    i.e. the point on the pad's boundary that actually reaches toward the button --
    rather than the blob's centroid. The centroid of a bulky foam pad sits well
    behind the pad's leading tip; aligning on it would leave the true contact point
    short of the target by roughly the pad's own radius.
    """
    hue_min = cv2.getTrackbarPos("MarkerHueMin", WINDOW)
    hue_max = cv2.getTrackbarPos("MarkerHueMax", WINDOW)
    sat_min = cv2.getTrackbarPos("MarkerSatMin", WINDOW)
    val_min = cv2.getTrackbarPos("MarkerValMin", WINDOW)
    mask = cv2.inRange(hsv, (hue_min, sat_min, val_min), (hue_max, 255, 255))
    mask = _clean_mask(mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_contour = None
    best_area = 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if area < MARKER_MIN_AREA:
            continue
        if area > best_area:
            best_contour = c
            best_area = area

    if best_contour is None:
        return mask, None

    M = cv2.moments(best_contour)
    if M["m00"] <= 0:
        return mask, None
    centroid = (M["m10"] / M["m00"], M["m01"] / M["m00"])

    pts = best_contour.reshape(-1, 2).astype(float)
    if target_pixel is not None:
        direction = np.array(target_pixel, dtype=float) - np.array(centroid)
        norm = np.linalg.norm(direction)
        if norm > 1e-6:
            direction /= norm
            projections = pts @ direction
            edge_point = tuple(pts[int(np.argmax(projections))])
        else:
            edge_point = centroid
    else:
        # No target known yet: default to the topmost boundary point (the wedge's
        # own tip, based on how these pads sit in-frame) as a reasonable default.
        edge_point = tuple(pts[int(np.argmin(pts[:, 1]))])

    return mask, {"center": edge_point, "centroid": centroid, "area": best_area}


def main():
    global _missing_marker_streak

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    try:
        pipeline.start(config)
    except Exception as e:
        print(f"FAILED to start pipeline: {e}", file=sys.stderr)
        sys.exit(1)

    build_trackbars()
    flip = False
    button_pixel_locked = None  # perceive once, per the plan (button doesn't move)

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            raw = np.asanyarray(color_frame.get_data())
            work = cv2.flip(raw, -1) if flip else raw
            hsv = cv2.cvtColor(work, cv2.COLOR_BGR2HSV)
            height, width = work.shape[:2]

            _, btn_candidates, btn_chosen = detect_buttons(hsv, height)
            if button_pixel_locked is None and btn_chosen is not None:
                button_pixel_locked = btn_chosen["center"]
                print(f"[button] locked pixel {button_pixel_locked}")

            _, marker = detect_marker(hsv)
            if marker is None:
                _missing_marker_streak += 1
                if _missing_marker_streak == 5:
                    print("[marker] WARNING: lost for 5+ consecutive frames")
            else:
                _missing_marker_streak = 0

            vis = work.copy()
            for c in btn_candidates:
                cx, cy = int(round(c["center"][0])), int(round(c["center"][1]))
                cv2.circle(vis, (cx, cy), 4, (60, 190, 255), 1)
                cv2.circle(vis, (cx, cy), 1, (60, 190, 255), -1)

            button_pt = None
            if button_pixel_locked is not None:
                button_pt = (int(round(button_pixel_locked[0])), int(round(button_pixel_locked[1])))
                cv2.circle(vis, button_pt, 6, (0, 0, 255), 2)
                cv2.circle(vis, button_pt, 2, (0, 0, 255), -1)
                cv2.putText(vis, "BUTTON", (button_pt[0] + 8, button_pt[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)

            marker_pt = None
            if marker is not None:
                marker_pt = (int(round(marker["center"][0])), int(round(marker["center"][1])))
                cv2.circle(vis, marker_pt, 6, (255, 220, 0), 2)
                cv2.circle(vis, marker_pt, 2, (255, 220, 0), -1)
                cv2.putText(vis, "CLAW", (marker_pt[0] + 8, marker_pt[1] + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 220, 0), 1)

            if button_pt is not None and marker_pt is not None:
                cv2.line(vis, button_pt, marker_pt, (255, 255, 255), 1)
                dist = float(np.hypot(button_pt[0] - marker_pt[0], button_pt[1] - marker_pt[1]))
                mid = ((button_pt[0] + marker_pt[0]) // 2, (button_pt[1] + marker_pt[1]) // 2)
                cv2.putText(vis, f"{dist:.0f}px", mid, cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1)

            status = f"flip={flip} button_locked={button_pixel_locked is not None} marker={'OK' if marker else 'LOST'}"
            cv2.putText(vis, status, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow(WINDOW, vis)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("f"):
                flip = not flip
                button_pixel_locked = None  # re-lock in the new orientation
            elif key == ord("r"):
                button_pixel_locked = None  # force re-detect the button pixel
            if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
