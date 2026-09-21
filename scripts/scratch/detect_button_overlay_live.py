"""Live button detection with a visible overlay window. READ-ONLY: only
displays annotated frames, never calls move_to_ee_pose or anything else
that moves the arm.

Same detection algorithm and camera plumbing as detect_button_pose_live.py
(reused, not re-copied a third time) but skips the pixel -> 3D -> robot-
frame math entirely -- this is for *looking at* what the detector picks,
not for producing a pose. Doesn't need arm_base_link/tf2 at all, so it
still works with just the camera up and no arm_server connection.

The overlay drawing itself mirrors render_button_detection_overlay.py's
main() (green = candidate, red = stable pick, orange = unconfirmed pick),
adapted from writing an output video to a live cv2.imshow window.

Usage (camera already running -- see PACHIRISU_SETUP.md / JETSON_SETUP.md,
or on a box without the full bring-up, any node publishing
<ns>/color/image_raw + .../camera_info is enough since this script never
looks up tf):
    python3 scripts/scratch/detect_button_overlay_live.py

Press 'q' or Esc in the window, or Ctrl-C in the terminal, to stop.
"""
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from render_button_detection_overlay import (  # noqa: E402
    StabilityTracker,
    detect_circles_multiscale,
    pick_start_button,
)

# The claw-tip pixel, located by hand (see the CLAW_PIXEL assignment below)
# rather than live-detected -- the camera's static right now, so this
# doesn't need to be re-derived every frame.
CLAW_PIXEL = (385, 404)

try:
    import rospy

    from feeding_deployment.interfaces.realsense_interface import RealSenseInterface  # noqa: E402

    ROS_VERSION = 1
except ModuleNotFoundError:
    import rclpy

    from feeding_deployment.ros2.node import get_node  # noqa: E402
    from feeding_deployment.ros2.realsense_ros2_interface import (  # noqa: E402
        RealSenseROS2Interface,
    )

    ROS_VERSION = 2

WINDOW = "button detection (upright view)"


def _is_shutdown():
    if ROS_VERSION == 1:
        return rospy.is_shutdown()
    return not rclpy.ok()


def draw_marker(vis, marker):
    """Claw-tip dot -- same colour probe_button_marker_detect.py uses for
    the gripper-marker point, no label."""
    if marker is None:
        return
    pt = (int(round(marker["center"][0])), int(round(marker["center"][1])))
    cv2.circle(vis, pt, 6, (255, 220, 0), 2)
    cv2.circle(vis, pt, 2, (255, 220, 0), -1)


def draw_overlay(upright_bgr, candidates, chosen, stable, band, marker):
    """Same colour/annotation scheme as render_button_detection_overlay.py's
    main(), just against a live frame instead of a VideoWriter."""
    vis = upright_bgr.copy()
    band_text = f"band {band[0]}-{band[1]}px" if band else "no plausible band"
    if candidates is None:
        cv2.putText(
            vis, f"no plausible button candidates | {band_text}", (20, 50),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA,
        )
        draw_marker(vis, marker)
        return vis

    for c in candidates:
        if c is chosen:
            continue
        cv2.circle(vis, c["center"], c["r"], (0, 255, 0), 3)
        cv2.circle(vis, c["center"], 3, (0, 255, 0), 4)

    chosen_color = (0, 0, 255) if stable else (0, 140, 255)
    cv2.circle(vis, chosen["center"], chosen["r"] + 4, chosen_color, 4)
    cv2.circle(vis, chosen["center"], 3, chosen_color, 4)
    coord_text = f"({chosen['center'][0]}, {chosen['center'][1]})px"
    text_pos = (chosen["center"][0] + 15, chosen["center"][1] - 15)
    cv2.putText(vis, coord_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(vis, coord_text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, chosen_color, 2, cv2.LINE_AA)

    status_text = "STABLE" if stable else "unconfirmed"
    cv2.putText(
        vis, f"{len(candidates)} candidates | {band_text} | {status_text}", (20, 50),
        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3, cv2.LINE_AA,
    )
    draw_marker(vis, marker)
    return vis


def main():
    if ROS_VERSION == 1:
        rospy.init_node("detect_button_overlay_live")
        realsense = RealSenseInterface()
    else:
        get_node("detect_button_overlay_live")
        # Matches whatever namespace the running realsense2_camera_node
        # actually published under -- pass e.g. /camera/camera as argv[1]
        # if the node was launched with a different camera_name/namespace.
        ns = sys.argv[1] if len(sys.argv) > 1 else "/camera/camera"
        realsense = RealSenseROS2Interface(
            color_topic=f"{ns}/color/image_raw",
            camera_info_topic=f"{ns}/color/camera_info",
            depth_topic=f"{ns}/aligned_depth_to_color/image_raw",
        )

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

    print("Waiting for camera data...")
    while not _is_shutdown():
        camera_data = realsense.get_camera_data()
        if camera_data["rgb_image"] is not None:
            break
        time.sleep(0.1)

    print(
        "Got camera data. Showing overlay (q/Esc to quit, 'r' to toggle the "
        "180-degree flip, READ-ONLY, no motion) ..."
    )
    tracker = StabilityTracker()
    # Only the real wrist-mounted camera is upside down; this camera isn't on
    # the robot right now, so don't assume the flip -- start upright and let
    # 'r' toggle it if the mount changes.
    flipped = False
    # get_camera_data() returns the latest frame every call, new or not --
    # without this check the loop reruns Canny/contours/multi-band Hough on
    # the same stale frame as fast as the CPU allows (pegged a full core at
    # 600%+), which starves the rclpy spin thread of the GIL and is what
    # actually made the window laggy, not the detection cost itself.
    last_stamp = None
    _last_loop_end = time.monotonic()
    _frame_count = 0

    while not _is_shutdown():
        _t_wall0 = time.monotonic()
        _gap_ms = (_t_wall0 - _last_loop_end) * 1000
        camera_data = realsense.get_camera_data()
        _t_get_ms = (time.monotonic() - _t_wall0) * 1000
        rgb_image = camera_data["rgb_image"]
        depth_image = camera_data["depth_image"]
        camera_info = camera_data["camera_info"]
        header = camera_data["header"]
        if rgb_image is None:
            time.sleep(0.1)
            continue

        stamp = (header.stamp.sec, header.stamp.nanosec) if header is not None else None
        if stamp is not None and stamp == last_stamp:
            # No new frame yet -- don't waste a detection pass on it, but
            # still service the window/keyboard so it stays responsive.
            key = cv2.waitKey(5) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                flipped = not flipped
                tracker.reset()
            continue
        last_stamp = stamp

        upright = cv2.rotate(rgb_image, cv2.ROTATE_180) if flipped else rgb_image
        gray = cv2.cvtColor(upright, cv2.COLOR_BGR2GRAY)
        upright_depth = (
            cv2.rotate(depth_image, cv2.ROTATE_180)
            if flipped and depth_image is not None
            else depth_image
        )

        _t_detect0 = time.monotonic()
        candidates, band = detect_circles_multiscale(gray, upright_depth, camera_info)
        _t_detect_ms = (time.monotonic() - _t_detect0) * 1000
        chosen = None
        stable = False
        if candidates is None:
            tracker.reset()
        else:
            chosen = pick_start_button(candidates)
            stable = tracker.update(chosen["center"])

        # CLAW_PIXEL was located in the unflipped view -- don't show it
        # against a flipped frame, it'd be in the wrong place.
        marker = {"center": CLAW_PIXEL} if not flipped else None

        _t_draw0 = time.monotonic()
        vis = draw_overlay(upright, candidates, chosen, stable, band, marker)
        _t_draw_ms = (time.monotonic() - _t_draw0) * 1000
        _t_show0 = time.monotonic()
        cv2.imshow(WINDOW, vis)
        key = cv2.waitKey(1) & 0xFF
        _t_show_ms = (time.monotonic() - _t_show0) * 1000
        _frame_count += 1
        if _frame_count % 10 == 0:
            print(
                f"[timing] gap={_gap_ms:.0f}ms get={_t_get_ms:.0f}ms "
                f"detect={_t_detect_ms:.0f}ms draw={_t_draw_ms:.0f}ms "
                f"imshow+waitKey={_t_show_ms:.0f}ms band={band}"
            )
        _last_loop_end = time.monotonic()
        if key in (ord("q"), 27):
            break
        if key == ord("r"):
            flipped = not flipped
            tracker.reset()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
