"""STEP 2 (only if step 1 said NO MATCH) -- build a reference from a camera frame.

Two stages, because they need different things from you:

  --grab   capture a frame from the camera and save it. Get the microwave
           panel clearly in view first. Sharp matters more than close.

  --mark   given roughly where the +30SEC button is in that frame, fit a circle
           to it and write reference.json.

The circle fit is not a nicety. The single most leveraged number in this whole
detector is the button's centre on the reference: every prediction is that one
point carried through the fitted transform, so an error there appears in every
frame forever. On the phone-video reference it was eyeballed, came out ~3px
off, and the bias was invisible to every automated score because the eval
labels carried the same bias -- it was caught by a human looking at the overlay.
So: give a rough click point, let the fit place the centre.

Usage:
    python3 make_reference.py --grab --realsense --out ref_frame.png
    python3 make_reference.py --grab --topic /camera/color/image_raw --out ref_frame.png

    # then eyeball roughly where the button is in ref_frame.png and:
    python3 make_reference.py --mark ref_frame.png --near 410 300 \
        --panel 330 250 520 380 --ref-dir ./wrist_ref
"""
import argparse
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np


def grab(args):
    if args.topic:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        rclpy.init()
        node = Node("button_ref_grab")
        bridge = CvBridge()
        box = {}
        node.create_subscription(Image, args.topic,
                                 lambda m: box.setdefault("img", bridge.imgmsg_to_cv2(m, "bgr8")), 10)
        end = node.get_clock().now().nanoseconds + int(10e9)
        while "img" not in box and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node(); rclpy.shutdown()
        if "img" not in box:
            sys.exit(f"no image on {args.topic}")
        img = box["img"]
    else:
        import pyrealsense2 as rs
        pipe = rs.pipeline(); cfg = rs.config()
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipe.start(cfg)
        try:
            for _ in range(15):
                frames = pipe.wait_for_frames()
            img = np.asanyarray(frames.get_color_frame().get_data())
        finally:
            pipe.stop()
    cv2.imwrite(args.out, img)
    sharp = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    print(f"wrote {args.out}  ({img.shape[1]}x{img.shape[0]}, sharpness {sharp:.0f})")
    if sharp < 100:
        print("  WARNING: this frame looks soft. A blurred reference matches badly at")
        print("  every distance. Steady the camera and grab another.")


def mark(args):
    img = cv2.imread(args.mark)
    if img is None:
        sys.exit(f"could not read {args.mark}")
    nx, ny = args.near
    r_guess = args.radius

    win = max(24, int(r_guess * 2.5))
    x0, y0 = max(0, nx - win), max(0, ny - win)
    x1, y1 = min(img.shape[1], nx + win), min(img.shape[0], ny + win)
    crop = cv2.medianBlur(cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY), 5)
    circles = cv2.HoughCircles(crop, cv2.HOUGH_GRADIENT, dp=1.0,
                               minDist=max(8, r_guess), param1=100, param2=30,
                               minRadius=int(r_guess * 0.5), maxRadius=int(r_guess * 1.8))
    if circles is None:
        print("no circle found near that point -- using your point as-is.")
        print("  Check --near is actually on the button and --radius is about right.")
        cx, cy, rr = float(nx), float(ny), float(r_guess)
        source = "hand-placed (no circle fit)"
    else:
        cc = np.round(circles[0]).astype(int)
        hx, hy = crop.shape[1] / 2, crop.shape[0] / 2
        x, y, rr = sorted(cc, key=lambda q: (q[0] - hx) ** 2 + (q[1] - hy) ** 2)[0]
        cx, cy = float(x0 + x), float(y0 + y)
        source = "circle-fit"
        print(f"circle fit moved the mark {np.hypot(cx-nx, cy-ny):.1f}px from your point")

    ref_dir = Path(args.ref_dir)
    ref_dir.mkdir(parents=True, exist_ok=True)
    img_name = "reference_frame.png"
    shutil.copy(args.mark, ref_dir / img_name)

    px0, py0, px1, py1 = args.panel
    meta = {
        "_comment": [
            "Reference built from a CAMERA frame (not phone video).",
            f"button_xy placed by {source}; do not adjust it by eye -- every",
            "prediction is this one point carried through the fitted transform,",
            "so an error here appears in every frame.",
            "crop bounds the control panel: include the printed labels and the",
            "dial. A low-texture panel leans heavily on its printed text for",
            "SIFT features.",
        ],
        "image": img_name,
        "crop": [int(px0), int(py0), int(px1), int(py1)],
        "button_xy": [round(cx, 1), round(cy, 1)],
        "button_radius": int(round(rr)),
        "appliance": args.appliance,
    }
    (ref_dir / "reference.json").write_text(json.dumps(meta, indent=2))

    vis = img.copy()
    cv2.rectangle(vis, (int(px0), int(py0)), (int(px1), int(py1)), (0, 255, 0), 2)
    cv2.circle(vis, (int(cx), int(cy)), int(rr), (0, 0, 255), 2)
    cv2.drawMarker(vis, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    check = ref_dir / "reference_check.png"
    cv2.imwrite(str(check), vis)
    print(f"wrote {ref_dir/'reference.json'}  button_xy={meta['button_xy']} r={meta['button_radius']}")
    print(f"\nLOOK AT {check} BEFORE USING THIS.")
    print("  Red marker must sit on the +30SEC button (bottom-right of the five),")
    print("  NOT on STOP/ECO next to it. Green box must enclose the panel.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--grab", action="store_true")
    ap.add_argument("--topic")
    ap.add_argument("--realsense", action="store_true")
    ap.add_argument("--out", default="ref_frame.png")
    ap.add_argument("--mark")
    ap.add_argument("--near", nargs=2, type=int, metavar=("X", "Y"),
                    help="roughly where the +30SEC button is")
    ap.add_argument("--panel", nargs=4, type=int, metavar=("X0", "Y0", "X1", "Y1"),
                    help="box around the whole control panel")
    ap.add_argument("--radius", type=int, default=14, help="rough button radius in px")
    ap.add_argument("--ref-dir", default="./wrist_ref")
    ap.add_argument("--appliance", default="comfee-cream")
    args = ap.parse_args()

    if args.grab:
        grab(args)
    elif args.mark:
        if not args.near or not args.panel:
            sys.exit("--mark needs both --near X Y and --panel X0 Y0 X1 Y1")
        mark(args)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
