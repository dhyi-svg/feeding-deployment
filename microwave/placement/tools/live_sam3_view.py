"""Live window: camera + SAM 3 cavity mask + floor + placement target, every frame. NEVER moves the robot.

    source microwave/sheppy/env.sh; cd ~/feeding-deployment-integration; export PYTHONPATH=$PWD/src:$PYTHONPATH
    DISPLAY=:1 python3 -u microwave/placement/tools/live_sam3_view.py      # q / Esc in the window quits

Each frame runs the Part 2 pipeline on that single look (rgbd_source -> cavity_segmentation ->
floor_target) and shows perception_viz's overlay + top-down side by side; a refusal shows the camera
with its reason in red. ~1-2 Hz (SAM 3 ~0.6 s/frame on the Jetson). The latest successful target is
written to --out/latest_place_move.yaml and latest_result.json (and printed when it changes by > 1 cm).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE.parents[1]))
import floor_target as ft  # noqa: E402
import perception_viz as viz  # noqa: E402
from cavity_segmentation import CavitySegmenter, SegmentationError  # noqa: E402
from rgbd_source import AcquisitionError, RosRGBDSource  # noqa: E402


def side_by_side(a, b):
    h = max(a.shape[0], b.shape[0])
    pad = lambda x: np.vstack([x, np.zeros((h - x.shape[0], x.shape[1], 3), np.uint8)])  # noqa: E731
    return np.hstack([pad(a), pad(b)])


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument("--config", type=Path)
    a.add_argument("--out", type=Path, default=Path.home() / "microwave_place_logs" / "live_view")
    a.add_argument("--scale", type=float, default=1.0, help="window size factor")
    args = a.parse_args(argv)
    cfg = ft.load_perception_config(args.config)
    args.out.mkdir(parents=True, exist_ok=True)
    src = RosRGBDSource(cfg.camera)
    seg = CavitySegmenter(cfg.segmentation)
    win = "SAM 3 microwave placement (q quits)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    last = None
    n = 0
    while True:
        t0 = time.time()
        try:
            fr = src.get()
        except AcquisitionError as e:
            print(f"camera: {e}")
            time.sleep(0.5)
            continue
        cm = cav = res = None
        try:
            cm = seg.segment(fr)
            cloud = ft.cavity_cloud(fr, cm.mask, cfg.cloud)
            cav, _, _, _ = ft.estimate_floor([cloud], [fr.cam_pos], cfg.cavity, 1)
            res = ft.placement_target(cav, cloud.points, cfg.container, cfg.target, cfg.cavity)
            reach = float(np.linalg.norm(res.container_center))
            status = (f"OK  SAM3 {cm.score:.2f}  cavity {cav.depth * 100:.0f}x{cav.width * 100:.0f}x{cav.height * 100:.0f} cm  "
                      f"centre reach {reach:.2f} m" + ("  OUT OF REACH (>0.91)" if reach > 0.91 else ""))
            ok = True
        except (SegmentationError, ft.PerceptionError) as e:
            status, ok = f"REFUSED: {e}"[:150], False
        img = viz.overlay(fr, cm, cav, res, status, ok)
        if cm is not None:
            cloud_pts = cav.floor_points if cav is not None else np.zeros((0, 3))
            img = side_by_side(img, viz.top_down([fr.cam_pos], cloud_pts, cav, res, size=img.shape[0]))
        cv2.putText(img, f"{1.0 / max(1e-3, time.time() - t0):.1f} Hz", (img.shape[1] - 70, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        if args.scale != 1.0:
            img = cv2.resize(img, None, fx=args.scale, fy=args.scale)
        cv2.imshow(win, img)
        n += 1
        if res is not None:
            c = res.container_center
            if last is None or np.linalg.norm(c - last) > 0.01:
                print(f"[{n}] container centre {np.round(c, 3).tolist()} floor z {res.floor_z:.3f} "
                      f"yaw {res.yaw_deg:+.1f}  reach {np.linalg.norm(c):.2f} m  ({res.frame}, m)")
                last = c.copy()
            (args.out / "latest_result.json").write_text(json.dumps({"ok": True, **res.to_dict()}, indent=1,
                                                                     default=float))
            (args.out / "latest_place_move.yaml").write_text(yaml.safe_dump(res.part1_config(), sort_keys=False))
            fr.save(args.out / "latest_frame.npz")
        elif n % 10 == 1:
            print(f"[{n}] {status}")
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
