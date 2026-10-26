"""Part 2 standalone test: where in the open microwave the held container goes. NEVER moves the robot.

    # live (sheppy profile with the camera + arm tf + calibration_tf up), 3 agreeing looks:
    source microwave/sheppy/env.sh
    python3 -u microwave/placement/perceive_placement.py --live --frames 3 [--show]
    # recorded frames (any of rgbd_source.load_recorded's formats), one look = not cross-checked:
    python3 -u microwave/placement/perceive_placement.py --recorded tests/data/microwave_lookin_2026-10-20.npz
    # skip SAM 3 with a saved mask (e.g. on a machine without the GPU model):
    python3 -u microwave/placement/perceive_placement.py --recorded <f.npz> --mask <f_mask.npz>
Run from the repo root.

Steps: acquire frame(s) -> SAM 3 cavity mask -> valid-depth cavity cloud (arm_base_link) -> floor and
cavity (planes, gravity-sorted) -> agreement across frames -> usable floor -> placement target. Prints
the floor contact, container-centre and tool-centre targets with their frame and units, every check,
and every warning; refusals say why (exit 2). Writes to --out (default ~/microwave_place_logs/perception_<t>/):
  overlay_<i>.jpg     camera view with mask, floor points, cavity box, usable floor, footprint, targets
  top_down.jpg        the same from above
  mask_<i>.npz        the SAM 3 mask (reusable with --mask)
  frame_<i>.npz       live frames (replay with --recorded)
  result.json         everything, machine-readable
  place_move.yaml     Part 1 input: python3 -u microwave/placement/place_move.py --config <out>/place_move.yaml
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))         # microwave/: xyz_move (Part 1)
import yaml  # noqa: E402

import floor_target as ft  # noqa: E402
import perception_viz as viz  # noqa: E402
from cavity_segmentation import CavitySegmenter, SegmentationError  # noqa: E402
from rgbd_source import AcquisitionError, RecordedSource, RosRGBDSource  # noqa: E402


def save_mask(path, cm):
    np.savez_compressed(path, mask=cm.mask, sam_mask=cm.sam_mask, score=cm.score, k_upright=cm.k_upright,
                        prompt=cm.prompt)


def load_mask(path, frame, seg_cfg):
    """A saved mask (save_mask, or just `mask` = SAM 3's raw mask) -> CavityMask, re-filtered with seg_cfg."""
    d = np.load(path, allow_pickle=False)
    raw = d["sam_mask"] if "sam_mask" in d else d["mask"]

    class Saved:
        def segment(self, rgb):
            return np.rot90(raw, k) if k else raw, float(d["score"]) if "score" in d else 1.0
    k = 0
    if seg_cfg.upright == "gravity":
        from point_cloud import upright_k
        k = upright_k(frame.base_T_cam)
    return CavitySegmenter(seg_cfg, model=Saved()).segment(frame)


def parse_args(argv=None):
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = a.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true", help="grab frames from the ROS 2 camera topics")
    src.add_argument("--recorded", nargs="+", type=Path, help="saved frame(s): .npz or capture frame dirs")
    a.add_argument("--mask", nargs="+", type=Path, help="saved mask .npz per recorded frame (skips SAM 3)")
    a.add_argument("--frames", type=int, default=3, help="live: frames to take (each one a look)")
    a.add_argument("--gap-s", type=float, default=0.5, help="live: seconds between frames")
    a.add_argument("--min-looks", type=int, help="looks that must agree (default: cavity.min_looks, but at most "
                                                 "the number of frames given)")
    a.add_argument("--config", type=Path, help=f"YAML overrides (default {ft.DEFAULT_CONFIG_FILE.name} if present)")
    a.add_argument("--grasp-offset", type=float, nargs=3, metavar=("X", "Y", "Z"),
                   help="tool frame -> container centre in the TOOL frame (m), MEASURED; enables the tool target")
    a.add_argument("--out", type=Path, help="output dir (default ~/microwave_place_logs/perception_<time>)")
    a.add_argument("--show", action="store_true", help="also open the debug images in a window")
    args = a.parse_args(argv)
    if args.mask and (not args.recorded or len(args.mask) != len(args.recorded)):
        a.error("--mask needs --recorded and one mask per recorded frame")
    return args


def main(argv=None):
    args = parse_args(argv)
    over = {"container": {"grasp_offset": list(args.grasp_offset)}} if args.grasp_offset else None
    cfg = ft.load_perception_config(args.config, over)
    out = args.out or Path.home() / "microwave_place_logs" / f"perception_{time.strftime('%Y%m%d_%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)
    print(f"output: {out}")

    try:
        if args.live:
            src = RosRGBDSource(cfg.camera)
            frames = []
            for i in range(args.frames):
                if i:
                    time.sleep(args.gap_s)
                frames.append(src.get())
                frames[-1].save(out / f"frame_{i + 1}.npz")
        else:
            frames = RecordedSource(args.recorded).frames
    except AcquisitionError as e:
        print(f"REFUSED (camera): {e}")
        return 2
    for i, fr in enumerate(frames):
        print(f"frame {i + 1}: {fr.source or 'live'}  {fr.bgr.shape[1]}x{fr.bgr.shape[0]}, camera frame "
              f"'{fr.frame_id}' at {np.round(fr.cam_pos, 3).tolist()} in {fr.base_frame}")

    masks = None
    seg = None
    try:
        if args.mask:
            masks = [load_mask(m, fr, cfg.segmentation) for m, fr in zip(args.mask, frames)]
        else:
            seg = CavitySegmenter(cfg.segmentation)
    except SegmentationError as e:
        print(f"REFUSED (segmentation): {e}")
        return 2
    min_looks = args.min_looks if args.min_looks is not None else min(cfg.cavity.min_looks, len(frames))

    res = cav = None
    status, ok = "", True
    try:
        res, obs, cav = ft.run_pipeline(frames, seg, cfg, min_looks=min_looks, masks=masks)
        status = "OK -- placement target computed (perception only; nothing moved)"
    except ft.PerceptionError as e:
        status, ok = f"REFUSED: {e}", False
        obs, cav = getattr(e, "observations", None) or [], getattr(e, "cavity", None)

    # images (also on a refusal: show what was seen)
    for i, fr in enumerate(frames):
        o = obs[i] if i < len(obs) else None
        cm = o.cmask if o is not None else (masks[i] if masks else None)
        if cm is not None:
            save_mask(out / f"mask_{i + 1}.npz", cm)
        img = viz.overlay(fr, cm, cav, res, status if i == 0 else (o.error if o is not None and o.error else ""), ok)
        import cv2
        cv2.imwrite(str(out / f"overlay_{i + 1}.jpg"), img)
    clouds = [o.cloud.points for o in obs if o is not None and o.cloud is not None]
    if clouds:
        import cv2
        cv2.imwrite(str(out / "top_down.jpg"), viz.top_down([fr.cam_pos for fr in frames], np.vstack(clouds), cav, res))

    print()
    if res is None:
        print(status)
        (out / "result.json").write_text(json.dumps({"ok": False, "reason": status}, indent=1))
        _show(out, args.show)
        return 2
    f = res.frame
    print(f"FRAME: {f} (= Part 1's base_link: +x forward, +z up, origin at the arm base). UNITS: metres, degrees.")
    print(f"  floor            z {res.floor_z:.4f} at the cavity centre, normal {np.round(res.floor_normal, 3).tolist()} "
          f"(tilt {res.floor_tilt_deg:.2f} deg), rms {res.floor_rms_m * 1000:.1f} mm")
    print(f"  support          {res.support_kind}, z {res.support_z:.4f}")
    print(f"  usable floor     along f [{res.usable_floor['front']:.3f}, {res.usable_floor['back']:.3f}], "
          f"across l [{res.usable_floor['right']:.3f}, {res.usable_floor['left']:.3f}]  "
          f"(f = insertion axis {np.round(res.insertion_axis, 3).tolist()})")
    print(f"  FLOOR CONTACT    {np.round(res.floor_contact, 4).tolist()}   support point under the container centre")
    print(f"  CONTAINER CENTRE {np.round(res.container_center, 4).tolist()}   Part 1 --target container --xyz")
    print(f"  TOOL CENTRE      {np.round(res.tool_center, 4).tolist() if res.tool_center is not None else 'n/a'}"
          f"   Part 1 tool_frame (from grasp_offset {res.grasp_offset})")
    print(f"  ORIENTATION      yaw {res.yaw_deg:+.2f} deg, roll {res.roll_deg:+.1f} deg (Part 1 --yaw-deg / --roll-deg)")
    print("checks:")
    for c in res.checks:
        print(f"  - {c}")
    print("warnings:" if res.warnings else "warnings: none")
    for w in res.warnings:
        print(f"  ! {w}")
    (out / "result.json").write_text(json.dumps({"ok": True, **res.to_dict()}, indent=1, default=float))
    (out / "place_move.yaml").write_text(
        "# Written by perceive_placement.py -- Part 1 input (python3 -u microwave/placement/place_move.py --config "
        "<this file>)\n" + yaml.safe_dump(res.part1_config(), sort_keys=False))
    print(f"\n{status}\nPart 1 input: {out / 'place_move.yaml'}" +
          ("" if res.grasp_offset is not None else "  (add the MEASURED grasp_offset before place_move can use it)"))
    _show(out, args.show)
    return 0


def _show(out, show):
    if not show:
        return
    import cv2
    for p in sorted(out.glob("overlay_*.jpg")) + [out / "top_down.jpg"]:
        if p.exists():
            cv2.imshow(p.name, cv2.imread(str(p)))
    print("(any key in an image window closes them)")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
