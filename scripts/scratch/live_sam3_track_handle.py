"""Live camera window that TRACKS the handle with SAM 3's video model.

Difference from live_sam3_handle_view.py, which used the image model: that one
re-detects from scratch on every frame, so each result is independent, there is
no notion of "the same handle" over time, and at ~8 s per CPU detection the
outline is always badly stale while the camera moves.

Here a Sam3VideoModel streaming session is opened ONCE, the text prompt is added
ONCE, and thereafter each frame is *propagated* through the session's memory.
That gives:

  * persistent object_id per tracked instance, so the grasp target cannot
    silently swap between two candidates between frames
  * cheaper per-frame cost than full detection
  * re-acquisition after brief occlusion, which is the arm's own gripper
    crossing the view during an approach
  * automatic non-overlapping masks -- the video processor applies
    object-wise non-overlap constraints, which removes the nested
    whole-handle vs grip-channel duplicate the image model returned

Frames are still fed from a worker thread so the display never blocks. Unlike
the image version the worker matters MORE here, because the session is
sequential: every frame it manages to push improves the track.

    python3 scripts/scratch/live_sam3_track_handle.py
    python3 scripts/scratch/live_sam3_track_handle.py --prompt "door handle"

Keys:  q / ESC quit      s  save frame      SPACE pause      r  restart track
"""
import argparse
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import torch

ap = argparse.ArgumentParser(
    description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--prompt", default="handle")
ap.add_argument("--repo", default="MTerryJack/sam3",
                help="facebook/sam3 is gated; default is an ungated mirror")
ap.add_argument("--save-dir", default="/tmp/sam3_track")
ap.add_argument("--dtype", default="bf16", choices=["fp32","fp16","bf16"],
                help="inference precision. bf16 runs ~2x faster than fp32 on the "
                     "4070 Ti's tensor cores with no range issues (default bf16).")
ap.add_argument("--reset-every", type=int, default=150,
                help="re-seed the tracking session every N frames. The session's "
                     "memory bank grows without bound otherwise -- observed "
                     "3.5 GB -> 11.5 GB GPU in ~10 min on 2026-09-20, which then "
                     "starved every other CUDA process. ~150 frames is ~30 s at "
                     "the bf16 rate. 0 disables (not recommended).")
ap.add_argument("--autosave", type=float, default=0.0,
                help="also write the overlay to <save-dir>/latest.png every N seconds. "
                     "Lets the window be inspected without needing keyboard focus.")
args = ap.parse_args()

from transformers import Sam3VideoModel, Sam3VideoProcessor  # noqa: E402

from feeding_deployment.ros2.realsense_ros2_interface import (  # noqa: E402
    RealSenseROS2Interface,
)

print(f"loading SAM 3 video model from {args.repo} ...")
t0 = time.time()
# safetensors only -- the bundled .pt is a pickle from an unvetted uploader.
DTYPE = {"fp32": torch.float32, "fp16": torch.float16, "bf16": torch.bfloat16}[args.dtype]
dev = "cuda" if torch.cuda.is_available() else "cpu"
if dev == "cpu" and DTYPE is not torch.float32:
    print("CPU has no fast half-precision path; forcing fp32")
    DTYPE = torch.float32
model = Sam3VideoModel.from_pretrained(
    args.repo, use_safetensors=True, dtype=DTYPE).eval()
processor = Sam3VideoProcessor.from_pretrained(args.repo)
model.to(dev)
print(f"loaded in {time.time()-t0:.1f}s on {dev.upper()} ({args.dtype})")

rs = RealSenseROS2Interface()
if not rs.wait_for_frames(30.0):
    raise SystemExit("No RGB-D frames -- is realsense2_camera up?")

_lock = threading.Lock()
_latest = None            # newest BGR frame for the worker
_result = None            # {"masks","boxes","ids","scores","t","dur"}
_stop = threading.Event()
_restart = threading.Event()
_printed_keys = False


def worker():
    """Own the streaming session and keep pushing the newest frame through it."""
    global _result, _printed_keys
    session = None
    n = 0
    while not _stop.is_set():
        if session is None or _restart.is_set():
            _restart.clear()
            session = processor.init_video_session(
                inference_device=dev, dtype=DTYPE)
            processor.add_text_prompt(session, args.prompt)
            n = 0
            print(f"[track] session started, prompt={args.prompt!r}")

        with _lock:
            frame = None if _latest is None else _latest.copy()
        if frame is None:
            time.sleep(0.05)
            continue

        rgb = np.ascontiguousarray(frame[:, :, ::-1])
        try:
            enc = processor(images=rgb, return_tensors="pt")
            pix = enc["pixel_values"][0].to(dev, dtype=DTYPE)
            t = time.time()
            with torch.no_grad():
                out = model(inference_session=session, frame=pix)
            dur = time.time() - t
            res = processor.postprocess_outputs(
                session, out, original_sizes=enc["original_sizes"])
        except Exception as e:
            print(f"[track] frame failed ({type(e).__name__}): {e}")
            session = None          # rebuild the session rather than wedge
            time.sleep(0.5)
            continue

        if not _printed_keys:
            print(f"[track] output keys: {list(res.keys())}")
            _printed_keys = True

        def grab(*names):
            for nm in names:
                if nm in res:
                    v = res[nm]
                    return v.cpu().numpy() if hasattr(v, "cpu") else np.asarray(v)
            return None

        masks = grab("masks", "binary_masks", "pred_masks")
        ids = grab("object_ids", "obj_ids")
        scores = grab("scores", "probs", "object_scores")
        boxes = grab("boxes", "boxes_xyxy")
        n += 1
        with _lock:
            _result = {"masks": masks, "ids": ids, "scores": scores,
                       "boxes": boxes, "t": time.time(), "dur": dur, "n": n}
        if args.reset_every > 0 and n >= args.reset_every:
            # Drop the whole session so its per-frame memory is freed. The
            # track re-seeds on the next frame; the object id may change.
            session = None
            torch.cuda.empty_cache() if dev == "cuda" else None
            print(f"[track] periodic reset after {n} frames")


threading.Thread(target=worker, daemon=True).start()

# Stable colour per object_id, so a tracked handle keeps its colour for as long
# as the track survives -- a colour change means the track was lost and re-made.
PALETTE = [(0, 255, 0), (0, 165, 255), (255, 128, 0), (255, 0, 255), (0, 255, 255)]
save_dir = Path(args.save_dir)
win = f"SAM3 TRACK -- '{args.prompt}'"
cv2.namedWindow(win, cv2.WINDOW_NORMAL)
cv2.resizeWindow(win, 960, 720)
paused = False
n_saved = 0
_last_autosave = 0.0
print("\nwindow open. q/ESC quit, s save, SPACE pause, r restart track.\n")

while True:
    if not paused:
        d = rs.get_camera_data()
        frame, depth = d["rgb_image"], d["depth_image"]
        if frame is None:
            time.sleep(0.03)
            continue
        with _lock:
            _latest = frame.copy()

    vis = frame.copy()
    with _lock:
        res = None if _result is None else dict(_result)

    n_found = 0
    if res is not None and res["masks"] is not None:
        masks = res["masks"]
        for i in range(len(masks)):
            m = np.asarray(masks[i]).astype(bool)
            if m.ndim == 3:
                m = m[0]
            if m.shape[:2] != vis.shape[:2] or not m.any():
                continue
            n_found += 1
            oid = int(res["ids"][i]) if res["ids"] is not None and i < len(res["ids"]) else i
            colour = PALETTE[oid % len(PALETTE)]
            contours, _ = cv2.findContours(m.astype(np.uint8), cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(vis, contours, -1, colour, 2)
            ys, xs = np.where(m)
            x1, y1, x2, y2 = xs.min(), ys.min(), xs.max(), ys.max()
            cv2.rectangle(vis, (x1, y1), (x2, y2), colour, 1)
            label = f"id{oid}"
            if res["scores"] is not None and i < len(res["scores"]):
                label += f" {float(res['scores'][i]):.2f}"
            if depth is not None and depth.shape[:2] == m.shape[:2]:
                dv = depth[m]
                dv = dv[dv > 0]
                if dv.size:
                    label += f" {np.median(dv)/1000:.3f}m"
            cv2.putText(vis, label, (int(x1), max(16, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)

    if res:
        hud = [f"TRACKING '{args.prompt}'   {n_found} object(s)",
               f"propagate {res['dur']:.2f}s   age {time.time()-res['t']:.1f}s   "
               f"frame #{res['n']}",
               f"{dev.upper()} {args.dtype}   {'PAUSED' if paused else 'live'}"]
    else:
        hud = ["starting track...", "", dev.upper()]
    for k, line in enumerate(hud):
        cv2.putText(vis, line, (8, 22 + 22 * k), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3)
        cv2.putText(vis, line, (8, 22 + 22 * k), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1)

    if args.autosave > 0 and time.time() - _last_autosave >= args.autosave:
        save_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(save_dir / "latest.png"), vis)
        _last_autosave = time.time()

    cv2.imshow(win, vis)
    k = cv2.waitKey(30) & 0xFF
    if k in (ord("q"), 27):
        break
    if k == ord(" "):
        paused = not paused
    if k == ord("r"):
        _restart.set()
        print("restarting track...")
    if k == ord("s"):
        save_dir.mkdir(parents=True, exist_ok=True)
        p = save_dir / f"sam3_track_{time.strftime('%H%M%S')}.png"
        cv2.imwrite(str(p), vis)
        n_saved += 1
        print(f"saved {p}")

_stop.set()
cv2.destroyAllWindows()
print(f"\ndone. {n_saved} frame(s) saved to {save_dir}")
