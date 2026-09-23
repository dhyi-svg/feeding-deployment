"""Add a VIEW to an existing multi-view button reference, from the live camera.

Use this when the detector starts abstaining after the camera/arm moves to a
new framing. The red Comfee panel is nearly textureless, so one reference view
only tolerates a small shift: on 2026-09-21 a ~2x closer framing dropped the
2-view reference to 1/12 locks (all at the 6-inlier floor, one pick ~18px off
the dome); adding a view built from that framing gave 12/12 locks at 60-70
inliers, sub-pixel. Adding a view widens what can be FOUND without changing
what is ACCEPTED (every candidate still clears the same guards), so it is the
right response -- do not retune MIN_INLIERS / the ratio cascade instead.

This is `make_reference.py` for the multi-view case: that script writes a
single-view reference.json and would OVERWRITE the existing views. This one
appends, and it refuses to install a view that does not validate.

What it does, in order (nothing here can move the arm):

  1. grabs N frames from the colour topic (frame 0 becomes the view's image;
     warns if soft),
  2. locates the button on every frame with a highlight-independent fit:
     Otsu-threshold the saturation channel in a window around --near (chrome
     dome = unsaturated, red panel = saturated), take the blob nearest the
     window centre, and average its min-enclosing-circle centre with its
     unweighted mask centroid. The saturation-WEIGHTED centroid used for the
     first two views was ~3px biased toward the specular highlight once the
     dome was ~12px; the two fits here agreed to 0.4px across 12 frames.
     Refuses if they disagree by more than --max-fit-disagree or the
     per-frame scatter is large,
  3. runs the EXISTING reference on the same frames (baseline lock rate),
  4. writes the new view into a scratch copy, grabs a SECOND batch of frames
     and validates on those (not the ones the view was fitted on), and checks
     every existing view still answers its own reference image within 0.5px,
  5. only then installs: copies the image as reference_frame<k>.png, backs up
     reference.json, appends the view, writes reference_check<k>.png.

The running detector_node loads the reference at startup -- restart it
afterwards.

Usage (camera up; NEVER `PYTHONPATH=src` under ROS 2 -- prepend, or rclpy is
lost):
    PYTHONPATH=$PWD/src:$PYTHONPATH python3 scripts/button_press/add_reference_view.py \
        --ref ~/wrist_ref_red --near 575 266 --panel 486 168 640 322 \
        --note "camera moved closer, panel ~x486-640"

    --near   roughly the START/+30SEC dome centre in the CURRENT framing (read
             it off rqt_image_view of /button_detector/debug_image or a saved
             frame). Only needs to be inside the dome; the fit places it.
    --panel  box around the whole 5-button panel INCLUDING the printed labels
             (the text carries most of the SIFT features on this panel).
    --also NAME X Y   (repeatable) rough centre of another named button in the current
             framing, when the existing views carry a 'buttons' table. Not normally
             needed: every other named button is located automatically by projecting the
             existing reference through the fitted homography and re-fitting the dome
             there; --also is the fallback for a name the projection cannot place.
    --dry-run  do everything except install.

Named buttons: when the existing views carry a ``buttons`` table (e.g. ``start_30s`` and
``timer_clock``), the new view gets the same table -- the primary --near dome is the
reference's ``default_button`` (or ``start_30s``), the others are projected + re-fitted.
The detector refuses a target that some view does not mark, so a view without the table
would silently break ``-p target_button:=timer_clock`` -- hence this step.
"""
import argparse
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np


def load_detector_class():
    here = Path(__file__).resolve()
    for cand in (here.parents[2] / "src",):  # scripts/button_press/<this> -> <repo>/src
        if (cand / "feeding_deployment").exists():
            sys.path.insert(0, str(cand))
            break
    from feeding_deployment.perception.appliance_perception.reference_button_detector import (
        ReferenceButtonDetector,
    )
    return ReferenceButtonDetector


def grab_frames(topic, n, timeout=15.0):
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image

    if not rclpy.ok():
        rclpy.init()
    node = Node("button_ref_add_view")
    bridge = CvBridge()
    frames = []
    node.create_subscription(
        Image, topic, lambda m: frames.append(bridge.imgmsg_to_cv2(m, "bgr8")), 10)
    t0 = time.monotonic()
    while len(frames) < n and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    if len(frames) < n:
        sys.exit(f"only {len(frames)}/{n} frames on {topic} in {timeout}s -- is the camera up?")
    return frames


def fit_button(img, near, win):
    """Highlight-independent dome centre: (rim-circle centre, mask centroid, radius)."""
    nx, ny = near
    x0, y0 = max(0, nx - win), max(0, ny - win)
    x1, y1 = min(img.shape[1], nx + win), min(img.shape[0], ny + win)
    sat = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)[..., 1]
    _, mask = cv2.threshold(sat, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return None
    centre = np.array([nx - x0, ny - y0], dtype=np.float32)
    c = min(cnts, key=lambda c: np.linalg.norm(np.array(cv2.minEnclosingCircle(c)[0]) - centre))
    (ex, ey), er = cv2.minEnclosingCircle(c)
    m = cv2.moments(c)
    if m["m00"] <= 0:
        return None
    return (np.array([x0 + ex, y0 + ey]),
            np.array([x0 + m["m10"] / m["m00"], y0 + m["m01"] / m["m00"]]),
            float(er))


def run_detector(det, frames, label, gt=None):
    locks, errs = 0, []
    for i, f in enumerate(frames):
        r = det.detect(f)
        if r["center"] is None:
            print(f"    f{i:02d} abstain inl={r['inliers']} {r.get('reason', '')}")
            continue
        locks += 1
        e = float(np.linalg.norm(np.array(r["center"]) - gt)) if gt is not None else float("nan")
        errs.append(e)
        print(f"    f{i:02d} LOCK view={r.get('view')} inl={r['inliers']:2d} "
              f"center=({r['center'][0]:.1f},{r['center'][1]:.1f}) err={e:.1f}px")
    worst = max(errs) if errs else float("nan")
    print(f"  {label}: {locks}/{len(frames)} locks, worst err {worst:.1f}px")
    return locks, worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True, help="reference dir holding reference.json")
    ap.add_argument("--near", nargs=2, type=int, required=True, metavar=("X", "Y"))
    ap.add_argument("--panel", nargs=4, type=int, required=True, metavar=("X0", "Y0", "X1", "Y1"))
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--frames", type=int, default=12, help="frames per batch (fit and validate)")
    ap.add_argument("--window", type=int, default=20, help="half-size of the fit window (px)")
    ap.add_argument("--max-fit-disagree", type=float, default=2.0,
                    help="max px between rim-circle and mask-centroid fits")
    ap.add_argument("--min-lock-frac", type=float, default=0.9,
                    help="validation lock rate below which the view is NOT installed")
    ap.add_argument("--max-val-err", type=float, default=3.0,
                    help="worst validation error (px) above which the view is NOT installed")
    ap.add_argument("--note", default="")
    ap.add_argument("--also", nargs=3, action="append", default=[], metavar=("NAME", "X", "Y"),
                    help="rough centre of another named button (fallback when projection fails)")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    ref_dir = Path(args.ref)
    ref_json = ref_dir / "reference.json"
    if not ref_json.exists():
        sys.exit(f"{ref_json} not found -- build the first view with make_reference.py")
    meta = json.loads(ref_json.read_text())
    if "views" not in meta:
        # Single-view form -> promote to the multi-view form, keeping it as view 1.
        keys = ("image", "crop", "button_xy", "button_radius")
        meta["views"] = [{k: meta[k] for k in keys if k in meta}]
        for k in keys:
            meta.pop(k, None)
    k = len(meta["views"]) + 1
    print(f"reference {ref_dir}: {k - 1} existing view(s); adding view {k}")

    # 1. frames to fit on ------------------------------------------------------
    print(f"\n[1] grabbing {args.frames} frames from {args.topic} ...")
    fit_frames = grab_frames(args.topic, args.frames)
    ref_img = fit_frames[0]
    sharp = cv2.Laplacian(cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    print(f"    frame 0 sharpness {sharp:.0f}" + ("  WARNING: soft -- steady the camera and rerun"
                                                    if sharp < 100 else ""))

    # 2. button fit ------------------------------------------------------------
    print(f"\n[2] fitting the dome near {tuple(args.near)} on every frame ...")
    fits = [fit_button(f, args.near, args.window) for f in fit_frames]
    fits = [f for f in fits if f is not None]
    if len(fits) < max(3, args.frames // 2):
        sys.exit("dome fit failed on most frames -- is --near inside the chrome dome?")
    rim = np.mean([f[0] for f in fits], axis=0)
    cen = np.mean([f[1] for f in fits], axis=0)
    rad = float(np.mean([f[2] for f in fits]))
    scatter = np.std([f[0] for f in fits], axis=0)
    disagree = float(np.linalg.norm(rim - cen))
    button_xy = (rim + cen) / 2
    print(f"    rim-circle    ({rim[0]:.1f},{rim[1]:.1f}) r={rad:.1f}  scatter=({scatter[0]:.2f},{scatter[1]:.2f})px")
    print(f"    mask-centroid ({cen[0]:.1f},{cen[1]:.1f})  disagreement {disagree:.1f}px")
    print(f"    button_xy = ({button_xy[0]:.1f},{button_xy[1]:.1f})  moved "
          f"{np.linalg.norm(button_xy - args.near):.1f}px from --near")
    if disagree > args.max_fit_disagree or scatter.max() > 1.5:
        sys.exit("the two fits disagree / scatter too much -- look at the dome by eye; "
                 "a highlight or a neighbouring dome is probably inside the window")
    # minEnclosingCircle includes the dark rim shadow; the visible dome is a bit smaller.
    button_radius = int(round(rad * 0.85))

    # 3. baseline --------------------------------------------------------------
    Detector = load_detector_class()
    print("\n[3] existing reference on these frames:")
    run_detector(Detector(ref_dir), fit_frames, "baseline", gt=button_xy)

    # 3b. other named buttons ----------------------------------------------------
    # Union of the names the existing views mark; the primary (--near) dome is the
    # reference's default. Each other dome is placed by projecting the existing
    # reference (detector run with that target) onto frame 0, then re-fitted with the
    # same highlight-independent fit, so the stored centre is measured, not projected.
    names = set()
    for v in meta["views"]:
        names.update((v.get("buttons") or {}).keys())
    primary = meta.get("default_button") or "start_30s"
    buttons = {}
    if names:
        buttons[primary] = [round(float(button_xy[0]), 1), round(float(button_xy[1]), 1)]
        also = {n: (int(x), int(y)) for n, x, y in args.also}
        print(f"\n[3b] locating the other named buttons {sorted(names - {primary})} ...")
        for name in sorted(names - {primary}):
            rough = also.get(name)
            if rough is None:
                r = Detector(ref_dir, target=name).detect(ref_img)
                # A 6-8 inlier lock projects ~50 px off (one dome pitch) often enough
                # that on 2026-09-21 it landed on label text and the fit "found" a 1.4 px
                # blob there. Only trust a projection from a solid lock.
                if r.get("center") is None or r.get("inliers", 0) < 15:
                    sys.exit(f"cannot place button {name!r}: existing reference lock on frame 0 is "
                             f"too weak ({r.get('inliers', 0)} inliers, {r.get('reason', 'ok')}). "
                             f"Pass --also {name} X Y (read it off the debug image).")
                rough = (int(round(r["center"][0])), int(round(r["center"][1])))
                print(f"    {name}: projected to {rough} (inliers={r.get('inliers')})")
            f_all = [fit_button(f, rough, args.window) for f in fit_frames]
            f_all = [f for f in f_all if f is not None]
            if len(f_all) < max(3, args.frames // 2):
                sys.exit(f"dome fit failed for {name!r} near {rough}")
            rim_n = np.mean([f[0] for f in f_all], axis=0)
            cen_n = np.mean([f[1] for f in f_all], axis=0)
            dis_n = float(np.linalg.norm(rim_n - cen_n))
            xy_n = (rim_n + cen_n) / 2
            print(f"    {name}: fit ({xy_n[0]:.1f},{xy_n[1]:.1f}) r={np.mean([f[2] for f in f_all]):.1f}"
                  f"  disagreement {dis_n:.1f}px  moved {np.linalg.norm(xy_n - rough):.1f}px from the projection")
            rad_n = float(np.mean([f[2] for f in f_all]))
            if dis_n > args.max_fit_disagree:
                sys.exit(f"{name!r}: rim/centroid fits disagree by {dis_n:.1f}px -- check the window by eye")
            # All five domes are the same size: a blob a different size is not a dome.
            if not 0.7 * rad <= rad_n <= 1.3 * rad:
                sys.exit(f"{name!r}: fitted radius {rad_n:.1f}px vs primary dome {rad:.1f}px -- "
                         f"the window near {rough} is not on a dome. Pass --also {name} X Y.")
            buttons[name] = [round(float(xy_n[0]), 1), round(float(xy_n[1]), 1)]

    # 4. scratch copy + validation ---------------------------------------------
    tmp = Path(tempfile.mkdtemp(prefix="button_ref_view_"))
    shutil.copytree(ref_dir, tmp, dirs_exist_ok=True)
    img_name = f"reference_frame{k}.png"
    cv2.imwrite(str(tmp / img_name), ref_img)
    new_view = {
        "image": img_name,
        "crop": [int(v) for v in args.panel],
        "button_xy": [round(float(button_xy[0]), 1), round(float(button_xy[1]), 1)],
        "button_radius": button_radius,
        **({"buttons": buttons} if buttons else {}),
        "note": (f"view {k}, {time.strftime('%Y-%m-%d %H:%M')} framing"
                 + (f": {args.note}" if args.note else "")
                 + "; button_xy = mean of Otsu-saturation rim-circle and mask-centroid fits "
                   f"over {len(fits)} frames (add_reference_view.py)"),
    }
    meta_new = json.loads(json.dumps(meta))
    meta_new["views"].append(new_view)
    (tmp / "reference.json").write_text(json.dumps(meta_new, indent=2))
    det_new = Detector(tmp)

    print(f"\n[4] grabbing a fresh batch of {args.frames} frames to validate on ...")
    val_frames = grab_frames(args.topic, args.frames)
    print("    new reference on fresh frames:")
    locks, worst = run_detector(det_new, val_frames, "validation", gt=button_xy)

    print("    regression -- each existing view on its own image:")
    regress_ok = True
    for vi, v in enumerate(meta["views"]):
        img = cv2.imread(str(ref_dir / v["image"]))
        r = det_new.detect(img)
        want = np.array(v["button_xy"], dtype=float)
        err = float(np.linalg.norm(np.array(r["center"]) - want)) if r["center"] else float("inf")
        ok = r["center"] is not None and r.get("view") == vi and err < 0.5
        regress_ok &= ok
        print(f"    view {vi + 1} {v['image']}: {'ok ' if ok else 'BAD'} "
              f"picked view={r.get('view')} inl={r['inliers']} err={err:.2f}px")

    lock_frac = locks / len(val_frames)
    good = lock_frac >= args.min_lock_frac and worst <= args.max_val_err and regress_ok
    print(f"\nlock rate {lock_frac:.0%} (need >= {args.min_lock_frac:.0%}), worst err {worst:.1f}px "
          f"(need <= {args.max_val_err}), regression {'ok' if regress_ok else 'FAILED'}")
    if not good:
        sys.exit(f"NOT installed. Scratch copy left at {tmp} for inspection.")
    if args.dry_run:
        print(f"--dry-run: not installed. Scratch copy at {tmp}")
        return

    # 5. install ---------------------------------------------------------------
    stamp = time.strftime("%Y%m%d-%H%M%S")
    backup = ref_dir / f"reference.json.bak-{k - 1}views-{stamp}"
    shutil.copy(ref_json, backup)
    shutil.copy(tmp / img_name, ref_dir / img_name)
    ref_json.write_text(json.dumps(meta_new, indent=2))
    vis = ref_img.copy()
    x0, y0, x1, y1 = args.panel
    cv2.rectangle(vis, (x0, y0), (x1, y1), (0, 255, 0), 2)
    p = (int(round(button_xy[0])), int(round(button_xy[1])))
    cv2.circle(vis, p, button_radius, (0, 0, 255), 2)
    cv2.drawMarker(vis, p, (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    check = ref_dir / f"reference_check{k}.png"
    cv2.imwrite(str(check), vis)
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"\ninstalled view {k} -> {ref_json}  (backup: {backup.name})")
    print(f"LOOK AT {check}: red marker on START/+30SEC, green box around the panel + labels.")
    print("Restart the detector node (feeding_deployment.button_press.detector_node) -- it only reads the reference at startup.")


if __name__ == "__main__":
    main()
