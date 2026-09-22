"""Score button detectors against the hand-placed ground truth.

Two numbers matter, and they are deliberately not collapsed into one:

  correct%  -- picked the START/+30SEC button (within one button radius of
               the labeled center), or correctly ABSTAINED on a frame where
               the button is not resolvable.
  wrong%    -- returned a confident pick that is NOT the +30SEC button.
               This includes any pick at all on a not-resolvable frame.

wrong% is the number that must go to zero. It is the one a threshold-
loosening "optimization" cannot improve by accident: relaxing a gate to
catch more frames converts MISS into CORRECT *and* into WRONG, so a change
that only trades coverage for confidence shows up here immediately.

Tolerance is one button radius, not a fixed pixel count: button spacing on
this panel is ~2.3 radii, so a larger tolerance would start crediting a
pick that actually landed on the neighbouring STOP/ECO button.
"""
import argparse
import json
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parent))
# The detector itself lives in the repo (single source of truth); make this
# script runnable on its own rather than only under PYTHONPATH=src.
sys.path.insert(0, str(ROOT.parents[2] / "src"))


def tolerance(label):
    return max(float(label["radius"]), 6.0)


def score_one(label, pick):
    """-> (outcome, distance_or_None). Outcome EXCLUDED means the frame has no
    trustworthy ground truth and must not count toward any rate."""
    status = label.get("status", "labeled")
    if status == "unverifiable":
        return "EXCLUDED", None
    if status == "abstain_expected":
        return ("CORRECT_ABSTAIN", None) if pick is None else ("WRONG_SHOULD_ABSTAIN", None)
    if pick is None:
        return "MISS", None
    gx, gy = label["center"]
    d = ((pick[0] - gx) ** 2 + (pick[1] - gy) ** 2) ** 0.5
    if d <= tolerance(label):
        return "CORRECT", d
    # Outside the button. Distinguish "would press a DIFFERENT button" from
    # "near the right button but not on it" -- only the former is a safety
    # failure. A pick is still unambiguously aimed at the target while it is
    # nearer the target than the neighbouring button is.
    # "Near the right button" has to mean NEAR. Requiring only that the pick be
    # nearer the target than the neighbour is not enough: a pick hundreds of px
    # away, off the panel entirely, still satisfies that by accident. Bound it by
    # the inter-button spacing as well, so IMPRECISE means "inside the panel
    # neighbourhood of the right button" and nothing further out can qualify.
    nb = label.get("neighbor_xy")
    if nb is not None:
        dn = ((pick[0] - nb[0]) ** 2 + (pick[1] - nb[1]) ** 2) ** 0.5
        spacing = ((label["center"][0] - nb[0]) ** 2 + (label["center"][1] - nb[1]) ** 2) ** 0.5
        if d < dn and d <= spacing:
            return "IMPRECISE", d
    return "WRONG", d


def run_homography(frames, labels):
    from feeding_deployment.perception.appliance_perception.reference_button_detector import ReferenceButtonDetector
    det = ReferenceButtonDetector(ROOT / "reference")
    out = {}
    for stem in labels:
        img = cv2.imread(str(frames / f"{stem}.png"))
        if img is None:
            continue
        r = det.detect(img)
        out[stem] = (r["center"], f"inliers={r.get('inliers',0)} {r.get('reason','')}".strip())
    return out


def run_hough(frames, labels):
    import importlib
    m = importlib.import_module("render_button_detection_overlay")
    out = {}
    for stem in labels:
        img = cv2.imread(str(frames / f"{stem}.png"))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cand, band = m.detect_circles_multiscale(gray)
        if cand is None:
            out[stem] = (None, "no plausible band")
        else:
            c = m.pick_start_button(cand)
            out[stem] = ((float(c["center"][0]), float(c["center"][1])),
                         f"band {band[0]}-{band[1]}px r={c['r']}")
    return out


DETECTORS = {"homography": run_homography, "hough": run_hough}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--detector", choices=list(DETECTORS) + ["all"], default="all")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    data = json.loads((ROOT / "labels.json").read_text())
    labels = data["labels"]
    frames = ROOT / "frames"
    manifest = {m["id"]: m for m in json.loads((ROOT / "manifest.json").read_text())}
    split_of = {k: manifest.get(k, {}).get("split", "?") for k in labels}

    names = list(DETECTORS) if args.detector == "all" else [args.detector]
    summary = {}
    for name in names:
        picks = DETECTORS[name](frames, labels)
        counts = {}
        rows = []
        for stem, lab in sorted(labels.items()):
            if stem not in picks:
                continue
            pick, note = picks[stem]
            outcome, d = score_one(lab, pick)
            counts[outcome] = counts.get(outcome, 0) + 1
            rows.append((stem, outcome, d, pick, note))

        excluded = counts.pop("EXCLUDED", 0)
        n = sum(counts.values())
        correct = counts.get("CORRECT", 0) + counts.get("CORRECT_ABSTAIN", 0)
        wrong = counts.get("WRONG", 0) + counts.get("WRONG_SHOULD_ABSTAIN", 0)
        imprecise = counts.get("IMPRECISE", 0)
        summary[name] = (correct, wrong, imprecise, counts.get("MISS", 0), n)

        print(f"\n=== {name} ===")
        for stem, outcome, d, pick, note in rows:
            tag = f"f{stem.split('_f')[-1]}"
            tag = f"{tag}[{split_of.get(stem,'?')[0].upper()}]"
            ds = f" d={d:6.1f}px" if d is not None else " " * 12
            ps = f" pick=({pick[0]:6.1f},{pick[1]:6.1f})" if pick else " pick=None          "
            print(f"  {tag:>7} {outcome:<22}{ds}{ps}  {note}")
        print(f"  --> ALL      correct {correct}/{n}  WRONG {wrong}  imprecise {imprecise}  "
              f"miss {counts.get('MISS',0)}  ({excluded} excluded)")
        for sp in ("tune", "holdout", "fresh"):
            sub = [r for r in rows if split_of.get(r[0]) == sp and r[1] != "EXCLUDED"]
            if not sub:
                continue
            c = sum(1 for r in sub if r[1].startswith("CORRECT"))
            w = sum(1 for r in sub if r[1].startswith("WRONG"))
            i2 = sum(1 for r in sub if r[1] == "IMPRECISE")
            m2 = sum(1 for r in sub if r[1] == "MISS")
            print(f"      {sp:<8} correct {c}/{len(sub)}  WRONG {w}  imprecise {i2}  miss {m2}")

    if len(summary) > 1:
        print("\n=== comparison ===")
        print(f"  {'detector':<12}{'correct':>9}{'WRONG':>8}{'imprec':>8}{'miss':>7}")
        for name, (c, w, i, m, n) in summary.items():
            print(f"  {name:<12}{c:>4}/{n:<4}{w:>8}{i:>8}{m:>7}")


if __name__ == "__main__":
    main()
