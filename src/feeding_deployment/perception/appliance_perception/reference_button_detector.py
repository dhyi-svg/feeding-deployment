"""Reference-homography detector for the microwave START/+30SEC button.

Measured on 21 hand-labeled frames of the Comfee rig (see
scripts/scratch/button_eval/FINDINGS.md for the harness and the full record):
16 correct, **0 wrong-picks**, 1 imprecise, 4 abstentions -- 7/11 correct with
zero wrong-picks on frames held out from tuning. The HoughCircles backend on
the same frames: 7 correct, 10 wrong. Those numbers are for ONE appliance in
ONE clip; nothing here establishes transfer to another microwave or another
lighting setup.

A note specific to this repo: unlike the GroundingDINO and HoughCircles
backends, this one needs no upside-down-camera correction. Those backends pick
the button by a "bottom row, rightmost" spatial rule, which only means anything
in a visually-upright frame, so they flip coordinates first. Here the button is
identified by a hand-placed mark carried through the fitted transform, and SIFT
matching plus a homography handle a 180-degree rotation natively.

Instead of finding circles and applying a "bottom row, rightmost" rule to
guess which one is START, this locates the *panel* by feature matching
against a reference image on which the +30SEC button was marked BY HAND,
once, then carries that mark through the estimated homography.

Why this shape:
  - Button identity is established by a human pointing at it on the
    reference, not re-derived per frame. The bottom-row/rightmost rule can
    and does mislabel when the circle detector misses one of the five (see
    the wrong-pick on the close-up frame where it found 3 buttons + a dial
    highlight and put START on bare panel).
  - The panel is a rigid planar target, so a homography is the physically
    correct model for how it reprojects under any viewpoint/scale. Nothing
    has to be tuned per working distance.
  - Confidence is the RANSAC inlier count, which cannot be inflated by
    loosening a threshold the way a stability/candidate-count gate can.

Cost: one reference per appliance model. A reference built on the cream
Comfee will not transfer to the red microwave.
"""
import json
from pathlib import Path

import cv2
import numpy as np

# A homography fitted from very few correspondences is numerically free to
# put the projected point anywhere; require a real consensus set.
# PROVISIONAL. Chosen from a ratio/inlier sweep over only SEVEN labeled
# frames, and chosen by looking at that same seven -- so this is fitted to
# the eval set and the 6/7 it scores there is optimistic. It sits mid-plateau
# (ratio >= 0.75 with min_inliers <= 8 all scored the same) rather than on an
# edge, which is mild evidence it is not a knife-edge fit, but it needs
# re-checking against a bigger label set and a held-out clip before it means
# anything.
#
# 6 sits mid-plateau (4, 5 and 6 all score identically; 7+ loses the most
# distant frame). Note what this implies: at 6 correspondences the inlier
# COUNT is not what is keeping the detector honest -- the geometric guards
# below (convex/area/aspect quad, resolvability) are. Lowering the floor from
# 8 to 6 raised recall with wrong-picks staying at zero, which is the evidence
# those fits were real; it is not licence to keep lowering it.
MIN_INLIERS = 6
# Lowe ratio test.
# No single Lowe ratio works across this footage's scale range. A tight ratio
# keeps correspondences clean enough to fit the tiny distant panel but starves
# the blurred close-up of matches; a loose one does the reverse -- measured
# directly: at 0.70 the most distant frame lands within 2.7px while the blurry
# close-up degenerates, and at 0.80 exactly the opposite. So try several, in
# order, and take the first fit that passes the geometric guards.
#
# This is a search, not a loosening: every candidate still has to clear the
# inlier floor, the convex/area/aspect quad checks and the resolvability
# check, so an extra attempt cannot admit a fit that a single-ratio run would
# have rejected. It only gives a good fit more chances to be found.
RATIO_CASCADE = (0.70, 0.75, 0.80, 0.85)
RATIO = 0.80

# Affine fallback.
#
# Every remaining failure on the eval set has ONE root cause: the fitted
# homography FOLDS (non-convex projected quad). Measured, not assumed -- with
# the area gate disabled, all four misses were rejected by the convexity check.
#
# The cause is over-parameterisation. A homography has 8 degrees of freedom; on
# the frames that fail, the panel is small, blurred or oblique and yields only a
# handful of correct correspondences. RANSAC then has enough freedom to fit a
# degenerate, folded solution that still explains those few points.
#
# An affine transform has 6 DOF and CANNOT fold: it maps the reference rectangle
# to a parallelogram, which is convex by construction. It cannot represent
# perspective foreshortening -- but the regime where it is needed is precisely
# the distant/small-panel one, where perspective across a ~10px panel is
# negligible anyway.
#
# It is a FALLBACK, not a replacement: homography is tried first at every ratio
# and affine is only reached when all of those are rejected. Every geometric
# guard still applies to the affine fit.
#
# ---------------------------------------------------------------------------
# MEASURED RESULT: THIS MAKES THINGS WORSE. DEFAULT OFF.
#
# On the eval set it converted exactly one miss into a WRONG-PICK (f0440, 44.6px
# off the target) and gained zero correct picks: 16 correct / 0 wrong / 4 miss
# became 16 correct / 1 wrong / 3 miss. It trades a safe abstention for a
# confident error, which is the wrong direction on the only metric that matters.
#
# Why the reasoning above was wrong: affine cannot FOLD, but it can still fit a
# confidently WRONG location. The folded quad was never the problem in itself --
# it was the SYMPTOM of a correspondence set too weak to determine the panel's
# pose. Constraining the model removes the symptom without fixing the cause, so
# the detector loses the signal that told it to abstain and starts answering
# from the same bad data.
#
# Kept here, off, because "we tried constraining the model and it silently
# converted an abstention into a wrong answer" is worth not rediscovering.
# ---------------------------------------------------------------------------
USE_AFFINE_FALLBACK = False
# Sanity bounds on the projected panel quad -- a valid homography maps the
# reference rectangle to a convex, non-degenerate quad of plausible size.
# NOTE: a MIN_QUAD_AREA_FRAC gate used to live here (0.0004 of the frame). It was
# removed after measuring that it rejected nothing the convexity check did not
# already reject -- every frame it fired on also produced a folded quad, and also
# had a projected button radius below MIN_PROJECTED_BUTTON_RADIUS_PX. It was an
# arbitrary number doing no independent work, and keeping it only made it look
# like there were several independent size guards when there was really one.
MAX_QUAD_AREA_FRAC = 2.5
MAX_ASPECT = 6.0
# Minimum apparent button radius, in query-frame pixels, below which the
# panel is too far away for a press target to mean anything and the
# detector abstains REGARDLESS of how well the homography fitted.
#
# This exists because abstention was otherwise incidental: on the most
# distant eval frame the detector happened to land just under the inlier
# threshold, so it abstained by luck, and a couple more inliers would have
# produced a confident pick on a 2px button. Scale is read off the
# homography itself (sqrt of the projected-quad area ratio), so this is a
# statement about physical resolvability, not another tuned gate.
MIN_PROJECTED_BUTTON_RADIUS_PX = 3.5


# Name given to the legacy single ``button_xy`` mark when a reference has no
# ``buttons`` table. It is the START/+30SEC button on every reference so far.
LEGACY_BUTTON_NAME = "start_30s"


def _marked_buttons(meta):
    """``{name: [x, y]}`` in reference-IMAGE coordinates.

    Two forms are accepted per view: the original single ``button_xy`` (one
    hand-placed mark, always START/+30SEC), and a ``buttons`` table naming
    several marks on the same image. When both are present ``button_xy`` is
    folded in under LEGACY_BUTTON_NAME so old references keep working and a
    new table only has to add the extra buttons.
    """
    buttons = dict(meta.get("buttons") or {})
    if "button_xy" in meta:
        buttons.setdefault(LEGACY_BUTTON_NAME, meta["button_xy"])
    if not buttons:
        raise SystemExit(f"reference view {meta.get('image')} marks no button "
                         "(needs 'button_xy' or a 'buttons' table)")
    return buttons


class _View:
    """One reference image of the panel, with the button(s) marked on it."""

    def __init__(self, ref_dir, meta, sift, target):
        img = cv2.imread(str(Path(ref_dir) / meta["image"]))
        if img is None:
            raise SystemExit(f"cannot read reference image {meta['image']}")
        x0, y0, x1, y1 = meta["crop"]
        self.meta = meta
        self.ref = img[y0:y1, x0:x1]
        buttons = _marked_buttons(meta)
        if target not in buttons:
            raise SystemExit(
                f"reference view {meta['image']} has no mark for button {target!r}; "
                f"it marks {sorted(buttons)}. Every view must mark the target.")
        # Marked target button, expressed in reference-CROP coordinates.
        self.ref_pt = np.float32([[buttons[target][0] - x0,
                                   buttons[target][1] - y0]]).reshape(-1, 1, 2)
        self.kp_ref, self.des_ref = sift.detectAndCompute(
            cv2.cvtColor(self.ref, cv2.COLOR_BGR2GRAY), None)
        h, w = self.ref.shape[:2]
        self.ref_corners = np.float32(
            [[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
        self.button_radius = meta.get("button_radius", 10)


class ReferenceButtonDetector:
    """Match the panel against one or more marked reference views.

    Multiple views exist because SIFT's scale invariance is not unlimited in
    practice: a reference taken at one working distance runs out of usable
    correspondences when the panel appears much larger or smaller, and the
    fitted homography then degenerates. A close, a mid and a distant view of
    the same panel mean whichever is nearest in scale can carry the match.

    All views are tried and the fit with the most RANSAC inliers wins, among
    those that pass every geometric guard. Adding a view therefore widens what
    can be FOUND without changing what is ACCEPTED -- each candidate still has
    to clear the same checks on its own. (Contrast the affine experiment
    recorded above, which widened what was accepted and immediately produced a
    wrong-pick.)
    """

    def __init__(self, ref_dir, min_inliers=MIN_INLIERS, ratio=None,
                 upscale=1.0, target=None):
        """``target`` names which marked button to report (a key of each view's
        ``buttons`` table). None takes the reference's ``default_button``, else
        the legacy single mark (START/+30SEC). The panel fit is identical for
        every target -- only the projected point changes."""
        self.min_inliers = min_inliers
        self.ratio = ratio
        # When a caller pins a ratio (the parameter sweep does) the cascade is
        # bypassed so the sweep measures what it thinks it is measuring.
        self.ratio_fixed = ratio is not None
        self.upscale = upscale
        ref_dir = Path(ref_dir)
        meta = json.loads((ref_dir / "reference.json").read_text())
        self.meta = meta
        self.sift = cv2.SIFT_create(nfeatures=4000)
        # "views" is the multi-reference form; a bare image/crop/button_xy at
        # the top level is still accepted as a single view.
        view_metas = meta.get("views") or [meta]
        self.target = target or meta.get("default_button") or LEGACY_BUTTON_NAME
        self.views = [_View(ref_dir, vm, self.sift, self.target) for vm in view_metas]
        self.matcher = cv2.BFMatcher()

    @staticmethod
    def _quad_ok(quad, frame_shape):
        pts = quad.reshape(-1, 2)
        area = abs(cv2.contourArea(pts.astype(np.float32)))
        frame_area = float(frame_shape[0] * frame_shape[1])
        if area > MAX_QUAD_AREA_FRAC * frame_area:
            return False, f"quad area {area/frame_area:.5f} of frame"
        if not cv2.isContourConvex(pts.astype(np.float32)):
            return False, "quad not convex"
        side = [np.linalg.norm(pts[i] - pts[(i + 1) % 4]) for i in range(4)]
        if min(side) <= 1e-6 or max(side) / min(side) > MAX_ASPECT:
            return False, f"quad aspect {max(side)/max(min(side),1e-6):.1f}"
        return True, ""

    def detect(self, bgr):
        """Best geometrically valid fit across every reference view and ratio,
        or the last failure (for diagnostics)."""
        last = None
        best = None
        ratios = (self.ratio,) if self.ratio_fixed else RATIO_CASCADE
        for vi, view in enumerate(self.views):
            for ratio in ratios:
                r = self._detect_one(bgr, ratio, view=view)
                if r["center"] is not None:
                    r["ratio"] = ratio
                    r["view"] = vi
                    if best is None or r["inliers"] > best["inliers"]:
                        best = r
                    break  # this view has answered; move to the next view
                last = r
                # An off-frame projection is a real answer ("the button is not
                # in this image"), not a failed fit -- do not keep searching
                # for a ratio that invents one.
                if r.get("reason") == "projected point off-frame":
                    break
        if best is not None:
            return best
        if USE_AFFINE_FALLBACK:
            for ratio in (self.ratio,) if self.ratio_fixed else RATIO_CASCADE:
                r = self._detect_one(bgr, ratio, affine=True)
                if r["center"] is not None:
                    r["ratio"] = ratio
                    r["model"] = "affine"
                    return r
                if r.get("reason") == "projected point off-frame":
                    return r
        last = last or {"center": None, "inliers": 0, "reason": "no fit"}
        return last

    def _detect_one(self, bgr, ratio, affine=False, view=None):
        view = view or self.views[0]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        kp, des = self.sift.detectAndCompute(gray, None)
        if des is None or view.des_ref is None or len(kp) < 2:
            return {"center": None, "inliers": 0, "reason": "no descriptors"}
        matches = self.matcher.knnMatch(view.des_ref, des, k=2)
        good = [m for pair in matches if len(pair) == 2
                for m, n in [pair] if m.distance < ratio * n.distance]
        if len(good) < 4:
            return {"center": None, "inliers": 0, "reason": f"only {len(good)} good matches"}

        src = np.float32([view.kp_ref[g.queryIdx].pt for g in good]).reshape(-1, 1, 2)
        dst = np.float32([kp[g.trainIdx].pt for g in good]).reshape(-1, 1, 2)
        if affine:
            A, mask = cv2.estimateAffine2D(src, dst, method=cv2.RANSAC,
                                           ransacReprojThreshold=4.0)
            H = None if A is None else np.vstack([A, [0.0, 0.0, 1.0]])
            # A collapsed affine (near-zero determinant) squashes the panel to a
            # line; the aspect/resolvability guards would catch most of it, but
            # reject it explicitly rather than relying on that.
            if H is not None and abs(float(np.linalg.det(H[:2, :2]))) < 1e-6:
                H = None
        else:
            H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
        inliers = int(mask.sum()) if mask is not None else 0
        if H is None:
            return {"center": None, "inliers": inliers,
                    "reason": "no affine fit" if affine else "no homography"}
        if inliers < self.min_inliers:
            return {"center": None, "inliers": inliers,
                    "reason": f"{inliers} inliers < {self.min_inliers}"}

        quad = cv2.perspectiveTransform(view.ref_corners, H)
        ok, why = self._quad_ok(quad, gray.shape)
        if not ok:
            return {"center": None, "inliers": inliers, "reason": why, "quad": quad}

        ref_area = float(view.ref.shape[0] * view.ref.shape[1])
        quad_area = abs(cv2.contourArea(quad.reshape(-1, 2).astype(np.float32)))
        scale = (quad_area / ref_area) ** 0.5 if ref_area > 0 else 0.0
        proj_r = view.button_radius * scale
        if proj_r < MIN_PROJECTED_BUTTON_RADIUS_PX:
            return {"center": None, "inliers": inliers,
                    "reason": f"button would be {proj_r:.1f}px -- too far to resolve"}

        pt = cv2.perspectiveTransform(view.ref_pt, H).reshape(2)
        h, w = gray.shape[:2]
        if not (0 <= pt[0] < w and 0 <= pt[1] < h):
            return {"center": None, "inliers": inliers, "reason": "projected point off-frame"}
        return {"center": (float(pt[0]), float(pt[1])), "inliers": inliers, "quad": quad}
