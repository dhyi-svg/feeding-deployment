# Button detection: eval harness + reference-homography detector

Working notes. Everything here is measured on a **very small** label set and
should be read as provisional until the label count goes up and a clip is
genuinely held out.

## Scope

**Only the Comfee clip** (`2C3103B0-...MP4`). The older IMG_38xx footage is a
different microwave (red, different finish and button labelling) and is out of
scope by decision -- this targets the appliance the work is actually for.

Consequence: there is no second appliance to hold out against, so the holdout
is **temporal** -- frames from the same clip that are never looked at while
tuning. That is weaker evidence than a different appliance (same lighting,
session and operator) and should not be read as evidence of cross-appliance
generalisation, which is untested.

## Current numbers (21 scored frames; 13 tune / 11 holdout)

| detector    | correct | WRONG | imprecise | miss |
|-------------|---------|-------|-----------|------|
| homography  | 16/21   | **0** | 1         | 4    |
| hough       | 7/21    | **10**| 0         | 4    |

Split for the homography detector: **tune 9/10, holdout 7/11, zero wrong-picks
on either.** Hough on the same holdout: 4/11 correct with 3 wrong.

The clean statement: **across 21 frames it has never pointed at the wrong
button.** It answers correctly or it abstains. Every failure is an abstention
from a geometric guard.

Both holdout misses are abstentions from the geometric guards: one non-convex
quad, one projected-quad area of 0.00036 against a `MIN_QUAD_AREA_FRAC` of
0.0004. That threshold was set arbitrarily and never fitted to anything, and
it is now implicated in a miss -- but it has **deliberately not been retuned**,
because tuning it against a holdout failure is exactly what destroys the value
of having a holdout. If it is changed, the next evaluation needs fresh frames.

`WRONG` = would press a different button, or is lost off the panel entirely;
includes any pick on a frame where the button is absent from the image. This
is the number that must reach zero. `imprecise` = outside the button but still
unambiguously aimed at it (nearer the target than the neighbouring STOP/ECO
button, and within the inter-button spacing). `miss` = abstained, the safe
failure.

Still **all one clip and one appliance**, and the parameters were tuned on
these same 10 frames. The red-microwave clips are the real holdout and have
not been touched yet.

## Why the metric is shaped this way

An earlier version of this work scored only "temporally stable frames," which
is trivially maximised by setting `STABLE_FRAMES=1` or widening the pixel
tolerance. Any optimiser pointed at that number will find those knobs. Scoring
wrong-picks separately means a change that merely trades coverage for
confidence shows up immediately instead of looking like progress.

## What the detector does

`detector_homography.py`. The +30SEC button is marked **by hand, once**, on a
sharp reference frame (frame 207, the sharpest in the clip); per-frame it is
located by SIFT match -> RANSAC homography -> project the mark. Button identity
comes from a human pointing at it, not from a per-frame "bottom row, rightmost"
rule that mislabels whenever the circle detector misses one of the five.

Confidence is the RANSAC inlier count, which cannot be inflated by loosening a
threshold the way a candidate-count or stability gate can.

Cost: one reference per appliance model. The Comfee reference will not transfer
to the red microwave -- that needs its own.

## How the detector handles scale

No single Lowe ratio works across this footage. Measured directly: at 0.70 the
most distant frame fits to within 2.7px while the blurred close-up degenerates;
at 0.80 exactly the reverse. `detect()` therefore tries a cascade
(0.70/0.75/0.80/0.85) and takes the first fit that passes the geometric guards.
That is a search, not a loosening -- every candidate still has to clear the
inlier floor and the convex/area/aspect/resolvability checks, so an extra
attempt cannot admit a fit a single-ratio run would have rejected.

## Things that turned out to be wrong along the way

1. **The inlier threshold was too strict, not the matcher too weak.** 12 -> 8
   turned two misses into correct picks with wrong-picks staying at 0, which is
   what shows those homographies were already right and merely thin. Defaults
   now sit mid-plateau (ratio 0.80, min_inliers 8), not on an edge.

2. **~20% of my first-pass labels were wrong, and it corrupted reported
   results.** `audit_labels.py` (overlay every label on its frame at high
   magnification, showing the neighbouring button too) found 4 bad labels out
   of ~20: f0280 one button LEFT of the target, f0560 in the GAP between the
   two bottom buttons, f0790 32px off on blank panel, f0520 ~7px off. **Two of
   them had been reported as detector failures when the detector was closer to
   the truth than the label was.** All four came from reading coordinates off
   too-wide zooms.

   This is the circularity risk in this setup made concrete: **the same agent
   places the labels and scores against them, so labelling mistakes present as
   detector mistakes.** Mitigations now in place: label only from the 4K source
   at high magnification (`source_zoom.py`), and run `audit_labels.py` before
   believing any reported failure. The audit is cheap and has a 100% hit rate
   so far at finding errors the scores alone could not distinguish from
   detector faults.

3. **My own label semantics were wrong, and mis-scored the detector.** The
   original schema had a boolean `resolvable`, and the most distant frame was
   marked `false` on the grounds that the button is ~2px and unreadable by eye.
   But a homography does not need to see the button -- it transfers a mark
   using surrounding panel texture, and in fact located that frame to within
   **2.8px**. Scoring it as "should have abstained" punished a correct result.
   `resolvable` is now a three-way `status`:
   - `labeled` -- trustworthy center, score against it
   - `abstain_expected` -- button genuinely absent (cropped off frame); any
     pick is a wrong-pick
   - `unverifiable` -- no trustworthy center could be placed; **excluded from
     scoring**, because "I cannot label this" is a claim about the labeler, not
     about what a detector should do

4. **A guard I added for the right reason fires for the wrong one.** The
   minimum-projected-button-radius check was added believing the distant-frame
   abstention was luck. It does NOT catch that case, and is still
   **unexercised by any test**. What actually rejects the bad fits is the
   **quad convexity check** -- a degenerate homography folds the projected
   panel, and that is what both remaining hard frames trip. Treat the radius
   guard as unvalidated.

5. **My first version of the IMPRECISE rule was too lenient and I nearly
   shipped it.** Defining "near the right button" as merely "nearer the target
   than the neighbour" let a 178px Hough error -- completely off the panel --
   score as imprecise rather than wrong, which flattered the baseline. Fixed by
   also bounding it by the inter-button spacing. Worth noting because it is the
   exact failure mode this whole harness exists to prevent, and it appeared in
   the harness itself.

6. **The inlier count is not what keeps this honest.** Lowering the floor
   8 -> 6 raised recall with wrong-picks staying at zero. At 6 correspondences
   the geometric guards are carrying the safety argument, not the consensus
   size. That is fine, but it means the quad checks must not be weakened.

## Known weaknesses right now

- **6 scored frames.** The tuning sweep was run on the same 6, so 5/6 is
  optimistic by construction. No held-out clip yet.
- **Labels are mine, placed by eye, and I am also scoring against them.**
  `audit_labels.py` renders them for spot-checking. A surprising result should
  be checked for label error first.
- Small-panel frames are now labeled from the **4K source** (panel ~100px)
  via `source_zoom.py`, not the 960px work frame (~25px). This mattered: an
  earlier by-eye guess from the work frame was 5px off on f0874, enough to
  flip its score.
- f0240 remains the one real miss: every ratio in the cascade produces a
  degenerate (non-convex or near-zero-area) quad. Small, angled and blurred.
- f0400 / f0480 / f0640 are unlabeled -- motion blur and panel rotation make a
  trustworthy by-eye center impossible; f0480 is explicitly `unverifiable`.
- f0700 scores CORRECT at 23px error against a 33px tolerance -- passing, but
  the weakest of the correct picks; worth watching as labels grow.
- Nothing here validates the pixel -> 3D -> `arm_base_link` half of
  `detect_start_button`. This footage has no depth and no calibration.

## Integration

The detector now lives in the repo at
`src/feeding_deployment/perception/appliance_perception/reference_button_detector.py`
(the eval imports it, so there is one source of truth), and is wired into
`AppliancePerception` as `BUTTON_BACKEND=reference`, with the reference
directory given by `BUTTON_REFERENCE_DIR`.

**Lab behaviour is unchanged by default.** When `BUTTON_REFERENCE_DIR` is unset
the backend is unavailable and `auto` falls through to Molmo -> GroundingDINO ->
Hough exactly as before. When it IS set, `auto` tries the reference first and
falls back on abstention.

Tests: `tests/test_reference_button_detector.py` (5 passing) covers the found
case, the upside-down case, and three abstention cases, using synthetic panels
so it needs no footage, models or camera.

Caveat: `tests/test_button_detection.py` and anything else importing
`appliance_perception` **could not be run on this machine** -- that module
imports `open3d`, which is not installed here. The integration wiring is
therefore unverified by execution; only the detector module itself is tested.

## Things that were tried and did NOT work

**Affine fallback (rejected, kept in the code disabled).** All four remaining
misses have one root cause: the fitted homography folds (non-convex quad). An
affine transform has 6 DOF instead of 8 and cannot fold, so falling back to one
looked principled. Measured: it converted one miss into a **WRONG-PICK** (44.6px
off) and gained zero correct picks. The reasoning was wrong because the folded
quad was never the problem itself -- it was the SYMPTOM of a correspondence set
too weak to determine the panel's pose. Constraining the model removed the
symptom without fixing the cause, so the detector lost the very signal telling
it to abstain and answered from the same bad data. `USE_AFFINE_FALLBACK=False`.

**Multi-reference (rejected).** Every abstention has the same root cause: too
few correspondences at that scale/angle, so the homography folds. Adding a
close (f108) and a mid-far (f610) reference view of the same panel alongside
the original looked like the fix -- and unlike the affine experiment it was
safe by construction, since each view's fit still had to clear every guard on
its own. Measured: **the score did not move at all** (22/29, 0 wrong, same six
misses, same reasons). The close-up view *was* being chosen on 4 frames, so it
produced better fits where matching already worked -- it just rescued nothing
where it did not. Cost: ~4x runtime (232ms vs 60ms per frame). Reverted to a
single view; the `views` key is still supported in reference.json for anyone
who wants to revisit it.

The conclusion these two experiments point at together: the six abstentions are
not a modelling or a reference-coverage problem. Those frames are small,
blurred or oblique enough that the image does not determine the panel's pose,
and no amount of reformulating the fit changes that. Making them answer would
mean better input, not a cleverer fit.

**A redundant size gate (removed).** `MIN_QUAD_AREA_FRAC = 0.0004` turned out to
reject nothing the convexity check did not already reject. Removing it left
every score identical, which is what makes it a simplification rather than a
tuning change.

## Next

1. The holdout has now informed design decisions (the affine experiment was
   diagnosed on it). A genuinely fresh set of frame indices from the clip is
   needed before the next number can be called clean.
2. The 4 abstentions are understood and are NOT a threshold problem: the panel
   is too small/blurred/oblique to yield correspondences that determine its
   pose. Making the detector answer on them means getting better
   correspondences, not relaxing a gate -- and the affine experiment is direct
   evidence that relaxing the model instead produces wrong answers.
3. Nothing here validates the pixel -> 3D -> arm_base_link half of
   `detect_start_button`; this footage has no depth and no calibration. A
   correct pixel is necessary, not sufficient, for a real press.
4. Untested on any other appliance or lighting. One reference per appliance.

## Files

- `extract_frames.py` -- builds the eval set from the clip (tune/holdout split,
  reference window excluded)
- `labels.json` -- ground truth + provenance + the log of label errors found
- `audit_labels.py` -- overlay every label on its frame; **run this before
  believing any reported failure**
- `source_zoom.py` / `zoom.py` / `zoom_batch.py` / `sheet.py` / `montage.py` --
  labeling aids; only `source_zoom.py` (4K source, high magnification) produces
  labels that have survived audit
- `detector_homography.py` -- the detector
- `eval.py` -- scoring, reported per split
- `reference/` -- the hand-marked reference frame and its metadata
