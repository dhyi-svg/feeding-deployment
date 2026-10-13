"""Button finder for the Comfee panel: find the 5 chrome domes by their 3+2 layout.

Pure OpenCV/numpy, no ROS, no reference images. press_button uses it for every measurement:
the first one from wherever the arm starts (~22-50+ cm) and the final one from the ~25 cm
staging pose.

COLOUR IS NEVER USED (2026-10-12): everything runs on the grey image + depth, so any panel colour
works (red Comfee here, a cream one at HERL).

How it works:
  1. candidates  dome-shaped spots on the grey image: a ring darker than the dome's own inside OR
                 the panel around it (red panel: inside 196 / ring 73 / panel 90 grey levels;
                 cream: 196 / 149 / 225). Ring sizes RING_RADII_PX, each kept only where it is
                 RING_FRAC_OF_SPACING of the dome spacing at that pixel's own depth; peaks are kept
                 per ring size, strongest RING_MAX each
  2. layout fit  for every ordered candidate pair taken as (top-middle, top-right) dome, predict
                 the other three domes from the known layout (top row of 3 at spacing s; bottom
                 row of 2, 0.92 s lower, offset half a step) and count candidates that land within
                 FIT_TOL * s. A fit must hit all 5 AND its spacing must measure DOME_SPACING_MM
                 in metres via depth -- label text and knob highlights fail one or the other.
                 Fitted one ring size at a time (a dome set is one size; similar size is still
                 checked, MIN_DOME_SIZE_RATIO), the 5 at one depth (DOME_Z_SPREAD_M). Across sizes
                 the most dome-like set (its weakest ring) wins; where several sizes give the same
                 layout, the largest ring (the dome's edge) sets the centres -- a dome's small
                 inner highlight sits ~3 mm off centre.
                 3 domes on top vs 2 below fixes "up", so a rolled camera still names them right.
  3. 3D          panel plane fitted (geometry.fit_plane_inverse_depth) to the depth pixels in the
                 button block's region except the domes (the depth sensor lies on chrome).

Tested 2026-10-03 on the 5 saved reference frames + 2 live ones: 5/5 domes named correctly from
~25 to ~50 cm, 3-4 px from the hand-marked centres far out. Up close (~17 cm) a dome is no longer
one highlight blob and the fit locked onto label text -- hence MIN_RANGE_M.
2026-10-12, colour-free version: start_30s within ~4 px of the old colour detector on the saved
close-up frames (red, and the same frames recoloured cream), and on live frames steadier than it
(one pixel, where the colour version flickered 11 px between two highlight spots). On the Jetson:
~0.25 s for a frame from scratch, ~10-20 ms with `hint` (the previous frame's fit: search only
around it, at its ring size), so a 7-frame measurement costs ~0.35 s.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from rammp_button_press.geometry import fit_plane_inverse_depth, pixel_ray

# Physical top-row pitch on the Comfee panel (measured 19.3 mm at 39 cm, 2026-10-03).
DOME_SPACING_MM = (16.8, 21.8)
NAMES = ("power_level", "wgt_time_defrost", "timer_clock", "stop_eco", "start_30s")
# Dome centres in units of the top-row spacing, origin = top-middle dome, x right, y DOWN the panel.
LAYOUT = np.array([[-1.0, 0.0], [0.0, 0.0], [1.0, 0.0], [-0.58, 0.92], [0.42, 0.92]])
FIT_TOL = 0.2                 # max dome miss, fraction of s (true fits: <= 0.17 on the test frames)
MIN_RANGE_M = 0.22            # closer than this, domes break into several highlights: abstain
MAX_RANGE_M = 1.2
# Blob size check, on the equivalent radius sqrt(area / pi). On 2026-10-12 (RAMMP Jetson), from a
# close view (~24 cm) a 3+2 fit using two specks in the "Timer/Clock" label (r 1.5 px) and three
# real domes (r 8.3-8.9 px) beat the true layout on 12/12 frames, putting start_30s on the
# Timer/Clock button: smallest/largest 0.17. True fits: 0.93 at 24 cm, 0.66 from 48 cm at a steep
# angle (where a dome is only its highlight, r 1.5-2.3 px). A size floor relative to the spacing
# (0.08 s) was tried and rejected those oblique real domes (0.061 s). Size only, no roundness.
MIN_DOME_SIZE_RATIO = 0.6     # the 5 spots are the same size: smallest >= this x the largest
# Dome candidates by SHAPE (grey image only): a dark ring of radius r with a brighter band outside.
NOMINAL_SPACING_M = 0.0198    # sets the ring sizes searched; the layout fit still MEASURES the spacing
RING_FRAC_OF_SPACING = (0.07, 0.32)   # dome ring radius / spacing: 0.18 close (24 cm), ~0.11 far (48 cm)
RING_RADII_PX = (3, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16)   # every size a dome takes from ~60 cm to ~22 cm (2 px: label specks pass as domes)
DOME_Z_SPREAD_M = 0.04        # the 5 domes are on one flat panel: their depths agree to this
RING_DARK = (0.8, 1.15)       # dark annulus, in units of r
RING_OUT = (1.3, 1.7)         # the panel just outside it
RING_IN = 0.5                 # the dome's inside, up to this radius
RING_MIN_CONTRAST = 35.0      # grey levels the ring must be darker by (a 3 px speck in an 8 px ring: ~27)
RING_MIN_STD = 4.0            # floor on the local contrast the score is divided by
SPOT_MERGE_FRAC = 1.0         # responses closer than this x ring size are one spot
SIZE_KEEP_FRAC = 0.7          # a spot's size = its largest ring scoring >= this x its best
RING_MIN_SCORE = 0.7          # (brighter side - ring) / local std; true domes scored 1.2-2.0
RING_MAX = 60                 # strongest peaks kept (the layout fit is O(n^2))
# Region (layout units) whose depth pixels the panel plane is fitted to: the button block and the
# panel around it, not the knob/display above or the microwave's bottom edge.
PLANE_REGION = np.array([[-2.2, -1.2], [2.2, -1.2], [2.2, 2.0], [-2.2, 2.0]])


@dataclass
class DomeFit:
    px: dict                  # name -> (u, v)
    s_px: float
    s_mm: float
    err: float                # mean dome miss / s
    z: float                  # median depth at the domes (m)
    region: np.ndarray        # 4x2 image polygon for the plane fit
    score: float = 0.0        # weakest of the 5 ring strengths (how dome-like the set is)
    r_px: float = 0.0         # ring radius the layout was found at


def _box_sum(img, h):
    """Sum over the (2h+1)^2 square centred on every pixel (zero outside the image)."""
    k = 2 * int(h) + 1
    return cv2.boxFilter(img, cv2.CV_32F, (k, k), normalize=False, borderType=cv2.BORDER_CONSTANT)


def _ring_scores(gray, gray2, r):
    """How much darker a SQUARE ring at radius r is than the dome's inside OR the panel outside
    it, over the local contrast. Squares (integral image: the same cost at every size) instead of
    circles: the formation checks are what make a match strict. Measured 2026-10-12 (grey levels):
    red panel inside 196 / ring 73 / panel 90; cream 196 / 149 / 225 -- whichever side is
    brighter counts."""
    h_in, h_r0, h_r1, h_o0, h_o1 = (max(0, int(round(f * r))) for f in
                                    (RING_IN, RING_DARK[0], RING_DARK[1], RING_OUT[0], RING_OUT[1]))
    h_r1 = max(h_r1, h_in + 1)
    h_r0 = min(h_r0, h_r1 - 1)
    h_o0 = max(h_o0, h_r1)
    h_o1 = max(h_o1, h_o0 + 1)                  # every band at least one pixel wide
    S = {h: _box_sum(gray, h) for h in {h_in, h_r0, h_r1, h_o0, h_o1}}
    area = {h: float((2 * h + 1) ** 2) for h in S}
    mi = S[h_in] / area[h_in]
    mr = (S[h_r1] - S[h_r0]) / (area[h_r1] - area[h_r0])
    mo = (S[h_o1] - S[h_o0]) / (area[h_o1] - area[h_o0])
    mean = S[h_o1] / area[h_o1]
    std = np.sqrt(np.maximum(_box_sum(gray2, h_o1) / area[h_o1] - mean * mean, 1.0))
    diff = np.maximum(mi - mr, mo - mr)
    # A real ring is darker by a real amount (domes: 45-120 grey levels), not merely relative to a
    # flat background's tiny contrast -- a speck in a big ring on a flat panel would score high.
    return np.where(diff >= RING_MIN_CONTRAST, diff / np.maximum(std, RING_MIN_STD), 0.0).astype(np.float32)


def candidates(bgr, depth_m, fx, hint=None):
    """[(u, v, z, r, score)] dome-shaped spots, found on the GREY image only (no colour): a ring
    darker than the dome's inside or the panel around it. Each ring size is kept only where it is
    RING_FRAC_OF_SPACING of the dome spacing at that pixel's own depth, and peaks are kept PER RING
    SIZE (a dome's small inner highlight and its own edge are both kept; fit_layout fits one size
    at a time). `hint` = the previous frame's DomeFit: search only around it, at its ring size."""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    h_img, w_img = gray.shape
    x_off = y_off = 0
    radii = RING_RADII_PX
    if hint is not None:
        pad = 1.5 * hint.s_px
        us = [p[0] for p in hint.px.values()]
        vs = [p[1] for p in hint.px.values()]
        x_off, y_off = int(max(0, min(us) - pad)), int(max(0, min(vs) - pad))
        x_end, y_end = int(min(w_img, max(us) + pad)), int(min(h_img, max(vs) + pad))
        gray, depth_m = gray[y_off:y_end, x_off:x_end], depth_m[y_off:y_end, x_off:x_end]
        h_img, w_img = gray.shape
        radii = [r for r in RING_RADII_PX if abs(r - hint.r_px) <= 3]   # room to find each spot's edge
    valid = depth_m[(depth_m > 0.1) & (depth_m < 2.0)]
    if valid.size < 100:
        return []
    z_lo, z_hi = (float(np.percentile(valid, q)) for q in (2, 98))
    r_lo = RING_FRAC_OF_SPACING[0] * fx * NOMINAL_SPACING_M / max(z_hi, MIN_RANGE_M)
    r_hi = RING_FRAC_OF_SPACING[1] * fx * NOMINAL_SPACING_M / max(z_lo, 0.15)
    gray2 = gray * gray
    out = []
    for r in radii:
        if not r_lo - 1 <= r <= r_hi + 1:               # no dome at any depth in view is this size
            continue
        nms = max(3, int(r / RING_FRAC_OF_SPACING[1] * 0.3) | 1)   # ~0.3 dome spacing at this size
        sc = _ring_scores(gray, gray2, r)
        peaks = (sc >= RING_MIN_SCORE) & (sc == cv2.dilate(sc, np.ones((nms, nms), np.uint8)))
        vs, us = np.nonzero(peaks)
        for i in np.argsort(-sc[vs, us])[:RING_MAX]:
            v, u = int(vs[i]), int(us[i])
            x0, x1 = int(max(0, u - 2 * r)), int(min(w_img, u + 2 * r + 1))
            y0, y1 = int(max(0, v - 2 * r)), int(min(h_img, v + 2 * r + 1))
            patch = depth_m[y0:y1, x0:x1]
            patch = patch[(patch > 0.1) & (patch < 2.0)]
            if patch.size < 5:
                continue
            z = float(np.median(patch))
            frac = r / (fx * NOMINAL_SPACING_M / z)        # ring radius / dome spacing at ITS depth
            if not RING_FRAC_OF_SPACING[0] <= frac <= RING_FRAC_OF_SPACING[1]:
                continue
            out.append((float(u + x_off), float(v + y_off), z, float(r), float(sc[v, u])))
    return _one_size_per_spot(out)


def _one_size_per_spot(resp):
    """Each spot answers at several ring sizes; give it ONE size: the largest ring it still
    answers clearly (>= SIZE_KEEP_FRAC of its best), i.e. its edge, and that ring's centre (a
    dome's small inner highlight sits ~3 mm off its centre). Then "5 of the same size" means it."""
    spots = []
    for c in sorted(resp, key=lambda c: -c[4]):
        for sp in spots:
            if np.hypot(c[0] - sp[0][0], c[1] - sp[0][1]) < SPOT_MERGE_FRAC * max(c[3], sp[0][3]):
                sp.append(c)
                break
        else:
            spots.append([c])
    out = []
    for sp in spots:
        best = sp[0][4]
        edge = max((c for c in sp if c[4] >= SIZE_KEEP_FRAC * best), key=lambda c: c[3])
        out.append((edge[0], edge[1], edge[2], edge[3], best))
    return out


def fit_layout(cands, fx, hint=None):
    """Best all-5 fit of LAYOUT to the candidate spots, or None. With `hint`, only the layout the
    previous frame found (start_30s within 0.3 spacing of it)."""
    fit = _fit_layout_one(cands, fx)
    if fit is not None and hint is not None and \
            np.hypot(*np.subtract(fit.px[NAMES[2]], hint.px[NAMES[2]])) >= 0.3 * hint.s_px:
        return None
    return fit


def _fit_layout_one(cands, fx):
    if len(cands) < 5:
        return None
    P = np.array([[c[0], c[1]] for c in cands])
    Z = np.array([c[2] for c in cands])
    Rb = np.array([c[3] for c in cands])
    Sc = np.array([c[4] if len(c) > 4 else 0.0 for c in cands])
    best = None
    # Only pairs whose spacing measures like the top row in mm (vectorised: most pairs fail here).
    D = np.linalg.norm(P[None] - P[:, None], axis=2)
    S_mm = D * Z[:, None] / fx * 1000.0
    ok = (S_mm >= DOME_SPACING_MM[0]) & (S_mm <= DOME_SPACING_MM[1])
    for i, j in zip(*np.nonzero(ok)):
        if True:
            s, s_mm = float(D[i, j]), float(S_mm[i, j])
            ex = (P[j] - P[i]) / s
            ey = np.array([-ex[1], ex[0]])   # panel "down" in the image when x is right
            pred = P[i] + s * (LAYOUT[:, :1] * ex + LAYOUT[:, 1:] * ey)
            dist = np.linalg.norm(P[None] - pred[:, None], axis=2)
            # Per predicted dome, of the spots within FIT_TOL take the one whose SIZE matches the
            # pair's own (a dome also carries small highlight spots); nearest breaks ties.
            r_ref = np.sqrt(Rb[i] * Rb[j])
            inside = dist < FIT_TOL * s
            if not inside.any(axis=1).all():
                continue
            cost = np.where(inside, np.abs(np.log(Rb[None] / r_ref)) + 0.01 * dist / s, np.inf)
            k = cost.argmin(1)
            e = dist[np.arange(5), k] / s
            if len(set(k)) < 5:
                continue
            r = Rb[k]
            if r.min() < MIN_DOME_SIZE_RATIO * r.max():
                continue
            if Z[k].max() - Z[k].min() > DOME_Z_SPREAD_M:
                continue
            # most dome-like set first (its weakest ring), geometry error to break ties
            key = (float(Sc[k].min()), -float(e.mean()))
            if best is None or key > best[0]:
                region = P[i] + s * (PLANE_REGION[:, :1] * ex + PLANE_REGION[:, 1:] * ey)
                best = (key, float(e.mean()), P[k], s, s_mm, float(np.median(Z[k])), region, float(np.median(Rb[k])))
    if best is None:
        return None
    key, err, pts, s, s_mm, z, region, r_med = best
    return DomeFit(px={n: tuple(map(float, p)) for n, p in zip(NAMES, pts)}, s_px=s, s_mm=s_mm, err=err,
                   z=z, region=region, score=key[0], r_px=r_med)


def detect(bgr, depth_m, fx, hint=None):
    """DomeFit for this frame, or None (no fit, or outside MIN_RANGE_M..MAX_RANGE_M)."""
    fit = fit_layout(candidates(bgr, depth_m, fx, hint), fx, hint)
    if fit is None or not MIN_RANGE_M <= fit.z <= MAX_RANGE_M:
        return None
    return fit


def panel_plane(bgr, depth_m, fit: DomeFit, intr):
    """Plane n.X = d (camera frame) through the panel's depth pixels around the domes."""
    mask = np.zeros(depth_m.shape, np.uint8)
    cv2.fillConvexPoly(mask, fit.region.astype(np.int32), 1)
    for u, v in fit.px.values():                      # not the domes: the depth sensor lies on chrome
        cv2.circle(mask, (int(round(u)), int(round(v))), int(np.ceil(0.35 * fit.s_px)), 0, -1)
    ok = (mask > 0) & (depth_m > 0.1) & (depth_m < 2.0)
    vs, us = np.where(ok)
    return fit_plane_inverse_depth(us, vs, depth_m[vs, us], *intr)


def button_xyz_cam(px, n, d, intr):
    """Camera-frame point where the button pixel's ray meets the plane n.X = d."""
    ray = pixel_ray(px, *intr)
    return ray * (d / float(np.dot(n, ray)))
