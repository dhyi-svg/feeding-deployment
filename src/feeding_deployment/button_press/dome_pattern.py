"""Far-range button finder for the Comfee panel: find the 5 chrome domes by their 3+2 layout.

Pure OpenCV/numpy, no ROS, no reference images. Complements the SIFT reference detector
(``detector_node``), which is accurate at 17-22 cm but loses its lock further out -- on
2026-10-03 at 39 cm it held only 6-8 inliers (preflight needs 8 for 2 s). This one is for the
COARSE stage: find the panel from wherever the arm is, so press_button can stage at ~22 cm and
hand over to SIFT for the fine measurement.

How it works:
  1. candidates  connected blobs that are desaturated and clearly brighter than the panel's own
                 red (relative, so it follows exposure), with red around them
  2. layout fit  for every ordered candidate pair taken as (top-middle, top-right) dome, predict
                 the other three domes from the known layout (top row of 3 at spacing s; bottom
                 row of 2, 0.92 s lower, offset half a step) and count candidates that land within
                 FIT_TOL * s. A fit must hit all 5 AND its spacing must measure DOME_SPACING_MM
                 in metres via depth -- label text and knob highlights fail one or the other.
                 3 domes on top vs 2 below fixes "up", so a rolled camera still names them right.
  3. 3D          panel plane fitted (geometry.fit_plane_inverse_depth) to the red pixels around
                 the domes -- not to the domes, where the depth sensor lies on chrome.

Tested 2026-10-03 on the 5 saved reference frames + 2 live ones: 5/5 domes named correctly from
~25 to ~50 cm, 3-4 px from the hand-marked centres far out. Up close (~17 cm) a dome is no longer
one highlight blob and the fit locked onto label text -- hence MIN_RANGE_M; SIFT owns that range.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from feeding_deployment.button_press.geometry import fit_plane_inverse_depth, pixel_ray

# Physical top-row pitch on the Comfee panel (measured 19.3 mm at 39 cm, 2026-10-03).
DOME_SPACING_MM = (16.8, 21.8)
NAMES = ("power_level", "wgt_time_defrost", "timer_clock", "stop_eco", "start_30s")
# Dome centres in units of the top-row spacing, origin = top-middle dome, x right, y DOWN the panel.
LAYOUT = np.array([[-1.0, 0.0], [0.0, 0.0], [1.0, 0.0], [-0.58, 0.92], [0.42, 0.92]])
FIT_TOL = 0.2                 # max dome miss, fraction of s (true fits: <= 0.17 on the test frames)
MIN_RANGE_M = 0.22            # closer than this, domes break into several highlights: use SIFT
MAX_RANGE_M = 1.2
# Region (layout units) whose red pixels the panel plane is fitted to: the button block and the
# red panel around it, not the knob/display above or the microwave's bottom edge.
PLANE_REGION = np.array([[-2.2, -1.2], [2.2, -1.2], [2.2, 2.0], [-2.2, 2.0]])


@dataclass
class DomeFit:
    px: dict                  # name -> (u, v)
    s_px: float
    s_mm: float
    err: float                # mean dome miss / s
    z: float                  # median depth at the domes (m)
    region: np.ndarray        # 4x2 image polygon for the plane fit


def _masks(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = ((H < 10) | (H > 170)) & (S > 110) & (V > 35)
    v_red = float(np.median(V[red])) if red.any() else 100.0
    # Capped: under bright light 1.3x a well-lit red would exceed 255 and nothing could be chrome.
    chrome = (S < 90) & (V > max(60.0, min(1.3 * v_red, 220.0)))
    return red, chrome


def candidates(bgr, depth_m, fx):
    """[(u, v, z)] chrome blobs with red around them and a usable depth nearby."""
    red, chrome = _masks(bgr)
    n, _, st, cen = cv2.connectedComponentsWithStats(chrome.astype(np.uint8))
    h_img, w_img = red.shape
    out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < 6 or max(w, h) > 0.15 * w_img:
            continue
        cx, cy = cen[i]
        r = 1.6 * max(w, h)
        x0, x1 = int(max(0, cx - r)), int(min(w_img, cx + r))
        y0, y1 = int(max(0, cy - r)), int(min(h_img, cy + r))
        if red[y0:y1, x0:x1].mean() < 0.5:
            continue
        patch = depth_m[y0:y1, x0:x1]
        patch = patch[(patch > 0.1) & (patch < 2.0)]
        if patch.size < 5:
            continue
        out.append((float(cx), float(cy), float(np.median(patch))))
    return out


def fit_layout(cands, fx):
    """Best all-5 fit of LAYOUT to the candidates, or None."""
    if len(cands) < 5:
        return None
    P = np.array([[c[0], c[1]] for c in cands])
    Z = np.array([c[2] for c in cands])
    best = None
    for i in range(len(P)):
        for j in range(len(P)):
            if i == j:
                continue
            ex = P[j] - P[i]
            s = float(np.linalg.norm(ex))
            s_mm = s * Z[i] / fx * 1000.0
            if not DOME_SPACING_MM[0] <= s_mm <= DOME_SPACING_MM[1]:
                continue
            ex = ex / s
            ey = np.array([-ex[1], ex[0]])   # panel "down" in the image when x is right
            pred = P[i] + s * (LAYOUT[:, :1] * ex + LAYOUT[:, 1:] * ey)
            dist = np.linalg.norm(P[None] - pred[:, None], axis=2)
            k = dist.argmin(1)
            e = dist[np.arange(5), k] / s
            if (e >= FIT_TOL).any() or len(set(k)) < 5:
                continue
            if best is None or e.mean() < best[0]:
                region = P[i] + s * (PLANE_REGION[:, :1] * ex + PLANE_REGION[:, 1:] * ey)
                best = (float(e.mean()), P[k], s, s_mm, float(np.median(Z[k])), region)
    if best is None:
        return None
    err, pts, s, s_mm, z, region = best
    return DomeFit(px={n: tuple(map(float, p)) for n, p in zip(NAMES, pts)}, s_px=s, s_mm=s_mm, err=err,
                   z=z, region=region)


def detect(bgr, depth_m, fx):
    """DomeFit for this frame, or None (no fit, or outside MIN_RANGE_M..MAX_RANGE_M)."""
    fit = fit_layout(candidates(bgr, depth_m, fx), fx)
    if fit is None or not MIN_RANGE_M <= fit.z <= MAX_RANGE_M:
        return None
    return fit


def panel_plane(bgr, depth_m, fit: DomeFit, intr):
    """Plane n.X = d (camera frame) through the red panel pixels around the domes."""
    red, chrome = _masks(bgr)
    mask = np.zeros(red.shape, np.uint8)
    cv2.fillConvexPoly(mask, fit.region.astype(np.int32), 1)
    ok = (mask > 0) & red & ~chrome & (depth_m > 0.1) & (depth_m < 2.0)
    vs, us = np.where(ok)
    return fit_plane_inverse_depth(us, vs, depth_m[vs, us], *intr)


def button_xyz_cam(px, n, d, intr):
    """Camera-frame point where the button pixel's ray meets the plane n.X = d."""
    ray = pixel_ray(px, *intr)
    return ray * (d / float(np.dot(n, ray)))
