"""The open microwave's cavity from the back wall + the front face + the opening's outline.

The alternative to `cavity_perception.estimate_cavity` (cavity_method: opening), for a view that does not
show the floor: a scene camera further back and lower down. It needs neither the floor nor the side walls,
and no depth inside the cavity except on the back wall. Any camera works; the wrist camera tests it too.

  back wall   largest camera-facing RANSAC plane in the far part of the cavity-mask depth, within
              back_wall_vertical_tol_deg of vertical (the Comfee's slopes ~12 deg). Insertion axis f =
              -normal, flattened. Bound = where the plane comes nearest within the cavity's height (a wall
              leaning away at the top: its foot, where the container ends up, even if the foot is hidden),
              or the surface measured just above the floor if that is nearer (the cove).
  front face  NOT a free fit (a control panel stands proud of the bezel and tilts one ~10 deg): a vertical
              plane parallel to the back wall, at the most common depth along f of the "microwave" pixels
              outside the (dilated) cavity and "microwave door" masks.
  opening     SAM 3's cavity outline ray-cast onto the front-face plane. The interior is seen through the
              opening, so the outline IS its rim, from any viewpoint, and needs no depth there.
  left/right  the opening's sides. Top: its top edge.
  floor       the opening's bottom lip. The back wall's plane cannot give it: a real cavity's lower corners
              are coved (10-30 rig: the wall leaves its plane ~4 cm above the floor and comes 3 cm forward),
              so that plane stops short. Its lowest point only sanity-checks the lip, from the camera's line
              of sight over the lip. This is the shell's floor; a turntable on it is not seen unless depth
              reaches it (floor_target: blind_support_height_m).
  tilt        the bottom lip's slope across the opening, the check on the camera calibration's roll that
              the fitted floor gives estimate_cavity. It cannot see a pitch.

Every failure is a CavityError with the reason. The result is a `Cavity`, so `merge_cavities` and
`floor_target.placement_target` are shared with estimate_cavity. Its floor plane is flat (normal = +z).
Frame: frame.base_frame, which must be gravity-aligned (+z up).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from cavity_perception import Cavity, CavityError, Z, _horizontal
from cavity_segmentation import largest_component


@dataclass
class OpeningConfig:
    depth_min_m: float = 0.15
    depth_max_m: float = 4.0
    far_percentile: float = 60.0       # the back wall is searched beyond this percentile along the view
    back_dist_m: float = 0.006         # RANSAC inlier distance (raise for a far camera's noisier depth)
    back_iters: int = 800
    max_ransac_points: int = 20000     # subsampled above this
    min_back_points: int = 300
    min_front_points: int = 200
    front_bin_m: float = 0.005         # depth histogram of the front-face candidates
    front_band_m: float = 0.01         # points within this of the peak are the front face
    roi_pad: float = 0.6               # front-face candidates: cavity box grown by this share of its size
    cavity_dilate_px: int = 7          # candidates stay this far outside the cavity mask ...
    door_dilate_px: int = 4            # ... and the door mask ("microwave" includes the open door)
    max_border_frac: float = 0.05      # outline share allowed on the image edge (that part is not the rim)
    floor_agree_m: float = 0.015       # back wall's lowest point vs the line of sight over the lip ...
    wall_cove_m: float = 0.05          # ... which may sit this much higher (coved corner, turntable)
    cove_reach_m: float = 0.06         # points this close to the wall's plane, low down, are its cove


def _dilate(mask, px):
    import cv2
    return cv2.dilate(mask.astype(np.uint8), np.ones((2 * px + 1, 2 * px + 1), np.uint8)) > 0


def fit_back_wall(pts, cam_pos, view, ccfg, ocfg, rng):
    """(normal facing the camera, point on the plane) of the largest back-wall candidate in `pts`, or None."""
    sin_tol = np.sin(np.radians(ccfg.back_wall_vertical_tol_deg))
    cos_view = np.cos(np.radians(ccfg.back_wall_max_view_angle_deg))
    best_n, best_p, best_count = None, None, 0
    for _ in range(ocfg.back_iters):
        s = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n = n / norm
        if abs(n @ Z) > sin_tol:
            continue
        if n @ (cam_pos - s[0]) < 0:
            n = -n
        if -(n @ view) < cos_view:
            continue
        count = int(np.count_nonzero(np.abs((pts - s[0]) @ n) < ocfg.back_dist_m))
        if count > best_count:
            best_n, best_p, best_count = n, s[0], count
    if best_n is None:
        return None
    q = pts[np.abs((pts - best_p) @ best_n) < ocfg.back_dist_m]
    c = q.mean(axis=0)
    n = np.linalg.svd(q - c, full_matrices=False)[2][-1]
    return (n if n @ best_n > 0 else -n), c


def front_face(candidates, back, f, ccfg, ocfg):
    """Position along f of the plane parallel to the back wall through the candidates, and its points."""
    s = candidates @ f
    lo, hi = back - ccfg.max_depth_m, back - ccfg.min_depth_m
    win = (s > lo) & (s < hi)
    if np.count_nonzero(win) < ocfg.min_front_points:
        raise CavityError(f"only {np.count_nonzero(win)} microwave-face points {ccfg.min_depth_m * 100:.0f}-"
                          f"{ccfg.max_depth_m * 100:.0f} cm in front of the back wall (need {ocfg.min_front_points}) "
                          "-- is the door open and the front of the microwave in view?")
    hist, edges = np.histogram(s[win], bins=np.arange(lo, hi + 1e-9, ocfg.front_bin_m))
    peak = edges[np.argmax(np.convolve(hist, [1, 2, 1], "same"))] + ocfg.front_bin_m / 2
    on = win & (np.abs(s - peak) < ocfg.front_band_m)
    if np.count_nonzero(on) < ocfg.min_front_points:
        raise CavityError(f"no front-face plane: {np.count_nonzero(on)} points at the most common depth "
                          f"(need {ocfg.min_front_points})")
    return float(np.median(s[on])), candidates[on]


def opening_outline(cavity_mask, rays_base, cam_pos, f, front, ocfg):
    """The cavity mask's outline ray-cast onto the plane f = front (the opening's rim, Nx3)."""
    import cv2
    h, w = cavity_mask.shape
    cnt, _ = cv2.findContours(cavity_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnt:
        raise CavityError("empty cavity mask")
    px = max(cnt, key=len)[:, 0, :]
    on_border = (px[:, 0] <= 1) | (px[:, 0] >= w - 2) | (px[:, 1] <= 1) | (px[:, 1] >= h - 2)
    if on_border.mean() > ocfg.max_border_frac:
        raise CavityError(f"{on_border.mean() * 100:.0f}% of the cavity outline is on the image edge "
                          f"(max {ocfg.max_border_frac * 100:.0f}%): the whole opening must be in view")
    px = px[~on_border]
    r = rays_base[px[:, 1], px[:, 0]]
    denom = r @ f
    ok = denom > 1e-6
    t = (front - cam_pos @ f) / denom[ok]
    return cam_pos + r[ok] * t[:, None]


def lip_tilt(outline, lip, mid_l, width, l):
    """(degrees, RMS m) of the bottom lip across the middle of the opening, or None if too little of it."""
    pl = outline @ l
    sel = (outline[:, 2] < lip + 0.03) & (np.abs(pl - mid_l) < 0.4 * width)
    if np.count_nonzero(sel) < 20:
        return None
    a, b = np.polyfit(pl[sel], outline[sel, 2], 1)
    return float(np.degrees(np.arctan(abs(a)))), float(np.std(outline[sel, 2] - (a * pl[sel] + b)))


def estimate_opening_cavity(frame, cmask, ccfg, ocfg: OpeningConfig = None, seed=0):
    """Cavity from one frame and its CavityMask (with the microwave mask; the door mask is optional).
    Raises CavityError."""
    ocfg = ocfg or OpeningConfig()
    if cmask.microwave is None:
        raise CavityError("no 'microwave' mask (needed for the front face)")
    rng = np.random.default_rng(seed)
    cav_m, _ = largest_component(cmask.sam_mask)
    depth, K, T = frame.depth_m.astype(float), frame.K, frame.base_T_cam
    cam = T[:3, 3].astype(float)
    h, w = depth.shape
    vv, uu = np.mgrid[0:h, 0:w]
    rays_cam = np.stack([(uu - K[0, 2]) / K[0, 0], (vv - K[1, 2]) / K[1, 1], np.ones_like(uu, float)], -1)
    rays = rays_cam @ T[:3, :3].T
    ok = np.isfinite(depth) & (depth > ocfg.depth_min_m) & (depth < ocfg.depth_max_m)
    P = rays * depth[..., None] + cam

    # back wall
    cav_pts = P[cav_m & ok]
    if len(cav_pts) < ocfg.min_back_points:
        raise CavityError(f"only {len(cav_pts)} cavity pixels have depth (need {ocfg.min_back_points})")
    view = _horizontal(cav_pts.mean(axis=0) - cam)
    if view is None:
        raise CavityError("camera is looking straight down -- no insertion direction")
    along = cav_pts @ view
    far = cav_pts[along > np.percentile(along, ocfg.far_percentile)]
    if len(far) > ocfg.max_ransac_points:
        far = far[rng.choice(len(far), ocfg.max_ransac_points, replace=False)]
    fit = fit_back_wall(far, cam, view, ccfg, ocfg, rng) if len(far) >= 3 else None
    if fit is None:
        raise CavityError("no back wall: no camera-facing near-vertical plane in the far part of the cavity")
    n_b, c_b = fit
    back_pts = cav_pts[np.abs((cav_pts - c_b) @ n_b) < ocfg.back_dist_m]
    if len(back_pts) < ocfg.min_back_points:
        raise CavityError(f"back wall has only {len(back_pts)} points (need {ocfg.min_back_points})")
    f = _horizontal(-n_b)
    if f is None:
        raise CavityError("back wall normal is vertical")
    l = np.cross(Z, f)
    back = float(np.median(back_pts @ f))         # refined from the plane once floor and top are known

    # front face
    ys, xs = np.nonzero(cav_m)
    pad_x, pad_y = int(ocfg.roi_pad * (xs.max() - xs.min())), int(ocfg.roi_pad * (ys.max() - ys.min()))
    roi = np.zeros_like(cav_m)
    roi[max(0, ys.min() - pad_y):ys.max() + pad_y, max(0, xs.min() - pad_x):xs.max() + pad_x] = True
    keep = np.asarray(cmask.microwave, bool) & ~_dilate(cav_m, ocfg.cavity_dilate_px) & roi & ok
    if cmask.door is not None:
        keep &= ~_dilate(np.asarray(cmask.door, bool), ocfg.door_dilate_px)
    front, front_pts = front_face(P[keep], back, f, ccfg, ocfg)

    # opening
    outline = opening_outline(cav_m, rays, cam, f, front, ocfg)
    if len(outline) < 40:
        raise CavityError(f"only {len(outline)} outline points on the front face")
    ol, oz = outline @ l, outline[:, 2]
    left, right = float(np.percentile(ol, 98)), float(np.percentile(ol, 2))
    top, lip = float(np.percentile(oz, 98)), float(np.percentile(oz, 2))
    width = left - right
    if not ccfg.min_width_m <= width <= ccfg.max_width_m:
        raise CavityError(f"opening width {width * 100:.1f} cm outside "
                          f"{ccfg.min_width_m * 100:.0f}-{ccfg.max_width_m * 100:.0f}")

    # floor, top, tilt
    # below the lip nothing real can be on the wall: those are the wall mirrored in a glass turntable (10-30 rig)
    real = back_pts[back_pts[:, 2] >= lip - ocfg.floor_agree_m, 2]
    if len(real) < ocfg.min_back_points:
        raise CavityError(f"back wall has only {len(real)} points above the opening's bottom lip z {lip:.3f}")
    foot = float(np.percentile(real, 2))
    sight = lip + (lip - cam[2]) * (back - front) / (front - cam @ f)   # the line of sight over the lip, at the wall
    foot_hidden = sight > lip + 0.002
    expected = sight if foot_hidden else lip
    if not -ocfg.floor_agree_m <= foot - expected <= ocfg.floor_agree_m + ocfg.wall_cove_m:
        raise CavityError(f"back wall's lowest point z {foot:.3f} vs {expected:.3f} expected from the opening's bottom "
                          f"lip z {lip:.3f} ({(foot - expected) * 100:+.1f} cm) "
                          "-- something on the floor, or a bad mask or calibration")
    floor_z = lip
    if not ccfg.floor_z_range[0] <= floor_z <= ccfg.floor_z_range[1]:
        raise CavityError(f"floor z {floor_z:.3f} outside {ccfg.floor_z_range}")
    height = top - floor_z
    if not ccfg.min_height_m <= height <= ccfg.max_height_m:
        raise CavityError(f"cavity height {height * 100:.1f} cm outside "
                          f"{ccfg.min_height_m * 100:.0f}-{ccfg.max_height_m * 100:.0f}")
    plane_at = [float((n_b @ c_b - n_b[2] * z) / (n_b @ f)) for z in (floor_z, top)]
    back = min(plane_at)
    low = (cav_pts[:, 2] > floor_z + 0.005) & (cav_pts[:, 2] < floor_z + 0.03) \
        & (np.abs((cav_pts - c_b) @ n_b) < ocfg.cove_reach_m)
    if np.count_nonzero(low) >= 30:
        back = min(back, float(np.median(cav_pts[low] @ f)))
    if not ccfg.min_depth_m <= back - front <= ccfg.max_depth_m:
        raise CavityError(f"cavity depth {(back - front) * 100:.1f} cm outside "
                          f"{ccfg.min_depth_m * 100:.0f}-{ccfg.max_depth_m * 100:.0f}")
    tilt = lip_tilt(outline, lip, 0.5 * (left + right), width, l)
    if tilt is None:
        raise CavityError("the opening's bottom lip is not visible enough to check the calibration's roll")
    if tilt[0] > ccfg.floor_max_tilt_deg:
        raise CavityError(f"the opening's bottom lip is {tilt[0]:.1f} deg from level (max {ccfg.floor_max_tilt_deg}) -- "
                          "camera calibration (roll) or a bad mask; refusing")

    mid = f * (0.5 * (front + back)) + l * (0.5 * (left + right)) + Z * floor_z
    return Cavity(f=f, l=l, front=front, back=back, left=left, right=right, floor_z=floor_z, top=top,
                  floor_normal=Z.copy(), floor_rms=tilt[1], floor_points=mid[None, :],
                  sources={"back": "back wall plane", "front": "plane parallel to the back wall",
                           "left": "opening outline", "right": "opening outline", "top": "opening outline",
                           "floor": f"bottom lip (wall's lowest point {foot:.3f}{', hidden' if foot_hidden else ''})",
                           "lip_tilt_deg": round(tilt[0], 2), "back_slope_deg": round(float(np.degrees(np.arcsin(abs(n_b @ Z)))), 1),
                           "n_back": len(back_pts), "n_front": len(front_pts),
                           "door_mask": cmask.door is not None})
