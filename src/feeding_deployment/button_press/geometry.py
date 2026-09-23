"""Pure geometry for the button press: panel plane fit, pixel rays, servo corrections.

No ROS, no arm, no pybullet -- only numpy -- so it is unit-testable off-robot. The
ROS-facing wrappers live in ``autonomous_press.Perception``.
"""

from __future__ import annotations

import numpy as np

# Drop depths this far off the median (background seen through gaps around the panel).
PLANE_Z_BAND_M = 0.05
# Refit after dropping points this far off the first fit.
PLANE_TRIM_M = 0.02
PLANE_MIN_POINTS = 200
PLANE_MAX_RESID_M = 0.01


class PlaneFitError(ValueError):
    """The depth pixels do not support a trustworthy plane."""


def fit_plane_inverse_depth(us, vs, z, fx, fy, cx, cy):
    """Plane ``n.X = d`` (camera frame) through depth pixels ``(us, vs, z)``.

    Fit by ORDINARY least squares on the depth map, not total-least-squares (SVD) on
    the 3D points. u,v are exact; all the noise is in z. A plane is exactly linear in
    inverse depth: 1/z = m.[u',v',1] with m = n/d. TLS instead assumes isotropic noise,
    and on 2026-09-21 at the 20 cm standoff that broke badly -- the vertical depth
    signal across the panel was only 0.30 cm against 0.64 cm per-pixel noise, so SVD
    read the weak axis as geometry and returned a 61 deg tilt ([0.11 0.88 -0.47]). OLS
    averages the noise down and recovers [0.15 0.29 -0.95] (19 deg), which matches both
    the fits taken further out and the quad's 1.019 keystone ratio.

    The normal is oriented back toward the camera (``n[2] < 0``).
    Returns ``(n, d, median_depth, n_points)``; raises :class:`PlaneFitError`.
    """
    us = np.asarray(us, dtype=float)
    vs = np.asarray(vs, dtype=float)
    zz = np.asarray(z, dtype=float)
    if len(zz) < PLANE_MIN_POINTS:
        raise PlaneFitError(f"only {len(zz)} valid panel depth pixels")
    up = (us - cx) / fx
    vp = (vs - cy) / fy
    # Background seen through gaps around the panel is only ~2% of the pixels but sits
    # at ~37 cm vs the panel's ~21 cm, and 1/z gives it enormous leverage. Drop it first.
    keep0 = np.abs(zz - np.median(zz)) <= PLANE_Z_BAND_M
    if keep0.sum() >= PLANE_MIN_POINTS:
        up, vp, zz = up[keep0], vp[keep0], zz[keep0]
    m = None
    for _ in range(2):
        A = np.stack([up, vp, np.ones_like(up)], axis=1)
        m, *_ = np.linalg.lstsq(A, 1.0 / zz, rcond=None)
        pred = 1.0 / (A @ m)
        keep = np.abs(zz - pred) <= PLANE_TRIM_M
        if keep.sum() < PLANE_MIN_POINTS or bool(keep.all()):
            break
        up, vp, zz = up[keep], vp[keep], zz[keep]
    A = np.stack([up, vp, np.ones_like(up)], axis=1)
    resid = float(np.std(zz - 1.0 / (A @ m)))
    nrm = float(np.linalg.norm(m))
    n, d_plane = m / nrm, 1.0 / nrm
    if n[2] > 0:  # make the normal point back toward the camera (-z); d flips with it
        n, d_plane = -n, -d_plane
    if resid > PLANE_MAX_RESID_M:
        raise PlaneFitError(f"residual {resid*100:.1f} cm -- not planar / bad depth")
    return n, d_plane, float(np.median(zz)), len(zz)


def pixel_ray(px, fx, fy, cx, cy) -> np.ndarray:
    """Unit ray (camera optical frame) through pixel ``px``."""
    r = np.array([(px[0] - cx) / fx, (px[1] - cy) / fy, 1.0])
    return r / np.linalg.norm(r)


def ray_plane_distance(n, d, ray) -> tuple[float, float]:
    """``(s, n.r)``: distance along unit ``ray`` from the camera to plane ``n.X = d``."""
    denom = float(np.dot(n, ray))
    return (d / denom if denom != 0 else float("inf")), denom


def lateral_correction_cam(e_px, z, fx, fy, cap) -> np.ndarray:
    """Camera-frame in-plane translation that moves a pixel error ``e_px`` to zero.

    Translating the camera +x makes a static point's u decrease, so the correction is
    +e (button right of claw -> move right). Scaled by the button's depth ``z`` and
    capped at ``cap`` metres.
    """
    d_cam = np.array([e_px[0] * z / fx, e_px[1] * z / fy, 0.0])
    mag = float(np.linalg.norm(d_cam))
    if mag > cap:
        d_cam *= cap / mag
    return d_cam


def max_joint_delta_deg(q_a, q_b) -> float:
    """Largest per-joint difference in degrees, with angle wrap-around."""
    diff = np.asarray(q_a, dtype=float) - np.asarray(q_b, dtype=float)
    return float(np.degrees(np.max(np.abs((diff + np.pi) % (2 * np.pi) - np.pi))))
