"""Pure geometry behind the autonomous button press (button_press.geometry).

The panel plane sets the approach travel cap, so a wrong tilt or distance there is a
wrong stopping point. The 2026-09-21 failure this guards: at a 20 cm standoff with
depth noise larger than the depth signal, an SVD fit read the noise as a 61 deg tilt;
the inverse-depth OLS fit recovers the true ~19 deg.
"""

import numpy as np
import pytest

from feeding_deployment.button_press.geometry import (
    PlaneFitError,
    fit_plane_inverse_depth,
    lateral_correction_cam,
    max_joint_delta_deg,
    pixel_ray,
    ray_plane_distance,
)

# D435i colour intrinsics at 640x480, roughly.
FX = FY = 615.0
CX, CY = 320.0, 240.0


def _plane_pixels(n, d, rng, noise_m=0.0, box=(200, 150, 440, 330), step=3):
    n = np.asarray(n, float) / np.linalg.norm(n)
    u0, v0, u1, v1 = box
    us, vs = np.meshgrid(np.arange(u0, u1, step), np.arange(v0, v1, step))
    us, vs = us.ravel().astype(float), vs.ravel().astype(float)
    rays = np.stack([(us - CX) / FX, (vs - CY) / FY, np.ones_like(us)], axis=1)
    z = d / (rays @ n)  # depth along the optical axis where each pixel ray meets the plane
    return us, vs, z + rng.normal(0.0, noise_m, size=z.shape), n


def _angle_deg(a, b):
    return float(np.degrees(np.arccos(np.clip(abs(np.dot(a, b)), -1, 1))))


def test_plane_exact():
    rng = np.random.default_rng(0)
    us, vs, z, n_true = _plane_pixels([0.15, 0.29, -0.95], -0.20, rng)
    n, d, z_med, npts = fit_plane_inverse_depth(us, vs, z, FX, FY, CX, CY)
    assert _angle_deg(n, n_true) < 0.01
    assert n[2] < 0, "normal must point back toward the camera"
    # n.X = d with n toward the camera: d is negative for a plane in front of it.
    assert d == pytest.approx(-0.20 * np.sign(np.dot(n, n_true)), abs=1e-6)
    assert 0.15 < z_med < 0.30 and npts == len(us)


def test_plane_close_range_noise_does_not_invent_tilt():
    """Per-pixel noise (0.64 cm) larger than the vertical depth signal, as on 2026-09-21."""
    rng = np.random.default_rng(1)
    us, vs, z, n_true = _plane_pixels([0.15, 0.29, -0.95], -0.20, rng, noise_m=0.0064)
    n, *_ = fit_plane_inverse_depth(us, vs, z, FX, FY, CX, CY)
    assert _angle_deg(n, n_true) < 3.0


def test_plane_ignores_background_through_gaps():
    rng = np.random.default_rng(2)
    us, vs, z, n_true = _plane_pixels([0.0, 0.0, -1.0], -0.21, rng, noise_m=0.002)
    z = z.copy()
    z[:: 50] = 0.37  # ~2% of pixels see the wall behind the panel
    n, d, *_ = fit_plane_inverse_depth(us, vs, z, FX, FY, CX, CY)
    assert _angle_deg(n, n_true) < 1.0
    assert abs(d) == pytest.approx(0.21, abs=0.003)


def test_plane_rejects_too_few_points_and_non_planar():
    rng = np.random.default_rng(3)
    with pytest.raises(PlaneFitError):
        fit_plane_inverse_depth([1.0] * 50, [1.0] * 50, [0.2] * 50, FX, FY, CX, CY)
    us, vs, _, _ = _plane_pixels([0, 0, -1], -0.2, rng)
    z = 0.2 + rng.uniform(-0.03, 0.03, size=us.shape)  # 6 cm of scatter: not a panel
    with pytest.raises(PlaneFitError):
        fit_plane_inverse_depth(us, vs, z, FX, FY, CX, CY)


def test_ray_plane_distance_matches_geometry():
    n = np.array([0.0, 0.0, -1.0])
    ray = pixel_ray((CX, CY), FX, FY, CX, CY)
    np.testing.assert_allclose(ray, [0, 0, 1])
    s, denom = ray_plane_distance(n, -0.2, ray)
    assert s == pytest.approx(0.2) and denom == pytest.approx(-1.0)
    off = pixel_ray((CX + FX, CY), FX, FY, CX, CY)  # 45 deg off-axis
    s, _ = ray_plane_distance(n, -0.2, off)
    assert s == pytest.approx(0.2 * np.sqrt(2))


def test_lateral_correction_sign_scale_and_cap():
    # Button 10 px right of the claw at 20 cm -> move the camera +x by 10*0.2/fx.
    d = lateral_correction_cam(np.array([10.0, 0.0]), 0.2, FX, FY, cap=0.03)
    np.testing.assert_allclose(d, [10 * 0.2 / FX, 0, 0])
    d = lateral_correction_cam(np.array([-600.0, 800.0]), 0.3, FX, FY, cap=0.03)
    assert np.linalg.norm(d) == pytest.approx(0.03)
    assert d[0] < 0 < d[1] and d[2] == 0


def test_max_joint_delta_wraps():
    assert max_joint_delta_deg([np.radians(179)] * 7, [np.radians(-179)] * 7) == pytest.approx(2.0)
    assert max_joint_delta_deg([0, 0, 0.1, 0, 0, 0, 0], [0] * 7) == pytest.approx(np.degrees(0.1))
