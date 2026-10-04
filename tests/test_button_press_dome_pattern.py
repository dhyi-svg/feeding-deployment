"""Tests for the far-range dome-layout detector, on synthetic panels (no camera)."""
import cv2
import numpy as np

from feeding_deployment.button_press import dome_pattern as dp

FX = 606.0
RED = (30, 30, 200)       # BGR


def panel(z=0.4, roll_deg=0.0, centre=(320, 260), distractors=True, pitch_mm=19.3):
    """Red panel with the 5 domes drawn at the true layout for depth z. Returns (bgr, depth, truth px)."""
    img = np.full((480, 640, 3), RED, np.uint8)
    s = pitch_mm / 1000 * FX / z
    a = np.radians(roll_deg)
    ex, ey = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    truth = {}
    for name, (u, v) in zip(dp.NAMES, dp.LAYOUT):
        p = np.array(centre) + s * (u * ex + v * ey)
        truth[name] = p
        cv2.circle(img, tuple(int(round(c)) for c in p), max(2, int(0.25 * s)), (225, 225, 225), -1)
    if distractors:
        # label text: small bright marks between and above the domes, at a smaller pitch
        for k in range(6):
            p = np.array(centre) + s * ((-1.2 + 0.45 * k) * ex + (-0.55) * ey)
            cv2.rectangle(img, tuple(int(c) for c in p - 1), tuple(int(c) for c in p + 1), (230, 230, 230), -1)
    return img, np.full((480, 640), z, np.float32), truth


def check(fit, truth, tol_px=1.5):
    assert fit is not None
    for name in dp.NAMES:
        assert np.linalg.norm(np.array(fit.px[name]) - truth[name]) < tol_px, name


def test_finds_and_names_all_five():
    img, d, truth = panel()
    fit = dp.detect(img, d, FX)
    check(fit, truth)
    assert abs(fit.s_mm - 19.3) < 1.0


def test_far_and_mid_range():
    for z in (0.3, 0.5, 0.7):
        img, d, truth = panel(z=z)
        check(dp.detect(img, d, FX), truth)


def test_rolled_camera_still_names_correctly():
    # 3 domes on top vs 2 below fixes "up": upside down, timer_clock is still the right end of the 3-row.
    for roll in (25.0, 180.0):
        img, d, truth = panel(roll_deg=roll)
        check(dp.detect(img, d, FX), truth)


def test_wrong_metric_size_rejected():
    # Same picture, but depth says it is half as far: the pitch measures ~9.7 mm, not a Comfee panel.
    img, _, _ = panel(z=0.4)
    assert dp.detect(img, np.full((480, 640), 0.2, np.float32), FX) is None


def test_too_close_abstains():
    img, d, _ = panel(z=0.4)
    fit = dp.fit_layout(dp.candidates(img, d, FX), FX)
    assert fit is not None
    # identical geometry reported at 18 cm is below MIN_RANGE_M -> detect() abstains
    img2, d2, _ = panel(z=0.18, centre=(320, 200))
    assert dp.detect(img2, d2, FX) is None


def test_four_domes_is_not_enough():
    img, d, truth = panel(distractors=False)
    p = truth["stop_eco"]
    cv2.circle(img, tuple(int(c) for c in p), 12, RED, -1)   # paint one dome out
    assert dp.detect(img, d, FX) is None


def test_button_xyz_on_plane():
    intr = (FX, FX, 320.0, 240.0)
    n, dd = np.array([0.0, 0.0, -1.0]), -0.4          # fronto-parallel plane at z = 0.4 (n toward camera)
    x = dp.button_xyz_cam((320.0 + 60.6, 240.0), n, dd, intr)
    np.testing.assert_allclose(x, [0.04, 0.0, 0.4], atol=1e-9)
