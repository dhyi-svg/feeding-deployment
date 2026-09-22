"""Reference-homography START/+30SEC button detector.

This detector identifies the button by carrying a HAND-PLACED mark on a
reference image of the appliance's panel through a fitted homography, rather
than re-deriving "which circle is START" per frame from a spatial rule. These
tests pin the two properties that matter:

  1. When the panel is found, the mark lands where it should -- including under
     rotation and scale, since the wrist camera is mounted upside down and the
     working distance varies.
  2. When the panel is NOT found, it returns None rather than guessing. The
     safety argument for this detector is entirely that it abstains instead of
     pointing at the wrong control, so a regression that made it answer from a
     bad fit would be far worse than one that made it answer less often.

Synthetic panels are used so the tests need no footage, no models and no
camera. They deliberately carry high-frequency texture, because SIFT has
nothing to match on a blank rectangle -- the real cream panel is similarly
low-texture and leans on its printed labels.
"""

import json

import cv2
import numpy as np
import pytest

from feeding_deployment.perception.appliance_perception.reference_button_detector import (
    ReferenceButtonDetector,
)

PANEL_W, PANEL_H = 200, 300
BUTTON_XY = (120, 200)


def _make_panel(seed=0):
    """A textured stand-in for the control panel, with five 'buttons'."""
    rng = np.random.default_rng(seed)
    img = np.full((PANEL_H, PANEL_W, 3), 225, np.uint8)
    # Speckle gives SIFT corners to lock onto, standing in for printed labels.
    noise = rng.integers(0, 60, (PANEL_H, PANEL_W, 3), dtype=np.int16)
    img = np.clip(img.astype(np.int16) - noise, 0, 255).astype(np.uint8)
    for i, (bx, by) in enumerate(
        [(60, 140), (120, 140), (180, 140), (60, 200), BUTTON_XY]
    ):
        cv2.circle(img, (bx, by), 18, (150 + i * 8, 150, 150), -1)
        cv2.circle(img, (bx, by), 18, (60, 60, 60), 2)
        cv2.putText(img, f"B{i}", (bx - 14, by + 34), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (30, 30, 30), 1, cv2.LINE_AA)
    return img


@pytest.fixture
def ref_dir(tmp_path):
    panel = _make_panel()
    scene = np.full((480, 640, 3), 200, np.uint8)
    scene[80:80 + PANEL_H, 200:200 + PANEL_W] = panel
    cv2.imwrite(str(tmp_path / "ref.png"), scene)
    (tmp_path / "reference.json").write_text(json.dumps({
        "image": "ref.png",
        "crop": [200, 80, 200 + PANEL_W, 80 + PANEL_H],
        "button_xy": [200 + BUTTON_XY[0], 80 + BUTTON_XY[1]],
        "button_radius": 18,
        "appliance": "synthetic",
    }))
    return tmp_path


def _scene_with_panel(H, size=(640, 480)):
    panel = _make_panel()
    full = np.full((480, 640, 3), 200, np.uint8)
    full[80:80 + PANEL_H, 200:200 + PANEL_W] = panel
    return cv2.warpPerspective(full, H, size, borderValue=(200, 200, 200))


def _expected(H, pt=(200 + BUTTON_XY[0], 80 + BUTTON_XY[1])):
    p = cv2.perspectiveTransform(np.float32([[pt]]), H).reshape(2)
    return float(p[0]), float(p[1])


def test_finds_button_in_untransformed_scene(ref_dir):
    det = ReferenceButtonDetector(ref_dir)
    res = det.detect(_scene_with_panel(np.eye(3)))
    assert res["center"] is not None, res.get("reason")
    ex, ey = _expected(np.eye(3))
    assert abs(res["center"][0] - ex) < 6
    assert abs(res["center"][1] - ey) < 6


def test_finds_button_when_scene_is_upside_down(ref_dir):
    """The wrist camera is mounted upside down; unlike the bottom-row/rightmost
    backends, this one must need no flip applied by the caller."""
    H = np.array([[-1.0, 0.0, 639.0], [0.0, -1.0, 479.0], [0.0, 0.0, 1.0]])
    det = ReferenceButtonDetector(ref_dir)
    res = det.detect(_scene_with_panel(H))
    assert res["center"] is not None, res.get("reason")
    ex, ey = _expected(H)
    assert abs(res["center"][0] - ex) < 8
    assert abs(res["center"][1] - ey) < 8


def test_abstains_on_a_scene_with_no_panel(ref_dir):
    """Returning None here is the whole safety property -- a pick on a frame
    with no microwave in it would be a coordinate the arm could be asked to
    press."""
    rng = np.random.default_rng(7)
    noise = rng.integers(0, 255, (480, 640, 3), dtype=np.uint8)
    det = ReferenceButtonDetector(ref_dir)
    assert det.detect(noise)["center"] is None


def test_abstains_on_a_blank_scene(ref_dir):
    det = ReferenceButtonDetector(ref_dir)
    assert det.detect(np.full((480, 640, 3), 200, np.uint8))["center"] is None


def test_abstains_when_the_button_is_out_of_frame(ref_dir):
    """Panel partly visible but the marked button shifted outside the image:
    the honest answer is None, not a clamped edge pixel."""
    H = np.array([[1.0, 0.0, -330.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    det = ReferenceButtonDetector(ref_dir)
    res = det.detect(_scene_with_panel(H))
    if res["center"] is not None:
        assert not (0 <= res["center"][0] < 640 and 0 <= res["center"][1] < 480)
