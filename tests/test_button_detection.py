"""Selection logic for the local (GroundingDINO) microwave start-button detector.

The lab locates the start button with a remote Molmo VLM. Where that server is
unreachable, ``detect_start_button_pixel_local`` picks the same button out of
GroundingDINO's boxes using the rule Molmo's own prompt states: the RIGHT one of
the two rectangular buttons on the BOTTOM row of the control panel.

The subtlety these tests pin down is orientation. The wrist camera is mounted
upside down, so "bottom" and "right" mean the bottom/right of the *flipped*
(visually upright) view, while ``detect_items`` hands back boxes in raw
original-image coordinates. Getting that backwards picks the diagonally opposite
button -- which looks plausible in a log and presses the wrong control.

GroundingDINO is stubbed out; these tests load no models and need no camera.
"""

import types

import numpy as np
import pytest

from feeding_deployment.perception.appliance_perception.appliance_perception import (
    AppliancePerception,
)

WIDTH, HEIGHT = 640, 480


def _upright_to_original(box, width=WIDTH, height=HEIGHT):
    """Raw camera coordinates for a box described in the upright view.

    The camera is mounted upside down, so the raw frame is the 180-degree
    rotation of what a person sees.
    """
    x1, y1, x2, y2 = box
    return (width - x2, height - y2, width - x1, height - y1)


def _center(box):
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def _perception_with_boxes(boxes, confidences):
    """An AppliancePerception whose detector returns exactly ``boxes``.

    Built with ``__new__`` so no models, ROS handles or camera are constructed --
    only the pure selection logic under test is exercised.
    """
    perception = AppliancePerception.__new__(AppliancePerception)
    detections = types.SimpleNamespace(
        xyxy=np.asarray(boxes, dtype=float),
        confidence=np.asarray(confidences, dtype=float),
    )
    perception.detect_items = (
        lambda image, classes, return_all=False: detections  # noqa: ARG005
    )
    return perception


@pytest.fixture(name="rgb_image")
def fixture_rgb_image():
    return np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)


def test_picks_bottom_row_rightmost_button(rgb_image):
    """A 2x2 control panel: the start button is bottom-right as a person sees it."""
    upright = {
        "top_left": (100, 90, 140, 130),
        "top_right": (380, 90, 420, 130),
        "bottom_left": (100, 290, 140, 330),
        "bottom_right_start": (380, 290, 420, 330),
    }
    names = list(upright)
    perception = _perception_with_boxes(
        [_upright_to_original(upright[n]) for n in names],
        [0.55, 0.60, 0.58, 0.62],
    )

    picked = perception.detect_start_button_pixel_local(rgb_image)

    start_x, start_y = _center(upright["bottom_right_start"])
    assert picked == (
        int(round(WIDTH - start_x)),
        int(round(HEIGHT - start_y)),
    )


def test_ignores_confidence_when_choosing(rgb_image):
    """Position decides, not score -- a confident top-row button must not win."""
    upright = {
        "top_right_confident": (380, 90, 420, 130),
        "bottom_right_start": (380, 290, 420, 330),
    }
    names = list(upright)
    perception = _perception_with_boxes(
        [_upright_to_original(upright[n]) for n in names],
        [0.99, 0.31],
    )

    picked = perception.detect_start_button_pixel_local(rgb_image)

    start_x, start_y = _center(upright["bottom_right_start"])
    assert picked == (
        int(round(WIDTH - start_x)),
        int(round(HEIGHT - start_y)),
    )


def test_rejects_whole_panel_detection(rgb_image):
    """A box covering the frame is the panel/appliance, not a button."""
    perception = _perception_with_boxes([[0.0, 0.0, float(WIDTH), float(HEIGHT)]], [0.9])

    assert perception.detect_start_button_pixel_local(rgb_image) is None


def test_no_detections_returns_none(rgb_image):
    """No boxes must return None so the caller's frame loop retries."""
    perception = _perception_with_boxes(np.zeros((0, 4)), np.zeros((0,)))

    assert perception.detect_start_button_pixel_local(rgb_image) is None


def test_slightly_uneven_row_still_groups_together(rgb_image):
    """Real detections are never pixel-aligned; a small y jitter is one row."""
    # ~2% of frame height apart, well inside BUTTON_ROW_TOL_FRAC (6%).
    upright = {
        "bottom_left": (100, 290, 140, 330),
        "bottom_right_start": (380, 298, 420, 338),
    }
    names = list(upright)
    perception = _perception_with_boxes(
        [_upright_to_original(upright[n]) for n in names],
        [0.60, 0.58],
    )

    picked = perception.detect_start_button_pixel_local(rgb_image)

    start_x, start_y = _center(upright["bottom_right_start"])
    assert picked == (
        int(round(WIDTH - start_x)),
        int(round(HEIGHT - start_y)),
    )
