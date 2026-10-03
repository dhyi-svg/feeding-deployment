"""Tests for the open-microwave cavity estimate used for plate placement.

Run with:
    PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_microwave_cavity.py -v

These exercise the pure geometry (segmented interior points -> cavity bounds ->
placement point) on synthetic point clouds, without the robot, camera, or ROS.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "microwave" / "placement"))
from microwave_cavity import (  # noqa: E402
    BACK_WALL_MARGIN,
    FRONT_MARGIN,
    SIDE_WALL_MARGIN,
    estimate_microwave_cavity,
    microwave_frame,
)

# A microwave frame (x left, y up, z forward into the microwave) in arm_base_link.
ROTATION = Rotation.from_quat([-0.5, 0.5, 0.5, -0.5]).as_matrix()
ORIGIN = np.array([-0.70, 0.05, 0.30])  # front-bottom-center of the cavity in arm_base_link


def _cavity_points(width=0.32, height=0.20, depth=0.30, n=3000, noise=0.002, seed=0):
    """Floor, back wall and side walls of a box cavity, in arm_base_link."""
    rng = np.random.default_rng(seed)
    u = rng.uniform(size=(n, 2))
    floor = np.c_[(u[:, 0] - 0.5) * width, np.zeros(n), u[:, 1] * depth]
    back = np.c_[(u[:, 0] - 0.5) * width, u[:, 1] * height, np.full(n, depth)]
    left = np.c_[np.full(n, width / 2), u[:, 0] * height, u[:, 1] * depth]
    right = np.c_[np.full(n, -width / 2), u[:, 0] * height, u[:, 1] * depth]
    local = np.vstack([floor, back, left, right]) + rng.normal(scale=noise, size=(4 * n, 3))
    return local @ ROTATION.T + ORIGIN


def _local(point):
    return (np.asarray(point) - ORIGIN) @ ROTATION


def test_valid_cavity_target_inside_bounds():
    cavity = estimate_microwave_cavity(_cavity_points(), ROTATION)
    lateral, up, forward = _local(cavity["placement_point"])

    assert abs(lateral) < 0.01          # laterally centered
    assert abs(up) < 0.01               # on the floor
    assert 0.0 < forward < 0.30         # behind the front, in front of the back wall
    assert cavity["insert_depth"] > 0.0


def test_target_respects_margins():
    cavity = estimate_microwave_cavity(_cavity_points(), ROTATION)
    target = cavity["placement_point"] @ ROTATION

    assert target[0] <= cavity["left"] - SIDE_WALL_MARGIN + 1e-9
    assert target[0] >= cavity["right"] + SIDE_WALL_MARGIN - 1e-9
    assert target[2] <= cavity["back"] - BACK_WALL_MARGIN + 1e-9
    assert target[2] >= cavity["front"] + FRONT_MARGIN - 1e-9


def test_some_nonfinite_points_are_ignored():
    points = _cavity_points()
    points[::10] = np.nan
    points[1::10, 2] = np.inf
    cavity = estimate_microwave_cavity(points, ROTATION)
    assert np.all(np.isfinite(cavity["placement_point"]))


def test_empty_points_rejected():
    with pytest.raises(ValueError):
        estimate_microwave_cavity(np.empty((0, 3)), ROTATION)


def test_all_nonfinite_points_rejected():
    with pytest.raises(ValueError):
        estimate_microwave_cavity(np.full((5000, 3), np.nan), ROTATION)


def test_wrong_shape_rejected():
    with pytest.raises(ValueError):
        estimate_microwave_cavity(np.zeros((5000, 2)), ROTATION)


def test_flat_surface_rejected():
    # A mask on the closed door / front panel: no depth behind the front.
    rng = np.random.default_rng(1)
    local = np.c_[rng.uniform(-0.15, 0.15, 5000), rng.uniform(0.0, 0.2, 5000), rng.normal(scale=0.002, size=5000)]
    with pytest.raises(ValueError, match="depth"):
        estimate_microwave_cavity(local @ ROTATION.T + ORIGIN, ROTATION)


def test_mask_leaking_out_of_microwave_rejected():
    with pytest.raises(ValueError, match="width"):
        estimate_microwave_cavity(_cavity_points(width=1.2), ROTATION)


def test_cavity_too_small_for_margins_rejected():
    with pytest.raises(ValueError, match="margins"):
        estimate_microwave_cavity(_cavity_points(depth=0.17), ROTATION)


def test_microwave_frame_from_door_normal():
    # rchi-cpu-5: microwave in +x, door normal pointing back at the arm (-x)
    frame = microwave_frame([-0.9988, -0.0480, 0.0])
    assert np.allclose(frame.T @ frame, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(frame), 1.0)
    assert np.allclose(frame[:, 1], [0, 0, 1])          # up
    assert frame[0, 2] > 0.99                           # forward ~ +x
    assert frame[1, 0] > 0.99                           # left ~ +y


def test_cavity_in_front_of_rig():
    frame = microwave_frame([-1.0, 0.0, 0.0])
    rng = np.random.default_rng(2)
    n = 3000
    u = rng.uniform(size=(n, 2))
    w, h, d = 0.32, 0.20, 0.30
    local = np.vstack([
        np.c_[(u[:, 0] - 0.5) * w, np.zeros(n), u[:, 1] * d],
        np.c_[(u[:, 0] - 0.5) * w, u[:, 1] * h, np.full(n, d)],
        np.c_[np.full(n, w / 2), u[:, 0] * h, u[:, 1] * d],
        np.c_[np.full(n, -w / 2), u[:, 0] * h, u[:, 1] * d],
    ])
    origin = np.array([0.75, -0.08, 0.19])
    cavity = estimate_microwave_cavity(local @ frame.T + origin, frame)
    target = cavity["placement_point"]
    assert 0.75 < target[0] < 0.75 + d                  # inside, along +x
    assert abs(target[1] - origin[1]) < 0.01
    assert abs(target[2] - origin[2]) < 0.01            # on the floor


def test_microwave_frame_rejects_vertical_normal():
    with pytest.raises(ValueError):
        microwave_frame([0.0, 0.0, 1.0])
