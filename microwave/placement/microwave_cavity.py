"""Open microwave's cavity from segmented depth points, and a container placement point in it.

Pure numpy (no ROS, no robot) so it can be checked offline (tests/test_microwave_cavity.py);
`real_gen3_ros2_place_container_microwave.py` feeds it the SAM 3 interior mask's depth points.
"""

import numpy as np

# Robust bounds are percentiles of the segmented points rather than min/max, so a
# few depth-noise / mask-edge pixels cannot move a wall.
BOUND_PERCENTILE = 5.0

# Points within this distance (m) of the front plane are treated as the rim / front
# lip and are excluded from the floor / wall estimates.
RIM_MARGIN = 0.02

# Minimum number of valid 3D points (overall, and behind the rim) before the
# segmentation + depth are trusted at all.
MIN_VALID_POINTS = 500

# Physical plausibility of the visible cavity (m). Min values reject a mask that
# landed on a flat surface (e.g. the door or the front panel: no depth behind the
# rim); max values reject a mask that leaked out of the microwave.
MIN_CAVITY_DEPTH = 0.15
MAX_CAVITY_DEPTH = 0.60
MIN_CAVITY_WIDTH = 0.20
MAX_CAVITY_WIDTH = 0.70
MIN_CAVITY_HEIGHT = 0.08

# Clearance (m) kept between the placement point (container center) and the cavity
# bounds. Initial values -- validate against the actual container on the robot.
SIDE_WALL_MARGIN = 0.10
BACK_WALL_MARGIN = 0.10
FRONT_MARGIN = 0.10


def microwave_frame(closed_normal):
    """Microwave frame in arm_base_link from DOOR_FILE's `closed_normal` (horizontal, out of the
    door toward the arm): columns x left, y up (world z), z forward into the microwave."""
    n = np.asarray(closed_normal, dtype=float).copy()
    n[2] = 0.0
    if not np.all(np.isfinite(n)) or np.linalg.norm(n) < 1e-6:
        raise ValueError(f"closed_normal {closed_normal} has no horizontal direction")
    forward = -n / np.linalg.norm(n)
    up = np.array([0.0, 0.0, 1.0])
    return np.column_stack([np.cross(up, forward), up, forward])


def estimate_microwave_cavity(points, rotation):
    """Estimate cavity bounds and a placement point from segmented interior points.

    points: (N, 3) points in arm_base_link from the interior mask + aligned depth.
    rotation: 3x3 microwave frame in arm_base_link -- columns x left, y up, z forward
        into the microwave. Bounds are measured along these axes.

    Returns a dict with the cavity bounds (in the rotated frame), the number of
    points used, and "placement_point": the container center on the cavity floor, in
    arm_base_link. Raises ValueError (with the reason) when the data is not a
    plausible open-microwave interior; callers must not fall back to a guess.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"Expected (N, 3) points, got shape {points.shape}")
    rotation = np.asarray(rotation, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("Placement rotation must be a finite 3x3 matrix")

    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < MIN_VALID_POINTS:
        raise ValueError(f"Only {len(points)} valid 3D points in the interior mask (need {MIN_VALID_POINTS})")

    # Coordinates along the placement axes: lateral (x), up (y), forward (z).
    local = points @ rotation
    lateral, up, forward = local[:, 0], local[:, 1], local[:, 2]

    front = np.percentile(forward, BOUND_PERCENTILE)
    back = np.percentile(forward, 100.0 - BOUND_PERCENTILE)
    depth = back - front
    if not MIN_CAVITY_DEPTH <= depth <= MAX_CAVITY_DEPTH:
        raise ValueError(f"Implausible cavity depth {depth:.3f} m behind the front "
                         f"(expected {MIN_CAVITY_DEPTH}-{MAX_CAVITY_DEPTH} m)")

    # Everything below is estimated only from points meaningfully behind the rim.
    inside = forward > front + RIM_MARGIN
    if np.count_nonzero(inside) < MIN_VALID_POINTS:
        raise ValueError(f"Only {np.count_nonzero(inside)} points lie behind the microwave front "
                         f"(need {MIN_VALID_POINTS})")
    lateral, up = lateral[inside], up[inside]

    floor = np.percentile(up, BOUND_PERCENTILE)
    top = np.percentile(up, 100.0 - BOUND_PERCENTILE)
    height = top - floor
    if height < MIN_CAVITY_HEIGHT:
        raise ValueError(f"Visible cavity height {height:.3f} m is below {MIN_CAVITY_HEIGHT} m")

    right = np.percentile(lateral, BOUND_PERCENTILE)
    left = np.percentile(lateral, 100.0 - BOUND_PERCENTILE)
    width = left - right
    if not MIN_CAVITY_WIDTH <= width <= MAX_CAVITY_WIDTH:
        raise ValueError(f"Implausible cavity width {width:.3f} m "
                         f"(expected {MIN_CAVITY_WIDTH}-{MAX_CAVITY_WIDTH} m)")

    # Container center: laterally centered, at the middle of the usable depth, on the floor.
    lateral_lo, lateral_hi = right + SIDE_WALL_MARGIN, left - SIDE_WALL_MARGIN
    forward_lo, forward_hi = front + FRONT_MARGIN, back - BACK_WALL_MARGIN
    if lateral_lo > lateral_hi or forward_lo > forward_hi:
        raise ValueError("Cavity is too small for the placement margins")
    target_local = np.array([
        0.5 * (lateral_lo + lateral_hi),
        floor,
        0.5 * (forward_lo + forward_hi),
    ])

    return {
        "placement_point": rotation @ target_local,
        "front": front,
        "back": back,
        "floor": floor,
        "top": top,
        "left": left,
        "right": right,
        # Distance from the front plane to the placement point along the insertion axis.
        "insert_depth": target_local[2] - front,
        "num_points": int(np.count_nonzero(inside)),
    }
