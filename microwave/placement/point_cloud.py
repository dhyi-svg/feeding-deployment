"""Depth -> filtered 3D points in arm_base_link, and the camera's roll relative to gravity.

Pure numpy/scipy (no ROS): the node and the CLI pass in the depth image, the camera
intrinsics K and the 4x4 `base_T_cam` (arm_base_link <- camera_color_optical_frame) from tf2.

Camera orientation (the wrist camera is mounted rotated 90 deg CCW since the last calibration):
the geometry needs nothing special -- every point goes through the full `base_T_cam` from tf2,
so as long as calibration_tf publishes the rotated calibration (tools/rotate_camera_calibration.py)
the points are right whatever the roll. Two things do depend on the roll and are handled here:
  * the COCO detector wants an upright image: `upright_k` picks the multiple of 90 deg that puts
    world-up at image-up for THIS frame (from tf, so it follows J7 too), and `rot_box_to_raw` maps
    the detection back to raw pixels;
  * a wrong calibration (roll not applied, or the wrong sign) shows up as a fitted floor whose
    normal is not vertical -- `cavity_perception` refuses it (floor_max_tilt_deg).
"""
from __future__ import annotations

import numpy as np
from scipy.spatial import cKDTree


def intrinsics(K):
    """(fx, fy, cx, cy) from a row-major 3x3 K (CameraInfo.k / CameraInfoCompat.K, list or array)."""
    K = np.asarray(K, dtype=float).reshape(-1)
    return K[0], K[4], K[2], K[5]


def transform_points(T, points):
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points @ T[:3, :3].T + T[:3, 3]


def deproject(depth_mm, K, roi=None, stride=1, depth_range=(0.12, 1.3), depth_corr_m=0.0):
    """Valid-depth pixels -> camera-frame points (m).

    depth_mm: HxW aligned depth in millimetres (float or uint16; 0/nan = no depth).
    roi: (x1, y1, x2, y2) raw-image pixel box, or None for the whole frame.
    Returns (points[N, 3], uv[N, 2] int pixel coords, n_sampled).
    """
    depth = np.asarray(depth_mm)
    h, w = depth.shape[:2]
    x1, y1, x2, y2 = (0, 0, w, h) if roi is None else roi
    x1, y1 = max(0, int(x1)), max(0, int(y1))
    x2, y2 = min(w, int(np.ceil(x2))), min(h, int(np.ceil(y2)))
    if x2 <= x1 or y2 <= y1:
        return np.zeros((0, 3)), np.zeros((0, 2), int), 0
    ys, xs = np.mgrid[y1:y2:stride, x1:x2:stride]
    xs, ys = xs.ravel(), ys.ravel()
    d = depth[ys, xs].astype(float) / 1000.0
    ok = np.isfinite(d) & (d > depth_range[0]) & (d < depth_range[1])
    fx, fy, cx, cy = intrinsics(K)
    z = d[ok]
    pts = np.stack([(xs[ok] - cx) * z / fx, (ys[ok] - cy) * z / fy, z + depth_corr_m], axis=1)
    return pts, np.stack([xs[ok], ys[ok]], axis=1), len(d)


def project(points_cam, K):
    """Camera-frame points -> pixel (u, v); points behind the camera give nan."""
    fx, fy, cx, cy = intrinsics(K)
    p = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(p[:, 2] > 1e-6, p[:, 2], np.nan)
        return np.stack([fx * p[:, 0] / z + cx, fy * p[:, 1] / z + cy], axis=1)


def voxel_downsample(points, voxel):
    """One point (the mean) per occupied voxel."""
    points = np.asarray(points, dtype=float)
    if len(points) == 0 or voxel <= 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, inv, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inv = inv.reshape(-1)
    sums = np.zeros((len(counts), 3))
    np.add.at(sums, inv, points)
    return sums / counts[:, None]


def remove_statistical_outliers(points, k=12, std_ratio=2.0):
    """Drop points whose mean distance to their k neighbours is > mean + std_ratio * std."""
    points = np.asarray(points, dtype=float)
    if len(points) <= k + 1:
        return points
    dist, _ = cKDTree(points).query(points, k=k + 1)
    mean_d = dist[:, 1:].mean(axis=1)
    keep = mean_d <= mean_d.mean() + std_ratio * mean_d.std()
    return points[keep]


def filter_cloud(points, cfg):
    """Voxel downsample + statistical outlier removal (PointCloudConfig)."""
    pts = voxel_downsample(points, cfg.voxel_m)
    return remove_statistical_outliers(pts, cfg.outlier_k, cfg.outlier_std)


def remove_self_points(points_base, tool_T, box):
    """Drop points on the robot's own hand: inside the tool-frame box |x| <= bx, |y| <= by, z in [z0, z1].
    `tool_T` = base_T_tool (4x4). The held container is removed separately (`remove_obb`)."""
    if len(points_base) == 0:
        return points_base
    bx, by, z0, z1 = box
    local = transform_points(np.linalg.inv(tool_T), points_base)
    inside = (np.abs(local[:, 0]) <= bx) & (np.abs(local[:, 1]) <= by) & (local[:, 2] >= z0) & (local[:, 2] <= z1)
    return points_base[~inside]


def remove_obb(points_base, center, rotation, half_extents, inflate=0.0):
    """Drop points inside an oriented box (centre, 3x3 rotation columns = box axes, half extents)."""
    if len(points_base) == 0:
        return points_base
    local = (np.asarray(points_base) - center) @ np.asarray(rotation)
    inside = np.all(np.abs(local) <= np.asarray(half_extents) + inflate, axis=1)
    return points_base[~inside]


# ---------------------------------------------------------------------------
# Camera roll vs gravity, and the upright image for the detector
# ---------------------------------------------------------------------------
def world_up_in_image(base_T_cam):
    """Unit 2D direction (image x right, y down) of world +z as seen by the camera, and the
    roll angle (deg) of the image: 0 = world-up is image-up, +90 = world-up points image-right."""
    up_cam = base_T_cam[:3, :3].T @ np.array([0.0, 0.0, 1.0])
    v = up_cam[:2]
    n = np.linalg.norm(v)
    if n < 1e-3:
        return None, None   # looking straight up/down: no image-up direction
    v = v / n
    return v, float(np.degrees(np.arctan2(v[0], -v[1])))


def rot90_vec(v, k):
    """Image-plane direction (x right, y down) after np.rot90(img, k) (k CCW quarter turns)."""
    x, y = float(v[0]), float(v[1])
    for _ in range(k % 4):
        x, y = y, -x
    return np.array([x, y])


def upright_k(base_T_cam):
    """k for np.rot90(img, k) that puts world-up closest to image-up (0 if undefined)."""
    v, _ = world_up_in_image(base_T_cam)
    if v is None:
        return 0
    return int(max(range(4), key=lambda k: -rot90_vec(v, k)[1]))


def rot_point_to_raw(u, v, k, raw_shape):
    """Pixel (u, v) in np.rot90(raw, k) -> pixel in the raw image (raw_shape = (H, W))."""
    h, w = raw_shape[:2]
    k %= 4
    if k == 0:
        return u, v
    if k == 1:     # rotated (H'=W, W'=H): raw (r, c) -> (W-1-c, r)
        return (w - 1) - v, u
    if k == 2:
        return (w - 1) - u, (h - 1) - v
    return v, (h - 1) - u   # k == 3


def rot_box_to_raw(box, k, raw_shape):
    """(x1, y1, x2, y2) in the rotated image -> axis-aligned box in raw pixels."""
    x1, y1, x2, y2 = box
    corners = np.array([rot_point_to_raw(u, v, k, raw_shape) for u, v in ((x1, y1), (x2, y1), (x1, y2), (x2, y2))])
    return (float(corners[:, 0].min()), float(corners[:, 1].min()),
            float(corners[:, 0].max()), float(corners[:, 1].max()))
