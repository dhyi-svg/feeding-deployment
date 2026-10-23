"""Where to look from, so the placement does not depend on a hand-picked look pose.

The wrist camera sits ~17 cm behind the tool frame and looks along the approach, so whatever
the gripper holds is in the middle of the image. Given a coarse idea of where the cavity is
(a saved look, or the current detection), this proposes camera viewpoints in front of the
opening, turns each into the tool pose that puts the camera there (via the live tool -> camera
transform), and scores it by what it would actually see:

  * the cavity's floor and back wall, sampled on a grid, projected into the candidate camera;
  * minus what the held box and the gripper (projected the same way) would cover;
  * views that would tilt the held box past `look.max_container_tilt_deg`, are out of reach, or
    have no IK solution inside the joint guards are dropped.

Pure numpy except the IK/limit check, which uses the same PlacementSim as the planner.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

Z = np.array([0.0, 0.0, 1.0])


@dataclass
class CoarseCavity:
    """Enough of the cavity to aim the camera: opening centre (at mid height), axes, size."""
    opening: np.ndarray
    f: np.ndarray
    width: float
    depth: float
    height: float

    @staticmethod
    def from_cavity(c):
        mid_l = (c.left + c.right) / 2
        return CoarseCavity(c.point(c.front, mid_l, (c.floor_z + c.top) / 2), c.f, c.width, c.depth, c.height)

    def l(self):
        return np.cross(Z, self.f)

    def sample_points(self, n=7):
        """Grid on the floor and the back wall -- what a useful view must see -- and their normals."""
        l = self.l()
        floor_z = self.opening[2] - self.height / 2
        pts, nrm = [], []
        for a in np.linspace(0.1, 0.9, n):
            for b in np.linspace(-0.4, 0.4, n):
                pts.append(self.opening + self.f * (a * self.depth) + l * (b * self.width)
                           + Z * (floor_z - self.opening[2]))
                nrm.append(Z)
                pts.append(self.opening + self.f * self.depth + l * (b * self.width)
                           + Z * (floor_z + a * self.height - self.opening[2]))
                nrm.append(-self.f)
        return np.array(pts), np.array(nrm)


@dataclass
class View:
    tool_pos: np.ndarray
    tool_quat: np.ndarray
    cam_T: np.ndarray
    visible: float            # fraction of the cavity sample points seen (in image, not covered)
    box_tilt_deg: float
    q: np.ndarray = None      # IK solution (filled by `feasible_views`)
    joint_dist_deg: float = 0.0


def look_at(cam_pos, target, roll_deg):
    zc = np.asarray(target, float) - cam_pos
    zc /= np.linalg.norm(zc)
    xc = np.cross(zc, Z)
    if np.linalg.norm(xc) < 1e-6:
        xc = np.array([1.0, 0.0, 0.0])
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    return np.column_stack([xc, yc, zc]) @ R.from_euler("z", roll_deg, degrees=True).as_matrix()


def occluder_polygons(cam_T, tool_T, K, shape, box_obb_local, hand_box):
    """Image mask (H/4 x W/4) of the held box + the hand as seen from cam_T."""
    h, w = shape[0] // 4, shape[1] // 4
    mask = np.zeros((h, w), np.uint8)
    T_cb = np.linalg.inv(cam_T)
    shapes = []
    if box_obb_local is not None:
        c, axes, half = box_obb_local
        shapes.append((c, axes, half))
    bx, by, z0, z1 = hand_box
    shapes.append((np.array([0, 0, (z0 + z1) / 2]), np.eye(3), np.array([bx, by, (z1 - z0) / 2])))
    for c, axes, half in shapes:
        corners = np.array([c + axes @ (half * s) for s in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)])
        world = corners @ tool_T[:3, :3].T + tool_T[:3, 3]
        pc = world @ T_cb[:3, :3].T + T_cb[:3, 3]
        if np.all(pc[:, 2] <= 0.01):
            continue
        pc[:, 2] = np.maximum(pc[:, 2], 0.03)      # a corner beside the lens: very near (covers a lot)
        u = (K[0] * pc[:, 0] / pc[:, 2] + K[2]) / 4
        v = (K[4] * pc[:, 1] / pc[:, 2] + K[5]) / 4
        hull = cv2.convexHull(np.clip(np.stack([u, v], 1), -1e4, 1e4).astype(np.int32))
        cv2.fillConvexPoly(mask, hull, 1)
    return mask


def visible_fraction(cam_T, K, shape, pts, occ_mask, normals=None, min_incidence_deg=12.0):
    """Fraction of the sample points in the image, not covered by the occluders, and (with normals)
    seen at >= min_incidence_deg off their surface -- a grazing look gives no usable depth."""
    T_cb = np.linalg.inv(cam_T)
    pc = pts @ T_cb[:3, :3].T + T_cb[:3, 3]
    ok = pc[:, 2] > 0.1
    if normals is not None:
        ray = pts - cam_T[:3, 3]
        ray /= np.linalg.norm(ray, axis=1, keepdims=True)
        ok &= -np.sum(ray * normals, axis=1) >= np.sin(np.radians(min_incidence_deg))
    u = K[0] * pc[:, 0] / np.where(ok, pc[:, 2], 1) + K[2]
    v = K[4] * pc[:, 1] / np.where(ok, pc[:, 2], 1) + K[5]
    h, w = shape
    inside = ok & (u >= 8) & (u < w - 8) & (v >= 8) & (v < h - 8)
    ui, vi = (u[inside] / 4).astype(int), (v[inside] / 4).astype(int)
    seen = np.count_nonzero(occ_mask[np.clip(vi, 0, occ_mask.shape[0] - 1), np.clip(ui, 0, occ_mask.shape[1] - 1)] == 0)
    return seen / len(pts)


def hold_orientation(f, yaw_deg, pitch_down_deg, up_local):
    """Tool rotation whose approach points along f turned by yaw, tipped nose-down by pitch, with the
    tool direction `up_local` (the held box's up) as close to world up as that allows."""
    h = R.from_euler("z", yaw_deg, degrees=True).apply(f)
    p = np.radians(pitch_down_deg)
    a = np.cos(p) * h - np.sin(p) * Z
    u = Z - a * (a @ Z)
    u /= np.linalg.norm(u)
    a_l = np.array([0.0, 0.0, 1.0])
    u_l = np.asarray(up_local, float)
    L = np.column_stack([a_l, u_l, np.cross(a_l, u_l)])
    W = np.column_stack([a, u, np.cross(a, u)])
    return W @ L.T


def candidate_views(coarse, tool_T_cam, K, shape, vcfg, max_reach, box_local=None, up_local=None, hand_box=None):
    """Scored viewpoints (best first), before IK.

    The camera looks along the approach, so whatever the hand holds sits right below the image
    centre. Instead of aiming the camera AT the cavity (which puts the held box squarely in the way),
    the hand keeps the box level (yaw free, nose-down tilt <= max_container_tilt_deg) and the
    camera is placed low in front of the opening: the cavity then appears above the box in the image.
    box_local: held box OBB in the TOOL frame (None = empty gripper; then up_local is just the hand's
    current up and the tilt may go to empty_max_tilt_deg)."""
    pts, nrm = coarse.sample_points()
    l = coarse.l()
    floor_z = coarse.opening[2] - coarse.height / 2
    holding = box_local is not None
    max_tilt = vcfg.max_container_tilt_deg if holding else vcfg.empty_max_tilt_deg
    pitches = [p for p in vcfg.pitches_down_deg if p <= max_tilt]
    up = up_local if up_local is not None else np.array([1.0, 0.0, 0.0])
    views = []
    for d in vcfg.distances_m:
        for hgt in vcfg.heights_above_floor_m:
            for s in vcfg.laterals_m:
                cam_pos = coarse.opening - coarse.f * d + l * s
                cam_pos[2] = floor_z + hgt
                for yaw in vcfg.yaws_deg:
                    for pitch in pitches:
                        tool_R = hold_orientation(coarse.f, yaw, pitch, up)
                        tool_T = np.eye(4)
                        tool_T[:3, :3] = tool_R
                        tool_T[:3, 3] = cam_pos - tool_R @ tool_T_cam[:3, 3]
                        if np.linalg.norm(tool_T[:3, 3]) > max_reach:
                            continue
                        cam_T = tool_T @ tool_T_cam
                        occ = occluder_polygons(cam_T, tool_T, K, shape, box_local, hand_box)
                        vis = visible_fraction(cam_T, K, shape, pts, occ, nrm, vcfg.min_incidence_deg)
                        if vis < vcfg.min_visible:
                            continue
                        views.append(View(tool_T[:3, 3].copy(), R.from_matrix(tool_R).as_quat(), cam_T, vis,
                                          float(pitch) if holding else 0.0))
    views.sort(key=lambda v: -v.visible)
    return views


def feasible_views(views, sim, q_now, limit=6, log=print):
    """IK + joint guards for the best views; ranked by visibility, then joint travel."""
    out = []
    for v in views:
        q, pe, oe = sim.ik(v.tool_pos, v.tool_quat, q_now)
        if pe > 0.005 or oe > 2.0 or sim.joint_problem(q):
            continue
        v.q = q
        v.joint_dist_deg = float(np.degrees(np.max(np.abs((q - q_now + np.pi) % (2 * np.pi) - np.pi))))
        out.append(v)
        if len(out) >= limit * 3:
            break
    out.sort(key=lambda v: (-round(v.visible, 1), v.joint_dist_deg))
    # different viewpoints, so a second look adds what the first missed (a side wall, the front)
    picked = []
    for v in out:
        if all(np.linalg.norm(v.tool_pos - w.tool_pos) > 0.06 or
               np.degrees((R.from_quat(v.tool_quat).inv() * R.from_quat(w.tool_quat)).magnitude()) > 8 for w in picked):
            picked.append(v)
    out = picked
    for v in out[:limit]:
        log(f"  view candidate: sees {v.visible * 100:.0f}% of the cavity, box tilt {v.box_tilt_deg:.0f} deg, "
            f"{v.joint_dist_deg:.0f} deg of joint travel, tool {np.round(v.tool_pos, 3).tolist()}")
    return out[:limit]
