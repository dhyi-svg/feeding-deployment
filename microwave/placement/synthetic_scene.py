"""Ray-cast depth images of an open microwave on a counter, for offline tests and replay dry runs.

Not used on the robot. The scene is a set of oriented boxes in arm_base_link: the microwave
shell (floor, ceiling, side walls with the control panel, back wall), a turntable, the door
opened 90 deg on the hinge side, the countertop, and optionally the held container. A pinhole
camera with any roll renders aligned depth (mm) exactly like the RealSense topic, plus a flat
colour image, and the microwave's exterior bounding box in pixels (what YOLO would return).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.spatial.transform import Rotation as R

Z = np.array([0.0, 0.0, 1.0])


@dataclass
class OBB:
    center: np.ndarray
    axes: np.ndarray        # 3x3, columns are the box axes in arm_base_link
    half: np.ndarray
    name: str = ""


@dataclass
class MicrowaveScene:
    origin: tuple = (0.62, 0.05, 0.10)   # front-bottom-centre of the cavity (arm_base_link)
    yaw_deg: float = 5.0                 # insertion axis f = (cos, sin, 0)
    width: float = 0.30
    depth: float = 0.29
    height: float = 0.20
    turntable: float = 0.012             # 0 = none
    door_open: bool = True
    door_side: float = 1.0               # +1 hinge on the left (+l), -1 on the right
    counter_gap: float = 0.03            # microwave feet
    held_box: OBB = None
    boxes: list = field(default_factory=list)

    def __post_init__(self):
        o = np.asarray(self.origin, float)
        y = np.radians(self.yaw_deg)
        self.f = np.array([np.cos(y), np.sin(y), 0.0])
        self.l = np.cross(Z, self.f)
        W, D, H = self.width, self.depth, self.height
        panel = 0.12
        t = 0.03

        def box(f0, f1, l0, l1, z0, z1, name):
            c = o + self.f * (f0 + f1) / 2 + self.l * (l0 + l1) / 2 + Z * (z0 + z1) / 2
            return OBB(c, np.column_stack([self.f, self.l, Z]), np.array([(f1 - f0) / 2, (l1 - l0) / 2, (z1 - z0) / 2]),
                       name)

        # door hinge side gets the thin wall, the other side the control panel
        s = self.door_side
        hinge_l = (W / 2 + t) * s
        panel_l = -(W / 2 + panel) * s
        lo, hi = min(hinge_l, panel_l), max(hinge_l, panel_l)
        self.boxes = [
            box(-0.02, D + 0.04, lo, hi, -0.05, 0.0, "floor"),
            box(-0.02, D + 0.04, lo, hi, H, H + 0.05, "ceiling"),
            box(D, D + 0.04, lo, hi, -0.05, H + 0.05, "back"),
            box(-0.02, D + 0.04, *sorted((W / 2 * s, hinge_l)), -0.05, H + 0.05, "hinge wall"),
            box(-0.02, D + 0.04, *sorted((-W / 2 * s, panel_l)), -0.05, H + 0.05, "panel"),
            box(-0.70, D + 0.15, -0.9, 0.9, -0.05 - self.counter_gap - 0.05, -0.05 - self.counter_gap, "counter"),
        ]
        if self.turntable > 0:
            self.boxes.append(box(0.04, D - 0.04, -0.11, 0.11, 0.0, self.turntable, "turntable"))
        if self.door_open:
            self.boxes.append(box(-0.02 - 0.42, -0.02, *sorted((hinge_l, hinge_l + 0.04 * s)), -0.05, H + 0.05, "door"))
        if self.held_box is not None:
            self.boxes.append(self.held_box)
        self.exterior = [b for b in self.boxes if b.name in ("floor", "ceiling", "back", "hinge wall", "panel")]

    # ground truth along the cavity axes
    def truth(self):
        o = np.asarray(self.origin, float)
        return {"front": float(o @ self.f), "back": float(o @ self.f + self.depth),
                "left": float(o @ self.l + self.width / 2), "right": float(o @ self.l - self.width / 2),
                "floor_z": float(o[2]), "top": float(o[2] + self.height),
                "support_z": float(o[2] + self.turntable)}


def look_at_camera(cam_pos, target, roll_deg=0.0):
    """base_T_cam (4x4) for an optical frame (z forward, x right, y down) at cam_pos looking at
    target, then rolled by roll_deg about its own optical axis (the wrist camera's mount)."""
    zc = np.asarray(target, float) - np.asarray(cam_pos, float)
    zc /= np.linalg.norm(zc)
    xc = np.cross(zc, Z)
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    Rm = np.column_stack([xc, yc, zc]) @ R.from_euler("z", roll_deg, degrees=True).as_matrix()
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = Rm, cam_pos
    return T


def default_K(w=640, h=480, f=605.0):
    return np.array([f, 0, w / 2, 0, f, h / 2, 0, 0, 1], float)


def render(scene, base_T_cam, K, shape=(480, 640), noise_mm=0.0, seed=0):
    """Aligned depth (mm, float32, 0 = no return), a flat BGR image, and the per-pixel box name."""
    h, w = shape
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    d_cam = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us, float)], axis=-1).reshape(-1, 3)
    Rm, o = base_T_cam[:3, :3], base_T_cam[:3, 3]
    d_world = d_cam @ Rm.T
    best = np.full(len(d_world), np.inf)
    hit = np.full(len(d_world), -1)
    for i, b in enumerate(scene.boxes):
        oc = (o - b.center) @ b.axes
        dc = d_world @ b.axes
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-b.half - oc) / dc
            t2 = (b.half - oc) / dc
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        ok = (tmax >= tmin) & (tmax > 0) & (tmin > 0.01)
        closer = ok & (tmin < best)
        best[closer], hit[closer] = tmin[closer], i
    depth = np.where(np.isfinite(best), best, 0.0) * 1000.0   # d_cam has z = 1 -> t is z-depth
    if noise_mm > 0:
        rng = np.random.default_rng(seed)
        depth = np.where(depth > 0, depth + rng.normal(0, noise_mm, depth.shape), 0.0)
    depth = depth.reshape(h, w).astype(np.float32)
    bgr = np.full((h, w, 3), 128, np.uint8)
    return depth, bgr, hit.reshape(h, w)


def exterior_box_px(scene, base_T_cam, K, shape=(480, 640)):
    """Pixel bounding box of the microwave shell's corners -- what the detector returns."""
    pts = []
    for b in scene.exterior:
        for sx in (-1, 1):
            for sy in (-1, 1):
                for sz in (-1, 1):
                    pts.append(b.center + b.axes @ (b.half * [sx, sy, sz]))
    pts = np.array(pts)
    T = np.linalg.inv(base_T_cam)
    pc = pts @ T[:3, :3].T + T[:3, 3]
    pc = pc[pc[:, 2] > 0.05]
    u = K[0] * pc[:, 0] / pc[:, 2] + K[2]
    v = K[4] * pc[:, 1] / pc[:, 2] + K[5]
    h, w = shape
    return (float(np.clip(u.min(), 0, w - 1)), float(np.clip(v.min(), 0, h - 1)),
            float(np.clip(u.max(), 0, w - 1)), float(np.clip(v.max(), 0, h - 1)))
