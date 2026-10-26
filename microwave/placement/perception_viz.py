"""Debug images for placement perception (pure cv2/numpy; no display needed to write them).

overlay   the camera image (turned upright for reading): SAM 3's mask tinted green, the filtered +
          dilated mask outlined yellow, the fitted floor points blue, the cavity box yellow, the usable
          floor cyan, the container footprint magenta, floor contact red, container centre orange,
          tool centre white; numbers and status at the bottom (red text = refused).
top_down  arm_base_link seen from above (+x up the image, +y to the left): cavity cloud grey, floor
          points blue, cavity / usable floor / footprint rectangles, the three targets, the insertion
          axis and the camera position; 1 px = 2 mm.
"""
from __future__ import annotations

import cv2
import numpy as np

import point_cloud as pcm

GREEN, YELLOW, BLUE, CYAN = (60, 200, 60), (0, 230, 255), (255, 120, 0), (255, 255, 0)
MAGENTA, RED, ORANGE, WHITE, GREY = (255, 0, 255), (0, 0, 255), (0, 140, 255), (255, 255, 255), (150, 150, 150)


def _proj(frame, pts):
    cam = pcm.transform_points(np.linalg.inv(frame.base_T_cam), np.asarray(pts, float).reshape(-1, 3))
    uv = pcm.project(cam, frame.K.reshape(-1))
    return uv


def _poly(img, frame, pts, color, closed=True, th=2):
    uv = _proj(frame, pts)
    if np.all(np.isfinite(uv)):
        cv2.polylines(img, [np.round(uv).astype(np.int32)], closed, color, th, cv2.LINE_AA)


def _dot(img, frame, p, color, r=6):
    uv = _proj(frame, p)[0]
    if np.all(np.isfinite(uv)):
        c = tuple(int(round(v)) for v in uv)
        cv2.circle(img, c, r, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(img, c, r - 2, color, -1, cv2.LINE_AA)


def _text_panel(img, lines, ok):
    h = 18 * len(lines) + 8
    panel = np.zeros((h, img.shape[1], 3), np.uint8)
    for i, (txt, col) in enumerate(lines):
        cv2.putText(panel, txt, (6, 18 * i + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col if ok else RED, 1, cv2.LINE_AA)
    return np.vstack([img, panel])


def overlay(frame, cmask=None, cav=None, res=None, status="", ok=True):
    img = frame.bgr.copy()
    if cmask is not None:
        tint = img.copy()
        tint[cmask.sam_mask] = (0.5 * tint[cmask.sam_mask] + 0.5 * np.array(GREEN)).astype(np.uint8)
        img = tint
        cnts, _ = cv2.findContours(cmask.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, cnts, -1, YELLOW, 1, cv2.LINE_AA)
    if cav is not None:
        fp = cav.floor_points[:: max(1, len(cav.floor_points) // 1500)]
        for uv in _proj(frame, fp):
            if np.all(np.isfinite(uv)):
                cv2.circle(img, (int(uv[0]), int(uv[1])), 1, BLUE, -1)
        c = cav.corners()          # front/back x right/left x floor/top
        for a, b in ((0, 2), (2, 6), (6, 4), (4, 0), (1, 3), (3, 7), (7, 5), (5, 1), (0, 1), (2, 3), (4, 5), (6, 7)):
            _poly(img, frame, [c[a], c[b]], YELLOW, closed=False, th=1)
    if res is not None:
        _poly(img, frame, res.usable_floor["corners"], CYAN)
        _poly(img, frame, res.footprint, MAGENTA)
        _dot(img, frame, res.floor_contact, RED)
        _dot(img, frame, res.container_center, ORANGE)
        if res.tool_center is not None:
            _dot(img, frame, res.tool_center, WHITE)
    k = cmask.k_upright if cmask is not None else pcm.upright_k(frame.base_T_cam)
    img = np.ascontiguousarray(np.rot90(img, k)) if k else img
    lines = [(status, WHITE)] if status else []
    if res is not None:
        cc, fc = res.container_center, res.floor_contact
        lines += [(f"floor z {res.floor_z:.3f} tilt {res.floor_tilt_deg:.1f}deg rms {res.floor_rms_m * 1000:.1f}mm "
                   f"support {res.support_kind} {res.support_z:.3f}", BLUE),
                  (f"floor contact   {np.round(fc, 3).tolist()} ({res.frame}, m)", RED),
                  (f"container centre {np.round(cc, 3).tolist()} yaw {res.yaw_deg:+.1f} roll {res.roll_deg:+.0f}", ORANGE),
                  (f"tool centre     {np.round(res.tool_center, 3).tolist() if res.tool_center is not None else 'n/a (grasp_offset unmeasured)'}", WHITE)]
    return _text_panel(img, lines or [("", WHITE)], ok)


def top_down(frame_positions, cloud, cav=None, res=None, scale_px_per_m=500, size=600):
    pts = np.asarray(cloud, float).reshape(-1, 3)
    center = (np.array([(cav.front + cav.back) / 2 * cav.f[0] + (cav.left + cav.right) / 2 * cav.l[0],
                        (cav.front + cav.back) / 2 * cav.f[1] + (cav.left + cav.right) / 2 * cav.l[1]])
              if cav is not None else (pts[:, :2].mean(axis=0) if len(pts) else np.zeros(2)))
    img = np.full((size, size, 3), 30, np.uint8)

    def px(p):
        d = np.asarray(p, float)[..., :2] - center
        return np.stack([size / 2 - d[..., 1] * scale_px_per_m, size / 2 - d[..., 0] * scale_px_per_m], -1)

    def poly(p, color, th=2):
        cv2.polylines(img, [np.round(px(p)).astype(np.int32)], True, color, th, cv2.LINE_AA)

    def dot(p, color, r=6):
        c = tuple(int(round(v)) for v in px(p))
        cv2.circle(img, c, r, (0, 0, 0), -1, cv2.LINE_AA)
        cv2.circle(img, c, r - 2, color, -1, cv2.LINE_AA)

    for q in np.round(px(pts)).astype(int):
        if 0 <= q[0] < size and 0 <= q[1] < size:
            img[q[1], q[0]] = GREY
    if cav is not None:
        for q in np.round(px(cav.floor_points)).astype(int):
            if 0 <= q[0] < size and 0 <= q[1] < size:
                img[q[1], q[0]] = BLUE
        c = cav.corners()
        poly(c[[0, 2, 6, 4]], YELLOW, 1)
        a0 = cav.point(cav.front, (cav.left + cav.right) / 2, 0)
        a1 = cav.point(cav.front + 0.08, (cav.left + cav.right) / 2, 0)
        cv2.arrowedLine(img, tuple(np.round(px(a0)).astype(int)), tuple(np.round(px(a1)).astype(int)), YELLOW, 2,
                        tipLength=0.3)
    if res is not None:
        poly(res.usable_floor["corners"], CYAN)
        poly(res.footprint, MAGENTA)
        dot(res.floor_contact, RED, 8)
        dot(res.container_center, ORANGE)
        if res.tool_center is not None:
            dot(res.tool_center, WHITE)
    for cp_ in frame_positions:
        q = np.round(px(cp_)).astype(int)
        if 0 <= q[0] < size and 0 <= q[1] < size:
            cv2.drawMarker(img, tuple(q), GREEN, cv2.MARKER_TRIANGLE_UP, 14, 2)
    cv2.putText(img, "top-down, arm_base_link: +x up, +y left, 1 px = 2 mm", (6, 16), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, WHITE, 1, cv2.LINE_AA)
    cv2.putText(img, "grey cloud / blue floor / yellow cavity / cyan usable / magenta container / red contact / "
                "orange centre / white tool / green cam", (6, size - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.33, WHITE, 1,
                cv2.LINE_AA)
    return img
