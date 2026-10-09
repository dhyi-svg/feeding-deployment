"""The open microwave's cavity from a live point cloud (arm_base_link), and where the box goes.

Replaces the SAM 3 mask + door-file axes (`microwave_cavity.py`, removed): nothing comes from
`~/.microwave_door.json`. Everything is measured from depth:

1. Sequential RANSAC planes over the detector-ROI cloud, each split into connected pieces,
   normals oriented toward the camera, classed by gravity: 'up' (floor-like), 'down'
   (ceiling-like), 'wall' (vertical), 'other'.
2. Back wall = a wall facing the camera such that an 'up' plane (the floor) runs right up to
   it -- the countertop in front of the microwave stops at the front face, and the front face
   has floor running 20-40 cm past it, so neither qualifies. Insertion axis f = -back normal.
3. Floor = the biggest such 'up' plane; refused unless it is level (normal within
   floor_max_tilt_deg of gravity -- a wrong camera roll in the calibration fails here), flat
   (RMS), big enough, below the camera and at a plausible height.
4. Opening (front) = where the floor starts (3rd percentile along f); side walls = vertical
   planes parallel to f behind the opening (the open door is in front of it, so it is never
   taken as a wall); missing walls fall back to the visible floor's extent (narrower = safe).
   Top = a ceiling plane, else the highest visible wall points (lower = safe).
5. `placement_target` puts the container's footprint inside the walls with clearances, at
   the support height measured under that footprint (a turntable counts), and refuses if
   anything else stands there.

Pure numpy/scipy; `CavityError` carries the reason. Callers never fall back to a guess.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

Z = np.array([0.0, 0.0, 1.0])


class CavityError(ValueError):
    """The cloud is not a plausible open-microwave interior (message says why)."""


@dataclass
class Plane:
    normal: np.ndarray       # unit, oriented toward the camera
    d: float                 # normal . x + d = 0
    points: np.ndarray       # inliers (one connected piece)
    rms: float
    kind: str = "other"

    @property
    def centroid(self):
        return self.points.mean(axis=0)

    def tilt_deg(self):
        """Angle between the normal and world +z."""
        return float(np.degrees(np.arccos(np.clip(self.normal @ Z, -1.0, 1.0))))


@dataclass
class Cavity:
    f: np.ndarray            # horizontal unit, into the microwave
    l: np.ndarray            # horizontal unit, left (z x f)
    front: float             # along f (dot product with arm_base_link coordinates)
    back: float
    left: float              # along l
    right: float
    floor_z: float           # floor plane height at the cavity centre
    top: float               # z
    floor_normal: np.ndarray
    floor_rms: float
    floor_points: np.ndarray
    sources: dict = field(default_factory=dict)   # which measurement gave each bound
    planes: list = field(default_factory=list)

    @property
    def depth(self):
        return self.back - self.front

    @property
    def width(self):
        return self.left - self.right

    @property
    def height(self):
        return self.top - self.floor_z

    def floor_tilt_deg(self):
        return float(np.degrees(np.arccos(np.clip(self.floor_normal @ Z, -1.0, 1.0))))

    def point(self, along_f, along_l, z):
        """arm_base_link point with the given f / l coordinates and height."""
        return self.f * along_f + self.l * along_l + Z * z

    def floor_z_at(self, xy):
        n = self.floor_normal
        c = self.floor_points.mean(axis=0)
        return float(c[2] - (n[0] * (xy[0] - c[0]) + n[1] * (xy[1] - c[1])) / n[2])

    def corners(self):
        """8 corners of the cavity box (for markers/collision): front/back x right/left x floor/top."""
        return np.array([self.point(a, b, c) for a in (self.front, self.back)
                         for b in (self.right, self.left) for c in (self.floor_z, self.top)])

    def summary(self):
        return (f"cavity depth {self.depth * 100:.1f} x width {self.width * 100:.1f} x height "
                f"{self.height * 100:.1f} cm; floor z {self.floor_z:.3f} (tilt {self.floor_tilt_deg():.1f} deg, "
                f"rms {self.floor_rms * 1000:.1f} mm); f {np.round(self.f, 3).tolist()}; "
                f"sources {self.sources}")


# ---------------------------------------------------------------------------
# planes
# ---------------------------------------------------------------------------
def fit_plane(points):
    """Least-squares plane: (unit normal, d, rms)."""
    c = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - c, full_matrices=False)
    n = vt[-1]
    dist = (points - c) @ n
    return n, float(-n @ c), float(np.sqrt(np.mean(dist ** 2)))


def ransac_plane(points, dist, iters, rng):
    """Best plane by inlier count, refit on its inliers. Returns (normal, d, inlier mask)."""
    n_pts = len(points)
    best, best_count = None, -1
    for _ in range(iters):
        p = points[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        count = int(np.count_nonzero(np.abs((points - p[0]) @ n) < dist))
        if count > best_count:
            best, best_count = (n, p[0]), count
    if best is None:
        return None, None, np.zeros(n_pts, bool)
    n, p0 = best
    mask = np.abs((points - p0) @ n) < dist
    n, d, _ = fit_plane(points[mask])
    mask = np.abs(points @ n + d) < dist
    return n, d, mask


def connected_pieces(points, eps, min_points, return_index=False):
    """Split points into connected components (neighbours within eps); pieces >= min_points,
    biggest first (or their index arrays with return_index)."""
    if len(points) < min_points:
        return []
    pairs = cKDTree(points).query_pairs(eps, output_type="ndarray")
    from scipy.sparse import coo_matrix
    m = coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])), shape=(len(points),) * 2)
    _, labels = connected_components(m, directed=False)
    counts = np.bincount(labels)
    idx = [np.flatnonzero(labels == lab) for lab in np.argsort(-counts) if counts[lab] >= min_points]
    return idx if return_index else [points[i] for i in idx]


def classify(plane, cfg):
    t = plane.tilt_deg()
    if t <= cfg.horizontal_tol_deg:
        return "up"
    if t >= 180.0 - cfg.horizontal_tol_deg:
        return "down"
    if abs(90.0 - t) <= cfg.vertical_tol_deg:
        return "wall"
    return "other"


def is_area(points, cell=0.015, min_interior=8, min_frac=0.15):
    """True if the points cover a 2-D area in xy (a floor/turntable), not a line (a horizontal
    slice through walls): enough occupied grid cells whose 4 neighbours are all occupied."""
    keys = np.unique(np.floor(points[:, :2] / cell).astype(np.int64), axis=0)
    occ = {tuple(k) for k in keys}
    interior = sum(1 for (i, j) in occ if {(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)} <= occ)
    return interior >= max(min_interior, min_frac * len(occ))


def horizontal_sheets(points, cam_pos, cfg):
    """Level surfaces as peaks of the height histogram (gravity is known: floor, turntable and
    ceiling are each a sharp z peak, walls spread thinly over z). Each peak +-sheet_half_m ->
    connected pieces -> plane fit; kept if level within horizontal_tol_deg. Returns (planes,
    mask of the points used)."""
    z = points[:, 2]
    used = np.zeros(len(points), bool)
    keep = np.zeros(len(points), bool)
    if len(points) == 0:
        return [], keep
    bins = np.arange(z.min() - 0.004, z.max() + 0.006, 0.002)
    hist, edges = np.histogram(z, bins)
    win = int(round(cfg.sheet_half_m / 0.002))
    counts = np.convolve(hist, np.ones(2 * win + 1, int), mode="same")
    planes = []
    for i in np.argsort(-counts):
        if counts[i] < cfg.min_plane_points:
            break
        zc = 0.5 * (edges[i] + edges[i + 1])
        sel = (~used) & (np.abs(z - zc) <= cfg.sheet_half_m)
        if np.count_nonzero(sel) < cfg.min_plane_points:
            continue
        sel_idx = np.flatnonzero(sel)
        for piece_idx in connected_pieces(points[sel], cfg.cluster_eps_m, cfg.min_plane_points, return_index=True):
            piece = points[sel_idx[piece_idx]]
            pn, pd, rms = fit_plane(piece)
            if pn @ (cam_pos - piece.mean(axis=0)) < 0:
                pn, pd = -pn, -pd
            pl = Plane(pn, pd, piece, rms)
            pl.kind = classify(pl, cfg)
            if pl.kind in ("up", "down") and is_area(piece):
                planes.append(pl)
                keep[sel_idx[piece_idx]] = True
        used |= sel
    return planes, keep


def extract_planes(points, cam_pos, cfg, seed=0):
    rng = np.random.default_rng(seed)
    points = np.ascontiguousarray(points, dtype=float)
    planes, in_sheet = horizontal_sheets(points, cam_pos, cfg)
    remaining = points[~in_sheet]
    for _ in range(cfg.max_planes):
        if len(remaining) < max(cfg.min_plane_points, 3):
            break
        n, d, mask = ransac_plane(remaining, cfg.plane_dist_m, cfg.plane_iters, rng)
        if n is None or np.count_nonzero(mask) < cfg.min_plane_points:
            break
        for piece in connected_pieces(remaining[mask], cfg.cluster_eps_m, cfg.min_plane_points):
            pn, pd, rms = fit_plane(piece)
            if pn @ (cam_pos - piece.mean(axis=0)) < 0:      # orient toward the camera
                pn, pd = -pn, -pd
            pl = Plane(pn, pd, piece, rms)
            pl.kind = classify(pl, cfg)
            planes.append(pl)
        remaining = remaining[~mask]
    return planes


def _refine_floor(floor, cam_pos, cfg, seed=1):
    """Re-fit the floor with a tighter band and keep the biggest connected sheet: a turntable
    ~1 cm above the floor otherwise drags the fit into a tilted average of the two."""
    n, d, mask = ransac_plane(floor.points, cfg.floor_refine_dist_m, cfg.plane_iters, np.random.default_rng(seed))
    if n is None or np.count_nonzero(mask) < cfg.min_plane_points:
        return floor
    pieces = connected_pieces(floor.points[mask], cfg.cluster_eps_m, cfg.min_plane_points)
    if not pieces:
        return floor
    pn, pd, rms = fit_plane(pieces[0])
    if pn @ (cam_pos - pieces[0].mean(axis=0)) < 0:
        pn, pd = -pn, -pd
    out = Plane(pn, pd, pieces[0], rms)
    out.kind = classify(out, cfg)
    return out if out.kind == "up" else floor


def _horizontal(v):
    v = np.array([v[0], v[1], 0.0])
    n = np.linalg.norm(v)
    return v / n if n > 1e-9 else None


# ---------------------------------------------------------------------------
# cavity
# ---------------------------------------------------------------------------
def estimate_cavity(points, cam_pos, cfg, seed=0):
    """Cavity bounds from the ROI cloud (arm_base_link). Raises CavityError."""
    points = np.asarray(points, float)
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < cfg.min_plane_points * 3:
        raise CavityError(f"only {len(points)} points in the microwave region")
    cam_pos = np.asarray(cam_pos, float)
    planes = extract_planes(points, cam_pos, cfg, seed)
    if not planes:
        raise CavityError("no planes found in the microwave region")
    ups = [p for p in planes if p.kind == "up"]
    walls = [p for p in planes if p.kind == "wall"]
    if not ups:
        tilts = ", ".join(f"{p.tilt_deg():.0f}" for p in sorted(planes, key=lambda p: -len(p.points))[:4])
        raise CavityError(f"no level upward-facing surface (largest planes' normals are {tilts} deg from "
                          "world up) -- is the camera calibration's roll right? (tools/check_camera_roll.py)")
    view_h = _horizontal(points.mean(axis=0) - cam_pos)
    if view_h is None:
        raise CavityError("camera is looking straight down -- no insertion direction")

    # back wall: faces the camera, and a floor runs up to it
    cos_view = np.cos(np.radians(cfg.back_wall_max_view_angle_deg))
    cands = []
    for w in walls:
        f = _horizontal(-w.normal)
        if f is not None and f @ view_h >= cos_view:
            cands.append((float(w.centroid @ view_h), w, f))
    floor = back_wall = f = None
    for _, w, fw in sorted(cands, key=lambda c: -c[0]):          # farthest first
        back = float(np.median(w.points @ fw))
        ok = [u for u in ups if back - cfg.floor_reaches_back_m <= np.percentile(u.points @ fw, 97) <= back + 0.02]
        if ok:
            # the LOWEST level sheet that runs to the back wall is the floor (a turntable sits on it);
            # where only the turntable reaches the back (the ring behind it hidden), the floor is the
            # sheet just below it in the same footprint (the visible ring in front / beside it)
            anchor = min(ok, key=lambda u: np.median(u.points[:, 2]))
            az, af = np.median(anchor.points[:, 2]), np.percentile(anchor.points @ fw, 3)
            below = [u for u in ups if 0.003 < az - np.median(u.points[:, 2]) <= cfg.floor_complex_band_m
                     and np.percentile(u.points @ fw, 97) <= back + 0.02
                     and np.percentile(u.points @ fw, 3) >= af - 0.08]
            floor = min(below, key=lambda u: np.median(u.points[:, 2])) if below else anchor
            back_wall, f = w, fw
            break
    if floor is None:
        raise CavityError(f"no floor that runs up to a back wall ({len(cands)} back-wall candidates, "
                          f"{len(ups)} level surfaces) -- is the door open and the camera looking in?")
    l = np.cross(Z, f)
    floor0, floor = floor, _refine_floor(floor, cam_pos, cfg)

    # floor checks
    tilt = float(np.degrees(np.arccos(np.clip(floor.normal @ Z, -1.0, 1.0))))
    if tilt > cfg.floor_max_tilt_deg:
        raise CavityError(f"floor normal {tilt:.1f} deg from world up (max {cfg.floor_max_tilt_deg}) -- "
                          "camera calibration (roll) or a sloped surface; refusing")
    if floor.rms > cfg.floor_max_rms_m:
        raise CavityError(f"floor fit RMS {floor.rms * 1000:.1f} mm (max {cfg.floor_max_rms_m * 1000:.0f})")
    # the floor complex: the floor plus level sheets just above it (turntable, rim) behind the back
    # wall's front -- together they give where the floor starts and how wide it is
    fz = float(np.median(floor.points[:, 2]))
    complex_pts = [floor.points] + [u.points for u in ups if u is not floor0
                                    and 0.0 <= np.median(u.points[:, 2]) - fz <= cfg.floor_complex_band_m
                                    and np.percentile(u.points @ f, 97) <= back + 0.02]
    complex_pts = np.vstack(complex_pts)
    ff, fl = complex_pts @ f, complex_pts @ l
    ext_f = np.percentile(ff, 97) - np.percentile(ff, 3)
    ext_l = np.percentile(fl, 97) - np.percentile(fl, 3)
    if min(ext_f, ext_l) < cfg.floor_min_extent_m:
        raise CavityError(f"visible floor only {ext_f * 100:.0f} x {ext_l * 100:.0f} cm "
                          f"(need {cfg.floor_min_extent_m * 100:.0f} cm each way)")
    floor_z = float(np.median(floor.points[:, 2]))
    if not cfg.floor_z_range[0] <= floor_z <= cfg.floor_z_range[1]:
        raise CavityError(f"floor z {floor_z:.3f} outside {cfg.floor_z_range}")
    if cam_pos[2] - floor_z < cfg.floor_below_camera_m:
        raise CavityError(f"floor z {floor_z:.3f} is not below the camera (z {cam_pos[2]:.3f})")

    sources = {}
    back = float(np.median(back_wall.points @ f))
    front = float(np.percentile(ff, 3))
    sources["front"], sources["back"] = "floor start", "back wall plane"
    # the exterior face around the opening (faces the camera, between the floor start and ~6 cm in front)
    for w in walls:
        if w is back_wall or _horizontal(-w.normal) is None or _horizontal(-w.normal) @ f < np.cos(np.radians(20)):
            continue
        wf = float(np.median(w.points @ f))
        if front - 0.06 <= wf <= front + 0.03 and wf > front:
            front, sources["front"] = wf, "front face plane"
    depth = back - front
    if not cfg.min_depth_m <= depth <= cfg.max_depth_m:
        raise CavityError(f"cavity depth {depth * 100:.1f} cm outside "
                          f"{cfg.min_depth_m * 100:.0f}-{cfg.max_depth_m * 100:.0f}")

    # side walls: parallel to f, behind the opening, beside the floor
    mid_l = 0.5 * (np.percentile(fl, 3) + np.percentile(fl, 97))
    left, right = float(np.percentile(fl, 97)), float(np.percentile(fl, 3))
    sources["left"] = sources["right"] = "visible floor extent"
    sin_par = np.sin(np.radians(25))
    best_l, best_r = None, None
    for w in walls:
        if w is back_wall or abs(w.normal @ f) > sin_par:
            continue
        if np.median(w.points @ f) < front + cfg.wall_behind_front_m:
            continue                                      # in front of the opening (the open door, the frame)
        wz = np.median(w.points[:, 2])
        if not floor_z - 0.02 <= wz <= floor_z + cfg.max_height_m:
            continue
        s = float(np.median(w.points @ l))
        if s > mid_l and w.normal @ l < 0 and (best_l is None or s < best_l):
            best_l = s
        elif s < mid_l and w.normal @ l > 0 and (best_r is None or s > best_r):
            best_r = s
    if best_l is not None and best_l >= left - 0.01:
        left, sources["left"] = best_l, "side wall plane"
    if best_r is not None and best_r <= right + 0.01:
        right, sources["right"] = best_r, "side wall plane"
    width = left - right
    if not cfg.min_width_m <= width <= cfg.max_width_m:
        raise CavityError(f"cavity width {width * 100:.1f} cm outside "
                          f"{cfg.min_width_m * 100:.0f}-{cfg.max_width_m * 100:.0f}")

    # top: a ceiling plane, else the highest visible wall points
    top, sources["top"] = None, None
    for c in planes:
        if c.kind != "down":
            continue
        cz = float(np.median(c.points[:, 2]))
        if np.median(c.points @ f) > front + 0.03 and cz > floor_z + cfg.min_height_m and (top is None or cz < top):
            top, sources["top"] = cz, "ceiling plane"
    # the opening's top edge: the lower edge of the front face above the opening (seen from above,
    # where the ceiling itself is hidden). Points within 3 cm of the front plane, inside the side walls.
    pf, pl = points @ f, points @ l
    rim = points[(np.abs(pf - front) <= 0.03) & (pl > right + 0.02) & (pl < left - 0.02)
                 & (points[:, 2] > floor_z + cfg.min_height_m)]
    if len(rim) >= 30:
        rim_top = float(np.percentile(rim[:, 2], 3))
        if top is None or rim_top < top:
            top, sources["top"] = rim_top, "front face above the opening"
    if top is None:
        wall_z = [np.percentile(w.points[:, 2], 97) for w in walls
                  if np.median(w.points @ f) > front]
        if wall_z:
            top, sources["top"] = float(max(wall_z)), "highest visible wall points"
    if top is None:
        raise CavityError("cavity top not measurable (no ceiling, no walls)")
    height = top - floor_z
    if not cfg.min_height_m <= height <= cfg.max_height_m:
        raise CavityError(f"cavity height {height * 100:.1f} cm outside "
                          f"{cfg.min_height_m * 100:.0f}-{cfg.max_height_m * 100:.0f}")

    return Cavity(f=f, l=l, front=front, back=back, left=left, right=right, floor_z=floor_z, top=top,
                  floor_normal=floor.normal, floor_rms=floor.rms, floor_points=floor.points,
                  sources=sources, planes=planes)


def support_height(cavity, points, center_f, center_l, half_f, half_l, band, top_margin=0.01):
    """Highest support under a footprint: the floor plane there, or a raised surface within `band`
    of it (turntable). Raises CavityError if something taller stands in the footprint."""
    pts = np.asarray(points, float)
    pf, pl = pts @ cavity.f, pts @ cavity.l
    inside = (np.abs(pf - center_f) <= half_f) & (np.abs(pl - center_l) <= half_l)
    col = pts[inside]
    xy = cavity.point(center_f, center_l, 0.0)[:2]
    z_floor = cavity.floor_z_at(xy)
    if len(col) == 0:
        return z_floor, 0
    above = col[:, 2] - z_floor
    blocking = (above > band) & (col[:, 2] < cavity.top - top_margin)
    if np.count_nonzero(blocking) >= 20:
        raise CavityError(f"{np.count_nonzero(blocking)} points stand {np.median(above[blocking]) * 100:.0f} cm "
                          "above the floor where the container goes -- something is in the microwave")
    band_pts = col[(above >= -0.01) & (above <= band), 2]
    if len(band_pts) < 30:
        return z_floor, len(band_pts)
    return max(z_floor, float(np.percentile(band_pts, 98))), len(band_pts)


def merge_cavities(cavs, cfg):
    """Agreement across looks: returns (merged Cavity, inlier indices). A look is an inlier when
    its axis, floor and centre agree with the median look. Raises CavityError if fewer than
    cfg.min_looks agree."""
    if len(cavs) < cfg.min_looks:
        raise CavityError(f"{len(cavs)} looks, need {cfg.min_looks}")
    keys = np.array([[c.front, c.back, c.left, c.right, c.floor_z, c.top] for c in cavs])
    med = np.median(keys, axis=0)
    fs = np.array([c.f for c in cavs])
    f_med = _horizontal(np.median(fs, axis=0))
    centers = np.array([c.point((c.front + c.back) / 2, (c.left + c.right) / 2, c.floor_z) for c in cavs])
    c_med = np.median(centers, axis=0)
    inl = [i for i, c in enumerate(cavs)
           if np.degrees(np.arccos(np.clip(c.f @ f_med, -1, 1))) <= cfg.agree_axis_deg
           and abs(c.floor_z - med[4]) <= cfg.agree_floor_m
           and np.linalg.norm(centers[i] - c_med) <= cfg.agree_target_m]
    if len(inl) < cfg.min_looks:
        raise CavityError(f"only {len(inl)} of {len(cavs)} looks agree (axis {cfg.agree_axis_deg} deg, floor "
                          f"{cfg.agree_floor_m * 100:.0f} cm, centre {cfg.agree_target_m * 100:.0f} cm); need {cfg.min_looks}")
    sel = [cavs[i] for i in inl]
    f = _horizontal(np.mean([c.f for c in sel], axis=0))
    l = np.cross(Z, f)
    # re-express every bound along the merged axes (the per-look axes differ by < agree_axis_deg)
    fronts = [c.point(c.front, (c.left + c.right) / 2, 0) @ f for c in sel]
    backs = [c.point(c.back, (c.left + c.right) / 2, 0) @ f for c in sel]
    lefts = [c.point((c.front + c.back) / 2, c.left, 0) @ l for c in sel]
    rights = [c.point((c.front + c.back) / 2, c.right, 0) @ l for c in sel]
    normals = np.array([c.floor_normal for c in sel])
    n = normals.mean(axis=0)
    n /= np.linalg.norm(n)
    merged = Cavity(f=f, l=l,
                    front=float(np.max(fronts)), back=float(np.min(backs)),       # conservative: the
                    left=float(np.min(lefts)), right=float(np.max(rights)),       # smallest box all looks see
                    floor_z=float(np.median([c.floor_z for c in sel])), top=float(np.min([c.top for c in sel])),
                    floor_normal=n, floor_rms=float(np.max([c.floor_rms for c in sel])),
                    floor_points=np.vstack([c.floor_points for c in sel]),
                    sources=sel[-1].sources, planes=sel[-1].planes)
    return merged, inl


# ---------------------------------------------------------------------------
# where the container goes
# ---------------------------------------------------------------------------
def aligned_orientation(quat_now, f, pcfg):
    """The hold orientation for insertion: the approach (tool z) along f, horizontal, and the
    tool direction that points up NOW pointing exactly up -- the smallest re-orientation that
    levels the held box and aims it into the cavity. Returns (quat, info). Raises CavityError
    if the correction is larger than allowed."""
    Rn = R.from_quat(quat_now).as_matrix()
    a_local = np.array([0.0, 0.0, 1.0])
    up_local = Rn.T @ Z
    up_local -= a_local * (up_local @ a_local)
    if np.linalg.norm(up_local) < 0.2:
        raise CavityError("the approach axis is (nearly) vertical -- not a level container hold")
    up_local /= np.linalg.norm(up_local)
    L = np.column_stack([a_local, up_local, np.cross(a_local, up_local)])
    W = np.column_stack([f, Z, np.cross(f, Z)])
    R_new = W @ L.T
    a_now = Rn[:, 2]
    a_h = _horizontal(a_now)
    yaw = float(np.degrees(np.arccos(np.clip(a_h @ f, -1, 1)))) if a_h is not None else 90.0
    pitch = float(np.degrees(np.arcsin(np.clip(a_now[2], -1, 1))))
    roll = float(np.degrees(np.arccos(np.clip((Rn @ up_local) @ Z, -1, 1))))
    info = {"yaw_deg": yaw, "approach_tilt_deg": pitch, "up_tilt_deg": roll, "up_local": up_local,
            "total_deg": float(np.degrees((R.from_matrix(Rn).inv() * R.from_matrix(R_new)).magnitude()))}
    if yaw > pcfg.max_yaw_correction_deg:
        raise CavityError(f"the held box points {yaw:.0f} deg off the cavity axis (max {pcfg.max_yaw_correction_deg})")
    if max(abs(pitch), roll) > pcfg.max_tilt_correction_deg:
        raise CavityError(f"the held box is tilted {max(abs(pitch), roll):.1f} deg "
                          f"(max {pcfg.max_tilt_correction_deg}) -- not a level hold")
    return R.from_matrix(R_new).as_quat(), info


def container_box_world(tool_pos, quat, ccfg):
    """Held box as (centre, 3x3 axes [along approach, lateral, up-ish], half extents) for a tool pose."""
    Rm = R.from_quat(quat).as_matrix()
    a = Rm[:, 2]
    up = _horizontal_perp_up(a)
    lat = np.cross(a, up)
    length = ccfg.far_past_tool - ccfg.near_past_tool
    center = (np.asarray(tool_pos, float) + a * (ccfg.near_past_tool + length / 2)
              + lat * ccfg.lateral_offset + up * (-ccfg.drop + ccfg.height / 2))
    return center, np.column_stack([a, lat, up]), np.array([length / 2, ccfg.width / 2, ccfg.height / 2])


def _horizontal_perp_up(a):
    up = Z - a * (a @ Z)
    n = np.linalg.norm(up)
    return up / n if n > 1e-6 else np.array([1.0, 0.0, 0.0])


@dataclass
class PlacementTarget:
    quat: np.ndarray            # hold orientation for pre-insert .. place (aligned, level)
    pre: np.ndarray             # tool positions
    above: np.ndarray
    contact: np.ndarray         # box bottom on the support (impedance lowering target)
    place: np.ndarray           # box bottom release_gap above it (position lowering end)
    support_z: float
    box_center: np.ndarray      # footprint centre on the support
    align_info: dict
    checks: list


def placement_target(cavity, cloud, quat_now, ccfg, pcfg, cav_cfg):
    """Tool poses for putting the held box on the cavity floor. Raises CavityError."""
    if ccfg.drop <= 0:
        raise CavityError("container.drop (tool frame -> container bottom, m) is not set -- measure it")
    quat, info = aligned_orientation(quat_now, cavity.f, pcfg)
    f = cavity.f
    length = ccfg.far_past_tool - ccfg.near_past_tool
    half_w = ccfg.width / 2
    checks = []

    # lateral: centred between the walls
    room = (cavity.width / 2) - half_w - pcfg.side_clearance_m
    checks.append(f"lateral room {room * 100:+.1f} cm (box {ccfg.width * 100:.0f} cm in {cavity.width * 100:.1f} cm, "
                  f"{pcfg.side_clearance_m * 100:.1f} cm each side)")
    if room < 0:
        raise CavityError(f"box does not fit between the side walls: {checks[-1]}")
    c_l = 0.5 * (cavity.left + cavity.right)
    # along f: near end just inside the opening; far end clear of the back wall
    c_f = cavity.front + pcfg.front_inside_m + length / 2
    far_gap = cavity.back - (c_f + length / 2)
    checks.append(f"back wall gap {far_gap * 100:.1f} cm (need {pcfg.back_clearance_m * 100:.1f})")
    if far_gap < pcfg.back_clearance_m:
        raise CavityError(f"box too long for the cavity depth: {checks[-1]}")

    support, n_sup = support_height(cavity, cloud, c_f, c_l, length / 2 + 0.01, half_w + 0.01,
                                    cav_cfg.support_band_m)
    checks.append(f"support z {support:.3f} (floor {cavity.floor_z:.3f}, {n_sup} points in the footprint)")

    # tool position for a box-centre (c_f, c_l) footprint, the box bottom at height zb
    Rm = R.from_quat(quat).as_matrix()
    a, up = Rm[:, 2], Z
    lat = np.cross(a, up)

    def tool_at(cf, zb):
        center = cavity.point(cf, c_l, 0.0)
        p = center - a * (ccfg.near_past_tool + length / 2) - lat * ccfg.lateral_offset
        p[2] = zb + ccfg.drop
        return p

    contact = tool_at(c_f, support)
    place = tool_at(c_f, support + pcfg.release_gap_m)
    above = tool_at(c_f, support + pcfg.insert_clearance_m)
    box_top = support + pcfg.insert_clearance_m + ccfg.height
    checks.append(f"box top while inserting {box_top:.3f} vs cavity top {cavity.top:.3f} "
                  f"(need {pcfg.top_clearance_m * 100:.0f} cm)")
    if box_top > cavity.top - pcfg.top_clearance_m:
        raise CavityError(f"box too tall for the opening: {checks[-1]}")
    far_end_above = above @ f + ccfg.far_past_tool
    pre = above - f * (far_end_above - (cavity.front - pcfg.pre_insert_standoff_m))
    reach = max(float(np.linalg.norm(p)) for p in (pre, above, contact))
    checks.append(f"max reach {reach:.3f} m (max {pcfg.max_reach_m})")
    if reach > pcfg.max_reach_m:
        raise CavityError(f"placement out of reach: {checks[-1]} -- move the base closer")
    return PlacementTarget(quat=quat, pre=pre, above=above, contact=contact, place=place, support_z=support,
                           box_center=cavity.point(c_f, c_l, support), align_info=info, checks=checks)
