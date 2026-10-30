"""Cavity mask + aligned depth -> microwave floor -> placement target in Part 1's frame.

All XYZ here is MEASURED: mask pixels with valid aligned depth, deprojected with the camera's K and
moved into arm_base_link with the frame's `base_T_cam`. Nothing is taken from the old door JSON.

1. cloud      every (dilated) cavity-mask pixel with depth in [depth_min, depth_max] -> camera
              points (+ depth_corr along the optical axis, 0 = uncorrected) -> arm_base_link ->
              8 mm voxels -> statistical outlier removal. Refused below min_valid_depth_frac / min_points.
2. floor      cavity_method "opening": `opening_cavity.estimate_opening_cavity` instead (back wall, front
              face and opening outline; no floor view needed). cavity_method "floor", the default:
              `cavity_perception.estimate_cavity` (unchanged; regression-tested on the 10-20 rig frame):
              gravity-sorted planes; the floor is the biggest level, upward-facing sheet that runs up to
              the back wall (a countertop in front of the microwave does not), refit in a 4 mm band
              (separates a turntable ~1 cm above it); refused unless level (<= floor_max_tilt_deg --
              also catches a wrong camera calibration), flat (RMS), big enough, below the camera.
              Side/back walls and the top bound the usable region; walls not seen fall back to the
              visible floor's extent (narrower = safe).
3. looks      several frames: `cavity_perception.merge_cavities` (axis, floor and centre must agree;
              the merged box is the smallest all looks saw). One frame = not cross-checked (warned).
4. target     usable floor = cavity rectangle inset by the clearances; the container (its length along
              the insertion axis f = Part 1's approach, held level by HOLD_LEVEL) is centred in it;
              the support height under its footprint (floor, or a turntable within support_band) gives
                floor_contact     = support point under the container centre
                container_center  = floor_contact + up * (place_gap + max corner rise + height / 2)
                tool_center       = container_center - R_goal @ grasp_offset   (Part 1's tool_goal;
                                    only with a MEASURED grasp_offset, else None)
              R_goal = Part 1's target_orientation(yaw = heading of f, roll = -90).

Frames/units: arm_base_link (= Part 1's base_link), metres; yaw/roll in degrees as Part 1 takes them.
"""
from __future__ import annotations

import copy
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
import yaml

import cavity_perception as cp
import point_cloud as pcm
from cavity_segmentation import SegmentationConfig
from opening_cavity import OpeningConfig, estimate_opening_cavity
from placement_config import CavityConfig, PointCloudConfig, _merge
from rgbd_source import BASE_FRAME, CameraConfig

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG_FILE = HERE / "perception_config.yaml"


class PerceptionError(RuntimeError):
    """The observation does not give a trustworthy target (message says why)."""


@dataclass
class ContainerSpec:
    """The held container. NOMINAL values are not measured on the rig -- the result says so."""
    length_m: float = 0.16             # along the approach / insertion axis. NOMINAL
    width_m: float = 0.12              # across it, horizontal. NOMINAL
    height_m: float = 0.112            # bottom -> top (OXO POP, measured by RAMMP)
    # tool frame -> container centre, in the TOOL frame (Part 1's --grasp-offset). None = not measured
    grasp_offset: tuple = None
    finger_below_tool_m: float = None  # lowest finger below the tool centre, gripper rolled to place. None = unchecked
    nominal: tuple = ("length_m", "width_m")   # which of the above are still guesses


@dataclass
class TargetConfig:
    side_clearance_m: float = 0.025    # container side -> side wall
    back_clearance_m: float = 0.04     # container far end -> back wall
    front_inside_m: float = 0.03       # container near end at least this far inside the opening
    top_clearance_m: float = 0.03      # container top -> cavity top at the target
    place_gap_m: float = 0.01          # container bottom above the support at the target (the last bit
                                       # down is the placement motion's job, not perception's)
    roll_deg: float = -90.0            # Part 1's task roll (wrist camera LEFT, confirmed 10-23)
    max_reach_m: float = 0.91          # = xyz_move.MAX_REACH_M: reported, Part 1 enforces it
    z_range_m: tuple = (0.12, 0.75)    # = xyz_move.Z_RANGE_M
    # cavity_method opening, footprint not seen: MEASURED height of the support above the shell floor
    # (turntable; 0 = none). None = refuse: an unseen turntable would eat the whole place_gap.
    blind_support_height_m: float = None


@dataclass
class PerceptionConfig:
    camera: CameraConfig = field(default_factory=CameraConfig)
    segmentation: SegmentationConfig = field(default_factory=SegmentationConfig)
    cloud: PointCloudConfig = field(default_factory=PointCloudConfig)
    cavity: CavityConfig = field(default_factory=CavityConfig)
    container: ContainerSpec = field(default_factory=ContainerSpec)
    target: TargetConfig = field(default_factory=TargetConfig)
    cavity_method: str = "floor"       # "floor" (estimate_cavity, needs the floor in view) | "opening"
    opening: OpeningConfig = field(default_factory=OpeningConfig)


def load_perception_config(path=None, overrides=None):
    """Defaults <- perception_config.yaml (or `path`) <- overrides. Unknown keys are errors."""
    cfg = PerceptionConfig()
    path = Path(path) if path else (DEFAULT_CONFIG_FILE if DEFAULT_CONFIG_FILE.exists() else None)
    if path is not None:
        _merge(cfg, yaml.safe_load(Path(path).read_text()) or {})
    _merge(cfg, copy.deepcopy(overrides or {}))
    if cfg.cavity_method not in ("floor", "opening"):
        raise ValueError(f"cavity_method must be 'floor' or 'opening', got {cfg.cavity_method!r}")
    go = cfg.container.grasp_offset
    if go is not None and len(go) != 3:
        raise ValueError("container.grasp_offset must be [x, y, z] (tool frame, m) or null")
    return cfg


# ---------------------------------------------------------------------------
# 1. cloud
# ---------------------------------------------------------------------------
@dataclass
class CavityCloud:
    points: np.ndarray       # filtered, arm_base_link
    n_mask: int
    n_valid: int

    @property
    def valid_frac(self):
        return self.n_valid / max(1, self.n_mask)


def cavity_cloud(frame, mask, cfg: PointCloudConfig):
    """Valid-depth mask pixels -> filtered arm_base_link points. Raises PerceptionError."""
    ys, xs = np.nonzero(mask)
    d = frame.depth_m[ys, xs].astype(float)
    ok = np.isfinite(d) & (d > cfg.depth_min_m) & (d < cfg.depth_max_m)
    n_mask, n_valid = len(d), int(ok.sum())
    if n_valid < max(1, cfg.min_valid_depth_frac * n_mask):
        raise PerceptionError(f"only {n_valid} of {n_mask} cavity pixels have depth in "
                              f"[{cfg.depth_min_m}, {cfg.depth_max_m}] m ({n_valid / max(1, n_mask) * 100:.0f}% < "
                              f"{cfg.min_valid_depth_frac * 100:.0f}%) -- glare, too close/far, or no aligned depth")
    K = frame.K
    z = d[ok]
    cam = np.stack([(xs[ok] - K[0, 2]) * z / K[0, 0], (ys[ok] - K[1, 2]) * z / K[1, 1], z + cfg.depth_corr_m], axis=1)
    pts = pcm.filter_cloud(pcm.transform_points(frame.base_T_cam, cam), cfg)
    if len(pts) < cfg.min_points:
        raise PerceptionError(f"{len(pts)} cavity points after filtering (< {cfg.min_points})")
    return CavityCloud(pts, n_mask, n_valid)


# ---------------------------------------------------------------------------
# 2-3. floor / cavity, across looks
# ---------------------------------------------------------------------------
def estimate_floor(clouds, cam_positions, cavity_cfg: CavityConfig, min_looks=None):
    """Per-look cavities, then their agreement. Returns (Cavity, per-look Cavities, inlier idx, notes).
    Raises PerceptionError."""
    cavs, notes = [], []
    for i, (c, cam) in enumerate(zip(clouds, cam_positions)):
        try:
            cavs.append(cp.estimate_cavity(c.points, cam, cavity_cfg))
        except cp.CavityError as e:
            notes.append(f"look {i + 1}: {e}")
    if not cavs:
        raise PerceptionError("no look gave a valid cavity: " + "; ".join(notes))
    return _merge_looks(cavs, cavity_cfg, min_looks, notes)


def estimate_opening(observations, cfg: PerceptionConfig, min_looks=None):
    """estimate_floor's return, from each look's back wall + front face + opening. Raises PerceptionError."""
    cavs, notes = [], []
    for i, o in enumerate(observations):
        try:
            cavs.append(estimate_opening_cavity(o.frame, o.cmask, cfg.cavity, cfg.opening))
        except cp.CavityError as e:
            notes.append(f"look {i + 1}: {e}")
    if not cavs:
        raise PerceptionError("no look gave a valid cavity: " + "; ".join(notes))
    cams = np.array([o.frame.cam_pos for o in observations])
    if len(cams) > 1 and np.ptp(cams, axis=0).max() < 0.01:
        notes.append("all looks are from one viewpoint: their agreement checks depth noise, not the calibration")
    return _merge_looks(cavs, cfg.cavity, min_looks, notes)


def _merge_looks(cavs, cavity_cfg, min_looks, notes):
    need = cavity_cfg.min_looks if min_looks is None else min_looks
    mcfg = copy.copy(cavity_cfg)
    mcfg.min_looks = need
    try:
        cav, inl = cp.merge_cavities(cavs, mcfg)
    except cp.CavityError as e:
        raise PerceptionError(f"looks disagree: {e}" + (f" ({'; '.join(notes)})" if notes else "")) from None
    if len(cavs) == 1:
        notes.append("single look: the floor/cavity is NOT cross-checked against another frame")
    return cav, cavs, inl, notes


# ---------------------------------------------------------------------------
# 4. target
# ---------------------------------------------------------------------------
@dataclass
class PlacementResult:
    frame: str                       # every point below is in this frame, metres
    floor_contact: np.ndarray        # support surface under the container centre
    container_center: np.ndarray     # where the container's centre goes  (Part 1 --target container)
    tool_center: np.ndarray          # where the tool centre goes, or None (no measured grasp_offset)
    yaw_deg: float                   # Part 1 --yaw-deg: heading of the insertion axis
    roll_deg: float                  # Part 1 --roll-deg
    grasp_offset: tuple
    floor_z: float                   # floor plane at the cavity centre
    floor_normal: np.ndarray
    floor_tilt_deg: float
    floor_rms_m: float
    support_z: float                 # floor or turntable under the footprint
    support_kind: str
    insertion_axis: np.ndarray       # f, horizontal, into the microwave
    usable_floor: dict               # rectangle (f, l coordinates and its 4 corners at support height)
    footprint: np.ndarray            # container's 4 bottom corners
    cavity: dict
    checks: list
    warnings: list

    def to_dict(self):
        out = {}
        for k, v in asdict(self).items():
            out[k] = v.tolist() if isinstance(v, np.ndarray) else v
        out["usable_floor"] = {k: (np.asarray(v).tolist() if isinstance(v, np.ndarray) else v)
                               for k, v in self.usable_floor.items()}
        return out

    def part1_config(self):
        """A place_move.py / xyz_move.py --config: the container centre, approach heading and roll."""
        d = {"xyz": [round(float(v), 4) for v in self.container_center], "target": "container",
             "yaw_deg": round(self.yaw_deg, 2), "roll_deg": self.roll_deg}
        if self.grasp_offset is not None:
            d["grasp_offset"] = [float(v) for v in self.grasp_offset]
        return d


SUPPORT_MIN_FRAC = 0.2       # a raised surface must hold this share of the footprint's points to be the support
SUPPORT_BIN_M = 0.004


def support_surface(cav, points, center_f, center_l, half_f, half_l, band):
    """What the container would stand on, as a height above the floor plane under the footprint: the highest
    level within `band` holding >= SUPPORT_MIN_FRAC of the footprint's points (a turntable covers most of it;
    a rim or a few noisy points do not). 0 (the floor plane) if the footprint has no points."""
    pts = np.asarray(points, float)
    inside = (np.abs(pts @ cav.f - center_f) <= half_f) & (np.abs(pts @ cav.l - center_l) <= half_l)
    col = pts[inside]
    out = {"height": 0.0, "n": len(col), "n_near": 0, "n_support": 0, "n_above": 0, "max_above": 0.0}
    if len(col) == 0:
        return out
    a = col[:, 2] - np.array([cav.floor_z_at(p[:2]) for p in col])
    a = a[(a >= -band) & (a <= band)]
    out["n_near"] = len(a)         # n counts the whole column, e.g. a ceiling seen from below
    best = None
    for lev in np.arange(-SUPPORT_BIN_M * 2, band + 1e-9, SUPPORT_BIN_M / 2):
        on = np.abs(a - lev) <= SUPPORT_BIN_M
        if on.sum() >= SUPPORT_MIN_FRAC * len(col):
            best = float(np.median(a[on]))
    if best is None:
        return out
    if abs(best) < SUPPORT_BIN_M:
        best = 0.0        # the floor itself: its fitted plane, not the noise around it
    above = a > best + 2 * SUPPORT_BIN_M
    out.update(height=best, n_support=int((np.abs(a - best) <= SUPPORT_BIN_M).sum()), n_above=int(above.sum()),
               max_above=float(a[above].max() - best) if above.any() else 0.0)
    return out


def placement_target(cav, cloud_points, spec: ContainerSpec, tcfg: TargetConfig, cavity_cfg: CavityConfig,
                     need_seen_support=False, frame=BASE_FRAME):
    """The container centre on the usable floor, and the three target points. Raises PerceptionError.
    need_seen_support: the floor came from the shell (opening_cavity), so an unseen footprint may hide a turntable."""
    import xyz_move as xm          # Part 1: its orientation convention is THE convention
    L, W, H = spec.length_m, spec.width_m, spec.height_m
    checks, warnings = [], []
    f, l = cav.f, cav.l
    tilt = cav.floor_tilt_deg()
    checks.append(f"floor z {cav.floor_z:.3f} m, tilt {tilt:.1f} deg (max {cavity_cfg.floor_max_tilt_deg}), "
                  f"rms {cav.floor_rms * 1000:.1f} mm, {len(cav.floor_points)} floor points")

    # usable floor: inside the walls by the clearances
    u = {"front": cav.front + tcfg.front_inside_m, "back": cav.back - tcfg.back_clearance_m,
         "right": cav.right + tcfg.side_clearance_m, "left": cav.left - tcfg.side_clearance_m}
    room_f, room_l = (u["back"] - u["front"]) - L, (u["left"] - u["right"]) - W
    checks.append(f"usable floor {(u['back'] - u['front']) * 100:.1f} x {(u['left'] - u['right']) * 100:.1f} cm "
                  f"(cavity {cav.depth * 100:.1f} x {cav.width * 100:.1f}); container {L * 100:.1f} x {W * 100:.1f} cm "
                  f"-> spare {room_f * 100:+.1f} along / {room_l * 100:+.1f} across")
    if room_f < 0:
        raise PerceptionError(f"container ({L * 100:.1f} cm long) does not fit the cavity depth with "
                              f"{tcfg.front_inside_m * 100:.0f} cm inside the opening and {tcfg.back_clearance_m * 100:.0f} "
                              f"cm from the back wall: {checks[-1]}")
    if room_l < 0:
        raise PerceptionError(f"container ({W * 100:.1f} cm wide) does not fit between the side walls with "
                              f"{tcfg.side_clearance_m * 100:.1f} cm each side: {checks[-1]}")
    c_f, c_l = 0.5 * (u["front"] + u["back"]), 0.5 * (u["left"] + u["right"])

    try:     # refuses if something taller than support_band stands where the container goes
        cp.support_height(cav, cloud_points, c_f, c_l, L / 2 + 0.01, W / 2 + 0.01, cavity_cfg.support_band_m)
    except cp.CavityError as e:
        raise PerceptionError(str(e)) from None
    z_floor_c = cav.floor_z_at(cav.point(c_f, c_l, 0.0)[:2])
    sup = support_surface(cav, cloud_points, c_f, c_l, L / 2 + 0.01, W / 2 + 0.01, cavity_cfg.support_band_m)
    support = z_floor_c + sup["height"]
    if sup["n_near"] < 30 and need_seen_support:
        if tcfg.blind_support_height_m is None:
            raise PerceptionError(f"only {sup['n_near']} depth points at the container footprint, so a turntable would "
                                  "not be seen: set target.blind_support_height_m (measured, 0 = none)")
        support += tcfg.blind_support_height_m
        warnings.append(f"container footprint not seen ({sup['n_near']} depth points): support is the shell floor + the "
                        f"configured {tcfg.blind_support_height_m * 1000:.0f} mm, and nothing checks the footprint is empty")
    kind = "floor" if support - z_floor_c < 0.003 else "raised support (turntable?)"
    checks.append(f"support under the footprint: {kind} at z {support:.3f} ({sup['height'] * 1000:+.0f} mm vs the floor "
                  f"plane; {sup['n_support']} of {sup['n']} footprint points on it)")
    if sup["n_near"] < 30 and not need_seen_support:
        warnings.append(f"only {sup['n']} depth points under the container footprint: the support height is the "
                        "floor plane extrapolated, not measured there")
    if sup["n_above"]:
        warnings.append(f"{sup['n_above']} footprint points stand up to {sup['max_above'] * 1000:.0f} mm above the support "
                        f"but cover < {SUPPORT_MIN_FRAC * 100:.0f}% of it (a rim, edge or depth noise) -- not used as support")
    floor_contact = cav.point(c_f, c_l, support)

    # held level (HOLD_LEVEL), the bottom is flat: lift it so the lowest corner keeps place_gap over a tilted support
    corners = np.array([cav.point(c_f + sf * L / 2, c_l + sl * W / 2, 0.0) for sf, sl in ((-1, -1), (-1, 1), (1, 1), (1, -1))])
    rise = max(0.0, max(cav.floor_z_at(c[:2]) - z_floor_c for c in corners))
    bottom = support + tcfg.place_gap_m + rise
    corners[:, 2] = bottom
    checks.append(f"container bottom at z {bottom:.3f}: {tcfg.place_gap_m * 1000:.0f} mm gap + {rise * 1000:.1f} mm for "
                  "the floor tilt under a level container")
    top = bottom + H
    checks.append(f"container top {top:.3f} vs cavity top {cav.top:.3f} (need {tcfg.top_clearance_m * 100:.0f} cm)")
    if top > cav.top - tcfg.top_clearance_m:
        raise PerceptionError(f"container too tall for the cavity: {checks[-1]}")
    center = floor_contact.copy()
    center[2] = bottom + H / 2

    yaw = float(np.degrees(np.arctan2(f[1], f[0])))
    rot = xm.target_orientation(yaw, tcfg.roll_deg)
    tool = None
    if spec.grasp_offset is None:
        warnings.append("container.grasp_offset not measured: no tool-centre target (Part 1 needs it for "
                        "--target container)")
    else:
        tool = center - rot.apply(np.asarray(spec.grasp_offset, float))
        reach = float(np.linalg.norm(tool))
        checks.append(f"tool target reach {reach:.3f} m, z {tool[2]:.3f}")
        if spec.finger_below_tool_m is None:
            warnings.append("container.finger_below_tool_m not measured: finger clearance over the floor is unchecked")
        else:
            finger = tool[2] - spec.finger_below_tool_m
            checks.append(f"lowest finger z {finger:.3f}, support {support:.3f}")
            if finger < support:
                raise PerceptionError(f"the lower finger would be {(support - finger) * 1000:.0f} mm into the support")
        if reach > tcfg.max_reach_m or not tcfg.z_range_m[0] <= tool[2] <= tcfg.z_range_m[1]:
            warnings.append(f"Part 1 will refuse this tool target: reach {reach:.3f} m (max {tcfg.max_reach_m}) / "
                            f"z {tool[2]:.3f} (band {tcfg.z_range_m}) -- bring the arm/microwave closer")
    nominal = list(spec.nominal or ())
    if nominal:
        warnings.append(f"container {', '.join(nominal)} are NOMINAL (not measured): the fit check uses guesses")

    ucorners = np.array([cav.point(a, b, support) for a, b in
                         ((u["front"], u["right"]), (u["front"], u["left"]), (u["back"], u["left"]), (u["back"], u["right"]))])
    return PlacementResult(
        frame=frame, floor_contact=floor_contact, container_center=center, tool_center=tool,
        yaw_deg=yaw, roll_deg=float(tcfg.roll_deg), grasp_offset=spec.grasp_offset,
        floor_z=float(cav.floor_z), floor_normal=np.asarray(cav.floor_normal, float), floor_tilt_deg=tilt,
        floor_rms_m=float(cav.floor_rms), support_z=float(support), support_kind=kind, insertion_axis=f,
        usable_floor={**u, "corners": ucorners}, footprint=corners,
        cavity={"front": cav.front, "back": cav.back, "left": cav.left, "right": cav.right, "floor_z": cav.floor_z,
                "top": cav.top, "depth": cav.depth, "width": cav.width, "height": cav.height,
                "f": cav.f.tolist(), "l": cav.l.tolist(), "sources": cav.sources},
        checks=checks, warnings=warnings)


# ---------------------------------------------------------------------------
# the whole pipeline on given frames
# ---------------------------------------------------------------------------
@dataclass
class Observation:
    frame: object
    cmask: object
    cloud: CavityCloud = None
    error: str = ""


def run_pipeline(frames, segmenter, cfg: PerceptionConfig, min_looks=None, masks=None, log=print):
    """frames -> (PlacementResult, observations, Cavity). `masks` (optional, one per frame) skips SAM 3
    (a saved CavityMask). Raises PerceptionError with every per-frame reason."""
    obs, problems = [], []
    opening = cfg.cavity_method == "opening"
    for i, fr in enumerate(frames):
        try:
            cm = masks[i] if masks is not None else segmenter.segment(fr)
            o = Observation(fr, cm)
            if opening and cm.microwave is None and segmenter is not None:
                segmenter.add_context(cm, fr)
            try:
                o.cloud = cavity_cloud(fr, cm.mask, cfg.cloud)
            except PerceptionError as e:
                if not opening:                      # opening needs no depth beyond the back wall's
                    raise
                o.cloud = CavityCloud(np.empty((0, 3)), int(cm.mask.sum()), 0)
                cm.notes.append(f"no footprint depth: {e}")
            log(f"frame {i + 1}: SAM 3 '{cm.prompt}' score {cm.score:.2f}, mask {int(cm.sam_mask.sum())} px "
                f"(upright turns {cm.k_upright}), depth valid {o.cloud.valid_frac * 100:.0f}%, "
                f"{len(o.cloud.points)} cavity points" + "".join(f"; {n}" for n in cm.notes))
        except Exception as e:  # noqa: BLE001 -- segmentation/cloud refusals are reported per frame
            if not isinstance(e, (PerceptionError, RuntimeError, ValueError)):
                raise
            o = Observation(fr, None, None, str(e))
            problems.append(f"frame {i + 1}: {e}")
            log(f"frame {i + 1}: REJECTED -- {e}")
        obs.append(o)
    good = [o for o in obs if o.cloud is not None]
    cav = None
    try:
        if not good:
            raise PerceptionError("no usable frame: " + "; ".join(problems))
        if opening:
            cav, _, _, notes = estimate_opening(good, cfg, min_looks)
        else:
            cav, _, _, notes = estimate_floor([o.cloud for o in good], [o.frame.cam_pos for o in good], cfg.cavity,
                                              min_looks)
        for n in notes:
            log(f"  {n}")
        cloud = np.vstack([o.cloud.points for o in good])
        res = placement_target(cav, cloud, cfg.container, cfg.target, cfg.cavity, need_seen_support=opening,
                               frame=good[0].frame.base_frame)
    except PerceptionError as e:
        e.observations, e.cavity = obs, cav          # for the debug images of a refusal
        raise
    res.warnings = notes + problems + placement_warnings(cfg) + res.warnings
    return res, obs, cav


def placement_warnings(cfg):
    out = []
    if cfg.cloud.depth_corr_m == 0.0:
        out.append("cloud.depth_corr_m = 0: depth is uncorrected for this camera (no touched-floor check yet)")
    return out
