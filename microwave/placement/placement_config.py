"""Every tunable of the microwave container placement, in one place.

Defaults live here; `placement_config.yaml` (next to this file) or `--config <file>` overrides
any subset, e.g.

    container:
      drop: 0.045
    lowering:
      mode: position

Values marked NOMINAL are not measured on the rig yet -- the README's hardware checklist says
how to measure each one. Pure Python (no ROS), so the tests load it directly.
"""
from __future__ import annotations

import copy
from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG_FILE = HERE / "placement_config.yaml"
REPO_ROOT = HERE.parents[1]


@dataclass
class DetectorConfig:
    """Real-time microwave detector: YOLO (COCO 'microwave', id 68) on the upright colour image."""
    model: str = str(REPO_ROOT / "yolo26s.pt")   # same weights as the open task's handle_detect.py
    class_ids: list = field(default_factory=lambda: [68])          # COCO microwave
    # classes YOLO confuses this red microwave with (bus 5, train 6, suitcase 28, oven 69,
    # refrigerator 72 -- handle_detect.py); used only when no primary class is found
    fallback_class_ids: list = field(default_factory=lambda: [5, 6, 28, 69, 72])
    conf: float = 0.30
    device: str = ""                   # "" = ultralytics default (GPU if free), "cpu" to force CPU
    # the image is rotated by a multiple of 90 deg so world-up is image-up before YOLO runs
    # (COCO models are not rotation invariant); derived from the live TF every frame
    upright_from_tf: bool = True


@dataclass
class StabilizerConfig:
    """Consistent detections required before any geometry is trusted."""
    min_consistent: int = 3            # >= this many detections must agree
    window: int = 8                    # considered: the newest this-many detections
    min_iou: float = 0.6               # a detection agrees with the median box at >= this IoU
    max_center_shift_px: float = 25.0  # ... and centre within this many pixels
    timeout_s: float = 20.0            # give up (refuse) after this long without agreement
    frame_period_s: float = 0.1        # sleep between detector frames
    max_frame_age_s: float = 1.0       # a camera frame older than this is not used


@dataclass
class PointCloudConfig:
    depth_min_m: float = 0.12
    depth_max_m: float = 1.30
    roi_margin_px: int = 12            # the stabilised box is grown by this before sampling depth
    stride_px: int = 2                 # sample every n-th pixel (640x480 -> ~77k candidate points)
    voxel_m: float = 0.008             # voxel downsample
    outlier_k: int = 12                # statistical outlier removal: neighbours ...
    outlier_std: float = 2.0           # ... and std-devs past the mean neighbour distance
    min_points: int = 1500             # fewer valid points after filtering -> refuse the frame
    min_valid_depth_frac: float = 0.25 # of the ROI samples (glossy/black interior drop-outs)
    depth_corr_m: float = 0.0          # along the optical axis; 0 like handle_detect.py (re-measure)


@dataclass
class CavityConfig:
    """Plane fits and plausibility of the open microwave's interior (all measured live)."""
    plane_dist_m: float = 0.006        # RANSAC inlier distance
    sheet_half_m: float = 0.005       # level sheets: points within this of a height-histogram peak
    floor_refine_dist_m: float = 0.004 # floor re-fit band (separates a turntable ~1 cm above the floor)
    plane_iters: int = 300
    max_planes: int = 10
    min_plane_points: int = 250
    horizontal_tol_deg: float = 12.0   # a plane within this of horizontal is a floor/ceiling candidate
    vertical_tol_deg: float = 12.0     # ... of vertical is a wall candidate
    cluster_eps_m: float = 0.03        # plane inliers are split into connected pieces at this gap
    # floor (the support surface)
    floor_max_tilt_deg: float = 6.0    # fitted floor normal vs world up (gravity) -- also catches a wrong camera roll
    floor_max_rms_m: float = 0.006
    floor_min_extent_m: float = 0.10   # visible floor must span this much along both cavity axes
    floor_reaches_back_m: float = 0.05 # floor inliers must come this close to the back wall
    floor_z_range: tuple = (-0.30, 0.80)   # plausible floor height in arm_base_link (m)
    floor_below_camera_m: float = 0.05     # the floor must be this far below the camera
    floor_complex_band_m: float = 0.025  # level sheets this close above the floor (turntable) bound the floor too
    support_band_m: float = 0.05       # points this far above the floor under the container = turntable/support
    # cavity extent
    min_depth_m: float = 0.15
    max_depth_m: float = 0.60
    min_width_m: float = 0.18
    max_width_m: float = 0.70
    min_height_m: float = 0.08
    max_height_m: float = 0.45
    wall_behind_front_m: float = 0.03  # a side-wall plane must sit this far behind the opening
    back_wall_max_view_angle_deg: float = 55.0  # back-wall normal vs the camera's horizontal view direction
    # agreement across looks
    min_looks: int = 3
    max_looks: int = 8
    agree_target_m: float = 0.02
    agree_floor_m: float = 0.01
    agree_axis_deg: float = 4.0


@dataclass
class ContainerConfig:
    """The held OXO box, in the TOOL frame (z = approach). Measure on the held box."""
    drop: float = -1.0                 # tool frame -> container BOTTOM, straight down (m). REQUIRED (< 0 = unset)
    near_past_tool: float = 0.0        # box starts this far past the tool frame along the approach (NOMINAL)
    far_past_tool: float = 0.214       # box ends this far past it: fingertips 0.062 + 6 in (0.152) stick-out (09-28)
    width: float = 0.12                # across the approach, horizontal (NOMINAL -- measure)
    height: float = 0.11               # bottom -> top (NOMINAL; OXO POP 0.112 measured by RAMMP)
    lateral_offset: float = 0.0        # box centre's offset from the tool axis, horizontal (NOMINAL)


@dataclass
class PlacementConfig:
    side_clearance_m: float = 0.025    # box side -> side wall
    back_clearance_m: float = 0.04     # box far end -> back wall
    front_inside_m: float = 0.03       # box near end at least this far inside the opening
    top_clearance_m: float = 0.03      # box top -> ceiling while inserted
    insert_clearance_m: float = 0.04   # box bottom this far above the support surface while inserting
    release_gap_m: float = 0.01        # position-mode lowering ends the box bottom this far above the support
    pre_insert_standoff_m: float = 0.08    # pre-insert: box far end this far in front of the opening
    max_yaw_correction_deg: float = 30.0   # alignment may turn the hand at most this much about vertical
    max_tilt_correction_deg: float = 10.0  # ... and level it by at most this much
    level_tol_deg: float = 2.0         # container up vs world up on every insertion/placement step
    max_reach_m: float = 0.915         # |tool position| from arm_base_link (rchi-cpu-5 grasp gate)
    retract_lift_m: float = 0.005      # after release, lift the open fingers this much before backing out


@dataclass
class PlannerConfig:
    min_clearance_m: float = 0.03      # every arm/gripper link and the held box vs obstacles
    lower_min_clearance_m: float = 0.01   # final lowering: box vs walls (floor contact is the goal)
    release_min_clearance_m: float = 0.005  # opening the fingers at the box, lifting and backing out
    obstacle_voxel_m: float = 0.03
    obstacle_corridor_m: float = 0.40  # only scene points this close to the planned path become obstacles
    max_obstacle_voxels: int = 2500
    wall_slab_m: float = 0.02          # thickness of the modelled cavity walls (placed outside the bounds)
    wall_band_m: float = 0.015         # scene points this close to a fitted wall/floor are that wall (not voxels)
    max_ori_err_deg: float = 2.0       # IK solution's orientation error
    j2_guard_deg: float = 125.0        # Kortex J2 limit +-128.9
    self_filter_box: tuple = (0.07, 0.07, -0.20, 0.09)   # tool-frame |x|,|y| and z range of the gripper's own points


@dataclass
class LoweringConfig:
    """Final lowering. 'impedance' = the arm's task compliant mode (kinova.py / compliant_controller.py);
    'position' = planned joint steps ending release_gap above the floor."""
    mode: str = "impedance"
    # compliant_controller.py's task mode runs a 6-DOF model with J6 FIXED at this angle
    # (hack_gen3_robotiq_2f_85.urdf; kinova.get_state inserts it in compliant mode). Verified
    # offline: model FK matches the full arm within 0.03 mm at this J6 and is 53 cm off at +67.6.
    model_j6_rad: float = -1.18039928
    j6_tol_deg: float = 3.0            # real J6 must be this close to model_j6 to enter compliant mode
    j6_null_gain: float = 0.25         # planner's null-space pull of J6 toward model_j6 per IK step
    speed_mps: float = 0.01            # commanded descent speed
    command_hz: float = 10.0
    press_m: float = 0.01              # command at most this far below the expected contact
    contact_lag_m: float = 0.008       # commanded - measured z beyond this ...
    contact_stall_mps: float = 0.002   # ... with the measured descent slower than this = contact
    contact_stall_s: float = 0.5
    hold_preload_m: float = 0.004      # after contact: hold the command this far below the measured pose
    settle_s: float = 1.0
    max_lateral_dev_m: float = 0.02    # abort if the hand drifts sideways more than this
    max_track_err_m: float = 0.06      # abort before the controller's own 10 cm trip
    timeout_s: float = 20.0


@dataclass
class Config:
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    stabilizer: StabilizerConfig = field(default_factory=StabilizerConfig)
    cloud: PointCloudConfig = field(default_factory=PointCloudConfig)
    cavity: CavityConfig = field(default_factory=CavityConfig)
    container: ContainerConfig = field(default_factory=ContainerConfig)
    placement: PlacementConfig = field(default_factory=PlacementConfig)
    planner: PlannerConfig = field(default_factory=PlannerConfig)
    lowering: LoweringConfig = field(default_factory=LoweringConfig)

    def to_dict(self):
        return asdict(self)


def _merge(obj, overrides, path=""):
    for key, val in (overrides or {}).items():
        names = {f.name for f in fields(obj)}
        if key not in names:
            raise ValueError(f"unknown config key {path}{key!r} (known: {sorted(names)})")
        cur = getattr(obj, key)
        if is_dataclass(cur):
            if not isinstance(val, dict):
                raise ValueError(f"config {path}{key} must be a mapping")
            _merge(cur, val, f"{path}{key}.")
        else:
            if isinstance(cur, tuple):
                val = tuple(val)
            elif isinstance(cur, float) and isinstance(val, int):
                val = float(val)
            setattr(obj, key, val)


def load_config(path=None, overrides=None):
    """Defaults <- the YAML file (DEFAULT_CONFIG_FILE if it exists and path is None) <- overrides dict."""
    cfg = Config()
    path = Path(path) if path else (DEFAULT_CONFIG_FILE if DEFAULT_CONFIG_FILE.exists() else None)
    if path is not None:
        data = yaml.safe_load(Path(path).read_text()) or {}
        _merge(cfg, data)
    _merge(cfg, copy.deepcopy(overrides or {}))
    if cfg.lowering.mode not in ("impedance", "position"):
        raise ValueError(f"lowering.mode must be 'impedance' or 'position', got {cfg.lowering.mode!r}")
    return cfg
