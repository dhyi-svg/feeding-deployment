"""The placement task end to end: perceive -> plan -> execute (-> release + retract).

Sequence (every leg planned and checked before the first motion):
  1. perceive   YOLO microwave box, >= 3 consistent detections -> ROI cloud from aligned depth
                -> arm_base_link (tf2) -> cavity (walls, fitted floor) from >= 3 agreeing looks
  2. plan       target from cavity + floor + container + margins; legs in the sim:
                  align + to pre-insert   current -> pre-insert, hand turned to aim the box
                                          into the cavity and levelled (impedance: J6 pulled
                                          to the compliant model's value on the way)
                  insert                  pre-insert -> above the spot, orientation fixed
                  lower                   above -> contact (impedance) / release_gap above (position)
                checked: IK pos+orientation, joint limits/guards, wrap, jump, reach, box level,
                clearance of links + held box to the cavity walls and the scene voxels
  3. execute    the legs as JointCommand steps (convergence-checked), the lowering by
                impedance (`impedance_lowering`) or the planned position steps
  4. release    (re-planned from where the hand actually is) open the gripper, lift 5 mm,
                back straight out to the pre-insert distance, then to park if that leg passes

The camera, arm and detector are passed in, so the same code runs on the robot (ROS frames,
ArmInterfaceClient, YOLO) and offline (replayed/synthetic frames, a sim-backed fake arm, a
fake detector) -- see tests/test_microwave_placement_*.py.
"""
from __future__ import annotations

import copy
import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

import cavity_perception as cp
import point_cloud as pcm
from impedance_lowering import ImpedanceRefused, j6_check, lower_with_impedance
from microwave_common import FINGERTIP_PAST_TOOL, execute_joint_plan, short_delta, wrap_joints
from microwave_detector import wait_for_stable
from placement_planner import PlacementSim, PlanError, park_target

from feeding_deployment.control.robot_controller.command_interface import OpenGripperCommand

PLACE_RECORD = Path.home() / ".microwave_place.json"
LOG_ROOT = Path.home() / "microwave_place_logs"
GRIPPER_HOLDING_MIN = 0.2      # gripper_pos below this = open = nothing held
GRIPPER_OPEN_MAX = 0.15        # after OpenGripperCommand the reading must drop below this
UNMOVED_TOL_DEG = 2.0
PLAN_MAX_AGE_S = 120.0
STAGES = ("pre-insert", "insert", "lower")


class PlacementRefused(RuntimeError):
    """A check failed; nothing (more) is commanded. str(e) says why."""


@dataclass
class Frame:
    bgr: np.ndarray
    depth_mm: np.ndarray
    K: np.ndarray            # 9, row-major
    base_T_cam: np.ndarray   # 4x4 arm_base_link <- camera_color_optical_frame
    stamp: float

    @property
    def cam_pos(self):
        return self.base_T_cam[:3, 3]

    def save(self, path):
        np.savez_compressed(path, bgr=self.bgr, depth_mm=self.depth_mm, K=self.K, base_T_cam=self.base_T_cam,
                            stamp=self.stamp)

    @staticmethod
    def load(path):
        d = np.load(path)
        return Frame(d["bgr"], d["depth_mm"], d["K"], d["base_T_cam"], float(d["stamp"]))


class RosFrameSource:
    """Latest RGB-D frame from realsense2_camera + arm_base_link <- camera from tf2."""

    def __init__(self, rs, tf, max_age_s=1.0):
        self.rs, self.tf, self.max_age_s = rs, tf, max_age_s

    def get(self):
        d = self.rs.get_camera_data()
        bgr, depth, cam = d["rgb_image"], d["depth_image"], d["camera_info"]
        if bgr is None or depth is None or cam is None:
            raise PlacementRefused("no camera frame (is realsense2_camera up with align_depth.enable:=true?)")
        from feeding_deployment.ros2.compat import stamp_to_sec_nanosec
        sec, nsec = stamp_to_sec_nanosec(cam.header.stamp)
        stamp = sec + nsec * 1e-9
        if time.time() - stamp > self.max_age_s:
            raise PlacementRefused(f"camera frame is {time.time() - stamp:.1f} s old -- the stream has stalled")
        tr = self.tf.get_frame_to_frame_transform(cam)
        if tr is None:
            raise PlacementRefused("no arm_base_link <- camera_color_optical_frame transform (calibration_tf up?)")
        return Frame(bgr, np.asarray(depth, np.float32), np.asarray(cam.K, float).reshape(-1),
                     np.asarray(self.tf.make_homogeneous_transform(tr), float), stamp)


class ReplayFrameSource:
    """Frames saved by an earlier run (`frame_*.npz` in a log dir), cycled."""

    def __init__(self, paths):
        self.frames = [Frame.load(p) for p in paths]
        if not self.frames:
            raise PlacementRefused("no frames to replay")
        self.i = 0

    def get(self):
        f = self.frames[self.i % len(self.frames)]
        self.i += 1
        return Frame(f.bgr, f.depth_mm, f.K, f.base_T_cam, time.time())


@dataclass
class Perception:
    cavity: cp.Cavity
    roi: tuple
    detections: list
    looks: list                    # per-look Cavity
    roi_cloud: np.ndarray          # last look's filtered ROI cloud (arm_base_link)
    scene_cloud: np.ndarray        # last look's full-frame cloud minus the hand + held box
    frame: Frame
    log_dir: Path
    timings: dict = field(default_factory=dict)


def held_box_obb(tool_pos, tool_quat, ccfg, up_local=None):
    """Held box (centre, axes, half) for the current tool pose; up_local default = tool dir now up."""
    Rm = R.from_quat(tool_quat).as_matrix()
    if up_local is None:
        up_local = Rm.T @ np.array([0.0, 0.0, 1.0])
        up_local[2] = 0.0
        up_local /= max(np.linalg.norm(up_local), 1e-9)
    a, up = np.array([0.0, 0.0, 1.0]), np.asarray(up_local, float)
    lat = np.cross(a, up)
    length = ccfg.far_past_tool - ccfg.near_past_tool
    c_local = a * (ccfg.near_past_tool + length / 2) + lat * ccfg.lateral_offset + up * (-max(ccfg.drop, 0) + ccfg.height / 2)
    return (np.asarray(tool_pos) + Rm @ c_local, Rm @ np.column_stack([a, lat, up]),
            np.array([length / 2, ccfg.width / 2, ccfg.height / 2]))


def scene_points(frame, cfg, tool_pose=None, box_obb=None, roi=None, stride=None):
    """Frame -> filtered points in arm_base_link, the hand and held box removed."""
    pts, _, n = pcm.deproject(frame.depth_mm, frame.K, roi=roi, stride=stride or cfg.cloud.stride_px,
                              depth_range=(cfg.cloud.depth_min_m, cfg.cloud.depth_max_m),
                              depth_corr_m=cfg.cloud.depth_corr_m)
    pts = pcm.transform_points(frame.base_T_cam, pts)
    if tool_pose is not None:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R.from_quat(tool_pose[1]).as_matrix(), tool_pose[0]
        pts = pcm.remove_self_points(pts, T, cfg.planner.self_filter_box)
    if box_obb is not None:
        pts = pcm.remove_obb(pts, *box_obb, inflate=0.02)
    return pts, n


def draw_overlay(frame, det_box=None, roi=None, cavity=None, target=None, msg="", ok=True, dets=()):
    vis = frame.bgr.copy()
    for d in dets:
        x1, y1, x2, y2 = map(int, d.box)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (180, 180, 180), 1)
    if roi is not None:
        x1, y1, x2, y2 = map(int, roi)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 160, 0), 2)
    T_cb = np.linalg.inv(frame.base_T_cam)

    def px(points):
        return pcm.project(pcm.transform_points(T_cb, points), frame.K)

    if cavity is not None:
        fp = cavity.floor_points[:: max(1, len(cavity.floor_points) // 1500)]
        for u, v in px(fp):
            if np.isfinite(u):
                cv2.circle(vis, (int(u), int(v)), 1, (0, 200, 0), -1)
        c = cavity.corners()
        edges = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]
        uv = px(c)
        for a, b in edges:
            if np.all(np.isfinite(uv[[a, b]])):
                cv2.line(vis, tuple(map(int, uv[a])), tuple(map(int, uv[b])), (0, 255, 255), 1)
    if target is not None:
        for p_, col in ((target.box_center, (0, 0, 255)), (target.contact, (255, 0, 255)), (target.pre, (255, 255, 0))):
            u, v = px(np.asarray(p_)[None])[0]
            if np.isfinite(u):
                cv2.circle(vis, (int(u), int(v)), 6, col, -1)
    cv2.putText(vis, msg[:95], (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if ok else (0, 0, 255), 2)
    return vis


def perceive(source, detector, cfg, tool_pose=None, log=print, log_dir=None, clock=time.time, sleep=time.sleep):
    """Stable detection -> >= min_looks agreeing cavity looks. Raises PlacementRefused."""
    log_dir = Path(log_dir) if log_dir else LOG_ROOT / time.strftime("%Y%m%d_%H%M%S")
    log_dir.mkdir(parents=True, exist_ok=True)
    t0 = clock()
    box_obb = None
    if tool_pose is not None and cfg.container.drop > 0:
        box_obb = held_box_obb(tool_pose[0], tool_pose[1], cfg.container)
    last = {}

    def one_detection():
        fr = source.get()
        last["frame"] = fr
        return detector.detect(fr.bgr, fr.base_T_cam, fr.stamp)

    try:
        roi, dets = wait_for_stable(one_detection, cfg.stabilizer, log=log, sleep=sleep, clock=clock)
    except TimeoutError as e:
        if "frame" in last:
            cv2.imwrite(str(log_dir / "no_detection.png"), draw_overlay(last["frame"], msg=str(e)[:95], ok=False))
        raise PlacementRefused(f"{e} -- overlays in {log_dir}") from None
    t_det = clock() - t0
    m = cfg.cloud.roi_margin_px
    roi = (roi[0] - m, roi[1] - m, roi[2] + m, roi[3] + m)

    looks, errors, frame, roi_cloud, merged, inl = [], [], None, None, None, None
    for i in range(1, cfg.cavity.max_looks + 1):
        if clock() - t0 > cfg.stabilizer.timeout_s * 2:
            errors.append("perception timeout")
            break
        fr = source.get()
        pts, n = scene_points(fr, cfg, tool_pose, box_obb, roi=roi)
        if n == 0 or len(pts) < cfg.cloud.min_valid_depth_frac * n:
            errors.append(f"look {i}: only {len(pts)}/{n} ROI pixels have depth")
            log(f"  {errors[-1]}")
            continue
        cloud = pcm.filter_cloud(pts, cfg.cloud)
        if len(cloud) < cfg.cloud.min_points:
            errors.append(f"look {i}: {len(cloud)} points after filtering (need {cfg.cloud.min_points})")
            log(f"  {errors[-1]}")
            continue
        fr.save(log_dir / f"frame_{i}.npz")
        try:
            cav = cp.estimate_cavity(cloud, fr.cam_pos, cfg.cavity, seed=i)
        except cp.CavityError as e:
            errors.append(f"look {i}: {e}")
            log(f"  {errors[-1]}")
            cv2.imwrite(str(log_dir / f"look_{i}.png"), draw_overlay(fr, roi=roi, msg=str(e), ok=False, dets=dets))
            continue
        looks.append(cav)
        frame, roi_cloud = fr, cloud
        log(f"  look {i}: {cav.summary()}")
        cv2.imwrite(str(log_dir / f"look_{i}.png"), draw_overlay(fr, roi=roi, cavity=cav, msg=f"look {i} ok", dets=dets))
        if len(looks) >= cfg.cavity.min_looks:
            try:
                merged, inl = cp.merge_cavities(looks, cfg.cavity)
                break
            except cp.CavityError as e:
                errors.append(str(e))
                log(f"  {e}")
    if merged is None:
        raise PlacementRefused(f"no consistent cavity from {len(looks)} usable looks: "
                               + "; ".join(errors[-3:]) + f" -- frames/overlays in {log_dir}")
    scene, _ = scene_points(frame, cfg, tool_pose, box_obb, stride=3)
    log(f"perceived in {clock() - t0:.1f} s ({len(inl)} agreeing looks): {merged.summary()}")
    (log_dir / "cavity.json").write_text(json.dumps(cavity_to_dict(merged), indent=1))
    return Perception(merged, roi, dets, looks, roi_cloud, scene, frame, log_dir,
                      {"detect_s": t_det, "total_s": clock() - t0})


def cavity_to_dict(c):
    return {"f": c.f.tolist(), "l": c.l.tolist(), "front": c.front, "back": c.back, "left": c.left, "right": c.right,
            "floor_z": c.floor_z, "top": c.top, "floor_normal": c.floor_normal.tolist(), "floor_rms": c.floor_rms,
            "floor_tilt_deg": c.floor_tilt_deg(), "sources": c.sources,
            "floor_points": c.floor_points[:: max(1, len(c.floor_points) // 2000)].tolist()}


def cavity_from_dict(d):
    return cp.Cavity(f=np.asarray(d["f"]), l=np.asarray(d["l"]), front=d["front"], back=d["back"], left=d["left"],
                     right=d["right"], floor_z=d["floor_z"], top=d["top"], floor_normal=np.asarray(d["floor_normal"]),
                     floor_rms=d["floor_rms"], floor_points=np.asarray(d["floor_points"]), sources=d["sources"])


# ---------------------------------------------------------------------------
# planning
# ---------------------------------------------------------------------------
def obstacle_voxels(scene_cloud, cavity, path_points, pcfg):
    """Scene points that are not the fitted cavity walls, near the path, as voxel centres."""
    pts = np.asarray(scene_cloud, float)
    if len(pts) == 0:
        return pts
    band = pcfg.wall_band_m
    pf, pl, pz = pts @ cavity.f, pts @ cavity.l, pts[:, 2]
    in_f = (pf > cavity.front - band) & (pf < cavity.back + band)
    in_l = (pl > cavity.right - band) & (pl < cavity.left + band)
    in_z = (pz > cavity.floor_z - band) & (pz < cavity.top + band)
    on_wall = in_f & in_l & in_z & (
        (np.abs(pz - cavity.floor_z) < band) | (np.abs(pz - cavity.top) < band) | (np.abs(pf - cavity.back) < band)
        | (np.abs(pl - cavity.left) < band) | (np.abs(pl - cavity.right) < band))
    # below the floor / beyond the walls inside the shell footprint is the shell itself: also modelled
    behind = in_l & (pf > cavity.back - band) & (pz > cavity.floor_z - 0.05) & (pz < cavity.top + 0.05)
    pts = pts[~on_wall & ~behind]
    vox = pcm.voxel_downsample(pts, pcfg.obstacle_voxel_m)
    if len(vox) == 0:
        return vox
    path = np.asarray(path_points, float)
    d = np.full(len(vox), np.inf)
    for a, b in zip(path[:-1], path[1:]):
        ab = b - a
        t = np.clip(((vox - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
        d = np.minimum(d, np.linalg.norm(vox - (a + t[:, None] * ab), axis=1))
    keep = d < pcfg.obstacle_corridor_m
    vox, d = vox[keep], d[keep]
    if len(vox) > pcfg.max_obstacle_voxels:
        vox = vox[np.argsort(d)[: pcfg.max_obstacle_voxels]]
    return vox


@dataclass
class PlacementPlan:
    time: float
    q0: np.ndarray
    target: cp.PlacementTarget
    cavity: cp.Cavity
    legs: list
    lowering: str
    expected_drop: float
    up_local: np.ndarray
    voxels: np.ndarray
    log_dir: Path

    def summary(self):
        lines = [f"lowering: {self.lowering}  (expected drop {self.expected_drop * 100:.1f} cm)",
                 f"pre-insert {np.round(self.target.pre, 3).tolist()}  above {np.round(self.target.above, 3).tolist()}  "
                 f"contact {np.round(self.target.contact, 3).tolist()}"]
        lines += [f"  {c}" for c in self.target.checks]
        for leg in self.legs:
            lines.append(f"  leg {leg.label}: {len(leg.joints)} steps, clearance {leg.worst_clearance[0] * 100:.1f} cm "
                         f"({leg.worst_clearance[1]}), box tilt <= {leg.worst_tilt_deg:.1f} deg, end J6 "
                         f"{np.degrees(leg.joints[-1][5]):.1f}")
        return "\n".join(lines)


def _flip_hint(sim, q, cfg):
    """The same tool pose with the wrist flipped (J5+180, -J6, J7+180), pulled to the model J6."""
    pos, quat = sim.fk(q)
    flip = wrap_joints(np.asarray(q, float) + np.radians([0, 0, 0, 0, 180, 0, 180]))
    flip[5] = -q[5]
    for _ in range(60):
        step = np.clip(cfg.lowering.model_j6_rad - flip[5], -np.radians(3), np.radians(3))
        nq, pe, oe = sim.ik_locked_j6(pos, quat, flip, flip[5] + step)
        if pe > 1e-3 or oe > 0.5 or sim.joint_problem(nq):
            break
        flip = nq
        if abs(flip[5] - cfg.lowering.model_j6_rad) < 1e-3:
            return flip
    return None


def plan_placement(arm, perception, cfg, log=print):
    """All legs in the sim, from the arm's current state. No motion. Raises PlacementRefused."""
    if cfg.container.drop <= 0:
        raise PlacementRefused("container.drop (tool frame -> box bottom, m) is not set -- measure it and pass "
                               "--container-drop or set it in placement_config.yaml")
    st = arm.get_state()
    q0 = wrap_joints(np.asarray(st["position"], float))
    ee = np.asarray(st["ee_pos"], float)
    if float(st["gripper_pos"]) < GRIPPER_HOLDING_MIN:
        raise PlacementRefused(f"gripper is open ({float(st['gripper_pos']):.2f}) -- nothing held")
    cav = perception.cavity
    try:
        target = cp.placement_target(cav, perception.scene_cloud, ee[3:7], cfg.container, cfg.placement, cfg.cavity)
    except cp.CavityError as e:
        raise PlacementRefused(f"placement target: {e}") from None
    for c in target.checks:
        log(f"  {c}")
    a = target.align_info
    log(f"  alignment: turn {a['total_deg']:.1f} deg (yaw {a['yaw_deg']:.1f}, approach tilt {a['approach_tilt_deg']:.1f}, "
        f"box up tilt {a['up_tilt_deg']:.1f})")
    impedance = cfg.lowering.mode == "impedance"
    # impedance: the reference leg ends at contact (the box stops the hand there; press_m is only how far
    # past it the command may run); position: release_gap above it
    lowest = target.contact if impedance else target.place

    sim = PlacementSim(cfg)
    sim.attach_box(cfg.container, a["up_local"])
    sim.add_cavity(cav, target.support_z)
    path = [ee[:3], target.pre, target.above, lowest]
    vox = obstacle_voxels(perception.scene_cloud, cav, path, cfg.planner)
    sim.add_voxels(vox, cfg.planner.obstacle_voxel_m)
    log(f"  obstacles: 5 cavity slabs + {len(vox)} scene voxels ({cfg.planner.obstacle_voxel_m * 100:.0f} cm)")
    bodies = sim.walls + sim.voxels
    no_floor = [w for w in sim.walls if w[1] != "cavity floor"]
    pc = cfg.planner
    j6 = cfg.lowering.model_j6_rad

    start_clear = sim.clearance(q0, bodies)
    log(f"  start clearance {start_clear[0] * 100:.1f} cm ({start_clear[1]})")
    if start_clear[0] < pc.min_clearance_m:
        raise PlacementRefused(f"the arm already starts {start_clear[0] * 100:.1f} cm from an obstacle "
                               f"({start_clear[1]}) -- move it clear and re-plan")
    legs = []
    try:
        leg = sim.plan_leg("align + to pre-insert", ee[:3], ee[3:7], target.pre, target.quat, q0, bodies,
                           pc.min_clearance_m, j6_pull=(j6, 6.0) if impedance else None, level=False, log=log)
        if leg.worst_tilt_deg > cfg.placement.max_tilt_correction_deg + cfg.placement.level_tol_deg:
            raise PlanError(f"align: box tilts {leg.worst_tilt_deg:.1f} deg on the way")
        legs.append(leg)
        q = leg.joints[-1]
        if impedance:
            ok, off = j6_check(q, cfg.lowering)
            if not ok:
                hint = _flip_hint(sim, q0, cfg)
                raise PlanError(
                    f"impedance lowering needs J6 at the compliant model's {np.degrees(j6):.1f} deg, but the arm only "
                    f"reaches J6 {np.degrees(q[5]):.1f} at pre-insert on this wrist branch ({off:.0f} deg off). "
                    + (f"The same hold with the wrist flipped is J = {np.round(np.degrees(hint), 1).tolist()} deg -- "
                       "hold the container in that configuration (flip the wrist BEFORE picking it up), "
                       if hint is not None else "No flipped configuration found either; ")
                    + "or plan with --lowering position.")
        legs.append(sim.plan_leg("insert", target.pre, target.quat, target.above, target.quat, q, bodies,
                                 pc.min_clearance_m, j6_lock=j6 if impedance else None, log=log))
        q = legs[-1].joints[-1]
        low = sim.plan_leg("lower", target.above, target.quat, lowest, target.quat, q, bodies,
                           pc.lower_min_clearance_m, j6_lock=j6 if impedance else None, box_bodies=no_floor, log=log)
        low.mode = "impedance" if impedance else "position"
        legs.append(low)
        # the fingers must open without hitting the walls or the support at the release pose
        rel_clear = sim.clearance(low.joints[-1], sim.walls, closed=False, box=False)
        log(f"  release: open fingers {rel_clear[0] * 100:.1f} cm from the cavity ({rel_clear[1]})")
        if rel_clear[0] < pc.release_min_clearance_m:
            raise PlanError(f"opening the gripper at the release pose: {rel_clear[1]} at {rel_clear[0] * 100:.1f} cm")
    except PlanError as e:
        raise PlacementRefused(f"plan: {e}") from None
    expected_drop = float(target.above[2] - target.contact[2])
    plan = PlacementPlan(time.time(), q0, target, cav, legs, low.mode, expected_drop, a["up_local"], vox,
                         perception.log_dir)
    save_plan(plan, perception)
    return plan


def save_plan(plan, perception):
    rec = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "log_dir": str(plan.log_dir), "lowering": plan.lowering,
           "q0": plan.q0.tolist(), "quat": plan.target.quat.tolist(), "pre": plan.target.pre.tolist(),
           "above": plan.target.above.tolist(), "contact": plan.target.contact.tolist(),
           "place": plan.target.place.tolist(), "support_z": plan.target.support_z, "up_local": plan.up_local.tolist(),
           "cavity": cavity_to_dict(plan.cavity), "checks": plan.target.checks,
           "legs": [{"label": leg.label, "mode": leg.mode, "joints": [list(map(float, q)) for q in leg.joints],
                     "clearance": leg.worst_clearance[0], "tilt": leg.worst_tilt_deg} for leg in plan.legs]}
    (plan.log_dir / "plan.json").write_text(json.dumps(rec, indent=1))
    np.save(plan.log_dir / "scene_cloud.npy", perception.scene_cloud.astype(np.float32))


# ---------------------------------------------------------------------------
# execution
# ---------------------------------------------------------------------------
def check_unmoved(arm, q_plan, tol_deg=UNMOVED_TOL_DEG):
    cur = np.asarray(arm.get_state()["position"], float)
    off = float(np.degrees(np.max(np.abs(short_delta(q_plan, cur)))))
    if off > tol_deg:
        raise PlacementRefused(f"the arm moved {off:.1f} deg (max joint) since the plan -- re-plan")


def execute_placement(arm, plan, cfg, stop_after="lower", log=print):
    """Run the plan up to and including `stop_after` (one of STAGES). The hand stays on the box.
    Returns (ok, message)."""
    if stop_after not in STAGES:
        raise PlacementRefused(f"stop_after must be one of {STAGES}")
    if time.time() - plan.time > PLAN_MAX_AGE_S:
        raise PlacementRefused(f"plan older than {PLAN_MAX_AGE_S:.0f} s -- re-plan")
    check_unmoved(arm, plan.q0)
    if float(arm.get_state()["gripper_pos"]) < GRIPPER_HOLDING_MIN:
        raise PlacementRefused("gripper is open -- nothing held")
    legs = {leg.label: leg for leg in plan.legs}
    for stage, label in (("pre-insert", "align + to pre-insert"), ("insert", "insert")):
        log(f"executing {label} ({len(legs[label].joints)} steps)")
        if not execute_joint_plan(arm, legs[label].joints, label):
            return False, f"{label} stopped early -- still holding the box; check the arm, re-plan."
        if stop_after == stage:
            return True, f"stopped after {stage} as asked (hand still on the box)"
    low = legs["lower"]
    if low.mode == "impedance":
        log("executing lower (impedance)")
        try:
            rep = lower_with_impedance(arm, plan.expected_drop, cfg.lowering, log=log)
        except ImpedanceRefused as e:
            return False, f"impedance lowering refused: {e} -- the box is held above the floor"
        ok, msg = rep["ok"], f"impedance lowering: {rep['reason']}"
    else:
        log(f"executing lower (position, {len(low.joints)} steps)")
        ok = execute_joint_plan(arm, low.joints, "lower")
        msg = "position lowering done" if ok else "position lowering stopped early"
    st = arm.get_state()
    record = json.loads((plan.log_dir / "plan.json").read_text())
    record["placed"] = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "ok": bool(ok), "message": msg,
                        "q": list(map(float, st["position"])), "ee_pos": list(map(float, st["ee_pos"]))}
    PLACE_RECORD.write_text(json.dumps(record, indent=1))
    return ok, (f"{msg}. EE {np.round(st['ee_pos'][:3], 3).tolist()}. Hand still on the box -- check it, then "
                "release.")


# ---------------------------------------------------------------------------
# release + retract (planned from where the hand actually is)
# ---------------------------------------------------------------------------
def plan_release(arm, cfg, record=None, log=print):
    rec = record or (json.loads(PLACE_RECORD.read_text()) if PLACE_RECORD.exists() else None)
    if rec is None or "placed" not in rec:
        raise PlacementRefused(f"no placement record ({PLACE_RECORD}) -- place first")
    st = arm.get_state()
    q0 = wrap_joints(np.asarray(st["position"], float))
    pos, quat = np.asarray(st["ee_pos"][:3], float), np.asarray(st["ee_pos"][3:7], float)
    if float(st["gripper_pos"]) < GRIPPER_HOLDING_MIN:
        raise PlacementRefused("gripper is already open")
    cav = cavity_from_dict(rec["cavity"])
    if np.linalg.norm(pos - np.asarray(rec["placed"]["ee_pos"][:3])) > 0.03:
        raise PlacementRefused("the hand is > 3 cm from where the placement left it -- refusing")
    log_dir = Path(rec["log_dir"])
    scene = np.load(log_dir / "scene_cloud.npy") if (log_dir / "scene_cloud.npy").exists() else np.zeros((0, 3))
    sim = PlacementSim(cfg)
    sim.add_cavity(cav, rec.get("support_z"))
    # the placed box stays where it is: an obstacle for the open hand on the way out -- the part
    # beyond the fingertips (the fingers are wrapped around the rest and slide straight off it)
    ccfg = cfg.container
    near = max(ccfg.near_past_tool, FINGERTIP_PAST_TOOL + 0.01)
    if ccfg.far_past_tool > near + 0.01:
        part = copy.copy(ccfg)
        part.near_past_tool = near
        c, axes, half = held_box_obb(pos, quat, part, np.asarray(rec["up_local"]))
        sim.walls.append(sim._slab(c, axes, half, "placed box"))
    lift = pos + np.array([0, 0, cfg.placement.retract_lift_m])
    approach = R.from_quat(quat).as_matrix()[:, 2]
    a_h = np.array([approach[0], approach[1], 0.0])
    a_h /= np.linalg.norm(a_h)
    back_m = float((lift - np.asarray(rec["pre"])) @ a_h)
    if not 0.0 < back_m < 0.6:
        raise PlacementRefused(f"back-out distance {back_m:.2f} m implausible -- refusing")
    out = lift - a_h * back_m
    park = park_target()
    path = [pos, lift, out] + ([park[0]] if park else [])
    vox = obstacle_voxels(scene, cav, path, cfg.planner)
    sim.add_voxels(vox, cfg.planner.obstacle_voxel_m)
    bodies = sim.walls + sim.voxels
    legs = []
    try:
        for label, p0, p1, clear in (("lift", pos, lift, cfg.planner.release_min_clearance_m),
                                     ("back out", lift, out, cfg.planner.release_min_clearance_m)):
            leg = _plan_open(sim, label, p0, quat, p1, quat, q0 if not legs else legs[-1].joints[-1], bodies, clear, log)
            legs.append(leg)
    except PlanError as e:
        raise PlacementRefused(f"release plan: {e} -- NOT releasing") from None
    park_leg = None
    if park is not None:
        try:
            park_leg = _plan_open(sim, "to park", out, quat, park[0], park[1], legs[-1].joints[-1], bodies,
                                  cfg.planner.min_clearance_m, log)
        except PlanError as e:
            log(f"  to park: {e} -- will stop after backing out (return the arm by hand)")
    return {"time": time.time(), "q0": q0, "legs": legs, "park_leg": park_leg}


def _plan_open(sim, label, p0, q0, p1, q1, seed, bodies, clear, log):
    """plan_leg with the gripper open and no held box; if the free IK fails a gate, again with J6
    held where it is (the way back out of an impedance placement mirrors its J6-locked insertion)."""
    kw = dict(level=False, closed=False, with_box=False, log=log)
    try:
        return sim.plan_leg(label, p0, q0, p1, q1, seed, bodies, clear, **kw)
    except PlanError as e:
        log(f"  {label}: {e} -- retrying with J6 held at {np.degrees(seed[5]):.1f} deg")
        return sim.plan_leg(label, p0, q0, p1, q1, seed, bodies, clear, j6_lock=float(seed[5]), **kw)


def execute_release(arm, plan, log=print):
    check_unmoved(arm, plan["q0"])
    log("releasing: opening the gripper")
    arm.execute_command(OpenGripperCommand())
    t0 = time.time()
    while time.time() - t0 < 3.0 and float(arm.get_state()["gripper_pos"]) > GRIPPER_OPEN_MAX:
        time.sleep(0.1)
    g = float(arm.get_state()["gripper_pos"])
    if g > GRIPPER_OPEN_MAX:
        return False, f"gripper did not open (reads {g:.2f}) -- NOT retracting"
    for leg in plan["legs"] + ([plan["park_leg"]] if plan["park_leg"] is not None else []):
        log(f"executing {leg.label} ({len(leg.joints)} steps)")
        if not execute_joint_plan(arm, leg.joints, leg.label):
            return False, f"{leg.label} stopped early -- gripper open, box released"
    st = arm.get_state()
    return True, (f"released and retracted{'' if plan['park_leg'] is not None else ' (no park leg: return by hand)'}; "
                  f"EE {np.round(st['ee_pos'][:3], 3).tolist()} gripper {float(st['gripper_pos']):.2f}")
