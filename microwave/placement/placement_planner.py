"""Obstacle-aware, level-hold planning for the placement, in the repo's PyBullet sim of the Gen3.

IK is the microwave scripts' existing one (`microwave_common.solve_ik`: PyBullet IK seeded from
the previous step, re-solved on a stall) with every solution validated here: position AND
orientation error, joint limits (URDF + the Kortex guards J2/J4/J6 the open/close scripts use),
the free joints' short-way wrap, and the per-step joint jump. For an impedance lowering the
legs inside the microwave additionally need J6 at the compliant controller's fixed-J6 model
value (`LoweringConfig.model_j6_rad`): those use a damped-least-squares IK with J6 held there
(the 7-DOF arm keeps a full 6-DOF pose with the other six joints).

Collision model (all in arm_base_link, placed in the sim world at the robot base):
  * the cavity's fitted walls / floor / ceiling as thin slabs just outside the measured bounds;
  * every other scene point (full depth frame, minus the hand and the held box, minus the
    points on the fitted walls) near the planned path, as 3 cm voxels in compound bodies --
    this is what models the open door, the microwave's front and the counter;
  * the held box, rigidly attached to the tool frame.
Every arm/gripper link and the box must stay `min_clearance_m` from all of it, checked at
each 2 cm step and twice in between; in the final lowering the box may approach the floor
(that is the goal) and the links keep `lower_min_clearance_m`.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from microwave_common import (ARM, FINGER_CLOSE_SIGN, FINGER_CLOSED_RAD, FINGER_JOINTS, J4_GUARD_DEG, J6_GUARD_DEG,
                              MAX_IK_ERR, MAX_STEP_JUMP_DEG, PARK_FILE, STEP_M, continuous_ok, load_park, make_sim,
                              short_delta, solve_ik, wrap_joints)

Z = np.array([0.0, 0.0, 1.0])


class PlanError(RuntimeError):
    """A planning gate failed (message says which and where)."""


@dataclass
class Leg:
    label: str
    joints: list                     # joint vectors to command in order
    poses: list                      # (pos, quat) target per step (tool frame, arm_base_link)
    worst_clearance: tuple = (9.0, "")
    worst_tilt_deg: float = 0.0
    mode: str = "position"           # 'position' = JointCommand steps; 'impedance' = compliant lowering
    info: dict = field(default_factory=dict)


class PlacementSim:
    """The robot sim + obstacle bodies + the held box. One per plan."""

    def __init__(self, cfg, up_local=None):
        self.cfg = cfg
        self.scene, self.rb = make_sim()
        self.c = self.rb.physics_client_id
        self.robot = self.rb.robot_id
        self.base = np.asarray(self.scene.robot_base_pose.position, float)
        self.ee = self.rb.end_effector_id
        self.movable = [j for j in range(p.getNumJoints(self.robot, physicsClientId=self.c))
                        if p.getJointInfo(self.robot, j, physicsClientId=self.c)[2] != p.JOINT_FIXED]
        self.arm_cols = [self.movable.index(j) for j in ARM]
        self.limits = [p.getJointInfo(self.robot, j, physicsClientId=self.c)[8:10] for j in ARM]
        self.walls, self.voxels, self.box_id = [], [], None
        self.box_local = None
        self.up_local = up_local

    # --- geometry in the sim world ---
    def _slab(self, center, axes, half, name):
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=list(map(float, half)), physicsClientId=self.c)
        q = R.from_matrix(axes).as_quat()
        bid = p.createMultiBody(0, cs, basePosition=(self.base + center).tolist(), baseOrientation=q.tolist(),
                                physicsClientId=self.c)
        return bid, name

    def add_cavity(self, cav, support_z=None):
        """Thin slabs just outside the measured bounds: floor (its top at the measured support --
        a turntable -- if given), ceiling, back, left, right."""
        t = self.cfg.planner.wall_slab_m
        f, l = cav.f, cav.l
        axes = np.column_stack([f, l, Z])
        span_f = (cav.front, cav.back + t)
        span_l = (cav.right - t, cav.left + t)
        span_z = (cav.floor_z - t, cav.top + t)

        def box(f0, f1, l0, l1, z0, z1, name):
            center = f * (f0 + f1) / 2 + l * (l0 + l1) / 2 + Z * (z0 + z1) / 2
            self.walls.append(self._slab(center, axes, [(f1 - f0) / 2, (l1 - l0) / 2, (z1 - z0) / 2], name))

        fz = cav.floor_z if support_z is None else max(cav.floor_z, support_z)
        box(*span_f, *span_l, fz - t, fz, "cavity floor")
        box(*span_f, *span_l, cav.top, cav.top + t, "cavity ceiling")
        box(cav.back, cav.back + t, *span_l, *span_z, "cavity back")
        box(*span_f, cav.left, cav.left + t, *span_z, "cavity left wall")
        box(*span_f, cav.right - t, cav.right, *span_z, "cavity right wall")

    def add_voxels(self, centers, voxel):
        """Scene points as voxel boxes, in compound bodies of <= 200 shapes."""
        centers = np.asarray(centers, float).reshape(-1, 3)
        for i in range(0, len(centers), 200):
            chunk = centers[i:i + 200]
            cs = p.createCollisionShapeArray(
                shapeTypes=[p.GEOM_BOX] * len(chunk), halfExtents=[[voxel / 2] * 3] * len(chunk),
                collisionFramePositions=chunk.tolist(), physicsClientId=self.c)
            bid = p.createMultiBody(0, cs, basePosition=self.base.tolist(), physicsClientId=self.c)
            self.voxels.append((bid, "scene points"))

    def attach_box(self, ccfg, up_local):
        """The held container, rigid in the tool frame (see cavity_perception.container_box_world)."""
        a = np.array([0.0, 0.0, 1.0])
        up = np.asarray(up_local, float)
        lat = np.cross(a, up)
        length = ccfg.far_past_tool - ccfg.near_past_tool
        center = a * (ccfg.near_past_tool + length / 2) + lat * ccfg.lateral_offset + up * (-ccfg.drop + ccfg.height / 2)
        self.box_local = (center, np.column_stack([a, lat, up]))
        half = [length / 2, ccfg.width / 2, ccfg.height / 2]
        cs = p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=self.c)
        self.box_id = p.createMultiBody(0, cs, basePosition=[0, 0, -10], physicsClientId=self.c)
        self.up_local = up

    # --- kinematics ---
    def set_q(self, q, closed=True):
        for j, jj in enumerate(ARM):
            p.resetJointState(self.robot, jj, float(q[j]), physicsClientId=self.c)
        for jj in FINGER_JOINTS:
            p.resetJointState(self.robot, jj, FINGER_CLOSE_SIGN[jj] * FINGER_CLOSED_RAD if closed else 0.0,
                              physicsClientId=self.c)

    def fk(self, q):
        """Tool pose (pos, quat) in arm_base_link for joints q."""
        self.set_q(q)
        ls = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True, physicsClientId=self.c)
        return np.asarray(ls[4]) - self.base, np.asarray(ls[5])

    def pose_error(self, q, pos, quat):
        pq, qq = self.fk(q)
        ang = float(np.degrees((R.from_quat(qq).inv() * R.from_quat(quat)).magnitude()))
        return float(np.linalg.norm(pq - pos)), ang

    def ik(self, pos, quat, seed):
        """Existing PyBullet IK (+ re-solves on a stall). Returns (q, pos err m, ori err deg)."""
        q, err = solve_ik(self.scene, self.rb, pos, quat, seed)
        for _ in range(3):
            if err <= MAX_IK_ERR * 0.25:
                break
            q, err = solve_ik(self.scene, self.rb, pos, quat, q)
        pe, oe = self.pose_error(q, pos, quat)
        return wrap_joints(q), pe, oe

    def ik_locked_j6(self, pos, quat, seed, j6, iters=150):
        """Damped least squares over J1-5,7 with J6 held at `j6`. Returns (q, pos err, ori err deg)."""
        q = wrap_joints(np.asarray(seed, float).copy())
        q[5] = j6
        tgt_R = R.from_quat(quat)
        cols = [k for k in range(7) if k != 5]
        lam = 0.05
        for _ in range(iters):
            pq, qq = self.fk(q)
            e_p = pos - pq
            e_r = (tgt_R * R.from_quat(qq).inv()).as_rotvec()
            if np.linalg.norm(e_p) < 2e-4 and np.linalg.norm(e_r) < np.radians(0.1):
                break
            J = self._jacobian(q)[:, cols]
            e = np.concatenate([e_p, e_r])
            dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
            n = np.max(np.abs(dq))
            if n > 0.15:
                dq *= 0.15 / n
            q[cols] += dq
        q = wrap_joints(q)
        pe, oe = self.pose_error(q, pos, quat)
        return q, pe, oe

    def _jacobian(self, q):
        self.set_q(q)
        states = [p.getJointState(self.robot, j, physicsClientId=self.c)[0] for j in self.movable]
        jt, jr = p.calculateJacobian(self.robot, self.ee, [0.0, 0.0, 0.0], states, [0.0] * len(states),
                                     [0.0] * len(states), physicsClientId=self.c)
        J = np.vstack([np.asarray(jt), np.asarray(jr)])
        return J[:, self.arm_cols]

    # --- gates ---
    def joint_problem(self, q):
        """None if q is inside every limit/guard, else what is violated."""
        deg = np.degrees(q)
        for k, (lo, hi) in enumerate(self.limits):
            if hi > lo and not lo <= q[k] <= hi:
                return f"J{k + 1} {deg[k]:.1f} outside the URDF limit"
        if abs(deg[1]) > self.cfg.planner.j2_guard_deg:
            return f"J2 {deg[1]:.1f} past the {self.cfg.planner.j2_guard_deg} guard"
        if abs(deg[3]) > J4_GUARD_DEG:
            return f"J4 {deg[3]:.1f} past the {J4_GUARD_DEG} guard"
        if abs(deg[5]) > J6_GUARD_DEG:
            return f"J6 {deg[5]:.1f} past the {J6_GUARD_DEG} guard"
        return None

    def clearance(self, q, bodies, closed=True, box=True, box_bodies=None):
        """Smallest distance from robot links (and the held box) to `bodies`: (metres, what)."""
        self.set_q(q, closed)
        best = (9.0, "")
        qd = 0.25
        for bid, name in bodies:
            for pt in p.getClosestPoints(self.robot, bid, qd, physicsClientId=self.c):
                if pt[8] < best[0]:
                    ln = (p.getJointInfo(self.robot, pt[3], physicsClientId=self.c)[12].decode()
                          if pt[3] >= 0 else "base")
                    best = (float(pt[8]), f"{ln} vs {name}")
        if box and self.box_id is not None:
            ls = p.getLinkState(self.robot, self.ee, computeForwardKinematics=True, physicsClientId=self.c)
            tR = R.from_quat(ls[5]).as_matrix()
            c_local, ax_local = self.box_local
            p.resetBasePositionAndOrientation(self.box_id, (np.asarray(ls[4]) + tR @ c_local).tolist(),
                                              R.from_matrix(tR @ ax_local).as_quat().tolist(), physicsClientId=self.c)
            for bid, name in (bodies if box_bodies is None else box_bodies):
                for pt in p.getClosestPoints(self.box_id, bid, qd, physicsClientId=self.c):
                    if pt[8] < best[0]:
                        best = (float(pt[8]), f"held box vs {name}")
        return best

    def box_tilt_deg(self, q):
        """Angle between the held box's up axis and world up."""
        _, qq = self.fk(q)
        up = R.from_quat(qq).as_matrix() @ self.up_local
        return float(np.degrees(np.arccos(np.clip(up @ Z, -1.0, 1.0))))

    # --- legs ---
    def plan_leg(self, label, p0, quat0, p1, quat1, q_start, bodies, min_clear, j6_lock=None, j6_pull=None,
                 level=True, box_bodies=None, closed=True, with_box=True, log=print):
        """Straight line p0 -> p1 (orientation slerped quat0 -> quat1), <= 2 cm / 5 deg steps,
        each IK-solved from the previous step and validated. j6_lock: hold J6 there (DLS IK).
        j6_pull: (target, max deg per step) -- after each step, drift J6 toward target along the
        arm's self-motion (pose kept). Raises PlanError."""
        rots = R.from_quat([quat0, quat1])
        ang = float(np.degrees((rots[0].inv() * rots[1]).magnitude()))
        n = max(1, int(np.ceil(max(np.linalg.norm(np.asarray(p1) - p0) / STEP_M, ang / 5.0))))
        slerp = Slerp([0, 1], rots)
        q = np.asarray(q_start, float)
        joints, poses, worst, worst_tilt = [], [], (9.0, ""), 0.0
        pcfg = self.cfg.planner
        for k in range(1, n + 1):
            s = k / n
            tgt = np.asarray(p0) + (np.asarray(p1) - p0) * s
            tq = slerp([s]).as_quat()[0]
            if j6_lock is not None:
                nq, pe, oe = self.ik_locked_j6(tgt, tq, q, j6_lock)
            else:
                nq, pe, oe = self.ik(tgt, tq, q)
                if j6_pull is not None:
                    nq, pe, oe = self._pull_j6(tgt, tq, nq, *j6_pull)
            jump = float(np.degrees(np.max(np.abs(short_delta(q, nq)))))
            for frac in (1 / 3, 2 / 3, 1.0):
                worst = min(worst, self.clearance(q + short_delta(q, nq) * frac, bodies, closed=closed, box=with_box,
                                                  box_bodies=box_bodies), key=lambda w: w[0])
            tilt = self.box_tilt_deg(nq) if self.up_local is not None else 0.0
            worst_tilt = max(worst_tilt, tilt)
            problem = None
            if pe > MAX_IK_ERR * 0.5:
                problem = f"IK position error {pe * 100:.2f} cm"
            elif oe > pcfg.max_ori_err_deg:
                problem = f"IK orientation error {oe:.1f} deg"
            elif jump > MAX_STEP_JUMP_DEG:
                problem = f"joint jump {jump:.1f} deg"
            elif not continuous_ok(q, nq):
                problem = "a free-spinning joint would wrap > 170 deg"
            elif self.joint_problem(nq):
                problem = self.joint_problem(nq)
            elif worst[0] < min_clear:
                problem = f"clearance {worst[0] * 100:.1f} cm < {min_clear * 100:.1f} ({worst[1]})"
            elif level and tilt > self.cfg.placement.level_tol_deg:
                problem = f"held box tilted {tilt:.1f} deg (max {self.cfg.placement.level_tol_deg})"
            if problem or k in (1, n) or k % 5 == 0:
                log(f"  {label} {k}/{n} -> {np.round(tgt, 3)}  err {pe * 100:.2f}cm/{oe:.1f}deg  jump {jump:.1f}  "
                    f"J4 {np.degrees(nq[3]):.0f} J6 {np.degrees(nq[5]):.0f}  clear {worst[0] * 100:.1f}cm ({worst[1]})  "
                    f"tilt {tilt:.1f}")
            if problem:
                raise PlanError(f"{label}: step {k}/{n} at {np.round(tgt, 3).tolist()}: {problem}")
            joints.append(nq)
            poses.append((tgt, tq))
            q = nq
        log(f"  {label}: {n} steps OK, worst clearance {worst[0] * 100:.1f} cm ({worst[1]}), "
            f"max box tilt {worst_tilt:.1f} deg, orientation change {ang:.0f} deg")
        return Leg(label, joints, poses, worst, worst_tilt)

    def _pull_j6(self, pos, quat, q, target, max_deg):
        """Move J6 up to max_deg toward target by re-solving the pose with J6 held at the new value."""
        delta = float(np.clip(np.degrees(target - q[5]), -max_deg, max_deg))
        if abs(delta) < 0.05:
            return q, *self.pose_error(q, pos, quat)
        for frac in (1.0, 0.5, 0.25):
            nq, pe, oe = self.ik_locked_j6(pos, quat, q, q[5] + np.radians(delta * frac))
            if pe < 1e-3 and oe < 0.5:
                return nq, pe, oe
        return q, *self.pose_error(q, pos, quat)

    def joint_move(self, label, q_from, q_to, bodies, min_clear, max_tilt):
        """Joint-linear move (what Kortex does for a JointCommand), checked every <= 1 deg."""
        q_from = wrap_joints(q_from)
        d = short_delta(q_from, q_to)
        n = max(2, int(np.ceil(np.degrees(np.max(np.abs(d))))))
        worst, worst_tilt = (9.0, ""), 0.0
        for k in range(n + 1):
            q = q_from + d * k / n
            worst = min(worst, self.clearance(q, bodies), key=lambda w: w[0])
            if self.up_local is not None:
                worst_tilt = max(worst_tilt, self.box_tilt_deg(q))
        if worst[0] < min_clear:
            raise PlanError(f"{label}: clearance {worst[0] * 100:.1f} cm ({worst[1]})")
        if worst_tilt > max_tilt:
            raise PlanError(f"{label}: held box tilts {worst_tilt:.1f} deg on the way (max {max_tilt})")
        if not continuous_ok(q_from, q_to):
            raise PlanError(f"{label}: a free-spinning joint would wrap > 170 deg")
        return Leg(label, [wrap_joints(q_to)], [self.fk(q_to)], worst, worst_tilt,
                   info={"joint_move_deg": float(np.degrees(np.max(np.abs(d))))})


def park_target():
    """(ee_pos, quat, joints) of the hand-placed park pose, or None."""
    if not PARK_FILE.exists():
        return None
    rec = json.loads(PARK_FILE.read_text())
    return np.asarray(rec["ee_pos"], float), np.asarray(rec["quat"], float), load_park()
