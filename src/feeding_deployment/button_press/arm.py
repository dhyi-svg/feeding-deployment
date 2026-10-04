"""Arm side of the button press: seeded PyBullet IK, gated joint steps, convergence checks.

Every move is "translate the end effector by d" -- same orientation unless a world-frame
rotation ``rot`` is also given (the one-command press turns the wrist to face the panel). It is
planned in a
headless PyBullet copy of the arm, refused if any gate fails (reach, height, IK error,
joint jump), and only then sent to the real arm as a joint command over the arm RPC.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from feeding_deployment.button_press import Abort
from feeding_deployment.button_press.geometry import max_joint_delta_deg
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import CartesianTrajectoryCommand, JointCommand
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

# ---- motion gates (same family as the grasp scripts; tighter where the moves are smaller)
MAX_IK_ERR_M = 0.005
MAX_IK_ROT_ERR_DEG = 1.0     # orientation miss allowed when a move also turns the wrist
SEED_GOOD_M = 0.001          # posture seed is honoured unquestioned below this; see solve_translation
MAX_JOINT_STEP_DEG = 10.0
MAX_REACH_M = 0.91           # existing rig constant -- do not raise without asking
Z_RANGE_M = (0.12, 0.75)       # min 0.25 -> 0.15 (user-approved 2026-09-27, button ~0.22 m above base);
                               # 0.15 -> 0.12 (user-approved 2026-10-03): button now at 0.171, one-command
                               # pre-press tool point at 0.128, deepest press point ~0.125
TRACK_ABORT_M = 0.01
CONVERGE_TOL_DEG = 1.0
# --smooth: straight lines are interpolated at this spacing, each point gated, then sent as ONE
# blended Cartesian trajectory (the fridge --smooth pattern, 2026-09-25).
SMOOTH_SPACING_M = 0.02
# Press strokes use finer spacing so kinova's final-waypoint taper (last 4 points, down to
# 0.04 m/s) applies: a 1.3 cm stroke at 2 cm spacing is ONE point and would arrive at cruise.
SMOOTH_PRESS_SPACING_M = 0.003
ARM = [1, 2, 3, 4, 5, 6, 7]
# Resolved from the package, not the cwd, so this runs from anywhere.
SCENE_CONFIG = str(Path(__file__).resolve().parents[1] / "simulation" / "configs" / "vention.yaml")


def wait_converged(ai, q_cmd, tol_deg=CONVERGE_TOL_DEG, timeout_s=6.0):
    """Block until the arm's joints are within tol_deg of q_cmd and at rest.

    A plain "velocity ~ 0" wait is not enough: Kortex's blocking move returns on
    ACTION_END *or* ACTION_ABORT, so the next command can land while the arm is still
    moving and be rejected (ROBOT_MOVEMENT_IN_PROGRESS) -- that sub-step is silently
    skipped. Seen 9 times in one evening's arm log on this rig (2026-09-20).
    """
    q_cmd = np.asarray(q_cmd, dtype=float)
    deadline = time.time() + timeout_s
    derr = float("inf")
    while time.time() < deadline:
        time.sleep(0.12)
        st = ai.get_state()
        derr = max_joint_delta_deg(st["position"], q_cmd)
        vel = float(np.max(np.abs(np.asarray(st["velocity"], dtype=float))))
        if derr < tol_deg and vel < 1e-3:
            break
    return derr


def wait_still(ai, target=None, timeout_s=20.0, settle_reads=4, tol_m=5e-4, near_m=0.002):
    """Block until the EE position has genuinely stopped; return it.

    After a Cartesian trajectory, kinova.move_cartesian_trajectory can return on a stale
    END/ABORT while the arm is still travelling -- or before it has started (then the
    position is steady at the START). So "steady" alone only counts once the EE is within
    `near_m` of `target`; far from it we keep waiting for 1.5 s of stillness instead
    (a genuinely stopped-short arm), then return. Same idea as _wait_still in
    scripts/real_gen3_ros2_sam3_grasp_fridge.py (2026-09-25).
    """
    deadline = time.time() + timeout_s
    prev, steady = None, 0
    cur = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
    while time.time() < deadline:
        time.sleep(0.1)
        cur = np.asarray(ai.get_state()["ee_pos"][:3], dtype=float)
        steady = steady + 1 if prev is not None and np.linalg.norm(cur - prev) < tol_m else 0
        prev = cur
        near = target is None or np.linalg.norm(cur - np.asarray(target)) < near_m
        if (near and steady >= settle_reads) or steady >= 15:
            break
    return cur


class Arm:
    def __init__(self, execute: bool):
        self.execute = execute
        self.ai = ArmInterfaceClient()
        scene = create_scene_description_from_config(SCENE_CONFIG, "skewer")
        self.sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
        self.rb = self.sim.robot
        self.base_pos = np.asarray(scene.robot_base_pose.position, dtype=float)
        bq = np.asarray(scene.robot_base_pose.orientation, dtype=float)
        # The grasp scripts treat base-frame == world-frame directions (fk() subtracts only
        # the base position). Hold that assumption explicitly instead of inheriting it.
        if abs(abs(bq[3]) - 1.0) > 1e-3:
            raise Abort(f"scene robot_base_pose is rotated ({bq}); this script assumes identity")

    # -- state --------------------------------------------------------------------------------
    def state(self):
        return self.ai.get_state()

    def joints(self):
        return np.asarray(self.state()["position"], dtype=float)

    def ee_pos(self):
        return np.asarray(list(self.state()["ee_pos"])[:3], dtype=float)

    def arm_state_name(self):
        try:
            return self.ai._arm_interface.get_arm_state()["name"]  # noqa: SLF001
        except Exception as e:  # noqa: BLE001
            return f"unknown ({type(e).__name__})"

    # -- kinematics ---------------------------------------------------------------------------
    def _set_sim(self, q):
        for i, jj in enumerate(ARM):
            p.resetJointState(self.rb.robot_id, jj, float(q[i]), physicsClientId=self.rb.physics_client_id)

    def fk(self, q):
        """(world position, world quaternion xyzw) of the sim EE for joint vector q."""
        self._set_sim(q)
        ls = p.getLinkState(self.rb.robot_id, self.rb.end_effector_id, physicsClientId=self.rb.physics_client_id)
        return np.asarray(ls[4], dtype=float), np.asarray(ls[5], dtype=float)

    def solve_translation(self, q_cur, d_base, posture=None, rot=None):
        """IK for 'current EE + d_base', same orientation (or turned by `rot`, see below).

        The target is relative to the sim's own FK of q_cur (not Kinova's ee_pos), so any
        constant sim-vs-Kortex tool-frame offset cancels instead of becoming a jump on
        step 1. The IK is SEEDED FROM `posture` (the run's start joints) rather than from
        q_cur: the Gen3 is redundant, and re-seeding from the current pose every step let
        the solution slide along the self-motion manifold -- on 2026-09-21 J1/J3
        counter-rotated 7 -> 10.6 deg per identical 2 cm step (30 deg total in 6 cm), which
        both tripped the joint-jump gate and shifted the wrench estimate by several N.
        Seeding from a fixed posture keeps every solution near one configuration. The anchor
        is honoured outright while it solves to better than SEED_GOOD_M; beyond that (a stale
        anchor, i.e. the arm has travelled away from it) the q_cur seed is solved too and the
        more accurate of the two wins. Run.reanchor_seed() refreshes the anchor per phase.
        `rot` (3x3, world frame) turns the EE as well: the IK orientation target is
        rot @ (sim orientation of q_cur). Left-multiplying a world-frame delta onto the sim's
        own orientation cancels a constant sim-vs-Kortex tool rotation the same way the
        position delta does. An orientation miss over MAX_IK_ROT_ERR_DEG fails the gates.
        Returns (q, ik_err_m, target_world, jump_deg, gate_failures).
        """
        pos, quat = self.fk(q_cur)
        target = pos + np.asarray(d_base, dtype=float)
        if rot is not None:
            quat = Rotation.from_matrix(np.asarray(rot, dtype=float) @ Rotation.from_quat(quat).as_matrix()).as_quat()
        seeds = [posture, q_cur] if posture is not None else [q_cur]
        q = None
        err = None
        rerr = 0.0
        for seed in seeds:
            self._set_sim(seed)
            sol = p.calculateInverseKinematics(
                self.rb.robot_id, self.rb.end_effector_id, list(target), list(quat),
                maxNumIterations=400, residualThreshold=1e-5, physicsClientId=self.rb.physics_client_id)
            q_try = np.asarray(sol[:7], dtype=float)
            got, got_q = self.fk(q_try)
            err_try = float(np.linalg.norm(got - target))
            # Orientation is only scored when this move turns the wrist; the translation-only
            # path keeps its original behaviour exactly.
            rerr_try = (float(np.degrees((Rotation.from_quat(got_q) * Rotation.from_quat(quat).inv()).magnitude()))
                        if rot is not None else 0.0)
            better = (q is None or (rerr_try <= MAX_IK_ROT_ERR_DEG, -err_try) > (rerr <= MAX_IK_ROT_ERR_DEG, -err))
            if better:
                q, err, rerr = q_try, err_try, rerr_try
            # Take the posture seed the moment it is GOOD, not merely legal. Accepting the
            # first solution under MAX_IK_ERR_M (5 mm) is what broke the 2026-09-21 stage-1
            # servo: a stale anchor returned ~3 mm error, passed the gate, and the more
            # accurate q_cur seed was never tried -- while the corrections being asked for
            # were themselves only 1-4 mm. Below SEED_GOOD_M the anchor is honoured (no
            # null-space drift); above it, both seeds are solved and the closer one wins.
            if err <= SEED_GOOD_M and rerr <= MAX_IK_ROT_ERR_DEG:
                break
        bad = []
        tb = target - self.base_pos
        if np.linalg.norm(tb) > MAX_REACH_M:
            bad.append(f"reach {np.linalg.norm(tb):.3f} > {MAX_REACH_M}")
        if not (Z_RANGE_M[0] <= tb[2] <= Z_RANGE_M[1]):
            bad.append(f"z {tb[2]:.3f} outside {Z_RANGE_M}")
        if err > MAX_IK_ERR_M:
            bad.append(f"IK err {err*100:.2f} cm > {MAX_IK_ERR_M*100:.1f}")
        if rerr > MAX_IK_ROT_ERR_DEG:
            bad.append(f"IK orientation err {rerr:.1f} deg > {MAX_IK_ROT_ERR_DEG}")
        jump = max_joint_delta_deg(q, q_cur)
        if jump > MAX_JOINT_STEP_DEG:
            bad.append(f"joint jump {jump:.1f} deg > {MAX_JOINT_STEP_DEG}")
        return q, err, target, jump, bad

    # -- execution ----------------------------------------------------------------------------
    def smooth_line(self, target, name, log, posture=None, spacing=SMOOTH_SPACING_M):
        """EE straight line to `target` (base xyz, Kinova tool frame), orientation kept, as
        ONE blended Cartesian trajectory -- no stop between points.

        Every interpolated point is pre-checked in the sim with the same gates as step()
        (reach, height band, IK error, per-point joint jump), chained from the current
        joints. Nothing is sent if any point fails. Kortex runs its own IK for the
        trajectory, so the sim chain is a reachability proxy; once sent there is no
        per-point abort, only the final-position check. Returns the final EE xyz.
        """
        st = self.state()
        ee0 = np.asarray(list(st["ee_pos"])[:3], dtype=float)
        quat = np.asarray(list(st["ee_pos"])[3:7], dtype=float)
        target = np.asarray(target, dtype=float)
        dist = float(np.linalg.norm(target - ee0))
        if dist < 1e-4:
            return ee0
        n = max(1, int(np.ceil(dist / spacing)))
        pts = [ee0 + (target - ee0) * (k / n) for k in range(1, n + 1)]
        q = np.asarray(st["position"], dtype=float)
        prev = ee0
        worst_err, worst_jump = 0.0, 0.0
        for i, pt in enumerate(pts):
            q, err, _, jump, bad = self.solve_translation(q, pt - prev, posture=posture)
            worst_err, worst_jump = max(worst_err, err), max(worst_jump, jump)
            if bad:
                log({"step": name, "smooth": True, "precheck_failed_at": i + 1, "gates": bad})
                raise Abort(f"{name}: smooth pre-check failed at point {i + 1}/{n}: {'; '.join(bad)}"
                            " -- arm not commanded")
            prev = pt
        print(f"  {name:22s} {dist*100:5.1f} cm smooth ({n} pts pre-checked: IK <= {worst_err*1000:.1f} mm,"
              f" joint step <= {worst_jump:.1f} deg)")
        rec = {"step": name, "smooth": True, "target": target.tolist(), "ee_before": ee0.tolist(),
               "n_points": n, "executed": False}
        if not self.execute:
            log(rec)
            return ee0
        ok = self.ai.execute_command(CartesianTrajectoryCommand([(pt, quat) for pt in pts]))
        fin = wait_still(self.ai, target)
        err = float(np.linalg.norm(fin - target))
        rec.update(executed=True, returned=bool(ok), ee_after=fin.tolist(), final_err_m=err)
        log(rec)
        print(f"  {name:22s} reached {np.round(fin, 3)}  ({err*1000:.1f} mm from target)")
        if err > TRACK_ABORT_M:
            raise Abort(f"{name}: ended {err*100:.1f} cm from the target -- HOLDING HERE")
        return fin

    def ee_pose(self):
        """(xyz, 3x3 rotation) of the Kinova tool frame in the base frame."""
        ee = list(self.state()["ee_pos"])
        return np.asarray(ee[:3], dtype=float), Rotation.from_quat(ee[3:7]).as_matrix()

    def precheck_poses(self, poses, name, log, posture=None, start=None):
        """Gate a whole path of (xyz, R) Kinova tool poses in the sim before anything moves.

        Chained from `start` = (q, xyz, R) -- the current joints and tool pose by default -- so
        several legs can be planned back to back. Raises Abort at the first failing point;
        returns (worst IK err m, worst joint step deg, end state for the next leg)."""
        if start is None:
            q = self.joints()
            prev_p, prev_R = self.ee_pose()
        else:
            q, prev_p, prev_R = start
        worst_err, worst_jump = 0.0, 0.0
        for i, (pt, Rt) in enumerate(poses):
            q, err, _, jump, bad = self.solve_translation(q, pt - prev_p, posture=posture, rot=Rt @ prev_R.T)
            worst_err, worst_jump = max(worst_err, err), max(worst_jump, jump)
            if bad:
                log({"step": name, "precheck_failed_at": i + 1, "n_points": len(poses), "gates": bad})
                raise Abort(f"{name}: pre-check failed at point {i + 1}/{len(poses)}: {'; '.join(bad)}"
                            " -- arm not commanded")
            prev_p, prev_R = pt, Rt
        return worst_err, worst_jump, (q, prev_p, prev_R)

    def smooth_poses(self, poses, name, log):
        """Send an already pre-checked (xyz, R) path as ONE blended Cartesian trajectory, then
        verify the final position. Same contract as smooth_line."""
        target = np.asarray(poses[-1][0], dtype=float)
        rec = {"step": name, "smooth": True, "target": target.tolist(), "n_points": len(poses), "executed": False}
        if not self.execute:
            log(rec)
            return self.ee_pos()
        traj = [(np.asarray(pt, dtype=float), Rotation.from_matrix(Rt).as_quat()) for pt, Rt in poses]
        ok = self.ai.execute_command(CartesianTrajectoryCommand(traj))
        fin = wait_still(self.ai, target)
        err = float(np.linalg.norm(fin - target))
        rec.update(executed=True, returned=bool(ok), ee_after=fin.tolist(), final_err_m=err)
        log(rec)
        print(f"  {name:22s} reached {np.round(fin, 3)}  ({err*1000:.1f} mm from target)")
        if err > TRACK_ABORT_M:
            raise Abort(f"{name}: ended {err*100:.1f} cm from the target -- HOLDING HERE")
        return fin

    def step(self, d_base, name, log, posture=None, rot=None):
        """Plan + gate + (if executing) move the EE by d_base (and turn it by the world-frame
        rotation `rot`, if given). Returns the joint vector reached."""
        q0 = self.joints()
        ee0 = self.ee_pos()
        q, err, target, jump, bad = self.solve_translation(q0, d_base, posture=posture, rot=rot)
        rec = {"step": name, "d_base": list(map(float, d_base)), "ik_err_m": err, "jump_deg": jump,
               "gates": bad, "q": q.tolist(), "ee_before": ee0.tolist(), "executed": False}
        print(f"  {name:22s} d={np.round(d_base*100, 2)} cm  IK {err*100:.2f} cm  jump {jump:4.1f} deg"
              f"  {'FAIL: ' + '; '.join(bad) if bad else 'ok'}")
        if bad:
            log(rec)
            raise Abort(f"gate failed at {name}: {'; '.join(bad)} -- arm not commanded")
        if not self.execute:
            log(rec)
            return q
        self.ai.execute_command(JointCommand(pos=q.tolist()))
        derr = wait_converged(self.ai, q)
        if derr >= CONVERGE_TOL_DEG:
            print(f"  {name}: settled {derr:.1f} deg short (Kortex likely dropped it) -- re-sending once")
            time.sleep(0.5)
            self.ai.execute_command(JointCommand(pos=q.tolist()))
            derr = wait_converged(self.ai, q)
        ee1 = self.ee_pos()
        track = float(np.linalg.norm((ee1 - ee0) - np.asarray(d_base)))
        rec.update(executed=True, converge_deg=derr, ee_after=ee1.tolist(), track_err_m=track)
        log(rec)
        print(f"  {name:22s} moved {np.round((ee1-ee0)*100, 2)} cm  converge {derr:.2f} deg  track {track*100:.2f} cm")
        if derr >= CONVERGE_TOL_DEG:
            raise Abort(f"{name}: still {derr:.1f} deg off after retry -- HOLDING HERE")
        if track > TRACK_ABORT_M:
            raise Abort(f"{name}: tracking error {track*100:.1f} cm > {TRACK_ABORT_M*100:.0f} -- HOLDING HERE")
        return q
