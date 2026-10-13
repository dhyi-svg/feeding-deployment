"""Seeded PyBullet IK and the motion gates -- no ROS, no arm.

Ported from ``feeding_deployment.button_press.arm.Arm`` (button-task). The IK, the seeding and
every gate constant are unchanged; what changed:

* The robot is the Gen3 URDF loaded on its own (``urdf/gen3_tool_frame_kinematic.urdf``: the
  same chain to the same ``tool_frame`` link the full ``FeedingDeploymentPyBulletSimulator``
  planned in, without meshes) instead of the whole feeding scene. The base sits at the origin,
  so sim world == arm base frame (the old scene offset the base by a pure translation, which
  every gate already subtracted). ``test/test_kinematics.py`` checks FK/IK against the full sim.
* ``precheck_poses`` also returns the joint solution of every point. The old stack checked the
  path here and then let Kortex run its OWN IK on a Cartesian trajectory; RAMMP's driver has no
  Cartesian action, so ``RammpArm`` sends exactly these joints (``execute_joint_trajectory``).
* NEW gate, joint limits (``JOINT_LIMIT_MARGIN_DEG``): with Kortex doing the IK this never had
  to be checked here; now our solutions are what the driver receives.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from rammp_button_press import Abort
from rammp_button_press.geometry import max_joint_delta_deg

# ---- motion gates (same family as the grasp scripts; tighter where the moves are smaller)
MAX_IK_ERR_M = 0.005
MAX_IK_ROT_ERR_DEG = 1.0     # orientation miss allowed when a move also turns the wrist
SEED_GOOD_M = 0.001          # posture seed is honoured unquestioned below this; see solve_translation
MAX_JOINT_STEP_DEG = 10.0
MAX_REACH_M = 0.91           # existing rig constant -- do not raise without asking
Z_RANGE_M = (0.12, 0.75)       # min 0.25 -> 0.15 (user-approved 2026-09-27, button ~0.22 m above base);
                               # 0.15 -> 0.12 (user-approved 2026-10-03): button now at 0.171, one-command
                               # pre-press tool point at 0.128, deepest press point ~0.125
# Revolute joints (J2/J4/J6) must stay this far inside their URDF limits (= kinova_gen3_node's
# gen3_7dof_2f85.urdf: J2 +-2.41, J4 +-2.66, J6 +-2.23 rad). New with the RAMMP port (see the
# module docstring). The arm repo's runbook (2026-09-01) saw a run abort with J4 at -2.660.
JOINT_LIMIT_MARGIN_DEG = 2.0

JOINTS = [f"joint_{i}" for i in range(1, 8)]
EE_LINK = "tool_frame"
URDF_NAME = "gen3_tool_frame_kinematic.urdf"


def _find_urdf() -> Path:
    """Source checkout (urdf/ next to the package dir) or colcon install (share/rammp_button_press/urdf)."""
    here = Path(__file__).resolve().parent
    candidates = [here.parent / "urdf" / URDF_NAME]
    try:
        from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415
        candidates.append(Path(get_package_share_directory("rammp_button_press")) / "urdf" / URDF_NAME)
    except Exception:  # noqa: BLE001 -- not installed / no ament: the source path is all there is
        pass
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError(f"{URDF_NAME} not found in {[str(c) for c in candidates]}")


class Kinematics:
    """Headless PyBullet copy of the arm: FK, the seeded IK and the gates."""

    def __init__(self, urdf: str | Path | None = None):
        self.cid = p.connect(p.DIRECT)
        self.robot_id = p.loadURDF(str(urdf or _find_urdf()), useFixedBase=True, physicsClientId=self.cid)
        by_name, self.limits = {}, []
        for j in range(p.getNumJoints(self.robot_id, physicsClientId=self.cid)):
            info = p.getJointInfo(self.robot_id, j, physicsClientId=self.cid)
            by_name[info[1].decode()] = j
            if info[12].decode() == EE_LINK:
                self.end_effector_id = j
        self.arm = [by_name[n] for n in JOINTS]
        for j in self.arm:
            info = p.getJointInfo(self.robot_id, j, physicsClientId=self.cid)
            revolute = info[2] == p.JOINT_REVOLUTE and info[8] < info[9]
            self.limits.append((info[8], info[9]) if revolute else None)
        self.base_pos = np.zeros(3)

    def close(self):
        p.disconnect(physicsClientId=self.cid)

    # -- kinematics ---------------------------------------------------------------------------
    def _set_sim(self, q):
        for i, jj in enumerate(self.arm):
            p.resetJointState(self.robot_id, jj, float(q[i]), physicsClientId=self.cid)

    def fk(self, q):
        """(base-frame position, quaternion xyzw) of the tool frame for joint vector q."""
        self._set_sim(q)
        ls = p.getLinkState(self.robot_id, self.end_effector_id, computeForwardKinematics=True,
                            physicsClientId=self.cid)
        return np.asarray(ls[4], dtype=float), np.asarray(ls[5], dtype=float)

    def limit_failures(self, q) -> list[str]:
        bad = []
        m = np.radians(JOINT_LIMIT_MARGIN_DEG)
        for i, lim in enumerate(self.limits):
            if lim is not None and not (lim[0] + m <= q[i] <= lim[1] - m):
                bad.append(f"J{i + 1} {np.degrees(q[i]):.1f} deg within {JOINT_LIMIT_MARGIN_DEG:.0f} deg of its "
                           f"limit [{np.degrees(lim[0]):.0f}, {np.degrees(lim[1]):.0f}]")
        return bad

    def solve_translation(self, q_cur, d_base, posture=None, rot=None):
        """IK for 'current EE + d_base', same orientation (or turned by `rot`, see below).

        The target is relative to the sim's own FK of q_cur (not the driver's ee pose), so any
        constant sim-vs-driver tool-frame offset cancels instead of becoming a jump on
        step 1. The IK is SEEDED FROM `posture` (the run's start joints) rather than from
        q_cur: the Gen3 is redundant, and re-seeding from the current pose every step let
        the solution slide along the self-motion manifold -- on 2026-09-21 J1/J3
        counter-rotated 7 -> 10.6 deg per identical 2 cm step (30 deg total in 6 cm), which
        tripped the joint-jump gate.
        Seeding from a fixed posture keeps every solution near one configuration. The anchor
        is honoured outright while it solves to better than SEED_GOOD_M; beyond that (a stale
        anchor, i.e. the arm has travelled away from it) the q_cur seed is solved too and the
        more accurate of the two wins. Run.reanchor_seed() refreshes the anchor per phase.
        `rot` (3x3, base frame) turns the EE as well: the IK orientation target is
        rot @ (sim orientation of q_cur). Left-multiplying a base-frame delta onto the sim's
        own orientation cancels a constant sim-vs-driver tool rotation the same way the
        position delta does. An orientation miss over MAX_IK_ROT_ERR_DEG fails the gates.
        Returns (q, ik_err_m, target, jump_deg, gate_failures).
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
                self.robot_id, self.end_effector_id, list(target), list(quat),
                maxNumIterations=400, residualThreshold=1e-5, physicsClientId=self.cid)
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
        bad += self.limit_failures(q)
        return q, err, target, jump, bad

    def precheck_poses(self, poses, name, log, start, posture=None):
        """Gate a whole path of (xyz, R) tool poses in the sim before anything moves.

        Chained from `start` = (q, xyz, R), so several legs can be planned back to back. Raises
        Abort at the first failing point; returns (worst IK err m, worst joint step deg, end
        state for the next leg, joint solution of every point)."""
        q, prev_p, prev_R = start
        worst_err, worst_jump, knots = 0.0, 0.0, []
        for i, (pt, Rt) in enumerate(poses):
            q, err, _, jump, bad = self.solve_translation(q, pt - prev_p, posture=posture, rot=Rt @ prev_R.T)
            worst_err, worst_jump = max(worst_err, err), max(worst_jump, jump)
            if bad:
                log({"step": name, "precheck_failed_at": i + 1, "n_points": len(poses), "gates": bad})
                raise Abort(f"{name}: pre-check failed at point {i + 1}/{len(poses)}: {'; '.join(bad)}"
                            " -- arm not commanded")
            knots.append(q)
            prev_p, prev_R = pt, Rt
        return worst_err, worst_jump, (q, prev_p, prev_R), knots
