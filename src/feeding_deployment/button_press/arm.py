"""Arm side of the button press: seeded PyBullet IK, gated joint steps, convergence checks.

Every move is "translate the end effector by d, same orientation". It is planned in a
headless PyBullet copy of the arm, refused if any gate fails (reach, height, IK error,
joint jump), and only then sent to the real arm as a joint command over the arm RPC.
"""
from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pybullet as p

from feeding_deployment.button_press import Abort
from feeding_deployment.button_press.geometry import max_joint_delta_deg
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import JointCommand
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

# ---- motion gates (same family as the grasp scripts; tighter where the moves are smaller)
MAX_IK_ERR_M = 0.005
SEED_GOOD_M = 0.001          # posture seed is honoured unquestioned below this; see solve_translation
MAX_JOINT_STEP_DEG = 10.0
MAX_REACH_M = 0.91           # existing rig constant -- do not raise without asking
Z_RANGE_M = (0.25, 0.75)
TRACK_ABORT_M = 0.01
CONVERGE_TOL_DEG = 1.0
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

    def solve_translation(self, q_cur, d_base, posture=None):
        """IK for 'current EE + d_base', same orientation.

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
        Returns (q, ik_err_m, target_world, jump_deg, gate_failures).
        """
        pos, quat = self.fk(q_cur)
        target = pos + np.asarray(d_base, dtype=float)
        seeds = [posture, q_cur] if posture is not None else [q_cur]
        q = None
        err = None
        for seed in seeds:
            self._set_sim(seed)
            sol = p.calculateInverseKinematics(
                self.rb.robot_id, self.rb.end_effector_id, list(target), list(quat),
                maxNumIterations=400, residualThreshold=1e-5, physicsClientId=self.rb.physics_client_id)
            q_try = np.asarray(sol[:7], dtype=float)
            got, _ = self.fk(q_try)
            err_try = float(np.linalg.norm(got - target))
            if q is None or err_try < err:
                q, err = q_try, err_try
            # Take the posture seed the moment it is GOOD, not merely legal. Accepting the
            # first solution under MAX_IK_ERR_M (5 mm) is what broke the 2026-09-21 stage-1
            # servo: a stale anchor returned ~3 mm error, passed the gate, and the more
            # accurate q_cur seed was never tried -- while the corrections being asked for
            # were themselves only 1-4 mm. Below SEED_GOOD_M the anchor is honoured (no
            # null-space drift); above it, both seeds are solved and the closer one wins.
            if err <= SEED_GOOD_M:
                break
        bad = []
        tb = target - self.base_pos
        if np.linalg.norm(tb) > MAX_REACH_M:
            bad.append(f"reach {np.linalg.norm(tb):.3f} > {MAX_REACH_M}")
        if not (Z_RANGE_M[0] <= tb[2] <= Z_RANGE_M[1]):
            bad.append(f"z {tb[2]:.3f} outside {Z_RANGE_M}")
        if err > MAX_IK_ERR_M:
            bad.append(f"IK err {err*100:.2f} cm > {MAX_IK_ERR_M*100:.1f}")
        jump = max_joint_delta_deg(q, q_cur)
        if jump > MAX_JOINT_STEP_DEG:
            bad.append(f"joint jump {jump:.1f} deg > {MAX_JOINT_STEP_DEG}")
        return q, err, target, jump, bad

    # -- execution ----------------------------------------------------------------------------
    def step(self, d_base, name, log, posture=None):
        """Plan + gate + (if executing) move the EE by d_base. Returns the joint vector reached."""
        q0 = self.joints()
        ee0 = self.ee_pos()
        q, err, target, jump, bad = self.solve_translation(q0, d_base, posture=posture)
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
