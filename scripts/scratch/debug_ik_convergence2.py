"""IK convergence check for TODAY's real detected handle position (2026-07-28).

Cleaned-up version: a single correct geometry_corrected_quat() helper (the first
draft of this script had a bug where the exactly-antiparallel special case picked
a rotation axis that happened to coincide with the vector being flipped, silently
making the "fix" a no-op whenever the look-at direction was exactly a world axis --
exactly the case that matters here, since the back-off convention is pure world -x).
No arm motion.
"""
import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses
from scipy.spatial.transform import Rotation as Rot, Slerp

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager,
)
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

CFG = "src/feeding_deployment/simulation/configs/vention.yaml"
ARM_JOINTS = [1, 2, 3, 4, 5, 6, 7]

HANDLE_POS = np.array([0.84064532, -0.18060141, 0.47008189])
HANDLE_QUAT = np.array([0.5, -0.5, -0.5, 0.5])  # xyzw, repo's fixed constant


def geometry_corrected_quat(base_quat, old_approach_local_axis, approach_vec):
    """Re-point base_quat's approach axis at approach_vec (world frame), preserving
    the roll base_quat had about its original axis. Correctly handles the
    exactly-antiparallel case (arbitrary but genuinely PERPENDICULAR axis, not a
    hardcoded world axis that may coincide with the vector being flipped)."""
    base_rot = Rot.from_quat(base_quat)
    old_z_world = base_rot.apply(old_approach_local_axis)
    cross = np.cross(old_z_world, approach_vec)
    dot = np.clip(np.dot(old_z_world, approach_vec), -1, 1)
    cross_norm = np.linalg.norm(cross)
    if cross_norm < 1e-8:
        if dot > 0:
            align_rot = Rot.identity()
        else:
            perp = np.array([1.0, 0.0, 0.0])
            if abs(np.dot(perp, old_z_world)) > 0.9:
                perp = np.array([0.0, 1.0, 0.0])
            axis = np.cross(old_z_world, perp)
            axis /= np.linalg.norm(axis)
            align_rot = Rot.from_rotvec(np.pi * axis)
    else:
        axis = cross / cross_norm
        angle = np.arctan2(cross_norm, dot)
        align_rot = Rot.from_rotvec(angle * axis)
    new_rot = align_rot * base_rot
    z_check = new_rot.apply(old_approach_local_axis)
    assert np.allclose(z_check, approach_vec, atol=1e-6), (z_check, approach_vec)
    return new_rot


def seed_joints(rb, joints):
    for i, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, joints[i], physicsClientId=rb.physics_client_id)


def solve_ik(rb, sd, seed, pose):
    seed_joints(rb, seed)
    wpose = multiply_poses(sd.robot_base_pose, pose)
    sol = p.calculateInverseKinematics(
        rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
        physicsClientId=rb.physics_client_id, maxNumIterations=200,
    )
    joints = [sol[k] for k in range(7)]
    seed_joints(rb, joints)
    link_state = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    err = float(np.linalg.norm(np.array(link_state[4]) - np.array(wpose.position)))
    lower, upper = np.array(rb.joint_lower_limits[:7]), np.array(rb.joint_upper_limits[:7])
    within = all(lower[i] - 1e-3 <= joints[i] <= upper[i] + 1e-3 for i in range(7) if not np.isinf(lower[i]))
    return joints, err, within


def hover_target(backoff):
    tp = HANDLE_POS - np.array([backoff, 0, 0])
    approach_vec = HANDLE_POS - tp
    approach_vec /= np.linalg.norm(approach_vec)
    corrected_rot = geometry_corrected_quat(HANDLE_QUAT, [0, 0, 1], approach_vec)
    return tp, tuple(corrected_rot.as_quat())


def main():
    ArmManager.register("ArmInterface")
    mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    mg.connect()
    ai = mg.ArmInterface()
    real_joints = list(ai.get_state()["position"])
    real_ee = ai.get_state()["ee_pos"]
    print(f"current real joints: {np.round(real_joints, 3)}")
    print(f"current real ee_pos: {np.round(real_ee, 4)}")

    sd = create_scene_description_from_config(CFG, "skewer")
    sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=False)
    rb = sim.robot
    n_finger = len(rb.get_joint_positions()) - 7
    real_seed = real_joints + [0.0] * n_finger
    home = list(rb.home_joint_positions[:7]) + [0.0] * n_finger
    lower = np.array(rb.joint_lower_limits[:7])
    upper = np.array(rb.joint_upper_limits[:7])
    lower_f = np.where(np.isinf(lower), -np.pi, lower)
    upper_f = np.where(np.isinf(upper), np.pi, upper)
    rng = np.random.default_rng(4)

    print("\n--- backoff sweep, PROPERLY geometry-corrected orientation, wide multistart ---")
    for backoff in [0.12, 0.15, 0.18, 0.22, 0.25, 0.28, 0.30, 0.35, 0.40]:
        tp, q = hover_target(backoff)
        pose_b = Pose(tuple(tp), q)
        best = None
        for i in range(80):
            cand = list(rng.uniform(lower_f, upper_f)) + [0.0] * n_finger
            joints, err, within = solve_ik(rb, sd, cand, pose_b)
            if best is None or (within, -err) > (best[2], -best[1]):
                best = (joints, err, within)
        print(f"  backoff={backoff:.2f}m pos={np.round(tp,3)} quat={np.round(q,3)}: "
              f"best err={best[1]*100:5.2f}cm within_limits={best[2]}")

    print("\n--- for comparison: RAW (uncorrected) HANDLE_QUAT, same backoff sweep ---")
    for backoff in [0.12, 0.20, 0.30, 0.40]:
        tp = HANDLE_POS - np.array([backoff, 0, 0])
        pose_raw = Pose(tuple(tp), tuple(HANDLE_QUAT))
        best = None
        for i in range(80):
            cand = list(rng.uniform(lower_f, upper_f)) + [0.0] * n_finger
            joints, err, within = solve_ik(rb, sd, cand, pose_raw)
            if best is None or (within, -err) > (best[2], -best[1]):
                best = (joints, err, within)
        print(f"  backoff={backoff:.2f}m RAW quat: best err={best[1]*100:5.2f}cm within_limits={best[2]}")

    print("\n--- final: chained interpolation from ACTUAL current real ee -> "
          "backoff=0.30m, geometry-corrected quat ---")
    tp, final_quat = hover_target(0.30)
    print(f"  target pos={np.round(tp,4)} quat={np.round(final_quat,4)}")
    cur_pos = np.array(real_ee[:3])
    cur_quat = np.array(real_ee[3:7])
    key_rots = Rot.from_quat([cur_quat, np.array(final_quat)])
    slerp = Slerp([0, 1], key_rots)
    seed = real_seed
    n_way = 10
    all_within = True
    for step in range(1, n_way + 1):
        t = step / n_way
        way_pos = tuple(cur_pos + (tp - cur_pos) * t)
        way_quat = tuple(slerp([t]).as_quat()[0])
        joints, err, within = solve_ik(rb, sd, seed, Pose(way_pos, way_quat))
        all_within = all_within and within
        print(f"  chain {step}/{n_way}: err={err*100:.2f}cm within_limits={within}")
        seed = list(joints) + [0.0] * n_finger
    print(f"\n  FINAL joint target: {list(np.round(joints, 5))}")
    print(f"  all steps within limits: {all_within}, final err: {err*100:.2f}cm")


if __name__ == "__main__":
    main()
