"""One-off: move to a hover pose near the microwave handle using seeded-IK +
joint-space control -- the Jetson-proven pattern for a big single jump (see
scripts/real_gen3_detect_grasp_microwave.py's solve_ik_seeded/move_and_check, and
the "seeded-IK fix" entry in TESTING_LOG.md). Not a grasp -- hover only, no gripper
action, stays well short of the handle.

This replaces a raw arm.set_ee_pose() cartesian call, which this session's real-arm
test showed can actually attempt a real, partially-executed trajectory (hit
JOINT_ACCELERATION_LIMIT_REACHED mid-move) rather than a clean no-op, contrary to
the LOCAL_DEPLOYMENT.md note that it "aborts... no motion" -- that note is not
reliable enough on its own for a real move this size.
"""

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

CFG = "src/feeding_deployment/simulation/configs/vention.yaml"
ARM_JOINTS = [1, 2, 3, 4, 5, 6, 7]

TARGET_POS = (0.59638588, -0.12272994, 0.45929806)
TARGET_QUAT = (0.5, -0.5, -0.5, 0.5)  # xyzw, same convention as detect_handle_and_placement
ABORT_M = 0.05


def solve_ik_seeded(rb, sd, target_pose, ai):
    cur = ai.get_state()["position"]
    for i, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, cur[i], physicsClientId=rb.physics_client_id)
    wpose = multiply_poses(sd.robot_base_pose, target_pose)
    sol = p.calculateInverseKinematics(
        rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
        physicsClientId=rb.physics_client_id, maxNumIterations=200,
    )
    joint_target = [sol[k] for k in range(7)]

    # Verify via FK: how close does the IK solution actually land, in sim?
    for i, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, joint_target[i], physicsClientId=rb.physics_client_id)
    link_state = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    sim_ee_world = np.array(link_state[4])
    target_world = np.array(wpose.position)
    ik_err = float(np.linalg.norm(sim_ee_world - target_world))
    return joint_target, ik_err


def main():
    ArmManager.register("ArmInterface")
    mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    mg.connect()
    ai = mg.ArmInterface()

    real_ee = ai.get_state()["ee_pos"]
    print(f"[hover] current ee_pos: {np.round(real_ee, 4)}")

    sd = create_scene_description_from_config(CFG, "skewer")
    sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=False)
    rb = sim.robot

    # Sanity check: seed sim from current real joints, then FK -- does the sim's
    # own idea of "current EE" (via robot_base_pose composition) match the real
    # ee_pos we already know? If not, the scene/frame config itself is wrong,
    # independent of anything to do with IK.
    cur = ai.get_state()["position"]
    for i, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, cur[i], physicsClientId=rb.physics_client_id)
    link_state = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    sim_ee_world = np.array(link_state[4])
    real_ee_pos = np.array(real_ee[:3])
    fk_sanity_err = float(np.linalg.norm(sim_ee_world - real_ee_pos))
    print(f"[hover] FK-of-current-joints sim EE (world frame): {np.round(sim_ee_world, 4)}")
    print(f"[hover] real ee_pos (arm-base frame):               {np.round(real_ee_pos, 4)}")
    print(f"[hover] FK sanity check error (sim world vs real ee_pos, no robot_base_pose composition): "
          f"{fk_sanity_err * 100:.2f}cm")
    print(f"[hover] sd.robot_base_pose: {sd.robot_base_pose}")

    target_pose = Pose(TARGET_POS, TARGET_QUAT)
    joint_target, ik_err = solve_ik_seeded(rb, sd, target_pose, ai)
    print(f"[hover] seeded IK solution: {np.round(joint_target, 4)}")
    print(f"[hover] sim ik_err: {ik_err * 100:.2f}cm")

    if ik_err > 0.02:
        print(f"[hover] ABORT: ik_err {ik_err*100:.1f}cm > 2cm threshold -- not commanding. "
              f"(Per TESTING_LOG.md, a bad seeded-IK solution here risks a wrist flip.)")
        return

    print("[hover] IK solution looks good. NOT commanding automatically -- "
          "confirm before sending set_joint_position.")
    print(f"[hover] joint_target = {list(np.round(joint_target, 5))}")


if __name__ == "__main__":
    main()
