"""Plan the full door-arc waypoint sequence (chain-seeded, joint-delta guarded)
for a single continuous multi-waypoint execution -- no motion here, sim-only
verification. Uses the CURRENT real grasp pose as start_pose and a
hinge-relative-offset derived from detection (robust to the grasp point having
shifted a bit since detection, e.g. from a manual backoff/teleop nudge)."""
import json
import sys

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator
from feeding_deployment.control.robot_controller.arm_interface import (
    ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY,
)

ARM_JOINTS = [1, 2, 3, 4, 5, 6, 7]

# From detection: hinge_pose - handle_pose (relative offset, robust to the grasp
# point having shifted since that detection).
HANDLE_POS_AT_DETECT = np.array([0.74838382, -0.17329798, 0.43816104])
HINGE_POS_AT_DETECT = np.array([0.77228347, -0.29278994, 0.43442196])
HINGE_OFFSET = HINGE_POS_AT_DETECT - HANDLE_POS_AT_DETECT

TARGET_ANGLE_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
WAYPOINT_SPACING_M = 0.02
MAX_JOINT_DELTA_DEG = 25.0
IK_ERR_ABORT = 0.02
REACH_GUARD = 0.90
OUT = sys.argv[2] if len(sys.argv) > 2 else "/tmp/arc_plan_continuous.json"

ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()
real_joints = list(ai.get_state()["position"])
ee = list(ai.get_state()["ee_pos"])
grasp = Pose(position=tuple(ee[:3]), orientation=tuple(ee[3:7]))
hinge = tuple(np.array(ee[:3]) + HINGE_OFFSET)
radius = float(np.linalg.norm(np.array(ee[:3]) - np.array(hinge)))
arc_length_m = radius * np.radians(TARGET_ANGLE_DEG)

print(f"grasp ee_pos: {np.round(ee[:3], 4)}")
print(f"hinge estimate: {np.round(hinge, 4)} (radius {radius*100:.1f}cm)")
print(f"target angle {TARGET_ANGLE_DEG}deg -> arc_length {arc_length_m*100:.1f}cm")

wps_pose = PerceptionInterface._generate_door_arc_waypoints(
    None, start_pose=grasp, hinge_position=hinge,
    arc_length_m=arc_length_m, waypoint_spacing_m=WAYPOINT_SPACING_M,
    direction=1, rotate_orientation=True,
)
print(f"{len(wps_pose)} waypoints")

sd = create_scene_description_from_config("src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=False)
rb = sim.robot

seed = np.array(real_joints)
all_ok = True
plan = []
for i, wp in enumerate(wps_pose, 1):
    pos, quat = wp.position, wp.orientation
    for j, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, seed[j], physicsClientId=rb.physics_client_id)
    wpose = multiply_poses(sd.robot_base_pose, Pose(tuple(pos), tuple(quat)))
    sol = p.calculateInverseKinematics(
        rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
        physicsClientId=rb.physics_client_id, maxNumIterations=200,
    )
    joints = np.array([sol[k] for k in range(7)])
    for j, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, joints[j], physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    ikerr = float(np.linalg.norm(np.array(ls[4]) - np.array(wpose.position)))
    delta = float(np.degrees(np.max(np.abs(joints - seed))))
    d = float(np.linalg.norm(pos))
    ok = ikerr <= IK_ERR_ABORT and d <= REACH_GUARD and delta <= MAX_JOINT_DELTA_DEG
    all_ok = all_ok and ok
    status = "OK" if ok else "BAD"
    print(f"  step {i}/{len(wps_pose)}: target {np.round(pos,3)} ({d*100:.0f}cm from base) "
          f"ik_err={ikerr*100:.2f}cm joint_delta={delta:.1f}deg {status}")
    plan.append({"pos": list(pos), "joints": joints.tolist()})
    seed = joints

print()
print("ALL WAYPOINTS OK:", all_ok)
with open(OUT, "w") as f:
    json.dump({"waypoints": plan, "all_ok": bool(all_ok), "target_angle_deg": TARGET_ANGLE_DEG,
               "hinge": list(hinge), "radius": radius}, f)
print(f"saved plan to {OUT}")
