import json
import sys

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses
from scipy.spatial.transform import Rotation as Rot, Slerp

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager,
)
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

ARM_JOINTS = [1, 2, 3, 4, 5, 6, 7]
HANDLE_POS = np.array([0.74838382, -0.17329798, 0.43816104])
HANDLE_QUAT = np.array([0.5, -0.5, -0.5, 0.5])
BACKOFF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.30
N_WAY = int(sys.argv[2]) if len(sys.argv) > 2 else 16
ROLL_DEG = float(sys.argv[3]) if len(sys.argv) > 3 else 90.0
OUT = sys.argv[4] if len(sys.argv) > 4 else "/tmp/hover_chain_plan_v2.json"


def geometry_corrected_quat(base_quat, old_axis, approach_vec, extra_roll_deg=0.0):
    base_rot = Rot.from_quat(base_quat)
    old_z = base_rot.apply(old_axis)
    cross = np.cross(old_z, approach_vec)
    dot = np.clip(np.dot(old_z, approach_vec), -1, 1)
    cn = np.linalg.norm(cross)
    if cn < 1e-8:
        align = Rot.identity() if dot > 0 else Rot.from_rotvec(np.pi * np.array([0, 1, 0]))
    else:
        axis = cross / cn
        angle = np.arctan2(cn, dot)
        align = Rot.from_rotvec(angle * axis)
    new_rot = align * base_rot
    if extra_roll_deg != 0.0:
        roll = Rot.from_rotvec(np.radians(extra_roll_deg) * np.array(old_axis))
        new_rot = new_rot * roll
    return new_rot


ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()
real_joints = np.array(ai.get_state()["position"])
real_ee = ai.get_state()["ee_pos"]
print(f"current real ee: {np.round(real_ee, 4)}")

sd = create_scene_description_from_config("src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=False)
rb = sim.robot

tp = HANDLE_POS - np.array([BACKOFF, 0, 0])
approach_vec = HANDLE_POS - tp
approach_vec /= np.linalg.norm(approach_vec)
final_quat = geometry_corrected_quat(HANDLE_QUAT, [0, 0, 1], approach_vec, ROLL_DEG).as_quat()
print(f"hover target pos={np.round(tp, 4)} quat(+{ROLL_DEG}deg roll)={np.round(final_quat, 4)}")

cur_pos = np.array(real_ee[:3])
cur_quat = np.array(real_ee[3:7])
key_rots = Rot.from_quat([cur_quat, final_quat])
slerp = Slerp([0, 1], key_rots)

seed = real_joints.copy()
all_ok = True
waypoints, positions = [], []
print(f"Chaining from {np.round(cur_pos, 4)} to hover {np.round(tp, 4)} (backoff={BACKOFF}) over {N_WAY} steps")
for step in range(1, N_WAY + 1):
    t = step / N_WAY
    way_pos = cur_pos + (tp - cur_pos) * t
    way_quat = slerp([t]).as_quat()[0]
    for i, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, seed[i], physicsClientId=rb.physics_client_id)
    wpose = multiply_poses(sd.robot_base_pose, Pose(tuple(way_pos), tuple(way_quat)))
    sol = p.calculateInverseKinematics(
        rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
        physicsClientId=rb.physics_client_id, maxNumIterations=200,
    )
    joints = np.array([sol[k] for k in range(7)])
    for i, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, joints[i], physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    err = np.linalg.norm(np.array(ls[4]) - np.array(wpose.position))
    delta = np.degrees(np.max(np.abs(joints - seed)))
    ok = err < 0.02 and delta < 20.0
    all_ok = all_ok and ok
    status = "OK" if ok else "BAD"
    print(f"  step {step}/{N_WAY}: pos={np.round(way_pos, 4)} ik_err={err*100:.2f}cm joint_delta={delta:.1f}deg {status}")
    waypoints.append(joints.tolist())
    positions.append(way_pos.tolist())
    seed = joints

print()
print("ALL STEPS OK:", all_ok)
print("FINAL joints:", list(np.round(seed, 5)))

with open(OUT, "w") as f:
    json.dump({"waypoints": waypoints, "positions": positions, "all_ok": bool(all_ok)}, f)
print(f"saved plan to {OUT}")
