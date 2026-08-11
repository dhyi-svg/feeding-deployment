"""OFFLINE: what does the EE actually do between two joint configurations?

The task scripts gate on the ENDPOINTS only -- IK error at the target and the
per-joint delta. Neither says anything about the PATH. A joint-space move
interpolates in joint space, so the end effector traces a curve, not a straight
line, and nothing in this repo collision-checks it.

This replays the interpolation in PyBullet (no arm, no camera) and reports where
the EE actually goes: total arc length vs the straight-line distance, how far it
bows away from that line, and its height range.

    python scripts/session/check_joint_path.py --to "-4.5,16.5,-132.4,-119.4,47.9,61.8,46.3"

--from defaults to the live arm if arm_server is up, else pass it explicitly.
Angles in degrees.
"""
import argparse

import numpy as np
import pybullet as p

from feeding_deployment.simulation.scene_description import (
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

ARM_JOINTS = [1, 2, 3, 4, 5, 6, 7]

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--to", required=True, help="target joints, comma-separated degrees")
parser.add_argument("--from", dest="frm", default=None, help="start joints, degrees (default: live arm)")
parser.add_argument("--steps", type=int, default=60)
args = parser.parse_args()

target = np.radians([float(v) for v in args.to.split(",")])
if args.frm:
    start = np.radians([float(v) for v in args.frm.split(",")])
else:
    from feeding_deployment.control.robot_controller.arm_interface import (
        ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager)
    ArmManager.register("ArmInterface")
    manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    manager.connect()
    start = np.asarray(manager.ArmInterface().get_state()["position"], dtype=float)

scene = create_scene_description_from_config(
    "src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
robot = sim.robot

def ee_at(q):
    for i, joint in enumerate(ARM_JOINTS):
        p.resetJointState(robot.robot_id, joint, float(q[i]),
                          physicsClientId=robot.physics_client_id)
    link = p.getLinkState(robot.robot_id, robot.end_effector_id,
                          physicsClientId=robot.physics_client_id)
    # Back out of sim-world into arm_base_link, the frame the scripts work in.
    return np.asarray(link[4]) - np.asarray(scene.robot_base_pose.position)

# Shortest periodic path per joint -- matches how the controller interpolates.
delta = (target - start + np.pi) % (2 * np.pi) - np.pi
path = np.array([ee_at(start + f * delta) for f in np.linspace(0.0, 1.0, args.steps)])

a, b = path[0], path[-1]
chord = float(np.linalg.norm(b - a))
arc = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))
direction = (b - a) / (chord if chord > 1e-9 else 1.0)
bow = float(np.max(np.linalg.norm(np.cross(path - a, direction), axis=1)))

print(f"per-joint delta (deg) : {[round(float(v), 1) for v in np.degrees(delta)]}")
print(f"max single joint      : {np.max(np.abs(np.degrees(delta))):.1f} deg")
print(f"joints moving > 20 deg: {int(np.sum(np.abs(np.degrees(delta)) > 20))} of 7")
print()
print(f"EE start / end        : {np.round(a, 3)} -> {np.round(b, 3)}")
print(f"straight-line distance: {chord * 100:6.1f} cm")
print(f"actual path length    : {arc * 100:6.1f} cm   ({arc / max(chord, 1e-9):.2f}x)")
print(f"max bow off the line  : {bow * 100:6.1f} cm")
print(f"EE height range (z)   : {path[:, 2].min():.3f} .. {path[:, 2].max():.3f} m")
print(f"EE reach range        : {np.linalg.norm(path, axis=1).min():.3f} .. "
      f"{np.linalg.norm(path, axis=1).max():.3f} m")
