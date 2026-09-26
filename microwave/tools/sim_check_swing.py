"""SIM ONLY -- never commands the arm. Chain seeded IK through a planned door-swing arc,
starting from the arm's REAL current joints, and print IK error / per-step joint jump / J6
for every waypoint. Used before every real swing on 2026-09-23.

    ARM_RPC_HOST=127.0.0.1 python3 microwave/tools/sim_check_swing.py 0.724 0.274 0.5425 50
    # closing: direction +1, dropping the last 3 waypoints like the close script
    ... sim_check_swing.py 0.7178 0.2562 0.5425 69 --direction 1 --stop-short 3

`--iters 200` (default) matches the swing scripts' step loop and the `--smooth` pre-check's
singularity probe: a stretch where it misses by > 1 cm is where Kortex's own Cartesian
trajectory aborted with SINGULARITY_REGION. `--iters 1000` shows whether the arc is reachable
at all (it was, on the arc that aborted).
"""
import argparse

import numpy as np
import pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

ARM = [1, 2, 3, 4, 5, 6, 7]

a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
a.add_argument("hinge", type=float, nargs=3, metavar=("X", "Y", "Z"))
a.add_argument("deg", type=float, help="swing angle, degrees")
a.add_argument("--direction", type=int, default=-1, choices=[-1, 1], help="-1 opened this door on 09-23")
a.add_argument("--stop-short", type=int, default=0, help="drop this many waypoints from the end")
a.add_argument("--spacing", type=float, default=0.02)
a.add_argument("--iters", type=int, default=200)
args = a.parse_args()

hinge = np.array(args.hinge)
st = ArmInterfaceClient().get_state()
ee, q = list(st["ee_pos"]), np.array(st["position"], float)
r = float(np.linalg.norm(np.array(ee[:3]) - hinge))
wps = PerceptionInterface._generate_door_arc_waypoints(
    None, start_pose=Pose(tuple(ee[:3]), tuple(ee[3:7])), hinge_position=tuple(hinge),
    arc_length_m=r * np.radians(args.deg), waypoint_spacing_m=args.spacing,
    direction=args.direction, rotate_orientation=True)
wps = wps[:len(wps) - args.stop_short] if args.stop_short else wps
scene = create_scene_description_from_config("src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
rb = FeedingDeploymentPyBulletSimulator(scene, use_gui=False).robot
c = rb.physics_client_id
print(f"radius {r * 100:.1f}cm, {len(wps)} waypoints, start J6 {np.degrees(q[5]):.1f}, IK iters {args.iters}")
for i, w in enumerate(wps):
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=c)
    wp = multiply_poses(scene.robot_base_pose, w)
    sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id, list(wp.position), list(wp.orientation),
                                       physicsClientId=c, maxNumIterations=args.iters,
                                       **({"residualThreshold": 1e-6} if args.iters > 200 else {}))
    nq = np.array(sol[:7])
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(nq[j]), physicsClientId=c)
    err = np.linalg.norm(np.array(rb.get_end_effector_pose().position) - np.array(wp.position))
    print(f"wp {i + 1}: {np.round(w.position, 3)} range {np.linalg.norm(w.position):.3f}  ik_err {err * 100:.2f}cm  "
          f"jump {np.degrees(np.max(np.abs(nq - q))):.1f}deg  J6 {np.degrees(nq[5]):.1f}")
    q = nq
