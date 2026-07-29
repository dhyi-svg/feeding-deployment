"""Door-opening arc from the current grasp pose. Uses the repo's own arc geometry.

Hinge is the PROVEN +0.32 m assumption, NOT the perceived hinge -- the plane-fit
hinge came out wrong-side at two viewpoints (TESTING_LOG 2026-07-29) and would
sweep the door the wrong way.
"""
import sys, time
import numpy as np, pybullet as p
from pybullet_helpers.geometry import Pose, multiply_poses
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.robot_controller.command_interface import JointCommand
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

DOOR_W = 0.32          # proven hinge offset, +y from the grasp pose
ARC_LEN, SPACING = 0.55, 0.05
DIRECTION = -1         # left-hinged: handle sweeps -x, toward the arm
TRACK_ABORT = 0.025    # per-waypoint EE tracking abort (door bind / latch / limit)
MAX_IK_ERR = 0.02
MAX_JUMP_DEG = 45.0    # per step; a big jump is a wrist flip
PAUSE_S = 1.0
ARM = [1,2,3,4,5,6,7]

ai = ArmInterfaceClient(); st = ai.get_state()
ee = list(st["ee_pos"]); start = Pose(tuple(ee[:3]), tuple(ee[3:7]))
q0 = np.asarray(st["position"], dtype=float)
g = float(st.get("gripper_pos"))
print(f"grasp pose {np.round(ee[:3],4)}  gripper {g:.4f}")
if g < 0.2:
    sys.exit("Gripper is OPEN -- nothing grasped. Refusing to run the arc.")

hinge = (start.position[0], start.position[1] + DOOR_W, start.position[2])
wps = PerceptionInterface._generate_door_arc_waypoints(
    None, start_pose=start, hinge_position=hinge, arc_length_m=ARC_LEN,
    waypoint_spacing_m=SPACING, direction=DIRECTION, rotate_orientation=True)
print(f"hinge {np.round(hinge,3)}  -> {len(wps)} waypoints, arc {ARC_LEN} m, dir {DIRECTION}")

scene = create_scene_description_from_config(
    "src/feeding_deployment/simulation/configs/vention.yaml","skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False); rb = sim.robot

def ik(pose, seed):
    for i,jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(seed[i]), physicsClientId=rb.physics_client_id)
    w = multiply_poses(scene.robot_base_pose, pose)
    sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id, list(w.position),
        list(w.orientation), maxNumIterations=400, residualThreshold=1e-5,
        physicsClientId=rb.physics_client_id)
    q = np.asarray(sol[:7])
    for i,jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[i]), physicsClientId=rb.physics_client_id)
    ls = p.getLinkState(rb.robot_id, rb.end_effector_id, physicsClientId=rb.physics_client_id)
    return q, float(np.linalg.norm(np.asarray(ls[4]) - np.asarray(w.position)))

# Plan every waypoint first: seeded IK, falling back to unseeded when it stalls
# (seeding is too conservative mid-arc -- TESTING_LOG 2026-07-14 caveat).
plan, seed = [], q0
neutral = np.zeros(7)
for i, wp in enumerate(wps, 1):
    q, err = ik(wp, seed); how = "seeded"
    if err > MAX_IK_ERR:
        q2, err2 = ik(wp, neutral)
        if err2 < err: q, err, how = q2, err2, "unseeded"
    jump = float(np.degrees(np.max(np.abs(q - seed))))
    ok = err <= MAX_IK_ERR and jump <= MAX_JUMP_DEG
    print(f"  wp{i:02d} {np.round(wp.position,3)}  ik {err*100:5.2f}cm {how:8s} jump {jump:5.1f}deg {'OK' if ok else 'STOP-PLAN-HERE'}")
    if not ok:
        print(f"  -> planning stops at wp{i-1}; will execute {len(plan)} waypoints")
        break
    plan.append((i, wp, q)); seed = q

if not plan: sys.exit("No feasible waypoints -- refusing.")
print(f"\nexecuting {len(plan)} waypoints at speed={ai.get_speed()} ...")
for i, wp, q in plan:
    ai.execute_command(JointCommand(pos=q.tolist()))
    for _ in range(150):
        time.sleep(0.2)
        s = ai.get_state()
        if float(np.max(np.abs(np.asarray(s["velocity"], dtype=float)))) < 1e-3: break
    time.sleep(0.4)
    s = ai.get_state()
    fin = np.asarray(list(s["ee_pos"])[:3], dtype=float)
    e = float(np.linalg.norm(fin - np.asarray(wp.position)))
    print(f"  wp{i:02d} EE {np.round(fin,4)} tracking {e*100:5.1f}cm gripper {float(s.get('gripper_pos')):.3f}")
    if e > TRACK_ABORT:
        print(f"  TRACKING ABORT at wp{i} ({e*100:.1f} cm) -- natural stop (bind/latch/limit). Holding.")
        break
    time.sleep(PAUSE_S)

s = ai.get_state(); fin = np.asarray(list(s["ee_pos"])[:3], dtype=float)
print(f"\nfinal EE {np.round(fin,4)}  gripper {float(s.get('gripper_pos')):.3f}")
print(f"handle swept {np.linalg.norm(fin - np.asarray(start.position))*100:.1f} cm from the grasp pose")
