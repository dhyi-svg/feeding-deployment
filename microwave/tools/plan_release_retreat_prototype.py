# SUPERSEDED by microwave_common.py (add_door_model / clearance / release_and_back_off) -- kept
# as the 2026-09-23 prototype. Hinge/grasp constants below are that afternoon's values.
# Usage: python3 microwave/tools/plan_release_retreat_prototype.py <out.json> [yaw_deg] [back_off_m]
"""PLAN ONLY -- never commands the arm. Release + retreat from a grasped, swung-open
microwave door, collision-checked in PyBullet against a conservative door model.

Door model (closed-door frame, arm_base_link): face plane at the fingertips' depth at
grasp time (the fingers demonstrably did not penetrate the door, so this is the closest
the face can be), 4 cm thick, y from free edge to hinge edge (detector: hinge edge
35 cm from handle, span 47 cm), z 0.25-0.80 (unmeasured, deliberately generous).
The door slab is then rotated about the vertical hinge axis by the measured swing angle
(initial grasp point -> current gripper point). The microwave body behind the closed
door face stays put.
"""
import json, sys
import numpy as np, pybullet as p
from scipy.spatial.transform import Rotation as R
from pybullet_helpers.geometry import Pose, multiply_poses
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator

HINGE = np.array([0.724, 0.274, 0.5425])
GRASP_XY0 = np.array([0.655, -0.0689])     # tool_frame right after today's grasp (door closed)
HANDLE_Y0 = -0.0722                           # detected handle y, door closed
FREE_EDGE_PAST_HANDLE = 0.122                 # span 47.2 - 35.0 cm
DOOR_T, Z_LO, Z_HI = 0.04, 0.25, 0.80
ARM = [1, 2, 3, 4, 5, 6, 7]
FINGER_JOINTS = [12, 14, 16, 17, 19, 21]
MIN_CLEAR = 0.03                              # plan must keep every link >= 3 cm from the door
J6_GUARD = 115.0
MAX_JUMP = 25.0

st = ArmInterfaceClient().get_state()
q0 = np.array(st["position"], float)
ee = np.array(st["ee_pos"], float)
pos0, quat0 = ee[:3], ee[3:7]
approach = R.from_quat(quat0).as_matrix()[:, 2]
out = -approach.copy(); out[2] = 0; out /= np.linalg.norm(out)

scene = create_scene_description_from_config("src/feeding_deployment/simulation/configs/vention.yaml", "skewer")
sim = FeedingDeploymentPyBulletSimulator(scene, use_gui=False)
rb, c = sim.robot, sim.robot.physics_client_id
base = scene.robot_base_pose

def set_q(q):
    for j, jj in enumerate(ARM):
        p.resetJointState(rb.robot_id, jj, float(q[j]), physicsClientId=c)
    for jj in FINGER_JOINTS:                       # gripper OPEN for the whole retreat
        p.resetJointState(rb.robot_id, jj, 0.0, physicsClientId=c)

def link_pos(link):
    return np.array(p.getLinkState(rb.robot_id, link, physicsClientId=c)[4])

def to_world(xyz):
    return np.array(multiply_poses(base, Pose(tuple(xyz), (0, 0, 0, 1))).position)

# --- where are the fingertips at grasp time? (current joints, rotated back to closed frame)
set_q(q0)
tip_world = link_pos(23)
tip_base = tip_world - np.array(base.position)          # base has identity rotation
theta = np.arctan2(*(pos0[:2] - HINGE[:2])[::-1]) - np.arctan2(*(GRASP_XY0 - HINGE[:2])[::-1])
def rot_about_hinge(xy, ang):
    d = xy - HINGE[:2]; c_, s_ = np.cos(ang), np.sin(ang)
    return HINGE[:2] + np.array([c_ * d[0] - s_ * d[1], s_ * d[0] + c_ * d[1]])
tip_closed = rot_about_hinge(tip_base[:2], -theta)
face_x = tip_closed[0] + 0.005
print(f"swing angle {np.degrees(theta):.1f} deg; fingertip (closed frame) x {tip_closed[0]:.3f} -> door face x {face_x:.3f}")

# --- door slab (rotated) + microwave body (fixed), as collision-only bodies
y_lo, y_hi = HANDLE_Y0 - FREE_EDGE_PAST_HANDLE, HINGE[1]
door_c_closed = np.array([face_x + DOOR_T / 2, (y_lo + y_hi) / 2])
door_c = rot_about_hinge(door_c_closed, theta)
half = [DOOR_T / 2, (y_hi - y_lo) / 2, (Z_HI - Z_LO) / 2]
door = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half, physicsClientId=c),
    basePosition=to_world([door_c[0], door_c[1], (Z_LO + Z_HI) / 2]),
    baseOrientation=R.from_euler("z", theta).as_quat(), physicsClientId=c)
body_x0 = face_x + DOOR_T
body = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX,
    halfExtents=[0.20, (y_hi - y_lo) / 2, (Z_HI - Z_LO) / 2], physicsClientId=c),
    basePosition=to_world([body_x0 + 0.20, (y_lo + y_hi) / 2, (Z_LO + Z_HI) / 2]), physicsClientId=c)
free_edge = rot_about_hinge(np.array([face_x, y_lo]), theta)
print(f"door free edge now at ({free_edge[0]:.3f}, {free_edge[1]:.3f}); outward direction {np.round(out, 3)}")

def clearance(q):
    set_q(q); best = (9.0, "")
    for obs, name in ((door, "door"), (body, "microwave body")):
        for pt in p.getClosestPoints(rb.robot_id, obs, 0.5, physicsClientId=c):
            if pt[8] < best[0]:
                ln = p.getJointInfo(rb.robot_id, pt[3], physicsClientId=c)[12].decode() if pt[3] >= 0 else "base"
                best = (pt[8], f"{ln} vs {name}")
    return best

# Null-space IK with the real Gen3 limits (J2/J4/J6 are the limited ones; J6 capped at the
# guard, not the hard limit) and a rest pose = the seed, so it prefers staying nearby.
NDOF = sum(1 for i in range(p.getNumJoints(rb.robot_id, physicsClientId=c))
           if p.getJointInfo(rb.robot_id, i, physicsClientId=c)[2] != p.JOINT_FIXED)
LIM = {1: 128.9, 3: 147.8, 5: J6_GUARD - 1.0, 6: 175.0}   # J7 capped: never cross +-180 (wrap direction on the arm unverified)
def ik(pos, quat, seed):
    set_q(seed)
    w = multiply_poses(base, Pose(tuple(pos), tuple(quat)))
    lo = [-6.3] * NDOF; hi = [6.3] * NDOF
    for j, deg in LIM.items(): lo[j], hi[j] = -np.radians(deg), np.radians(deg)
    rest = list(seed) + [0.0] * (NDOF - 7)
    sol = p.calculateInverseKinematics(rb.robot_id, rb.end_effector_id, list(w.position), list(w.orientation),
        physicsClientId=c, maxNumIterations=200)
    q = np.array(sol[:7]); set_q(q)
    return q, float(np.linalg.norm(link_pos(rb.end_effector_id) - np.array(w.position)))

print(f"start clearance (gripper opened in place): {clearance(q0)[0] * 100:.1f} cm ({clearance(q0)[1]})")

A_LEN = float(sys.argv[3]) if len(sys.argv) > 3 else 0.12
legs = [(f"A retreat straight out {A_LEN * 100:.0f} cm", out * A_LEN),
        ("B out 12 cm more + up 8 cm, turning", out * 0.12 + np.array([0, 0, 0.08]))]
YAW = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
q, cur, plan, ok = q0, pos0.copy(), [], True
for name, delta in legs:
    n = max(2, int(np.ceil(np.linalg.norm(delta) / 0.02)))
    leg_min = (9.0, "")
    print(f"\n{name}: {n} steps")
    for k in range(1, n + 1):
        tgt = cur + delta * k / n
        # leg A is a pure straight back-off (fingers slide off the vertical bar the way
        # they went on); the yaw that unwinds J6 ramps in over leg B only.
        frac = 0.0 if name.startswith("A") else k / n
        quat = (R.from_euler("z", np.radians(YAW * frac)) * R.from_quat(quat0)).as_quat()
        nq, err = ik(tgt, quat, q)
        jump = float(np.degrees(np.max(np.abs(nq - q)))); j6 = float(np.degrees(nq[5]))
        worst = min((clearance(q + (nq - q) * s / 4) for s in range(1, 5)), key=lambda t: t[0])
        leg_min = min(leg_min, worst, key=lambda t: t[0])
        flag = []
        if err > 0.01: flag.append("IK")
        if jump > MAX_JUMP: flag.append("JUMP")
        if abs(j6) > J6_GUARD: flag.append("J6")
        if worst[0] < MIN_CLEAR: flag.append("CLEARANCE")
        if abs(np.degrees(nq[6])) > 176: flag.append("J7")
        print(f"  {k}: {np.round(tgt, 3)} ik {err * 100:.2f}cm jump {jump:.1f} J6 {j6:.1f} "
              f"clear {worst[0] * 100:.1f}cm ({worst[1]}) {'  <-- ' + ','.join(flag) if flag else ''}")
        ok &= not flag
        plan.append({"leg": name, "pos": tgt.tolist(), "quat": list(quat), "joints": nq.tolist()})
        q = nq
    cur = cur + delta
    print(f"  leg min clearance {leg_min[0] * 100:.1f} cm ({leg_min[1]})")

json.dump(plan, open(sys.argv[1] if len(sys.argv) > 1 else "/dev/null", "w"), indent=1)
print("\nPLAN", "PASSES all gates" if ok else "FAILS a gate -- do not execute", "(nothing commanded)")
