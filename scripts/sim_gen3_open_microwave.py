"""Drive the Gen3 arm through the microwave door-opening arc in PyBullet.

Loads the repo's own FeedingDeploymentPyBulletSimulator (Kinova Gen3 + Robotiq
gripper, vention scene), builds the same door-opening arc geometry as
perception_interface._generate_door_arc_waypoints (reused from
draw_microwave_waypoints), and plays the arm through it.

Notes:
  * The repo's own sim cartesian controller (plan_to_ee_pose) needs a stepping /
    real-time loop and its IKFast build is unreliable on this custom URDF, so we
    drive the arm with PyBullet's native calculateInverseKinematics + resetJointState
    (instant, analytic-ish) for a clean kinematic playback.
  * The arc is anchored at the arm's current (reachable) end-effector pose rather
    than an absolute microwave location, so every waypoint is in-workspace. The
    MOTION -- a yaw sweep about a vertical hinge -- is identical to the real
    open_microwave arc; only the anchor differs. Swap in real handle/hinge poses
    (or a recorded pkl) to place it at an actual appliance.

Usage:
  python sim_gen3_open_microwave.py --mode direct   # render montage PNGs, exit
  python sim_gen3_open_microwave.py --mode gui       # watch the arm move live
"""
import argparse
import sys
import tempfile
import time
from pathlib import Path

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).parent))
from draw_microwave_waypoints import generate_door_arc_waypoints  # noqa: E402
from feeding_deployment.simulation.scene_description import (  # noqa: E402
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import (  # noqa: E402
    FeedingDeploymentPyBulletSimulator,
)
from pybullet_helpers.geometry import Pose  # noqa: E402

from scipy.spatial.transform import Rotation as R  # noqa: E402

CFG = "src/feeding_deployment/simulation/configs/vention.yaml"
ARM_JOINTS = [1, 2, 3, 4, 5, 6, 7]  # first 7 movable joints of this URDF

# Forward-facing grasp: the gripper's approach axis (EE local_z) points along
# world +y (horizontal), not down -- so it can drive the fingers onto a vertical
# handle. Anchor is a verified in-workspace, IK-reachable pose for this scene.
APPROACH = np.array([0.7, -0.7, 0.0])         # world direction the gripper points
GRASP_POS = np.array([1.45, 3.0, 0.54])       # reachable forward anchor (grasp + pre-grasp)


def forward_orientation():
    """EE quaternion whose local_z = APPROACH (forward) and local_x = world up."""
    a = APPROACH / np.linalg.norm(APPROACH)
    up = np.array([0, 0, 1.0])
    x = up - np.dot(up, a) * a
    x /= np.linalg.norm(x)
    y = np.cross(a, x)
    return tuple(R.from_matrix(np.column_stack([x, y, a])).as_quat())


def build_forward_grasp_arc():
    """Forward-facing pre-grasp/grasp + microwave-style opening arc (world frame)."""
    q = forward_orientation()
    grasp = Pose(position=tuple(GRASP_POS), orientation=q)
    a = APPROACH / np.linalg.norm(APPROACH)
    pre_grasp = Pose(position=tuple(GRASP_POS - 0.12 * a), orientation=q)  # back off along -approach
    # vertical hinge 0.28 m to the side, perpendicular to the approach in the ground plane
    perp = np.array([a[1], -a[0], 0.0])
    hinge = tuple(GRASP_POS + 0.28 * perp)
    opening = generate_door_arc_waypoints(
        start_pose=grasp, hinge_position=hinge,
        arc_length_m=0.35, waypoint_spacing_m=0.05,
        direction=1, rotate_orientation=True)  # sweep the door open
    return pre_grasp, grasp, hinge, opening


def sphere(pcid, pos, color, radius=0.012):
    vs = p.createVisualShape(p.GEOM_SPHERE, radius=radius,
                             rgbaColor=list(color) + [1], physicsClientId=pcid)
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs,
                      basePosition=list(pos), physicsClientId=pcid)


def set_arm(bid, pcid, ik_solution):
    for i, j in enumerate(ARM_JOINTS):
        p.resetJointState(bid, j, ik_solution[i], physicsClientId=pcid)


def set_fingers(r, closed):
    """Open/close the gripper. close_fingers/open_fingers reset the finger joints
    kinematically (verified: gripper joint -> ~0.8 closed, 0.0 open), so this shows
    up immediately without stepping the sim."""
    if closed:
        r.close_fingers()
    else:
        r.open_fingers()


def ik(bid, ee_link, pcid, pose):
    return p.calculateInverseKinematics(bid, ee_link, list(pose.position),
                                        list(pose.orientation), physicsClientId=pcid,
                                        maxNumIterations=200, residualThreshold=1e-4)


def snap(pcid, target, out):
    view = p.computeViewMatrix([target[0] + 0.55, target[1] - 0.8, target[2] + 0.55],
                               target, [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(55, 1024 / 768, 0.05, 4.0)
    _, _, rgb, _, _ = p.getCameraImage(1024, 768, viewMatrix=view, projectionMatrix=proj,
                                       renderer=p.ER_TINY_RENDERER, physicsClientId=pcid)
    img = np.reshape(rgb, (768, 1024, 4))[:, :, :3].astype(np.uint8)
    try:
        import cv2
        cv2.imwrite(str(out), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print("  ->", out)
    except Exception as e:  # pylint: disable=broad-except
        print("  snapshot skipped:", e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["gui", "direct"], default="direct")
    ap.add_argument("--out", type=Path, default=Path(tempfile.gettempdir()) / "gen3_microwave")
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    sd = create_scene_description_from_config(CFG, "skewer")
    sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=(args.mode == "gui"))
    r = sim.robot
    bid, pcid, ee_link = r.robot_id, r.physics_client_id, r.end_effector_id

    pre_grasp, grasp, hinge, opening = build_forward_grasp_arc()
    appr = np.array(p.getMatrixFromQuaternion(grasp.orientation)).reshape(3, 3)[:, 2]
    print(f"grasp {tuple(round(v,3) for v in grasp.position)} | "
          f"gripper approach (world) {tuple(round(v,2) for v in appr)} | "
          f"hinge {tuple(round(v,3) for v in hinge)} | {len(opening)} waypoints")

    # draw arc + hinge for context
    sphere(pcid, hinge, [0.2, 0.4, 1.0], 0.02)
    sphere(pcid, grasp.position, [1, 0, 1], 0.018)
    for wp in opening:
        sphere(pcid, wp.position, [0, 0.9, 0.3])
    target = list(grasp.position)

    def go(pose):
        set_arm(bid, pcid, ik(bid, ee_link, pcid, pose))
        got = r.get_end_effector_pose()
        ga = np.array(p.getMatrixFromQuaternion(got.orientation)).reshape(3, 3)[:, 2]
        perr = np.linalg.norm(np.array(got.position) - np.array(pose.position))
        # "tilt" = how far the approach axis is from horizontal (0 deg = level/forward).
        # The arc intentionally yaws the gripper as the door swings, so we check that
        # it stays horizontal, not that it matches the initial heading.
        tilt = np.degrees(np.arcsin(np.clip(abs(ga[2]), 0, 1)))
        return perr, tilt

    # approach with an OPEN gripper, facing forward
    set_fingers(r, closed=False)
    pe, ae = go(pre_grasp)
    print(f"  pre_grasp : pos_err {pe*1000:4.1f} mm | tilt {ae:4.1f} deg (gripper open)")
    if args.mode == "direct":
        snap(pcid, target, args.out / "0_pregrasp_open.png")

    # move onto the handle and CLOSE the gripper -> grasp confirmed
    pe, ae = go(grasp)
    set_fingers(r, closed=True)
    print(f"  grasp     : pos_err {pe*1000:4.1f} mm | tilt {ae:4.1f} deg | GRIPPER CLOSED")
    if args.mode == "direct":
        snap(pcid, target, args.out / "1_grasp_closed.png")

    # swing the door open, holding the grasp (fingers stay closed)
    errs, aerrs = [], []
    for k, wp in enumerate(opening):
        pe, ae = go(wp)
        errs.append(pe)
        aerrs.append(ae)
        if args.mode == "gui":
            time.sleep(0.35)
        elif k in (len(opening) // 2, len(opening) - 1):
            snap(pcid, target, args.out / f"2_open_wp{k:02d}.png")
    print(f"  opening arc: {len(opening)} waypoints | mean pos_err {np.mean(errs)*1000:.1f} mm "
          f"(max {np.max(errs)*1000:.1f}) | mean tilt {np.mean(aerrs):.1f} deg")

    if args.mode == "gui":
        print("GUI open; Ctrl-C or close the window to exit. Looping grasp+open ...")
        while p.isConnected(pcid):
            set_fingers(r, closed=False)
            go(pre_grasp)
            time.sleep(0.5)
            go(grasp)
            set_fingers(r, closed=True)
            time.sleep(0.4)
            for wp in opening:
                set_arm(bid, pcid, ik(bid, ee_link, pcid, wp))
                time.sleep(0.28)
            for wp in reversed(opening):
                set_arm(bid, pcid, ik(bid, ee_link, pcid, wp))
                time.sleep(0.2)


if __name__ == "__main__":
    main()
