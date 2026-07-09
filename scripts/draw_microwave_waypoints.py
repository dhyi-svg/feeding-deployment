"""Draw microwave door-opening waypoints in a PyBullet scene.

Standalone (no ROS / rviz / executive). It reproduces the repo's own arc
geometry -- perception_interface.PerceptionInterface._generate_door_arc_waypoints
(copied verbatim below; that method uses no `self`) -- to build the microwave
opening/push waypoints, draws them as coordinate frames + a path polyline, and:
  * renders a screenshot (works headless, DIRECT mode), and
  * writes a schema-correct handle_opening_pos.pkl template.

Frame: arm_base_link (x forward, y left, z up). Door rotates in the xy plane
about a vertical hinge -- exactly the assumption the real method makes.

Usage:
  python draw_microwave_waypoints.py --mode direct   # render PNG + pkl, exit
  python draw_microwave_waypoints.py --mode gui       # interactive window (spins)
"""
import argparse
import pickle
import tempfile
import time
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data
from pybullet_helpers.geometry import Pose
from scipy.spatial.transform import Rotation as R

# Generated artifacts (pkl template + screenshot) go to a temp dir by default so
# they never dirty the tracked scripts/ folder; override with --out.
OUT = Path(tempfile.gettempdir()) / "microwave_waypoints"


# --- verbatim copy of perception_interface._generate_door_arc_waypoints ------
def generate_door_arc_waypoints(start_pose, hinge_position, arc_length_m,
                                waypoint_spacing_m, direction=1,
                                rotate_orientation=True):
    if arc_length_m <= 0:
        return []
    if waypoint_spacing_m <= 0:
        raise ValueError("waypoint_spacing_m must be > 0")
    if direction not in (-1, 1):
        raise ValueError("direction must be either +1 or -1")
    hx, hy, hz = start_pose.position
    cx, cy, _ = hinge_position
    radius_vec = np.array([hx - cx, hy - cy], dtype=float)
    radius = np.linalg.norm(radius_vec)
    if radius < 1e-8:
        raise ValueError("Handle pose is too close to hinge pose; radius is ~0.")
    start_theta = np.arctan2(radius_vec[1], radius_vec[0])
    total_angle = arc_length_m / radius
    num_segments = max(1, int(np.ceil(arc_length_m / waypoint_spacing_m)))
    start_rot = R.from_quat(start_pose.orientation)
    waypoints = []
    for i in range(1, num_segments + 1):
        frac = i / num_segments
        delta_angle = direction * frac * total_angle
        theta = start_theta + delta_angle
        x = cx + radius * np.cos(theta)
        y = cy + radius * np.sin(theta)
        z = hz
        if rotate_orientation:
            orientation = (R.from_euler("z", delta_angle) * start_rot).as_quat()
        else:
            orientation = start_pose.orientation
        waypoints.append(Pose(position=(x, y, z), orientation=orientation))
    return waypoints
# ----------------------------------------------------------------------------


def build_microwave_poses():
    """Plausible left-hinged microwave, ~0.30 m door, ~0.30 m off the base.

    EDIT hinge/grasp here (or replace with values from a real recording) to
    match your microwave. Arc params below mirror perceive_handle_opening_poses
    for handle_type == "microwave".
    """
    # gripper facing the door: point tool +z toward -x (into the door front)
    grasp_orient = R.from_euler("z", np.pi).as_quat()
    grasp_pose = Pose(position=(0.50, 0.15, 0.30), orientation=grasp_orient)
    hinge_pos = (0.50, -0.15, 0.30)              # left-hinged (to the -y side)

    opening = generate_door_arc_waypoints(
        start_pose=grasp_pose, hinge_position=hinge_pos,
        arc_length_m=0.55, waypoint_spacing_m=0.05,
        direction=-1, rotate_orientation=True)   # microwave is left hinged

    push_start = opening[-6]
    push = generate_door_arc_waypoints(
        start_pose=push_start, hinge_position=hinge_pos,
        arc_length_m=0.50, waypoint_spacing_m=0.05,
        direction=-1, rotate_orientation=True)

    pre_grasp = Pose(position=(0.38, 0.15, 0.30), orientation=grasp_orient)
    post_release = Pose(position=opening[-1].position, orientation=opening[-1].orientation)
    return {
        "grasp_pose": grasp_pose, "pre_grasp_pose": pre_grasp,
        "opening_waypoints": opening, "post_release_pose": post_release,
        "push_pose": push_start, "pre_push_pose": push_start, "push_waypoints": push,
        "_hinge_position": hinge_pos,   # not a real key; kept for drawing only
    }


def frame_lines(pose, length=0.06):
    """Return (start, end, color) triples for the pose's x/y/z axes."""
    o = np.array(pose.position)
    m = np.array(p.getMatrixFromQuaternion(pose.orientation)).reshape(3, 3)
    return [(o, o + length * m[:, 0], [1, 0, 0]),
            (o, o + length * m[:, 1], [0, 1, 0]),
            (o, o + length * m[:, 2], [0, 0, 1])]


def sphere(pos, color, radius=0.012):
    """Real body (captured by getCameraImage, unlike debug lines)."""
    vs = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=list(color) + [1])
    p.createMultiBody(baseMass=0, baseVisualShapeIndex=vs, basePosition=list(pos))


def draw(poses):
    # arm_base_link frame at origin: debug axes (GUI) + a gray sphere (image)
    for s, e, c in frame_lines(Pose(position=(0, 0, 0), orientation=(0, 0, 0, 1)), 0.15):
        p.addUserDebugLine(s, e, c, lineWidth=3)
    p.addUserDebugText("arm_base_link", [0, 0, 0.02], [1, 1, 1], textSize=1.2)
    sphere((0, 0, 0), [0.5, 0.5, 0.5], 0.02)

    hinge = poses["_hinge_position"]
    p.addUserDebugLine([hinge[0], hinge[1], hinge[2] - 0.2],
                       [hinge[0], hinge[1], hinge[2] + 0.2], [0.2, 0.4, 1.0], lineWidth=4)
    p.addUserDebugText("hinge", [hinge[0], hinge[1], hinge[2] + 0.22], [0.4, 0.6, 1.0])
    sphere(hinge, [0.2, 0.4, 1.0], 0.02)

    def draw_arc(wps, color, label):
        prev = None
        for wp in wps:
            for s, e, c in frame_lines(wp):          # per-waypoint orientation (GUI)
                p.addUserDebugLine(s, e, c, lineWidth=2)
            sphere(wp.position, color)               # position (image + GUI)
            if prev is not None:
                p.addUserDebugLine(prev, wp.position, color, lineWidth=3)
            prev = np.array(wp.position)
        if wps:
            p.addUserDebugText(label, wps[len(wps) // 2].position, color)

    # grasp start (magenta) then the two arcs
    p.addUserDebugText("grasp", poses["grasp_pose"].position, [1, 0, 1])
    sphere(poses["grasp_pose"].position, [1, 0, 1], 0.018)
    draw_arc(poses["opening_waypoints"], [0, 0.9, 0.3], "opening arc")  # green
    draw_arc(poses["push_waypoints"], [1.0, 0.6, 0.0], "push arc")      # orange


def save_pkl_template(poses):
    d = {k: v for k, v in poses.items() if not k.startswith("_")}
    # complete the schema with valid placeholders so the file loads as a template
    z = poses["opening_waypoints"][-1]
    for key in ["placement_pose", "behind_placement_pose", "before_above_closing_waypoint",
                "above_closing_waypoint", "closing_waypoint", "pull_closing_waypoint",
                "pre_pull_pose", "behind_pull_closing_waypoint", "above_pull_closing_waypoint",
                "above_push_closing_waypoint"]:
        d.setdefault(key, z)
    for key in ["closing_waypoints", "offset_closing_waypoints",
                "pull_closing_waypoints", "push_closing_waypoints"]:
        d.setdefault(key, list(reversed(poses["opening_waypoints"])))
    out = OUT / "handle_opening_pos.pkl"
    with open(out, "wb") as f:
        pickle.dump({"last_handle_poses": d}, f)
    return out, d


def main():
    global OUT  # pylint: disable=global-statement
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["gui", "direct"], default="direct")
    ap.add_argument("--out", type=Path, default=OUT,
                    help="dir for the pkl template + screenshot (default: a temp dir)")
    args = ap.parse_args()

    OUT = args.out
    OUT.mkdir(parents=True, exist_ok=True)

    poses = build_microwave_poses()
    pkl_path, d = save_pkl_template(poses)
    print(f"opening_waypoints: {len(poses['opening_waypoints'])} | "
          f"push_waypoints: {len(poses['push_waypoints'])}")
    print(f"pkl template ({len(d)} keys) -> {pkl_path}")

    try:
        p.connect(p.GUI if args.mode == "gui" else p.DIRECT)
    except Exception:  # pylint: disable=broad-except
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.resetDebugVisualizerCamera(cameraDistance=1.1, cameraYaw=50,
                                 cameraPitch=-35, cameraTargetPosition=[0.45, 0, 0.3])
    draw(poses)

    target = [0.5, -0.05, 0.3]
    view = p.computeViewMatrix(cameraEyePosition=[1.15, -0.75, 0.9],
                               cameraTargetPosition=target, cameraUpVector=[0, 0, 1])
    proj = p.computeProjectionMatrixFOV(fov=55, aspect=1024 / 768, nearVal=0.05, farVal=3.0)
    w, h, rgb, _, _ = p.getCameraImage(1024, 768, viewMatrix=view, projectionMatrix=proj,
                                       renderer=p.ER_BULLET_HARDWARE_OPENGL
                                       if args.mode == "gui" else p.ER_TINY_RENDERER)
    img = np.reshape(rgb, (h, w, 4))[:, :, :3].astype(np.uint8)
    png = OUT / "microwave_waypoints.png"
    try:
        import cv2
        cv2.imwrite(str(png), cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
        print(f"screenshot -> {png}")
    except Exception as e:  # pylint: disable=broad-except
        print("screenshot skipped:", e)

    if args.mode == "gui":
        print("GUI open; close the window or Ctrl-C to exit.")
        while p.isConnected():
            p.stepSimulation()
            time.sleep(1 / 60)


if __name__ == "__main__":
    main()
