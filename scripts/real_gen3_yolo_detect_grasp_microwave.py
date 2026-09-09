"""Detect the microwave handle and grasp it on the REAL Gen3 -- YOLO11 variant.

Sibling to `real_gen3_detect_grasp_microwave.py` (the GroundingDINO version):
identical pipeline and constants, with ONLY the appliance-box detector swapped.
Companion to `real_gen3_open_microwave.py` (the door-arc runner): run this
FIRST to get the arm gripping the handle, then hand off to the arc script.

Written 2026-09-04 on a dev machine with no camera/arm/`ultralytics` available
-- syntax-checked only, NOT yet run against a real microwave/arm. Needs a real
run on the Jetson or Pachirisu rig to validate detection quality and whether
dropping the forced-CPU workaround (see below) is actually safe there.

Why YOLO11 instead of GroundingDINO: GroundingDINO is an open-vocabulary
text-prompted detector (CPU-only on the Jetson -- it hits an NVML assert on
the Tegra iGPU) and needed for the *handle*-level text prompt used elsewhere
in this repo. But this script's own detection target has always been the
whole **microwave** appliance (see `detect_microwave_box` below) -- and COCO
already has a "microwave" class (id 68), so a plain closed-set pretrained
Ultralytics YOLO11 model can detect it directly, no text prompt, no SAM stack,
and (unlike GroundingDINO on this rig) no known CPU-only constraint -- lighter
and likely faster. Everything downstream of the box (plane-fit + DBSCAN
protruding-cluster handle-centroid, eye-in-hand calib, grasp) is UNCHANGED --
see `real_gen3_detect_grasp_microwave.py`'s own docstring for how that logic
was cross-checked against the repo's `appliance_perception.py` and
`TESTING_LOG.md`.

Pipeline (all local, no ROS, no SAM, no tf2):
  1. Grab one RGB+D frame from the wrist-mounted RealSense (pyrealsense2).
  2. YOLO11 (Ultralytics, pretrained on COCO) detects class 68 ("microwave")
     -> a box. Device is whatever Ultralytics picks automatically (GPU if
     available, else CPU) -- there is no known reason to force CPU here, but
     that has not been verified on the Jetson's Tegra iGPU yet.
  3. Deproject the box's pixels to camera-frame 3D points (pinhole, using the
     frame's own intrinsics -- no camera_info topic).
  4. open3d segment_plane fits the door's main planar surface; points in front
     of it (0-7cm) are the "protruding" candidates.
  5. DBSCAN clusters the protruding points; the largest cluster's median is the
     handle centroid (y overridden to top_most_y - 0.04, matching the
     microwave case in detect_handle_and_placement).
  6. Empirical corrections (from teleop ground-truth, baked into the
     GroundingDINO version's 2026-07-14 session -- carried over unchanged;
     re-validate once this detector has real runs on the rig):
       DEPTH_CORR=0.16 -- perception overestimates depth ~16cm; scale the
                          handle ray in by 16cm (camera origin -> point vector
                          shortened proportionally, not just a z-shift).
       LAT_CORR=0.07   -- protruding-cluster centroid sits ~7cm off the latch;
                          shift -y (camera frame).
  7. camera-frame point -> arm-base frame via the easy_handeye2 eye-in-hand
     calib (~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib --
     static end_effector_link -> camera_color_optical_frame transform) chained
     with the live get_state() EE pose (base_link -> end_effector_link from
     the real arm's current joints). This replaces tf2 entirely.
  8. Fixed grasp orientation (the repo's own constant, appliance_perception.py
     detect_handle_and_placement); grasp EE = handle - GRIP_EXT (0.065m) along
     the local approach (-z) axis; pre-grasp = handle - (GRIP_EXT + 0.10m).
  9. open_gripper -> move to pre-grasp (seeded IK: a big single jump) -> step
     in over 4 sub-steps toward the grasp pose (seeded IK each step; abort if
     tracking error > 2.5cm -- door binding / unexpected contact) ->
     close_gripper -> PAUSE for a human grip check before running the arc
     script.

Needs `ultralytics` installed (not in pyproject.toml -- this repo installs
perception-model deps ad hoc, same as groundingdino-py/supervision; see
CLAUDE.md/TESTING_LOG.md):
  .venv/bin/pip install ultralytics

Run from the repo root, arm already bearing on the microwave handle in frame,
with the arm server + bulldog bypass already up (see TESTING_LOG.md's
"Setup -- talking to the arm" section):

  PYTHONPATH=$HOME/.local/lib/python3.10/site-packages ARM_RPC_HOST=127.0.0.1 \\
      .venv/bin/python scripts/real_gen3_yolo_detect_grasp_microwave.py
"""
import os

import numpy as np
import open3d as o3d
import pyrealsense2 as rs
import pybullet as p
import yaml
from pybullet_helpers.geometry import Pose, multiply_poses
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN
from ultralytics import YOLO

from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator
from feeding_deployment.control.robot_controller.arm_interface import (
    ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY,
)

CALIB_PATH = os.path.expanduser("~/.ros2/easy_handeye2/calibrations/wrist_camera_calib.calib")
DEPTH_CORR = 0.16   # perception overestimates depth ~16cm; scale the handle ray in by this
LAT_CORR = 0.07     # protruding-cluster centroid sits ~7cm off the latch; shift -y
GRIP_EXT = 0.065     # grasp EE = handle - 6.5cm along approach (was 9cm; 6.5 is the
                     # proven "grip a bit more forward -> firmer" fix)
PRE_GRASP_EXTRA = 0.10  # pre-grasp is GRIP_EXT + this much further back
N_SUBSTEPS = 4
SUBSTEP_ABORT_M = 0.025  # 2.5cm tracking-error abort
# Fixed grasp orientation -- the repo's OWN constant, not reconstructed
# (appliance_perception.py:627-629, detect_handle_and_placement).
GRASP_QUAT = (-0.5, 0.5, 0.5, -0.5)

# YOLO11 detection config. COCO class 68 is "microwave" -- closed-set, no text
# prompt needed (unlike GroundingDINO's "microwave" class-name prompt).
YOLO_MODEL = "yolo11n.pt"   # nano; swap to yolo11s.pt/yolo11m.pt if accuracy is too low
COCO_MICROWAVE_CLASS_ID = 68
YOLO_CONF_THRESHOLD = 0.3   # matches the GroundingDINO version's BOX_THRESHOLD default

CFG = "src/feeding_deployment/simulation/configs/vention.yaml"
ARM_JOINTS = [1, 2, 3, 4, 5, 6, 7]


def capture_rgbd():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(config)
    for _ in range(15):  # let auto-exposure settle
        pipeline.wait_for_frames()
    frames = pipeline.wait_for_frames(5000)
    color = frames.get_color_frame()
    depth = frames.get_depth_frame()
    intr = color.profile.as_video_stream_profile().intrinsics
    color_img = np.asanyarray(color.get_data())
    depth_img = np.asanyarray(depth.get_data()).astype(np.float32) / 1000.0  # mm -> m
    pipeline.stop()
    return color_img, depth_img, intr


def detect_microwave_box(bgr_image):
    """YOLO11 (Ultralytics, pretrained COCO) detect -> highest-confidence
    [x1,y1,x2,y2] box for class 68 ("microwave")."""
    model = YOLO(YOLO_MODEL)
    results = model.predict(
        bgr_image,
        classes=[COCO_MICROWAVE_CLASS_ID],
        conf=YOLO_CONF_THRESHOLD,
        verbose=False,
    )
    boxes = results[0].boxes
    if boxes is None or len(boxes) == 0:
        raise RuntimeError("No microwave detection.")
    best = int(boxes.conf.argmax())
    return boxes.xyxy[best].cpu().numpy()


def pixel2world(u, v, depth_m, fx, fy, cx, cy):
    return np.array([(depth_m / fx) * (u - cx), (depth_m / fy) * (v - cy), depth_m])


def find_handle_centroid_camera_frame(bgr_image, depth_img, intr, box):
    """Repo's own logic (detect_handle_and_placement), ROS-free: deproject the
    box -> segment_plane -> protruding-cluster DBSCAN -> centroid."""
    x1, y1, x2, y2 = box.astype(int)
    fx, fy, cx, cy = intr.fx, intr.fy, intr.ppx, intr.ppy

    pts_3d, pixels = [], []
    for v in range(y1, y2):
        for u in range(x1, x2):
            d = float(depth_img[v, u])
            if 0.05 < d < 2.0:
                pts_3d.append(pixel2world(u, v, d, fx, fy, cx, cy))
                pixels.append((u, v))
    if not pts_3d:
        raise RuntimeError("No valid 3D points from detection box.")
    pts_3d = np.array(pts_3d)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts_3d)
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.02, ransac_n=3, num_iterations=500)
    outliers = np.setdiff1d(np.arange(len(pts_3d)), inliers)
    plane_points = pts_3d[inliers]
    plane_depth = float(np.median(plane_points[:, 2]))

    protruding = np.array([p for p in pts_3d[outliers] if plane_depth - 0.07 < p[2] < plane_depth])
    if len(protruding) == 0:
        raise RuntimeError("No protruding (handle) points in front of the door plane.")

    labels = DBSCAN(eps=0.02, min_samples=50).fit(protruding).labels_
    valid = labels >= 0
    if not np.any(valid):
        raise RuntimeError("DBSCAN found no clusters -- no handle.")
    unique, counts = np.unique(labels[valid], return_counts=True)
    cluster = protruding[labels == unique[np.argmax(counts)]]

    centroid = np.median(cluster, axis=0)
    centroid[1] = np.max(cluster[:, 1]) - 0.04  # microwave case (see detect_handle_and_placement)
    return centroid


def load_eye_in_hand_calib(path):
    """easy_handeye2 .calib -> Pose (end_effector_link -> camera_color_optical_frame)."""
    with open(path) as f:
        d = yaml.safe_load(f)
    t, r = d["transform"]["translation"], d["transform"]["rotation"]
    return Pose((t["x"], t["y"], t["z"]), (r["x"], r["y"], r["z"], r["w"]))


def solve_ik_seeded(rb, sd, target_pose, ai):
    """Seed sim joints from the arm's CURRENT real joints before IK -- correct
    for a big single jump (pre-grasp / each sub-step here), per the seeded-IK
    fix in TESTING_LOG.md (prevents a far-away/flipped IK solution)."""
    cur = ai.get_state()["position"]
    for i, jj in enumerate(ARM_JOINTS):
        p.resetJointState(rb.robot_id, jj, cur[i], physicsClientId=rb.physics_client_id)
    wpose = multiply_poses(sd.robot_base_pose, target_pose)
    sol = p.calculateInverseKinematics(
        rb.robot_id, rb.end_effector_id, list(wpose.position), list(wpose.orientation),
        physicsClientId=rb.physics_client_id, maxNumIterations=200)
    return [sol[k] for k in range(7)]


def move_and_check(ai, target_pos, joint_target, abort_m):
    ai.set_joint_position(joint_target)
    for _ in range(60):
        stt = ai.get_state()
        if (np.linalg.norm(np.array(stt["ee_pos"][:3]) - np.array(target_pos)) < 0.02
                and max(abs(x) for x in stt["velocity"]) < 0.01):
            break
    got = np.array(ai.get_state()["ee_pos"][:3])
    err = float(np.linalg.norm(got - np.array(target_pos)))
    print(f"  reached {np.round(got, 3)} | tracking err {err * 100:.1f}cm")
    if err > abort_m:
        raise RuntimeError(f"ABORT: tracking err {err * 100:.1f}cm > {abort_m * 100:.0f}cm -- "
                            "unexpected contact / bad detection. Not advancing.")


def main():
    ArmManager.register("ArmInterface")
    mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    mg.connect()
    ai = mg.ArmInterface()
    ai.set_speed("low")

    print("Capturing RGB-D frame ...")
    color_img, depth_img, intr = capture_rgbd()

    print("Detecting microwave (YOLO11) ...")
    box = detect_microwave_box(color_img)
    centroid_cam = find_handle_centroid_camera_frame(color_img, depth_img, intr, box)
    print(f"  raw handle centroid (camera frame): {np.round(centroid_cam, 3)}")

    # Empirical corrections (camera frame).
    ray_scale = (centroid_cam[2] - DEPTH_CORR) / centroid_cam[2]
    centroid_cam = centroid_cam * ray_scale
    centroid_cam[1] -= LAT_CORR
    print(f"  corrected handle centroid (camera frame): {np.round(centroid_cam, 3)}")

    ee = list(ai.get_state()["ee_pos"])
    ee_pose_base = Pose(tuple(ee[:3]), tuple(ee[3:7]))  # base_link -> end_effector_link (live FK)
    effector_to_camera = load_eye_in_hand_calib(CALIB_PATH)
    base_to_camera = multiply_poses(ee_pose_base, effector_to_camera)
    handle_pos_base = multiply_poses(
        base_to_camera, Pose(tuple(centroid_cam), (0.0, 0.0, 0.0, 1.0))
    ).position
    print(f"  handle position (arm-base frame): {np.round(handle_pos_base, 3)}")

    handle_pose = Pose(tuple(handle_pos_base), GRASP_QUAT)
    grasp_pose = multiply_poses(handle_pose, Pose((0.0, 0.0, -GRIP_EXT), (0.0, 0.0, 0.0, 1.0)))
    pre_grasp_pose = multiply_poses(
        handle_pose, Pose((0.0, 0.0, -(GRIP_EXT + PRE_GRASP_EXTRA)), (0.0, 0.0, 0.0, 1.0))
    )

    sd = create_scene_description_from_config(CFG, "skewer")
    sim = FeedingDeploymentPyBulletSimulator(sd, use_gui=False)
    rb = sim.robot

    print("Opening gripper ...")
    ai.open_gripper()

    print(f"Moving to pre-grasp {np.round(pre_grasp_pose.position, 3)} (seeded IK, single jump) ...")
    joints = solve_ik_seeded(rb, sd, pre_grasp_pose, ai)
    move_and_check(ai, pre_grasp_pose.position, joints, abort_m=SUBSTEP_ABORT_M)

    print(f"Stepping in to grasp {np.round(grasp_pose.position, 3)} over {N_SUBSTEPS} sub-steps ...")
    start = np.array(pre_grasp_pose.position)
    end = np.array(grasp_pose.position)
    for s in range(1, N_SUBSTEPS + 1):
        sub_pos = start + (end - start) * (s / N_SUBSTEPS)
        sub_pose = Pose(tuple(sub_pos), GRASP_QUAT)
        joints = solve_ik_seeded(rb, sd, sub_pose, ai)
        print(f"  sub-step {s}/{N_SUBSTEPS} -> {np.round(sub_pos, 3)}")
        move_and_check(ai, sub_pos, joints, abort_m=SUBSTEP_ABORT_M)

    print("Closing gripper ...")
    ai.close_gripper()

    print("\nGrasped. PAUSING for a human grip check -- confirm the handle is "
          "actually gripped before running the door-arc script.")
    input("Press Enter once confirmed (Ctrl-C to abort) ...")
    print("OK -- run scripts/real_gen3_open_microwave.py next.")


if __name__ == "__main__":
    main()
