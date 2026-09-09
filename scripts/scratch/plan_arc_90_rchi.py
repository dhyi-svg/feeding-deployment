import json
import numpy as np
from pathlib import Path
from pybullet_helpers.geometry import Pose
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.control.robot_controller.arm_interface import (
    ArmManager, NUC_HOSTNAME, ARM_RPC_PORT, RPC_AUTHKEY,
)

FIXED_HINGE = np.array([0.7177, 0.0699, 0.5585])
TARGET_ANGLE_DEG = 90.0
WAYPOINT_SPACING_M = 0.02
S = Path("/tmp/door_arc_pachirisu.json")

ArmManager.register("ArmInterface")
mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
mg.connect()
ai = mg.ArmInterface()

ee = list(ai.get_state()["ee_pos"])
g = float(ai.get_state().get("gripper_pos"))
print(f"current ee_pos: {np.round(ee[:3], 4)}  gripper {g:.4f}")
grasp = Pose(position=tuple(ee[:3]), orientation=tuple(ee[3:7]))
radius = float(np.linalg.norm(np.array(ee[:3]) - FIXED_HINGE))
arc_length_m = radius * np.radians(TARGET_ANGLE_DEG)
wps_pose = PerceptionInterface._generate_door_arc_waypoints(
    None, start_pose=grasp, hinge_position=tuple(FIXED_HINGE),
    arc_length_m=arc_length_m, waypoint_spacing_m=WAYPOINT_SPACING_M,
    direction=-1, rotate_orientation=True)
wps = [list(w.position) + list(w.orientation) for w in wps_pose]
S.write_text(json.dumps({"wps": wps, "i": 0, "hinge": list(FIXED_HINGE), "radius": radius}))
print(f"hinge (fixed): {np.round(FIXED_HINGE, 4)} (radius {radius*100:.1f}cm)")
print(f"{len(wps)} waypoints, target {TARGET_ANGLE_DEG}deg, arc_length {arc_length_m*100:.1f}cm")
ranges = [np.linalg.norm(w.position) for w in wps_pose]
print(f"range: min {min(ranges):.3f} max {max(ranges):.3f}")
print("PLAN ONLY -- state saved, nothing commanded.")
