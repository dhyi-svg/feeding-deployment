"""Move the hand to a better look-in pose: back from the microwave, up, and tilted down.
Straight Cartesian leg, IK/limit/jump/collision-checked in the sim before anything moves;
collision against the scene points of a saved frame (the last look). Dry run unless --execute.

    python3 -u microwave/placement/tools/move_look_pose.py --back 0.12 --up 0.10 --tilt 10            # plan only
    python3 -u microwave/placement/tools/move_look_pose.py --back 0.12 --up 0.10 --tilt 10 --execute
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))
import placement_workflow as wf  # noqa: E402
import point_cloud as pcm  # noqa: E402
from microwave_common import execute_joint_plan  # noqa: E402
from placement_config import load_config  # noqa: E402
from placement_planner import PlacementSim, PlanError  # noqa: E402

a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
a.add_argument("--back", type=float, default=0.12, help="m, away from where the hand points (horizontal)")
a.add_argument("--up", type=float, default=0.10, help="m")
a.add_argument("--tilt", type=float, default=10.0, help="deg, extra nose-down tilt")
a.add_argument("--frame", help="frame_*.npz for obstacles (default: newest run's)")
a.add_argument("--execute", action="store_true")
args = a.parse_args()
if not (0 <= args.back <= 0.25 and 0 <= args.up <= 0.2 and -45 <= args.tilt <= 45):
    sys.exit("back 0..0.25 m, up 0..0.2 m, tilt -45..45 deg only")

cfg = load_config()
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient  # noqa: E402
ai = ArmInterfaceClient()
st = ai.get_state()
q0 = np.asarray(st["position"], float)
p0, quat0 = np.asarray(st["ee_pos"][:3], float), np.asarray(st["ee_pos"][3:7], float)
approach = R.from_quat(quat0).as_matrix()[:, 2]
a_h = np.array([approach[0], approach[1], 0.0])
a_h /= np.linalg.norm(a_h)
side = np.cross([0.0, 0.0, 1.0], a_h)
p1 = p0 - a_h * args.back + np.array([0, 0, args.up])
# +tilt = nose DOWN: about +side (= z x approach), the approach turns toward -z
# (10-20 fix: the first version used -side, which tilted the hand UP)
quat1 = (R.from_rotvec(side * np.radians(args.tilt)) * R.from_quat(quat0)).as_quat()
a1 = R.from_quat(quat1).as_matrix()[:, 2]
assert args.tilt == 0 or np.sign(approach[2] - a1[2]) == np.sign(args.tilt), 'tilt sign check failed'
print(f"from {np.round(p0, 3).tolist()} -> {np.round(p1, 3).tolist()}, approach pitch "
      f"{np.degrees(np.arcsin(approach[2])):+.1f} -> {np.degrees(np.arcsin(a1[2])):+.1f} deg (negative = nose down), "
      f"reach {np.linalg.norm(p1):.3f} m")

sim = PlacementSim(cfg)
frame = Path(args.frame) if args.frame else max(wf.LOG_ROOT.glob("2*/frame_*.npz"), key=os.path.getmtime)
fr = wf.Frame.load(frame)
pts, _ = wf.scene_points(fr, cfg, tool_pose=(p0, quat0), stride=3)
vox = pcm.voxel_downsample(pts, cfg.planner.obstacle_voxel_m)
sim.add_voxels(vox, cfg.planner.obstacle_voxel_m)
print(f"obstacles: {len(vox)} voxels from {frame}")
try:
    leg = sim.plan_leg("to look pose", p0, quat0, p1, quat1, q0, sim.voxels, cfg.planner.min_clearance_m,
                       level=False, with_box=False)
except PlanError as e:
    sys.exit(f"REFUSED: {e} -- nothing commanded")
print(f"end J {np.round(np.degrees(leg.joints[-1]), 1).tolist()}")
if not args.execute:
    print("DRY RUN -- nothing commanded")
else:
    ok = execute_joint_plan(ai, leg.joints, "to look pose")
    st = ai.get_state()
    print(f"{'DONE' if ok else 'STOPPED EARLY'}: EE {np.round(st['ee_pos'][:3], 3).tolist()} "
          f"({np.linalg.norm(np.asarray(st['ee_pos'][:3]) - p1) * 100:.1f} cm from target)")
sys.stdout.flush()
try:
    from feeding_deployment.ros2.node import shutdown
    shutdown()
except Exception:  # noqa: BLE001
    pass
os._exit(0)
