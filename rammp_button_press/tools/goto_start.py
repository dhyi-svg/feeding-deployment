"""Attended: back out BACK_M along the panel normal, then straight line to the saved start pose
(~/press_logs/start_joints.json tool pose). Both legs sim-gated (same gates + move limits as press_button).
    python3 goto_start.py            # plan + gates only
    python3 goto_start.py --execute
"""
import json, sys, time
from pathlib import Path
import numpy as np, rclpy
from scipy.spatial.transform import Rotation
from rammp_button_press import Abort
from rammp_button_press.panel_frame import pose_path
from rammp_button_press.press_button import (SMOOTH_PRESS_SPACING_M, ROT_STEP_MAX_DEG, RAY_STEP_MAX_M, check_move)
from rammp_button_press.rammp_arm import RammpArm

BACK_M = 0.01
execute = "--execute" in sys.argv
logs = sorted(Path.home().glob("press_logs/press_*.jsonl"))
meas = [json.loads(l) for f in logs for l in f.read_text().splitlines() if '"measure_prepress_offset"' in l][-1]
n_out = np.array(meas["panel_R"])[:, 2]
st = json.loads((Path.home() / "press_logs" / "start_joints.json").read_text())
# start_joints.json stores RammpArm.state()["ee_pos"]: already the TOOL frame (/ee_state + TOOL_Z)
p_s, R_s = np.array(st["ee_pos"][:3]), Rotation.from_quat(st["ee_pos"][3:7]).as_matrix()
out = open(Path.home() / "press_logs" / f"goto_start_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "w")
log = lambda r: (out.write(json.dumps(r) + "\n"), out.flush())

rclpy.init()
arm = RammpArm(execute=execute)
try:
    problems, info = arm.status()
    for i in info: print(" ", i)
    print(f"  gripper_pos {arm.state()['gripper_pos']:.3f}")
    if problems: raise Abort("; ".join(problems))
    p0, R0 = arm.ee_pose(); q0 = arm.joints()
    p1 = p0 + BACK_M * n_out
    leg1 = pose_path(p0, R0, p1, R0, SMOOTH_PRESS_SPACING_M, ROT_STEP_MAX_DEG)
    check_move(p1, R0, p_s, R_s, "start pose")
    leg2 = pose_path(p1, R0, p_s, R_s, RAY_STEP_MAX_M, ROT_STEP_MAX_DEG)
    e1, j1, state, k1 = arm.precheck_poses(leg1, "back out", log, start=(q0, p0, R0))
    e2, j2, _, k2 = arm.precheck_poses(leg2, "to start", log, start=state)
    print(f"  back out  {len(leg1):3d} pts  {BACK_M*100:.1f} cm along the normal   IK <= {e1*1000:.2f} mm  joint step <= {j1:.2f} deg  ok")
    turn = np.degrees(np.arccos(np.clip((np.trace(R0.T @ R_s) - 1) / 2, -1, 1)))
    print(f"  to start  {len(leg2):3d} pts  {np.linalg.norm(p_s-p1)*100:.1f} cm, wrist turns {turn:.1f} deg   IK <= {e2*1000:.2f} mm  joint step <= {j2:.2f} deg  ok")
    print(f"  tool {np.round(p0,4)} -> {np.round(p1,4)} -> {np.round(p_s,4)}")
    print(f"  final joints (deg) {np.round(np.degrees(k2[-1]),1)}  vs saved start {np.round(np.degrees(st['joints']),1)}")
    if not execute:
        print("  DRY RUN: nothing sent"); sys.exit(0)
    arm.take_control()
    arm.smooth_poses(leg1, k1, (q0, p0, R0), "back out", log)
    p, R = arm.ee_pose(); q = arm.joints()
    e2, j2, _, k2 = arm.precheck_poses(leg2 := pose_path(p, R, p_s, R_s, RAY_STEP_MAX_M, ROT_STEP_MAX_DEG), "to start", log, start=(q, p, R))
    arm.smooth_poses(leg2, k2, (q, p, R), "to start", log)
except Abort as e:
    print(f"ABORT: {e}"); sys.exit(2)
finally:
    arm.close()
    if rclpy.ok(): rclpy.shutdown()
