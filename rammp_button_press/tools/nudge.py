"""Attended nudge: move the tool straight along the panel normal by DIST m (+ = toward the panel),
orientation fixed. Panel normal from the latest measure_prepress_offset log. Every gate as in press_button.
    python3 nudge.py 0.01            # plan + sim gates only
    python3 nudge.py 0.01 --execute  # e-stop attended, explicit go-ahead
"""
import json, sys, time
from pathlib import Path
import numpy as np, rclpy
from rammp_button_press import Abort
from rammp_button_press.panel_frame import pose_path, to_panel
from rammp_button_press.press_button import (SMOOTH_PRESS_SPACING_M, ROT_STEP_MAX_DEG, FINGERTIP_PAST_TOOL_M)
from rammp_button_press.rammp_arm import RammpArm

MAX_NUDGE_M = 0.02
dist = float(sys.argv[1]); execute = "--execute" in sys.argv
if not 0 < abs(dist) <= MAX_NUDGE_M:
    sys.exit(f"|dist| must be in (0, {MAX_NUDGE_M}]")
logf = sorted(Path.home().glob("press_logs/press_*.jsonl"))
meas = [json.loads(l) for f in logf for l in f.read_text().splitlines() if '"measure_prepress_offset"' in l][-1]
origin, R = np.array(meas["panel_origin"]), np.array(meas["panel_R"])
n_out = R[:, 2]
out = open(Path.home() / "press_logs" / f"nudge_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "w")
log = lambda r: (out.write(json.dumps(r) + "\n"), out.flush())

rclpy.init()
arm = RammpArm(execute=execute)
try:
    problems, info = arm.status()
    for i in info: print(" ", i)
    g = arm.state()["gripper_pos"]
    print(f"  gripper_pos {g:.3f}")
    if problems: raise Abort("; ".join(problems))
    p0, R0 = arm.ee_pose(); q0 = arm.joints()
    tip0 = to_panel(p0 + R0[:, 2] * FINGERTIP_PAST_TOOL_M, origin, R)
    p1 = p0 - dist * n_out
    path = pose_path(p0, R0, p1, R0, SMOOTH_PRESS_SPACING_M, ROT_STEP_MAX_DEG)
    start = (q0, p0, R0)
    err, jump, _, knots = arm.precheck_poses(path, f"nudge {dist*100:+.1f} cm", log, start=start)
    print(f"  tool {np.round(p0,4)} -> {np.round(p1,4)}  ({dist*100:+.1f} cm along -panel normal {np.round(n_out,3)})")
    print(f"  fingertip vs detected panel: {tip0[2]*100:+.1f} cm out now -> {(tip0[2]-dist)*100:+.1f} cm")
    print(f"  sim gates: {len(path)} points, worst IK err {err*1000:.2f} mm, worst joint step {jump:.2f} deg -> OK")
    if not execute:
        print("  DRY RUN: nothing sent"); sys.exit(0)
    arm.take_control()
    arm.smooth_poses(path, knots, start, f"nudge {dist*100:+.1f} cm", log)
    p2, _ = arm.ee_pose()
    print(f"  moved {np.dot(p0 - p2, n_out)*100:+.2f} cm toward the panel; tool now {np.round(p2,4)}")
except Abort as e:
    print(f"ABORT: {e}"); sys.exit(2)
finally:
    arm.close()
    if rclpy.ok(): rclpy.shutdown()
