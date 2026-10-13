"""Hardware hold test (step 2): open an ee_pose_impedance session at the CURRENT pose, hold it ~0.9 s
without moving, measure sag/drift, close. Aborts (closes, driver holds) if the tool drifts > 5 mm.
    python3 imp_hold.py            # checks only, opens nothing
    python3 imp_hold.py --execute  # attended, e-stop manned, explicit go-ahead
"""
import sys, json, time
from pathlib import Path
import numpy as np, rclpy
from rammp_button_press import Abort
from rammp_button_press.rammp_arm import RammpArm
execute = "--execute" in sys.argv
out = open(Path.home() / "press_logs" / f"imp_hold_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "w")
log = lambda r: (out.write(json.dumps(r) + "\n"), out.flush())
rclpy.init(); arm = RammpArm(execute=execute)
try:
    problems, info = arm.status()
    for i in info: print(" ", i)
    if problems: raise Abort("; ".join(problems))
    p, R = arm.ee_pose()
    print(f"  tool {np.round(p, 4)}  gripper {arm.state()['gripper_pos']:.3f}")
    if not execute:
        print("  DRY: would open an ee_pose_impedance session here and hold ~0.9 s"); sys.exit(0)
    arm.take_control()
    arm.press_impedance(p, R[:, 2], 0.0, "hold test", log)    # depth 0: hold only, n unused
    print("  hold test passed")
except Abort as e:
    print(f"ABORT: {e}"); sys.exit(2)
finally:
    arm.close()
    if rclpy.ok(): rclpy.shutdown()
