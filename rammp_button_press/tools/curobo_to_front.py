"""cuRobo move to the 'front' start pose (joints from today's first runs). Dry by default."""
import sys, json, time, numpy as np, rclpy
from pathlib import Path
from scipy.spatial.transform import Rotation
from rammp_button_press import Abort
from rammp_button_press.rammp_arm import RammpArm
FRONT = np.radians([-117.1, -47.6, -52.0, -124.1, 57.1, 25.4, 78.2])
execute = "--execute" in sys.argv
out = open(Path.home() / "press_logs" / f"curobo_front_{time.strftime('%Y%m%d_%H%M%S')}.jsonl", "w")
log = lambda r: (out.write(json.dumps(r) + "\n"), out.flush())
rclpy.init(); arm = RammpArm(execute=execute)
try:
    problems, info = arm.status()
    if problems: raise Abort("; ".join(problems))
    p, q = arm.kin.fk(FRONT); Rm = Rotation.from_quat(q).as_matrix()
    print(f"  from tool {np.round(arm.ee_pos(), 3)} to front {np.round(p, 3)}")
    arm.take_control()
    arm.curobo_move(p, Rm, "to front", log, track_abort_m=0.05)
    print("  DRY: planned and gated only" if not execute else "  done")
except Abort as e:
    print(f"ABORT: {e}"); sys.exit(2)
finally:
    arm.close()
    if rclpy.ok(): rclpy.shutdown()
