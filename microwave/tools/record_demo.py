"""READ-ONLY: log a hand-guided demo at ~10 Hz (joints, EE pose, gripper) to a CSV.

Never commands the arm. Stop with Ctrl-C / SIGINT / SIGTERM, or by creating the file
given by --stop-file. Used 09-28 to record the user's push-open (release, back off,
push the door the rest of the way) so it can be translated into planner legs.

    ARM_RPC_HOST=127.0.0.1 python3 microwave/tools/record_demo.py --out microwave/demos/microwave_demo.csv
"""
import argparse, csv, signal, time
from pathlib import Path

import numpy as np

from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient

a = argparse.ArgumentParser()
a.add_argument("--out", required=True)
a.add_argument("--hz", type=float, default=10.0)
a.add_argument("--stop-file", default="/tmp/record_demo.stop")
args = a.parse_args()

stop = {"now": False}
for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda *_: stop.update(now=True))
Path(args.stop_file).unlink(missing_ok=True)

ai = ArmInterfaceClient()
out = Path(args.out).expanduser()
with out.open("w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["t"] + [f"j{i}_deg" for i in range(1, 8)] + ["x", "y", "z", "qx", "qy", "qz", "qw", "gripper"])
    t0, n = time.time(), 0
    print(f"recording to {out} at {args.hz:.0f} Hz -- stop with Ctrl-C or: touch {args.stop_file}", flush=True)
    while not stop["now"] and not Path(args.stop_file).exists():
        try:
            st = ai.get_state()
        except Exception as e:   # keep recording through a transient RPC hiccup
            print(f"get_state failed: {e}", flush=True)
            time.sleep(0.5)
            continue
        q = np.degrees(np.asarray(st["position"], dtype=float))
        w.writerow([f"{time.time() - t0:.2f}"] + [f"{v:.2f}" for v in q]
                   + [f"{v:.4f}" for v in list(st["ee_pos"])[:7]] + [f"{float(st['gripper_pos']):.4f}"])
        n += 1
        if n % 50 == 0:
            fh.flush()
        time.sleep(1.0 / args.hz)
print(f"stopped after {n} samples ({time.time() - t0:.1f} s) -> {out}", flush=True)
