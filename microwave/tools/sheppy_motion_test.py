"""First-motion checks for `SheppyArm` (attended, e-stop in hand). One test per run:

    python3 microwave/tools/sheppy_motion_test.py joint      # J6 +5 deg, then back
    python3 microwave/tools/sheppy_motion_test.py gripper    # close, then open (empty air)
    python3 microwave/tools/sheppy_motion_test.py cartesian  # tool +3 cm along base x, then back

Each prints where it starts and ends; a move that doesn't land within tolerance stops the test.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # microwave/
import numpy as np

from sheppy_arm import SheppyArm
from feeding_deployment.control.robot_controller.command_interface import (
    CartesianTrajectoryCommand, CloseGripperCommand, JointCommand, OpenGripperCommand)


def joint_test(ai):
    q0 = np.array(ai.get_state()["position"])
    for label, tgt in (("J6 +5 deg", q0 + np.radians([0, 0, 0, 0, 0, 5, 0])), ("back", q0)):
        print(f"{label}: J6 {np.degrees(ai.get_state()['position'][5]):.2f} -> {np.degrees(tgt[5]):.2f} deg")
        ok = ai.execute_command(JointCommand(pos=tgt.tolist()))
        q = np.array(ai.get_state()["position"])
        err = np.degrees(np.max(np.abs((q - tgt + np.pi) % (2 * np.pi) - np.pi)))
        print(f"  ok={ok}  max joint error {err:.2f} deg")
        if not ok or err > 1.0:
            return


def gripper_test(ai):
    for label, cmd in (("close", CloseGripperCommand()), ("open", OpenGripperCommand())):
        g0 = ai.get_state()["gripper_pos"]
        ai.execute_command(cmd)
        print(f"{label}: gripper {g0:.3f} -> {ai.get_state()['gripper_pos']:.3f}")


def cartesian_test(ai):
    ee = np.array(ai.get_state()["ee_pos"])
    p0, quat = ee[:3], ee[3:]
    for label, tgt in (("+3 cm x", p0 + [0.03, 0, 0]), ("back", p0)):
        cur = np.array(ai.get_state()["ee_pos"][:3])
        steps = [cur + (tgt - cur) * k / 3 for k in (1, 2, 3)]          # 1 cm waypoints
        print(f"{label}: tool {np.round(cur, 4)} -> {np.round(tgt, 4)}")
        ok = ai.execute_command(CartesianTrajectoryCommand([(s, quat) for s in steps]))
        st = ai.get_state()["ee_pos"]
        perr = np.linalg.norm(np.array(st[:3]) - tgt)
        rerr = np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(st[3:], quat))))))
        print(f"  ok={ok}  position error {perr * 1000:.1f} mm  orientation error {rerr:.2f} deg")
        if not ok or perr > 0.005:
            return


if __name__ == "__main__":
    test = sys.argv[1] if len(sys.argv) > 1 else ""
    fn = {"joint": joint_test, "gripper": gripper_test, "cartesian": cartesian_test}.get(test)
    if fn is None:
        sys.exit(__doc__)
    arm = SheppyArm()
    try:
        fn(arm)
    finally:
        arm.close()
