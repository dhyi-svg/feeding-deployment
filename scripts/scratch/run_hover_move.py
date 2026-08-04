"""Command the validated hover-only joint target (no grasp, no gripper action).
Low speed, waits for settle, reports final tracking error. See TESTING_LOG.md /
chat for how this target was derived (geometry-corrected seeded IK chain)."""
import time

import numpy as np

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT, NUC_HOSTNAME, RPC_AUTHKEY, ArmManager,
)

JOINT_TARGET = [-2.02906, -0.67763, -1.02388, -1.77224, 0.35732, 0.5143, 1.83124]
TARGET_POS = (0.6906, -0.1806, 0.4701)


def main():
    ArmManager.register("ArmInterface")
    mg = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
    mg.connect()
    ai = mg.ArmInterface()
    ai.set_speed("low")

    before = ai.get_state()["ee_pos"]
    print(f"[hover-move] before: ee_pos={np.round(before, 4)}")
    print(f"[hover-move] commanding joint target: {np.round(JOINT_TARGET, 4)}")
    ai.set_joint_position(JOINT_TARGET)

    time.sleep(0.5)  # give the arm time to actually start moving before checking velocity
    for _ in range(100):
        stt = ai.get_state()
        if max(abs(x) for x in stt["velocity"]) < 0.01:
            break
        time.sleep(0.1)

    after = ai.get_state()
    got = np.array(after["ee_pos"][:3])
    err = float(np.linalg.norm(got - np.array(TARGET_POS)))
    print(f"[hover-move] after:  ee_pos={np.round(after['ee_pos'], 4)}")
    print(f"[hover-move] joint position: {np.round(after['position'], 4)}")
    print(f"[hover-move] tracking error vs target pos: {err*100:.2f}cm")


if __name__ == "__main__":
    main()
