"""Set the arm speed preset. Usage: arm_set_speed.py [low|medium|high]  (default low)

The microwave task scripts READ the speed but never set it, so whatever the arm was
last left at is what you get. Set it explicitly every session.

Must run AFTER bulldog_bypass.py -- set_speed calls _require_bulldog() and will
assert if the bypass has not registered yet.

Presets (kinova.py choose_from_speed_presets):
    low    30 deg/s joint,  60 deg/s^2 accel, 0.15 m/s linear
    medium 40 deg/s joint,  80 deg/s^2 accel, 0.20 m/s linear
    high   50 deg/s joint, 100 deg/s^2 accel, 0.25 m/s linear
"low" is the floor through this API; set_joint_limits is not RPC-exposed.
"""
import sys

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)

speed = sys.argv[1] if len(sys.argv) > 1 else "low"

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
arm = manager.ArmInterface()
arm.set_speed(speed)
print(f"speed is now: {arm.get_speed()}")
