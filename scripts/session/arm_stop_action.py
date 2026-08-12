"""DIAGNOSTIC ONLY -- THIS DOES NOT STOP THE ARM. Verified broken on hardware 2026-08-12.

Calls ArmInterface.stop_action() -> KinovaArm.stop_action() -> Base.StopAction().

During a live door-opening arc this printed success and the arm kept moving for 11 more
seconds (arm_commands_log.txt: `stop_action` at 12:45:11, four more `set_joint_position`
through 12:45:21, halted only by the bulldog kill at 12:45:22). Two reasons:

  1. Base.StopAction() did not abort the in-flight motion. move_angular() commands via
     base.ExecuteAction(); Base.Stop() -- what emergency_stop() calls -- is what stops
     this arm.
  2. It cancels at most ONE motion. A multi-waypoint script immediately commands the
     next. Only a latch stops a loop: emergency_stop sets emergency_stop_active, after
     which every set_joint_position asserts.

To actually stop the arm: the PHYSICAL E-STOP, or `pkill -f bulldog_bypass.py` (~1 s,
latches). See JETSON_TESTING.md section 6.
"""
import sys

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)

print(__doc__.split("\n")[0], file=sys.stderr)
print("Continuing anyway (diagnostic). Use the physical e-stop to actually stop.\n",
      file=sys.stderr)

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
manager.ArmInterface().stop_action()
print("stop_action sent. NOTE: this is NOT known to stop motion -- see the docstring.")
