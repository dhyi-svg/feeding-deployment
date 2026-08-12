"""Stop motion NOW and latch, so a running script cannot send its next command.

    python scripts/session/arm_halt.py           # stop + latch
    python scripts/session/arm_halt.py --clear   # release the latch
    python scripts/session/arm_halt.py --status  # read the latch, no side effects

*** UNVERIFIED ON HARDWARE. The physical e-stop remains the only proven stop. ***

Replaces arm_stop_action.py, which called Base.StopAction() -- verified on 2026-08-12 to
NOT stop the arm, while the running arc kept issuing waypoints for 11 more seconds. This
calls Base.Stop() (what emergency_stop uses) AND latches, so every later motion command
asserts server-side and the client loop dies. Both halves are required: a stop that only
cancels the current motion is useless against a loop, which is what --steps produces.

Unlike emergency_stop it is recoverable with --clear; no arm_server restart needed.
"""
import argparse

from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--clear", action="store_true", help="release the halt latch")
parser.add_argument("--status", action="store_true", help="read the latch only")
args = parser.parse_args()

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
arm = manager.ArmInterface()

if args.status:
    print(f"halted: {arm.is_halted()}")
elif args.clear:
    arm.clear_halt()
else:
    arm.halt()
