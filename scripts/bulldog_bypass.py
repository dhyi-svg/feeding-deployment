"""Minimal bulldog BYPASS for arm-only testing on this single-machine rig.

WARNING: this replaces the real bulldog e-stop hub with nothing. The real bulldog
subscribes to /experimentor_estop and halts the arm when that fires; this does NOT.
Your only stop while this runs is the PHYSICAL e-stop or killing a process. Keep a
hand on the hardware e-stop.

What it does: unlocks arm motion by calling ArmInterface.register_bulldog() over
RPC (flips bulldog_ready=True), then feeds the required heartbeat via is_alive()
faster than BULLDOG_HEARTBEAT_TIMEOUT (1.0 s) so the arm's monitor doesn't
emergency-stop. Retained safety property: if THIS process dies, the heartbeat
stops and the arm emergency-stops within ~1 s.

Neither register_bulldog() nor is_alive() moves the arm -- this only unlocks the
ability to command motion. Run with the arm server up:

    ARM_RPC_HOST=127.0.0.1 python scripts/bulldog_bypass.py
"""
import time

from feeding_deployment.control.robot_controller.arm_interface import (
    ArmManager,
    NUC_HOSTNAME,
    ARM_RPC_PORT,
    RPC_AUTHKEY,
)

HEARTBEAT_PERIOD = 0.3  # must stay < BULLDOG_HEARTBEAT_TIMEOUT (1.0 s)

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
arm = manager.ArmInterface()

arm.register_bulldog()
print(
    f"bulldog BYPASS active — arm motion UNLOCKED. Heartbeat every {HEARTBEAT_PERIOD}s.\n"
    "NO software e-stop is running. Keep the physical e-stop in reach.\n"
    "Ctrl-C to stop the bypass (arm loses heartbeat and emergency-stops within ~1 s)."
)
try:
    while True:
        arm.is_alive()
        time.sleep(HEARTBEAT_PERIOD)
except KeyboardInterrupt:
    print("\nbypass stopped — arm will emergency-stop within ~1 s (heartbeat lost).")
