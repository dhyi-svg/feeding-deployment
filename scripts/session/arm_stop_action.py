"""Abort the in-flight motion WITHOUT latching emergency stop.

Unlike emergency_stop() (and unlike killing bulldog_bypass.py), this leaves the arm
accepting commands afterwards -- no restart of arm_server.py + bypass needed. This
is the FIRST thing to reach for when a motion looks wrong but is not dangerous.

Escalation order: this -> pkill -f bulldog_bypass.py -> physical e-stop.
The physical e-stop always beats all of them.
"""
from feeding_deployment.control.robot_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmManager,
)

ArmManager.register("ArmInterface")
manager = ArmManager(address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY)
manager.connect()
manager.ArmInterface().stop_action()
print("stop_action sent -- motion aborted, no e-stop latched, arm still commandable.")
