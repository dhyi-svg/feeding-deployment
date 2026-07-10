"""Stub base RPC server for arm-only testing on this single-machine deployment.

The real base server (`base_controller/base_server.py`) drives a Vention/Arduino
base that is parked and unpowered on this rig. Bulldog nonetheless refuses to
start unless a base RPC server answers on BASE_RPC_PORT -- it does
`BaseManager.connect()` and then calls `register_bulldog()`, `is_alive()`, and
`emergency_stop()` on the base interface. This serves a **no-op** BaseInterface so
bulldog's handshake succeeds without any base hardware.

Nothing here moves anything -- there is no hardware behind it.

Binds to the same host bulldog connects to (arm_interface.NUC_HOSTNAME, which honors
ARM_RPC_HOST), so run it with the same env as the arm server:

    ARM_RPC_HOST=127.0.0.1 python scripts/stub_base_server.py
"""
import signal
import sys

# NUC_HOSTNAME is imported from arm_interface (not base_interface) because that is
# exactly what bulldog uses for BOTH the arm and base connections, and it honors
# the ARM_RPC_HOST override.
from feeding_deployment.control.robot_controller.arm_interface import NUC_HOSTNAME
from feeding_deployment.control.base_controller.base_interface import (
    BaseManager,
    BASE_RPC_PORT,
    RPC_AUTHKEY,
)


class StubBaseInterface:
    """No-op stand-in for the parked base.

    Implements the full public surface of the real BaseInterface so any caller
    (bulldog today; a base client tomorrow) gets a valid, harmless response.
    """

    def __init__(self):
        self.bulldog_ready = False

    def is_alive(self):
        return True

    def register_bulldog(self):
        self.bulldog_ready = True
        print("Bulldog registered with stub base (no-op).")

    def emergency_stop(self):
        print("emergency_stop() -> no-op (parked base, nothing to stop)")

    def stop(self):
        pass

    def set_speeds(self, speed_a, speed_b):
        # Parked base: accept and ignore any speed command.
        pass

    def close(self):
        pass


stub_base_instance = StubBaseInterface()
BaseManager.register("BaseInterface", lambda: stub_base_instance)

signal_triggered = False


def signal_handler(sig, frame):
    global signal_triggered
    if not signal_triggered:
        signal_triggered = True
        print(f"Signal {sig} received, stopping stub base server.")
        sys.exit(0)
    else:
        print(f"Signal {sig} received, but handler is already processing.")


signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGQUIT, signal_handler)

if __name__ == "__main__":
    manager = BaseManager(address=(NUC_HOSTNAME, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    print(f"Stub base RPC server started at {NUC_HOSTNAME}:{BASE_RPC_PORT}")
    server.serve_forever()
