# Entrypoint (run on the NUC) for the Vention base RPC server. Mirrors
# arm_server.py: owns a single VentionBase instance (the Arduino is plugged into
# the NUC) wrapped in a BaseInterface, and serves it over RPC so the cmd_vel
# bridge / teleop on the compute box, and bulldog, can drive and stop the base.

import signal
import sys

from feeding_deployment.control.base_controller.vention_arduino_control import VentionBase
from feeding_deployment.control.base_controller.base_interface import (
    BaseInterface,
    BaseManager,
    NUC_HOSTNAME,
    BASE_RPC_PORT,
    RPC_AUTHKEY,
)

# Stable by-id path for the base Arduino (previously hardcoded in the cmd_vel bridge).
ARDUINO_PORT = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_03536383236351603052-if00"
ARDUINO_BAUD = 115200

# Create a single instance of VentionBase and BaseInterface
vention_base_instance = VentionBase(port_id=ARDUINO_PORT, baud=ARDUINO_BAUD)
base_interface_instance = BaseInterface(vention_base_instance)

# Register BaseInterface but return the existing instance
BaseManager.register("BaseInterface", lambda: base_interface_instance)

# Flag to check if the signal handler has been triggered
signal_triggered = False


# Signal handler function to stop + disconnect the base on Ctrl-C or Ctrl-\
def signal_handler(sig, frame):
    global signal_triggered
    if not signal_triggered:
        signal_triggered = True
        print(f"Signal {sig} received, stopping the base.")
        base_interface_instance.close()
        sys.exit(0)
    else:
        print(f"Signal {sig} received, but handler is already processing.")


# Register signal handler for Ctrl-C (SIGINT) and Ctrl-\ (SIGQUIT)
signal.signal(signal.SIGINT, signal_handler)  # Handle Ctrl-C
signal.signal(signal.SIGQUIT, signal_handler)  # Handle Ctrl-\

if __name__ == "__main__":
    manager = BaseManager(address=(NUC_HOSTNAME, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
    server = manager.get_server()
    print(f"Base manager server started at {NUC_HOSTNAME}:{BASE_RPC_PORT}")
    server.serve_forever()
