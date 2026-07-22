"""Client-side wrapper for driving the Vention base over RPC.

Used by the cmd_vel bridge and the teleop scripts on the compute box. Mirrors
ArmInterfaceClient (arm_client.py) but is intentionally thin: it does not block on
/watchdog_status, since callers only need the base RPC server (and bulldog, which
gates set_speeds via _require_bulldog) to be up.
"""

from feeding_deployment.control.base_controller.base_interface import (
    BaseManager,
    NUC_HOSTNAME,
    BASE_RPC_PORT,
    RPC_AUTHKEY,
)


class BaseInterfaceClient:
    def __init__(self):
        # Register BaseInterface (no lambda needed on the client-side)
        BaseManager.register("BaseInterface")

        # Client setup
        self.manager = BaseManager(address=(NUC_HOSTNAME, BASE_RPC_PORT), authkey=RPC_AUTHKEY)
        self.manager.connect()

        # This will now use the single, shared instance of BaseInterface
        self._base_interface = self.manager.BaseInterface()

    def set_speeds(self, speed_a, speed_b):
        return self._base_interface.set_speeds(speed_a, speed_b)

    def stop(self):
        return self._base_interface.stop()

    def get_encoders(self):
        """Encoder snapshot dict or None (see VentionBase.read_encoders).
        Raises AttributeError if the NUC is still running a pre-encoder
        base_server (the proxy's exposed list comes from the server)."""
        return self._base_interface.get_encoders()
