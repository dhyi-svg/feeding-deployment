"""READ-ONLY Kortex probe: arm state + servoing mode. No motion.

Opens its OWN Kortex session, so run it only when arm_server.py is NOT running --
one process at a time may hold the arm. Once arm_server is up, use
check_tf_vs_fk.py (which goes through the RPC) instead.

Want: ARMSTATE_SERVOING_READY (7). ARMSTATE_SERVOING_MANUALLY_CONTROLLED (9) means
something else holds control -- see JETSON_TESTING.md.
"""
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2, Session_pb2
from kortex_api.RouterClient import RouterClient
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport

ARM_IP, ARM_PORT = "192.168.1.10", 10000

transport = TCPTransport()
router = RouterClient(transport, lambda ex: print("router:", ex))
transport.connect(ARM_IP, ARM_PORT)

info = Session_pb2.CreateSessionInfo()
info.username, info.password = "admin", "admin"
info.session_inactivity_timeout = 60000
info.connection_inactivity_timeout = 2000
session = SessionManager(router)
session.CreateSession(info)

base = BaseClient(router)
state = base.GetArmState().active_state
name = next(
    (n for n in dir(Base_pb2) if n.startswith("ARMSTATE") and getattr(Base_pb2, n) == state),
    str(state),
)
print(f"ARM STATE : {name} ({state})")
try:
    print(f"SERVOMODE : {base.GetServoingMode().servoing_mode}")
except Exception as e:  # diagnostic only -- never fatal
    print(f"servoing mode read failed: {e}")

session.CloseSession()
transport.disconnect()
