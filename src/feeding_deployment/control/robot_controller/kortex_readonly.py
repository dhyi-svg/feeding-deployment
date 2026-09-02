"""READ-ONLY Kortex feedback: arm state without taking control of the arm.

:class:`~feeding_deployment.control.robot_controller.kinova.KinovaArm` is a *controller*.
Constructing it clears faults, forces every actuator into position control mode and takes
the ``/tmp/kinova.lock`` single-instance lock -- all things that fight anything else
driving the arm. Recording data while a human teleops (Xbox controller plugged into the
arm, firmware-side) needs the opposite: a session that only ever reads.

This opens its own Kortex session and calls ``BaseCyclicClient.RefreshFeedback()``. It
never calls ``ClearFaults``, ``SetServoingMode``, ``ExecuteAction`` or any ``Send*``
command, and never touches the lock file -- so in principle it can run alongside
``arm_server.py`` *or* alongside teleop.

**UNVERIFIED on hardware:** whether the arm accepts this second session while the Xbox
controller holds control. Check it with ``record_teleop_demo.py --check`` *while
teleoping* before trusting it for a real recording.

``get_state()`` returns the same dict ``KinovaArm.get_state()`` and
``ArmInterfaceClient.get_state()`` do, so anything duck-typed on that contract -- e.g.
:class:`~feeding_deployment.ros2.joint_state_bridge.JointStateBridge` -- takes it
unchanged. The conversions below are copied from ``KinovaArm.get_state()``'s
non-cyclic branch deliberately: a recording whose joint angles differ in sign or wrap
from what the rest of the stack reports is worse than no recording.
"""

from __future__ import annotations

import math

import numpy as np
from scipy.spatial.transform import Rotation as R

try:
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.messages import Base_pb2, Session_pb2
    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.UDPTransport import UDPTransport

    KORTEX_AVAILABLE = True
except ModuleNotFoundError:  # so `--help` and imports work off-robot
    KORTEX_AVAILABLE = False

DEFAULT_IP = "192.168.1.10"
TCP_PORT = 10000
UDP_PORT = 10001

# Generous, because nothing here polls on a fixed schedule the way a controller does;
# a recorder paused at a breakpoint should not silently lose its session.
SESSION_INACTIVITY_MS = 60000
CONNECTION_INACTIVITY_MS = 2000


class KortexReadOnlyFeedback:
    """Read-only Kortex session exposing the repo's ``get_state()`` contract."""

    def __init__(
        self,
        ip: str = DEFAULT_IP,
        credentials: tuple[str, str] = ("admin", "admin"),
        with_base_client: bool = True,
    ) -> None:
        if not KORTEX_AVAILABLE:
            raise RuntimeError("kortex_api is not importable in this interpreter")

        self.ip = ip
        self._credentials = credentials
        self._closed = False
        self._connections: list[tuple] = []  # (session_manager, transport)

        # UDP is the real-time feedback port; RefreshFeedback is served there.
        self.base_cyclic = BaseCyclicClient(self._connect(UDPTransport(), UDP_PORT))
        # TCP only for GetArmState(), a diagnostic. Optional so a recording can run
        # with a single session if the arm ever turns out to be stingy with them.
        self.base = BaseClient(self._connect(TCPTransport(), TCP_PORT)) if with_base_client else None

        feedback = self.base_cyclic.RefreshFeedback()
        self.actuator_count = len(feedback.actuators)

    def _connect(self, transport, port: int):
        router = RouterClient(transport, RouterClient.basicErrorCallback)
        transport.connect(self.ip, port)
        info = Session_pb2.CreateSessionInfo()
        info.username, info.password = self._credentials
        info.session_inactivity_timeout = SESSION_INACTIVITY_MS
        info.connection_inactivity_timeout = CONNECTION_INACTIVITY_MS
        session = SessionManager(router)
        session.CreateSession(info)
        self._connections.append((session, transport))
        return router

    def get_state(self) -> dict:
        """One feedback sample, in ``KinovaArm.get_state()``'s shape."""
        feedback = self.base_cyclic.RefreshFeedback()
        n = self.actuator_count

        q, dq, tau = np.zeros(n), np.zeros(n), np.zeros(n)
        for i in range(n):
            q[i] = math.radians(feedback.actuators[i].position)
            if q[i] > np.pi:  # the arm reports 0..360; the repo works in -pi..pi
                q[i] -= 2 * np.pi
            dq[i] = math.radians(feedback.actuators[i].velocity)
            tau[i] = -feedback.actuators[i].torque

        ee = np.zeros(7)
        base = feedback.base
        ee[:3] = (base.tool_pose_x, base.tool_pose_y, base.tool_pose_z)
        ee[3:] = R.from_euler(
            "xyz",
            np.deg2rad([base.tool_pose_theta_x, base.tool_pose_theta_y, base.tool_pose_theta_z]),
        ).as_quat()

        return {
            "position": q,
            "velocity": dq,
            "effort": tau,
            "ee_pos": ee,
            "gripper_pos": feedback.interconnect.gripper_feedback.motor[0].position / 100.0,
            "ee_twist": np.array(
                [
                    base.tool_twist_linear_x,
                    base.tool_twist_linear_y,
                    base.tool_twist_linear_z,
                    base.tool_twist_angular_x,
                    base.tool_twist_angular_y,
                    base.tool_twist_angular_z,
                ]
            ),
            "ee_force": np.array(
                [
                    base.tool_external_wrench_force_x,
                    base.tool_external_wrench_force_y,
                    base.tool_external_wrench_force_z,
                ]
            ),
            "ee_torque": np.array(
                [
                    base.tool_external_wrench_torque_x,
                    base.tool_external_wrench_torque_y,
                    base.tool_external_wrench_torque_z,
                ]
            ),
            # The one signal that says whether the gripper is holding something.
            # gripper_pos saturates near 1.0 with or without a handle between the
            # fingers (see CLAUDE.md); motor current does not -- it stays elevated
            # while the fingers stall against an object.
            "gripper_current": getattr(
                feedback.interconnect.gripper_feedback.motor[0], "current_motor", None
            ),
        }

    def get_arm_state(self) -> str:
        """``ARMSTATE_*`` name, or a reason string. Never raises -- it is a diagnostic.

        Expect ``ARMSTATE_SERVOING_MANUALLY_CONTROLLED`` while something else (teleop)
        holds the arm, and ``ARMSTATE_SERVOING_READY`` when nothing does.
        """
        if self.base is None:
            return "UNAVAILABLE (no base client)"
        try:
            state = self.base.GetArmState().active_state
        except Exception as e:  # noqa: BLE001 -- diagnostics must not kill a recording
            return f"UNAVAILABLE ({e})"
        return next(
            (n for n in dir(Base_pb2) if n.startswith("ARMSTATE") and getattr(Base_pb2, n) == state),
            str(state),
        )

    def close(self) -> None:
        """Close both sessions. Idempotent."""
        if self._closed:
            return
        self._closed = True
        options = RouterClientSendOptions()
        options.timeout_ms = 1000
        for session, transport in self._connections:
            try:
                session.CloseSession(options)
            except Exception:  # noqa: BLE001 -- keep tearing down the rest
                pass
            try:
                transport.disconnect()
            except Exception:  # noqa: BLE001
                pass
        self._connections = []

    def __enter__(self) -> "KortexReadOnlyFeedback":
        return self

    def __exit__(self, *_) -> None:
        self.close()
