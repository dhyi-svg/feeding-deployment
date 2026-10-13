"""Autonomous microwave button press on RAMMP's arm driver (kinova_gen3_node, ROS 2).

The port of ``feeding_deployment.button_press`` (branch ``button-task``) from the Cornell arm RPC
(``ArmInterfaceClient``) to RAMMP's stack under sheppy, run natively next to it ("hybrid
workflow"). What changed and why: ``README.md`` next to ``package.xml``.

``dome_pattern``, ``panel_frame`` and ``geometry`` are copies of the originals with only the
import path changed; ``test/test_sync_with_feeding_deployment.py`` fails if they drift.
``kinematics`` holds the sim IK and every motion gate (no ROS); ``rammp_arm`` talks to the
driver; ``perception`` holds the camera frames and tf; ``press_button`` is the driver script.
"""


class Abort(SystemExit):
    """Stop the run and HOLD the arm where it is.

    Raised by every gate; ``press_button.main`` catches it and prints why.
    """
