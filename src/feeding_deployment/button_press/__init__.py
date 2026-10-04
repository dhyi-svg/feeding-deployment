"""Autonomous microwave button press (ROS 2): find the button, go in front of it, press.

``press_button`` is the one-command driver: the dome-layout detector (``dome_pattern``)
finds the button, depth gives the panel plane and the PANEL FRAME (``panel_frame``), the arm
goes to a stored pre-press spot in that frame, presses along the panel normal and returns.
Dry run unless ``--execute``. Its ROS inputs (camera frames, tf) live in ``perception`` and
its IK/motion gates in ``arm``. Pure, unit-tested pieces (no ROS / arm / pybullet):
``dome_pattern``, ``panel_frame`` and ``geometry``.

Plain-language walkthrough: ``README.md`` in this directory. Bring-up:
``scripts/button_press/bringup.sh``; procedure: ``docs/button_press_runbook.md``.

Not yet wired into ``actions.press_microwave_button.PressMicrowaveButtonHLA`` -- that HLA
still runs the older open-loop pre-press/press pose sequence.
"""


class Abort(SystemExit):
    """Stop the run and HOLD the arm where it is.

    Raised by every gate; ``press_button.main`` catches it and prints why.
    """
