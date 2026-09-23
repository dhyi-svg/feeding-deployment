"""Autonomous microwave button press (ROS 2): visual servo onto the button + force stop.

Proven on hardware 2026-09-21/22 (rchi-cpu-5, Comfee microwave): the arm pressed
``timer_clock`` unaided and the microwave registered it. Three processes, one per module:

  ``detector_node``     ROS 2 node. Finds the named button by carrying a hand-placed mark
                        on a multi-view reference image through a SIFT homography
                        (``perception.appliance_perception.reference_button_detector``).
                        Publishes ``/button_detector/{button_pixel,claw_pixel,panel_quad,
                        status,debug_image}``. Abstains (publishes no pixel) rather than
                        guess.
  ``press_detector``    Read-only Kortex session on the tool wrench. Publishes
                        ``/press_detector/{force_dev,pressed}``; re-baselined live by
                        SIGUSR1 (found via its pidfile).
  ``autonomous_press``  The driver. Servo the button pixel onto the fixed fingertip pixel,
                        step along the fingertip ray, stop on a confirmed force rise,
                        press, retract. Dry run unless ``--execute``.

The driver's ROS inputs live in ``perception`` and its IK/motion gates in ``arm``.
Pure, unit-tested pieces (no ROS / arm / pybullet): ``geometry`` (panel plane fit, rays,
servo corrections) and ``contact`` (the approach contact rule).

Plain-language walkthrough of what all of this does: ``README.md`` in this directory.

Bring-up and the run ladder: ``docs/button_press_runbook.md``. Perception bring-up:
``launch/ros2/button_press_bringup.launch.py``. Building/extending the reference:
``scripts/button_press/``.

Not yet wired into ``actions.press_microwave_button.PressMicrowaveButtonHLA`` -- that HLA
still runs the older open-loop pre-press/press pose sequence.
"""


class Abort(SystemExit):
    """Stop the run and HOLD the arm where it is (nothing auto-retracts).

    Raised by every gate in the driver; ``Run.run`` catches it and prints the way back.
    """
