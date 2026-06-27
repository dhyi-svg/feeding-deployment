"""Helper for temporarily overriding the collision-detection threshold.

HLAs can wrap a section of execution in ``with collision_threshold(value):`` to
raise or lower the sensitivity of :class:`CollisionSensor` for the duration of
that section. The prior threshold is always restored on exit -- including when
the wrapped code raises -- so the sensor never gets stuck at an overridden value.
"""
from contextlib import contextmanager

import rospy

from feeding_deployment_msgs.srv import SetCollisionThreshold


@contextmanager
def collision_threshold(value, wait_timeout=2.0):
    """Temporarily set the collision threshold; always restore the prior value.

    Args:
        value: New collision threshold to apply for the duration of the block.
        wait_timeout: Seconds to wait for the /set_collision_threshold service.
    """
    rospy.wait_for_service("/set_collision_threshold", timeout=wait_timeout)
    proxy = rospy.ServiceProxy("/set_collision_threshold", SetCollisionThreshold)
    previous = proxy(float(value)).previous_threshold
    try:
        yield
    finally:
        # Restore the exact prior value, even on exception. Restoring `previous`
        # (rather than a constant default) keeps nested usage correct.
        proxy(previous)
