"""Helper for temporarily overriding the collision-detection threshold.

HLAs can wrap a section of execution in ``with
collision_threshold(value):`` to raise or lower the sensitivity of
:class:`CollisionSensor` for the duration of that section. The prior
threshold is always restored on exit -- including when the wrapped code
raises -- so the sensor never gets stuck at an overridden value.
"""

from contextlib import contextmanager

try:
    import rospy

    from feeding_deployment_msgs.srv import SetCollisionThreshold

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False


@contextmanager
def collision_threshold(value, wait_timeout=2.0):
    """Temporarily set the collision threshold; always restore the prior value.

    Every call site guards this with ``if self.robot_interface is not None``
    (else ``nullcontext()``), so in practice this only runs on the real
    robot. When ROS isn't importable (e.g. standalone HLA-in-sim runs on this
    box), it's a no-op that just logs what it would have set -- so code paths
    that call this directly (rather than through the nullcontext() guard)
    still work off-robot.

    Args:
        value: New collision threshold to apply for the duration of the block.
        wait_timeout: Seconds to wait for the /set_collision_threshold service.
    """
    if not ROSPY_IMPORTED:
        print(f"[collision_threshold] no ROS -- would set threshold {value}")
        yield
        return
    rospy.wait_for_service("/set_collision_threshold", timeout=wait_timeout)
    proxy = rospy.ServiceProxy("/set_collision_threshold", SetCollisionThreshold)
    previous = proxy(float(value)).previous_threshold
    try:
        yield
    finally:
        # Restore the exact prior value, even on exception. Restoring `previous`
        # (rather than a constant default) keeps nested usage correct.
        proxy(previous)
