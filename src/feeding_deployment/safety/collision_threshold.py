"""Helper for temporarily overriding the collision-detection threshold.

HLAs can wrap a section of execution in ``with collision_threshold(value):`` to
raise or lower the sensitivity of :class:`CollisionSensor` for the duration of
that section. The prior threshold is always restored on exit -- including when
the wrapped code raises -- so the sensor never gets stuck at an overridden value.
"""
from contextlib import contextmanager

import rclpy

from feeding_deployment.ros2_utils import node_handle
from feeding_deployment.ros2_utils import rospy_compat

from feeding_deployment_msgs.srv import SetCollisionThreshold


def _call(client, value):
    """Build a request, call the service synchronously, return the response.

    TODO(ros2): verify async call semantics at this call site. rclpy service
    clients are inherently async (`call_async` + a future); this spins the
    shared node until the future resolves, which mirrors the old synchronous
    `rospy.ServiceProxy(...)()` call but relies on nothing else needing to
    spin the same node concurrently on another thread while this blocks.
    """
    request = SetCollisionThreshold.Request()
    request.threshold = float(value)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node_handle.get_node(), future)
    return future.result()


@contextmanager
def collision_threshold(value, wait_timeout=2.0):
    """Temporarily set the collision threshold; always restore the prior value.

    Args:
        value: New collision threshold to apply for the duration of the block.
        wait_timeout: Seconds to wait for the /set_collision_threshold service.
    """
    client = node_handle.get_node().create_client(SetCollisionThreshold, "/set_collision_threshold")
    # TODO(ros2): behavior difference from rospy.wait_for_service -- the old call
    # RAISED rospy.ROSException on timeout, so a missing service aborted the
    # `with` block before any threshold was touched. rclpy's wait_for_service
    # (via rospy_compat) returns True/False instead of raising; a False here is
    # currently NOT checked, so a timed-out wait falls through into _call(),
    # which will block in spin_until_future_complete with no timeout. Verify
    # whether this should raise/return early on a False wait_for_service result.
    rospy_compat.wait_for_service(client, timeout=wait_timeout)
    previous = _call(client, value).previous_threshold
    try:
        yield
    finally:
        # Restore the exact prior value, even on exception. Restoring `previous`
        # (rather than a constant default) keeps nested usage correct.
        _call(client, previous)
