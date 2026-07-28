"""rospy-shaped convenience wrappers over the shared node from
``node_handle.py``.

Purpose: rather than every migrated file inventing its own way to write
"log something", "sleep", "check for shutdown", "convert rospy.Duration"
etc., this module gives one place with the (best-effort) rclpy equivalent,
so ~50 files can migrate those call sites near-mechanically:
``rospy.loginfo(...)`` -> ``rospy_compat.loginfo(...)``, etc.

This is deliberately NOT a full rospy shim/emulation layer -- publishers,
subscribers, services, and service clients are NOT wrapped here, because
their constructor argument shapes differ enough between rospy and rclpy
(argument order, required message/service *type* on the rclpy side,
queue_size -> QoS profile) that a thin wrapper would hide real per-call-site
decisions (queue depth -> QoS reliability/depth, latch=True -> QoS
durability=TRANSIENT_LOCAL, etc.) that deserve a human/reviewer's eyes.
Those call sites use ``node_handle.get_node().create_publisher(...)`` /
``create_subscription(...)`` / ``create_service(...)`` / ``create_client(...)``
directly, with the conversion made explicit at each site.

Anything below marked "no direct rclpy equivalent" is a best-effort stand-in
-- grep for its name in ``ROS2_MIGRATION_NOTES.md`` for the caveat.
"""
import time as _time
from typing import Any, Callable, Optional, Type

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node

from .node_handle import get_node, init_node, is_shutdown, shutdown  # noqa: F401  (re-exported)

# rospy.Time / rospy.Duration analogues. rclpy's Time/Duration are similar
# enough (nanoseconds-based, arithmetic works) that most call sites can use
# these directly; ``rclpy.time.Time`` objects are tied to a clock *type*
# (ROS_TIME vs SYSTEM_TIME) in a way rospy.Time never was -- if you see
# mismatched-clock-type errors at runtime, that's why.
Time = rclpy.time.Time
Duration = rclpy.duration.Duration


class ROSException(Exception):
    """Best-effort analogue of ``rospy.ROSException``.

    rclpy does not have one unified exception type for "something about
    ROS communication went wrong" -- it raises more specific exceptions
    (e.g. ``rclpy.service.ServiceException``, timeouts return False/None
    instead of raising). Call sites that used to do
    ``except rospy.ROSException:`` were left with a TODO(ros2) comment
    rather than guessing which specific exception(s) apply; this class
    exists so those call sites still parse and so a bare
    ``except ROSException`` doesn't silently swallow unrelated errors it
    didn't used to.
    """


class ROSInterruptException(Exception):
    """Best-effort analogue of ``rospy.ROSInterruptException`` (raised by
    rospy.sleep()/Rate.sleep() on shutdown). rclpy's ``time.sleep``-based
    :func:`sleep` below does not raise this on shutdown -- callers relying
    on that to break out of a sleep should instead check :func:`is_shutdown`
    themselves (see the TODO(ros2) left at any such call site).
    """


def _fmt(msg: Any, args: tuple) -> str:
    return msg % args if args else str(msg)


def loginfo(msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.loginfo``."""
    get_node().get_logger().info(_fmt(msg, args))


def logwarn(msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.logwarn``."""
    get_node().get_logger().warn(_fmt(msg, args))


def logerr(msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.logerr``."""
    get_node().get_logger().error(_fmt(msg, args))


def logdebug(msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.logdebug``."""
    get_node().get_logger().debug(_fmt(msg, args))


def logfatal(msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.logfatal``."""
    get_node().get_logger().fatal(_fmt(msg, args))


def loginfo_throttle(period_sec: float, msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.loginfo_throttle(period, msg)``."""
    get_node().get_logger().info(_fmt(msg, args), throttle_duration_sec=period_sec)


def logwarn_throttle(period_sec: float, msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.logwarn_throttle(period, msg)``."""
    get_node().get_logger().warn(_fmt(msg, args), throttle_duration_sec=period_sec)


def logerr_throttle(period_sec: float, msg: Any, *args: Any) -> None:
    """Analogue of ``rospy.logerr_throttle(period, msg)``."""
    get_node().get_logger().error(_fmt(msg, args), throttle_duration_sec=period_sec)


def sleep(duration_sec: float) -> None:
    """Analogue of ``rospy.sleep(seconds)``.

    Plain ``time.sleep`` -- does NOT respect simulated/``use_sim_time``
    clocks the way ``rospy.sleep`` did (rospy.sleep was ROS-clock-aware).
    Nothing in this codebase currently sets ``use_sim_time``, but if that
    changes, call sites here should switch to
    ``get_node().get_clock().sleep_for(Duration(seconds=duration_sec))``
    (rclpy >= Humble) or a rate/timer-based wait instead.
    """
    _time.sleep(duration_sec)


def now() -> rclpy.time.Time:
    """Analogue of ``rospy.Time.now()``."""
    return get_node().get_clock().now()


def Rate(hz: float):  # noqa: N802 - mirrors rospy.Rate's class-like call
    """Analogue of ``rospy.Rate(hz)``. Returns an ``rclpy.timer.Rate``."""
    return get_node().create_rate(hz)


def spin(node: Optional[Node] = None) -> None:
    """Analogue of ``rospy.spin()``."""
    rclpy.spin(node or get_node())


def wait_for_message(
    topic: str,
    msg_type: Type[Any],
    timeout: Optional[float] = None,
    node: Optional[Node] = None,
):
    """Best-effort analogue of ``rospy.wait_for_message(topic, msg_type,
    timeout)``.

    rclpy has no built-in "block until exactly one message arrives"
    helper (as of Jazzy). This creates a temporary subscription, spins the
    node until a message arrives or ``timeout`` elapses, then tears the
    subscription down. Uses default QoS -- if the publisher uses a
    non-default QoS (e.g. sensor data QoS / best-effort), pass a
    pre-built subscription instead, or extend this helper with a
    ``qos_profile`` argument.

    Raises :class:`ROSException` on timeout, matching rospy's behavior.
    """
    n = node or get_node()
    received: list = []

    def _cb(msg: Any) -> None:
        received.append(msg)

    sub = n.create_subscription(msg_type, topic, _cb, 10)
    start = _time.monotonic()
    try:
        while not received:
            rclpy.spin_once(n, timeout_sec=0.1)
            if timeout is not None and (_time.monotonic() - start) > timeout:
                raise ROSException(
                    f"timeout waiting for message on topic '{topic}'"
                )
    finally:
        n.destroy_subscription(sub)
    return received[0]


def wait_for_service(client, timeout: Optional[float] = None) -> bool:
    """Analogue of ``rospy.wait_for_service(service_name, timeout)``.

    Unlike rospy (which resolves a service by name alone), rclpy requires
    an already-constructed client (``node.create_client(SrvType, name)``)
    because the service *type* is needed up front. Call sites that used to
    do ``rospy.wait_for_service("/name")`` with no client nearby were left
    with a TODO(ros2) to construct one first.
    """
    return client.wait_for_service(timeout_sec=timeout)
