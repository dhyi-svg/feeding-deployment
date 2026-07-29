"""A single process-wide rclpy node, spun on a background thread.

ROS 1 code in this repo calls ``rospy.init_node`` once and then creates
publishers/subscribers freely from anywhere. rclpy has no such global: every
publisher, subscriber and tf2 buffer needs an explicit ``Node``, and something
must spin it. This module provides that missing global so the ported perception
code reads as closely as possible to the rospy original.

Usage::

    from feeding_deployment.ros2.node import get_node
    node = get_node()                    # inits rclpy + starts the executor once
    pub = node.create_publisher(...)

The executor runs in a daemon thread, so callbacks (camera frames, tf) keep
arriving while the executive blocks on a long arm motion -- matching how
rospy's own subscriber threads behaved.
"""

from __future__ import annotations

import atexit
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

_NODE: Node | None = None
_EXECUTOR: MultiThreadedExecutor | None = None
_THREAD: threading.Thread | None = None
_LOCK = threading.Lock()

DEFAULT_NODE_NAME = "feeding_deployment"


def get_node(name: str = DEFAULT_NODE_NAME) -> Node:
    """Return the shared node, initialising rclpy and the executor on first call.

    Safe to call from any thread and any number of times; only the first call
    does any work. ``name`` is only honoured on that first call.
    """
    global _NODE, _EXECUTOR, _THREAD  # pylint: disable=global-statement
    with _LOCK:
        if _NODE is not None:
            return _NODE

        if not rclpy.ok():
            rclpy.init()

        _NODE = Node(name)
        # Multi-threaded so a slow callback (e.g. a 30 s GroundingDINO pass
        # triggered from a subscriber) cannot stall camera/tf callbacks.
        _EXECUTOR = MultiThreadedExecutor()
        _EXECUTOR.add_node(_NODE)

        _THREAD = threading.Thread(
            target=_EXECUTOR.spin, name="rclpy-executor", daemon=True
        )
        _THREAD.start()

        atexit.register(shutdown)
        return _NODE


def node_is_running() -> bool:
    """True if the shared node has been created and rclpy is still up."""
    return _NODE is not None and rclpy.ok()


def shutdown() -> None:
    """Tear down the executor and node. Idempotent; safe from atexit."""
    global _NODE, _EXECUTOR, _THREAD  # pylint: disable=global-statement
    with _LOCK:
        if _EXECUTOR is not None:
            _EXECUTOR.shutdown()
            _EXECUTOR = None
        if _NODE is not None:
            _NODE.destroy_node()
            _NODE = None
        _THREAD = None
        if rclpy.ok():
            rclpy.shutdown()
