"""Shared rclpy Node singleton bridging this codebase's rospy-style implicit
global node usage to ROS 2's explicit Node object model.

Background
----------
ROS 1 idiom used pervasively in this codebase: some entry-point script calls
``rospy.init_node(name)`` once, and then *any* other file -- classes,
helpers, whatever -- freely calls bare ``rospy.Publisher(...)``,
``rospy.get_param(...)``, ``rospy.loginfo(...)``, etc. with no node handle
passed around. There is exactly one implicit global node per process and
everyone just reaches for it.

rclpy has no implicit global node: every publisher, subscriber, timer,
parameter, logger, and clock is a method on an explicit ``Node`` instance,
and ``rclpy.init()`` must run before any ``Node`` is constructed.

Chosen strategy: a lazily-created, per-process **singleton** ``rclpy.Node``,
handed out by :func:`get_node`. Call :func:`init_node` once near your
script's real entry point (mirrors ``rospy.init_node(name)``) to give the
node a meaningful name; everything else can just call :func:`get_node` the
same way it used to reach for the ``rospy`` module directly.

Why a singleton instead of threading a ``Node``/``self.node`` through every
constructor in ~50 files (the more "idiomatic modern rclpy" approach): this
is a migration of a large, working, hardware-adjacent codebase, not a
rewrite. Threading an explicit Node parameter through every HLA, interface,
controller, and perception class's ``__init__`` would touch call sites far
outside the 52 files that actually import ``rospy`` today (anything that
constructs one of these objects), and would make the diff far harder to
review file-by-file or to bisect if something regresses near the real arm.
The singleton keeps each file's diff local and mechanical: "s/rospy/node_handle
or rospy_compat/", basically.

Tradeoffs, on the record:
  - This does NOT support multiple distinct nodes coexisting in one
    process. Nothing in this codebase currently needs that (each
    entry-point script -- arm_server.py, run.py, individual safety
    daemons -- is its own OS process already), but it's a real constraint
    if that ever changes.
  - It is not the pattern the ROS 2 docs recommend for new code. If/when
    pieces of this codebase get rewritten (rather than migrated) -- e.g.
    turning long-running daemons into proper composable-node components --
    switching that piece to an explicitly-constructed, explicitly-spun
    ``Node`` created in its own ``main()`` is the natural next step, and
    should be a small change since ``get_node()`` calls are easy to grep
    for and replace with ``self.node``.
  - Logging before ``init_node()``/``get_node()`` has been called anywhere
    will silently create a node named "feeding_deployment_node" rather than
    erroring the way an un-init'd ``rospy`` call would (rospy raises).
    This is a deliberate leniency to avoid ordering-sensitive crashes
    during the migration; grep for ``init_node(`` calls if a node's name
    looks wrong in logs.

See ``rospy_compat.py`` in this same package for logging/time/sleep/spin
wrappers built on top of this module that mirror the rospy call surface
more closely, and ``ROS2_MIGRATION_NOTES.md`` at the repo root for the
full writeup.
"""
import atexit
import threading
from typing import Any, Optional

import rclpy
from rclpy.node import Node

_lock = threading.Lock()
_node: Optional[Node] = None
_owns_rclpy_init = False


def init_node(name: str = "feeding_deployment_node", **kwargs: Any) -> Node:
    """ROS 2 analogue of ``rospy.init_node(name)``.

    Idempotent within a process: the first caller's ``name`` wins; later
    calls (including the lazy one inside :func:`get_node`) just return the
    already-created shared node, same as calling ``rospy.init_node`` twice
    in one process was already a no-op/warning in ROS 1.

    Also calls ``rclpy.init()`` if it hasn't been called yet in this
    process. Safe to call from a script that is also a library import
    elsewhere (mirrors rospy's tolerance of this).
    """
    global _node, _owns_rclpy_init  # pylint: disable=global-statement
    with _lock:
        if _node is not None:
            return _node
        if not rclpy.ok():
            rclpy.init(args=None)
            _owns_rclpy_init = True
        _node = rclpy.create_node(name, **kwargs)
        atexit.register(_shutdown)
        return _node


def get_node() -> Node:
    """ROS 2 analogue of "just use ``rospy.X()`` from anywhere".

    Lazily creates a default-named node if :func:`init_node` was never
    called explicitly. Prefer calling ``init_node(name)`` once near your
    script's entry point so the node gets a meaningful name for logging/
    introspection (``ros2 node list``, etc.).
    """
    if _node is None:
        return init_node()
    return _node


def has_node() -> bool:
    """True if a shared node has already been created in this process."""
    return _node is not None


def is_shutdown() -> bool:
    """ROS 2 analogue of ``rospy.is_shutdown()``."""
    return not rclpy.ok()


def _shutdown() -> None:
    global _node, _owns_rclpy_init  # pylint: disable=global-statement
    with _lock:
        if _node is not None:
            try:
                _node.destroy_node()
            except Exception:  # pylint: disable=broad-except
                pass
            _node = None
        if _owns_rclpy_init and rclpy.ok():
            rclpy.shutdown()
            _owns_rclpy_init = False


def shutdown() -> None:
    """ROS 2 analogue of ``rospy.signal_shutdown(reason)`` / node teardown.

    Safe to call more than once (also registered via ``atexit``).
    """
    _shutdown()
