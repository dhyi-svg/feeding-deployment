"""ROS 1 <-> ROS 2 message adapters.

The perception code in this repo was written against ROS 1 message classes and
indexes camera intrinsics as ``camera_info.K[0]`` with timestamps at
``header.stamp.secs`` / ``.nsecs``. ROS 2 renamed both:

===========================  ==========================
ROS 1 (Noetic)               ROS 2 (Humble)
===========================  ==========================
``CameraInfo.K/D/R/P``       ``CameraInfo.k/d/r/p``
``Time.secs/.nsecs``         ``Time.sec/.nanosec``
===========================  ==========================

Rather than rewrite every detector (and fork them away from upstream), we adapt
at the boundary: the ROS 2 RealSense consumer wraps each ``CameraInfo`` in
``CameraInfoCompat``, which exposes the ROS 1 spelling while forwarding to the
live ROS 2 message. Detection math in ``appliance_perception`` and friends then
runs unmodified on both distros.

Everything here is read-only and allocation-light -- ``CameraInfoCompat`` holds a
reference, it does not copy the message.
"""

from __future__ import annotations


class StampCompat:
    """A ROS 2 ``builtin_interfaces/Time`` wearing ROS 1's field names.

    Exposes ``secs``/``nsecs`` (ROS 1) alongside ``sec``/``nanosec`` (ROS 2) so
    either spelling resolves.
    """

    __slots__ = ("_stamp",)

    def __init__(self, stamp) -> None:
        self._stamp = stamp

    @property
    def sec(self) -> int:
        return int(getattr(self._stamp, "sec", getattr(self._stamp, "secs", 0)))

    @property
    def nanosec(self) -> int:
        return int(
            getattr(self._stamp, "nanosec", getattr(self._stamp, "nsecs", 0))
        )

    # ROS 1 spelling.
    secs = sec
    nsecs = nanosec

    def __repr__(self) -> str:
        return f"StampCompat(sec={self.sec}, nanosec={self.nanosec})"


class HeaderCompat:
    """A ROS 2 ``std_msgs/Header`` with a ROS 1-compatible ``stamp``."""

    __slots__ = ("_header", "stamp")

    def __init__(self, header) -> None:
        self._header = header
        self.stamp = StampCompat(header.stamp)

    @property
    def frame_id(self) -> str:
        return self._header.frame_id

    @property
    def seq(self) -> int:
        # ROS 2 dropped Header.seq; detectors only ever log it.
        return int(getattr(self._header, "seq", 0))


class CameraInfoCompat:
    """A ROS 2 ``sensor_msgs/CameraInfo`` wearing ROS 1's field names.

    Provides ``K``/``D``/``R``/``P`` (ROS 1) on top of ROS 2's ``k``/``d``/``r``/
    ``p``, plus a ``header`` whose ``stamp`` answers to ``secs``/``nsecs``. Also
    exposes ``fx``/``fy``/``cx``/``cy`` so it can stand in for the repo's own
    :class:`~feeding_deployment.utils.camera_utils.CustomCameraInfo` wherever
    that is what a helper expects.
    """

    __slots__ = ("_msg", "header")

    def __init__(self, msg) -> None:
        self._msg = msg
        self.header = HeaderCompat(msg.header)

    # -- intrinsics, ROS 1 spelling ---------------------------------------
    @property
    def K(self):  # noqa: N802 -- deliberately ROS 1's name
        return self._intrinsic("k")

    @property
    def D(self):  # noqa: N802
        return self._intrinsic("d")

    @property
    def R(self):  # noqa: N802
        return self._intrinsic("r")

    @property
    def P(self):  # noqa: N802
        return self._intrinsic("p")

    def _intrinsic(self, lower: str):
        # Prefer the ROS 2 lowercase field; fall back to ROS 1's uppercase so a
        # ROS 1 message passed in here still works.
        value = getattr(self._msg, lower, None)
        if value is None:
            value = getattr(self._msg, lower.upper())
        return value

    # -- passthrough -------------------------------------------------------
    @property
    def width(self) -> int:
        return int(self._msg.width)

    @property
    def height(self) -> int:
        return int(self._msg.height)

    @property
    def distortion_model(self) -> str:
        return self._msg.distortion_model

    # -- CustomCameraInfo-compatible scalars -------------------------------
    @property
    def fx(self) -> float:
        return float(self.K[0])

    @property
    def fy(self) -> float:
        return float(self.K[4])

    @property
    def cx(self) -> float:
        return float(self.K[2])

    @property
    def cy(self) -> float:
        return float(self.K[5])

    @property
    def msg(self):
        """The wrapped ROS 2 message, for code that needs the real thing."""
        return self._msg

    def __repr__(self) -> str:
        return (
            f"CameraInfoCompat({self.width}x{self.height}, "
            f"fx={self.fx:.1f}, fy={self.fy:.1f}, "
            f"cx={self.cx:.1f}, cy={self.cy:.1f})"
        )


def stamp_to_sec_nanosec(stamp) -> tuple[int, int]:
    """``(sec, nanosec)`` from either a ROS 1 or ROS 2 time message."""
    sec = getattr(stamp, "sec", None)
    if sec is None:
        sec = getattr(stamp, "secs", 0)
    nanosec = getattr(stamp, "nanosec", None)
    if nanosec is None:
        nanosec = getattr(stamp, "nsecs", 0)
    return int(sec), int(nanosec)
