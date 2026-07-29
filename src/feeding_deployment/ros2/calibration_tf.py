"""Publish the saved easy_handeye2 hand-eye calibration as a ROS 2 static transform.

``easy_handeye2`` stores its result as a small YAML file under
``~/.ros2/easy_handeye2/calibrations/<name>.calib``::

    parameters:
      calibration_type: eye_in_hand
      robot_base_frame: base_link
      robot_effector_frame: end_effector_link
      tracking_base_frame: camera_color_optical_frame
    transform:
      translation: {x: ..., y: ..., z: ...}
      rotation:    {x: ..., y: ..., z: ..., w: ...}

For an **eye-in-hand** calibration the transform is
``robot_effector_frame -> tracking_base_frame`` (i.e. where the camera sits on
the wrist), so publishing it static under ``end_effector_link`` completes the
chain ``base_link -> ... -> end_effector_link -> camera_color_optical_frame``.
Combined with the joint-state bridge + ``robot_state_publisher``, tf2 can then
answer the ``arm_base_link -> camera_color_optical_frame`` lookup the repo's
detectors make.

This also publishes an identity ``base_link -> arm_base_link`` alias, because
the repo's perception and RViz code names the arm base ``arm_base_link`` while
the Kinova description calls it ``base_link``.

Run standalone::

    python -m feeding_deployment.ros2.calibration_tf
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from feeding_deployment.ros2.node import get_node

DEFAULT_CALIB_PATH = (
    Path.home() / ".ros2" / "easy_handeye2" / "calibrations" / "wrist_camera_calib.calib"
)

# The repo names the arm base "arm_base_link"; kortex_description calls it
# "base_link". Published as identity so both names resolve.
REPO_ARM_BASE_FRAME = "arm_base_link"


class CalibrationLoadError(RuntimeError):
    """The calibration file is missing or does not have the expected shape."""


def load_calibration(path: Path | str = DEFAULT_CALIB_PATH) -> dict:
    """Parse an easy_handeye2 ``.calib`` file into a plain dict.

    Returns keys ``parent_frame``, ``child_frame``, ``translation`` (xyz) and
    ``rotation`` (xyzw), already resolved for the calibration type.
    """
    path = Path(path).expanduser()
    if not path.is_file():
        raise CalibrationLoadError(
            f"No hand-eye calibration at {path}. Run easy_handeye2, or pass "
            f"--calib with the correct path."
        )

    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    try:
        params = data["parameters"]
        transform = data["transform"]
        translation = transform["translation"]
        rotation = transform["rotation"]
    except (KeyError, TypeError) as e:
        raise CalibrationLoadError(f"{path} is not an easy_handeye2 calibration: {e}") from e

    calibration_type = params.get("calibration_type", "eye_in_hand")
    if calibration_type == "eye_in_hand":
        # Camera is mounted on the wrist: effector -> camera.
        parent_frame = params["robot_effector_frame"]
    elif calibration_type == "eye_on_base":
        # Camera is fixed in the world: base -> camera.
        parent_frame = params["robot_base_frame"]
    else:
        raise CalibrationLoadError(f"Unknown calibration_type {calibration_type!r} in {path}")

    return {
        "calibration_type": calibration_type,
        "parent_frame": parent_frame,
        "child_frame": params["tracking_base_frame"],
        "translation": (
            float(translation["x"]),
            float(translation["y"]),
            float(translation["z"]),
        ),
        "rotation": (
            float(rotation["x"]),
            float(rotation["y"]),
            float(rotation["z"]),
            float(rotation["w"]),
        ),
        "path": str(path),
    }


def _make_transform(node, parent: str, child: str, xyz, quat_xyzw) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (
        float(v) for v in xyz
    )
    (
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w,
    ) = (float(v) for v in quat_xyzw)
    return t


def publish_calibration_tf(
    calib_path: Path | str = DEFAULT_CALIB_PATH,
    publish_arm_base_alias: bool = True,
    repeat_hz: float = 20.0,
):
    """Publish the hand-eye calibration (and the arm-base alias) as TF.

    Sends them once on ``/tf_static`` AND re-sends them on ``/tf`` at
    ``repeat_hz`` with fresh timestamps.

    The repeat is not redundancy for its own sake. ``/tf_static`` is latched
    (TRANSIENT_LOCAL), which in principle serves late-joining listeners -- but
    with several independent static publishers on the topic (here:
    robot_state_publisher plus this node) a listener can end up with only some
    publishers' transforms and report "two or more unconnected trees". That
    happened live on 2026-07-29 and killed a detection mid-run. Re-sending on
    ``/tf`` is what actually guarantees delivery, and the fresh stamps also stop
    lookups at a recent camera timestamp from failing with "extrapolation into
    the past".

    Returns ``(static_broadcaster, dynamic_broadcaster)``; keep a reference to
    both for as long as the transforms should stay published.
    """
    calib = load_calibration(calib_path)
    node = get_node()
    broadcaster = StaticTransformBroadcaster(node)

    transforms = [
        _make_transform(
            node,
            calib["parent_frame"],
            calib["child_frame"],
            calib["translation"],
            calib["rotation"],
        )
    ]
    if publish_arm_base_alias:
        transforms.append(
            _make_transform(
                node, "base_link", REPO_ARM_BASE_FRAME, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
            )
        )

    broadcaster.sendTransform(transforms)

    # Re-send on /tf with current stamps. See the docstring: latched /tf_static
    # alone has proved unreliable with multiple publishers on this rig.
    dynamic_broadcaster = TransformBroadcaster(node)
    specs = [
        (t.header.frame_id, t.child_frame_id,
         (t.transform.translation.x, t.transform.translation.y, t.transform.translation.z),
         (t.transform.rotation.x, t.transform.rotation.y,
          t.transform.rotation.z, t.transform.rotation.w))
        for t in transforms
    ]

    def _republish():
        dynamic_broadcaster.sendTransform(
            [_make_transform(node, parent, child, xyz, quat) for parent, child, xyz, quat in specs]
        )

    node.create_timer(1.0 / repeat_hz, _republish)

    node.get_logger().info(
        f"Published {calib['calibration_type']} calibration from {calib['path']}: "
        f"{calib['parent_frame']} -> {calib['child_frame']}"
        + (f", plus base_link -> {REPO_ARM_BASE_FRAME}" if publish_arm_base_alias else "")
        + f" (static, and repeated on /tf at {repeat_hz:g} Hz)"
    )
    return broadcaster, dynamic_broadcaster


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calib", default=str(DEFAULT_CALIB_PATH))
    parser.add_argument(
        "--no-arm-base-alias",
        action="store_true",
        help="skip the identity base_link -> arm_base_link transform",
    )
    args = parser.parse_args()

    node = get_node()
    # Held for the process lifetime: a StaticTransformBroadcaster that goes out
    # of scope stops latching its transforms.
    _broadcasters = publish_calibration_tf(  # noqa: F841
        args.calib, publish_arm_base_alias=not args.no_arm_base_alias
    )
    node.get_logger().info("Static calibration TF latched; Ctrl-C to stop.")
    try:
        while rclpy.ok():
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
