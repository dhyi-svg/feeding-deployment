"""Keep the detector resident so scripts stop paying ~6 min of startup each run.

Measured on the Jetson (2026-08-11): imports 43 s, GroundedSAM() 42 s warm / ~110 s
cold, scene+sim 10 s, and ~2.4 GB of libraries and weights paged off a 13.5 MB/s
microSD -- roughly 3 min of disk before any compute. All of it is per-invocation and
none of it is per-detection, so a dry run followed by an execute pays it twice.

Server holds the model and the camera subscriber; clients ask for a detection over the
same multiprocessing-manager RPC the arm already uses. Detection maths is untouched --
this calls the real AppliancePerception.detect_handle_and_placement.
"""
import os
import time
from multiprocessing.managers import BaseManager

HOST = os.environ.get("DETECTION_RPC_HOST", "127.0.0.1")
PORT = int(os.environ.get("DETECTION_RPC_PORT", "5055"))
AUTHKEY = b"feeding_deployment_detection"


def _frame_age_sec(data):
    """Seconds since the frame's header stamp, or None when unstamped."""
    stamp = getattr(data.get("header"), "stamp", None)
    if stamp is None:
        return None
    sec = getattr(stamp, "sec", getattr(stamp, "secs", 0))
    nsec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", 0))
    return time.time() - (sec + nsec * 1e-9)


class DetectionService:
    """Loads the detector once; answers detect() calls for the process's lifetime."""

    def __init__(self, log_dir: str | None = None) -> None:
        from feeding_deployment.perception.appliance_perception.appliance_perception import (
            AppliancePerception,
        )
        from feeding_deployment.perception.grounded_sam import GroundedSAM
        from feeding_deployment.ros2.realsense_ros2_interface import RealSenseROS2Interface

        data_logger = None
        if log_dir:
            from pathlib import Path

            from feeding_deployment.integration.data_logger import DataLogger

            data_logger = DataLogger(Path(log_dir), day=1)
            data_logger.begin_hla("detection_server")

        self._rs = RealSenseROS2Interface()
        if not self._rs.wait_for_frames(30.0):
            raise RuntimeError("No RGB-D frames -- is microwave_bringup running?")
        self._apc = AppliancePerception(GroundedSAM(), data_logger=data_logger)

    def corrections(self) -> tuple:
        return (self._apc.handle_depth_corr, self._apc.handle_lat_corr)

    def detect(self, handle_type: str = "microwave handle", max_frame_age_sec: float = 5.0):
        """One detection on a fresh frame. Plain tuples so nothing exotic is pickled.

        Nothing positional is cached -- the frame and the tf lookup are both live, so
        moving the arm or the appliance is tracked. Only the model weights persist.
        """
        # get_camera_data returns the LAST frame forever if the camera stops. A
        # detection also holds the GIL for ~60 s, so the rclpy executor thread cannot
        # deliver frames while one runs and the stored frame is always stale straight
        # afterwards. Poll for a fresh one before deciding the camera is really dead.
        deadline = time.time() + max(max_frame_age_sec, 10.0)
        while True:
            data = self._rs.get_camera_data()
            age = _frame_age_sec(data)
            if age is None or age <= max_frame_age_sec:
                break
            if time.time() > deadline:
                from feeding_deployment.ros2.node import executor_alive

                why = ("the rclpy executor thread is DEAD -- restart the detection "
                       "server" if not executor_alive() else
                       "the camera has stopped publishing")
                raise RuntimeError(
                    f"Camera frame is {age:.1f}s old (limit {max_frame_age_sec}s) -- "
                    f"{why}. Refusing a stale detection."
                )
            time.sleep(0.1)
        handle, _hinge, _placement, top = self._apc.detect_handle_and_placement(
            handle_type, data["rgb_image"], data["camera_info"], data["depth_image"]
        )
        if handle is None:
            return None
        top_z = float(top.position[2]) if top is not None else float("nan")
        return {
            "position": tuple(float(v) for v in handle.position),
            "orientation": tuple(float(v) for v in handle.orientation),
            "top_z": top_z,
        }


class DetectionManager(BaseManager):
    pass


def serve(log_dir: str | None = None) -> None:
    """Load the detector, then block serving requests."""
    service = DetectionService(log_dir=log_dir)
    DetectionManager.register("DetectionService", callable=lambda: service)
    manager = DetectionManager(address=(HOST, PORT), authkey=AUTHKEY)
    server = manager.get_server()
    print(f"detection server ready on {HOST}:{PORT} -- model stays loaded. Ctrl-C to stop.",
          flush=True)
    server.serve_forever()


def connect(required: bool = False):
    """Return a service proxy, or None when no server is running."""
    DetectionManager.register("DetectionService")
    manager = DetectionManager(address=(HOST, PORT), authkey=AUTHKEY)
    try:
        manager.connect()
    except Exception as e:  # noqa: BLE001 -- absence is a normal, expected state
        if required:
            raise RuntimeError(f"No detection server on {HOST}:{PORT}: {e}") from None
        return None
    return manager.DetectionService()
