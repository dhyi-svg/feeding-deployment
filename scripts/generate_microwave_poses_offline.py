"""Run the REAL ``PerceptionInterface.perceive_handle_opening_poses("microwave")``
offline, with the detector stubbed, and write ``handle_opening_pos.pkl``.

Why this exists
---------------
``perceive_handle_opening_poses`` is where the microwave skill's geometry
actually lives: the grasp/pre-grasp offsets, the door arc, the push and closing
waypoint families, and the top-of-appliance z clamping. On hardware it sits
behind a camera, GroundingDINO and tf2, so it is normally only exercised with
the robot powered and the microwave in front of it.

Everything downstream of ``detect_handle_and_placement`` is pure geometry. So we
supply the four poses that detector returns and run the genuine method -- no
reimplementation, no copy of the arc code (``scripts/draw_microwave_waypoints.py``
keeps a deliberate verbatim copy for drawing; this instead calls the real thing,
so it catches drift between the two).

The pickle it writes is exactly what the HLA replay path consumes, so::

    python scripts/generate_microwave_poses_offline.py --log_dir <dir>
    python scripts/validate_open_microwave_hla_sim.py --log_dir <dir>

runs the real ``OpenDoorHLA.open_microwave()`` against real perception geometry
with no hardware at all.

The default handle pose is a plausible microwave standing in front of the arm,
NOT a measured ground truth -- pass ``--handle-pos`` to use a real one.
"""

import argparse
import pickle
from types import SimpleNamespace
from pathlib import Path

import numpy as np
from pybullet_helpers.geometry import Pose

from feeding_deployment.interfaces.perception_interface import PerceptionInterface

# Fixed microwave-handle grasp orientation, as detect_handle_and_placement
# returns it (appliance_perception.py).
GRASP_QUAT = (-0.5, 0.5, 0.5, -0.5)

# A plausible left-hinged microwave in front of the arm, in arm_base_link.
# Assumption, not a measurement -- override with --handle-pos.
DEFAULT_HANDLE_POS = (0.60, -0.10, 0.48)
# Door width: the hinge sits one door-width to +y of the handle (left-hinged).
DEFAULT_DOOR_WIDTH = 0.32
# How far above the handle the top of the appliance sits.
DEFAULT_TOP_OF_APPLIANCE_DZ = 0.12


class _StubApplianceDetector:
    """Stands in for AppliancePerception, returning fixed detection poses."""

    def __init__(self, handle_pose, hinge_pose, placement_pose, top_pose):
        self._poses = (handle_pose, hinge_pose, placement_pose, top_pose)
        self._last_images = {}

    def detect_handle_and_placement(self, handle_type, rgb_image, camera_info, depth_image):
        del handle_type, rgb_image, camera_info, depth_image
        return self._poses


class _StubRealSense:
    """A camera that always has a frame, so the detection retry loop exits.

    ``camera_info`` must be non-None (the caller treats a missing one as a dead
    stream); the stub detector never reads it, so nominal 640x480 intrinsics
    are enough.
    """

    @staticmethod
    def get_camera_data():
        camera_info = SimpleNamespace(
            K=[600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0],
            D=[0.0] * 5,
            width=640,
            height=480,
            header=SimpleNamespace(
                frame_id="camera_color_optical_frame",
                stamp=SimpleNamespace(secs=0, nsecs=0),
            ),
        )
        return {
            "rgb_image": np.zeros((480, 640, 3), dtype=np.uint8),
            "camera_info": camera_info,
            "depth_image": np.zeros((480, 640), dtype=np.float32),
            "header": camera_info.header,
        }


def build_perception_interface(handle_pos, door_width, top_dz, log_dir):
    """A PerceptionInterface with only what perceive_handle_opening_poses touches.

    Built with ``__new__`` so no camera, no models, no ROS and no arm are
    constructed -- the real method still runs, against stubbed detections.
    """
    handle_pose = Pose(position=tuple(handle_pos), orientation=GRASP_QUAT)
    hinge_pose = Pose(
        position=(handle_pos[0], handle_pos[1] + door_width, handle_pos[2]),
        orientation=GRASP_QUAT,
    )
    # Where a plate would be set down inside; only its offsets are exercised.
    placement_pose = Pose(
        position=(handle_pos[0] + 0.20, handle_pos[1] + door_width / 2.0, handle_pos[2]),
        orientation=GRASP_QUAT,
    )
    top_pose = Pose(
        position=(handle_pos[0], handle_pos[1], handle_pos[2] + top_dz),
        orientation=GRASP_QUAT,
    )

    perception = PerceptionInterface.__new__(PerceptionInterface)
    perception.simulation = False
    perception.log_dir = log_dir
    perception.data_logger = None
    perception.last_handle_poses = None
    perception._realsense = _StubRealSense()
    perception._appliance_perception = _StubApplianceDetector(
        handle_pose, hinge_pose, placement_pose, top_pose
    )
    # Auto-confirm: there is no operator and no webapp in an offline run.
    perception._terminal_confirmation = lambda detection_type, vis_image=None: True
    # RViz sync is a live-visualisation side effect; nothing to sync offline.
    perception.sync_rviz = lambda: None
    return perception


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--log_dir",
        type=Path,
        default=Path("src/feeding_deployment/integration/log/microwave_hla_sim_validation"),
        help="where handle_opening_pos.pkl is written",
    )
    parser.add_argument(
        "--handle-pos",
        type=float,
        nargs=3,
        default=list(DEFAULT_HANDLE_POS),
        metavar=("X", "Y", "Z"),
        help="microwave handle position in arm_base_link (default is a plausible pose, not a measurement)",
    )
    parser.add_argument("--door-width", type=float, default=DEFAULT_DOOR_WIDTH)
    parser.add_argument("--top-dz", type=float, default=DEFAULT_TOP_OF_APPLIANCE_DZ)
    args = parser.parse_args()

    log_dir = args.log_dir
    log_dir.mkdir(parents=True, exist_ok=True)

    perception = build_perception_interface(
        args.handle_pos, args.door_width, args.top_dz, log_dir
    )

    print(f"Handle pose (arm_base_link): {tuple(args.handle_pos)}")
    print("Running the real perceive_handle_opening_poses('microwave') ...")
    poses = perception.perceive_handle_opening_poses("microwave")

    print(f"\nProduced {len(poses)} pose entries:")
    for name, value in poses.items():
        if isinstance(value, list):
            print(f"  {name:34s} {len(value)} waypoints")
        else:
            pos = np.asarray(value.position)
            print(f"  {name:34s} [{pos[0]: .3f}, {pos[1]: .3f}, {pos[2]: .3f}]")

    out = log_dir / "handle_opening_pos.pkl"
    # perceive_handle_opening_poses already wrote this when log_dir is set;
    # verify rather than assume.
    if not out.exists():
        with open(out, "wb") as f:
            pickle.dump({"last_handle_poses": poses}, f)
    print(f"\nWrote {out}")
    print("Next: python scripts/validate_open_microwave_hla_sim.py --log_dir", log_dir)


if __name__ == "__main__":
    main()
