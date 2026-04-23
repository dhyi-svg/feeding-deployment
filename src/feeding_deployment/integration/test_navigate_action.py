"""Run a focused NavigateHLA test to a named location."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from feeding_deployment.actions.navigate import NavigateHLA
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.simulation.scene_description import (
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator


def test_navigate_action(
    scene_config: str,
    transfer_type: str,
    run_on_robot: bool,
    location: str,
    speed: str,
    use_gui: bool,
) -> None:
    """Instantiate NavigateHLA and navigate to a named location."""
    if run_on_robot:
        robot_interface = ArmInterfaceClient()  # type: ignore  # pylint: disable=no-member
    else:
        robot_interface = None

    log_dir = Path(__file__).parent / "log" / "test_navigate_action"
    log_dir.mkdir(parents=True, exist_ok=True)

    scene_config_path = (
        Path(__file__).parent.parent / "simulation" / "configs" / f"{scene_config}.yaml"
    )
    scene_description = create_scene_description_from_config(
        str(scene_config_path),
        transfer_type,
    )
    sim = FeedingDeploymentPyBulletSimulator(scene_description, use_gui=use_gui)

    hla_hyperparams = {"max_motion_planning_time": 10.0}
    navigate_hla = NavigateHLA(
        sim,
        robot_interface,
        None,
        None,
        None,
        hla_hyperparams,
        None,
        None,
        False,
        log_dir,
        log_dir,
        log_dir / "execution_log.txt",
        log_dir,
    )

    pose = navigate_hla._load_target_pose(location)
    print(
        "Loaded named location "
        f"'{location}' from {navigate_hla._location_yaml()} "
        f"(frame={pose['frame_id']}, "
        f"position=({pose['x']:.4f}, {pose['y']:.4f}, {pose['z']:.4f}))."
    )

    navigate_hla._navigate_to_target(location, speed)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene_config", type=str, default="vention")
    parser.add_argument("--transfer_type", type=str, default="outside")
    parser.add_argument("--run_on_robot", action="store_true")
    parser.add_argument("--use_gui", action="store_true")
    parser.add_argument("--location", type=str, default="microwave")
    parser.add_argument("--speed", type=str, default="medium")
    parser.add_argument(
        "--locations_file",
        type=str,
        default="",
        help="Optional override for FEEDING_NAV_LOCATIONS_FILE.",
    )
    args = parser.parse_args()

    if args.speed not in {"low", "medium", "high"}:
        raise ValueError(
            f"Invalid speed '{args.speed}'. Must be one of: low, medium, high."
        )

    if args.locations_file:
        os.environ["FEEDING_NAV_LOCATIONS_FILE"] = args.locations_file

    test_navigate_action(
        scene_config=args.scene_config,
        transfer_type=args.transfer_type,
        run_on_robot=args.run_on_robot,
        location=args.location,
        speed=args.speed,
        use_gui=args.use_gui,
    )