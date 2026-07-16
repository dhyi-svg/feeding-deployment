"""Run a focused NavigateHLA test to a named location.

Standalone nav-offset loop (no preference pipeline): the PositionOffset stored
in the navigate_to_<location>.yaml behavior tree is read and applied to the
goal, and after arrival you can teleop the robot to the exact desired pose and
have the new TOTAL offset written straight back to that YAML -- the next run
then navigates to the corrected pose. Same measurement/write-back mechanism as
the deployment's web-interface adjust prompt, minus the iPad.
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
from pathlib import Path

import yaml

from relational_structs import Object

from feeding_deployment.actions.base import nav_target_type
from feeding_deployment.actions.navigate import NavigateHLA
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.simulation.scene_description import (
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import (
    FeedingDeploymentPyBulletSimulator,
    NullSimulator,
)

# src/feeding_deployment/actions/behavior_trees -- the same factory source
# run.py seeds new users from (Path(__file__).parents[1] / "actions" / ...).
_FACTORY_BT_DIR = Path(__file__).parent.parent / "actions" / "behavior_trees"


def _read_position_offset(bt_dir: Path, location: str) -> list[float] | None:
    """PositionOffset [dx, dy, dyaw] from navigate_to_<location>.yaml, or None
    if the file/param is missing (treated as zero offset by the HLA)."""
    fpath = bt_dir / f"navigate_to_{location}.yaml"
    if not fpath.exists():
        print(f"[nav-offset] {fpath} not found; navigating with zero offset.")
        return None
    with open(fpath, "r", encoding="utf-8") as f:
        # The file contains a !hla tag the safe loader doesn't know; we only
        # need the parameters block, so blank the tag before parsing.
        data = yaml.safe_load(f.read().replace("!hla", "")) or {}
    for param in data.get("parameters", []):
        if param.get("name") == "PositionOffset":
            return [float(v) for v in param.get("value", [0.0, 0.0, 0.0])]
    print(f"[nav-offset] no PositionOffset in {fpath.name}; navigating with zero offset.")
    return None


def _terminal_adjustment(
    hla: NavigateHLA,
    location: str,
    nominal_pose: dict,
    commanded_pose: dict,
    prev_offset: list[float] | None,
) -> None:
    """Terminal replacement for the deployment's iPad adjust prompt.

    Teleop the base (e.g. vention_teleop_keyboard.py or the Xbox node in a
    separate terminal), press Enter, and the measured TOTAL offset (final pose
    vs the NOMINAL mapped goal) is written back to the behavior tree.
    """
    pose_before = hla._read_base_pose_se2()
    if pose_before is None:
        print("[nav-offset] localization stale; cannot measure an adjustment.")
        return

    answer = input(
        f"\nAdjust the robot's position at '{location}'? "
        "Teleop it to the desired pose, then confirm. [y/N] "
    ).strip().lower()
    if answer not in ("y", "yes"):
        print("[nav-offset] position accepted as-is; offset unchanged.")
        return

    input(
        "Drive the robot to the desired pose now (keyboard/joystick teleop), "
        "then press Enter to measure... "
    )

    hla._wait_for_localization_settle(
        hla._ADJUST_SETTLE_S, "measuring the adjusted pose"
    )
    pose_after = hla._read_base_pose_se2()
    if pose_after is None:
        print("[nav-offset] localization stale after the adjustment; not updating.")
        return

    nominal_se2 = hla._pose_dict_se2(nominal_pose)
    commanded_se2 = hla._pose_dict_se2(commanded_pose)
    movement = hla._se2_relative(*pose_before, *pose_after)
    total = hla._se2_relative(*nominal_se2, *pose_after)

    if (
        math.hypot(movement[0], movement[1]) < hla._MIN_ADJUST_XY_M
        and abs(movement[2]) < hla._MIN_ADJUST_YAW_RAD
    ):
        print(
            f"[nav-offset] movement below the {hla._MIN_ADJUST_XY_M * 100:.0f}cm/"
            f"{math.degrees(hla._MIN_ADJUST_YAW_RAD):.0f}deg threshold; not updating."
        )
        return

    clamped = [
        hla._clamp(total[0], hla._MAX_OFFSET_XY_M),
        hla._clamp(total[1], hla._MAX_OFFSET_XY_M),
        hla._clamp(total[2], hla._MAX_OFFSET_YAW_RAD),
    ]
    if clamped != list(total):
        print(
            "[nav-offset] total offset saturated at the bounds; consider "
            "re-capturing the named location instead."
        )

    objects = (Object(location, nav_target_type), Object(location, nav_target_type))
    node_name = f"NavigateTo{location.capitalize()}"
    result = hla.process_behavior_tree_parameter_update(
        objects, {}, node_name, "PositionOffset",
        [float(clamped[0]), float(clamped[1]), float(clamped[2])],
    )
    print(
        f"[nav-offset] user moved dx={movement[0] * 100:+.1f}cm "
        f"dy={movement[1] * 100:+.1f}cm dyaw={math.degrees(movement[2]):+.1f}deg; "
        f"new total offset [{clamped[0]:+.3f}, {clamped[1]:+.3f}, {clamped[2]:+.3f}]. "
        f"BT update: {result}"
    )
    outcome = "updated" if str(result).startswith("Success") else "bt_write_failed"
    hla._log_offset_event(
        location, outcome,
        nominal_se2, commanded_se2, pose_before, pose_after, prev_offset,
        total_offset=clamped,
    )


def test_navigate_action(
    scene_config: str,
    transfer_type: str,
    run_on_robot: bool,
    location: str,
    speed: str,
    use_gui: bool,
    behavior_tree_dir: Path | None,
    ignore_offset: bool,
    adjust: bool,
    use_interface: bool,
    no_waits: bool,
    assume_from: str | None = None,
) -> None:
    """Instantiate NavigateHLA and navigate to a named location."""
    rospy_mod = None
    if run_on_robot or use_interface:
        # Mirror run.py: the node must exist BEFORE ArmInterfaceClient, whose
        # constructor blocks on rospy.wait_for_message("/watchdog_status", ...)
        # -- without an initialized node that wait never connects and hangs.
        # disable_signals=True (unlike run.py, which lives in rospy.spin())
        # keeps Ctrl+C raising a normal KeyboardInterrupt in this script.
        import rospy

        rospy_mod = rospy
        if not rospy.core.is_initialized():
            rospy.init_node(
                "test_navigate_action", anonymous=True, disable_signals=True
            )

    if run_on_robot:
        robot_interface = ArmInterfaceClient()  # type: ignore  # pylint: disable=no-member
    else:
        robot_interface = None

    log_dir = Path(__file__).parent / "log" / "test_navigate_action"
    log_dir.mkdir(parents=True, exist_ok=True)

    # Working behavior-tree copy, mirroring run.py's new-user seeding: the
    # factory trees are copied once into the test's log dir and all reads and
    # offset write-backs happen against the copy -- accumulated offsets persist
    # across test runs without mutating the repo's factory YAMLs. Delete the
    # dir to re-seed from factory. An explicitly passed --behavior_tree_dir
    # (e.g. a deployment user's log/<user>/behavior_trees) is used as-is.
    if behavior_tree_dir is None:
        behavior_tree_dir = log_dir / "behavior_trees"
        if not behavior_tree_dir.exists():
            behavior_tree_dir.mkdir(parents=True)
            assert _FACTORY_BT_DIR.exists()
            for original_bt_filename in _FACTORY_BT_DIR.glob("*.yaml"):
                shutil.copy(original_bt_filename, behavior_tree_dir)
            print(f"[nav-offset] seeded behavior trees from factory into {behavior_tree_dir}")
    elif not behavior_tree_dir.exists():
        raise FileNotFoundError(
            f"--behavior_tree_dir {behavior_tree_dir} does not exist; pass an "
            "existing (seeded) directory or omit the flag to use the test copy."
        )

    web_interface = None
    if use_interface:
        # Full deployment interface path: the post-arrival adjust prompt shows
        # on the iPad (Position OK / Adjust -> navigation teleop screen), and
        # nav failures get the same recovery teleop as a real meal. Requires
        # the rosbridge/webapp stack to be up with a client connected (the
        # feeding tmux scripts), otherwise the page handshakes block forever.
        # The rospy node was already initialized above (before
        # ArmInterfaceClient); imports stay local so sim-only runs never need
        # ROS.
        import queue

        from feeding_deployment.integration.data_logger import DataLogger
        from feeding_deployment.interfaces.web_interface import WebInterface

        web_interface = WebInterface(queue.Queue(), DataLogger(log_dir))

    scene_config_path = (
        Path(__file__).parent.parent / "simulation" / "configs" / f"{scene_config}.yaml"
    )
    scene_description = create_scene_description_from_config(
        str(scene_config_path),
        transfer_type,
    )
    # Mirror run.py: --no_waits swaps in the NullSimulator (no PyBullet) and
    # suppresses interactive waits, including the post-arrival adjust prompts.
    if no_waits:
        sim = NullSimulator(scene_description)
    else:
        sim = FeedingDeploymentPyBulletSimulator(scene_description, use_gui=use_gui)

    hla_hyperparams = {"max_motion_planning_time": 10.0}
    navigate_hla = NavigateHLA(
        sim,
        robot_interface,
        None,
        None,
        web_interface,
        hla_hyperparams,
        None,
        None,
        no_waits,
        log_dir,
        behavior_tree_dir,
        log_dir / "execution_log.txt",
        log_dir,
    )

    nominal_pose = navigate_hla._load_target_pose(location)
    print(
        "Loaded named location "
        f"'{location}' from {navigate_hla._location_yaml()} "
        f"(frame={nominal_pose['frame_id']}, "
        f"position=({nominal_pose['x']:.4f}, {nominal_pose['y']:.4f}, {nominal_pose['z']:.4f}))."
    )

    position_offset = None if ignore_offset else _read_position_offset(behavior_tree_dir, location)
    if position_offset is not None:
        print(
            f"[nav-offset] applying stored offset from {behavior_tree_dir}/"
            f"navigate_to_{location}.yaml: "
            f"dx={position_offset[0]:+.3f}m dy={position_offset[1]:+.3f}m "
            f"dyaw={math.degrees(position_offset[2]):+.1f}deg"
        )

    scripted_terminal_leg = False
    try:
        # Go through the PUBLIC entry point so the harness matches deployment
        # behavior in navigate.py: route-specific via points (e.g. sink via
        # kitchen_enter) and any origin-gated logged-nav branches live there,
        # not in _navigate_to_target(). The harness has no PDDL executive, so
        # --assume_from substitutes the origin execute_action() would stash.
        navigate_method = getattr(navigate_hla, f"navigate_to_{location}")
        if assume_from is not None:
            navigate_hla._nav_origin = assume_from
            print(f"[nav] harness: assumed origin = {assume_from!r}")
        navigate_method(speed, position_offset=position_offset)
        scripted_terminal_leg = (
            navigate_hla._logged_nav_enabled()
            and location == "microwave"
            and assume_from == "fridge"
        )

        # With the interface, the HLA's own iPad adjust prompt already ran
        # inside _navigate_to_target; without it, offer the terminal-driven
        # equivalent so the offset loop works standalone.
        if adjust and run_on_robot and web_interface is None and not no_waits:
            # A scripted terminal leg never applied the offset to the motion,
            # so the adjustment measures against commanded == nominal.
            commanded_pose = (
                dict(nominal_pose) if scripted_terminal_leg
                else navigate_hla._apply_offset_to_pose(nominal_pose, position_offset)
            )
            _terminal_adjustment(
                navigate_hla, location, nominal_pose, commanded_pose, position_offset
            )
    finally:
        # Unlike run.py (which parks in rospy.spin() and lives until its tmux
        # pane is killed), this script must return to the shell: stop the
        # WebInterface's non-daemon worker threads and shut rospy down, or the
        # interpreter never exits after main() returns.
        if web_interface is not None:
            try:
                web_interface.stop_all_threads()
            except Exception as e:
                print(f"Could not stop web-interface threads cleanly: {e}")
        if rospy_mod is not None:
            rospy_mod.signal_shutdown("test_navigate_action complete")
    print("test_navigate_action done.")


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
    parser.add_argument(
        "--behavior_tree_dir",
        type=str,
        default="",
        help=(
            "Directory holding navigate_to_*.yaml. Default: a working copy "
            "under the test log dir, seeded once from the factory trees "
            "(mirrors run.py's per-user seeding); pass a per-user "
            "log/<user>/behavior_trees dir to test against deployment state."
        ),
    )
    parser.add_argument(
        "--ignore_offset",
        action="store_true",
        help="Navigate to the raw mapped pose, ignoring any stored PositionOffset.",
    )
    parser.add_argument(
        "--no_adjust",
        action="store_true",
        help="Skip the post-arrival terminal adjustment prompt.",
    )
    parser.add_argument(
        "--use_interface",
        action="store_true",
        help=(
            "Use the deployment web interface (iPad) for the post-arrival "
            "adjust prompt and failure-recovery teleop. Requires the "
            "rosbridge/webapp stack to be running with a client connected."
        ),
    )
    parser.add_argument(
        "--no_waits",
        action="store_true",
        help=(
            "Mirror run.py's --no_waits: NullSimulator instead of PyBullet and "
            "no interactive waits (offsets are still applied, but the adjust "
            "prompts are skipped)."
        ),
    )
    parser.add_argument(
        "--logged_nav",
        action="store_true",
        help=(
            "Mirror run.py's --logged-navigation (sets FEEDING_LOGGED_NAV=1): "
            "scripted kitchen legs instead of move_base. Combine with "
            "--assume_from to pick which scripted leg fires."
        ),
    )
    parser.add_argument(
        "--assume_from",
        type=str,
        default="",
        help=(
            "Origin location to assume (the harness has no PDDL executive to "
            "provide ?from). Used by any origin-specific route logic; with "
            "--logged_nav, e.g. --assume_from fridge --location microwave "
            "drives the scripted 1.4 m approach, and --assume_from microwave "
            "--location table drives the scripted kitchen egress."
        ),
    )
    args = parser.parse_args()

    if args.speed not in {"low", "medium", "high"}:
        raise ValueError(
            f"Invalid speed '{args.speed}'. Must be one of: low, medium, high."
        )

    if args.locations_file:
        os.environ["FEEDING_NAV_LOCATIONS_FILE"] = args.locations_file

    if args.logged_nav:
        os.environ["FEEDING_LOGGED_NAV"] = "1"
        print("[logged-nav] Hardcoded navigation mode ENABLED (FEEDING_LOGGED_NAV=1).")

    try:
        test_navigate_action(
            scene_config=args.scene_config,
            transfer_type=args.transfer_type,
            run_on_robot=args.run_on_robot,
            location=args.location,
            speed=args.speed,
            use_gui=args.use_gui,
            behavior_tree_dir=(
                Path(args.behavior_tree_dir).expanduser().resolve()
                if args.behavior_tree_dir
                else None
            ),
            ignore_offset=args.ignore_offset,
            adjust=not args.no_adjust,
            use_interface=args.use_interface,
            no_waits=args.no_waits,
            assume_from=args.assume_from or None,
        )
    except KeyboardInterrupt:
        # disable_signals=True keeps Ctrl+C as a plain KeyboardInterrupt; the
        # function's finally block has already stopped the worker threads.
        print("\ntest_navigate_action interrupted.")
