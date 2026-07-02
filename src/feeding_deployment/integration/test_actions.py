"""Testing pick and stow tool actions of the integrated system."""

from pathlib import Path
import shutil
import queue

try:
    import rospy

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False

from relational_structs import Object
from pybullet_helpers.geometry import Pose
from pybullet_helpers.link import get_relative_link_pose

from feeding_deployment.actions.base import tool_type, table_type, plate_type, appliance_type
from feeding_deployment.actions.pick_tool import PickToolHLA
from feeding_deployment.actions.pick_plate import PickPlateFromApplianceHLA, PickPlateFromTableHLA
from feeding_deployment.actions.stow_tool import StowToolHLA
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.interfaces.web_interface import WebInterface
from feeding_deployment.integration.data_logger import DataLogger
from feeding_deployment.interfaces.rviz_interface import RVizInterface
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.wrist_controller.wrist_controller import WristInterface
from feeding_deployment.simulation.scene_description import create_scene_description_from_config
from feeding_deployment.simulation.simulator import FeedingDeploymentPyBulletSimulator


def _tool_id(sim, tool: str) -> int:
    return {"utensil": sim.utensil_id, "drink": sim.drink_id, "wipe": sim.wipe_id}[tool]


def _attach_tool_to_gripper(sim, tool: str) -> None:
    """Set the sim state so the robot is holding the given tool."""
    sim.held_object_name = tool
    sim.held_object_id = _tool_id(sim, tool)
    sim.robot.set_finger_state(sim.scene_description.tool_grasp_fingers_value)
    finger_frame_id = sim.robot.link_from_name("finger_tip")
    end_effector_link_id = sim.robot.link_from_name(sim.robot.tool_link_name)
    sim.held_object_tf = get_relative_link_pose(
        sim.robot.robot_id, finger_frame_id, end_effector_link_id, sim.physics_client_id
    )


def test_PickToolHLA(tool, sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir):

    assert tool in ["utensil", "drink", "wipe"], f"Tool {tool} not recognized"

    high_level_action = PickToolHLA(sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, None, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir)

    # PickTool requires an empty gripper.
    sim.held_object_name = None

    tool_obj = Object(tool, tool_type)
    table_obj = Object("table", table_type)
    high_level_action.execute_action(objects=[tool_obj, table_obj], params={})


def test_StowToolHLA(tool, sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir):

    assert tool in ["utensil", "drink", "wipe"], f"Tool {tool} not recognized"

    high_level_action = StowToolHLA(sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, None, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir)

    # StowTool requires the tool to be held.
    _attach_tool_to_gripper(sim, tool)
    if robot_interface is not None:
        rviz_interface.tool_update(True, sim.held_object_name, Pose((0, 0, 0), (0, 0, 0, 1)))

    tool_obj = Object(tool, tool_type)
    table_obj = Object("table", table_type)
    high_level_action.execute_action(objects=[tool_obj, table_obj], params={})


def test_PickPlateHLA(location, sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir):

    assert location in ["table", "fridge", "microwave"], f"Location {location} not recognized"

    hla_cls = PickPlateFromTableHLA if location == "table" else PickPlateFromApplianceHLA
    high_level_action = hla_cls(sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, None, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir)

    # PickPlate requires an empty gripper.
    sim.held_object_name = None

    plate_obj = Object("plate", plate_type)
    location_obj = Object(location, table_type if location == "table" else appliance_type)
    # Speed / HandleColor / ColorRange come from the behavior tree's parameter defaults.
    high_level_action.execute_action(objects=[plate_obj, location_obj], params={})


def _main(
    scene_config: str, transfer_type: str, run_on_robot: bool, use_interface: bool, simulate_head_perception: bool, use_gui: bool, max_motion_planning_time: float = 10, tool: str = "utensil", no_waits: bool = False, action: str = "tool", location: str = "table"
) -> None:
    """Testing pick and stow tool actions."""

    if ROSPY_IMPORTED:
        rospy.init_node("test_actions")
    else:
        assert not run_on_robot, "Need ROS to run on robot"

    # logs are saved in user/scenario directory
    log_dir = Path(__file__).parent / "log" / "test_actions"
    if log_dir.exists():
        shutil.rmtree(log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    execution_log = Path(__file__).parent / "log" / "execution_log.txt" # in root log directory
    run_behavior_tree_dir = log_dir / "behavior_trees"
    gesture_detectors_dir = log_dir / "gesture_detectors"

    # Copy the initial behavior trees into a directory for this run.
    run_behavior_tree_dir.mkdir(exist_ok=True)
    original_behavior_tree_dir = Path(__file__).parents[1] / "actions" / "behavior_trees"
    assert original_behavior_tree_dir.exists()
    for original_bt_filename in original_behavior_tree_dir.glob("*.yaml"):
        shutil.copy(original_bt_filename, run_behavior_tree_dir)

    # Copy the initial gesture detection file into a directory for this run.
    gesture_detectors_dir.mkdir(exist_ok=True)
    original_gesture_detection_filepath = Path(__file__).parents[1] / "perception" / "gestures_perception" / "synthesized_gesture_detectors.py"
    assert original_gesture_detection_filepath.exists()
    shutil.copy(original_gesture_detection_filepath, gesture_detectors_dir)

    # Initialize the interface to the robot.
    if run_on_robot:
        robot_interface = ArmInterfaceClient()  # type: ignore  # pylint: disable=no-member
        wrist_interface = WristInterface()
    else:
        robot_interface = None
        wrist_interface = None

    data_logger = DataLogger(state_dir=log_dir)

    if use_interface:
        task_selection_queue = queue.Queue()
        web_interface = WebInterface(task_selection_queue=task_selection_queue, data_logger=data_logger)
    else:
        web_interface = None

    # Initialize the perceiver (e.g., get joint states or human head poses).
    perception_interface = PerceptionInterface(robot_interface=robot_interface, simulate_head_perception=simulate_head_perception, data_logger=data_logger)

    scene_config_path = Path(__file__).parent.parent / "simulation" / "configs" / f"{scene_config}.yaml"
    scene_description = create_scene_description_from_config(str(scene_config_path), transfer_type)
    sim = FeedingDeploymentPyBulletSimulator(scene_description, use_gui=use_gui)

    if robot_interface is not None:
        rviz_interface = RVizInterface(scene_description)
    else:
        rviz_interface = None

    hla_hyperparams = {"max_motion_planning_time": max_motion_planning_time}

    if action == "pick_plate":
        # Pick the plate from the given location (table / fridge / microwave).
        test_PickPlateHLA(location, sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir)
    else:
        # Pick the tool, then stow it.
        test_PickToolHLA(tool, sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir)
        test_StowToolHLA(tool, sim, robot_interface, perception_interface, rviz_interface, web_interface, hla_hyperparams, wrist_interface, no_waits, log_dir, run_behavior_tree_dir, execution_log, gesture_detectors_dir)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--scene_config", type=str, default="vention")
    parser.add_argument("--transfer_type", type=str, default="inside")
    parser.add_argument("--run_on_robot", action="store_true")
    parser.add_argument("--use_interface", action="store_true")
    parser.add_argument("--simulate_head_perception", action="store_true")
    parser.add_argument("--use_gui", action="store_true")
    parser.add_argument("--max_motion_planning_time", type=float, default=10.0)
    parser.add_argument("--tool", type=str, default="utensil")
    parser.add_argument("--no_waits", action="store_true")
    parser.add_argument("--action", type=str, default="tool", choices=["tool", "pick_plate"])
    parser.add_argument("--location", type=str, default="table", choices=["table", "fridge", "microwave"])
    args = parser.parse_args()

    _main(args.scene_config, args.transfer_type, args.run_on_robot, args.use_interface, args.simulate_head_perception, args.use_gui, args.max_motion_planning_time, args.tool, args.no_waits, args.action, args.location)
