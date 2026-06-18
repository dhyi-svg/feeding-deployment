from typing import Any
import time

from relational_structs import (
    LiftedAtom,
    LiftedOperator,
    Object,
    Variable,
)
from feeding_deployment.actions.base import (
    HighLevelAction,
    microwave_type,
    GripperFree,
    InFrontOf,
    DoorClosed,
    PlateAt,
    FoodHeated,
)


class PressMicrowaveButtonHLA(HighLevelAction):
    """Press the microwave button."""

    def get_name(self) -> str:
        return "PressMicrowaveButton"

    def get_operator(self) -> LiftedOperator:
        microwave = Variable("?microwave", microwave_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[microwave],
            preconditions={
                LiftedAtom(GripperFree, []),
                LiftedAtom(InFrontOf, [microwave]),
                LiftedAtom(DoorClosed, [microwave]),
                LiftedAtom(PlateAt, [microwave]),
            },
            add_effects={
                LiftedAtom(FoodHeated, []),
            },
            delete_effects=set(),
        )

    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params
        assert len(objects) == 1
        microwave = objects[0]
        assert self.sim.scene_description.scene_label == "vention"
        assert microwave.name == "microwave", (
            "The object parameter for PressMicrowaveButtonHLA should be the microwave"
        )
        return "press_microwave_button.yaml"

    def press_microwave_button(self, speed: str) -> None:
        assert self.sim.held_object_name is None
        print("Pressing microwave button ...")

        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.microwave_closeup_gaze_pos)

        time.sleep(2.0) # wait for the robot to stabilize before perception
        press_button_poses = self.perception_interface.perceive_button_pressing_poses(web_interface=self.web_interface)

        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)
        self.close_gripper() # just in case the gripper is open
        self.move_to_ee_pose(press_button_poses["pre_press_pose"])
        runs = 1
        for i in range(runs): # Change this to depend on predicted heating time
            self.move_to_ee_pose(press_button_poses["press_pose"])
            self.move_to_ee_pose(press_button_poses["intermediate_pose"])
        self.move_to_ee_pose(press_button_poses["pre_press_pose"])
        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)

        for i in range(runs):
            print(f"Waiting for the microwave to finish heating... (iteration {i+1}/{runs})")
            time.sleep(30) # wait for the microwave to finish heating. 