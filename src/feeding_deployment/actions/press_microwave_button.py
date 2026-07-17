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
    TeleopTakeoverException,
    microwave_type,
    GripperFree,
    InFrontOf,
    DoorClosed,
    PlateAt,
    FoodHeated,
)

from feeding_deployment.safety.collision_threshold import collision_threshold

class PressMicrowaveButtonHLA(HighLevelAction):
    """Press the microwave button."""

    # Collision threshold applied while moving into the handle grasp pose, where
    # contact with the handle produces larger torque error. Tune on the real
    # robot; reverts to the sensor default automatically after the move.
    PULL_COLLISION_THRESHOLD = 25.0
    SLIGHT_PUSH_COLLISION_THRESHOLD = 15.0
    HARD_PUSH_COLLISION_THRESHOLD = 25.0

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

    def press_microwave_button(self, speed: str, duration: float, manip_confirm_mode, autocontinue_seconds) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        slight_push_threshold = (
            collision_threshold(self.SLIGHT_PUSH_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )

        # Each button press adds 30 seconds to the microwave timer.
        num_presses = max(1, int(round(duration / 30.0)))
        print(f"Pressing microwave button {num_presses} times (duration={duration}s) ...")

        self.report_activity("Looking at the microwave buttons")
        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.microwave_closeup_gaze_pos)

        self.settle_camera()
        confirm_mode, confirm_autocontinue_s = self._confirm_page_args(manip_confirm_mode, autocontinue_seconds)
        try:
            press_button_poses = self.perception_interface.perceive_button_pressing_poses(
                web_interface=self.web_interface, confirm_mode=confirm_mode,
                confirm_autocontinue_s=confirm_autocontinue_s)
        except RuntimeError as e:
            # Button detection failed (molmo/ngrok down, or no camera frames).
            # Hand control to the user like the joint-limit path in base.py:
            # they press the button themselves and choose "next" (skill counts
            # as done -> FoodHeated) or "redo" (retry, tunnel may be back).
            if self.web_interface is None:
                raise
            print(f"Button detection failed ({e}); handing control to the user.")
            post_action = self._run_takeover_recovery_and_get_choice(
                failure_context="button_detection_failure"
            )
            raise TeleopTakeoverException(
                "Button detection failed; user recovered via teleop",
                redo_current=(post_action == "redo"),
            )

        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)
        self.close_gripper() # just in case the gripper is open
        self.move_to_ee_pose(press_button_poses["pre_press_pose"])
        self.report_activity(f"Pressing the microwave start button ({num_presses}x)")
        with slight_push_threshold:
            for i in range(num_presses):
                self.move_to_ee_pose(press_button_poses["press_pose"])
                self.move_to_ee_pose(press_button_poses["intermediate_pose"])
        self.move_to_ee_pose(press_button_poses["pre_press_pose"])
        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)

        for i in range(num_presses):
            print(f"Waiting for the microwave to finish heating... (iteration {i+1}/{num_presses})")
            self.report_activity(f"Heating the food in the microwave… ({i + 1}/{num_presses})")
            time.sleep(30) # wait for the microwave to finish heating.