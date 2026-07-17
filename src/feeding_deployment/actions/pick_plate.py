from typing import Any

from contextlib import nullcontext

from relational_structs import (
    GroundAtom,
    GroundOperator,
    LiftedAtom,
    LiftedOperator,
    Object,
    Predicate,
    Type,
    Variable,
)
from feeding_deployment.actions.base import (
    HighLevelAction,
    tool_type,
    plate_type,
    table_type,
    appliance_type,
    holder_type,
    GripperFree,
    Holding,
    InFrontOf,
    DoorOpen,
    PlateAt,
)

from feeding_deployment.safety.collision_threshold import collision_threshold

class PickPlateFromApplianceHLA(HighLevelAction):
    """Pick the plate from an appliance (fridge or microwave)."""

    def get_name(self) -> str:
        return "PickPlateFromAppliance"

    def get_operator(self) -> LiftedOperator:
        plate = Variable("?plate", plate_type)
        appliance = Variable("?appliance", appliance_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[plate, appliance],
            preconditions={
                LiftedAtom(GripperFree, []),
                LiftedAtom(InFrontOf, [appliance]),
                LiftedAtom(DoorOpen, [appliance]),
                LiftedAtom(PlateAt, [appliance]),
            },
            add_effects={
                LiftedAtom(Holding, [plate]),
            },
            delete_effects={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [appliance]),
            },
        )

    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params
        assert len(objects) == 2
        plate, appliance = objects
        assert self.sim.scene_description.scene_label == "vention"
        print("Plate name:", plate.name)
        assert plate.name == "plate"
        assert appliance.name in ["fridge", "microwave"]

        return f"pick_plate_from_{appliance.name}.yaml"
    
    def pick_plate_from_fridge(self, speed: str, handle_color, color_range, manip_confirm_mode, autocontinue_seconds) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Picking plate from fridge ...")

        self.report_activity("Looking inside the fridge for the plate")
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_contents_gaze_pos)
        self.settle_camera()
        confirm_mode, confirm_autocontinue_s = self._confirm_page_args(manip_confirm_mode, autocontinue_seconds)
        result = self.perception_interface.perceive_attachment_poses(handle_type="bottom textured fridge door", handle_color=handle_color, color_range=color_range, web_interface=self.web_interface, confirm_mode=confirm_mode, confirm_autocontinue_s=confirm_autocontinue_s)

        pickup_pose = result["pickup_pose"]
        pre_pickup_pose = result["pre_pickup_pose"]
        above_pickup_pose = result["above_pickup_pose"]
        post_pickup_pose = result["post_pickup_pose"]

        new_color = result["handle_color"]
        new_range = result["color_range"]
        orig_color = list(handle_color) if hasattr(handle_color, '__iter__') else handle_color
        if new_color != orig_color or abs(new_range - float(color_range)) > 1e-9:
            objects = (Object("plate", plate_type), Object("fridge", appliance_type))
            self.process_behavior_tree_parameter_update(objects, {}, "PickPlateFromAppliance", "PlateHandleColor", new_color)
            self.process_behavior_tree_parameter_update(objects, {}, "PickPlateFromAppliance", "PlateHandleColorTolerance", new_range)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_inside_intermediate_pos)

        self.report_activity("Reaching into the fridge for the plate")
        self.move_to_ee_pose(pre_pickup_pose)
        self.close_gripper()
        self.move_to_ee_pose(pickup_pose)
        self.open_gripper()
        self.report_activity("Lifting the plate out of the fridge")
        self.move_to_ee_pose(above_pickup_pose)
        self.move_to_ee_pose(post_pickup_pose)
        
        self.move_to_ee_pose(self.sim.scene_description.fridge_inside_intermediate_pose)
        self.move_to_ee_pose(self.sim.scene_description.fridge_above_intermediate_pose)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)

    def pick_plate_from_microwave(self, speed: str, handle_color, color_range, manip_confirm_mode, autocontinue_seconds) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Picking plate from microwave ...")

        self.report_activity("Looking inside the microwave for the plate")
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.right_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.microwave_inside_gaze_pos)

        self.settle_camera()
        # The camera is physically flipped for the microwave-inside gaze, so the frame is
        # already upright -- tell perception not to re-flip the user-facing images.
        confirm_mode, confirm_autocontinue_s = self._confirm_page_args(manip_confirm_mode, autocontinue_seconds)
        result = self.perception_interface.perceive_attachment_poses(handle_type="microwave", handle_color=handle_color, color_range=color_range, web_interface=self.web_interface, camera_flipped=True, confirm_mode=confirm_mode, confirm_autocontinue_s=confirm_autocontinue_s)
        pickup_pose = result["pickup_pose"]
        pre_pickup_pose = result["pre_pickup_pose"]

        new_color = result["handle_color"]
        new_range = result["color_range"]
        orig_color = list(handle_color) if hasattr(handle_color, '__iter__') else handle_color
        if new_color != orig_color or abs(new_range - float(color_range)) > 1e-9:
            objects = (Object("plate", plate_type), Object("microwave", appliance_type))
            self.process_behavior_tree_parameter_update(objects, {}, "PickPlateFromAppliance", "PlateHandleColor", new_color)
            self.process_behavior_tree_parameter_update(objects, {}, "PickPlateFromAppliance", "PlateHandleColorTolerance", new_range)

        self.move_to_joint_positions(self.sim.scene_description.right_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)

        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.microwave_plate_staging_pos)
        self.report_activity("Reaching into the microwave for the plate")
        self.move_to_ee_pose(pre_pickup_pose)
        self.move_to_ee_pose(pickup_pose)
        # input("Press Enter to open gripper and pick up the plate ...")
        self.open_gripper()
        self.report_activity("Lifting the plate out of the microwave")
        self.move_to_ee_pose(self.sim.scene_description.microwave_plate_staging_pose)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)

        # perceived_poses = self.perception_interface.get_perceived_poses()
        # behind_placement_pose = perceived_poses["behind_placement_pose"]
        # placement_pose = perceived_poses["placement_pose"]

        # self.close_gripper()
        # self.move_to_joint_positions(self.sim.scene_description.microwave_plate_staging_pos)
        # self.move_to_ee_pose(behind_placement_pose)
        # self.move_to_ee_pose(placement_pose)
        # self.open_gripper()
        # self.move_to_ee_pose(self.sim.scene_description.microwave_plate_staging_pose)


class PickPlateFromHolderHLA(HighLevelAction):
    """Pick the plate from the holder."""

    # Collision threshold applied while moving into the handle grasp pose, where
    # contact with the handle produces larger torque error. Tune on the real
    # robot; reverts to the sensor default automatically after the move.
    HOLDER_COLLISION_THRESHOLD = 20.0

    def get_name(self) -> str:
        return "PickPlateFromHolder"

    def get_operator(self) -> LiftedOperator:
        plate = Variable("?plate", plate_type)
        holder = Variable("?holder", holder_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[plate, holder],
            preconditions={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [holder]),
            },
            add_effects={
                LiftedAtom(Holding, [plate]),
            },
            delete_effects={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [holder]),
            },
        )

    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params
        assert len(objects) == 2
        plate, holder = objects
        assert self.sim.scene_description.scene_label == "vention"
        assert plate.name == "plate"
        assert holder.name == "holder"
        return "pick_plate_from_holder.yaml"
    
    def pick_plate_from_holder(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        holder_threshold = (
            collision_threshold(self.HOLDER_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )

        print("Picking plate from holder ...")

        self.report_activity("Reaching for the plate on the stand")
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_intermediate_pos)
        self.move_to_joint_positions(self.sim.scene_description.above_plate_holder_pos)
        self.close_gripper()
        self.report_activity("Picking up the plate")
        self.move_to_ee_pose(self.sim.scene_description.inside_plate_holder_pose)

        with holder_threshold:
            self.open_gripper()
            self.report_activity("Lifting the plate off the stand")
            self.move_to_ee_pose(self.sim.scene_description.above_plate_holder_pose)

        self.move_to_ee_pose(self.sim.scene_description.intermediate_plate_holder_pose)
        self.move_to_joint_positions(self.sim.scene_description.behind_intermediate_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)

class PickPlateFromTableHLA(HighLevelAction):
    """Pick the plate from the table."""

    def get_name(self) -> str:
        return "PickPlateFromTable"

    def get_operator(self) -> LiftedOperator:
        plate = Variable("?plate", plate_type)
        table = Variable("?table", table_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[plate, table],
            preconditions={
                LiftedAtom(GripperFree, []),
                LiftedAtom(InFrontOf, [table]),
                LiftedAtom(PlateAt, [table]),
            },
            add_effects={
                LiftedAtom(Holding, [plate]),
            },
            delete_effects={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [table]),
            },
        )

    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params
        assert len(objects) == 2
        plate, table = objects
        assert self.sim.scene_description.scene_label == "vention"
        assert plate.name == "plate"
        assert table.name == "table"
        return "pick_plate_from_table.yaml"

    def pick_plate_from_table(self, speed: str, handle_color, color_range, manip_confirm_mode, autocontinue_seconds) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Picking plate from table ...")

        self.report_activity("Recording a picture of the plate after feeding")
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.above_plate_pos)
        self.log_camera_image("plate_after_feeding", settle_s=5.0)
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)

        self.report_activity("Looking at the table for the plate")
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.table_plate_gaze_pos)
        self.settle_camera()

        confirm_mode, confirm_autocontinue_s = self._confirm_page_args(manip_confirm_mode, autocontinue_seconds)
        result = self.perception_interface.perceive_attachment_poses(handle_type="table", handle_color=handle_color, color_range=color_range, web_interface=self.web_interface, handle_orientation="left", confirm_mode=confirm_mode, confirm_autocontinue_s=confirm_autocontinue_s)
        pickup_pose = result["pickup_pose"]
        pre_pickup_pose = result["pre_pickup_pose"]
        above_pickup_pose = result["above_pickup_pose"]

        new_color = result["handle_color"]
        new_range = result["color_range"]
        orig_color = list(handle_color) if hasattr(handle_color, '__iter__') else handle_color
        if new_color != orig_color or abs(new_range - float(color_range)) > 1e-9:
            objects = (Object("plate", plate_type), Object("table", table_type))
            self.process_behavior_tree_parameter_update(objects, {}, "PickPlateFromTable", "PlateHandleColor", new_color)
            self.process_behavior_tree_parameter_update(objects, {}, "PickPlateFromTable", "PlateHandleColorTolerance", new_range)

        self.move_to_joint_positions(self.sim.scene_description.table_intermediate_pos)
        self.report_activity("Reaching for the plate on the table")
        self.move_to_ee_pose(pre_pickup_pose)
        self.close_gripper()
        self.move_to_ee_pose(pickup_pose)
        self.open_gripper()
        self.report_activity("Lifting the plate off the table")
        self.move_to_ee_pose(above_pickup_pose)
        self.move_to_ee_pose(self.sim.scene_description.table_intermediate_pose)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
