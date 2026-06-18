from typing import Any

import time

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
    
    def pick_plate_from_fridge(self, speed: str) -> None:
        assert self.sim.held_object_name is None
        print("Picking plate from fridge ...")

        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_contents_gaze_pos)
        attachment_poses = self.perception_interface.perceive_attachment_poses(web_interface=self.web_interface)

        pickup_pose = attachment_poses["pickup_pose"]
        pre_pickup_pose = attachment_poses["pre_pickup_pose"]
        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_inside_intermediate_pos)

        self.move_to_ee_pose(pre_pickup_pose)
        self.close_gripper()
        self.move_to_ee_pose(pickup_pose)
        self.open_gripper()
        self.move_to_ee_pose(pre_pickup_pose)
        
        self.move_to_ee_pose(self.sim.scene_description.fridge_inside_intermediate_pose)
        self.move_to_ee_pose(self.sim.scene_description.fridge_another_inside_intermediate_pose)
        self.move_to_ee_pose(self.sim.scene_description.fridge_above_intermediate_pose)
        # self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)

    def pick_plate_from_microwave(self, speed: str) -> None:
        assert self.sim.held_object_name is None
        print("Picking plate from microwave ...")
        
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.right_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.microwave_inside_gaze_pos)

        time.sleep(2.0)
        attachment_poses = self.perception_interface.perceive_attachment_poses(web_interface=self.web_interface)
        pickup_pose = attachment_poses["pickup_pose"]
        pre_pickup_pose = attachment_poses["pre_pickup_pose"]

        self.move_to_joint_positions(self.sim.scene_description.right_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)

        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.microwave_plate_staging_pos)
        self.move_to_ee_pose(pre_pickup_pose)
        self.move_to_ee_pose(pickup_pose)
        # input("Press Enter to open gripper and pick up the plate ...")
        self.open_gripper()
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
        print("Picking plate from holder ...")

        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_intermediate_pos)
        self.move_to_joint_positions(self.sim.scene_description.above_plate_holder_pos)
        self.close_gripper()
        self.move_to_ee_pose(self.sim.scene_description.inside_plate_holder_pose)
        self.open_gripper()
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

    def pick_plate_from_table(self, speed: str) -> None:
        assert self.sim.held_object_name is None
        print("Picking plate from table ...")

        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.table_plate_staging_pos)

        placement_poses = self.perception_interface.get_perceived_table_placement_poses()

        self.move_to_ee_pose(placement_poses["pre_table_placement_pose"])
        self.close_gripper()
        self.move_to_ee_pose(placement_poses["table_placement_pose"])
        self.open_gripper()
        self.move_to_ee_pose(placement_poses["pre_table_placement_pose"])
        self.move_to_ee_pose(self.sim.scene_description.table_plate_staging_pose)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
