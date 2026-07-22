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
    table_type,
    GripperFree,
    Holding,
    InFrontOf,
)

class PickToolHLA(HighLevelAction):
    """Pick up a tool (utensil, drink, or wipe)."""

    def get_name(self) -> str:
        return "PickTool"

    def get_operator(self) -> LiftedOperator:
        tool = Variable("?tool", tool_type)
        table = Variable("?table", table_type)
        return LiftedOperator(
            self.get_name(),
            parameters=[tool, table],
            preconditions={
                LiftedAtom(GripperFree, []),
                LiftedAtom(InFrontOf, [table]),
            },
            add_effects={LiftedAtom(Holding, [tool])},
            delete_effects={LiftedAtom(GripperFree, [])},
        )
    
    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params  # not used right now
        assert len(objects) == 2
        tool = objects[0]
        table = objects[1]
        assert self.sim.scene_description.scene_label == "vention"
        assert tool.name in ["utensil", "drink", "wipe"]
        assert table.name == "table"
        return f"pick_{tool.name}.yaml"
    
    def pick_utensil(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        self.report_activity("Picking up the feeding utensil")
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.utensil_above_mount_pos)

        with self.low_speed(restore=speed):
            self.move_to_ee_pose(self.sim.scene_description.utensil_inside_mount_pose)
            self.grasp_tool("utensil")

            if self.wrist_interface is not None:
                time.sleep(1.0) # wait for the utensil to be connected
                print("Resetting wrist controller ...")
                self.wrist_interface.set_velocity_mode()
                self.wrist_interface.reset()

            self.move_to_ee_pose(self.sim.scene_description.utensil_outside_mount_pose)

        self.move_to_ee_pose(self.sim.scene_description.utensil_outside_above_mount_pose)
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.before_transfer_pos)
        
    def pick_drink(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        self.report_activity("Looking for the drink")
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.drink_gaze_pos)

        drink_poses = self.perception_interface.perceive_drink_pickup_poses()

        self.report_activity("Picking up the drink")
        self.move_to_joint_positions(self.sim.scene_description.drink_staging_pos)
        self.move_to_ee_pose(drink_poses['pre_grasp_pose'])

        with self.low_speed(restore=speed):
            self.move_to_ee_pose(drink_poses['inside_bottom_pose'])
            self.move_to_ee_pose(drink_poses['inside_top_pose'])

            self.grasp_tool("drink")
            self.move_to_ee_pose(drink_poses['post_grasp_pose'])

        self.perception_interface.record_drink_pickup_joint_pos()

    def pick_wipe(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        self.report_activity("Picking up the wipe")
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.wipe_above_mount_pos)

        with self.low_speed(restore=speed):
            self.move_to_ee_pose(self.sim.scene_description.wipe_inside_mount_pose)
            self.grasp_tool("wipe")
            self.move_to_ee_pose(self.sim.scene_description.wipe_outside_mount_pose)

        self.move_to_ee_pose(self.sim.scene_description.wipe_outside_above_mount_pose)
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.before_transfer_pos)

    def pick_plate(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:    
            self.robot_interface.set_speed(speed)

        self.report_activity("Picking up the plate")
        if self.perception_interface.last_plate_poses is None:
            self.move_to_joint_positions(self.sim.scene_description.retract_pos)
            self.close_gripper()
            self.move_to_joint_positions(self.sim.scene_description.above_plate_pos)
            plate_poses = self.perception_interface.perceive_plate_pickup_poses()
        else:
            plate_poses = self.perception_interface.last_plate_poses

        print("Moving to plate staging position ...")
        self.move_to_joint_positions(self.sim.scene_description.plate_staging_pos)
        print("Moving to plate pre-grasp pose ...")
        self.move_to_ee_pose(plate_poses['pre_grasp_pose'])
        print("Moving to plate inside bottom pose ...")
        self.move_to_ee_pose(plate_poses['inside_bottom_pose'])
        print("Moving to plate inside top pose ...")
        self.move_to_ee_pose(plate_poses['inside_top_pose'])
        print("Grasping plate ...")
        self.grasp_tool("plate")
        # print("Moving to plate post-grasp pose ...")
        # self.move_to_ee_pose(plate_poses['post_grasp_pose'])

        self.perception_interface.record_plate_pickup_joint_pos()
