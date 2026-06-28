from typing import Any

from relational_structs import (
    LiftedAtom,
    LiftedOperator,
    Object,
    Variable,
)
from feeding_deployment.actions.base import (
    HighLevelAction,
    tool_type,
    plate_type,
    table_type,
    sink_type,
    holder_type,
    appliance_type,
    GripperFree,
    Holding,
    InFrontOf,
    DoorOpen,
    PlateAt,
    TableSeen,
    FoodHeated,
)

from feeding_deployment.safety.collision_threshold import collision_threshold

class PlacePlateInApplianceHLA(HighLevelAction):
    """Place the plate into an appliance (fridge or microwave)."""

    def get_name(self) -> str:
        return "PlacePlateInAppliance"

    def get_operator(self) -> LiftedOperator:
        plate = Variable("?plate", plate_type)
        appliance = Variable("?appliance", appliance_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[plate, appliance],
            preconditions={
                LiftedAtom(Holding, [plate]),
                LiftedAtom(InFrontOf, [appliance]),
                LiftedAtom(DoorOpen, [appliance]),
            },
            add_effects={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [appliance]),
            },
            delete_effects={
                LiftedAtom(Holding, [plate]),
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
        assert plate.name == "plate"
        assert appliance.name in ["fridge", "microwave"]

        return f"place_plate_in_{appliance.name}.yaml"

    def place_plate_in_fridge(self, speed: str) -> None:
        # assert self.sim.held_object_name == "plate"
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Placing plate in fridge ...")

    def place_plate_in_microwave(self, speed: str) -> None:
        # assert self.sim.held_object_name == "plate"
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Placing plate in microwave ...")

        perceived_poses = self.perception_interface.get_perceived_poses()
        behind_placement_pose = perceived_poses["behind_placement_pose"]
        placement_pose = perceived_poses["placement_pose"]

        self.move_to_joint_positions(self.sim.scene_description.microwave_plate_staging_pos)
        self.move_to_ee_pose(placement_pose)
        self.close_gripper()
        self.move_to_ee_pose(behind_placement_pose)
        self.move_to_ee_pose(self.sim.scene_description.microwave_plate_staging_pose)

class PlacePlateOnHolderHLA(HighLevelAction):
    """Place the plate onto the holder."""

    # Collision threshold applied while moving into the handle grasp pose, where
    # contact with the handle produces larger torque error. Tune on the real
    # robot; reverts to the sensor default automatically after the move.
    HOLDER_COLLISION_THRESHOLD = 20.0

    def get_name(self) -> str:
        return "PlacePlateOnHolder"

    def get_operator(self) -> LiftedOperator:
        plate = Variable("?plate", plate_type)
        holder = Variable("?holder", holder_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[plate, holder],
            preconditions={
                LiftedAtom(Holding, [plate]),
            },
            add_effects={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [holder]),
            },
            delete_effects={
                LiftedAtom(Holding, [plate]),
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
        return "place_plate_on_holder.yaml"

    def place_plate_on_holder(self, speed: str) -> None:
        # assert self.sim.held_object_name == "plate"
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        holder_threshold = (
            collision_threshold(self.HOLDER_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )

        print("Placing plate on holder ...")

        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_intermediate_pos)
        self.move_to_joint_positions(self.sim.scene_description.intermediate_plate_holder_pos)
        self.move_to_joint_positions(self.sim.scene_description.above_plate_holder_pos)
        
        with holder_threshold:
            self.move_to_ee_pose(self.sim.scene_description.inside_plate_holder_pose)
            self.close_gripper()

        self.move_to_ee_pose(self.sim.scene_description.above_plate_holder_pose)
        self.move_to_joint_positions(self.sim.scene_description.behind_intermediate_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)

class PlacePlateInSinkHLA(HighLevelAction):
    """Place the plate in the sink."""

    def get_name(self) -> str:
        return "PlacePlateInSink"
    
    def get_operator(self) -> LiftedOperator:
        plate = Variable("?plate", plate_type)
        sink = Variable("?sink", sink_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[plate, sink],
            preconditions={
                LiftedAtom(Holding, [plate]),
                LiftedAtom(InFrontOf, [sink]),
            },
            add_effects={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [sink]),
            },
            delete_effects={
                LiftedAtom(Holding, [plate]),
            },
        )

    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params
        assert len(objects) == 2
        plate, sink = objects
        assert self.sim.scene_description.scene_label == "vention"
        assert plate.name == "plate"
        assert sink.name == "sink"

        return f"place_plate_in_sink.yaml"

    def place_plate_in_sink(self, speed: str) -> None:
        # assert self.sim.held_object_name == "plate"
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Placing plate in sink ...")

        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.right_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.sink_gaze_pos)

        placement_poses = self.perception_interface.perceive_sink_placement_poses(web_interface=self.web_interface)

        self.move_to_joint_positions(self.sim.scene_description.sink_plate_staging_pos)
        self.move_to_ee_pose(placement_poses["sink_placement_pose"])
        self.close_gripper()
        self.move_to_ee_pose(self.sim.scene_description.sink_plate_staging_pose)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)



class PlacePlateOnTableHLA(HighLevelAction):
    """Place the plate on the table."""

    def get_name(self) -> str:
        return "PlacePlateOnTable"
    
    def get_operator(self) -> LiftedOperator:
        plate = Variable("?plate", plate_type)
        table = Variable("?table", table_type)
        
        return LiftedOperator(
            self.get_name(),
            parameters=[plate, table],
            preconditions={
                LiftedAtom(Holding, [plate]),
                LiftedAtom(InFrontOf, [table]),
                LiftedAtom(TableSeen, []),
                # Food must be heated before it reaches the table. With
                # FoodHeated unset, planning-to-preconditions routes the plate
                # through the microwave; the "no microwave" preference adds
                # FoodHeated up front so the planner serves directly.
                LiftedAtom(FoodHeated, []),
            },
            add_effects={
                LiftedAtom(GripperFree, []),
                LiftedAtom(PlateAt, [table]),
            },
            delete_effects={
                LiftedAtom(Holding, [plate]),
                LiftedAtom(TableSeen, []),
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

        return f"place_plate_on_table.yaml"

    def place_plate_on_table(self, speed: str) -> None:
        # assert self.sim.held_object_name == "plate"
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Placing plate on table ...")

        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.table_plate_staging_pos)

        placement_poses = self.perception_interface.get_perceived_table_placement_poses()

        self.move_to_ee_pose(placement_poses["pre_table_placement_pose"])
        self.move_to_ee_pose(placement_poses["table_placement_pose"])
        self.close_gripper()
        self.move_to_ee_pose(placement_poses["behind_table_placement_pose"])
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)