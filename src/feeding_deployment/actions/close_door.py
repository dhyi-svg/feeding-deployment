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
    appliance_type,
    GripperFree,
    InFrontOf,
    DoorOpen,
    DoorClosed,
    SafeToNavigate,
)

from feeding_deployment.safety.collision_threshold import collision_threshold

class CloseDoorHLA(HighLevelAction):
    """Close a door (fridge or microwave)."""

    # Collision threshold applied while moving into the handle grasp pose, where
    # contact with the handle produces larger torque error. Tune on the real
    # robot; reverts to the sensor default automatically after the move.
    PULL_COLLISION_THRESHOLD = 25.0
    SLIGHT_PUSH_COLLISION_THRESHOLD = 15.0
    HARD_PUSH_COLLISION_THRESHOLD = 40.0

    def get_name(self) -> str:
        return "CloseDoor"

    def get_operator(self) -> LiftedOperator:
        appliance = Variable("?appliance", appliance_type)
        return LiftedOperator(
            self.get_name(),
            parameters=[appliance],
            preconditions={
                LiftedAtom(GripperFree, []),
                LiftedAtom(InFrontOf, [appliance]),
                LiftedAtom(DoorOpen, [appliance]),
            },
            add_effects={
                LiftedAtom(DoorClosed, [appliance]),
                LiftedAtom(SafeToNavigate, []),
            },
            delete_effects={LiftedAtom(DoorOpen, [appliance])},
        )

    def get_behavior_tree_filename(
        self,
        objects: tuple[Object, ...],
        params: dict[str, Any],
    ) -> str:
        del params
        assert len(objects) == 1
        appliance = objects[0]
        assert self.sim.scene_description.scene_label == "vention"
        assert appliance.name in ["fridge", "microwave"]
        return f"close_{appliance.name}.yaml"

    def close_fridge(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)
        print("Closing fridge door ...")

        pull_threshold = (
            collision_threshold(self.PULL_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )
        slight_push_threshold = (
            collision_threshold(self.SLIGHT_PUSH_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )
        hard_push_threshold = (
            collision_threshold(self.HARD_PUSH_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )

        self.report_activity("Reaching for the fridge handle")
        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.open_gripper()

        handle_closing_poses = self.perception_interface.perceive_handle_closing_poses("bottom textured fridge door")

        print("moving to pre-pull pose: ", handle_closing_poses["pre_pull_pose"])
        self.move_to_ee_pose(handle_closing_poses["pre_pull_pose"])

        print("moving to pull closing waypoint: ", handle_closing_poses["pull_closing_waypoint"])
        self.move_to_ee_pose(handle_closing_poses["pull_closing_waypoint"])

        with pull_threshold:
            self.report_activity("Swinging the fridge door closed")
            self.close_gripper()
            self.move_to_ee_pose_trajectory(handle_closing_poses["pull_closing_waypoints"])
            self.open_gripper()

        self.move_to_ee_pose(handle_closing_poses["behind_pull_closing_waypoint"])
        self.move_to_ee_pose(handle_closing_poses["above_pull_closing_waypoint"])
        self.move_to_ee_pose(handle_closing_poses["above_push_closing_waypoint"])
        self.move_to_ee_pose(handle_closing_poses["push_closing_waypoints"][0])

        with hard_push_threshold:
            self.report_activity("Pushing the fridge door shut")
            self.move_to_ee_pose_trajectory(handle_closing_poses["push_closing_waypoints"])
            time.sleep(1.0)
            self.move_to_ee_pose(handle_closing_poses["push_closing_waypoints"][-3])

        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)

    def close_microwave(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)
        print("Closing microwave door ...")

        pull_threshold = (
            collision_threshold(self.PULL_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )
        slight_push_threshold = (
            collision_threshold(self.SLIGHT_PUSH_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )
        hard_push_threshold = (
            collision_threshold(self.HARD_PUSH_COLLISION_THRESHOLD)
            if self.robot_interface is not None
            else nullcontext()
        )

        self.report_activity("Reaching for the microwave door")
        # self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.close_gripper()

        handle_closing_poses = self.perception_interface.perceive_handle_closing_poses("microwave")

        self.move_to_ee_pose(handle_closing_poses["before_above_closing_waypoint"])
        self.open_gripper()
        self.move_to_ee_pose(handle_closing_poses["above_closing_waypoint"])
        self.move_to_ee_pose(handle_closing_poses["closing_waypoint"])

        with slight_push_threshold:
            self.report_activity("Swinging the microwave door closed")
            self.move_to_ee_pose_trajectory(handle_closing_poses["closing_waypoints"])
            time.sleep(1.0) # wait for the door to be fully closed before moving the arm away
            self.move_to_ee_pose(handle_closing_poses["closing_waypoints"][-3])

        self.close_gripper()
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_ee_pose(handle_closing_poses["offset_closing_waypoints"][0])

        with hard_push_threshold:
            self.report_activity("Pushing the microwave door shut")
            self.move_to_ee_pose_trajectory(handle_closing_poses["offset_closing_waypoints"])
            self.move_to_ee_pose(handle_closing_poses["pre_grasp_pose"])

        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)