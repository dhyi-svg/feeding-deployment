from typing import Any

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


class CloseDoorHLA(HighLevelAction):
    """Close a door (fridge or microwave)."""

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
        del speed
        assert self.sim.held_object_name is None
        print("Closing fridge door ...")

    def close_microwave(self, speed: str) -> None:
        del speed
        assert self.sim.held_object_name is None
        print("Closing microwave door ...")

        handle_closing_poses = self.perception_interface.perceive_handle_closing_poses("microwave")

        # visualize on rviz
        poses = []
        poses.append(handle_closing_poses["pre_grasp_pose"])
        poses.append(handle_closing_poses["grasp_pose"])
        poses.extend(handle_closing_poses["opening_waypoints"])
        poses.append(handle_closing_poses["post_release_pose"])
        poses.append(handle_closing_poses["pre_push_pose"])
        poses.append(handle_closing_poses["push_pose"])
        poses.extend(handle_closing_poses["push_waypoints"])
        poses.append(handle_closing_poses["before_above_closing_waypoint"])
        poses.append(handle_closing_poses["above_closing_waypoint"])
        poses.append(handle_closing_poses["closing_waypoint"])
        poses.extend(handle_closing_poses["closing_waypoints"])
        print(f"Visualizing {len(poses)} handle opening poses in RViz ...")
        self.rviz_interface.visualize_poses(poses, frame_id="arm_base_link", ns="handle_closing_poses")

        # self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)

        self.move_to_ee_pose(handle_closing_poses["pre_grasp_pose"])
        self.open_gripper()
        self.move_to_ee_pose(handle_closing_poses["grasp_pose"])
        self.close_gripper()
        # self.move_to_ee_pose(handle_closing_poses["post_grasp_pose"])
        self.move_to_ee_pose_trajectory(handle_closing_poses["opening_waypoints"])
        self.open_gripper()
        self.move_to_ee_pose(handle_closing_poses["post_release_pose"])
        
        # self.move_to_joint_positions(self.sim.scene_description.fridge_door_intermediate_restract_pos)

        self.move_to_ee_pose(handle_closing_poses["pre_push_pose"])
        self.move_to_ee_pose(handle_closing_poses["push_pose"])
        self.move_to_ee_pose_trajectory(handle_closing_poses["push_waypoints"])

        self.move_to_ee_pose(handle_closing_poses["before_above_closing_waypoint"])
        self.move_to_ee_pose(handle_closing_poses["above_closing_waypoint"])
        self.move_to_ee_pose(handle_closing_poses["closing_waypoint"])
        self.move_to_ee_pose_trajectory(handle_closing_poses["closing_waypoints"])

        self.close_gripper()
        self.move_to_ee_pose(handle_closing_poses["offset_closing_waypoints"][0])
        self.move_to_ee_pose_trajectory(handle_closing_poses["offset_closing_waypoints"])