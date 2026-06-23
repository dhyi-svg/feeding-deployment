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


class OpenDoorHLA(HighLevelAction):
    """Open a door (fridge or microwave)."""

    def get_name(self) -> str:
        return "OpenDoor"

    def get_operator(self) -> LiftedOperator:
        appliance = Variable("?appliance", appliance_type)
        return LiftedOperator(
            self.get_name(),
            parameters=[appliance],
            preconditions={
                LiftedAtom(GripperFree, []),
                LiftedAtom(InFrontOf, [appliance]),
                LiftedAtom(DoorClosed, [appliance]),
            },
            add_effects={LiftedAtom(DoorOpen, [appliance])},
            delete_effects={
                LiftedAtom(DoorClosed, [appliance]),
                LiftedAtom(SafeToNavigate, []),
            },
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
        return f"open_{appliance.name}.yaml"

    def open_fridge(self, speed: str) -> None:

        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Opening fridge door ...")
        # return
        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_door_gaze_pos)

        handle_opening_poses = self.perception_interface.perceive_handle_opening_poses("bottom textured fridge door", web_interface=self.web_interface)

        # visualize on rviz
        poses = []
        poses.append(handle_opening_poses["pre_grasp_pose"])
        poses.append(handle_opening_poses["grasp_pose"])
        poses.extend(handle_opening_poses["opening_waypoints"])
        poses.append(handle_opening_poses["post_release_pose"])
        poses.append(handle_opening_poses["pre_push_pose"])
        poses.append(handle_opening_poses["push_pose"])
        poses.extend(handle_opening_poses["push_waypoints"])
        poses.extend(handle_opening_poses["pull_closing_waypoints"])
        poses.append(handle_opening_poses["pull_closing_waypoint"])
        poses.append(handle_opening_poses["pre_pull_pose"])
        poses.extend(handle_opening_poses["push_closing_waypoints"])
        poses.append(handle_opening_poses["above_pull_closing_waypoint"])
        poses.append(handle_opening_poses["above_push_closing_waypoint"])
        print(f"Visualizing {len(poses)} handle opening poses in RViz ...")
        self.rviz_interface.visualize_poses(poses, frame_id="arm_base_link", ns="handle_opening_poses")

        # self.move_to_joint_positions(self.sim.scene_description.home_pos)
        # self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)
        self.move_to_joint_positions(self.sim.scene_description.behind_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.microwave_plate_staging_pos)

        self.move_to_ee_pose(handle_opening_poses["pre_grasp_pose"])
        self.open_gripper()
        self.move_to_ee_pose(handle_opening_poses["grasp_pose"])
        self.close_gripper()
        # self.move_to_ee_pose(handle_opening_poses["post_grasp_pose"])
        self.move_to_ee_pose_trajectory(handle_opening_poses["opening_waypoints"])
        self.open_gripper()
        self.move_to_ee_pose(handle_opening_poses["post_release_pose"])
        
        # self.move_to_joint_positions(self.sim.scene_description.fridge_door_intermediate_restract_pos)

        self.move_to_ee_pose(handle_opening_poses["pre_push_pose"])
        self.move_to_ee_pose(handle_opening_poses["push_pose"])
        self.move_to_ee_pose_trajectory(handle_opening_poses["push_waypoints"])

        time.sleep(3)
        self.move_to_ee_pose(handle_opening_poses["push_waypoints"][-3])

        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        
    def open_microwave(self, speed: str) -> None:
        assert self.sim.held_object_name is None

        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Opening microwave door ...")

        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)

        handle_opening_poses = self.perception_interface.perceive_handle_opening_poses("microwave", web_interface=self.web_interface)

        # visualize on rviz
        poses = []
        poses.append(handle_opening_poses["pre_grasp_pose"])
        poses.append(handle_opening_poses["grasp_pose"])
        poses.extend(handle_opening_poses["opening_waypoints"])
        poses.append(handle_opening_poses["post_release_pose"])
        poses.append(handle_opening_poses["pre_push_pose"])
        poses.append(handle_opening_poses["push_pose"])
        poses.extend(handle_opening_poses["push_waypoints"])
        poses.append(handle_opening_poses["before_above_closing_waypoint"])
        poses.append(handle_opening_poses["above_closing_waypoint"])
        poses.append(handle_opening_poses["closing_waypoint"])
        poses.extend(handle_opening_poses["closing_waypoints"])
        print(f"Visualizing {len(poses)} handle opening poses in RViz ...")
        self.rviz_interface.visualize_poses(poses, frame_id="arm_base_link", ns="handle_opening_poses")

        # self.move_to_joint_positions(self.sim.scene_description.home_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)

        self.move_to_ee_pose(handle_opening_poses["pre_grasp_pose"])
        self.open_gripper()
        self.move_to_ee_pose(handle_opening_poses["grasp_pose"])
        self.close_gripper()
        # self.move_to_ee_pose(handle_opening_poses["post_grasp_pose"])
        self.move_to_ee_pose_trajectory(handle_opening_poses["opening_waypoints"])
        self.open_gripper()
        self.move_to_ee_pose(handle_opening_poses["post_release_pose"])
        
        # self.move_to_joint_positions(self.sim.scene_description.fridge_door_intermediate_restract_pos)

        self.move_to_ee_pose(handle_opening_poses["pre_push_pose"])
        self.move_to_ee_pose(handle_opening_poses["push_pose"])
        self.move_to_ee_pose_trajectory(handle_opening_poses["push_waypoints"])

        time.sleep(3)
        self.move_to_ee_pose(handle_opening_poses["push_waypoints"][-3])

        # intermediate pose to avoid collisions with the microwave door
        joint_positions = self.get_joint_positions()
        joint_positions[0] = 0
        self.move_to_joint_positions(joint_positions)

        # self.move_to_joint_positions(self.sim.scene_description.microwave_plate_staging_pos)
        self.move_to_joint_positions(self.sim.scene_description.left_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)