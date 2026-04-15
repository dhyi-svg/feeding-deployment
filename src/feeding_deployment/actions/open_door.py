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

        # set speed of the robot to highest
        self.robot_interface.set_speed("high")
        assert self.sim.held_object_name is None
        print("Opening fridge door ...")
        return
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.fridge_door_gaze_pos)

        handle_opening_poses = self.perception_interface.perceive_handle_opening_poses("white fridge door")

        # visualize on rviz
        poses = []
        poses.append(handle_opening_poses["pre_grasp_pose"])
        poses.append(handle_opening_poses["grasp_pose"])
        poses.extend(handle_opening_poses["opening_waypoints"])
        poses.append(handle_opening_poses["post_release_pose"])
        poses.append(handle_opening_poses["pre_push_pose"])
        poses.append(handle_opening_poses["push_pose"])
        poses.extend(handle_opening_poses["push_waypoints"])
        print(f"Visualizing {len(poses)} handle opening poses in RViz ...")
        self.rviz_interface.visualize_poses(poses, frame_id="base_link", ns="handle_opening_poses")

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
        
    def open_microwave(self, speed: str) -> None:
        assert self.sim.held_object_name is None
        print("Opening microwave door ...")
        return
        self.move_to_joint_positions(self.sim.scene_description.retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.microwave_closeup_gaze_pos)

        time.sleep(5.0) # wait for the robot to stabilize before perception
        press_button_poses = self.perception_interface.perceive_button_pressing_poses()

        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)
        self.close_gripper() # just in case the gripper is open
        self.move_to_ee_pose(press_button_poses["pre_press_pose"])
        self.move_to_ee_pose(press_button_poses["press_pose"])
        self.move_to_ee_pose(press_button_poses["intermediate_pose"])
        self.move_to_ee_pose(press_button_poses["press_pose"])
        self.move_to_ee_pose(press_button_poses["intermediate_pose"])
        self.move_to_ee_pose(press_button_poses["press_pose"])
        self.move_to_ee_pose(press_button_poses["pre_press_pose"])
        self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)

        # handle_opening_poses = self.perception_interface.perceive_handle_opening_poses("microwave")

        # # visualize on rviz
        # poses = []
        # poses.append(handle_opening_poses["pre_grasp_pose"])
        # poses.append(handle_opening_poses["grasp_pose"])
        # poses.extend(handle_opening_poses["opening_waypoints"])
        # poses.append(handle_opening_poses["post_release_pose"])
        # poses.append(handle_opening_poses["pre_push_pose"])
        # poses.append(handle_opening_poses["push_pose"])
        # poses.extend(handle_opening_poses["push_waypoints"])
        # poses.append(handle_opening_poses["before_above_closing_waypoint"])
        # poses.append(handle_opening_poses["above_closing_waypoint"])
        # poses.append(handle_opening_poses["closing_waypoint"])
        # poses.extend(handle_opening_poses["closing_waypoints"])
        # print(f"Visualizing {len(poses)} handle opening poses in RViz ...")
        # self.rviz_interface.visualize_poses(poses, frame_id="base_link", ns="handle_opening_poses")

        # # self.move_to_joint_positions(self.sim.scene_description.home_pos)
        # self.move_to_joint_positions(self.sim.scene_description.fridge_door_staging_pos)

        # self.move_to_ee_pose(handle_opening_poses["pre_grasp_pose"])
        # self.open_gripper()
        # self.move_to_ee_pose(handle_opening_poses["grasp_pose"])
        # self.close_gripper()
        # # self.move_to_ee_pose(handle_opening_poses["post_grasp_pose"])
        # self.move_to_ee_pose_trajectory(handle_opening_poses["opening_waypoints"])
        # self.open_gripper()
        # self.move_to_ee_pose(handle_opening_poses["post_release_pose"])
        
        # # self.move_to_joint_positions(self.sim.scene_description.fridge_door_intermediate_restract_pos)

        # self.move_to_ee_pose(handle_opening_poses["pre_push_pose"])
        # self.move_to_ee_pose(handle_opening_poses["push_pose"])
        # self.move_to_ee_pose_trajectory(handle_opening_poses["push_waypoints"])

        # self.move_to_ee_pose(handle_opening_poses["before_above_closing_waypoint"])
        # self.move_to_ee_pose(handle_opening_poses["above_closing_waypoint"])
        # self.move_to_ee_pose(handle_opening_poses["closing_waypoint"])
        # self.move_to_ee_pose_trajectory(handle_opening_poses["closing_waypoints"])

        # self.close_gripper()
        # self.move_to_ee_pose(handle_opening_poses["offset_closing_waypoints"][0])
        # self.move_to_ee_pose_trajectory(handle_opening_poses["offset_closing_waypoints"])