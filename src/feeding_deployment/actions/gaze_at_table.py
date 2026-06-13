from typing import Any

from relational_structs import (
    LiftedAtom,
    LiftedOperator,
    Object,
    Variable,
)
from feeding_deployment.actions.base import (
    HighLevelAction,
    table_type,
    GripperFree,
    InFrontOf,
    TableSeen,
)


class GazeAtTableHLA(HighLevelAction):
    """Gaze at the table to perceive the placement surface while gripper is free."""

    def get_name(self) -> str:
        return "GazeAtTable"

    def get_operator(self) -> LiftedOperator:
        table = Variable("?table", table_type)

        return LiftedOperator(
            self.get_name(),
            parameters=[table],
            preconditions={
                LiftedAtom(InFrontOf, [table]),
                LiftedAtom(GripperFree, []),
            },
            add_effects={
                LiftedAtom(TableSeen, []),
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
        (table,) = objects
        assert self.sim.scene_description.scene_label == "vention"
        assert table.name == "table"
        return "gaze_at_table.yaml"

    def gaze_at_table(self, speed: str) -> None:
        print("Gazing at table ...")

        # self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        
        print("Moving down by 0.2m")
        from pybullet_helpers.geometry import Pose
        import numpy as np
        ee = np.asarray(self.robot_interface.get_state()["ee_pos"], dtype=float)
        gaze_pose = Pose(
            position=ee[:3].copy() + np.array([0.0, 0.0, -0.1]),
            orientation=ee[3:].copy(), # should be in quaternion format already
        )

        current_pose = self.robot_interface.get_state()["ee_pos"]
        print("Current distance from gaze_pose:", np.linalg.norm(np.asarray(current_pose[:3]) - np.asarray(gaze_pose.position)))
        self.move_to_ee_pose(gaze_pose)
        current_pose = self.robot_interface.get_state()["ee_pos"]
        print("New distance from gaze_pose:", np.linalg.norm(np.asarray(current_pose[:3]) - np.asarray(gaze_pose.position)))
        print("Successfully moved down.")

        self.move_to_joint_positions(self.sim.scene_description.table_gaze_pos)

        placement_poses = self.perception_interface.perceive_table_placement_poses()
        print("Table placement poses:", placement_poses)

        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
