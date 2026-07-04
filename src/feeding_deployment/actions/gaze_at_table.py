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

    # manip_confirm_mode defaults to None so per-user behavior trees that
    # predate the AskForManipulationConfirmation parameter still execute
    # (today's wait-for-the-user placement-detection page).
    def gaze_at_table(self, speed: str, manip_confirm_mode=None) -> None:
        if self.robot_interface is not None:
            self.robot_interface.set_speed(speed)

        print("Gazing at table ...")

        self.report_activity("Looking at the table")
        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
        self.move_to_joint_positions(self.sim.scene_description.table_gaze_pos)

        confirm_mode, confirm_autocontinue_s = self._confirm_page_args(manip_confirm_mode)
        placement_poses = self.perception_interface.perceive_table_placement_poses(
            web_interface=self.web_interface, confirm_mode=confirm_mode,
            confirm_autocontinue_s=confirm_autocontinue_s)
        print("Table placement poses:", placement_poses)

        self.move_to_joint_positions(self.sim.scene_description.left_back_retract_pos)
