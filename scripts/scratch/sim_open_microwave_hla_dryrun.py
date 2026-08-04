"""Sim-only, zero-hardware-risk test of the REAL OpenDoorHLA/behavior-tree
machinery for "microwave", per the 2026-07-30 TESTING_LOG finding that
HighLevelAction is duck-typed (it just stores whatever robot_interface /
perception_interface objects it's handed) and is therefore not actually
blocked by ArmInterfaceClient/PerceptionInterface's netft_rdt_driver import
chain -- only those two concrete classes are.

This does NOT move any robot and does NOT touch the camera: robot_interface is
None (every HLA move method takes its no-robot sim-visualization branch) and
NullSimulator is used (mirrors run.py's/test_navigate_action.py's own
--no_waits path -- no PyBullet GUI needed). The only real "wiring" under test
is: can OpenDoorHLA be constructed with duck-typed substitutes, does the real
open_microwave.yaml behavior tree load or its !hla tags resolve, and does the
real open_microwave() control flow run to completion against a stub
perception_interface without crashing.

The stub's pose VALUES are placeholders, not validated for real execution --
in this sim-only/robot_interface=None mode base.py's move_to_* methods never
read pose content (see move_to_ee_pose/move_to_ee_pose_trajectory: the
robot_interface-is-None branch calls sim.visualize_plan(None) unconditionally,
ignoring the pose/traj argument entirely). Do not reuse these numbers for a
real-hardware attempt.
"""

import sys
from pathlib import Path

import numpy as np
from pybullet_helpers.geometry import Pose
from relational_structs import Object

from feeding_deployment.actions.base import microwave_type
from feeding_deployment.actions.open_door import OpenDoorHLA
from feeding_deployment.simulation.scene_description import (
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import (
    FeedingDeploymentPyBulletSimulator,
    NullSimulator,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
FACTORY_BT_DIR = REPO_ROOT / "src" / "feeding_deployment" / "actions" / "behavior_trees"
LOG_DIR = Path(__file__).parent / "log" / "sim_open_microwave_hla_dryrun"
LOG_DIR.mkdir(parents=True, exist_ok=True)

# A real handle_pose from a past live Pachirisu detection (2026-07-28/07-30
# sessions), just so the stub returns numbers of the right shape/scale rather
# than zeros. Not re-derived live here -- see the "not done" note at the
# bottom for the live-detection follow-up.
HANDLE_POS = np.array([0.74838382, -0.17329798, 0.43816104])
DUMMY_QUAT = (0.5, -0.5, -0.5, 0.5)  # unit quaternion; exact value unvalidated


def _dummy_pose(offset=(0.0, 0.0, 0.0)) -> Pose:
    return Pose(tuple(HANDLE_POS + np.array(offset)), DUMMY_QUAT)


class StubMicrowavePerceptionAdapter:
    """Duck-typed perception_interface stand-in: implements only the one
    method OpenDoorHLA.open_microwave() actually calls. No camera, no ROS."""

    def perceive_handle_opening_poses(
        self, handle_type, web_interface=None, confirm_mode=None, confirm_autocontinue_s=0.0
    ):
        assert handle_type == "microwave"
        pre_grasp_pose = _dummy_pose((-0.30, 0.0, 0.0))
        grasp_pose = _dummy_pose()
        # Door-arc / push / closing phases are NOT modeled for this rig yet
        # (non-vertical hinge axis, unbuilt push-phase -- see
        # MICROWAVE_FRIDGE_STATUS_SUMMARY.md). These are structural
        # placeholders only, to satisfy open_microwave()'s dict-key/list-length
        # requirements (push_waypoints[-5] needs len>=5); their values are
        # never read in this robot_interface=None run (see module docstring).
        return {
            "pre_grasp_pose": pre_grasp_pose,
            "grasp_pose": grasp_pose,
            "opening_waypoints": [],
            "post_release_pose": grasp_pose,
            "pre_push_pose": grasp_pose,
            "push_pose": grasp_pose,
            "push_waypoints": [grasp_pose] * 5,
            "before_above_closing_waypoint": grasp_pose,
            "above_closing_waypoint": grasp_pose,
            "closing_waypoint": grasp_pose,
            "closing_waypoints": [],
        }


def main() -> None:
    scene_config_path = REPO_ROOT / "src" / "feeding_deployment" / "simulation" / "configs" / "vention.yaml"
    scene_description = create_scene_description_from_config(str(scene_config_path), "skewer")
    # --real_sim: bonus check of a suspected dormant bug -- move_to_joint_positions/
    # move_to_ee_pose always pass plan=None to sim.visualize_plan() (the actual
    # planning call is commented out repo-wide), and FeedingDeploymentPyBulletSimulator
    # .visualize_plan() does `for sim_state in plan`, which should TypeError on None.
    # NullSimulator.visualize_plan() tolerates any args, which is why the default
    # path above doesn't hit this.
    if "--real_sim" in sys.argv:
        sim = FeedingDeploymentPyBulletSimulator(scene_description, use_gui=False)
    else:
        sim = NullSimulator(scene_description)

    hla_hyperparams = {"max_motion_planning_time": 10.0}
    open_door_hla = OpenDoorHLA(
        sim,
        None,  # robot_interface -- sim only, no hardware
        StubMicrowavePerceptionAdapter(),
        None,  # rviz_interface
        None,  # web_interface
        hla_hyperparams,
        None,  # wrist_interface
        None,  # flair
        True,  # no_waits
        LOG_DIR,
        FACTORY_BT_DIR,  # read-only: this run never calls process_behavior_tree_*
        LOG_DIR / "execution_log.txt",
        LOG_DIR,
    )

    objects = (Object("microwave", microwave_type),)
    print(f"Constructed OpenDoorHLA duck-typed with stub perception + NullSimulator.")
    print(f"Behavior tree file resolved to: {open_door_hla.get_behavior_tree_filename(objects, {})}")
    print("Ticking the real open_microwave.yaml behavior tree via execute_action() ...")
    open_door_hla.execute_action(objects, {})
    print("\nSUCCESS: open_microwave behavior tree ticked to completion with no exceptions.")


if __name__ == "__main__":
    main()
