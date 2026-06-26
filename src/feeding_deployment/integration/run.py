"""The main entry point for running the integrated system."""

import json
from collections import namedtuple
from pathlib import Path
from typing import Any, Callable, List
import queue
import os
import sys
import signal
import shutil
import numpy as np
from feeding_deployment.utils.anthropic_llm import AnthropicLLM
import time
import types
import inspect
import base64
import itertools
from dataclasses import dataclass
from PIL import Image

try:
    import rospy
    from std_msgs.msg import String

    ROSPY_IMPORTED = True
except ModuleNotFoundError:
    ROSPY_IMPORTED = False

from relational_structs import (
    GroundAtom,
    LiftedAtom,
    Object,
    PDDLDomain,
    PDDLProblem,
    Predicate,
)
from relational_structs.utils import parse_pddl_plan, get_object_combinations
from tomsutils.pddl_planning import run_pddl_planner
from tomsutils.spaces import EnumSpace
from pybullet_helpers.geometry import Pose

from feeding_deployment.actions.base import (
    object_type,
    tool_type,
    plate_type,
    plate_location_type,
    nav_target_type,
    table_type,
    sink_type,
    appliance_type,
    fridge_type,
    microwave_type,
    holder_type,
    GripperFree,
    Holding,
    IsUtensil,
    PlateInView,
    ToolPrepared,
    ToolTransferDone,
    EmulateTransferDone,
    ResetPos,
    HomePos,
    InFrontOf,
    DoorOpen,
    DoorClosed,
    PlateAt,
    FoodHeated,
    SafeToNavigate,
    TableSeen,
    GroundHighLevelAction,
    ResetHLA,
    HomeHLA,
    pddl_plan_to_hla_plan,
    interpret_user_update_request,
    load_behavior_tree,
    save_behavior_tree,
    NodeModificationUserUpdateRequest,
    NodeAdditionUserRequest,
    UserUpdateRequest,
    ParameterizedActionBehaviorTreeNode,
    TeleopTakeoverException,
)
from feeding_deployment.actions.navigate import NavigateHLA
from feeding_deployment.actions.open_door import OpenDoorHLA
from feeding_deployment.actions.close_door import CloseDoorHLA
from feeding_deployment.actions.pick_plate import (
    PickPlateFromApplianceHLA,
    PickPlateFromHolderHLA,
    PickPlateFromTableHLA,
)
from feeding_deployment.actions.place_plate import (
    PlacePlateOnHolderHLA,
    PlacePlateInApplianceHLA,
    PlacePlateOnTableHLA,
    PlacePlateInSinkHLA,
)
from feeding_deployment.actions.press_microwave_button import PressMicrowaveButtonHLA
from feeding_deployment.actions.pick_tool import PickToolHLA
from feeding_deployment.actions.stow_tool import StowToolHLA
from feeding_deployment.actions.transfer_tool import TransferToolHLA
from feeding_deployment.actions.emulate_transfer import EmulateTransferHLA
from feeding_deployment.actions.acquisition import AcquireBiteHLA
from feeding_deployment.actions.gaze_at_table import GazeAtTableHLA
from feeding_deployment.interfaces.perception_interface import PerceptionInterface
from feeding_deployment.interfaces.web_interface import WebInterface
from feeding_deployment.interfaces.rviz_interface import RVizInterface
from feeding_deployment.control.robot_controller.arm_client import ArmInterfaceClient
from feeding_deployment.control.wrist_controller.wrist_controller import WristInterface
from feeding_deployment.simulation.scene_description import (
    SceneDescription,
    create_scene_description_from_config,
)
from feeding_deployment.simulation.simulator import (
    FeedingDeploymentPyBulletSimulator,
    FeedingDeploymentWorldState,
    NullSimulator,
)
from feeding_deployment.actions.flair.flair import FLAIR
from feeding_deployment.transparency.query_llm import TransparencyQuery
from feeding_deployment.integration.preference_context import build_preference_context
from feeding_deployment.integration.checkpoint import CheckpointStore
from feeding_deployment.integration.data_logger import DataLogger
from feeding_deployment.integration.preference_session import PreferenceSession
from feeding_deployment.preference_learning.methods.prediction_model import PredictionModel, PREF_OPTIONS
from feeding_deployment.preference_learning.config.physical_capabilities import (
    PHYSICAL_CAPABILITY_PROFILES,
)
from feeding_deployment.preference_learning.config import MEALS, SETTINGS, TIMES_OF_DAY

# Used for preference prediction when --physical_profile_file is not passed.
DEFAULT_PHYSICAL_PROFILE = (
    "This user has moderate voluntary control of their arms and is able to press "
    "physical buttons. They can lean forward to reach food during outside-mouth "
    "transfers. They have good neck and head control and can open their mouth wide "
    "and perform head gestures. They interact with the web interface on their "
    "personal device using their arms."
)

# Preference dimensions asked at the start of the meal (before fetching the
# plate). The finalized wait drives the autocontinue of later correction pages.
_INITIAL_PREF_DIMS = ["robot_speed", "wait_before_autocontinue_seconds"]

# Preference dimensions asked at the table, just before feeding begins.
_TABLE_PREF_DIMS = [
    "skewering_axis",
    "web_interface_confirmation",
    "bite_dipping_preference",
    "transfer_mode",
    "outside_mouth_distance",
    "convey_robot_ready_for_initiating_transfer",
    "convey_robot_ready_for_completing_transfer",
    "detect_user_ready_for_initiating_transfer_feeding",
    "detect_user_ready_for_initiating_transfer_drinking",
    "detect_user_ready_for_initiating_transfer_wiping",
    "detect_user_completed_transfer_feeding",
    "detect_user_completed_transfer_drinking",
    "detect_user_completed_transfer_wiping",
    "retract_between_bites",
]

# All the high level actions we want to consider.
HLAS = {
    NavigateHLA,
    OpenDoorHLA,
    CloseDoorHLA,
    PickToolHLA,
    StowToolHLA,
    PickPlateFromApplianceHLA,
    PickPlateFromHolderHLA,
    PickPlateFromTableHLA,
    PlacePlateOnHolderHLA,
    PlacePlateInApplianceHLA,
    PlacePlateOnTableHLA,
    PlacePlateInSinkHLA,
    PressMicrowaveButtonHLA,
    AcquireBiteHLA,
    TransferToolHLA,
    EmulateTransferHLA,
    ResetHLA,
    HomeHLA,
    GazeAtTableHLA,
}

assert os.environ.get("PYTHONHASHSEED") == "0", \
        "Please add `export PYTHONHASHSEED=0` to your bash profile!"

class _Runner:
    """A class for running the integrated system."""

    def __init__(self, scene_config: str, user: str, transfer_type: str, run_on_robot: bool, use_interface: bool, use_gui: bool, simulate_head_perception: bool, max_motion_planning_time: float,
                 resume_from_state: str = "", no_waits: bool = False,
                 physical_profile_label: str | None = None,
                 pref_mode: str = "none",
                 day: int | None = None) -> None:
        self.run_on_robot = run_on_robot
        self.use_interface = use_interface
        self.simulate_head_perception = simulate_head_perception
        self.max_motion_planning_time = max_motion_planning_time
        self.no_waits = no_waits
        self.deployment_user = user
        self.physical_profile_label = physical_profile_label.strip() if physical_profile_label else None
        # The deployment day number (mandatory CLI --day): the single source of
        # truth for both per-day release logging and the preference-learning day.
        self._day = day
        self._pref_mode = pref_mode
        self._prediction_model: PredictionModel | None = None
        self._pref_session: PreferenceSession | None = None
        self.predicted_bundle: dict[str, str] | None = None

        # Cross-day persistent state (behavior trees, gesture detectors, llm
        # cache, preference memory, latest perception poses) lives directly under
        # the user directory -- there is no scenario level. The per-day release
        # record goes in log/<user>/day_<NN>/ (see self.data_logger below).
        self.log_dir = Path(__file__).parent / "log" / user
        self.execution_log = Path(__file__).parent / "log" / "execution_log.txt" # in root log directory
        self.run_behavior_tree_dir = self.log_dir / "behavior_trees"
        self.gesture_detectors_dir = self.log_dir / "gesture_detectors"
        self._gesture_detection_filepath = self.gesture_detectors_dir / "synthesized_gesture_detectors.py"

        if not self.log_dir.exists():  # new user: seed persistent state from defaults
            os.makedirs(self.log_dir, exist_ok=True)

            # Copy the initial behavior trees into the user directory, where they
            # will be modified based on user feedback.
            self.run_behavior_tree_dir.mkdir(exist_ok=True)
            original_behavior_tree_dir = Path(__file__).parents[1] / "actions" / "behavior_trees"
            assert original_behavior_tree_dir.exists()
            for original_bt_filename in original_behavior_tree_dir.glob("*.yaml"):
                shutil.copy(original_bt_filename, self.run_behavior_tree_dir)

            # Copy the initial gesture detection file into the user directory,
            # where it will be updated from LLM-based few-shot learning.
            self.gesture_detectors_dir.mkdir(exist_ok=True)
            original_gesture_detection_filepath = Path(__file__).parents[1] / "perception" / "gestures_perception" / "synthesized_gesture_detectors.py"
            assert original_gesture_detection_filepath.exists()
            shutil.copy(original_gesture_detection_filepath, self.gesture_detectors_dir)

        # Single logs handle: owns the shared state dir (== self.log_dir) and, when
        # --day is given, the per-day release record under log/<user>/day_<NN>/.
        # Disabled (no-op for release logging) when --day is omitted.
        self.data_logger = DataLogger(state_dir=self.log_dir, day=day)

        # Checkpointing. The store owns saved_states/: numbered NN_<skill>.p
        # checkpoints track the deterministic plate journey (prep + finish), the
        # four after_*_pickup.p are the feeding recovery points, and last_state.p
        # always holds the last completed skill. See _save_state / _load_from_state.
        self._ckpt = CheckpointStore(Path(__file__).parent / "saved_states")
        # Resume target (base name) or None for a fresh run.
        self._resume_state_name: str | None = resume_from_state or None
        # Set by _load_from_state on resume: the phase the checkpoint was taken
        # in ("prep" | "feeding" | "finish"), the restored preference snapshot,
        # and the physical profile the checkpoint was produced under.
        self._resume_phase: str | None = None
        self._resume_pref_state: dict | None = None
        self._resume_physical_profile: str | None = None
        self._resume_day: int | None = None

        if resume_from_state == "":
            # clear behavior tree execution log
            with open(self.execution_log, "w") as f:
                f.write("")
            # Fresh run: drop the previous meal's numbered checkpoints (their
            # numbering is only valid within one meal's plan). last_state.p,
            # after_*_pickup.p, and manual *.pkl files are preserved.
            self._ckpt.clear_ephemeral()
            # Also drop the standalone preference snapshot so a new meal never
            # inherits the previous meal's in-progress corrections.
            self._ckpt.clear_pref()

        # Initialize the interface to the robot.
        if run_on_robot:
            self.robot_interface = ArmInterfaceClient()  # type: ignore  # pylint: disable=no-member
            self.wrist_interface = WristInterface()
            self.robot_interface.set_speed("medium")
        else:
            self.robot_interface = None
            self.wrist_interface = None

        self.llm = AnthropicLLM(
            model_name="claude-opus-4-8",
            cache_dir=self.log_dir / "llm_cache",
        )

        # Initialize the perceiver (e.g., get joint states or human head poses).
        self.perception_interface = PerceptionInterface(robot_interface=self.robot_interface, simulate_head_perception=self.simulate_head_perception, data_logger=self.data_logger)

        # Initialize the simulator.
        scene_config_path = Path(__file__).parent.parent / "simulation" / "configs" / f"{scene_config}.yaml"
        self.scene_description = create_scene_description_from_config(str(scene_config_path), transfer_type)

        if run_on_robot:
            if not np.allclose(self.scene_description.initial_joints, self.perception_interface.get_robot_joints(), atol=0.2):
                print("Initial joint state in scene description does not match the actual robot joint state.")
                print("Initial Robot Joints:", self.perception_interface.get_robot_joints())
                print("Initial Joints in Scene Description:", self.scene_description.initial_joints)
                
        else:
            print("Running in simulation mode.")

        grounded_sam = self.perception_interface._grounded_sam if hasattr(self.perception_interface, '_grounded_sam') else None
        self.flair = FLAIR(self.log_dir, grounded_sam=grounded_sam)

        if self.run_on_robot:
            self.rviz_interface = RVizInterface(self.scene_description)
        else:
            self.rviz_interface = None

        if self.no_waits:
            self.sim = NullSimulator(self.scene_description)
        else:
            self.sim = FeedingDeploymentPyBulletSimulator(self.scene_description, use_gui=use_gui, ignore_user=True)

        if self.use_interface:
            # Initialize the web interface.
            self.task_selection_queue = queue.Queue()
            self.web_interface = WebInterface(self.task_selection_queue, self.data_logger)
        else:
            self.web_interface = None

        # Let a "Take Over" request best-effort abort the in-flight arm move.
        if self.web_interface is not None and self.robot_interface is not None:
            self.web_interface.register_takeover_stop(self.robot_interface.stop_action)

        # Create skills for high-level planning.
        hla_hyperparams = {"max_motion_planning_time": max_motion_planning_time}
        print("Creating HLAs...")
        self.hlas = {
            cls(self.sim, self.robot_interface, self.perception_interface, self.rviz_interface, self.web_interface, hla_hyperparams,
                self.wrist_interface, self.flair, self.no_waits, self.log_dir, self.run_behavior_tree_dir, self.execution_log, self.gesture_detectors_dir,
                self.register_gesture_detector, self.load_synthesized_gestures) for cls in HLAS  # type: ignore
        }
        print("HLAs created.")
        self.hla_name_to_hla = {hla.get_name(): hla for hla in self.hlas}
        self.operators = {hla.get_operator() for hla in self.hlas}
        self.predicates: set[Predicate] = {
            ToolPrepared,
            GripperFree,
            Holding,
            ToolTransferDone,
            EmulateTransferDone,
            IsUtensil,
            PlateInView,
            ResetPos,
            HomePos,
            InFrontOf,
            DoorOpen,
            DoorClosed,
            PlateAt,
            FoodHeated,
            SafeToNavigate,
            TableSeen,
        }
        self.types = {
            object_type,
            plate_type,
            tool_type,
            plate_location_type,
            nav_target_type,
            sink_type,
            table_type,
            appliance_type,
            fridge_type,
            microwave_type,
            holder_type,
        }
        self.domain = PDDLDomain(
            "AssistedFeeding", self.operators, self.predicates, self.types
        )

        self.drink = Object("drink", tool_type)
        self.wipe = Object("wipe", tool_type)
        self.utensil = Object("utensil", tool_type)
        self.plate = Object("plate", plate_type)

        self.fridge = Object("fridge", fridge_type)
        self.microwave = Object("microwave", microwave_type)

        self.holder = Object("holder", holder_type)

        self.sink = Object("sink", sink_type)
        self.table = Object("table", table_type)

        self.all_objects = {
            self.drink,
            self.wipe,
            self.utensil,
            self.plate,
            self.fridge,
            self.microwave,
            self.sink,
            self.table,
            self.holder,
        }
        self.object_name_to_object = {
            "drink": self.drink,
            "wipe": self.wipe,
            "utensil": self.utensil,
            "plate": self.plate,
            "fridge": self.fridge,
            "microwave": self.microwave,
            "sink": self.sink,
            "table": self.table,
            "holder": self.holder,
        }
        # Create all ground HLAs that will be used.
        self._all_ground_hlas = []
        for hla_name, hla in sorted(self.hla_name_to_hla.items()):
            types = [p.type for p in hla.get_operator().parameters]
            for obj_combo in get_object_combinations(sorted(self.all_objects), types):
                # Major hack. The proper way to do this would be to define subtypes
                if "AcquireBite" in hla_name:
                    assert len(obj_combo) == 2
                    if obj_combo[0].name != "utensil" or obj_combo[1].name != "table":
                        # print(f"Skipping invalid HLA grounding: {hla_name} with object {obj_combo[0].name}")
                        continue
                if hla_name == "Navigate":
                    assert len(obj_combo) == 2
                    if obj_combo[0] == obj_combo[1]:
                        # print(f"Skipping invalid HLA grounding: {hla_name} with identical nav targets {obj_combo[0].name}")
                        continue
                if hla_name == "PressMicrowaveButton":
                    assert len(obj_combo) == 1
                    if obj_combo[0].name != "microwave":
                        # print(f"Skipping invalid HLA grounding: {hla_name} with object {obj_combo[0].name}")
                        continue
                if hla_name == "PickPlateFromTable" or hla_name == "PlacePlateOnTable":
                    assert len(obj_combo) == 2
                    if obj_combo[0].name != "plate" or obj_combo[1].name != "table":
                        # print(f"Skipping invalid HLA grounding: {hla_name} with objects {obj_combo[0].name}, {obj_combo[1].name}")
                        continue
                if hla_name == "PickPlateFromHolder" or hla_name == "PlacePlateOnHolder":
                    assert len(obj_combo) == 2
                    if obj_combo[0].name != "plate" or obj_combo[1].name != "holder":
                        # print(f"Skipping invalid HLA grounding: {hla_name} with objects {obj_combo[0].name}, {obj_combo[1].name}")
                        continue
                if hla_name == "PlacePlateInSinkHLA":
                    assert len(obj_combo) == 2
                    if obj_combo[0].name != "plate" or obj_combo[1].name != "sink":
                        # print(f"Skipping invalid HLA grounding: {hla_name} with objects {obj_combo[0].name}, {obj_combo[1].name}")
                        continue
                if hla_name in {"PlacePlateInAppliance", "PickPlateFromAppliance"}:
                    assert len(obj_combo) == 2
                    if obj_combo[0].name != "plate" or obj_combo[1].name not in {"fridge", "microwave"}:
                        # print(f"Skipping invalid HLA grounding: {hla_name} with objects {obj_combo[0].name}, {obj_combo[1].name}")
                        continue
                if hla_name in {"PickTool", "StowTool", "TransferTool"}:
                    assert len(obj_combo) == 2
                    if obj_combo[0].name == "plate" or obj_combo[1].name != "table":
                        # print(f"Skipping invalid HLA grounding: {hla_name} with object {obj_combo[0].name}")
                        continue
                # print(f"Adding ground HLA: {hla_name} with objects {[obj.name for obj in obj_combo]}")
                ground_hla = (hla, obj_combo)
                self._all_ground_hlas.append(ground_hla)
        # Rewrite the behavior trees to avoid any inconsistencies.
        for hla, objs in self._all_ground_hlas:
            # Super Hack: skip the plate transfer behavior tree.
            if hla == self.hla_name_to_hla["TransferTool"] and objs[0].name == "plate":
                continue
            try:
                bt_filepath = hla.behavior_tree_dir / hla.get_behavior_tree_filename(objs, {})
            except NotImplementedError:
                continue
            bt = load_behavior_tree(bt_filepath, hla)
            save_behavior_tree(bt, bt_filepath, hla)

        # Track the current high-level state.
        # Staged personalization starts the plate in the fridge so the meal
        # begins with the robot retrieving it (and the fridge color correction).
        # With no personalization, keep the legacy start (plate on the holder).
        plate_start = self.fridge if self._pref_mode != "none" else self.holder
        self.current_atoms = {
            GroundAtom(GripperFree, []),
            GroundAtom(ToolPrepared, [self.wipe]),
            GroundAtom(ToolPrepared, [self.drink]),
            GroundAtom(IsUtensil, [self.utensil]),
            GroundAtom(DoorClosed, [self.fridge]),
            GroundAtom(DoorClosed, [self.microwave]),
            GroundAtom(InFrontOf, [self.microwave]),
            GroundAtom(PlateAt, [plate_start]),
            # GroundAtom(Holding, [self.plate]),
            GroundAtom(SafeToNavigate, []),
            # GroundAtom(FoodHeated, []),
        }

        self.transparency_query = TransparencyQuery(self.log_dir)
        print("Initialized transparency query.")

        if self._resume_state_name:
            self._load_from_state()
            print("WARNING: The system state has been restored to:")
            print(f"  phase={self._resume_phase}, last completed skill #{self._ckpt.index}")
            print(" ", sorted(self.current_atoms))

            # Physical profile mismatch: the checkpoint was produced under one
            # profile; resuming under a different one taints any post-resume
            # repredictions and the end-of-meal learning update (predictions are
            # profile-dependent). Already-finalized preferences are restored from
            # the snapshot and are unaffected.
            if self._resume_physical_profile != self.physical_profile_label:
                print("WARNING: physical profile DIFFERS from the checkpoint's profile!")
                print("  checkpoint profile:", self._resume_physical_profile)
                print("  current profile:   ", self.physical_profile_label)
                print("  Continuing predicts remaining/learned preferences under the new "
                      "profile, inconsistent with the rest of this meal.")
                if not self._confirm("Continue despite the profile mismatch? [y/n] "):
                    self.stop_all_threads()
                    sys.exit(0)

            # Day mismatch: the checkpoint records the day it was produced on.
            # Resuming under a different --day would log this meal's release data
            # and write its preference memory under the wrong day number.
            if self._resume_day is not None and self._resume_day != self._day:
                print("WARNING: --day DIFFERS from the checkpoint's day!")
                print("  checkpoint day:", self._resume_day)
                print("  current --day: ", self._day)
                print("  Continuing logs release data and writes preference memory "
                      "under the current --day, not the day this meal started on.")
                if not self._confirm("Continue despite the day mismatch? [y/n] "):
                    self.stop_all_threads()
                    sys.exit(0)

            if not self._confirm("Are you sure you want to continue from here? [y/n] "):
                self.stop_all_threads()
                sys.exit(0)

        print("Runner is ready.")
        self.active = True
        self.preference_context: dict[str, str] | None = None

    def ensure_preference_context(self) -> dict[str, str]:
        """Require a valid preference context before the web session; no implicit defaults."""
        if self.preference_context is None:
            raise RuntimeError(
                "preference_context is required but unset. Each run must set it explicitly "
                "(e.g. non-empty --pref_meal with --use_interface, or call "
                "set_meal_preference_context(meal, setting, time_of_day) before run()). "
                "Context is not loaded from or saved to disk; after a crash, supply it again."
            )
        return self.preference_context

    def set_meal_preference_context(
        self,
        meal: str,
        setting: str,
        time_of_day: str,
    ) -> dict[str, str]:
        """Validated observable context for this run (meal / setting / time); in-memory only."""
        self.preference_context = build_preference_context(
            meal=meal,
            setting=setting,
            time_of_day=time_of_day,
        )
        return self.preference_context

    # ------------------------------------------------------------------ #
    # Staged personalization
    # ------------------------------------------------------------------ #
    def _collect_preference_context(self) -> dict[str, str]:
        """Observable meal context for this run. Honors an explicit
        --pref_meal preset; otherwise collects it via terminal or the web
        preference_context page."""
        if self.preference_context is not None:
            return self.preference_context  # preset via --pref_meal (debug/replay)

        if self._pref_mode == "terminal":
            from feeding_deployment.integration.terminal_preferences import (
                terminal_collect_context,
            )
            ctx_dict = terminal_collect_context()
        else:
            ctx_dict = self.web_interface.get_meal_context(
                meals=list(MEALS),
                settings=list(SETTINGS),
                times_of_day=list(TIMES_OF_DAY),
            )
        return self.set_meal_preference_context(
            meal=ctx_dict["meal"],
            setting=ctx_dict["setting"],
            time_of_day=ctx_dict["time_of_day"],
        )

    def _build_prediction_model(self) -> PredictionModel:
        """Construct the preference prediction model. Used both on a fresh meal
        and when rehydrating a resumed session (the model is never pickled)."""
        assert self.physical_profile_label is not None, (
            "physical_profile_label is required for preference prediction "
            "(pass --physical_profile_file)."
        )
        assert self._day is not None, "--day is required for preference prediction"
        model = PredictionModel(
            user=self.deployment_user,
            physical_profile_label="deployment_physical_profile",
            logs_dir=self.log_dir / "preference_learning",
            physical_profile_description=self.physical_profile_label,
        )
        # Reject day gaps, then re-hydrate cross-day memory (LTM summary +
        # episodic history) from days strictly before today, so this fresh
        # process predicts with the accumulated learning of days 1..N-1.
        model.validate_sequential_day(self._day)
        model.load_prior_memory(self._day)
        return model

    def _build_preference_session(self, model: PredictionModel, ctx: dict) -> PreferenceSession:
        """Construct a PreferenceSession wired to this run's interfaces. Terminal
        mode reuses the staged session via a stdin correction adapter."""
        correction_interface = self.web_interface
        if self._pref_mode == "terminal":
            from feeding_deployment.integration.terminal_preferences import (
                TerminalCorrectionInterface,
            )
            correction_interface = TerminalCorrectionInterface()
        return PreferenceSession(
            model,
            self.run_behavior_tree_dir,
            dict(ctx),
            web_interface=correction_interface,
            data_logger=self.data_logger,
            scene_description=self.sim.scene_description,
            hla_map=self.hla_name_to_hla,
            flair=self.flair,
            # Persist the session on every locked correction so a crash before the
            # next sub-skill checkpoint loses nothing (see _save_state / resume).
            on_change=self._ckpt.save_pref,
        )

    def _restore_preference_session(self, state: dict) -> None:
        """Rehydrate the per-meal preference session on resume: rebuild the model
        and re-apply the restored bundle to the BTs/scene WITHOUT predicting or
        asking. After this, finalized dims are skipped by ask() and finalize_meal
        still reflects pre-crash corrections."""
        self.preference_context = dict(state["context"])
        self._prediction_model = self._build_prediction_model()
        self._pref_session = self._build_preference_session(
            self._prediction_model, self.preference_context
        )
        self._pref_session.resume_from_state(state)
        print("Restored preference session from checkpoint:",
              json.dumps(self._pref_session._loggable_bundle(), indent=2))

    def _start_preference_session(self, resume: bool = False) -> None:
        """Predict (or, on resume, reuse the restored bundle), ask the initial
        dims, then run the staged meal preparation (fetch plate, microwave, place
        on table) asking the relevant prefs at each stage.

        On ``resume`` the session has already been rehydrated via
        _restore_preference_session: prediction is skipped, and the already-asked
        dims are skipped by PreferenceSession.ask(); only still-open dims are
        asked and only not-yet-done skills actually execute (the planner replans
        completed steps to empty plans)."""
        if resume:
            assert self._pref_session is not None, "resume requires a restored session"
            ctx = self.ensure_preference_context()
            print("Resuming preference session; context:", ctx)
        else:
            ctx = self._collect_preference_context()
            print("Preference context (meal / setting / time_of_day):", ctx)
            self._prediction_model = self._build_prediction_model()
            self._pref_session = self._build_preference_session(self._prediction_model, dict(ctx))
            # Predict everything before asking the initial dims (speed +
            # autocontinue wait). The finalized wait drives later autocontinue.
            self._pref_session.start()
            print("Predicted preference bundle (initial):",
                  json.dumps(self._pref_session._loggable_bundle(), indent=2))

        self._pref_session.ask(_INITIAL_PREF_DIMS)

        if resume:
            # The prep pipeline index to resume from was restored from the
            # checkpoint (steps before it completed on the prior run).
            assert self._resume_prep_step is not None, "prep-phase resume requires resume_prep_step"
            resume_from_step = self._resume_prep_step
        else:
            # First durable sim checkpoint of the meal, written BEFORE any skill
            # executes, so even a kill during the very first navigation is
            # resumable (the initial prefs already ride in pref_session.p). This
            # also overwrites any stale last_state.p left by a previous meal.
            # resume_prep_step=0 -> a resume from here re-runs the whole pipeline.
            self._save_state(
                self.sim.get_current_state(), self.current_atoms,
                phase="prep", completed_skill="meal_start", resume_prep_step=0,
            )
            resume_from_step = 0

        self._run_meal_preparation(resume_from_step)

    # Fixed, ordered prep pipeline. These indices ARE the resume_prep_step values
    # saved in checkpoints, so they must stay stable. Steps with a process_user_command
    # checkpoint their progress; the two ask() steps do not (their dims are restored
    # via pref_session.p and re-asking is idempotent).
    _PREP_PLACE_ON_HOLDER = 0
    _PREP_CLOSE_FRIDGE = 1
    _PREP_MICROWAVE = 2
    _PREP_PLACE_ON_TABLE = 3
    _PREP_TABLE_DIMS = 4

    def _run_meal_preparation(self, resume_from_step: int = 0) -> None:
        """Run the fixed prep pipeline from ``resume_from_step``.

        Each step has a stable index (the ``_PREP_*`` constants). Steps before
        ``resume_from_step`` completed on a prior run and are skipped. The
        in-progress step (== resume_from_step) is re-issued; its process_user_command
        re-plans from the restored atoms, so the planner emits only the remaining
        sub-skills (e.g. an already-heated plate keeps FoodHeated, so no microwave
        detour). This replaces the old per-command "skip if already achieved"
        guards -- completed work is skipped by index, not re-issued-then-detected."""
        session = self._pref_session

        if resume_from_step <= self._PREP_PLACE_ON_HOLDER:
            # Fridge -> holder (fridge color correction happens during the pickup).
            self.process_user_command(
                GroundHighLevelAction(self.hla_name_to_hla["PlacePlateOnHolder"], (self.plate, self.holder)),
                phase="prep", prep_step=self._PREP_PLACE_ON_HOLDER,
            )

        if resume_from_step <= self._PREP_CLOSE_FRIDGE:
            # Close the fridge before pausing to ask the microwave preference, so it
            # is not left hanging open while we wait on the user (the planner would
            # otherwise close it only as the first step of PlacePlateOnTable).
            self.process_user_command(
                GroundHighLevelAction(self.hla_name_to_hla["CloseDoor"], (self.fridge,)),
                phase="prep", prep_step=self._PREP_CLOSE_FRIDGE,
            )

        if resume_from_step <= self._PREP_MICROWAVE:
            # Microwave preference (single dim). "no microwave" sets FoodHeated so
            # the planner serves directly; a duration leaves it unset so the planner
            # routes through the microwave and writes the BT duration. Skipped by
            # index on a resume past here -> an already-heated plate's FoodHeated is
            # preserved (no re-heat).
            session.ask(["microwave_time"])
            microwave_duration = session.apply_microwave(self.current_atoms, GroundAtom(FoodHeated, []))
            if microwave_duration is None:
                print("Microwave preference: no microwave (FoodHeated added to planner state).")
            else:
                print(f"Microwave preference: {microwave_duration}s (planner will include microwave steps).")

        if resume_from_step <= self._PREP_PLACE_ON_TABLE:
            # (Holder ->) [microwave ->] table. Microwave color correction happens
            # during the microwave pickup if the plate is routed there.
            self.process_user_command(
                GroundHighLevelAction(self.hla_name_to_hla["PlacePlateOnTable"], (self.plate, self.table)),
                phase="prep", prep_step=self._PREP_PLACE_ON_TABLE,
            )

        if resume_from_step <= self._PREP_TABLE_DIMS:
            # Table dims, just before feeding.
            session.ask(_TABLE_PREF_DIMS)

    def _finalize_preference_session(self) -> None:
        """One memory update at meal end with the full finalized bundle."""
        if self._pref_session is None:
            return
        assert self._day is not None, "--day is required"
        day = self._day
        existing = self._prediction_model.working_memory_dir / f"day_{day:04d}.json"
        if existing.exists():
            print(f"[learn] NOTE: day {day} memory already exists; finalize will OVERWRITE "
                  f"day_{day:04d}.json (working/episodic/long_term).")
        print(f"[learn] Updating memory models (day {day}) ...")
        self._pref_session.finalize_meal(day)
        print(f"[learn] Memory update complete (day {day}).")
        self._pref_session = None

    def run(self, continuous = True) -> None:

        assert self.web_interface is not None, "Run takes user commands from the web interface which is None."

        # Gate the meal on the user pressing "Start Meal" on the home page. Home
        # is the webapp's default/refresh route, so waiting for this user-initiated
        # press (rather than blindly jumping forward) keeps startup robust against
        # refreshes and slow webapp connections.
        self.web_interface.wait_for_start_meal()

        resuming = self._resume_phase is not None
        if resuming:
            # --- Resume: the world state/atoms were restored in __init__. Rebuild
            # the preference session (if the crashed meal had one), then route by
            # the phase the checkpoint was taken in. Only "prep" replays the
            # staged meal preparation (which skips done skills via empty replans
            # and skips finalized prefs); "feeding"/"finish" jump straight to the
            # task-selection page with atoms matching the real world. ---
            if self._resume_pref_state is not None:
                self._restore_preference_session(self._resume_pref_state)
            if self._resume_phase == "prep":
                self._start_preference_session(resume=True)
            else:
                print(f"Resuming in phase '{self._resume_phase}'; skipping meal preparation.")
            self.web_interface.ready_for_task_selection()
        elif self._pref_mode == "none":
            self.web_interface.ready_for_task_selection()
        else:
            # --- Staged personalization: predict full bundle, then ask the
            # initial dims. The rest are asked just-in-time during the meal
            # (microwave at the holder, table dims at the table) and colors at
            # each pickup. A single memory update happens at meal end. ---
            self._start_preference_session()
            self.web_interface.ready_for_task_selection()
        last_task_type = None
        while self.active:
            # Take-Over pressed while idle (no skill running): launch teleop here,
            # then return to task selection. (Mid-skill takeovers are handled in
            # execute_robot_command, not here.)
            if self.web_interface is not None and self.web_interface.consume_takeover():
                print("User-initiated takeover while idle; launching teleop ...")
                try:
                    self.hla_name_to_hla["Reset"].run_manual_teleop_recovery(failure_context="user_initiated_idle")
                except Exception as e:
                    print(f"Manual teleop recovery error: {e}")
                self.web_interface.ready_for_task_selection()
                continue
            if not continuous:
                resp = input("Press 'y' to continue RUNNER, 'n' to stop: ")
                while resp not in ["y", "n"]:
                    resp = input("Press 'y' to continue RUNNER, 'n' to stop: ")
                if resp == "n":
                    break
            try:
                task_selection_command = self.task_selection_queue.get(timeout=1)
                self.web_interface.clear_received_messages() # So that only the latest message is processed
                task, task_type = task_selection_command["task"], task_selection_command["type"]
                self.data_logger.log_event("task_command", task=task, task_type=task_type)
                if task == "reset":
                    self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["Reset"], ()))
                    last_task_type = None
                elif task == "finish_feeding":
                    # PlacePlateInSink plans PickPlateFromTable first, so the
                    # table color correction happens here. After the plate is
                    # away, write the single per-day memory update.
                    self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["PlacePlateInSink"], (self.plate, self.sink)), phase="finish")
                    self._finalize_preference_session()
                    last_task_type = None
                elif task == "meal_assistance":
                    if task_type == "bite":
                        self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["TransferTool"], (self.utensil,self.table)))
                    elif task_type == "sip":
                        self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["TransferTool"], (self.drink,self.table)))
                    elif task_type == "wipe":
                        self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["TransferTool"], (self.wipe,self.table)))
                    last_task_type = task_type
                elif task == "personalization":
                    if task_type == "transparency":
                        while self.active:
                            query = self.web_interface.get_transparency_request()
                            if query:
                                response = self.transparency_query.answer_query(query)
                                self.web_interface.update_transparency_response(response)
                            else:
                                break
                    elif task_type == "adaptability":
                        while self.active:
                            adaptation_request = self.web_interface.get_adaptability_request()
                            if adaptation_request:
                                user_input = input("Do you want to manually process this? (y/n): ")
                                while user_input not in ["y", "n"]:
                                    user_input = input("Please enter 'y' or 'n': ")
                                if user_input == "y":
                                    update_summary = input("Please enter the update summary to show on the web interface: ")
                                    self.web_interface.update_adaptability_response(update_summary)
                                else:
                                    try:
                                        print("Processing user update request:", adaptation_request)
                                        update_summary = self.process_user_update_request(adaptation_request)
                                        print('Processed user update request.')
                                        self.web_interface.update_adaptability_response(update_summary)
                                    except Exception as e:
                                        print(f"Update failed: {str(e)}")
                                        self.web_interface.update_adaptability_response("Adaptation failed. Please try rephrasing the request.")
                            else:
                                break
                    elif task_type == "gesture":
                        print("Triggered gesture")
                        gesture_task_type = self.web_interface.get_gesture_type()
                        print(f"Gesture task type: {gesture_task_type}")
                        if gesture_task_type == "add":
                            gesture_label, gesture_description = self.web_interface.get_new_gesture_details()
                            self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["EmulateTransfer"], (), {"test_mode": False, "gesture_label":gesture_label, "gesture_description": gesture_description} ))
                        else: # test
                            self.process_user_command(GroundHighLevelAction(self.hla_name_to_hla["EmulateTransfer"], (), {"test_mode": True} ))
                    last_task_type = task_type
                elif task == "teleop":
                    # User-initiated manual teleop recovery (between-tasks).
                    # Blocks until the user taps Done on the teleop screen.
                    print("Launching user-initiated manual teleop recovery ...")
                    try:
                        self.hla_name_to_hla["Reset"].run_manual_teleop_recovery(failure_context="user_initiated")
                    except Exception as e:
                        # Never let a teleop hiccup crash the executive loop.
                        print(f"Manual teleop recovery error: {e}")
                    last_task_type = None
                else:
                    print(f"Invalid task selection: {task_selection_command}")
                    last_task_type = None
                # self.web_interface.clear_received_messages() # So that only the latest message is processed
                # time.sleep(1.0)
                self.web_interface.ready_for_task_selection(last_task_type=last_task_type)
                print("Ready for next user command.")
                print("Current web interface page:", self.web_interface.current_page)
            except queue.Empty:
                # Wait for user commands.
                time.sleep(0.1) 
                continue

    def stop_all_threads(self) -> None:
        self.active = False
        if self.web_interface is not None:
            self.web_interface.stop_all_threads()
        self.data_logger.close()

    @staticmethod
    def _confirm(prompt: str) -> bool:
        """Block on a y/n prompt; return True for 'y', False for 'n'."""
        resp = input(prompt).strip().lower()
        while resp not in ("y", "n"):
            resp = input("Please enter 'y' or 'n': ").strip().lower()
        return resp == "y"

    def signal_handler(self, signal, frame):
        print("\nReceived SIGINT.")
        self.stop_all_threads()
        print("\nprogram exiting gracefully")
        sys.exit(0)

    def process_user_command(
        self, user_command: GroundHighLevelAction | set[GroundAtom],
        phase: str = "feeding",
        prep_step: int | None = None,
    ) -> None:
        """Process a user command.

        ``phase`` ("prep" | "feeding" | "finish") is set by the call site and
        controls checkpoint naming: prep/finish skills are numbered NN_<skill>.p
        (the deterministic plate journey), while feeding pickups get their fixed
        after_*_pickup.p recovery state. See CheckpointStore.

        ``prep_step``: the index of this command in the fixed prep pipeline (see
        _run_meal_preparation), or None for feeding/finish. Each sub-skill
        checkpoint records ``resume_prep_step`` = the prep step to (re)start from
        on resume: ``prep_step + 1`` once the command's last sub-skill completes,
        else ``prep_step`` (re-run it; the planner re-derives the remaining
        sub-skills from the restored atoms)."""

        print(f"Working towards user command: {user_command}")

        # Plan to the preconditions of the HLA.
        if isinstance(user_command, GroundHighLevelAction):
            goal_atoms = user_command.get_preconditions()
        else:
            goal_atoms = user_command
        
        if goal_atoms.issubset(self.current_atoms):
            print("Preconditions already satisfied; no planning needed.")
            plan_ops = []
        else:
            problem = PDDLProblem(
                self.domain.name,
                "AssistedFeeding",
                self.all_objects,
                self.current_atoms,
                goal_atoms,
            )

            # print("Current atoms:", sorted(self.current_atoms))
            # print("Goal atoms:", sorted(goal_atoms))
            # print("Operators:", [op.name for op in self.operators])
            # print("DOMAIN:")
            # print(str(self.domain))
            # print("PROBLEM:")
            # print(str(problem))

            plan_strs = run_pddl_planner(
                str(self.domain), str(problem), planner="fd-opt",
            )
            assert plan_strs is not None
            plan_ops = parse_pddl_plan(plan_strs, self.domain, problem)

        print("Found plan to the preconditions of the command:")
        for i, op in enumerate(plan_ops):
            print(f"{i}. {op.short_str}")
        plan_hlas = pddl_plan_to_hla_plan(plan_ops, self.hlas)
        # Append the user command to the plan if it's an action.
        if isinstance(user_command, GroundHighLevelAction):
            plan_hlas.append(user_command)

        # Build the ordered list of skill names (snake_case behavior-tree names,
        # e.g. "acquire_bite") so the web interface can show last/current/next.
        skill_plan_names = []
        for gh in plan_hlas:
            try:
                skill_plan_names.append(
                    gh.hla.get_behavior_tree_filename(gh.objects, gh.params).removesuffix(".yaml")
                )
            except Exception:
                skill_plan_names.append(gh.hla.get_name())

        # On the deterministic prep replay the webapp may still be on the home
        # page (no preference ask navigated it there). Move it to the execution
        # view before the skills run -- a single jump from whatever page it's on,
        # so there is no race with a back-to-back preference jump. Feeding/finish
        # navigate via their own selection/perception pages, so leave them be.
        if phase == "prep" and plan_hlas and self.web_interface is not None:
            self.web_interface.switch_to_explanation_page()

        for i, ground_hla in enumerate(plan_hlas):
            print(f"Refining {ground_hla}")
            # Tell the web interface which skill is now executing.
            if self.web_interface is not None:
                self.web_interface.publish_skill_plan(skill_plan_names, i)
            operator = ground_hla.get_operator()

            # import ipdb; ipdb.set_trace()
            assert operator.preconditions.issubset(self.current_atoms)

            # Execute the high-level plan in simulation. On a mid-skill takeover
            # the user chooses, via the teleop Done button, whether to redo this
            # skill (re-run it) or continue to the next (treat it as done, so we
            # fall through and apply its effects below).
            while True:
                try:
                    ground_hla.execute_action()
                    break
                except TeleopTakeoverException as e:
                    if e.redo_current:
                        print(f"User chose to redo {ground_hla} after teleop; re-running the skill.")
                        continue
                    print(f"User chose to continue past {ground_hla} after teleop; treating it as done.")
                    break
                except RuntimeError as e:
                    print(f"HLA execution failed: {e}")
                    print(f"Aborting task and returning to task selection page.")
                    if self.web_interface is not None:
                        self.web_interface.ready_for_task_selection()
                    return

            sim_state = self.sim.get_current_state()

            # Hack: if the action is navigation, we want to update the InFrontOf predicate for the target and remove it for all other navigation targets, since we assume the robot can only be in front of one navigation target at a time. For other actions, we just apply the add and delete effects as normal.
            self.current_atoms -= operator.delete_effects
            self.current_atoms |= operator.add_effects

            # if ground_hla.hla.get_name() == "Navigate":
            #     known_nav_targets = {self.fridge, self.microwave, self.sink, self.table}
            #     target = ground_hla.objects[0]
            #     for obj in known_nav_targets:
            #         if obj != target:
            #             self.current_atoms.discard(InFrontOf([obj]))

            # Super hack: the drink and wipe are always prepared.
            self.current_atoms.add(ToolPrepared([self.wipe]))
            self.current_atoms.add(ToolPrepared([self.drink]))

            # Color is a preference dimension: after a plate pickup, finalize
            # the location's plate color (the pickup wrote any user color
            # correction back to its BT YAML). A change propagates to the still-
            # open color dims via reprediction (same physical plate).
            if self._pref_session is not None:
                try:
                    bt_name = ground_hla.hla.get_behavior_tree_filename(
                        ground_hla.objects, ground_hla.params
                    ).removesuffix(".yaml")
                except Exception:
                    bt_name = ""
                if bt_name.startswith("pick_plate_from_"):
                    location = bt_name[len("pick_plate_from_"):]
                    if location in ("fridge", "microwave", "table"):
                        self._pref_session.record_color(location)

            # Save the latest state in case we want to resume execution
            # after a crash. skill_plan_names[i] is this skill's BT filename
            # (or HLA name fallback) -- the same string used for checkpoint names.
            # resume_prep_step advances to the next prep step once this command's
            # LAST sub-skill is done (the macro is complete); otherwise it stays
            # on this step so resume re-issues it and the planner finishes it.
            resume_prep_step = None
            if prep_step is not None:
                macro_complete = (i == len(plan_hlas) - 1)
                resume_prep_step = prep_step + 1 if macro_complete else prep_step
            self._save_state(
                sim_state, self.current_atoms,
                phase=phase, completed_skill=skill_plan_names[i],
                resume_prep_step=resume_prep_step,
            )

    def make_video(self, outfile: Path) -> None:
        """Create a video of the simulated trajectory."""
        self.sim.make_simulation_video(outfile)
        print(f"Saved video to {outfile}")

    def _save_state(
        self,
        sim_state: FeedingDeploymentWorldState,
        atoms: set[GroundAtom],
        *,
        phase: str,
        completed_skill: str,
        resume_prep_step: int | None = None,
    ) -> None:
        """Checkpoint after a completed sub-HLA. The store writes last_state.p
        plus any numbered/named target for this skill. The preference snapshot
        rides along so resume continues without re-asking and the end-of-meal
        learning update stays honest.

        ``resume_prep_step`` is the prep pipeline index to (re)start from if this
        checkpoint is resumed (None for feeding/finish, where resume routes by
        phase and bypasses prep). See _run_meal_preparation."""
        core_payload = {
            "sim_state": sim_state,
            "atoms": atoms,
            "physical_profile": self.physical_profile_label,
            "day": self._day,
            "resume_prep_step": resume_prep_step,
            "preference_session": (
                self._pref_session.capture_state() if self._pref_session is not None else None
            ),
        }
        written = self._ckpt.save(core_payload, phase=phase, completed_skill=completed_skill)
        print(f"Saved system state -> {', '.join(p.name for p in written)}")

    def _load_from_state(self) -> None:
        payload = self._ckpt.load(self._resume_state_name)
        self.current_atoms = payload["atoms"]
        self._resume_phase = payload["phase"]
        self._resume_physical_profile = payload["physical_profile"]
        self._resume_day = payload["day"]
        # Prep pipeline index to (re)start from (None for feeding/finish, which
        # bypass prep on resume). See _run_meal_preparation.
        self._resume_prep_step = payload["resume_prep_step"]
        # Prefer the standalone preference snapshot (written on every correction)
        # over the sim checkpoint's embedded copy (only as fresh as the last
        # sub-skill), so the latest corrections are restored regardless of which
        # skill boundary the crash fell on. Fall back to the embedded copy.
        self._resume_pref_state = self._ckpt.load_pref() or payload["preference_session"]
        sim_state = payload["sim_state"]
        if sim_state is not None:
            assert isinstance(sim_state, FeedingDeploymentWorldState)
            self.sim.sync(sim_state)
            if self.rviz_interface is not None:
                self.rviz_interface.joint_state_update(sim_state.robot_joints)
                if sim_state.held_object:
                    self.rviz_interface.tool_update(True, sim_state.held_object, Pose((0, 0, 0), (0, 0, 0, 1)))

        print(
            f"Loaded system state '{self._resume_state_name}' "
            f"(phase={self._resume_phase}, after skill #{self._ckpt.index}: "
            f"{payload['completed_skill']})"
        )

    def process_user_update_request(self, request_text: str) -> str:
        """Validate and update behavior trees."""
        self.perception_interface.sync_rviz()
        available_hla_object_names = []
        for hla, obj_combo in self._all_ground_hlas:
            hla_name = hla.get_name()
            object_strs = [obj.name for obj in obj_combo]
            objects_str = ", ".join(object_strs)
            available_hla_object_name = f"hla_name={hla_name}, hla_object_names=({objects_str},)"
            available_hla_object_names.append(available_hla_object_name)
        requested_updates = interpret_user_update_request(request_text, self.llm, available_hla_object_names, self.run_behavior_tree_dir)
        if len(requested_updates) == 0:
            raise ValueError("No valid updates requested.")
        all_update_messages = []
        for update in requested_updates:
            assert isinstance(update, UserUpdateRequest)
            if update.hla_name not in self.hla_name_to_hla:
                print(f"BT UPDATE FAILED: Unknown HLA name {update.hla_name}")
                raise ValueError(f"BT UPDATE FAILED: Unknown HLA name {update.hla_name}")
            hla = self.hla_name_to_hla[update.hla_name]
            hla_object_list = []
            failed_object_name = None
            for obj_name in update.hla_object_names:
                if obj_name not in self.object_name_to_object:
                    failed_object_name = obj_name
                    break
                hla_object_list.append(self.object_name_to_object[obj_name])
            if failed_object_name is not None:
                print(f"BT UPDATE FAILED: Unknown object name {failed_object_name}")
                raise ValueError(f"BT UPDATE FAILED: Unknown object name {failed_object_name}")
            ground_hla = GroundHighLevelAction(hla, tuple(hla_object_list))            
            if isinstance(update, NodeModificationUserUpdateRequest):
                message = ground_hla.process_behavior_tree_parameter_update(update.node_name, update.parameter_name, update.new_value)
                print(message)
                all_update_messages.append((update, message))
            elif isinstance(update, NodeAdditionUserRequest):
                message = ground_hla.process_behavior_tree_node_addition(update.new_node_type, update.new_node_parameters,
                                                               update.anchor_node_name, update.before_or_after)
                print(message)
                all_update_messages.append((update, message))
            else:
                print("Not implemented")
                raise NotImplementedError
        # TODO query LLM to summarize all_update_messages
        all_update_str = ""
        for request, message in all_update_messages:
            all_update_str += f"\nRequest: {request}"
            all_update_str += f"\nResult: {message}"
        prompt = f"""A user requested the following change to a robot assisted feeding system:

"{request_text}"
                
Here is a log of changes that were requested to behavior trees and the results:

{all_update_str}

Write a VERY BRIEF summary of all the changes for a non-technical end user. Make sure not to use technical terms like "behavior tree".
"""
        summary = self.llm.sample_completions(prompt, imgs=None, temperature=0.0, seed=0)[0]
        print("SUMMARY:", summary)
        return summary

    def register_gesture_detector(self, gesture_fn_name: str, gesture_fn_text: str) -> bool:
        """Add the gesture function to this run's python file."""
        with open(self._gesture_detection_filepath, "r", encoding="utf-8") as f:
            gesture_file_text = f.read()
        assert f"def {gesture_fn_name}(" not in gesture_file_text
        gesture_file_text += "\n" + gesture_fn_text + "\n"
        with open(self._gesture_detection_filepath, "w", encoding="utf-8") as f:
            f.write(gesture_file_text)
        # Immediately add the new gesture to specific BT nodes.
        gesture_interaction_parameters = [
            "InitiateTransferInteraction",
            "TransferCompleteInteraction",
        ]
        for hla, objs in self._all_ground_hlas:
            try:
                bt_filepath = hla.behavior_tree_dir / hla.get_behavior_tree_filename(objs, {})
            except NotImplementedError:
                continue
            bt = load_behavior_tree(bt_filepath, hla)
            for node in bt.walk():
                if isinstance(node, ParameterizedActionBehaviorTreeNode):
                    for parameter_name in gesture_interaction_parameters:
                        parameter = node.get_parameter(parameter_name)
                        if parameter is None:
                            continue
                        assert parameter.is_user_editable
                        assert isinstance(parameter.space, EnumSpace)
                        current_choices = list(parameter.space.elements)
                        new_choices = current_choices + [gesture_fn_name]
                        new_parameter_space = EnumSpace(new_choices)
                        parameter.space = new_parameter_space
            save_behavior_tree(bt, bt_filepath, hla)
            
        print(f"Registered new gesture detection function: {gesture_fn_name}")    

    def load_synthesized_gestures(self) -> list[tuple[str, Callable]]:
        """Returns a list of function names and functions."""
        with open(self._gesture_detection_filepath, "r", encoding="utf-8") as f:
            gesture_file_text = f.read()
        synthesized_gesture_module = types.ModuleType('synthesized_gestures')
        exec(gesture_file_text, synthesized_gesture_module.__dict__)
        return inspect.getmembers(synthesized_gesture_module, inspect.isfunction)
    
    def get_plate_pose(self) -> Pose | None:
        skill = self.hla_name_to_hla["PickTool"]
        skill.move_to_joint_positions(self.sim.scene_description.retract_pos)
        skill.close_gripper()
        skill.move_to_joint_positions(self.sim.scene_description.plate_gaze_pos)
        self.perception_interface.perceive_plate_pickup_poses()
        skill.move_to_joint_positions(self.sim.scene_description.retract_pos)
        if self.perception_interface.last_plate_poses:
            plate_pose = self.perception_interface.last_plate_poses["plate_pose"]
            return plate_pose
        else:
            print("No plate pose detected.")
            return None
        
    def get_drink_pose(self) -> Pose | None:
        skill = self.hla_name_to_hla["PickTool"]
        skill.move_to_joint_positions(self.sim.scene_description.retract_pos)
        skill.close_gripper()
        skill.move_to_joint_positions(self.sim.scene_description.drink_gaze_pos)
        self.perception_interface.perceive_drink_pickup_poses()
        skill.move_to_joint_positions(self.sim.scene_description.retract_pos)
        if self.perception_interface.last_drink_poses:
            drink_pose = self.perception_interface.last_drink_poses["drink_pose"]
            return drink_pose
        else:
            print("No drink pose detected.")
            return None
    
    def get_multitask_personalization_state(self, user_request: str, occluded: bool = False,
                                            actively_detect_plate: bool = False,
                                            actively_detect_drink: bool = False) -> dict[str, Any]:
        """Get a sufficient state for multitask personalization."""
        mp_state = {"user_request": user_request}

        if actively_detect_plate:
            mp_state["plate_pose"] = self.get_plate_pose()

        if actively_detect_drink:
            mp_state["drink_pose"] = self.get_drink_pose()

        mp_state["robot_joints"] = self.perception_interface.get_robot_joints()

        if occluded:
            mp_state["occluded"] = True

        self.perception_interface.sync_rviz()

        return mp_state
    
    def update_scene_spec(self, scene_spec_updates: dict[str, Any]) -> None:
        """Update the scene spec with the given updates."""
        for key, value in scene_spec_updates.items():
            if hasattr(self.scene_description, key):
                setattr(self.scene_description, key, value)
            else:
                raise ValueError(f"Invalid scene spec update: {key}")
        print("Updated scene spec:", scene_spec_updates)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--scene_config", type=str, default="vention") # name of the scene config (rough head-plate-robot setup)
    parser.add_argument("--user", type=str, default="") # name of the user
    # parser.add_argument("--transfer_type", type=str, default="inside")
    parser.add_argument("--transfer_type", type=str, default="outside")
    parser.add_argument("--run_on_robot", action="store_true")
    parser.add_argument("--use_interface", action="store_true")
    parser.add_argument("--use_gui", action="store_true")
    parser.add_argument("--simulate_head_perception", action="store_true")
    parser.add_argument("--make_videos", action="store_true")
    parser.add_argument("--max_motion_planning_time", type=float, default=10.0)
    parser.add_argument("--resume_from_state", type=str, default="")
    parser.add_argument("--no_waits", action="store_true")
    parser.add_argument(
        "--pref_mode",
        type=str,
        choices=["none", "terminal", "interface"],
        default=None,
        help="Preference interaction mode. If omitted, defaults to 'interface' "
             "when --use_interface is set and 'none' otherwise. "
             "'none': no personalization. "
             "'terminal': predict + correct via terminal prompts (must be passed explicitly). "
             "'interface': predict + correct via web interface (requires frontend).",
    )
    parser.add_argument(
        "--pref_meal",
        type=str,
        default="",
        help="Meal label (preference_learning.config.MEALS). "
             "Required with --pref_mode=interface. With --pref_mode=terminal, "
             "context is collected interactively and this flag is ignored.",
    )
    parser.add_argument(
        "--physical_profile_file",
        type=str,
        default="",
        help="UTF-8 text file describing the user's physical capabilities. "
             "Required with --pref_mode=terminal or --pref_mode=interface.",
    )
    parser.add_argument("--pref_setting", type=str, default="Personal", help="Dining setting (must match preference_learning.config.SETTINGS).")
    parser.add_argument(
        "--pref_time_of_day", type=str, default="morning", help="Time of day (must match preference_learning.config.TIMES_OF_DAY).")
    parser.add_argument(
        "--day", type=int, required=True,
        help="Deployment day number (mandatory). The single source of truth for "
             "BOTH per-day release logging (log/<user>/day_<NN>/) and the "
             "preference-learning memory day (log/<user>/preference_learning/.../"
             "day_<NNNN>.json). Days must be run sequentially with no gaps; a "
             "fresh run re-runs the whole meal and OVERWRITES that day's memory, "
             "while --resume_from_state continues a crashed meal of the same day.",
    )
    args = parser.parse_args()

    # Resolve pref_mode when not passed explicitly: interface runs personalize via
    # the web interface; everything else defaults to no personalization. Terminal
    # mode must always be requested explicitly.
    if args.pref_mode is None:
        args.pref_mode = "interface" if args.use_interface else "none"

    if args.user == "":
        raise ValueError("Please provide a user name.")

    if args.run_on_robot or args.use_interface:
        if not ROSPY_IMPORTED:
            raise ModuleNotFoundError("Need ROS to run on robot or use interface")
        else:
            rospy.init_node("feeding_deployment", anonymous=True)

    physical_profile_label: str | None = None
    if args.pref_mode in ("terminal", "interface"):
        if args.physical_profile_file.strip():
            profile_path = Path(args.physical_profile_file.strip())
            if not profile_path.is_file():
                raise ValueError(f"physical profile file not found: {profile_path}")
            physical_profile_label = profile_path.read_text(encoding="utf-8").strip()
            if not physical_profile_label:
                raise ValueError(f"physical profile file is empty: {profile_path}")
        else:
            # No file passed: fall back to the default deployment profile.
            physical_profile_label = DEFAULT_PHYSICAL_PROFILE

    runner = _Runner(args.scene_config,
                     args.user,
                     args.transfer_type,
                     args.run_on_robot,
                     args.use_interface,
                     args.use_gui,
                     args.simulate_head_perception,
                     args.max_motion_planning_time,
                     args.resume_from_state,
                     args.no_waits,
                     physical_profile_label=physical_profile_label,
                     pref_mode=args.pref_mode,
                     day=args.day)

    if args.pref_mode == "interface" and args.pref_meal.strip():
        # Optional preset (debug/replay): if --pref_meal is given, skip the web
        # context page and use it directly. Otherwise context is collected from
        # the preference_context page at the start of the meal.
        runner.set_meal_preference_context(
            meal=args.pref_meal.strip(),
            setting=args.pref_setting,
            time_of_day=args.pref_time_of_day,
        )
        print("Meal preference context (this run only):", runner.preference_context)

    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, runner.signal_handler)

    # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateInSink"], (runner.plate, runner.sink)))
    # for i in range(5):
    #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateOnTable"], (runner.plate, runner.table)))
    #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateOnHolder"], (runner.plate, runner.holder)))

    # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PickPlateFromAppliance"], (runner.plate, runner.fridge)))
    # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["OpenDoor"], (runner.fridge,)))
    # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["CloseDoor"], (runner.fridge,)))

    if not args.use_interface:
        # for i in range(3):
        #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PickPlateFromHolder"], (runner.plate, runner.holder)))
            # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateOnHolder"], (runner.plate, runner.holder)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["OpenDoor"], (runner.microwave,)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PickPlateFromAppliance"], (runner.plate, runner.fridge)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateInAppliance"], (runner.plate, runner.microwave)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["CloseDoor"], (runner.fridge,)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PickPlateFromAppliance"], (runner.plate, runner.microwave)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["OpenDoor"], (runner.fridge,)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["TransferTool"], (runner.utensil,runner.table)))
        
        for i in range(3):
            input("Press Enter to execute open and close the microwave door ...")
            runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["OpenDoor"], (runner.microwave,)))
            runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["CloseDoor"], (runner.microwave,)))
        # for i in range(10):
        #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["Reset"], ()))
        #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["Home"], ()))
        #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PickTool"], (runner.utensil,runner.table)))
        #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["StowTool"], (runner.utensil,runner.table)))
        # runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateInSink"], (runner.plate, runner.sink)))
        # for i in range(3):
        #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateOnTable"], (runner.plate, runner.table)))
        #     runner.process_user_command(GroundHighLevelAction(runner.hla_name_to_hla["PlacePlateOnHolder"], (runner.plate, runner.holder)))
    else:
        runner.run()

    if args.make_videos:
        output_path = Path(__file__).parent / "videos" / "full.mp4"
        runner.make_video(output_path)

    if args.run_on_robot:
        rospy.spin()