"""Translate a ground-truth preference bundle into BT YAML parameter writes
and scene-level configuration changes.

Every bundle field that maps to a BT parameter is declared in _BT_MAPPING.
apply_bundle_to_behavior_trees() iterates that mapping, loads the affected
YAML files, overwrites `value` entries, and saves them back to disk so that
subsequent execute_action() calls pick up the new values (BT YAMLs are
loaded fresh on each tick).
"""

from __future__ import annotations

import os
import tempfile
import warnings as _warnings
from pathlib import Path
from typing import Any

import yaml

from feeding_deployment.preference_learning.config.preference_bundle import (
    DEFAULT_BITE_ORDERING,
)

# ---------------------------------------------------------------------------
# Value translators: bundle option string  →  BT parameter value
# ---------------------------------------------------------------------------

_SPEED_MAP = {"slow": "low", "medium": "medium", "fast": "high"}

# Confirmation-page mode shared by the three confirm_* dims: 0 = skip the page,
# 1 = show it but proceed automatically after the auto-continue wait, 2 = block
# until the user answers.
_CONFIRM_MODE_MAP = {
    "no": 0,
    "yes (with auto-continue countdown)": 1,
    "yes (without any auto-continue)": 2,
}

_RETRACT_MAP = {"yes": 1, "no": 0}

# "no autocontinue" -> 0.0: the web pages treat a non-positive countdown as
# "show the page and wait indefinitely". Only the two feeding wait dims offer
# it; the mealprep dim's options never contain the key.
_AUTOCONTINUE_MAP = {"15 sec": 15.0, "30 sec": 30.0, "60 sec": 60.0, "no autocontinue": 0.0}

_OUTSIDE_MOUTH_DISTANCE_MAP = {"near": 0.07, "medium": 0.1, "far": 0.13}

_CONVEY_READY_MAP = {
    "no cue": "silent",
    "speech": "voice",
    "LED": "led",
    "speech + LED": "voice_led",
}

_INITIATE_TRANSFER_MAP = {
    "open mouth": "open_mouth",
    "button": "button",
    "autocontinue": "auto_timeout",
}

_COMPLETE_TRANSFER_MAP = {
    "perception": "sense",
    "button": "button",
    "autocontinue": "auto_timeout",
}

_TRANSFER_MODE_MAP = {
    "inside mouth transfer": "inside",
    "outside mouth transfer": "outside",
}

_SKEWERING_AXIS_MAP = {
    "parallel to major axis": "horizontal",
    "perpendicular to major axis": "vertical",
}


def _dipping_depth_translate(val: str) -> float | None:
    # NOTE: "do not dip" only skips the FoodDippingDepth BT write — it does NOT
    # prevent FLAIR's autonomous planner from choosing to dip. The dipping
    # decision is made independently in inference_class.py
    # (get_autonomous_action), which looks at plate contents and user
    # preferences. To truly suppress dipping when the user says "do not dip",
    # a flag must be passed into FLAIR's planning logic so that
    # get_autonomous_action never selects a dip action. This is separate from
    # the BT parameter layer and is not yet implemented.
    if val == "do not dip":
        return None
    return {"less": 0.01, "more": 0.03}[val]

# ---------------------------------------------------------------------------
# Declarative mapping:  bundle field  →  list of (yaml_filename, bt_param_name, translator)
#
# "translator" is either a dict (direct lookup) or a callable (value → bt_value).
# A translator may return None to signal "skip this write" (e.g. outside_mouth_distance
# when the bundle value is "not applicable").
# ---------------------------------------------------------------------------

_ALL_BT_YAMLS: list[str] = [
    "acquire_bite.yaml",
    "close_fridge.yaml",
    "close_microwave.yaml",
    "emulate_transfer.yaml",
    "gaze_at_table.yaml",
    "navigate_to_fridge.yaml",
    "navigate_to_microwave.yaml",
    "navigate_to_sink.yaml",
    "navigate_to_table.yaml",
    "open_fridge.yaml",
    "open_microwave.yaml",
    "pick_drink.yaml",
    "pick_plate_from_fridge.yaml",
    "pick_plate_from_holder.yaml",
    "pick_plate_from_microwave.yaml",
    "pick_plate_from_table.yaml",
    "pick_utensil.yaml",
    "pick_wipe.yaml",
    "place_plate_in_fridge.yaml",
    "place_plate_in_microwave.yaml",
    "place_plate_in_sink.yaml",
    "place_plate_on_holder.yaml",
    "place_plate_on_table.yaml",
    "press_microwave_button.yaml",
    "stow_drink.yaml",
    "stow_utensil.yaml",
    "stow_wipe.yaml",
    "transfer_drink.yaml",
    "transfer_utensil.yaml",
    "transfer_wipe.yaml",
]

_TRANSFER_YAMLS = ["transfer_utensil.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"]


def _outside_mouth_translate(val: str) -> float | None:
    if val == "not applicable":
        return None
    return _OUTSIDE_MOUTH_DISTANCE_MAP[val]


def _microwave_duration_translate(val: str) -> float | None:
    """Translate microwave_time bundle value to BT MicrowaveDuration seconds.

    "no microwave" returns None (skip the BT write — the planner already
    excludes the PressMicrowaveButton HLA via the FoodHeated atom).
    """
    if val == "no microwave":
        return None
    return {"1 min": 60.0, "2 min": 120.0, "3 min": 180.0}[val]


# Each entry: (bundle_field, yaml_files, bt_param_name, translator)
_BT_MAPPING: list[tuple[str, list[str], str, dict | Any]] = [
    # Speed — all 29 YAMLs
    ("robot_speed", _ALL_BT_YAMLS, "Speed", _SPEED_MAP),

    # Feeding pickup confirmation (bite / drink / wipe verification pages) —
    # one parameter name across all three skills.
    ("confirm_feeding_pickup",
     ["acquire_bite.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"],
     "PickupConfirmMode", _CONFIRM_MODE_MAP),

    # Navigation arrival confirmation (post-arrival position check/adjust page)
    ("confirm_navigation_arrival",
     ["navigate_to_fridge.yaml", "navigate_to_microwave.yaml",
      "navigate_to_sink.yaml", "navigate_to_table.yaml"],
     "ArrivalConfirmMode", _CONFIRM_MODE_MAP),

    # Manipulation confirmation (detection confirms at pickups / door handles /
    # microwave button / table+sink placement + plate-release confirms).
    # pick_plate_from_holder is excluded: the holder pose is known, no
    # detection page is shown there. The table-placement detection page fires
    # in gaze_at_table (not place_plate_on_table), hence its YAML here too;
    # door closing uses cached poses (no page), so close_* are excluded.
    ("confirm_manipulation",
     ["pick_plate_from_fridge.yaml", "pick_plate_from_microwave.yaml",
      "pick_plate_from_table.yaml", "place_plate_in_microwave.yaml",
      "place_plate_on_table.yaml", "place_plate_in_sink.yaml",
      "press_microwave_button.yaml", "gaze_at_table.yaml",
      "open_fridge.yaml", "open_microwave.yaml"],
     "ManipulationConfirmMode", _CONFIRM_MODE_MAP),

    # Autocontinue wait times. Every autocontinue duration is a BT parameter,
    # split across three dims:
    #   - task_selection: the next-task pages after a bite/sip
    #     (TaskReselectionAutocontinueSeconds in transfer_utensil,
    #     transfer_drink).
    #   - feeding_pickup: the bite-selection page
    #     (BiteSelectionAutocontinueSeconds) and the bite/drink/wipe
    #     pickup-confirmation pages (PickupConfirmAutocontinueSeconds, one
    #     name across all three pickup skills) -- both written with the same
    #     value.
    #   - mealprep: the detection/plate-release confirmation pages
    #     (ManipulationConfirmAutocontinueSeconds) and the post-arrival
    #     position check (ArrivalConfirmAutocontinueSeconds) -- both written
    #     with the same value.
    ("wait_before_autocontinue_task_selection",
     ["transfer_utensil.yaml", "transfer_drink.yaml"],
     "TaskReselectionAutocontinueSeconds", _AUTOCONTINUE_MAP),
    ("wait_before_autocontinue_feeding_pickup",
     ["acquire_bite.yaml"],
     "BiteSelectionAutocontinueSeconds", _AUTOCONTINUE_MAP),
    ("wait_before_autocontinue_feeding_pickup",
     ["acquire_bite.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"],
     "PickupConfirmAutocontinueSeconds", _AUTOCONTINUE_MAP),
    ("wait_before_autocontinue_mealprep",
     ["pick_plate_from_fridge.yaml", "pick_plate_from_microwave.yaml",
      "pick_plate_from_table.yaml", "place_plate_in_microwave.yaml",
      "place_plate_on_table.yaml", "place_plate_in_sink.yaml",
      "press_microwave_button.yaml", "gaze_at_table.yaml",
      "open_fridge.yaml", "open_microwave.yaml"],
     "ManipulationConfirmAutocontinueSeconds", _AUTOCONTINUE_MAP),
    ("wait_before_autocontinue_mealprep",
     ["navigate_to_fridge.yaml", "navigate_to_microwave.yaml",
      "navigate_to_sink.yaml", "navigate_to_table.yaml"],
     "ArrivalConfirmAutocontinueSeconds", _AUTOCONTINUE_MAP),

    # Outside-mouth distance
    ("outside_mouth_distance", _TRANSFER_YAMLS, "OutsideMouthDistance", _outside_mouth_translate),

    # Convey robot ready for initiating transfer
    ("convey_robot_ready_for_initiating_transfer", _TRANSFER_YAMLS,
     "RobotTransferStartCue", _CONVEY_READY_MAP),

    # Detect user ready (per-tool)
    ("detect_user_ready_for_initiating_transfer_feeding",
     ["transfer_utensil.yaml"], "UserTransferReadySignal", _INITIATE_TRANSFER_MAP),
    ("detect_user_ready_for_initiating_transfer_drinking",
     ["transfer_drink.yaml"], "UserTransferReadySignal", _INITIATE_TRANSFER_MAP),
    ("detect_user_ready_for_initiating_transfer_wiping",
     ["transfer_wipe.yaml"], "UserTransferReadySignal", _INITIATE_TRANSFER_MAP),

    # Convey robot ready for completing transfer
    ("convey_robot_ready_for_completing_transfer", _TRANSFER_YAMLS,
     "RobotTransferArrivedCue", _CONVEY_READY_MAP),

    # Detect user completed transfer (per-tool)
    ("detect_user_completed_transfer_feeding",
     ["transfer_utensil.yaml"], "UserTransferDoneSignal", _COMPLETE_TRANSFER_MAP),
    ("detect_user_completed_transfer_drinking",
     ["transfer_drink.yaml"], "UserTransferDoneSignal", _COMPLETE_TRANSFER_MAP),
    ("detect_user_completed_transfer_wiping",
     ["transfer_wipe.yaml"], "UserTransferDoneSignal", _COMPLETE_TRANSFER_MAP),

    # Skewering axis
    ("skewering_axis", ["acquire_bite.yaml"], "SkeweringOrientation", _SKEWERING_AXIS_MAP),

    # Bite dipping preference → FoodDippingDepth (see _dipping_depth_translate for "do not dip" caveat)
    ("bite_dipping_preference", ["acquire_bite.yaml"], "FoodDippingDepth", _dipping_depth_translate),

    # Microwave duration
    ("microwave_time", ["press_microwave_button.yaml"], "MicrowaveDuration", _microwave_duration_translate),

    # Retract between bites (utensil only — drink/wipe don't have repeated transfer loops)
    ("retract_between_bites", ["transfer_utensil.yaml"], "RetractAfterTransfer", _RETRACT_MAP),
]


# ---------------------------------------------------------------------------
# YAML helpers
# ---------------------------------------------------------------------------

class _HlaTag:
    """Placeholder for the !hla YAML tag so we can round-trip without losing it."""

    def __init__(self, value: str) -> None:
        self.value = value

    def __repr__(self) -> str:
        return f"!hla {self.value}"


def _hla_constructor(loader: yaml.SafeLoader, node: yaml.Node) -> _HlaTag:
    return _HlaTag(loader.construct_scalar(node))


def _hla_representer(dumper: yaml.Dumper, tag: _HlaTag) -> yaml.Node:
    return dumper.represent_scalar("!hla", tag.value)


_Loader = type("_Loader", (yaml.SafeLoader,), {})
_Loader.add_constructor("!hla", _hla_constructor)

_Dumper = type("_Dumper", (yaml.Dumper,), {})
_Dumper.add_representer(_HlaTag, _hla_representer)


def _load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.load(f, Loader=_Loader)


def _save_yaml(path: Path, data: dict) -> None:
    # Atomic write: dump to a temp file in the same directory, then os.replace onto
    # the target (atomic on POSIX, same filesystem). The BT executor reads these
    # YAMLs fresh on each skill (load_behavior_tree), possibly from a *different*
    # thread than the writer (the settings-edit apply-worker). A plain truncate-then-
    # write could hand a concurrent reader a partial file -> YAML parse error ->
    # FatalSkillFailure. With os.replace, a reader always sees a complete old-or-new
    # file. Single-threaded callers are unaffected.
    text = yaml.dump(data, Dumper=_Dumper, sort_keys=False, default_flow_style=False)
    path = Path(path)
    fd, tmp_name = tempfile.mkstemp(dir=str(path.parent), prefix=path.name + ".", suffix=".tmp")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
        os.replace(tmp_name, path)
    except BaseException:
        # Best-effort cleanup; never leave a stray temp file on failure.
        try:
            os.unlink(tmp_name)
        except OSError:
            pass
        raise


def _set_param_value(bt_data: dict, param_name: str, new_value: Any) -> bool:
    """Find a parameter by name in the YAML dict and set its value.

    Returns True if the parameter was found and updated.
    """
    for param in bt_data.get("parameters", []):
        if param["name"] == param_name:
            param["value"] = new_value
            return True
    return False


# Canonical blocks for the confirmation-mode BT parameters. Used by
# _upsert_confirm_param to migrate pre-existing per-user BT trees on the first
# apply: (a) a tree that predates the parameter gets it APPENDED — safe only
# because the factory YAMLs and the skill function signatures also put these
# parameters LAST (positional binding), and the skills default the argument
# when an old YAML doesn't pass it; (b) a tree whose feeding confirmation
# params still carry the legacy Enum [0, 1] space gets it widened to [0, 1, 2]
# so mode 2 passes space.contains() at execution.
_CONFIRM_MODE_SPACE = {"type": "Enum", "elements": [0, 1, 2]}

_CONFIRM_PARAM_BLOCKS: dict[str, dict] = {
    "PickupConfirmMode": {
        "name": "PickupConfirmMode",
        "description": "Bite/drink/wipe pickup confirmation page mode: 0 = skip, 1 = show with autocontinue, 2 = wait for the user.",
        "space": dict(_CONFIRM_MODE_SPACE),
        "is_user_editable": True,
        "value": 2,
    },
    "ArrivalConfirmMode": {
        "name": "ArrivalConfirmMode",
        "description": "Post-arrival position check/adjust page mode: 0 = skip, 1 = show with autocontinue, 2 = wait for the user.",
        "space": dict(_CONFIRM_MODE_SPACE),
        "is_user_editable": True,
        "value": 1,
    },
    "ManipulationConfirmMode": {
        "name": "ManipulationConfirmMode",
        "description": "Manipulation confirmation page mode (detection confirm / plate release): 0 = skip, 1 = show with autocontinue, 2 = wait for the user.",
        "space": dict(_CONFIRM_MODE_SPACE),
        "is_user_editable": True,
        "value": 2,
    },
}


def _upsert_confirm_param(bt_data: dict, param_name: str, new_value: Any) -> None:
    """Set a confirmation-mode parameter, migrating the YAML if needed (see
    _CONFIRM_PARAM_BLOCKS). Always leaves the param present with the [0,1,2]
    space and the given value."""
    for param in bt_data.get("parameters", []):
        if param["name"] == param_name:
            param["value"] = new_value
            param["space"] = dict(_CONFIRM_MODE_SPACE)
            return
    block = dict(_CONFIRM_PARAM_BLOCKS[param_name])
    block["space"] = dict(_CONFIRM_MODE_SPACE)
    block["value"] = new_value
    bt_data.setdefault("parameters", []).append(block)


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def apply_bundle_to_behavior_trees(
    bundle: dict[str, str],
    bt_dir: Path,
) -> list[str]:
    """Write preference-bundle values into the BT YAML files on disk.

    Returns a list of warning strings for edge cases (e.g. unsupported
    combined cues).
    """
    warnings: list[str] = []
    # Cache: yaml_filename -> loaded dict (load each file at most once)
    loaded: dict[str, dict] = {}
    dirty: set[str] = set()

    for bundle_field, yaml_files, bt_param, translator in _BT_MAPPING:
        bundle_val = bundle.get(bundle_field)
        if bundle_val is None:
            continue

        # Translate
        if callable(translator) and not isinstance(translator, dict):
            bt_val = translator(bundle_val)
        else:
            bt_val = translator.get(bundle_val)
            if bt_val is None:
                warnings.append(
                    f"No mapping for {bundle_field}={bundle_val!r}; skipping BT write."
                )
                continue

        if bt_val is None:
            # Explicit skip (e.g. outside_mouth_distance="not applicable")
            continue

        for fname in yaml_files:
            fpath = bt_dir / fname
            if not fpath.exists():
                warnings.append(f"BT YAML not found: {fpath}")
                continue
            if fname not in loaded:
                loaded[fname] = _load_yaml(fpath)
            if bt_param in _CONFIRM_PARAM_BLOCKS:
                # Confirmation-mode params upsert + space-migrate (older
                # per-user trees lack them or carry the legacy [0,1] space).
                _upsert_confirm_param(loaded[fname], bt_param, bt_val)
                dirty.add(fname)
            elif _set_param_value(loaded[fname], bt_param, bt_val):
                dirty.add(fname)
            else:
                warnings.append(
                    f"Parameter {bt_param!r} not found in {fname}; skipping."
                )

    for fname in dirty:
        _save_yaml(bt_dir / fname, loaded[fname])

    return warnings


def apply_transfer_mode(bundle: dict[str, str]) -> None:
    """Validate the transfer_mode dim WITHOUT applying anything.

    The robot's actual transfer type is fixed by the command line
    (scene_description.transfer_type) and must never be edited by code, and
    TransferToolHLA's inside/outside transfer object must never be swapped at
    runtime. This deployment only performs outside-mouth transfer: the dim is
    still asked/learned, but an 'inside mouth transfer' value reaching the
    apply path (predicted or user-selected) fails loudly rather than silently
    diverging from what the robot actually does.
    """
    mode = bundle.get("transfer_mode")
    if mode is None:
        return

    if mode not in _TRANSFER_MODE_MAP:
        raise ValueError(
            f"Unknown transfer_mode={mode!r}. "
            f"Expected one of {list(_TRANSFER_MODE_MAP.keys())}."
        )
    if mode == "inside mouth transfer":
        raise RuntimeError(
            "inside mouth transfer is not supported in this deployment; "
            "transfer_type is fixed by the command line."
        )


_MICROWAVE_TIME_MAP = {
    "no microwave": None,
    "1 min": 60,
    "2 min": 120,
    "3 min": 180,
}


def apply_microwave_preference(
    bundle: dict[str, str],
    current_atoms: set,
    food_heated_atom: Any,
) -> int | None:
    """Adjust planner atoms based on the microwave_time preference.

    If "no microwave", adds the FoodHeated atom to current_atoms so the PDDL
    planner skips the entire microwave sequence (navigate to microwave, open
    door, place plate, press button, pick plate, close door).

    For "1 min"/"2 min"/"3 min", ensures FoodHeated is absent (the planner
    will include microwave steps).

    Returns the microwave duration in seconds, or None for "no microwave".
    The caller can use this value to set the actual microwave timer when the
    PressMicrowaveButton HLA executes — this wiring is not yet implemented.
    """
    microwave_time = bundle.get("microwave_time")
    if microwave_time is None:
        return None

    duration = _MICROWAVE_TIME_MAP.get(microwave_time)
    if duration is None and microwave_time != "no microwave":
        raise ValueError(
            f"Unknown microwave_time={microwave_time!r}. "
            f"Expected one of {list(_MICROWAVE_TIME_MAP.keys())}."
        )

    if microwave_time == "no microwave":
        current_atoms.add(food_heated_atom)
    else:
        current_atoms.discard(food_heated_atom)

    return duration


def apply_dip_preference(
    bundle: dict[str, str],
    flair: Any,
) -> None:
    """Set FLAIR's allow_dip flag based on bite_dipping_preference.

    When "do not dip", sets allow_dip=False so get_autonomous_action
    deterministically strips any dip the LLM suggests.
    """
    dip_pref = bundle.get("bite_dipping_preference")
    if dip_pref is None or flair is None:
        return
    flair.set_allow_dip(dip_pref != "do not dip")


def apply_bite_ordering(
    bundle: dict[str, str],
    flair: Any,
) -> None:
    """Push the predicted/corrected bite_ordering text into FLAIR as the user's
    bite-ordering preference (consumed by FLAIR's preference planner).

    Always sets a non-empty string (falls back to DEFAULT_BITE_ORDERING) so
    flair.is_preference_set() stays true and the (removed) meal_setup branch is
    never re-entered.
    """
    if flair is None:
        return
    ordering = bundle.get("bite_ordering") or DEFAULT_BITE_ORDERING
    flair.set_preference(ordering)
