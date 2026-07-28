"""Tests for Step 4: apply_preferences module (BT YAML writes + transfer mode).

Run with:
    PYTHONPATH=src python -m pytest tests/test_apply_preferences.py -v
"""

from __future__ import annotations

import shutil
from pathlib import Path
import pytest
import yaml

from feeding_deployment.integration.apply_preferences import (
    apply_bundle_to_behavior_trees,
    apply_transfer_mode,
    apply_microwave_preference,
    apply_dip_preference,
    _SPEED_MAP,
    _CONFIRM_MAP,
    _COUNTDOWN_MAP,
    _OUTSIDE_MOUTH_DISTANCE_MAP,
    _CONVEY_READY_MAP,
    _INITIATE_TRANSFER_MAP,
    _COMPLETE_TRANSFER_MAP,
    _TRANSFER_MODE_MAP,
    _SKEWERING_AXIS_MAP,
    _MICROWAVE_TIME_MAP,
    _RETRACT_MAP,
    _dipping_depth_translate,
    _microwave_duration_translate,
    _load_yaml,
)
from feeding_deployment.preference_learning.methods.utils import PREF_FIELDS

BT_SOURCE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "feeding_deployment"
    / "actions"
    / "behavior_trees"
)


@pytest.fixture()
def bt_dir(tmp_path: Path) -> Path:
    """Copy all source BT YAMLs into a temp directory for isolated testing."""
    dest = tmp_path / "behavior_trees"
    shutil.copytree(BT_SOURCE, dest)
    return dest


def _load(bt_dir: Path, fname: str) -> dict:
    return _load_yaml(bt_dir / fname)


def _get_param_value(data: dict, param_name: str):
    for p in data.get("parameters", []):
        if p["name"] == param_name:
            return p["value"]
    raise KeyError(f"parameter {param_name!r} not found")


# -------------------------------------------------------------------
# Value translator unit tests
# -------------------------------------------------------------------


class TestValueTranslators:

    def test_speed_map_covers_all_bundle_options(self):
        assert set(_SPEED_MAP.keys()) == {"slow", "medium", "fast"}

    def test_confirm_map(self):
        # Single sentinel-encoded float per confirm page.
        assert _CONFIRM_MAP == {
            "skip": -1.0,
            "countdown (15 sec)": 15.0,
            "countdown (30 sec)": 30.0,
            "countdown (60 sec)": 60.0,
            "wait for me": 0.0,
        }

    def test_countdown_map(self):
        # Feeding-page countdowns (no skip): wait for me -> 0.0.
        assert _COUNTDOWN_MAP == {
            "countdown (15 sec)": 15.0,
            "countdown (30 sec)": 30.0,
            "countdown (60 sec)": 60.0,
            "wait for me": 0.0,
        }
        for v in _COUNTDOWN_MAP.values():
            assert isinstance(v, float)

    def test_outside_mouth_distance_values_in_range(self):
        for v in _OUTSIDE_MOUTH_DISTANCE_MAP.values():
            assert 0.05 <= v <= 0.15

    def test_convey_ready_maps_speech_plus_led_to_voice_led(self):
        assert _CONVEY_READY_MAP["speech + LED"] == "voice_led"

    def test_initiate_transfer_map(self):
        assert _INITIATE_TRANSFER_MAP["open mouth"] == "open_mouth"
        assert _INITIATE_TRANSFER_MAP["proceed automatically after a pause"] == "auto_timeout"

    def test_complete_transfer_map(self):
        assert _COMPLETE_TRANSFER_MAP["perception"] == "sense"
        assert _COMPLETE_TRANSFER_MAP["proceed automatically after a pause"] == "auto_timeout"

    def test_skewering_axis_map(self):
        assert _SKEWERING_AXIS_MAP["parallel to major axis"] == "horizontal"
        assert _SKEWERING_AXIS_MAP["perpendicular to major axis"] == "vertical"

    def test_dipping_depth_translate(self):
        assert _dipping_depth_translate("less") == 0.01
        assert _dipping_depth_translate("more") == 0.03
        assert _dipping_depth_translate("do not dip") is None

    def test_microwave_duration_translate(self):
        assert _microwave_duration_translate("30 secs") == 30.0
        assert _microwave_duration_translate("1 min") == 60.0
        assert _microwave_duration_translate("2 min") == 120.0
        assert _microwave_duration_translate("no microwave") is None

    def test_retract_map(self):
        assert _RETRACT_MAP["yes"] == 1
        assert _RETRACT_MAP["no"] == 0


# -------------------------------------------------------------------
# apply_bundle_to_behavior_trees: YAML round-trip
# -------------------------------------------------------------------


class TestApplyBundleToBehaviorTrees:

    def test_speed_written_to_all_yamls(self, bt_dir: Path):
        bundle = {"robot_speed": "fast"}
        warnings = apply_bundle_to_behavior_trees(bundle, bt_dir)

        for fname in bt_dir.glob("*.yaml"):
            data = _load(bt_dir, fname.name)
            assert _get_param_value(data, "Speed") == "high", (
                f"Speed not updated in {fname.name}"
            )
        assert not warnings

    def test_speed_slow_maps_to_low(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"robot_speed": "slow"}, bt_dir)
        data = _load(bt_dir, "acquire_bite.yaml")
        assert _get_param_value(data, "Speed") == "low"

    # The single sentinel-encoded confirm param per page (skip=-1, wait=0,
    # countdown=N) written by each confirm_* dim.
    _MANIPULATION_CONFIRM_YAMLS = [
        "pick_plate_from_fridge.yaml", "pick_plate_from_microwave.yaml",
        "pick_plate_from_table.yaml", "place_plate_in_microwave.yaml",
        "place_plate_on_table.yaml", "place_plate_in_sink.yaml",
        "press_microwave_button.yaml", "gaze_at_table.yaml",
        "open_fridge.yaml", "open_microwave.yaml",
    ]
    _NAV_CONFIRM_YAMLS = [
        "navigate_to_fridge.yaml", "navigate_to_microwave.yaml",
        "navigate_to_sink.yaml", "navigate_to_table.yaml",
    ]

    def test_feeding_confirmation_skip(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"confirm_feeding_pickup": "skip"}, bt_dir)
        for fname in ("acquire_bite.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"):
            assert _get_param_value(_load(bt_dir, fname), "PickupConfirm") == -1.0

    def test_feeding_confirmation_countdown(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"confirm_feeding_pickup": "countdown (30 sec)"}, bt_dir)
        for fname in ("acquire_bite.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"):
            assert _get_param_value(_load(bt_dir, fname), "PickupConfirm") == 30.0

    def test_navigation_confirmation_wait(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"confirm_navigation_arrival": "wait for me"}, bt_dir)
        for fname in self._NAV_CONFIRM_YAMLS:
            assert _get_param_value(_load(bt_dir, fname), "ArrivalConfirm") == 0.0

    def test_manipulation_confirmation_countdown(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"confirm_manipulation": "countdown (15 sec)"}, bt_dir)
        for fname in self._MANIPULATION_CONFIRM_YAMLS:
            assert _get_param_value(_load(bt_dir, fname), "ManipulationConfirm") == 15.0

    def test_bite_selection_countdown_written(self, bt_dir: Path):
        warnings = apply_bundle_to_behavior_trees(
            {"wait_before_autocontinue_bite_selection": "countdown (30 sec)"}, bt_dir)
        assert warnings == []
        assert _get_param_value(_load(bt_dir, "acquire_bite.yaml"), "BiteSelectionAutocontinueSeconds") == 30.0
        # Does not touch the next-task or confirm params.
        assert _get_param_value(_load(bt_dir, "transfer_drink.yaml"), "TaskReselectionAutocontinueSeconds") != 30.0

    def test_task_selection_countdown_written(self, bt_dir: Path):
        warnings = apply_bundle_to_behavior_trees(
            {"wait_before_autocontinue_task_selection": "countdown (30 sec)"}, bt_dir)
        assert warnings == []
        for fname in ["transfer_utensil.yaml", "transfer_drink.yaml"]:
            assert _get_param_value(_load(bt_dir, fname), "TaskReselectionAutocontinueSeconds") == 30.0
        assert _get_param_value(_load(bt_dir, "acquire_bite.yaml"), "BiteSelectionAutocontinueSeconds") != 30.0

    def test_feeding_page_wait_writes_zero(self, bt_dir: Path):
        bundle = {
            "wait_before_autocontinue_task_selection": "wait for me",
            "wait_before_autocontinue_bite_selection": "wait for me",
        }
        warnings = apply_bundle_to_behavior_trees(bundle, bt_dir)
        assert warnings == []
        for fname in ["transfer_utensil.yaml", "transfer_drink.yaml"]:
            assert _get_param_value(_load(bt_dir, fname), "TaskReselectionAutocontinueSeconds") == 0.0
        assert _get_param_value(_load(bt_dir, "acquire_bite.yaml"), "BiteSelectionAutocontinueSeconds") == 0.0

    def test_outside_mouth_distance_near(self, bt_dir: Path):
        bundle = {"outside_mouth_distance": "near"}
        apply_bundle_to_behavior_trees(bundle, bt_dir)

        for fname in ["transfer_utensil.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"]:
            data = _load(bt_dir, fname)
            assert _get_param_value(data, "OutsideMouthDistance") == 0.07

    def test_outside_mouth_distance_not_applicable_skips(self, bt_dir: Path):
        original = _get_param_value(_load(bt_dir, "transfer_utensil.yaml"), "OutsideMouthDistance")
        bundle = {"outside_mouth_distance": "not applicable"}
        apply_bundle_to_behavior_trees(bundle, bt_dir)

        after = _get_param_value(_load(bt_dir, "transfer_utensil.yaml"), "OutsideMouthDistance")
        assert after == original

    def test_convey_ready_initiating(self, bt_dir: Path):
        bundle = {"convey_robot_ready_for_initiating_transfer": "LED"}
        apply_bundle_to_behavior_trees(bundle, bt_dir)

        for fname in ["transfer_utensil.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"]:
            data = _load(bt_dir, fname)
            assert _get_param_value(data, "RobotTransferStartCue") == "led"

    def test_convey_ready_completing(self, bt_dir: Path):
        bundle = {"convey_robot_ready_for_completing_transfer": "no cue"}
        apply_bundle_to_behavior_trees(bundle, bt_dir)

        for fname in ["transfer_utensil.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"]:
            data = _load(bt_dir, fname)
            assert _get_param_value(data, "RobotTransferArrivedCue") == "silent"

    def test_detect_initiate_per_tool(self, bt_dir: Path):
        bundle = {
            "detect_user_ready_for_initiating_transfer_feeding": "button",
            "detect_user_ready_for_initiating_transfer_drinking": "proceed automatically after a pause",
            "detect_user_ready_for_initiating_transfer_wiping": "open mouth",
        }
        apply_bundle_to_behavior_trees(bundle, bt_dir)

        assert _get_param_value(
            _load(bt_dir, "transfer_utensil.yaml"), "UserTransferReadySignal"
        ) == "button"
        assert _get_param_value(
            _load(bt_dir, "transfer_drink.yaml"), "UserTransferReadySignal"
        ) == "auto_timeout"
        assert _get_param_value(
            _load(bt_dir, "transfer_wipe.yaml"), "UserTransferReadySignal"
        ) == "open_mouth"

    def test_detect_complete_per_tool(self, bt_dir: Path):
        bundle = {
            "detect_user_completed_transfer_feeding": "perception",
            "detect_user_completed_transfer_drinking": "button",
            "detect_user_completed_transfer_wiping": "proceed automatically after a pause",
        }
        apply_bundle_to_behavior_trees(bundle, bt_dir)

        assert _get_param_value(
            _load(bt_dir, "transfer_utensil.yaml"), "UserTransferDoneSignal"
        ) == "sense"
        assert _get_param_value(
            _load(bt_dir, "transfer_drink.yaml"), "UserTransferDoneSignal"
        ) == "button"
        assert _get_param_value(
            _load(bt_dir, "transfer_wipe.yaml"), "UserTransferDoneSignal"
        ) == "auto_timeout"

    def test_skewering_axis_parallel(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"skewering_axis": "parallel to major axis"}, bt_dir)
        data = _load(bt_dir, "acquire_bite.yaml")
        assert _get_param_value(data, "SkeweringOrientation") == "horizontal"

    def test_skewering_axis_perpendicular(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"skewering_axis": "perpendicular to major axis"}, bt_dir)
        data = _load(bt_dir, "acquire_bite.yaml")
        assert _get_param_value(data, "SkeweringOrientation") == "vertical"

    def test_dipping_preference_more(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"bite_dipping_preference": "more"}, bt_dir)
        data = _load(bt_dir, "acquire_bite.yaml")
        assert _get_param_value(data, "FoodDippingDepth") == 0.03

    def test_dipping_preference_less(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"bite_dipping_preference": "less"}, bt_dir)
        data = _load(bt_dir, "acquire_bite.yaml")
        assert _get_param_value(data, "FoodDippingDepth") == 0.01

    def test_dipping_preference_do_not_dip_skips(self, bt_dir: Path):
        original = _get_param_value(_load(bt_dir, "acquire_bite.yaml"), "FoodDippingDepth")
        apply_bundle_to_behavior_trees({"bite_dipping_preference": "do not dip"}, bt_dir)
        after = _get_param_value(_load(bt_dir, "acquire_bite.yaml"), "FoodDippingDepth")
        assert after == original

    def test_microwave_duration_2_min(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"microwave_time": "2 min"}, bt_dir)
        data = _load(bt_dir, "press_microwave_button.yaml")
        assert _get_param_value(data, "MicrowaveDuration") == 120.0

    def test_microwave_duration_1_min(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"microwave_time": "1 min"}, bt_dir)
        data = _load(bt_dir, "press_microwave_button.yaml")
        assert _get_param_value(data, "MicrowaveDuration") == 60.0

    def test_microwave_duration_30_secs(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"microwave_time": "30 secs"}, bt_dir)
        data = _load(bt_dir, "press_microwave_button.yaml")
        assert _get_param_value(data, "MicrowaveDuration") == 30.0

    def test_microwave_no_microwave_skips(self, bt_dir: Path):
        original = _get_param_value(_load(bt_dir, "press_microwave_button.yaml"), "MicrowaveDuration")
        apply_bundle_to_behavior_trees({"microwave_time": "no microwave"}, bt_dir)
        after = _get_param_value(_load(bt_dir, "press_microwave_button.yaml"), "MicrowaveDuration")
        assert after == original

    def test_retract_between_bites_yes(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"retract_between_bites": "yes"}, bt_dir)
        data = _load(bt_dir, "transfer_utensil.yaml")
        assert _get_param_value(data, "RetractAfterTransfer") == 1

    def test_retract_between_bites_no(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"retract_between_bites": "no"}, bt_dir)
        data = _load(bt_dir, "transfer_utensil.yaml")
        assert _get_param_value(data, "RetractAfterTransfer") == 0

    def test_retract_only_affects_utensil(self, bt_dir: Path):
        apply_bundle_to_behavior_trees({"retract_between_bites": "yes"}, bt_dir)
        for fname in ["transfer_drink.yaml", "transfer_wipe.yaml"]:
            data = _load(bt_dir, fname)
            with pytest.raises(KeyError):
                _get_param_value(data, "RetractAfterTransfer")


class TestApplyBundleWarnings:

    def test_speech_plus_led_writes_voice_led(self, bt_dir: Path):
        bundle = {"convey_robot_ready_for_initiating_transfer": "speech + LED"}
        warnings = apply_bundle_to_behavior_trees(bundle, bt_dir)
        assert not any("speech + LED" in w for w in warnings)

        for fname in ["transfer_utensil.yaml", "transfer_drink.yaml", "transfer_wipe.yaml"]:
            data = _load(bt_dir, fname)
            assert _get_param_value(data, "RobotTransferStartCue") == "voice_led"

    def test_missing_yaml_file_produces_warning(self, bt_dir: Path):
        (bt_dir / "acquire_bite.yaml").unlink()
        bundle = {"confirm_feeding_pickup": "wait for me"}
        warnings = apply_bundle_to_behavior_trees(bundle, bt_dir)
        assert any("not found" in w for w in warnings)

    def test_unknown_bundle_field_ignored(self, bt_dir: Path):
        bundle = {"nonexistent_field": "whatever"}
        warnings = apply_bundle_to_behavior_trees(bundle, bt_dir)
        assert not warnings

    def test_empty_bundle_no_changes(self, bt_dir: Path):
        before = _load(bt_dir, "acquire_bite.yaml")
        speed_before = _get_param_value(before, "Speed")

        warnings = apply_bundle_to_behavior_trees({}, bt_dir)
        assert not warnings

        after = _load(bt_dir, "acquire_bite.yaml")
        assert _get_param_value(after, "Speed") == speed_before


class TestApplyBundleFullBundle:
    """Apply a realistic full bundle and verify no errors."""

    def test_full_bundle_applies_without_error(self, bt_dir: Path):
        bundle = {
            "robot_speed": "slow",
            "confirm_feeding_pickup": "countdown (60 sec)",
            "confirm_navigation_arrival": "countdown (15 sec)",
            "confirm_manipulation": "wait for me",
            "wait_before_autocontinue_task_selection": "wait for me",
            "wait_before_autocontinue_bite_selection": "countdown (60 sec)",
            "outside_mouth_distance": "far",
            "convey_robot_ready_for_initiating_transfer": "speech",
            "detect_user_ready_for_initiating_transfer_feeding": "open mouth",
            "detect_user_ready_for_initiating_transfer_drinking": "button",
            "detect_user_ready_for_initiating_transfer_wiping": "proceed automatically after a pause",
            "convey_robot_ready_for_completing_transfer": "LED",
            "detect_user_completed_transfer_feeding": "perception",
            "detect_user_completed_transfer_drinking": "proceed automatically after a pause",
            "detect_user_completed_transfer_wiping": "button",
            "transfer_mode": "outside mouth transfer",
            "microwave_time": "no microwave",
            "skewering_axis": "parallel to major axis",
            "retract_between_bites": "yes",
            "bite_dipping_preference": "do not dip",
        }
        warnings = apply_bundle_to_behavior_trees(bundle, bt_dir)
        assert not any("Error" in w for w in warnings)

        data = _load(bt_dir, "acquire_bite.yaml")
        assert _get_param_value(data, "Speed") == "low"
        assert _get_param_value(data, "PickupConfirm") == 60.0            # countdown 60 s
        assert _get_param_value(data, "BiteSelectionAutocontinueSeconds") == 60.0
        ut = _load(bt_dir, "transfer_utensil.yaml")
        assert _get_param_value(ut, "TaskReselectionAutocontinueSeconds") == 0.0   # wait for me
        nav = _load(bt_dir, "navigate_to_table.yaml")
        assert _get_param_value(nav, "ArrivalConfirm") == 15.0           # countdown 15 s
        pk = _load(bt_dir, "pick_plate_from_fridge.yaml")
        assert _get_param_value(pk, "ManipulationConfirm") == 0.0        # wait for me
        wp = _load(bt_dir, "transfer_wipe.yaml")
        assert _get_param_value(wp, "PickupConfirm") == 60.0


# -------------------------------------------------------------------
# apply_transfer_mode
# -------------------------------------------------------------------


class TestApplyTransferMode:
    """apply_transfer_mode is validate-only: the robot's transfer type is
    fixed by the command line, so it never applies anything and raises loudly
    on the unsupported 'inside mouth transfer' value."""

    def test_outside_is_noop(self):
        apply_transfer_mode({"transfer_mode": "outside mouth transfer"})

    def test_inside_raises(self):
        with pytest.raises(RuntimeError, match="inside mouth transfer"):
            apply_transfer_mode({"transfer_mode": "inside mouth transfer"})

    def test_unknown_transfer_mode_raises(self):
        with pytest.raises(ValueError, match="Unknown transfer_mode"):
            apply_transfer_mode({"transfer_mode": "sideways"})

    def test_no_transfer_mode_in_bundle_is_noop(self):
        apply_transfer_mode({})


# -------------------------------------------------------------------
# apply_microwave_preference
# -------------------------------------------------------------------


class _FakeAtom:
    """Minimal stand-in for GroundAtom(FoodHeated, []) so tests don't need
    the heavy relational_structs import."""

    def __init__(self, name: str) -> None:
        self.name = name

    def __eq__(self, other: object) -> bool:
        return isinstance(other, _FakeAtom) and self.name == other.name

    def __hash__(self) -> int:
        return hash(self.name)


class TestApplyMicrowavePreference:

    def _food_heated(self) -> _FakeAtom:
        return _FakeAtom("FoodHeated")

    def test_no_microwave_adds_food_heated(self):
        atoms: set = set()
        fh = self._food_heated()
        result = apply_microwave_preference({"microwave_time": "no microwave"}, atoms, fh)
        assert fh in atoms
        assert result is None

    def test_1_min_removes_food_heated(self):
        fh = self._food_heated()
        atoms = {fh}
        result = apply_microwave_preference({"microwave_time": "1 min"}, atoms, fh)
        assert fh not in atoms
        assert result == 60

    def test_2_min_returns_120(self):
        fh = self._food_heated()
        atoms: set = set()
        result = apply_microwave_preference({"microwave_time": "2 min"}, atoms, fh)
        assert fh not in atoms
        assert result == 120

    def test_30_secs_returns_30(self):
        fh = self._food_heated()
        atoms: set = set()
        result = apply_microwave_preference({"microwave_time": "30 secs"}, atoms, fh)
        assert result == 30

    def test_no_microwave_time_in_bundle_is_noop(self):
        fh = self._food_heated()
        atoms: set = set()
        result = apply_microwave_preference({}, atoms, fh)
        assert fh not in atoms
        assert result is None

    def test_unknown_microwave_time_raises(self):
        fh = self._food_heated()
        with pytest.raises(ValueError, match="Unknown microwave_time"):
            apply_microwave_preference({"microwave_time": "5 min"}, set(), fh)

    def test_microwave_time_map_covers_all_options(self):
        assert set(_MICROWAVE_TIME_MAP.keys()) == {"no microwave", "30 secs", "1 min", "2 min"}


# -------------------------------------------------------------------
# apply_dip_preference
# -------------------------------------------------------------------


class _FakeFlair:
    """Minimal stand-in for FLAIR so tests don't need heavy imports."""

    def __init__(self) -> None:
        self._allow_dip = True

    def set_allow_dip(self, allow: bool) -> None:
        self._allow_dip = allow


class TestApplyDipPreference:

    def test_do_not_dip_sets_false(self):
        flair = _FakeFlair()
        apply_dip_preference({"bite_dipping_preference": "do not dip"}, flair)
        assert flair._allow_dip is False

    def test_less_keeps_true(self):
        flair = _FakeFlair()
        apply_dip_preference({"bite_dipping_preference": "less"}, flair)
        assert flair._allow_dip is True

    def test_more_keeps_true(self):
        flair = _FakeFlair()
        apply_dip_preference({"bite_dipping_preference": "more"}, flair)
        assert flair._allow_dip is True

    def test_missing_key_is_noop(self):
        flair = _FakeFlair()
        apply_dip_preference({}, flair)
        assert flair._allow_dip is True

    def test_none_flair_is_noop(self):
        apply_dip_preference({"bite_dipping_preference": "do not dip"}, None)
