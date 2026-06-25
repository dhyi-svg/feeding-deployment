"""Tests for the staged PreferenceSession (predict -> ask -> color -> finalize).

Run with:
    PYTHONPATH=src python -m pytest tests/test_preference_session.py -v

The PredictionModel is faked (no OpenAI / no network) and the web interface is
scripted, so these exercise the session's control flow — locking, reprediction
pinning, color recording/propagation, the wait->autocontinue wiring, and the
single per-day memory update — without robot hardware.
"""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest

from feeding_deployment.integration.preference_session import PreferenceSession
from feeding_deployment.integration.apply_preferences import (
    _load_yaml,
    _save_yaml,
    _set_param_value,
)
from feeding_deployment.preference_learning.config.preference_bundle import (
    COLOR_FIELDS,
    color_to_bt,
    parse_color,
)
from feeding_deployment.preference_learning.methods.prediction_model import (
    PREF_KIND,
    PREF_OPTIONS,
)
from feeding_deployment.preference_learning.methods.utils import PREF_FIELDS

BT_SOURCE = Path("src/feeding_deployment/actions/behavior_trees")
CTX = {"meal": "eggs", "setting": "Personal", "time_of_day": "morning"}


class FakeModel:
    """Deterministic stand-in for PredictionModel. Predicts the first option for
    categorical dims and the seed for color dims; corrected values are pinned.
    Records predict/update calls for assertions."""

    def __init__(self):
        self.predict_calls = []
        self.update_calls = []

    def predict_bundle(self, context, corrected, color_seeds=None):
        self.predict_calls.append({"corrected": dict(corrected), "color_seeds": dict(color_seeds or {})})
        out = {}
        for f in PREF_FIELDS:
            if PREF_KIND.get(f) == "color":
                out[f] = parse_color((color_seeds or {}).get(f))
            else:
                out[f] = PREF_OPTIONS[f][0]
        for k, v in corrected.items():
            out[k] = v
        return out

    def update(self, day, context, corrected, ground_truth_bundle):
        self.update_calls.append({
            "day": day, "corrected": dict(corrected), "gt": dict(ground_truth_bundle),
        })

    def next_day(self):
        return 1


class FakeWeb:
    """Scripted stepwise correction interface. ``answers`` maps field -> value;
    absent fields are confirmed (echo the prediction)."""

    def __init__(self, answers=None):
        self.answers = answers or {}
        self.steps = []
        self.started = None
        self.finished = False

    def start_preference_correction(self, total, secs):
        self.started = (total, secs)

    def send_preference_step(self, field, predicted, options, step, total, autocontinue_seconds):
        self.steps.append((field, autocontinue_seconds))
        return self.answers.get(field, predicted)

    def finish_preference_correction(self):
        self.finished = True


@pytest.fixture()
def bt_dir(tmp_path: Path) -> Path:
    dest = tmp_path / "behavior_trees"
    shutil.copytree(BT_SOURCE, dest)
    return dest


def _set_yaml_color(bt_dir: Path, location: str, hsv_range) -> None:
    fp = bt_dir / f"pick_plate_from_{location}.yaml"
    data = _load_yaml(fp)
    hc, cr = color_to_bt(hsv_range)
    _set_param_value(data, "HandleColor", hc)
    _set_param_value(data, "ColorRange", cr)
    _save_yaml(fp, data)


# ---------------------------------------------------------------------------


def test_start_predicts_full_bundle_including_colors(bt_dir):
    s = PreferenceSession(FakeModel(), bt_dir, CTX)
    s.start()
    assert set(s.bundle.keys()) >= set(PREF_FIELDS)
    for f in COLOR_FIELDS:
        assert isinstance(s.bundle[f], dict)


def test_color_seed_comes_from_bt_and_falls_back(bt_dir):
    # Seed the fridge YAML with a distinctive color; the FakeModel echoes seeds.
    _set_yaml_color(bt_dir, "fridge", {"h": 10, "s": 20, "v": 30, "range": 0.2})
    s = PreferenceSession(FakeModel(), bt_dir, CTX)
    s.start()
    assert s.bundle["plate_color_fridge"] == {"h": 10, "s": 20, "v": 30, "range": 0.2}


def test_ask_confirm_finalizes_without_correction(bt_dir):
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    s.ask(["robot_speed", "wait_before_autocontinue_seconds"])
    assert "robot_speed" in s.finalized
    assert "robot_speed" not in s.corrected  # confirmed, not corrected


def test_ask_correction_repredicts_and_pins(bt_dir):
    model = FakeModel()
    web = FakeWeb({"robot_speed": "fast"})
    s = PreferenceSession(model, bt_dir, CTX, web_interface=web)
    s.start()
    n_before = len(model.predict_calls)
    s.ask(["robot_speed", "skewering_axis"])

    assert s.bundle["robot_speed"] == "fast"
    assert "robot_speed" in s.corrected
    # a correction triggers a reprediction of the still-open dims ...
    assert len(model.predict_calls) > n_before
    # ... and the correction is pinned in that reprediction.
    assert model.predict_calls[-1]["corrected"].get("robot_speed") == "fast"
    assert web.finished


def test_wait_pref_drives_autocontinue(bt_dir):
    web = FakeWeb()
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=web)
    s.start()
    assert s.wait_seconds == 10.0  # default before finalized
    s._finalize("wait_before_autocontinue_seconds", "100 sec", changed=True)
    assert s.wait_seconds == 100.0


def test_record_color_correction_finalizes_and_propagates(bt_dir):
    model = FakeModel()
    s = PreferenceSession(model, bt_dir, CTX)
    s.start()
    # User corrected the fridge color during pickup (written to the BT YAML).
    _set_yaml_color(bt_dir, "fridge", {"h": 10, "s": 20, "v": 30, "range": 0.2})
    n_before = len(model.predict_calls)
    s.record_color("fridge")

    assert "plate_color_fridge" in s.finalized
    assert "plate_color_fridge" in s.corrected
    assert s.bundle["plate_color_fridge"] == {"h": 10, "s": 20, "v": 30, "range": 0.2}
    assert len(model.predict_calls) > n_before  # propagated reprediction


def test_record_color_confirm_is_not_a_correction(bt_dir):
    model = FakeModel()
    s = PreferenceSession(model, bt_dir, CTX)
    s.start()
    # Leave the YAML at the predicted color -> no change.
    predicted = s.bundle["plate_color_microwave"]
    _set_yaml_color(bt_dir, "microwave", predicted)
    s.record_color("microwave")
    assert "plate_color_microwave" in s.finalized
    assert "plate_color_microwave" not in s.corrected


def test_finalize_meal_one_update_with_full_bundle(bt_dir):
    model = FakeModel()
    s = PreferenceSession(model, bt_dir, CTX)
    s.start()
    s.finalize_meal(day=7)
    assert len(model.update_calls) == 1
    call = model.update_calls[0]
    assert call["day"] == 7
    assert set(call["gt"].keys()) >= set(PREF_FIELDS)
    # Every dim is finalized (unasked dims confirmed at the prediction).
    assert all(f in s.finalized for f in PREF_FIELDS)


def test_ask_skips_already_finalized_dims(bt_dir):
    # Resume scenario: robot_speed was asked before the crash (finalized). On
    # re-ask it must be skipped, and only the still-open dim is shown.
    web = FakeWeb()
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=web)
    s.start()
    s._finalize("robot_speed", "fast", changed=True)
    s.ask(["robot_speed", "skewering_axis"])
    asked = [field for field, _secs in web.steps]
    assert asked == ["skewering_axis"]
    assert s.bundle["robot_speed"] == "fast"  # untouched


def test_capture_resume_roundtrip(bt_dir):
    # Build a session with a correction + a recorded color, snapshot it, then
    # rehydrate a brand-new session (as on a crash/restart) and confirm the
    # per-meal state and the actuated color survive.
    model = FakeModel()
    web = FakeWeb({"robot_speed": "fast"})
    s = PreferenceSession(model, bt_dir, CTX, web_interface=web)
    s.start()
    s.ask(["robot_speed"])
    _set_yaml_color(bt_dir, "fridge", {"h": 10, "s": 20, "v": 30, "range": 0.2})
    s.record_color("fridge")

    snapshot = s.capture_state()
    assert snapshot["corrected"]["robot_speed"] == "fast"
    assert "plate_color_fridge" in snapshot["finalized"]

    s2 = PreferenceSession(FakeModel(), bt_dir, CTX)
    s2.resume_from_state(snapshot)
    assert s2.finalized == s.finalized
    assert s2.corrected == s.corrected
    assert s2.bundle == s.bundle
    assert s2.bundle["plate_color_fridge"] == {"h": 10, "s": 20, "v": 30, "range": 0.2}


def test_resume_then_finalize_meal_keeps_precrash_corrections(bt_dir):
    # The end-of-meal learning update must reflect a correction made before the
    # crash, even though a fresh model is built on resume.
    web = FakeWeb({"robot_speed": "fast"})
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=web)
    s.start()
    s.ask(["robot_speed"])
    snapshot = s.capture_state()

    model2 = FakeModel()
    s2 = PreferenceSession(model2, bt_dir, CTX)
    s2.resume_from_state(snapshot)
    s2.finalize_meal(day=3)

    assert len(model2.update_calls) == 1
    assert model2.update_calls[0]["corrected"].get("robot_speed") == "fast"


def test_resume_does_not_reask_finalized_color(bt_dir):
    # A color recorded before the crash stays finalized after resume, so a
    # repeated record_color() is a no-op (no second correction / reprediction).
    s = PreferenceSession(FakeModel(), bt_dir, CTX)
    s.start()
    _set_yaml_color(bt_dir, "fridge", {"h": 1, "s": 2, "v": 3, "range": 0.1})
    s.record_color("fridge")
    snapshot = s.capture_state()

    model2 = FakeModel()
    s2 = PreferenceSession(model2, bt_dir, CTX)
    s2.resume_from_state(snapshot)
    n_before = len(model2.predict_calls)
    s2.record_color("fridge")  # already finalized -> early return
    assert len(model2.predict_calls) == n_before  # no reprediction
    assert "plate_color_fridge" in s2.finalized


def test_repredictions_do_not_write_memory(bt_dir):
    model = FakeModel()
    web = FakeWeb({"robot_speed": "fast"})
    s = PreferenceSession(model, bt_dir, CTX, web_interface=web)
    s.start()
    s.ask(["robot_speed", "skewering_axis"])
    _set_yaml_color(bt_dir, "fridge", {"h": 5, "s": 5, "v": 5, "range": 0.3})
    s.record_color("fridge")
    # No memory update until finalize_meal, despite multiple repredictions.
    assert model.update_calls == []
    s.finalize_meal(day=1)
    assert len(model.update_calls) == 1
