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
    parse_nav_offset,
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
    categorical dims and the seed for color / nav-offset dims; corrected values
    are pinned. Records predict/update calls for assertions."""

    def __init__(self):
        self.predict_calls = []
        self.update_calls = []

    def predict_bundle(self, context, corrected, confirmed=None, color_seeds=None, nav_offset_seeds=None):
        self.predict_calls.append({
            "corrected": dict(corrected),
            "confirmed": dict(confirmed or {}),
            "color_seeds": dict(color_seeds or {}),
            "nav_offset_seeds": dict(nav_offset_seeds or {}),
        })
        out = {}
        for f in PREF_FIELDS:
            if PREF_KIND.get(f) == "color":
                out[f] = parse_color((color_seeds or {}).get(f))
            elif PREF_KIND.get(f) == "nav_offset":
                out[f] = parse_nav_offset((nav_offset_seeds or {}).get(f))
            elif PREF_KIND.get(f) == "text":
                out[f] = f"predicted {f}"
            else:
                out[f] = PREF_OPTIONS[f][0]
        for k, v in corrected.items():
            out[k] = v
        return out

    def update(self, day, context, corrected, ground_truth_bundle):
        self.update_calls.append({
            "day": day, "corrected": dict(corrected), "gt": dict(ground_truth_bundle),
        })


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

    def send_preference_step(self, field, predicted, options, step, total, autocontinue_seconds, kind="categorical"):
        self.steps.append((field, autocontinue_seconds))
        return self.answers.get(field, predicted)

    def finish_preference_correction(self):
        self.finished = True


# Only the YAMLs the session reads or seeds from: the color / nav-offset
# write-back targets plus the categorical-dim targets. Copying — and rewriting
# on every _apply_non_planning — all 31 factory trees made this suite take
# minutes; files absent from the fixture only produce (unasserted) warnings
# from apply_bundle_to_behavior_trees.
_FIXTURE_YAMLS = [
    "acquire_bite.yaml",
    "navigate_to_fridge.yaml",
    "navigate_to_microwave.yaml",
    "navigate_to_sink.yaml",
    "navigate_to_table.yaml",
    "pick_plate_from_fridge.yaml",
    "pick_plate_from_microwave.yaml",
    "pick_plate_from_table.yaml",
    "press_microwave_button.yaml",
    "transfer_drink.yaml",
    "transfer_utensil.yaml",
    "transfer_wipe.yaml",
]


@pytest.fixture()
def bt_dir(tmp_path: Path) -> Path:
    dest = tmp_path / "behavior_trees"
    dest.mkdir()
    for name in _FIXTURE_YAMLS:
        shutil.copy(BT_SOURCE / name, dest / name)
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


def test_repredict_receives_confirmed_and_corrected_split(bt_dir):
    # robot_speed is confirmed (accepted as predicted), skewering_axis is
    # corrected. The reprediction triggered by the correction must receive the
    # true correction under ``corrected`` and the passive accept under
    # ``confirmed`` — both pinned, but distinguishable in the prompt.
    model = FakeModel()
    other = PREF_OPTIONS["skewering_axis"][1]
    web = FakeWeb({"skewering_axis": other})
    s = PreferenceSession(model, bt_dir, CTX, web_interface=web)
    s.start()
    s.ask(["robot_speed", "skewering_axis"])
    # The correction hit the LAST dim of the batch, so its reprediction runs in
    # the background -- settle it before asserting.
    assert s.wait_for_reprediction(5.0)

    last = model.predict_calls[-1]
    assert last["corrected"] == {"skewering_axis": other}
    assert last["confirmed"].get("robot_speed") == PREF_OPTIONS["robot_speed"][0]
    assert "skewering_axis" not in last["confirmed"]


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
    assert s.wait_for_reprediction(5.0)  # propagation runs in the background
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


# ---------------------------------------------------------------------------
# Settings overlay: view / edit already-set preferences
# ---------------------------------------------------------------------------


def test_settings_view_lists_finalized_categorical_only(bt_dir):
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    s.ask(["robot_speed", "skewering_axis"])
    view = s.settings_view()
    assert {p["field"] for p in view} == {"robot_speed", "skewering_axis"}
    # No color dims are ever exposed in the overlay.
    assert all(p["field"] not in COLOR_FIELDS for p in view)
    p = next(p for p in view if p["field"] == "robot_speed")
    assert p["editable"] is True
    assert p["value"] == s.bundle["robot_speed"]
    assert p["options"] == PREF_OPTIONS["robot_speed"]
    assert p["description"]  # carries a non-empty description


def test_edit_applies_correction_and_carries_to_finalize(bt_dir):
    model = FakeModel()
    s = PreferenceSession(model, bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    s.ask(["robot_speed"])  # confirmed at first option
    other = PREF_OPTIONS["robot_speed"][1]
    assert s.edit("robot_speed", other) is True
    assert s.bundle["robot_speed"] == other
    assert s.corrected["robot_speed"] == other
    s.finalize_meal(day=3)
    assert model.update_calls[-1]["gt"]["robot_speed"] == other
    assert model.update_calls[-1]["corrected"]["robot_speed"] == other


def test_edit_rejects_invalid_option_and_color_dim(bt_dir):
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    s.ask(["robot_speed"])
    assert s.edit("robot_speed", "not-an-option") is False
    assert s.edit("plate_color_fridge", {"h": 1, "s": 2, "v": 3, "range": 0.1}) is False
    # unchanged
    assert s.bundle["robot_speed"] == PREF_OPTIONS["robot_speed"][0]


def test_microwave_time_locks_after_apply_microwave(bt_dir):
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    s.ask(["microwave_time"])
    v = next(p for p in s.settings_view() if p["field"] == "microwave_time")
    assert v["editable"] is True  # editable before the routing is committed

    s.apply_microwave(set(), object())  # commits FoodHeated routing -> locks the dim

    v = next(p for p in s.settings_view() if p["field"] == "microwave_time")
    assert v["editable"] is False
    before = s.bundle["microwave_time"]
    other = next(o for o in PREF_OPTIONS["microwave_time"] if o != before)
    assert s.edit("microwave_time", other) is False  # no-op while locked
    assert s.bundle["microwave_time"] == before


def test_transfer_mode_edit_defers_reinit_until_flush(bt_dir):
    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    s.ask(["transfer_mode"])
    before = s.bundle["transfer_mode"]
    other = next(o for o in PREF_OPTIONS["transfer_mode"] if o != before)
    assert s.edit("transfer_mode", other) is True
    assert s._pending_transfer_reinit is True  # deferred, not applied on the worker
    # Settle the background reprediction first: its apply re-arms the pending
    # flag, exactly as run.py's join-then-flush ordering settles it.
    assert s.wait_for_reprediction(5.0)
    s.flush_pending_inmemory()  # main-thread safe boundary
    assert s._pending_transfer_reinit is False


def test_settings_view_and_edit_are_thread_safe(bt_dir):
    """A reader thread iterating settings_view() concurrently with edits must never
    raise (e.g. 'dict changed size during iteration').

    The YAML actuation is stubbed out: this test targets the in-memory
    bookkeeping races (file atomicity has its own dedicated test below), and
    200 edits x ~30 YAML rewrites each made it take minutes."""
    import threading

    s = PreferenceSession(FakeModel(), bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    s._apply_non_planning = lambda *a, **k: []
    s._write_open_colors_to_bt = lambda: None
    s._write_open_nav_offsets_to_bt = lambda: None
    s._color_seeds = lambda: {}
    s._nav_offset_seeds = lambda: {}
    cat = [d for d in PREF_FIELDS if PREF_KIND.get(d) != "color"][:6]
    s.ask(cat)

    errors = []
    stop = threading.Event()

    def reader():
        try:
            while not stop.is_set():
                for p in s.settings_view():
                    _ = (p["field"], p["value"], p["editable"])
        except Exception as e:  # pragma: no cover - failure path
            errors.append(e)

    t = threading.Thread(target=reader)
    t.start()
    try:
        opts = PREF_OPTIONS["robot_speed"]
        for i in range(200):
            s.edit("robot_speed", opts[i % len(opts)])
    finally:
        stop.set()
        t.join()
    assert s.wait_for_reprediction(5.0)  # drain the coalesced background worker
    assert not errors


def test_save_yaml_is_atomic_under_concurrent_reads(tmp_path):
    """A concurrent reader must always see a complete YAML file, never a partial
    one (the property that prevents a settings edit from crashing a skill)."""
    import threading
    import time

    fp = tmp_path / "bt.yaml"
    _save_yaml(fp, {"parameters": [{"name": "A", "value": 0}]})

    errors = []
    stop = threading.Event()

    def writer():
        i = 0
        while not stop.is_set():
            _save_yaml(fp, {"parameters": [{"name": "A", "value": i}], "pad": "x" * 4000})
            i += 1

    def reader():
        try:
            while not stop.is_set():
                d = _load_yaml(fp)
                assert isinstance(d, dict) and "parameters" in d
        except Exception as e:  # pragma: no cover - failure path
            errors.append(e)

    tw, tr = threading.Thread(target=writer), threading.Thread(target=reader)
    tw.start()
    tr.start()
    time.sleep(0.5)
    stop.set()
    tw.join()
    tr.join()
    assert not errors


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
    # Join before simulating the pickup's color write-back: mirrors run.py's
    # pre-skill barrier (pick_plate_* consumes predictions), without which the
    # in-flight robot_speed repredict could interleave with this raw write.
    assert s.wait_for_reprediction(5.0)
    _set_yaml_color(bt_dir, "fridge", {"h": 10, "s": 20, "v": 30, "range": 0.2})
    s.record_color("fridge")
    # Settle the background propagation so the snapshot and the live bundle
    # compare against the same settled state.
    assert s.wait_for_reprediction(5.0)

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


def test_on_change_persists_every_correction(bt_dir):
    # The durability hook must fire the instant each correction is locked, so a
    # crash before the next sub-skill checkpoint loses nothing. The latest
    # snapshot must round-trip into a resumed session.
    saved = []
    web = FakeWeb({"robot_speed": "fast"})
    s = PreferenceSession(
        FakeModel(), bt_dir, CTX, web_interface=web,
        on_change=lambda state: saved.append(state),
    )
    s.start()  # prediction only -> no _finalize, no persist
    assert saved == []

    s.ask(["robot_speed", "wait_before_autocontinue_seconds"])
    assert saved, "ask() must persist on each finalize"
    assert saved[-1]["corrected"].get("robot_speed") == "fast"

    _set_yaml_color(bt_dir, "fridge", {"h": 7, "s": 8, "v": 9, "range": 0.1})
    s.record_color("fridge")
    assert "plate_color_fridge" in saved[-1]["finalized"]

    # The most recent snapshot fully restores the session (the resume contract).
    s2 = PreferenceSession(FakeModel(), bt_dir, CTX)
    s2.resume_from_state(saved[-1])
    assert s2.corrected == s.corrected
    assert s2.finalized == s.finalized


def test_on_change_failure_does_not_break_meal(bt_dir):
    # Persistence is best-effort: a failing hook must not abort a correction.
    def boom(_state):
        raise RuntimeError("disk full")

    s = PreferenceSession(
        FakeModel(), bt_dir, CTX, web_interface=FakeWeb(), on_change=boom,
    )
    s.start()
    s.ask(["robot_speed"])  # must not raise
    assert "robot_speed" in s.finalized


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


def test_repredict_runs_in_background_and_join_waits(bt_dir):
    """A correction must return immediately (LLM off the critical path) while
    wait_for_reprediction blocks until the repredicted values are applied."""
    import threading
    import time

    release = threading.Event()

    class SlowModel(FakeModel):
        def predict_bundle(self, *args, **kwargs):
            if self.predict_calls:  # block repredictions only, not start()
                assert release.wait(5.0), "test gate never released"
            return super().predict_bundle(*args, **kwargs)

    model = SlowModel()
    s = PreferenceSession(model, bt_dir, CTX)
    s.start()

    _set_yaml_color(bt_dir, "fridge", {"h": 10, "s": 20, "v": 30, "range": 0.2})
    t0 = time.monotonic()
    s.record_color("fridge")
    assert time.monotonic() - t0 < 1.0, "record_color must not block on the LLM"
    # The correction itself is locked synchronously even while the LLM runs.
    assert "plate_color_fridge" in s.corrected
    assert not s.wait_for_reprediction(timeout=0.05)  # still in flight

    release.set()
    assert s.wait_for_reprediction(5.0)
    assert len(model.predict_calls) >= 2  # start + background repredict


def test_repredict_triggers_coalesce(bt_dir):
    """Settings edits arriving while a repredict is in flight coalesce into ONE
    extra pass carrying the newest corrections -- never one LLM call per edit.
    (edit() is the burst source: unlike ask()/record_*, it does not join on
    entry, mirroring the settings apply-worker thread.)"""
    import threading

    release = threading.Event()
    started = threading.Event()

    class SlowModel(FakeModel):
        def predict_bundle(self, *args, **kwargs):
            if self.predict_calls:  # gate repredictions only, not start()
                started.set()
                assert release.wait(5.0)
            return super().predict_bundle(*args, **kwargs)

    model = SlowModel()
    s = PreferenceSession(model, bt_dir, CTX, web_interface=FakeWeb())
    s.start()
    # Confirm (not correct) three dims so they are finalized and editable
    # without triggering any repredict yet.
    s.ask(["robot_speed", "skewering_axis", "web_interface_confirmation"])
    assert len(model.predict_calls) == 1  # start() only

    # First edit starts the worker; wait until its pass is provably inside the
    # (gated) LLM call, then land two more edits mid-flight: they coalesce into
    # a single follow-up pass.
    assert s.edit("robot_speed", PREF_OPTIONS["robot_speed"][1]) is True
    assert started.wait(5.0), "worker never reached the gated LLM call"
    assert s.edit("skewering_axis", PREF_OPTIONS["skewering_axis"][1]) is True
    assert s.edit("web_interface_confirmation", PREF_OPTIONS["web_interface_confirmation"][1]) is True

    release.set()
    assert s.wait_for_reprediction(5.0)
    # start (1) + gated first pass (1) + one coalesced pass (1) = 3.
    assert len(model.predict_calls) == 3
    # The final pass was pinned with every edit made.
    assert set(model.predict_calls[-1]["corrected"]) >= {
        "robot_speed", "skewering_axis", "web_interface_confirmation",
    }


def test_bt_consumes_predictions_predicate():
    from feeding_deployment.integration.preference_session import bt_consumes_predictions

    for name in ("pick_plate_from_fridge", "navigate_to_table", "transfer_utensil",
                 "transfer_drink", "acquire_bite"):
        assert bt_consumes_predictions(name), name
    for name in ("place_plate_on_holder", "open_fridge", "close_microwave",
                 "press_microwave_button", "gaze_at_table", "place_plate_in_sink"):
        assert not bt_consumes_predictions(name), name


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
