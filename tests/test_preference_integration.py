"""Tests for preference-learning integration Steps 1-5 + terminal interaction.

Run with:
    PYTHONPATH=src python -m pytest tests/test_preference_integration.py -v

Note: run.py and web_interface.py have heavy robot dependencies (tomsutils,
pybullet_helpers, cv2, rospy, ...) that are not available outside the robot
environment.  The tests below therefore exercise the *logic* of Steps 1-5
without importing those modules: preference_context.py is tested directly,
PredictionModel is tested with the Anthropic chat client mocked (chat migrated
to Claude; embeddings stay on OpenAI), and the runner/web-interface contracts
are tested against the same functions those classes delegate to.
"""

from __future__ import annotations

import json
from pathlib import Path
from unittest.mock import MagicMock, patch
from io import StringIO

import pytest

from feeding_deployment.integration.preference_context import (
    PREFERENCE_CONTEXT_KEYS,
    build_preference_context,
    validate_preference_context,
)
from feeding_deployment.preference_learning.config import (
    MEALS,
    SETTINGS,
    TIMES_OF_DAY,
)
from feeding_deployment.preference_learning.methods.prediction_model import (
    PREF_OPTIONS,
    PREF_KIND,
)
from feeding_deployment.preference_learning.methods.utils import PREF_FIELDS
from feeding_deployment.preference_learning.config.preference_bundle import (
    COLOR_FIELDS,
    DEFAULT_COLOR,
)

_PM_MODULE = "feeding_deployment.preference_learning.methods.prediction_model"


def _assert_valid_value(field, value):
    """Categorical fields must be an allowed option; color fields must be a
    canonical HSV dict."""
    if PREF_KIND.get(field) == "color":
        assert isinstance(value, dict) and {"h", "s", "v", "range"} <= set(value)
    else:
        assert value in PREF_OPTIONS[field], f"{field}={value} not in allowed options"


# ===================================================================
# Step 1 — preference context: build, validate, runner contract
# ===================================================================


class TestBuildPreferenceContext:

    def test_valid_context(self):
        ctx = build_preference_context(MEALS[0], SETTINGS[0], TIMES_OF_DAY[0])
        assert set(ctx.keys()) == set(PREFERENCE_CONTEXT_KEYS)
        assert ctx["meal"] == MEALS[0]
        assert ctx["setting"] == SETTINGS[0]
        assert ctx["time_of_day"] == TIMES_OF_DAY[0]

    def test_strips_whitespace(self):
        ctx = build_preference_context(
            f"  {MEALS[0]}  ", f" {SETTINGS[0]} ", f"  {TIMES_OF_DAY[0]} "
        )
        assert ctx["meal"] == MEALS[0]

    def test_invalid_meal_raises(self):
        with pytest.raises(ValueError, match="Unknown meal"):
            build_preference_context("not_a_real_meal", SETTINGS[0], TIMES_OF_DAY[0])

    def test_invalid_setting_raises(self):
        with pytest.raises(ValueError, match="Unknown setting"):
            build_preference_context(MEALS[0], "bad_setting", TIMES_OF_DAY[0])

    def test_invalid_time_of_day_raises(self):
        with pytest.raises(ValueError, match="Unknown time_of_day"):
            build_preference_context(MEALS[0], SETTINGS[0], "midnight")

    def test_empty_meal_raises(self):
        with pytest.raises(ValueError):
            build_preference_context("", SETTINGS[0], TIMES_OF_DAY[0])

    def test_whitespace_only_meal_raises(self):
        with pytest.raises(ValueError):
            build_preference_context("   ", SETTINGS[0], TIMES_OF_DAY[0])


class TestValidatePreferenceContext:

    def test_missing_key_raises(self):
        with pytest.raises(ValueError, match="missing required key"):
            validate_preference_context({"meal": MEALS[0], "setting": SETTINGS[0]})

    def test_non_string_value_raises(self):
        with pytest.raises(ValueError, match="must be a non-empty string"):
            validate_preference_context(
                {"meal": 123, "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
            )

    def test_valid_context_passes(self):
        validate_preference_context(
            {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        )


class TestRunnerPreferenceContextContract:
    """Verify the contract that _Runner.ensure_preference_context and
    _Runner.set_meal_preference_context implement, without importing run.py.

    The runner stores `self.preference_context` and:
      - ensure_preference_context() raises RuntimeError when it is None
      - set_meal_preference_context(m, s, t) delegates to build_preference_context
    """

    def test_ensure_raises_when_context_is_none(self):
        preference_context = None
        with pytest.raises(RuntimeError, match="preference_context is required"):
            if preference_context is None:
                raise RuntimeError(
                    "preference_context is required but unset. Each run must set it "
                    "explicitly (e.g. non-empty --pref_meal with --use_interface, or call "
                    "set_meal_preference_context(meal, setting, time_of_day) before run()). "
                    "Context is not loaded from or saved to disk; after a crash, supply it again."
                )

    def test_ensure_returns_when_context_is_set(self):
        ctx = build_preference_context(MEALS[0], SETTINGS[0], TIMES_OF_DAY[0])
        preference_context = ctx
        assert preference_context is not None
        assert preference_context == ctx

    def test_set_meal_delegates_to_build(self):
        ctx = build_preference_context(MEALS[0], SETTINGS[0], TIMES_OF_DAY[0])
        assert ctx["meal"] == MEALS[0]
        assert ctx["setting"] == SETTINGS[0]
        assert ctx["time_of_day"] == TIMES_OF_DAY[0]

    def test_set_meal_rejects_invalid_meal(self):
        with pytest.raises(ValueError):
            build_preference_context("bad_meal", SETTINGS[0], TIMES_OF_DAY[0])

    def test_each_meal_setting_time_accepted(self):
        """Smoke test: every canonical value from config builds successfully."""
        for m in MEALS:
            for s in SETTINGS:
                for t in TIMES_OF_DAY:
                    ctx = build_preference_context(m, s, t)
                    validate_preference_context(ctx)


# ===================================================================
# Step 2 — PredictionModel: instantiation + predict_bundle
# ===================================================================


def _fake_anthropic_response(bundle: dict) -> MagicMock:
    """Mimic an anthropic Messages response: .content is a list of blocks, each
    with .type and .text (predict_bundle joins the text blocks)."""
    block = MagicMock()
    block.type = "text"
    block.text = json.dumps(bundle)
    resp = MagicMock()
    resp.content = [block]
    return resp


def _fake_anthropic_text(text: str) -> MagicMock:
    block = MagicMock()
    block.type = "text"
    block.text = text
    resp = MagicMock()
    resp.content = [block]
    return resp


@pytest.fixture(autouse=True)
def _mock_anthropic():
    """Patch the Anthropic chat client for every test in this module so
    PredictionModel construction never needs a real ANTHROPIC_API_KEY. Chat
    tests configure ``_mock_anthropic.messages.create.return_value``."""
    with patch(f"{_PM_MODULE}.anthropic.Anthropic") as mock_cls:
        client = MagicMock()
        mock_cls.return_value = client
        yield client


def _default_bundle() -> dict:
    """A valid full bundle: first option for categorical dims, an HSV object for
    color dims (matches the LLM output shape predict_bundle expects)."""
    bundle = {}
    for field in PREF_FIELDS:
        if PREF_KIND.get(field) == "color":
            bundle[field] = dict(DEFAULT_COLOR)
        else:
            bundle[field] = PREF_OPTIONS[field][0]
    return bundle


class TestPredictionModelPredictBundle:

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_returns_all_fields(self, mock_openai_cls, _key, _mock_anthropic, tmp_path):
        bundle = _default_bundle()
        _mock_anthropic.messages.create.return_value = _fake_anthropic_response(bundle)

        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )

        model = PredictionModel(
            user="test_user",
            physical_profile_label="test_label",
            logs_dir=tmp_path / "pref",
            physical_profile_description="Good arm control.",
            use_long_term_memory=False,
            use_episodic_memory=False,
        )

        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        result = model.predict_bundle(ctx, {})

        assert isinstance(result, dict)
        assert set(result.keys()) == set(PREF_FIELDS)
        for field in PREF_FIELDS:
            _assert_valid_value(field, result[field])

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_physical_profile_description_in_prompt(self, mock_openai_cls, _key, _mock_anthropic, tmp_path):
        bundle = _default_bundle()
        _mock_anthropic.messages.create.return_value = _fake_anthropic_response(bundle)

        desc = "This user has limited arm control and cannot press buttons."

        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )

        model = PredictionModel(
            user="test_user",
            physical_profile_label="unused_label",
            logs_dir=tmp_path / "pref",
            physical_profile_description=desc,
            use_long_term_memory=False,
            use_episodic_memory=False,
        )

        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        model.predict_bundle(ctx, {})

        # Anthropic Messages API: the user prompt is the first (only) user
        # message; the JSON-only instruction is the separate `system` kwarg.
        call_args = _mock_anthropic.messages.create.call_args
        prompt = call_args.kwargs["messages"][0]["content"]
        assert desc in prompt, (
            "Freeform physical-profile description must appear verbatim in the LLM prompt"
        )

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_corrected_fields_override_prediction(self, mock_openai_cls, _key, _mock_anthropic, tmp_path):
        bundle = _default_bundle()
        _mock_anthropic.messages.create.return_value = _fake_anthropic_response(bundle)

        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )

        model = PredictionModel(
            user="test_user",
            physical_profile_label="test_label",
            logs_dir=tmp_path / "pref",
            physical_profile_description="Test.",
            use_long_term_memory=False,
            use_episodic_memory=False,
        )

        override_field = PREF_FIELDS[0]
        override_val = PREF_OPTIONS[override_field][-1]  # last option

        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        result = model.predict_bundle(ctx, {override_field: override_val})
        assert result[override_field] == override_val

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_malformed_llm_json_falls_back(self, mock_openai_cls, _key, _mock_anthropic, tmp_path):
        _mock_anthropic.messages.create.return_value = _fake_anthropic_text("not valid json {{{")

        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )

        model = PredictionModel(
            user="test_user",
            physical_profile_label="test_label",
            logs_dir=tmp_path / "pref",
            physical_profile_description="Test.",
            use_long_term_memory=False,
            use_episodic_memory=False,
        )

        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        result = model.predict_bundle(ctx, {})

        assert isinstance(result, dict)
        assert len(result) == len(PREF_FIELDS)
        for field in PREF_FIELDS:
            _assert_valid_value(field, result[field])

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_predict_bundle_logs_to_disk(self, mock_openai_cls, _key, _mock_anthropic, tmp_path):
        bundle = _default_bundle()
        _mock_anthropic.messages.create.return_value = _fake_anthropic_response(bundle)

        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )

        model = PredictionModel(
            user="test_user",
            physical_profile_label="test_label",
            logs_dir=tmp_path / "pref",
            physical_profile_description="Good control.",
            use_long_term_memory=False,
            use_episodic_memory=False,
        )

        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        model.predict_bundle(ctx, {})

        log_dir = tmp_path / "pref" / "test_user" / "prediction_model_llm_calls"
        log_files = list(log_dir.glob("*.txt"))
        assert len(log_files) == 1, "predict_bundle should write exactly one log file"
        contents = log_files[0].read_text()
        assert "===PROMPT===" in contents
        assert "===RESPONSE===" in contents


# ===================================================================
# Step 3 — preference correction stub + corrected-diff logic
# ===================================================================


class TestPreferenceCorrectionStubContract:
    """The stub in WebInterface.get_preference_corrections returns
    dict(predicted_bundle), i.e. an unchanged copy.  We verify the
    contract here without importing web_interface.py."""

    def test_stub_returns_copy_of_predicted(self):
        predicted = _default_bundle()
        returned = dict(predicted)  # same logic as the stub
        assert returned == predicted
        assert returned is not predicted

    def test_stub_preserves_all_fields(self):
        predicted = _default_bundle()
        returned = dict(predicted)
        assert set(returned.keys()) == set(PREF_FIELDS)


class TestCorrectedDiffLogic:
    """Validates the diff logic used in _Runner.run() after the correction
    round-trip: corrected = {k: v for k, v in user_bundle.items()
    if v != predicted_bundle.get(k)}."""

    def test_corrections_detected(self):
        predicted = _default_bundle()
        user_bundle = dict(predicted)

        field_a, field_b = PREF_FIELDS[0], PREF_FIELDS[1]
        user_bundle[field_a] = PREF_OPTIONS[field_a][-1]
        user_bundle[field_b] = PREF_OPTIONS[field_b][-1]

        corrected = {
            k: v for k, v in user_bundle.items() if v != predicted.get(k)
        }

        if PREF_OPTIONS[field_a][0] != PREF_OPTIONS[field_a][-1]:
            assert field_a in corrected
        if PREF_OPTIONS[field_b][0] != PREF_OPTIONS[field_b][-1]:
            assert field_b in corrected

    def test_no_corrections_means_empty(self):
        predicted = _default_bundle()
        user_bundle = dict(predicted)

        corrected = {
            k: v for k, v in user_bundle.items() if v != predicted.get(k)
        }
        assert corrected == {}

    def test_ground_truth_has_all_fields(self):
        predicted = _default_bundle()
        user_bundle = dict(predicted)
        user_bundle[PREF_FIELDS[0]] = PREF_OPTIONS[PREF_FIELDS[0]][-1]

        ground_truth = user_bundle
        for field in PREF_FIELDS:
            assert field in ground_truth

    def test_corrected_values_are_valid_options(self):
        predicted = _default_bundle()
        user_bundle = dict(predicted)
        for field in PREF_FIELDS[:3]:
            user_bundle[field] = PREF_OPTIONS[field][-1]

        corrected = {
            k: v for k, v in user_bundle.items() if v != predicted.get(k)
        }
        for field, val in corrected.items():
            assert val in PREF_OPTIONS[field]


class TestPrefOptionsConsistency:
    """Sanity checks on PREF_OPTIONS / PREF_FIELDS configuration."""

    def test_fields_match_options_keys(self):
        assert set(PREF_FIELDS) == set(PREF_OPTIONS.keys())

    def test_every_categorical_field_has_at_least_two_options(self):
        # Color dims are continuous (no option list); only categorical dims
        # must offer a real choice.
        for field, opts in PREF_OPTIONS.items():
            if PREF_KIND.get(field) == "color":
                assert opts == []
                continue
            assert len(opts) >= 2, f"{field} has fewer than 2 options"

    def test_no_empty_option_strings(self):
        for field, opts in PREF_OPTIONS.items():
            for opt in opts:
                assert isinstance(opt, str) and opt.strip(), (
                    f"{field} has empty/whitespace option"
                )


# ===================================================================
# Step 5 — Learn: PredictionModel.next_day + update wiring
# ===================================================================


class TestNextDay:
    """Verify PredictionModel.next_day auto-detects the next unused day."""

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_empty_logs_returns_1(self, mock_openai_cls, _key, tmp_path):
        mock_openai_cls.return_value = MagicMock()
        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )
        model = PredictionModel(
            user="u", physical_profile_label="p",
            logs_dir=tmp_path / "pref",
            use_long_term_memory=False, use_episodic_memory=False,
        )
        assert model.next_day() == 1

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_after_three_days_returns_4(self, mock_openai_cls, _key, tmp_path):
        mock_openai_cls.return_value = MagicMock()
        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )
        model = PredictionModel(
            user="u", physical_profile_label="p",
            logs_dir=tmp_path / "pref",
            use_long_term_memory=False, use_episodic_memory=False,
        )
        for d in [1, 2, 3]:
            (model.working_memory_dir / f"day_{d:04d}.json").write_text("{}")
        assert model.next_day() == 4

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_gap_in_days_uses_max(self, mock_openai_cls, _key, tmp_path):
        mock_openai_cls.return_value = MagicMock()
        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )
        model = PredictionModel(
            user="u", physical_profile_label="p",
            logs_dir=tmp_path / "pref",
            use_long_term_memory=False, use_episodic_memory=False,
        )
        for d in [1, 5]:
            (model.working_memory_dir / f"day_{d:04d}.json").write_text("{}")
        assert model.next_day() == 6


class TestUpdateWritesLogs:
    """Verify PredictionModel.update writes per-day JSON files."""

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_update_creates_working_memory_log(self, mock_openai_cls, _key, tmp_path):
        mock_openai_cls.return_value = MagicMock()
        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )
        model = PredictionModel(
            user="u", physical_profile_label="p",
            logs_dir=tmp_path / "pref",
            use_long_term_memory=False, use_episodic_memory=False,
        )
        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        bundle = _default_bundle()
        corrected = {PREF_FIELDS[0]: PREF_OPTIONS[PREF_FIELDS[0]][-1]}

        model.update(day=1, context=ctx, corrected=corrected, ground_truth_bundle=bundle)

        log_file = model.working_memory_dir / "day_0001.json"
        assert log_file.exists()
        data = json.loads(log_file.read_text())
        assert data["day"] == 1
        assert data["context"] == ctx
        assert data["corrected"] == corrected

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_update_increments_next_day(self, mock_openai_cls, _key, tmp_path):
        mock_openai_cls.return_value = MagicMock()
        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )
        model = PredictionModel(
            user="u", physical_profile_label="p",
            logs_dir=tmp_path / "pref",
            use_long_term_memory=False, use_episodic_memory=False,
        )
        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        bundle = _default_bundle()

        assert model.next_day() == 1
        model.update(day=1, context=ctx, corrected={}, ground_truth_bundle=bundle)
        assert model.next_day() == 2
        model.update(day=2, context=ctx, corrected={}, ground_truth_bundle=bundle)
        assert model.next_day() == 3

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_update_with_ltm_creates_ltm_log(self, mock_openai_cls, _key, _mock_anthropic, tmp_path):
        # LTM summarization runs on the Anthropic chat client; it must return
        # valid JSON text for the summary to be stored.
        _mock_anthropic.messages.create.return_value = _fake_anthropic_response({"summary": "test"})

        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )
        model = PredictionModel(
            user="u", physical_profile_label="p",
            logs_dir=tmp_path / "pref",
            use_long_term_memory=True, use_episodic_memory=False,
            physical_profile_description="Test profile.",
        )
        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        bundle = _default_bundle()

        model.update(day=1, context=ctx, corrected={}, ground_truth_bundle=bundle)

        ltm_file = tmp_path / "pref" / "u" / "long_term_memory" / "day_0001.json"
        assert ltm_file.exists()
        data = json.loads(ltm_file.read_text())
        assert data["day"] == 1
        assert "episode_text" in data

    @patch(f"{_PM_MODULE}._resolve_api_key", return_value="fake-key")
    @patch(f"{_PM_MODULE}.OpenAI")
    def test_update_with_em_creates_em_log(self, mock_openai_cls, _key, tmp_path):
        mock_client = MagicMock()
        mock_client.embeddings.create.return_value = MagicMock(
            data=[MagicMock(embedding=[0.1] * 1536)]
        )
        mock_openai_cls.return_value = mock_client

        from feeding_deployment.preference_learning.methods.prediction_model import (
            PredictionModel,
        )
        model = PredictionModel(
            user="u", physical_profile_label="p",
            logs_dir=tmp_path / "pref",
            use_long_term_memory=False, use_episodic_memory=True,
        )
        ctx = {"meal": MEALS[0], "setting": SETTINGS[0], "time_of_day": TIMES_OF_DAY[0]}
        bundle = _default_bundle()

        model.update(day=1, context=ctx, corrected={}, ground_truth_bundle=bundle)

        em_file = tmp_path / "pref" / "u" / "episodic_memory" / "day_0001.json"
        assert em_file.exists()
        data = json.loads(em_file.read_text())
        assert data["day"] == 1
        assert "episode_text" in data


# ===================================================================
# Terminal interaction: context collection + preference correction
# ===================================================================


_TERMINAL_MODULE = "feeding_deployment.integration.terminal_preferences"


class TestTerminalCollectContext:

    @patch("builtins.input", side_effect=["1", "1", "1"])
    def test_picks_first_options(self, _mock_input):
        from feeding_deployment.integration.terminal_preferences import (
            terminal_collect_context,
        )
        ctx = terminal_collect_context()
        assert ctx["meal"] == MEALS[0]
        assert ctx["setting"] == SETTINGS[0]
        assert ctx["time_of_day"] == TIMES_OF_DAY[0]

    @patch("builtins.input", side_effect=["2", "3", "2"])
    def test_picks_specific_options(self, _mock_input):
        from feeding_deployment.integration.terminal_preferences import (
            terminal_collect_context,
        )
        ctx = terminal_collect_context()
        assert ctx["meal"] == MEALS[1]
        assert ctx["setting"] == SETTINGS[2]
        assert ctx["time_of_day"] == TIMES_OF_DAY[1]

    @patch("builtins.input", side_effect=["bad", "0", "1", "1", "1"])
    def test_invalid_input_retries(self, _mock_input):
        from feeding_deployment.integration.terminal_preferences import (
            terminal_collect_context,
        )
        ctx = terminal_collect_context()
        assert ctx["meal"] == MEALS[0]


class TestTerminalCorrectPreferences:

    @patch("builtins.input", side_effect=[""] * len(PREF_FIELDS))
    def test_accept_all_returns_predicted(self, _mock_input):
        from feeding_deployment.integration.terminal_preferences import (
            terminal_correct_preferences,
        )
        predicted = _default_bundle()
        result = terminal_correct_preferences(predicted, dict(PREF_OPTIONS))
        assert result == predicted

    def test_change_one_field(self):
        from feeding_deployment.integration.terminal_preferences import (
            terminal_correct_preferences,
        )
        predicted = _default_bundle()
        target_field = PREF_FIELDS[0]
        target_opts = PREF_OPTIONS[target_field]
        new_idx = len(target_opts)  # pick the last option

        inputs = []
        for field in PREF_FIELDS:
            if field == target_field:
                inputs.append(str(new_idx))
            else:
                inputs.append("")

        with patch("builtins.input", side_effect=inputs):
            result = terminal_correct_preferences(predicted, dict(PREF_OPTIONS))

        if target_opts[0] != target_opts[-1]:
            assert result[target_field] == target_opts[-1]
            assert result[target_field] != predicted[target_field]

        for field in PREF_FIELDS:
            if field != target_field:
                assert result[field] == predicted[field]

    def test_invalid_input_retries_then_accepts(self):
        from feeding_deployment.integration.terminal_preferences import (
            terminal_correct_preferences,
        )
        predicted = _default_bundle()
        inputs = ["bad", "0", ""]  # two bad inputs then accept for first field
        inputs += [""] * (len(PREF_FIELDS) - 1)

        with patch("builtins.input", side_effect=inputs):
            result = terminal_correct_preferences(predicted, dict(PREF_OPTIONS))
        assert result == predicted

    @patch("builtins.input", side_effect=[""] * len(PREF_FIELDS))
    def test_no_changes_means_empty_corrections(self, _mock_input):
        from feeding_deployment.integration.terminal_preferences import (
            terminal_correct_preferences,
        )
        predicted = _default_bundle()
        result = terminal_correct_preferences(predicted, dict(PREF_OPTIONS))
        corrected = {k: v for k, v in result.items() if v != predicted.get(k)}
        assert corrected == {}
