"""Per-meal personalization session: staged, just-in-time preference handling.

One ``PreferenceSession`` is created per meal. It owns the live preference
bundle and drives the *staged* flow described in the design:

1. ``start(context)``           -- predict the FULL bundle (categorical + color),
                                   seed colors from the per-user behavior-tree
                                   YAML, and apply the (non-planning) dims.
2. ``ask(dims)``                -- show the user the prediction for each dim in
                                   ``dims`` one at a time (stepwise web page).
                                   A *correction* repredicts the still-open dims
                                   (finalized dims stay pinned); a *non-
                                   correction* finalizes that dim as ground
                                   truth (the prediction was right). Each dim is
                                   applied immediately.
3. ``apply_microwave(...)``     -- explicit, correctly-timed FoodHeated mutation
                                   driven by the (already-asked) microwave_time.
4. ``record_color(location)``   -- called after a plate pickup executes. Reads
                                   the (possibly user-corrected) color from the
                                   pickup's BT YAML, finalizes that color dim,
                                   and repredicts the still-open dims so a
                                   correction propagates to the other pickups.
5. ``finalize_meal(day)``       -- exactly ONE memory update with the full
                                   finalized ground-truth bundle.

Reprediction only ever *reads* (``predict_bundle``); memory is written exactly
once, in ``finalize_meal``.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from feeding_deployment.preference_learning.config.preference_bundle import (
    COLOR_FIELDS,
    COLOR_FIELD_BY_LOCATION,
    DEFAULT_COLOR,
    color_from_bt,
    color_to_bt,
    format_color,
    parse_color,
)
from feeding_deployment.preference_learning.methods.prediction_model import (
    PREF_KIND,
    PREF_OPTIONS,
)
from feeding_deployment.preference_learning.methods.utils import PREF_FIELDS

from feeding_deployment.integration.apply_preferences import (
    _load_yaml,
    _save_yaml,
    _set_param_value,
    apply_bundle_to_behavior_trees,
    apply_dip_preference,
    apply_microwave_preference,
    apply_transfer_mode,
)

_COLOR_FIELD_SET = set(COLOR_FIELDS)

# Default autocontinue (seconds) for the correction page before the user's
# wait_before_autocontinue_seconds preference has been finalized.
_DEFAULT_AUTOCONTINUE_SECONDS = 10.0


def _pickup_yaml_name(location: str) -> str:
    return f"pick_plate_from_{location}.yaml"


def _wait_pref_to_seconds(value: Optional[str]) -> float:
    """'10 sec' -> 10.0. Falls back to the default on anything unexpected."""
    if not value:
        return _DEFAULT_AUTOCONTINUE_SECONDS
    try:
        return float(str(value).split()[0])
    except (ValueError, IndexError):
        return _DEFAULT_AUTOCONTINUE_SECONDS


class PreferenceSession:
    def __init__(
        self,
        prediction_model: Any,
        run_behavior_tree_dir: Path,
        context: Dict[str, Any],
        *,
        web_interface: Any = None,
        data_logger: Any = None,
        scene_description: Any = None,
        hla_map: Optional[Dict[str, Any]] = None,
        flair: Any = None,
        on_change: Optional[Callable[[Dict[str, Any]], None]] = None,
    ) -> None:
        self._model = prediction_model
        self._bt_dir = Path(run_behavior_tree_dir)
        self.context = dict(context)
        self._web = web_interface
        self._logger = data_logger
        self._scene = scene_description
        self._hla_map = hla_map or {}
        self._flair = flair
        # Called with capture_state() whenever a correction is locked, so the
        # latest preference state is persisted immediately (see _finalize).
        self._on_change = on_change

        # Live bundle: categorical fields -> str, color fields -> canonical dict.
        self.bundle: Dict[str, Any] = {}
        # Dims locked as ground truth this episode (corrected OR confirmed).
        self.finalized: set[str] = set()
        # Dims the user actively CHANGED (learning signal); subset of finalized.
        self.corrected: Dict[str, Any] = {}

    # ------------------------------------------------------------------ #
    # Color seeds / BT YAML I/O
    # ------------------------------------------------------------------ #
    def _read_color_seed(self, field: str) -> Dict[str, Any]:
        """Current saved color for a color field from its pickup BT YAML.

        Day 1 this is the factory default copied into the per-user tree; later
        days it is the last corrected/confirmed value (the tree persists across
        days). Falls back to DEFAULT_COLOR if the YAML/param is missing.
        """
        location = field.rsplit("_", 1)[-1]  # plate_color_fridge -> fridge
        fpath = self._bt_dir / _pickup_yaml_name(location)
        if not fpath.exists():
            return dict(DEFAULT_COLOR)
        data = _load_yaml(fpath)
        handle_color = None
        color_range = None
        for param in data.get("parameters", []):
            if param.get("name") == "HandleColor":
                handle_color = param.get("value")
            elif param.get("name") == "ColorRange":
                color_range = param.get("value")
        return color_from_bt(handle_color, color_range)

    def _color_seeds(self) -> Dict[str, Any]:
        return {f: self._read_color_seed(f) for f in COLOR_FIELDS}

    def _write_color_to_bt(self, field: str, color: Dict[str, Any]) -> None:
        """Write a canonical color into its pickup BT YAML (HandleColor/ColorRange)."""
        location = field.rsplit("_", 1)[-1]
        fpath = self._bt_dir / _pickup_yaml_name(location)
        if not fpath.exists():
            return
        data = _load_yaml(fpath)
        handle_color, color_range = color_to_bt(color)
        changed = False
        changed |= _set_param_value(data, "HandleColor", handle_color)
        changed |= _set_param_value(data, "ColorRange", color_range)
        if changed:
            _save_yaml(fpath, data)

    def _write_open_colors_to_bt(self) -> None:
        """Push current predictions for still-open color dims into their BT YAML
        so the next pickup uses the latest prediction. Finalized colors are left
        as-is (they already hold the user's ground truth)."""
        for field in COLOR_FIELDS:
            if field in self.finalized:
                continue
            color = self.bundle.get(field)
            if isinstance(color, dict):
                self._write_color_to_bt(field, color)

    # ------------------------------------------------------------------ #
    # Prediction
    # ------------------------------------------------------------------ #
    def _overrides(self) -> Dict[str, Any]:
        """Finalized values, pinned during (re)prediction so they never flip."""
        return {f: self.bundle[f] for f in self.finalized if f in self.bundle}

    def _predict(self) -> Dict[str, Any]:
        return self._model.predict_bundle(
            self.context,
            self._overrides(),
            color_seeds=self._color_seeds(),
        )

    def _repredict_open(self) -> None:
        """Refresh predictions for all still-open dims (finalized dims pinned)."""
        pred = self._predict()
        for field in PREF_FIELDS:
            if field in self.finalized:
                continue
            if field in pred:
                self.bundle[field] = pred[field]
        self._write_open_colors_to_bt()

    # ------------------------------------------------------------------ #
    # Apply
    # ------------------------------------------------------------------ #
    def _apply_non_planning(self) -> List[str]:
        """Apply all BT-parameter dims + transfer mode + dip from the current
        bundle. Idempotent and free of planner side effects (microwave atom is
        applied separately via apply_microwave)."""
        warnings = apply_bundle_to_behavior_trees(self._categorical_bundle(), self._bt_dir)
        if self._scene is not None:
            apply_transfer_mode(self._categorical_bundle(), self._scene, self._hla_map)
        apply_dip_preference(self._categorical_bundle(), self._flair)
        return warnings

    def _categorical_bundle(self) -> Dict[str, str]:
        """Bundle restricted to categorical (string) fields, for the apply_*
        helpers (which expect string values)."""
        return {
            f: v
            for f, v in self.bundle.items()
            if f not in _COLOR_FIELD_SET and isinstance(v, str)
        }

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def start(self) -> None:
        """Predict the full bundle, seed/write colors, apply non-planning dims."""
        self.bundle = self._predict()
        self._write_open_colors_to_bt()
        warnings = self._apply_non_planning()
        for w in warnings:
            print(f"[preference-apply] WARNING: {w}")
        self._log(
            "preference_predicted",
            stage="start",
            predicted_bundle=self._loggable_bundle(),
        )

    @property
    def wait_seconds(self) -> float:
        """Autocontinue timeout for correction pages, from the (possibly
        finalized) wait_before_autocontinue_seconds preference."""
        return _wait_pref_to_seconds(self.bundle.get("wait_before_autocontinue_seconds"))

    def ask(self, dims: List[str]) -> None:
        """Show the prediction for each categorical dim in ``dims`` one at a
        time; lock each as ground truth; repredict still-open dims after any
        correction; apply after each. Color dims are NOT asked here (they use
        the pickup color picker)."""
        dims = [d for d in dims if PREF_KIND.get(d) != "color"]
        # Resume: dims already locked as ground truth (asked/corrected before a
        # crash) are not re-asked. This also lets a partially-completed ask()
        # batch resume on exactly the still-open dims.
        dims = [d for d in dims if d not in self.finalized]
        if not dims:
            return

        # No web interface (e.g. unit tests): treat every prediction as confirmed.
        if self._web is None or not hasattr(self._web, "send_preference_step"):
            for field in dims:
                self._finalize(field, self.bundle.get(field), changed=False)
            self._apply_non_planning()
            return

        total = len(dims)
        self._web.start_preference_correction(total, self.wait_seconds)
        try:
            for step, field in enumerate(dims):
                predicted = self.bundle.get(field)
                user_value = self._web.send_preference_step(
                    field=field,
                    predicted=predicted,
                    options=list(PREF_OPTIONS.get(field, [])),
                    step=step,
                    total=total,
                    autocontinue_seconds=self.wait_seconds,
                )
                if user_value is None:
                    user_value = predicted
                changed = user_value != predicted
                self._finalize(field, user_value, changed=changed)
                if changed:
                    # Reflect the correction immediately and refresh open dims.
                    self.bundle[field] = user_value
                    self._repredict_open()
        finally:
            self._web.finish_preference_correction()

        self._apply_non_planning()
        self._log(
            "preference_asked",
            dims=dims,
            ground_truth=self._loggable_bundle(only=dims),
            corrected=[d for d in dims if d in self.corrected],
        )

    def apply_microwave(self, current_atoms: set, food_heated_atom: Any) -> Optional[int]:
        """Apply the (already-asked) microwave_time to the planner atoms.

        'no microwave' adds FoodHeated (planner skips the microwave detour);
        a duration leaves it unset (planner routes through the microwave) and
        the duration is written to the BT by _apply_non_planning. Returns the
        duration in seconds, or None for 'no microwave'."""
        return apply_microwave_preference(
            self._categorical_bundle(), current_atoms, food_heated_atom
        )

    def record_color(self, location: str) -> None:
        """Finalize a plate-color dim after its pickup executed, reading the
        (possibly user-corrected) color back from the pickup BT YAML. A change
        is fed into reprediction so it propagates to the other (open) color
        dims (same physical plate)."""
        field = COLOR_FIELD_BY_LOCATION.get(location)
        if field is None or field in self.finalized:
            return

        observed = self._read_color_seed(field)
        predicted = self.bundle.get(field)
        changed = parse_color(predicted) != observed if predicted is not None else False

        self._finalize(field, observed, changed=changed)
        self.bundle[field] = observed
        if changed:
            self._repredict_open()
            self._apply_non_planning()

        self._log(
            "preference_color_recorded",
            location=location,
            field=field,
            color=format_color(observed),
            changed=changed,
        )

    def finalize_meal(self, day: int) -> Dict[str, Any]:
        """Single per-day memory update with the full finalized bundle.

        Any dims never explicitly asked/recorded are finalized now at their
        predicted value (a confirmation = the prediction was right)."""
        for field in PREF_FIELDS:
            if field not in self.finalized and field in self.bundle:
                self._finalize(field, self.bundle[field], changed=False)

        ground_truth = dict(self.bundle)
        self._model.update(
            day=day,
            context=self.context,
            corrected=dict(self.corrected),
            ground_truth_bundle=ground_truth,
        )
        self._log(
            "preference_finalized",
            day=day,
            ground_truth_bundle=self._loggable_bundle(),
            corrected=sorted(self.corrected.keys()),
        )
        return ground_truth

    # ------------------------------------------------------------------ #
    # Checkpoint / resume
    # ------------------------------------------------------------------ #
    def capture_state(self) -> Dict[str, Any]:
        """Serializable per-meal session state for checkpointing.

        Only the live bundle and the finalized/corrected bookkeeping are stored
        -- the prediction model is rebuilt separately on resume (it owns LLM /
        memory / disk handles), and color *values* already persist in the pickup
        BT YAMLs. Sufficient to (a) avoid re-asking, and (b) keep the end-of-meal
        learning update honest."""
        return {
            "context": dict(self.context),
            "bundle": dict(self.bundle),
            "finalized": set(self.finalized),
            "corrected": dict(self.corrected),
        }

    def resume_from_state(self, state: Dict[str, Any]) -> None:
        """Re-hydrate per-meal state after a crash and re-apply it to the BTs /
        scene WITHOUT predicting or asking. Open-dim predictions already live in
        the persistent pickup BT YAMLs from the original run; this just makes the
        in-memory bundle and the actuated config consistent again so the meal can
        continue and ``finalize_meal`` reflects the corrections made before the
        crash."""
        self.context = dict(state["context"])
        self.bundle = dict(state["bundle"])
        self.finalized = set(state["finalized"])
        self.corrected = dict(state["corrected"])
        self._write_open_colors_to_bt()
        self._apply_non_planning()

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _finalize(self, field: str, value: Any, *, changed: bool) -> None:
        if field is None:
            return
        if field in _COLOR_FIELD_SET:
            value = parse_color(value)
        self.bundle[field] = value
        self.finalized.add(field)
        if changed:
            self.corrected[field] = value
        # Persist the latest preference state immediately so a crash after this
        # correction (but before the next sub-skill checkpoint) loses nothing.
        if self._on_change is not None:
            try:
                self._on_change(self.capture_state())
            except Exception as e:  # persistence must never break the meal
                print(f"[preference-session] on_change persist failed: {e}")

    def _loggable_bundle(self, only: Optional[List[str]] = None) -> Dict[str, Any]:
        fields = only if only is not None else list(self.bundle.keys())
        out: Dict[str, Any] = {}
        for f in fields:
            v = self.bundle.get(f)
            out[f] = format_color(v) if f in _COLOR_FIELD_SET and isinstance(v, dict) else v
        return out

    def _log(self, category: str, **fields: Any) -> None:
        if self._logger is not None:
            try:
                self._logger.log_event(category, context=dict(self.context), **fields)
            except Exception as e:  # logging must never break the meal
                print(f"[preference-session] log_event failed: {e}")
