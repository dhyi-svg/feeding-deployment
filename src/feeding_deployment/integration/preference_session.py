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

Threading: repredictions triggered by corrections run on a single coalescing
BACKGROUND worker so the robot is not stationary during the LLM call. Every
consumer of predictions joins first -- ``ask()`` before showing each step,
``record_color``/``record_nav_offset`` on entry, ``finalize_meal`` on entry,
and run.py via ``wait_for_reprediction()`` before executing any skill whose
behavior tree reads prediction-produced parameters (see
``bt_consumes_predictions``). All BT-YAML writers serialize on a dedicated
mutex (acquired BEFORE the session lock, always in that order) so concurrent
appliers can never interleave lost-update file writes.
"""

from __future__ import annotations

import threading
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from feeding_deployment.preference_learning.config.preference_bundle import (
    COLOR_FIELDS,
    COLOR_FIELD_BY_LOCATION,
    DEFAULT_COLOR,
    DEFAULT_NAV_OFFSET,
    NAV_OFFSET_BOUNDS,
    NAV_OFFSET_FIELDS,
    OFFSET_FIELD_BY_LOCATION,
    PREFERENCE_BUNDLE,
    TEXT_FIELDS,
    color_from_bt,
    color_to_bt,
    format_color,
    format_nav_offset,
    nav_offset_from_bt,
    nav_offset_to_bt,
    nav_offsets_equal,
    parse_color,
    parse_nav_offset,
)
from feeding_deployment.preference_learning.config.mealtime_context import (
    food_items_for_flair,
)
from feeding_deployment.preference_learning.methods.prediction_model import (
    PREF_DESCRIPTIONS,
    PREF_KIND,
    PREF_OPTIONS,
)
from feeding_deployment.preference_learning.methods.utils import PREF_FIELDS

_PREF_LABELS: Dict[str, str] = {dim.field: dim.label for dim in PREFERENCE_BUNDLE}

from feeding_deployment.integration.apply_preferences import (
    _load_yaml,
    _save_yaml,
    _set_param_value,
    apply_bite_ordering,
    apply_bundle_to_behavior_trees,
    apply_dip_preference,
    apply_microwave_preference,
    apply_transfer_mode,
)

_COLOR_FIELD_SET = set(COLOR_FIELDS)
_TEXT_FIELD_SET = set(TEXT_FIELDS)
_NAV_OFFSET_FIELD_SET = set(NAV_OFFSET_FIELDS)

# Default autocontinue (seconds) for the correction page before the user's
# wait_before_autocontinue_seconds preference has been finalized.
_DEFAULT_AUTOCONTINUE_SECONDS = 10.0

# ---------------------------------------------------------------------------
# Staged ask schedule + deployment defaults, shared by run.py and the terminal
# emulator (emulate_preference_pipeline.py) so the staged flow has a single
# source of truth.
# ---------------------------------------------------------------------------

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
INITIAL_PREF_DIMS = ["robot_speed", "wait_before_autocontinue_seconds"]

# Behavior trees whose parameters come from (re)prediction: plate pickups read
# HandleColor/ColorRange, navigations read PositionOffset, and the feeding
# skills read the table dims. run.py joins the background reprediction before
# executing these; every other skill only reads dims that are finalized before
# it can run (Speed from the initial ask, MicrowaveDuration from the locked
# microwave ask), which repredictions never touch.
_PREDICTION_CONSUMING_BT_PREFIXES = (
    "pick_plate_from_",
    "navigate_to_",
    "transfer_",
    "acquire_bite",
)


def bt_consumes_predictions(bt_name: str) -> bool:
    """True if the skill's behavior tree reads parameters that a pending
    background reprediction may still be about to (re)write."""
    return str(bt_name).startswith(_PREDICTION_CONSUMING_BT_PREFIXES)


# Preference dimensions asked at the table, just before feeding begins.
TABLE_PREF_DIMS = [
    "skewering_axis",
    "web_interface_confirmation",
    "bite_dipping_preference",
    "bite_ordering",
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


def _pickup_yaml_name(location: str) -> str:
    return f"pick_plate_from_{location}.yaml"


def _nav_yaml_name(location: str) -> str:
    return f"navigate_to_{location}.yaml"


# Full parameter block for upserting PositionOffset into a per-user navigate
# BT YAML that predates the parameter (per-user trees are copied from factory
# only for NEW users, so existing deployments never pick it up otherwise).
# Must match the factory navigate_to_*.yaml definition.
_NAV_OFFSET_PARAM = {
    "name": "PositionOffset",
    "description": (
        "Learned SE(2) offset (dx m, dy m, dyaw rad) applied to the nominal "
        "goal pose in the goal's local frame, accumulated from the user's "
        "post-arrival position corrections."
    ),
    "space": {
        "type": "Box",
        "lower": [-NAV_OFFSET_BOUNDS["dx"], -NAV_OFFSET_BOUNDS["dy"], -NAV_OFFSET_BOUNDS["dyaw"]],
        "upper": [NAV_OFFSET_BOUNDS["dx"], NAV_OFFSET_BOUNDS["dy"], NAV_OFFSET_BOUNDS["dyaw"]],
    },
    "is_user_editable": True,
}


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
        # Per-open-dim reasons + latent-factor inference from the most recent
        # (re)prediction.
        self.last_explanations: Dict[str, str] = {}
        self.last_latent_inference: str = ""

        # Guards the in-memory bundle/finalized/corrected against the settings-edit
        # path (a separate WebInterface worker thread calling settings_view()/edit())
        # racing the main pipeline thread (ask/record_color/finalize_meal). Reentrant
        # so the public methods can call each other. The slow LLM predict_bundle call
        # is deliberately done OUTSIDE this lock (see edit()).
        self._lock = threading.RLock()
        # Dims that are no longer live-editable from the settings overlay because
        # their effect is already committed for this meal (currently just
        # microwave_time once apply_microwave routes the planner). settings_view()
        # marks these editable=False and edit() ignores them.
        self._locked: set[str] = set()
        # Set by a settings edit that changed transfer_mode (and by every
        # background reprediction apply); the transfer-object re-init
        # (apply_transfer_mode) is deferred to flush_pending_inmemory() on
        # the main thread so it never swaps the transfer under an in-flight motion.
        self._pending_transfer_reinit = False

        # Serializes every BT-YAML writing section (_apply_non_planning and the
        # open-color/nav writers) across the main thread, the settings
        # apply-worker, and the background repredict worker. apply_bundle...
        # does load->modify->atomic-replace per file, so two concurrent appliers
        # could otherwise lose each other's parameter updates (each replaces the
        # whole file from its own stale load). LOCK ORDER: this mutex is always
        # acquired BEFORE self._lock, never the other way around.
        self._bt_write_mutex = threading.Lock()

        # Background reprediction: one coalescing worker at a time. A trigger
        # while the worker is running sets ``dirty`` and the worker loops once
        # more (the in-flight prediction was computed without the newest
        # correction; pinning keeps it safe, the extra pass makes it fresh).
        self._repredict_cv = threading.Condition()
        self._repredict_running = False
        self._repredict_dirty = False

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
    # Nav-offset seeds / BT YAML I/O (mirrors the color block above)
    # ------------------------------------------------------------------ #
    def _read_nav_offset_seed(self, field: str) -> Dict[str, Any]:
        """Current saved offset for a nav-offset field from its navigate BT YAML.

        Day 1 this is the factory zero offset copied into the per-user tree;
        later days it is the accumulated total from the user's post-arrival
        adjustments (the tree persists across days). Falls back to
        DEFAULT_NAV_OFFSET if the YAML/param is missing (e.g. a per-user tree
        that predates the parameter).
        """
        location = field.rsplit("_", 1)[-1]  # nav_offset_fridge -> fridge
        fpath = self._bt_dir / _nav_yaml_name(location)
        if not fpath.exists():
            return dict(DEFAULT_NAV_OFFSET)
        data = _load_yaml(fpath)
        value = None
        for param in data.get("parameters", []):
            if param.get("name") == "PositionOffset":
                value = param.get("value")
        return nav_offset_from_bt(value)

    def _nav_offset_seeds(self) -> Dict[str, Any]:
        return {f: self._read_nav_offset_seed(f) for f in NAV_OFFSET_FIELDS}

    def _write_nav_offset_to_bt(self, field: str, offset: Dict[str, Any]) -> None:
        """Write a canonical offset into its navigate BT YAML (PositionOffset).

        Unlike colors (whose params shipped in the factory YAMLs from day one),
        a pre-existing per-user tree may lack the parameter entirely -- upsert
        the full block in that case so the value isn't silently dropped."""
        location = field.rsplit("_", 1)[-1]
        fpath = self._bt_dir / _nav_yaml_name(location)
        if not fpath.exists():
            return
        data = _load_yaml(fpath)
        value = nav_offset_to_bt(offset)
        if _set_param_value(data, "PositionOffset", value):
            _save_yaml(fpath, data)
        else:
            data.setdefault("parameters", []).append({**_NAV_OFFSET_PARAM, "value": value})
            _save_yaml(fpath, data)

    def _write_open_nav_offsets_to_bt(self) -> None:
        """Push current predictions for still-open nav-offset dims into their
        BT YAML so the next navigation uses the latest prediction. Finalized
        offsets are left as-is (they already hold the user's ground truth)."""
        for field in NAV_OFFSET_FIELDS:
            if field in self.finalized:
                continue
            offset = self.bundle.get(field)
            if isinstance(offset, dict):
                self._write_nav_offset_to_bt(field, offset)

    # ------------------------------------------------------------------ #
    # Prediction
    # ------------------------------------------------------------------ #
    def _pinned_split(self) -> tuple[Dict[str, Any], Dict[str, Any]]:
        """Finalized values split into (corrected, confirmed), both pinned during
        (re)prediction so they never flip. Corrected = dims the user actively
        changed this meal; confirmed = dims they accepted as-predicted."""
        with self._lock:
            corrected = {
                f: self.bundle[f]
                for f in self.finalized
                if f in self.corrected and f in self.bundle
            }
            confirmed = {
                f: self.bundle[f]
                for f in self.finalized
                if f not in self.corrected and f in self.bundle
            }
        return corrected, confirmed

    def _predict(
        self,
        color_seeds: Optional[Dict[str, Any]] = None,
        nav_offset_seeds: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        # predict_bundle may call an LLM (slow). Callers must NOT hold self._lock
        # across this (see _repredict_open) so the settings worker / execution
        # thread are never blocked on the network. Callers may pass pre-captured
        # seeds so they can later detect external YAML writes that landed while
        # the LLM call was in flight.
        corrected, confirmed = self._pinned_split()
        pred = self._model.predict_bundle(
            self.context,
            corrected,
            confirmed=confirmed,
            color_seeds=color_seeds if color_seeds is not None else self._color_seeds(),
            nav_offset_seeds=nav_offset_seeds if nav_offset_seeds is not None else self._nav_offset_seeds(),
        )
        # Per-open-dim reasons + latent-factor inference from this prediction
        # (logged at start(); shown by the terminal emulator). Best-effort:
        # absent on models without them.
        self.last_explanations = dict(getattr(self._model, "last_explanations", {}) or {})
        self.last_latent_inference = str(getattr(self._model, "last_latent_inference", "") or "")
        return pred

    def _repredict_open(self) -> None:
        """Refresh predictions for all still-open dims (finalized dims pinned).

        Runs on the background repredict worker (normal path) or on whatever
        thread calls it directly; the LLM call happens with NO locks held, the
        bundle/YAML update takes the BT-write mutex before the session lock.

        External writes win: a color/nav value someone else wrote to the YAML
        while the LLM call was in flight (the pickup color picker, the
        post-arrival teleop write-back) is FRESHER than this prediction --
        detect it by comparing the YAML against the seed this prediction was
        computed from, and leave both the YAML and the bundle value alone (the
        following record_* finalizes the external value and repredicts again)."""
        color_seeds = self._color_seeds()
        nav_offset_seeds = self._nav_offset_seeds()
        pred = self._predict(color_seeds, nav_offset_seeds)  # LLM OUTSIDE all locks
        with self._bt_write_mutex:
            with self._lock:
                stale: set[str] = set()
                for field in COLOR_FIELDS:
                    if field not in self.finalized and self._read_color_seed(field) != color_seeds[field]:
                        stale.add(field)
                for field in NAV_OFFSET_FIELDS:
                    if field not in self.finalized and not nav_offsets_equal(
                        self._read_nav_offset_seed(field), nav_offset_seeds[field]
                    ):
                        stale.add(field)
                if stale:
                    print(
                        "[preference-session] external write landed during "
                        f"reprediction; keeping it for: {sorted(stale)}"
                    )
                for field in PREF_FIELDS:
                    if field in self.finalized or field in stale:
                        continue
                    if field in pred:
                        self.bundle[field] = pred[field]
                for field in COLOR_FIELDS:
                    if field in self.finalized or field in stale:
                        continue
                    color = self.bundle.get(field)
                    if isinstance(color, dict):
                        self._write_color_to_bt(field, color)
                for field in NAV_OFFSET_FIELDS:
                    if field in self.finalized or field in stale:
                        continue
                    offset = self.bundle.get(field)
                    if isinstance(offset, dict):
                        self._write_nav_offset_to_bt(field, offset)

    # ------------------------------------------------------------------ #
    # Background reprediction worker
    # ------------------------------------------------------------------ #
    def _schedule_repredict(self) -> None:
        """Queue a background reprediction (repredict open dims + re-apply).

        Coalescing: at most one worker runs at a time; a trigger while one is
        in flight marks the state dirty and the worker runs one more pass with
        the newest corrections before exiting. Callers return immediately --
        consumers synchronize via ``wait_for_reprediction``."""
        with self._repredict_cv:
            self._repredict_dirty = True
            if not self._repredict_running:
                self._repredict_running = True
                threading.Thread(
                    target=self._repredict_worker,
                    name="pref-repredict",
                    daemon=True,
                ).start()

    def _repredict_worker(self) -> None:
        while True:
            with self._repredict_cv:
                if not self._repredict_dirty:
                    self._repredict_running = False
                    self._repredict_cv.notify_all()
                    return
                self._repredict_dirty = False
            try:
                self._repredict_open()
                # Apply repredicted categorical values to the BTs + FLAIR. The
                # transfer-object reconstruction must happen on the MAIN thread
                # (never under an in-flight motion), so defer it exactly like
                # the settings-edit path does: flush_pending_inmemory() runs it
                # at the next skill boundary.
                self._apply_non_planning(reinit_transfer=False)
                with self._lock:
                    self._pending_transfer_reinit = True
            except Exception as e:  # a failed repredict must never wedge joiners
                # Previous predictions stay applied; the next correction (or
                # this loop's dirty pass) retries with a fresh LLM call.
                print(f"[preference-session] background reprediction failed: {e}")

    def wait_for_reprediction(self, timeout: Optional[float] = None) -> bool:
        """Block until no background reprediction is running or queued.

        The join barrier for every prediction consumer: ask() steps, the
        record_* entry points, finalize_meal, the terminal emulator's
        explanation printer, and run.py before prediction-consuming skills.
        Returns False only if ``timeout`` expired first. Callers must not hold
        the session lock or the BT-write mutex."""
        with self._repredict_cv:
            return self._repredict_cv.wait_for(
                lambda: not self._repredict_running and not self._repredict_dirty,
                timeout,
            )

    # ------------------------------------------------------------------ #
    # Apply
    # ------------------------------------------------------------------ #
    def _apply_non_planning(self, *, reinit_transfer: bool = True) -> List[str]:
        """Apply all BT-parameter dims + (optionally) transfer mode + dip from the
        current bundle. Idempotent and free of planner side effects (microwave atom is
        applied separately via apply_microwave).

        ``reinit_transfer=False`` skips the transfer-object reconstruction
        (apply_transfer_mode); the settings-edit worker and the background
        repredict worker pass this and defer that reconstruction to
        flush_pending_inmemory() on the main thread so the transfer object is
        never swapped under an in-flight transfer motion."""
        with self._bt_write_mutex:
            bundle = self._categorical_bundle()  # single consistent snapshot
            warnings = apply_bundle_to_behavior_trees(bundle, self._bt_dir)
            if reinit_transfer and self._scene is not None:
                apply_transfer_mode(bundle, self._scene, self._hla_map)
            apply_dip_preference(bundle, self._flair)
            apply_bite_ordering(bundle, self._flair)
            return warnings

    def _categorical_bundle(self) -> Dict[str, str]:
        """Bundle restricted to categorical (string) fields, for the apply_*
        helpers (which expect string values)."""
        with self._lock:
            return {
                f: v
                for f, v in self.bundle.items()
                if f not in _COLOR_FIELD_SET and isinstance(v, str)
            }

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #
    def _apply_food_items(self) -> None:
        """Derive FLAIR's food items (solids/dips) from the chosen meal and set
        them on FLAIR. Deterministic (no LLM); replaces the old meal_setup
        food-item entry. No-op when there is no FLAIR (unit tests / replay)."""
        if self._flair is None:
            return
        meal = str(self.context.get("meal", ""))
        food_items = food_items_for_flair(meal)  # raises KeyError if not in catalog
        if not food_items["solid"]:
            raise ValueError(
                f"Meal {meal!r} has no solid food items to give FLAIR for detection."
            )
        self._flair.set_food_items(food_items)

    def _clean_text_correction(self, field: str, text: str) -> str:
        """Best-effort grammar/grounding cleanup of a free-text correction using
        FLAIR's meal parser (the same parser the old meal_setup used). Returns the
        cleaned string, or the raw text on any failure -- the user's input is
        never dropped."""
        if field != "bite_ordering" or not text or self._flair is None:
            return text
        parser = getattr(self._flair, "new_meal_parser", None)
        if parser is None:
            return text
        try:
            food_items = food_items_for_flair(str(self.context.get("meal", "")))
            food_str = ", ".join(food_items["solid"] + food_items["dip"])
            _solids, _dips, cleaned = parser.parse_user_message(food_str, text)
            return cleaned or text
        except Exception as e:  # cleanup must never break the correction flow
            print(f"[preference-session] bite-ordering cleanup failed: {e}")
            return text

    def start(self) -> None:
        """Predict the full bundle, seed/write colors + nav offsets, apply
        non-planning dims. Synchronous: nothing can overlap the very first
        prediction, and the initial ask needs it."""
        self._apply_food_items()
        with self._lock:
            finalized_before = set(self.finalized)
        pred = self._predict()
        with self._bt_write_mutex:
            with self._lock:
                # Merge rather than replace: a settings edit racing start()'s
                # LLM call may have finalized a dim already -- never clobber a
                # finalized value with the (pre-edit) prediction.
                for field in PREF_FIELDS:
                    if field in self.finalized:
                        continue
                    if field in pred:
                        self.bundle[field] = pred[field]
                self._write_open_colors_to_bt()
                self._write_open_nav_offsets_to_bt()
                finalized_during = self.finalized - finalized_before
        warnings = self._apply_non_planning()
        for w in warnings:
            print(f"[preference-apply] WARNING: {w}")
        if finalized_during:
            # A settings edit landed while the initial LLM call was in flight.
            # Its own scheduled repredict pass may already have finished, in
            # which case the merge above just overwrote the open dims with
            # predictions NOT conditioned on that edit -- run one more pass.
            self._schedule_repredict()
        self._log(
            "preference_predicted",
            stage="start",
            predicted_bundle=self._loggable_bundle(),
            explanations=dict(self.last_explanations),
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
        the pickup color picker); nav-offset dims are NOT asked here either
        (they are corrected physically via the post-arrival teleop
        adjustment)."""
        dims = [d for d in dims if PREF_KIND.get(d) not in ("color", "nav_offset")]
        # Resume: dims already locked as ground truth (asked/corrected before a
        # crash) are not re-asked. This also lets a partially-completed ask()
        # batch resume on exactly the still-open dims.
        dims = [d for d in dims if d not in self.finalized]
        if not dims:
            return

        # No web interface (e.g. unit tests): treat every prediction as confirmed.
        if self._web is None or not hasattr(self._web, "send_preference_step"):
            self.wait_for_reprediction()
            for field in dims:
                self._finalize(field, self.bundle.get(field), changed=False)
            self._apply_non_planning()
            return

        total = len(dims)
        self._web.start_preference_correction(total, self.wait_seconds)
        try:
            for step, field in enumerate(dims):
                # Join any in-flight background reprediction before reading the
                # prediction for this step, so the page always shows values
                # conditioned on every correction made so far (a mid-batch
                # correction therefore waits here, exactly like the old
                # synchronous flow; the LAST correction of a batch overlaps the
                # following robot motion instead).
                self.wait_for_reprediction()
                predicted = self.bundle.get(field)
                user_value = self._web.send_preference_step(
                    field=field,
                    predicted=predicted,
                    options=list(PREF_OPTIONS.get(field, [])),
                    step=step,
                    total=total,
                    autocontinue_seconds=self.wait_seconds,
                    kind=PREF_KIND.get(field, "categorical"),
                )
                if user_value is None:
                    user_value = predicted
                # A free-text "Other..." correction is cleaned for grammar /
                # grounding (raw text kept on any failure -- never dropped).
                if PREF_KIND.get(field) == "text" and user_value != predicted:
                    user_value = self._clean_text_correction(field, user_value)
                changed = user_value != predicted
                self._finalize(field, user_value, changed=changed)
                if changed:
                    # The correction itself is locked synchronously (above);
                    # refreshing the open dims happens in the background.
                    self._schedule_repredict()
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
        # The microwave routing is now committed to the planner for this meal, so
        # microwave_time can no longer be honored from the settings overlay without
        # a mid-plan re-plan -> lock it (settings_view marks it editable=False).
        with self._lock:
            self._locked.add("microwave_time")
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

        # Settle any in-flight reprediction first so the observed-vs-predicted
        # comparison runs against the final prediction the pickup actually used
        # (same semantics as the old synchronous flow; by pickup time a prior
        # repredict has long finished, so this join is normally instant).
        self.wait_for_reprediction()

        observed = self._read_color_seed(field)
        predicted = self.bundle.get(field)
        changed = parse_color(predicted) != observed if predicted is not None else False

        self._finalize(field, observed, changed=changed)
        # An edit-triggered worker pass scheduled between our entry join and
        # the finalize above may have overwritten this (then-still-open) YAML
        # value with its prediction; re-assert the observed ground truth. A
        # no-op in the normal case, and any pass starting from here on skips
        # the field (finalized).
        with self._bt_write_mutex:
            self._write_color_to_bt(field, observed)
        if changed:
            # Propagation to the other open color dims happens in the
            # background; the next pickup joins before reading its YAML.
            self._schedule_repredict()

        self._log(
            "preference_color_recorded",
            location=location,
            field=field,
            color=format_color(observed),
            changed=changed,
        )

    def record_nav_offset(self, location: str) -> None:
        """Finalize a nav-offset dim after its navigation executed, reading the
        (possibly teleop-adjusted, accumulated) total offset back from the
        navigate BT YAML.

        Unlike record_color, a location can be navigated to several times in
        one meal, so an already-finalized dim is RE-finalized whenever the
        total changed again -- the latest accumulated total is the ground
        truth for this meal."""
        field = OFFSET_FIELD_BY_LOCATION.get(location)
        if field is None:
            return

        # Same join-first rationale as record_color: compare against the
        # settled prediction the navigation actually drove with.
        self.wait_for_reprediction()

        observed = self._read_nav_offset_seed(field)
        with self._lock:
            previous = self.bundle.get(field)
        changed = previous is not None and not nav_offsets_equal(previous, observed)
        if field in self.finalized and not changed:
            return

        self._finalize(field, observed, changed=changed)
        # Same rationale as record_color: undo a mid-window worker overwrite
        # of this YAML value (no-op in the normal case).
        with self._bt_write_mutex:
            self._write_nav_offset_to_bt(field, observed)
        if changed:
            self._schedule_repredict()

        self._log(
            "preference_nav_offset_recorded",
            location=location,
            field=field,
            offset=format_nav_offset(observed),
            changed=changed,
        )

    # ------------------------------------------------------------------ #
    # Settings overlay: view + edit already-set preferences
    # ------------------------------------------------------------------ #
    def settings_view(self) -> List[Dict[str, Any]]:
        """Snapshot of the finalized, categorical dims for the settings overlay.

        Only dims the user has already set this meal are shown (still-open dims are
        re-predicted from the user's edits, so they need not appear). Color dims are
        excluded (they use the live-camera picker). Locked dims are included but
        marked ``editable=False``. Safe to call from the WebInterface thread."""
        out: List[Dict[str, Any]] = []
        with self._lock:
            for field in PREF_FIELDS:  # stable display order
                # Color dims use the live-camera picker; text dims (e.g.
                # bite_ordering) have no option list to render as chips and are
                # only editable at their ask() step; nav-offset dims are
                # corrected physically via teleop -- exclude all three here.
                if field not in self.finalized or PREF_KIND.get(field) in ("color", "text", "nav_offset"):
                    continue
                out.append({
                    "field": field,
                    "label": _PREF_LABELS.get(field, field.replace("_", " ").title()),
                    "value": self.bundle.get(field),
                    "options": list(PREF_OPTIONS.get(field, [])),
                    "description": PREF_DESCRIPTIONS.get(field, ""),
                    "editable": field not in self._locked,
                })
        return out

    def edit(self, field: str, value: Any) -> bool:
        """Apply a settings-overlay edit to an already-finalized categorical dim.

        Treated as a *correction* to ground truth: updates the bundle, records it in
        ``corrected``, re-predicts the still-open dims against the new value, and
        applies it to the BTs immediately. The transfer-object re-init is deferred to
        flush_pending_inmemory() (main thread). Returns True if applied, False if the
        edit was ignored (color dim, locked, or not a valid option). Called from the
        WebInterface apply-worker thread; the slow LLM re-prediction runs outside the
        session lock."""
        if PREF_KIND.get(field) in ("color", "text", "nav_offset"):
            return False
        with self._lock:
            if field in self._locked:
                return False
            if value not in PREF_OPTIONS.get(field, []):
                return False
            changed = value != self.bundle.get(field)

        # _finalize takes the lock itself (and persists via on_change).
        self._finalize(field, value, changed=changed)
        if changed:
            # Apply the edited value to the BTs + dip immediately (fast, no
            # LLM) so the next skill sees it even before the reprediction
            # lands; defer the transfer-object reconstruction so it never
            # swaps under an in-flight transfer (flush_pending_inmemory).
            self._apply_non_planning(reinit_transfer=False)
            if field == "transfer_mode":
                with self._lock:
                    self._pending_transfer_reinit = True
            # Re-propagate the still-open dims in the background; consumers
            # (ask steps, prediction-consuming skills) join before reading.
            self._schedule_repredict()
        self._log("preference_settings_edit", field=field, value=value, changed=changed)
        return True

    def flush_pending_inmemory(self) -> None:
        """Run deferred in-memory applies on the MAIN thread at a safe boundary
        (run.py calls this just before each execute_action). Currently only the
        transfer-object re-init, deferred from edit() so the transfer object is
        never reconstructed under an in-flight transfer motion."""
        with self._lock:
            pending = self._pending_transfer_reinit
            self._pending_transfer_reinit = False
        if pending and self._scene is not None:
            apply_transfer_mode(self._categorical_bundle(), self._scene, self._hla_map)

    def finalize_meal(self, day: int) -> Dict[str, Any]:
        """Single per-day memory update with the full finalized bundle.

        Any dims never explicitly asked/recorded are finalized now at their
        predicted value (a confirmation = the prediction was right)."""
        # The ground truth must include the final repredicted values for the
        # never-asked open dims -- settle the background worker first.
        self.wait_for_reprediction()
        for field in PREF_FIELDS:
            if field not in self.finalized and field in self.bundle:
                self._finalize(field, self.bundle[field], changed=False)

        with self._lock:
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
        learning update honest.

        Deliberately does NOT join the background reprediction (run.py snapshots
        right after record_* schedules one; joining here would reintroduce the
        post-pickup stall). A crash in that window resumes with the pre-repredict
        open predictions -- safe, merely re-correctable."""
        with self._lock:
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
        self._apply_food_items()
        with self._bt_write_mutex:
            with self._lock:
                self._write_open_colors_to_bt()
                self._write_open_nav_offsets_to_bt()
        self._apply_non_planning()

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #
    def _finalize(self, field: str, value: Any, *, changed: bool) -> None:
        if field is None:
            return
        if field in _COLOR_FIELD_SET:
            value = parse_color(value)
        elif field in _NAV_OFFSET_FIELD_SET:
            value = parse_nav_offset(value)
        with self._lock:
            self.bundle[field] = value
            self.finalized.add(field)
            if changed:
                self.corrected[field] = value
            snapshot = self.capture_state()  # reentrant lock
        # Persist the latest preference state immediately so a crash after this
        # correction (but before the next sub-skill checkpoint) loses nothing.
        # Done OUTSIDE the lock so a slow on_change can't block settings_view / the
        # execution thread. Persistence must never break the meal.
        if self._on_change is not None:
            try:
                self._on_change(snapshot)
            except Exception as e:
                print(f"[preference-session] on_change persist failed: {e}")

    def _loggable_bundle(self, only: Optional[List[str]] = None) -> Dict[str, Any]:
        fields = only if only is not None else list(self.bundle.keys())
        out: Dict[str, Any] = {}
        for f in fields:
            v = self.bundle.get(f)
            if f in _COLOR_FIELD_SET and isinstance(v, dict):
                out[f] = format_color(v)
            elif f in _NAV_OFFSET_FIELD_SET and isinstance(v, dict):
                out[f] = format_nav_offset(v)
            else:
                out[f] = v
        return out

    def _log(self, category: str, **fields: Any) -> None:
        if self._logger is not None:
            try:
                self._logger.log_event(category, context=dict(self.context), **fields)
            except Exception as e:  # logging must never break the meal
                print(f"[preference-session] log_event failed: {e}")
