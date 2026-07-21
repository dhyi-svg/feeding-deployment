from __future__ import annotations

import json
from typing import Any, Dict, List, Optional
from pathlib import Path
from datetime import datetime

from openai import OpenAI
import anthropic

import feeding_deployment.preference_learning.config as root_config  # type: ignore
from feeding_deployment.preference_learning.config.preference_bundle import (
    PREFERENCE_BUNDLE as _PREF_BUNDLE_DIMS,
    COLOR_FIELDS,
    TEXT_FIELDS,
    NAV_OFFSET_FIELDS,
    DEFAULT_COLOR,
    DEFAULT_NAV_OFFSET,
    DEFAULT_BITE_ORDERING,
    parse_color,
    format_color,
    parse_nav_offset,
    format_nav_offset,
)
from feeding_deployment.preference_learning.methods.episodic_memory import EpisodicMemoryModel
from feeding_deployment.preference_learning.methods.full_history_memory import (
    FullHistoryMemoryModel,
)
from feeding_deployment.preference_learning.methods.long_term_memory import (
    LongTermMemoryModel,
    _extract_json_object,
)
from feeding_deployment.preference_learning.methods.prompts.bundle_prediction import (
    get_bundle_prediction_prompt,
)
from feeding_deployment.preference_learning.methods.utils import _episode_text, PREF_FIELDS, _resolve_api_key, _retry_on_rate_limit
from feeding_deployment.utils.llm_config import (
    PREDICTION_CLAUDE_MODEL,
    PREDICTION_EFFORT,
    PREDICTION_FAST_MODE,
)

# Cross-day memory backends. "three_layer" is semantic LTM summary + episodic
# retrieval + working memory; "single_full_history" replaces the first two with
# every prior finalized meal verbatim; "no_memory" predicts from working memory
# alone. Working memory (current-meal context + corrections) is live state, not
# long-term memory, so it exists in every mode.
MEMORY_MODES = ("three_layer", "single_full_history", "no_memory")
# Default backend for deployment, the emulator, and offline eval.
DEFAULT_MEMORY_MODE = "single_full_history"

PREF_OPTIONS: Dict[str, List[str]] = {name: opts for (name, _, opts) in root_config.PREFERENCE_BUNDLE}
PREF_DESCRIPTIONS: Dict[str, str] = {dim.field: dim.description for dim in _PREF_BUNDLE_DIMS}
PREF_KIND: Dict[str, str] = {dim.field: getattr(dim, "kind", "categorical") for dim in _PREF_BUNDLE_DIMS}
_COLOR_FIELD_SET = set(COLOR_FIELDS)
_TEXT_FIELD_SET = set(TEXT_FIELDS)
_NAV_OFFSET_FIELD_SET = set(NAV_OFFSET_FIELDS)

# Per-field default for text dims when the LLM output is empty/missing (kept
# non-empty so downstream consumers -- e.g. FLAIR -- always get a usable value).
_TEXT_DEFAULTS: Dict[str, str] = {"bite_ordering": DEFAULT_BITE_ORDERING}


def _strip_code_fences(s: str) -> str:
    s = s.strip()
    if "```" not in s:
        return s
    if "```json" in s:
        return s.split("```json", 1)[1].split("```", 1)[0].strip()
    return s.split("```", 1)[1].split("```", 1)[0].strip()


def _safe_json_load(s: str) -> Optional[Dict[str, Any]]:
    try:
        data = json.loads(s)
        return data if isinstance(data, dict) else None
    except Exception:
        return None


def _bundle_present(data: Optional[Dict[str, Any]]) -> bool:
    """True when a parsed response actually carries a preference bundle. Guards
    against valid-JSON-but-no-bundle responses (e.g. the model derails on the
    nested latent_scores object and closes the JSON before emitting any
    preference keys) -- treated the same as unparseable JSON so the caller
    retries instead of silently defaulting every dimension. Requires at least
    half of the categorical dims to be present, tolerating a stray omission
    (which the per-field validator backfills) while catching a missing bundle."""
    if not data:
        return False
    required = [f for f in PREF_FIELDS if PREF_KIND.get(f) == "categorical"]
    if not required:
        return True
    present = sum(1 for f in required if f in data)
    return present >= max(1, len(required) // 2)


def _get_meal_info(meal: str) -> Dict[str, Any]:
    known_meal = meal in root_config.MEAL_STRUCTURE
    info = root_config.MEAL_STRUCTURE.get(meal, {})
    dippable = list(info.get("dippable_items", []) or [])
    sauces = list(info.get("sauces", []) or [])
    return {
        "known_meal": known_meal,
        "dippable_items": dippable,
        "sauces": sauces,
        "has_dippable": len(dippable) > 0,
        "has_sauce": len(sauces) > 0,
    }


def _apply_hard_rules(prefs: Dict[str, str], meal: str, corrected: Dict[str, str]) -> Dict[str, str]:
    out = dict(prefs)
    meal_info = _get_meal_info(meal)

    if out.get("transfer_mode") == "inside mouth transfer" and "outside_mouth_distance" not in corrected:
        out["outside_mouth_distance"] = "not applicable"

    if (
        "bite_dipping_preference" not in corrected
        and meal_info.get("known_meal", False)
        and ((not meal_info["has_dippable"]) or (not meal_info["has_sauce"]))
    ):
        out["bite_dipping_preference"] = "do not dip"

    return out

def _build_options_block(
    color_seeds: Optional[Dict[str, Any]] = None,
    nav_offset_seeds: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Bundle-prediction options block.
    - categorical field: ``- field: [opt1, opt2, ...]``
    - color field:       ``- field: HSV object {...}; seed = h=..,s=..,v=..,range=..``
    - nav offset field:  ``- field: offset object {...}; seed = dx=..,dy=..,dyaw=..``
    - text field:        ``- field: free-text string (...)``
    """
    color_seeds = color_seeds or {}
    nav_offset_seeds = nav_offset_seeds or {}
    lines: List[str] = []
    for field in PREF_FIELDS:
        if PREF_KIND.get(field) == "color":
            seed = parse_color(color_seeds.get(field), seed=DEFAULT_COLOR)
            lines.append(
                f'- {field}: HSV object {{"h":0-179,"s":0-255,"v":0-255,"range":0.0-1.0}}; '
                f"seed = {format_color(seed)}"
            )
        elif PREF_KIND.get(field) == "nav_offset":
            seed = parse_nav_offset(nav_offset_seeds.get(field), seed=DEFAULT_NAV_OFFSET)
            lines.append(
                f'- {field}: offset object {{"dx":-0.5-0.5 (m),"dy":-0.5-0.5 (m),'
                f'"dyaw":-0.785-0.785 (rad)}}; seed = {format_nav_offset(seed)}'
            )
        elif PREF_KIND.get(field) == "text":
            lines.append(
                f"- {field}: free-text string (a single concise natural-language "
                f"sentence grounded in this meal's foods/dips)"
            )
        else:
            opts = PREF_OPTIONS[field]
            lines.append(f"- {field}: [{', '.join(opts)}]")
    return "\n".join(lines)


def _build_meal_contents_block(meal: str) -> str:
    """Human-readable solids/dips for the chosen meal, so the LLM can ground a
    text dim (e.g. bite_ordering) in concrete foods. Falls back gracefully for
    a meal not in the catalog."""
    info = _get_meal_info(meal)
    if not info["known_meal"]:
        return f"meal: {meal} (contents unknown)"
    solids = ", ".join(info["dippable_items"]) or "(none)"
    dips = ", ".join(info["sauces"]) or "(none)"
    return f"solid items: {solids}\ndips/sauces: {dips}"


def _format_corrected_block(corrected: Dict[str, Any]) -> str:
    """
    Each line is key=value. Color corrections are rendered with the compact
    HSV encoding so the model sees the corrected handle color.
    """
    if not corrected:
        return "(none)"
    out = []
    for k, v in corrected.items():
        if k in _COLOR_FIELD_SET:
            out.append(f"{k}={format_color(v if isinstance(v, dict) else {})}")
        elif k in _NAV_OFFSET_FIELD_SET:
            out.append(f"{k}={format_nav_offset(v if isinstance(v, dict) else {})}")
        else:
            out.append(f"{k}={v}")
    return "\n".join(out)


def _format_prior_predictions_block(prior_predictions: Optional[List[Dict[str, Any]]]) -> str:
    """Render THIS meal's earlier model predictions (the session's in-memory
    within-meal history) so a reprediction can self-diagnose same-context bias.
    One entry per (re)prediction, oldest first: the latent-trait scores it
    asserted, its latent-factor inference, and the open-dim values it predicted.
    Returns an empty sentinel when feedback is disabled or this is the first
    prediction of the meal."""
    if not prior_predictions:
        return "(none — this is your first prediction this meal)"
    rounds = []
    for i, entry in enumerate(prior_predictions, start=1):
        trig = entry.get("trigger") or "initial"
        scores = entry.get("latent_scores") or {}
        score_bits = []
        for k in ("pace", "trust", "proximity", "communication"):
            v = scores.get(k)
            if isinstance(v, dict):
                score_bits.append(f"{k}={v.get('score')}")
            elif v is not None:
                score_bits.append(f"{k}={v}")
        score_line = ", ".join(score_bits) if score_bits else "(none)"
        inference_txt = entry.get("latent_inference") or ""
        predicted = entry.get("predicted") or {}
        pred_line = (
            "; ".join(f"{k}={val}" for k, val in predicted.items()) if predicted else "(none)"
        )
        rounds.append(
            f"Prediction {i} (after: {trig}):\n"
            f"  latent_scores: {score_line}\n"
            f"  latent_inference: {inference_txt}\n"
            f"  predicted open dims: {pred_line}"
        )
    return "\n".join(rounds)

def _day_path(dir_path: Path, day: int) -> Path:
    return dir_path / f"day_{day:04d}.json"

def _write_json(path: Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


class PredictionModel:
    """
    Combines a cross-day memory backend (see ``memory_mode`` / MEMORY_MODES):
    - "three_layer": LongTermMemoryModel (semantic memory summary, as JSON
      string) + EpisodicMemoryModel (episodic retrieval over history)
    - "single_full_history": FullHistoryMemoryModel (every prior finalized
      meal verbatim; no summarization, no retrieval)
    - "no_memory": nothing cross-day
    with working memory (current context + corrected so far), and calls the
    LLM to predict the preference bundle.

    ``memory_mode=None`` resolves to DEFAULT_MEMORY_MODE, except that passing
    either ``use_long_term_memory`` / ``use_episodic_memory`` implies
    "three_layer" -- they are its sub-ablations (which of LTM/EM to enable)
    and may not be combined with the other modes, which derive both as False.
    """

    def __init__(
        self,
        user: str,
        physical_profile_label: str,
        logs_dir: Path,
        retry_fn = _retry_on_rate_limit,
        memory_mode: Optional[str] = None,
        use_long_term_memory: Optional[bool] = None,
        use_episodic_memory: Optional[bool] = None,
        k_retrieve: int = 5,
        max_full_history_days: Optional[int] = None,
        chat_model: str = PREDICTION_CLAUDE_MODEL,
        embed_model: str = "text-embedding-3-small",
        physical_profile_description: str | None = None,
    ) -> None:

        if memory_mode is None:
            memory_mode = (
                "three_layer"
                if (use_long_term_memory is not None or use_episodic_memory is not None)
                else DEFAULT_MEMORY_MODE
            )
        if memory_mode not in MEMORY_MODES:
            raise ValueError(
                f"Unknown memory_mode={memory_mode!r}. Valid: {', '.join(MEMORY_MODES)}"
            )
        if memory_mode == "three_layer":
            use_long_term_memory = True if use_long_term_memory is None else use_long_term_memory
            use_episodic_memory = True if use_episodic_memory is None else use_episodic_memory
        else:
            if use_long_term_memory is not None or use_episodic_memory is not None:
                raise ValueError(
                    "use_long_term_memory / use_episodic_memory are three-layer "
                    f"sub-ablations; do not pass them with memory_mode={memory_mode!r}."
                )
            use_long_term_memory = use_episodic_memory = False
        self.memory_mode = memory_mode

        self.user = user
        self.physical_profile_label = physical_profile_label
        self.physical_profile_description = physical_profile_description
        self.client = anthropic.Anthropic()  # chat (reads ANTHROPIC_API_KEY)
        # Latched True on the first hard 4xx from a fast-mode request (no
        # access / bad request) so later calls skip the doomed fast attempt.
        # Rate limits do NOT latch -- fast capacity replenishes continuously.
        self._fast_mode_unavailable = False
        # Embeddings stay on OpenAI (falls back to OPENAI_API_KEY). Only the
        # episodic retrieval layer needs them, so the other memory modes never
        # require an OpenAI key.
        self.embed_client: Optional[OpenAI] = (
            OpenAI(api_key=_resolve_api_key(None)) if use_episodic_memory else None
        )
        self.chat_model = chat_model
        self.embed_model = embed_model
        self._retry = retry_fn
        self.logs_dir = logs_dir
        self.long_term_memory_model: Optional[LongTermMemoryModel] = None
        self.episodic_memory_model: Optional[EpisodicMemoryModel] = None
        self.full_history_memory_model: Optional[FullHistoryMemoryModel] = None

        if use_long_term_memory:
            self.long_term_memory_dir = self.logs_dir / user / "long_term_memory"
            self.long_term_memory_dir.mkdir(parents=True, exist_ok=True)
            self.long_term_memory_model = LongTermMemoryModel(
                physical_profile_label=self.physical_profile_label,
                client=self.client,
                chat_model=self.chat_model,
                retry_fn=retry_fn,
                logs_dir=self.logs_dir / "long_term_memory_llm_calls",
                physical_profile_description=self.physical_profile_description,
            )
            
        if use_episodic_memory:
            self.episodic_memory_dir = self.logs_dir / user / "episodic_memory"
            self.episodic_memory_dir.mkdir(parents=True, exist_ok=True)
            self.episodic_memory_model = EpisodicMemoryModel(
                client=self.embed_client,
                embed_model=self.embed_model,
                cache_path=self.logs_dir / "embeddings.json",
                retry_fn=self._retry,
                k_retrieve=k_retrieve,
            )

        if memory_mode == "single_full_history":
            self.full_history_memory_dir = self.logs_dir / user / "full_history_memory"
            self.full_history_memory_dir.mkdir(parents=True, exist_ok=True)
            self.full_history_memory_model = FullHistoryMemoryModel(
                max_days=max_full_history_days,
            )

        self.working_memory_dir = self.logs_dir / user / "working_memory"
        self.working_memory_dir.mkdir(parents=True, exist_ok=True)
        
        self.working_memory_calls_dir = self.logs_dir / user / "prediction_model_llm_calls"
        self.working_memory_calls_dir.mkdir(parents=True, exist_ok=True)

        # Per-open-dim reasons from the most recent predict_bundle call (the
        # LLM's "explanations" object); {} when absent/malformed. The leading
        # "latent_inference" sentence(s) ride along in last_latent_inference.
        self.last_explanations: Dict[str, str] = {}
        self.last_latent_inference: str = ""
        # The LLM's scored predictions over the four stable latent user-traits
        # (pace / trust / proximity / communication), each
        # {"score": 1-5, "confidence": ..., "why": ...}; {} when absent/malformed.
        # This is the model's inferred latent state, graded offline against the
        # user's held-out pre-meal self-report. Extra response key, not a
        # preference field (the validation loop only reads PREF_FIELDS).
        self.last_latent_scores: Dict[str, Any] = {}

    @staticmethod
    def _scan_day_files(dir_path: Path, current_day: int):
        """Yield ``(day, path)`` for ``day_*.json`` files with day < current_day,
        in ascending day order. Files with an unparseable stem are skipped."""
        found = []
        for p in sorted(dir_path.glob("day_*.json")):
            try:
                d = int(p.stem.split("_", 1)[1])
            except (ValueError, IndexError):
                continue
            if d < current_day:
                found.append((d, p))
        found.sort(key=lambda t: t[0])
        return found

    def existing_days(self) -> set[int]:
        """All finalized day numbers, from the canonical ``working_memory`` marker
        (written unconditionally at finalize)."""
        days: set[int] = set()
        for p in self.working_memory_dir.glob("day_*.json"):
            try:
                days.add(int(p.stem.split("_", 1)[1]))
            except (ValueError, IndexError):
                continue
        return days

    def validate_sequential_day(self, current_day: int) -> None:
        """Reject day gaps: every day in ``1..current_day-1`` must already be
        finalized. The deployment runs strictly sequentially, so a hole (a skipped
        day, or a prior day that crashed before finalize) is an error. Re-running
        or resuming ``current_day`` itself is allowed."""
        missing = sorted(set(range(1, current_day)) - self.existing_days())
        if missing:
            raise ValueError(
                f"Cannot start day {current_day}: missing finalized memory for "
                f"day(s) {missing}. Deployment days must be sequential with no "
                f"gaps -- run the missing day(s) first."
            )

    def load_prior_memory(self, current_day: int) -> None:
        """Re-hydrate cross-day memory from disk for days strictly before
        ``current_day``. Each daily run is a fresh process; without this the LTM
        summary and episodic history start empty and prior days' learning is lost.

        LTM is cumulative, so only the latest prior day's summary is needed;
        episodic and full-history each need every prior episode_text. The
        strictly-less-than cutoff keeps a re-run/resume of ``current_day`` from
        folding its own prior record back into itself."""
        if self.long_term_memory_model:
            ltm_files = self._scan_day_files(self.long_term_memory_dir, current_day)
            if ltm_files:
                _, latest_path = ltm_files[-1]
                try:
                    record = json.loads(latest_path.read_text(encoding="utf-8"))
                    raw = record.get("ltm_summary_raw", "")
                    summary = raw if isinstance(raw, str) else json.dumps(raw, ensure_ascii=False)
                    if summary.strip():
                        self.long_term_memory_model.load_summary(summary)
                except Exception as e:
                    print(f"Warning: failed to load LTM summary from {latest_path}: {e}", flush=True)

        if self.episodic_memory_model:
            texts: List[str] = []
            for _, path in self._scan_day_files(self.episodic_memory_dir, current_day):
                try:
                    record = json.loads(path.read_text(encoding="utf-8"))
                except Exception as e:
                    print(f"Warning: skipping unreadable episodic record {path}: {e}", flush=True)
                    continue
                txt = record.get("episode_text", "")
                if txt:
                    texts.append(txt)
            if texts:
                self.episodic_memory_model.load_history(texts)

        if self.full_history_memory_model:
            texts = []
            for _, path in self._scan_day_files(self.full_history_memory_dir, current_day):
                try:
                    record = json.loads(path.read_text(encoding="utf-8"))
                except Exception as e:
                    print(f"Warning: skipping unreadable full-history record {path}: {e}", flush=True)
                    continue
                txt = record.get("episode_text", "")
                if txt:
                    texts.append(txt)
            if texts:
                self.full_history_memory_model.load_history(texts)

    def update(self, day: int, context: Dict[str, Any], corrected: Dict[str, str], ground_truth_bundle: Dict[str, str]) -> None:
        
        ep_txt = _episode_text(day=day, context=context, prefs=ground_truth_bundle)
        
        if self.long_term_memory_model:
            print(f"  [long_term_memory_model] Updating summary (day {day}) ...", flush=True)
            self.long_term_memory_model.add_episode(ep_txt)
            long_term_memory = self.long_term_memory_model.get_ltm()  # JSON string (or empty)

            log_ltm_summary = long_term_memory
            try:
                log_ltm_summary = json.loads(long_term_memory)
            except Exception:
                print(
                    f"Warning: long_term_memory_model summary for logging is not valid JSON. "
                    f"Logging raw string. Summary:\n{long_term_memory}\n",
                    flush=True,
                )
                
            # Per-day logs we will write once at the end of the day
            long_term_memory_record = {
                "day": day,
                "context": context,
                "episode_text": ep_txt,
                "ltm_summary_raw": log_ltm_summary,
            }
            
            _write_json(_day_path(self.long_term_memory_dir, day), long_term_memory_record)

        if self.episodic_memory_model:
            # Update retrieval history after finishing the interactive correction loop
            self.episodic_memory_model.add_episode(ep_txt)
            
            episodic_memory_record = {
                "day": day,
                "context": context,
                "corrected": dict(corrected),
                "episode_text": ep_txt,
                "retrieved_episodes": self.episodic_memory_model.get_last_retrieved(),
            }
            _write_json(_day_path(self.episodic_memory_dir, day), episodic_memory_record)

        if self.full_history_memory_model:
            self.full_history_memory_model.add_episode(ep_txt)

            full_history_memory_record = {
                "day": day,
                "context": context,
                "corrected": dict(corrected),
                "episode_text": ep_txt,
                "ground_truth_bundle": dict(ground_truth_bundle),
            }
            _write_json(_day_path(self.full_history_memory_dir, day), full_history_memory_record)

        # Always written, in every memory mode: the canonical "day finalized"
        # marker that existing_days()/validate_sequential_day key off.
        working_memory_record = {
            "day": day,
            "context": context,
            "corrected": dict(corrected)
        }
        _write_json(_day_path(self.working_memory_dir, day), working_memory_record)

    def _create_prediction_message(self, **request_kwargs: Any) -> Any:
        """One prediction request, preferring fast mode when enabled.

        Fast mode (research preview) serves the same Opus weights at up to
        2.5x output tokens/sec for 2x price -- the prediction call is
        output-dominated (adaptive thinking + the full-bundle JSON), which is
        exactly where that speedup lands. The fast attempt uses max_retries=0
        so ANY failure falls through to standard speed immediately instead of
        stalling the robot on fast-mode capacity:
        - RateLimitError with a fast-mode quota of 0 (the API reports
          "research-preview access not granted" as a zero per-minute limit;
          verified against the live API): latch _fast_mode_unavailable so
          later calls skip the doomed attempt -- one clear message per run.
        - RateLimitError with a real (non-zero) quota: fall back for this
          call only; capacity replenishes continuously.
        - other 4xx: latch _fast_mode_unavailable.
        - 5xx / connection errors: transient; fall back for this call.
        The standard-speed path keeps the caller's normal retry behavior.
        """
        if PREDICTION_FAST_MODE and not self._fast_mode_unavailable:
            try:
                return self.client.with_options(max_retries=0).beta.messages.create(
                    speed="fast", betas=["fast-mode-2026-02-01"], **request_kwargs
                )
            except anthropic.RateLimitError as e:
                try:
                    fast_limit = e.response.headers.get("anthropic-fast-input-tokens-limit")
                except Exception:
                    fast_limit = None
                if fast_limit is not None and str(fast_limit).strip() == "0":
                    self._fast_mode_unavailable = True
                    print(
                        "[prediction] fast mode is not enabled for this org "
                        "(quota 0 -- research-preview access not granted); "
                        "using standard speed from now on.", flush=True,
                    )
                else:
                    print("[prediction] fast mode rate-limited; using standard speed for this call.", flush=True)
            except anthropic.APIStatusError as e:
                if e.status_code < 500:
                    self._fast_mode_unavailable = True
                    print(
                        f"[prediction] fast mode unavailable (HTTP {e.status_code}); "
                        "using standard speed from now on.", flush=True,
                    )
                else:
                    print(f"[prediction] fast mode request failed (HTTP {e.status_code}); using standard speed for this call.", flush=True)
            except anthropic.APIConnectionError:
                print("[prediction] fast mode connection error; using standard speed for this call.", flush=True)
        return self.client.messages.create(**request_kwargs)

    def predict_bundle(
        self,
        context: Dict[str, Any],
        corrected: Dict[str, Any],
        confirmed: Optional[Dict[str, Any]] = None,
        color_seeds: Optional[Dict[str, Any]] = None,
        nav_offset_seeds: Optional[Dict[str, Any]] = None,
        prior_predictions: Optional[List[Dict[str, Any]]] = None,
    ) -> Dict[str, Any]:
        """
        Returns predicted_bundle.

        ``corrected`` holds the dims the user actively CHANGED this meal;
        ``confirmed`` holds the dims the user accepted as-predicted. Both are
        pinned (never flipped in the output) but are rendered as separate
        prompt blocks — a correction is evidence about the meal's latent
        factors, a confirmation merely says the prediction was acceptable.

        ``color_seeds`` maps each plate_color_* field to its current saved color
        (canonical dict / BT YAML value). Color predictions are seeded with these
        and fall back to them when the LLM output cannot be parsed.

        ``nav_offset_seeds`` maps each nav_offset_* field to its current saved
        offset (canonical dict / BT YAML value); same seeding/fallback contract
        as colors.
        """
        color_seeds = color_seeds or {}
        nav_offset_seeds = nav_offset_seeds or {}
        confirmed = confirmed or {}
        # All finalized dims; a dim in both maps takes the corrected value.
        pinned = {**confirmed, **corrected}

        episodic_memory = ""
        if self.episodic_memory_model:
            # Retrieval keys off the informative signal: true corrections only.
            episodic_memory = self.episodic_memory_model.retrieve(context, corrected)

        long_term_memory = ""
        if self.long_term_memory_model:
            long_term_memory = self.long_term_memory_model.get_ltm()  # JSON string (or empty)

        full_history_block = ""
        if self.full_history_memory_model:
            full_history_block = self.full_history_memory_model.get_memory_block()

        # Prompt blocks
        options_block = _build_options_block(color_seeds, nav_offset_seeds)
        corrected_block = _format_corrected_block(corrected)
        confirmed_block = _format_corrected_block(confirmed)
        meal_contents_block = _build_meal_contents_block(str(context.get("meal", "")))
        prior_predictions_block = _format_prior_predictions_block(prior_predictions)

        prompt = get_bundle_prediction_prompt(
            physical_profile_label=self.physical_profile_label,
            ltm_summary=long_term_memory,
            retrieved_block=episodic_memory,
            memory_mode=self.memory_mode,
            full_history_block=full_history_block,
            context=context,
            corrected_block=corrected_block,
            confirmed_block=confirmed_block,
            options_block=options_block,
            meal_contents=meal_contents_block,
            prior_predictions_block=prior_predictions_block,
            physical_profile_description=self.physical_profile_description,
        )

        def _call() -> Any:
            return self._create_prediction_message(
                model=self.chat_model,
                max_tokens=16000,
                # Adaptive thinking + PREDICTION_EFFORT (see llm_config.py for
                # the effort-sweep rationale): the reprediction must commit to
                # values implied by this meal's corrections (cross-dimension
                # correlations). Thinking tokens share the max_tokens budget
                # with the JSON output.
                thinking={"type": "adaptive"},
                output_config={"effort": PREDICTION_EFFORT},
                system="Return JSON only. No extra text.",
                messages=[
                    {"role": "user", "content": prompt},
                ],
            )

        def _parse_response(resp) -> Optional[Dict[str, Any]]:
            raw = ("".join(b.text for b in resp.content if b.type == "text")).strip()
            raw = _strip_code_fences(raw)
            # Tolerate prose around the object via balanced-brace extraction.
            return _safe_json_load(raw) or _safe_json_load(_extract_json_object(raw))

        resp = self._retry(_call)
        data = _parse_response(resp)
        if not _bundle_present(data):
            # Either unparseable JSON (data is None) or valid JSON that omitted
            # the preference bundle (e.g. the model derails on the nested
            # latent_scores object and closes early). Both would otherwise fall
            # back to seeds/pinned/defaults, silently discarding the prediction
            # for the whole meal -- one fresh attempt first.
            print("Warning: bundle prediction missing or malformed (bad JSON or absent preference keys); retrying once ...", flush=True)
            resp = self._retry(_call)
            data2 = _parse_response(resp)
            if _bundle_present(data2):
                data = data2
            else:
                data = data or data2
        data = data or {}

        if self.working_memory_calls_dir:
            log_file = self.working_memory_calls_dir / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            # usage.speed reports which speed actually served the request
            # ("fast" or "standard"); absent on responses without usage.
            served_speed = getattr(getattr(resp, "usage", None), "speed", None) or "standard"
            header = f"===MODEL===\n{self.chat_model} (speed={served_speed}, effort={PREDICTION_EFFORT})\n\n"
            if data:
                log_file.write_text(f"{header}===PROMPT===\n{prompt}\n\n===RESPONSE===\n{json.dumps(data, indent=2)}", encoding="utf-8")
            else:
                log_file.write_text(f"{header}===PROMPT===\n{prompt}\n\n===RESPONSE===\nFailed to parse response as JSON. Raw response:\n{resp}", encoding="utf-8")

        # Per-open-dim reasons + the leading latent-factor inference; extra
        # response keys, not preference fields (the validation loop below only
        # reads PREF_FIELDS).
        expl = data.get("explanations")
        self.last_explanations = dict(expl) if isinstance(expl, dict) else {}
        self.last_latent_inference = str(data.get("latent_inference") or "")
        ls = data.get("latent_scores")
        self.last_latent_scores = dict(ls) if isinstance(ls, dict) else {}

        # Validate each field; categorical -> allowed option (fallback to
        # pinned/default), color -> parsed HSV (fallback to seed), nav
        # offset -> parsed dx/dy/dyaw (fallback to seed), text -> free string
        # (fallback to pinned/per-field default, never empty).
        out: Dict[str, Any] = {}
        for field in PREF_FIELDS:
            if PREF_KIND.get(field) == "color":
                seed = parse_color(color_seeds.get(field), seed=DEFAULT_COLOR)
                out[field] = parse_color(data.get(field), seed=seed)
            elif PREF_KIND.get(field) == "nav_offset":
                seed = parse_nav_offset(nav_offset_seeds.get(field), seed=DEFAULT_NAV_OFFSET)
                out[field] = parse_nav_offset(data.get(field), seed=seed)
            elif PREF_KIND.get(field) == "text":
                val = str(data.get(field, "")).strip()
                out[field] = val or pinned.get(field) or _TEXT_DEFAULTS.get(field, "")
            else:
                val = str(data.get(field, "")).strip()
                if val in PREF_OPTIONS[field]:
                    out[field] = val
                else:
                    out[field] = pinned.get(field, PREF_OPTIONS[field][0])

        out = _apply_hard_rules(out, meal=str(context.get("meal", "")), corrected=pinned)

        # Pinned (confirmed + corrected) always overrides (canonicalize
        # color/offset values).
        for k, v in pinned.items():
            if k in _COLOR_FIELD_SET:
                out[k] = parse_color(v, seed=parse_color(color_seeds.get(k), seed=DEFAULT_COLOR))
            elif k in _NAV_OFFSET_FIELD_SET:
                out[k] = parse_nav_offset(v, seed=parse_nav_offset(nav_offset_seeds.get(k), seed=DEFAULT_NAV_OFFSET))
            else:
                out[k] = v

        # Final validation (categorical only; color/nav-offset/text values are
        # already canonical free-form and have no fixed option list).
        for field in PREF_FIELDS:
            if PREF_KIND.get(field) in ("color", "text", "nav_offset"):
                continue
            if out[field] not in PREF_OPTIONS[field]:
                out[field] = PREF_OPTIONS[field][0]
        return out
