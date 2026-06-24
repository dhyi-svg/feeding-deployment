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
    DEFAULT_COLOR,
    parse_color,
    format_color,
)
from feeding_deployment.preference_learning.methods.episodic_memory import EpisodicMemoryModel
from feeding_deployment.preference_learning.methods.long_term_memory import LongTermMemoryModel
from feeding_deployment.preference_learning.methods.prompts.bundle_prediction import (
    get_bundle_prediction_prompt,
)
from feeding_deployment.preference_learning.methods.utils import _episode_text, PREF_FIELDS, _resolve_api_key, _retry_on_rate_limit

PREF_OPTIONS: Dict[str, List[str]] = {name: opts for (name, _, opts) in root_config.PREFERENCE_BUNDLE}
PREF_DESCRIPTIONS: Dict[str, str] = {dim.field: dim.description for dim in _PREF_BUNDLE_DIMS}
PREF_KIND: Dict[str, str] = {dim.field: getattr(dim, "kind", "categorical") for dim in _PREF_BUNDLE_DIMS}
_COLOR_FIELD_SET = set(COLOR_FIELDS)


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

def _build_options_block(color_seeds: Optional[Dict[str, Any]] = None) -> str:
    """
    Bundle-prediction options block.
    - categorical field: ``- field: [opt1, opt2, ...]``
    - color field:       ``- field: HSV object {...}; seed = h=..,s=..,v=..,range=..``
    """
    color_seeds = color_seeds or {}
    lines: List[str] = []
    for field in PREF_FIELDS:
        if PREF_KIND.get(field) == "color":
            seed = parse_color(color_seeds.get(field), seed=DEFAULT_COLOR)
            lines.append(
                f'- {field}: HSV object {{"h":0-179,"s":0-255,"v":0-255,"range":0.0-1.0}}; '
                f"seed = {format_color(seed)}"
            )
        else:
            opts = PREF_OPTIONS[field]
            lines.append(f"- {field}: [{', '.join(opts)}]")
    return "\n".join(lines)


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
        else:
            out.append(f"{k}={v}")
    return "\n".join(out)

def _day_path(dir_path: Path, day: int) -> Path:
    return dir_path / f"day_{day:04d}.json"

def _write_json(path: Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2)


class PredictionModel:
    """
    Combines:
    - LongTermMemoryModel (semantic memory summary, as JSON string)
    - EpisodicMemoryModel (episodic retrieval over history)
    - Working memory (current context + corrected so far)
    And calls the LLM to predict the preference bundle.
    """

    def __init__(
        self,
        user: str,
        physical_profile_label: str,
        logs_dir: Path,
        retry_fn = _retry_on_rate_limit,
        use_long_term_memory: bool = True,
        use_episodic_memory: bool = True,
        k_retrieve: int = 5,
        chat_model: str = "claude-opus-4-8",
        embed_model: str = "text-embedding-3-small",
        physical_profile_description: str | None = None,
    ) -> None:
        
        self.user = user
        self.physical_profile_label = physical_profile_label
        self.physical_profile_description = physical_profile_description
        self.client = anthropic.Anthropic()  # chat (reads ANTHROPIC_API_KEY)
        self.embed_client = OpenAI(api_key=_resolve_api_key())  # embeddings stay on OpenAI
        self.chat_model = chat_model
        self.embed_model = embed_model
        self._retry = retry_fn
        self.logs_dir = logs_dir
        self.long_term_memory_model: Optional[LongTermMemoryModel] = None
        self.episodic_memory_model: Optional[EpisodicMemoryModel] = None
        
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
            
        self.working_memory_dir = self.logs_dir / user / "working_memory"
        self.working_memory_dir.mkdir(parents=True, exist_ok=True)
        
        self.working_memory_calls_dir = self.logs_dir / user / "prediction_model_llm_calls"
        self.working_memory_calls_dir.mkdir(parents=True, exist_ok=True)

    def next_day(self) -> int:
        """Return the next unused day number by scanning existing ``day_NNNN.json`` files."""
        existing = sorted(self.working_memory_dir.glob("day_*.json"))
        if not existing:
            return 1
        last_stem = existing[-1].stem          # e.g. "day_0003"
        return int(last_stem.split("_", 1)[1]) + 1

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
            
        working_memory_record = {
            "day": day,
            "context": context,
            "corrected": dict(corrected)
        }
        _write_json(_day_path(self.working_memory_dir, day), working_memory_record)

    def predict_bundle(
        self,
        context: Dict[str, Any],
        corrected: Dict[str, Any],
        color_seeds: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        """
        Returns predicted_bundle.

        ``color_seeds`` maps each plate_color_* field to its current saved color
        (canonical dict / BT YAML value). Color predictions are seeded with these
        and fall back to them when the LLM output cannot be parsed.
        """
        color_seeds = color_seeds or {}

        episodic_memory = ""
        if self.episodic_memory_model:
            episodic_memory = self.episodic_memory_model.retrieve(context, corrected)

        long_term_memory = ""
        if self.long_term_memory_model:
            long_term_memory = self.long_term_memory_model.get_ltm()  # JSON string (or empty)

        # Prompt blocks
        options_block = _build_options_block(color_seeds)
        corrected_block = _format_corrected_block(corrected)

        prompt = get_bundle_prediction_prompt(
            physical_profile_label=self.physical_profile_label,
            ltm_summary=long_term_memory,
            retrieved_block=episodic_memory,
            context=context,
            corrected_block=corrected_block,
            options_block=options_block,
            physical_profile_description=self.physical_profile_description,
        )

        def _call() -> Any:
            return self.client.messages.create(
                model=self.chat_model,
                max_tokens=15000,
                system="Return JSON only. No extra text.",
                messages=[
                    {"role": "user", "content": prompt},
                ],
            )

        resp = self._retry(_call)
        raw = ("".join(b.text for b in resp.content if b.type == "text")).strip()
        raw = _strip_code_fences(raw)
        data = _safe_json_load(raw) or {}
            
        if self.working_memory_calls_dir:
            log_file = self.working_memory_calls_dir / f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
            if data:
                log_file.write_text(f"===PROMPT===\n{prompt}\n\n===RESPONSE===\n{json.dumps(data, indent=2)}", encoding="utf-8")
            else:
                log_file.write_text(f"===PROMPT===\n{prompt}\n\n===RESPONSE===\nFailed to parse response as JSON. Raw response:\n{resp}", encoding="utf-8")

        # Validate each field; categorical -> allowed option (fallback to
        # corrected/default), color -> parsed HSV (fallback to seed).
        out: Dict[str, Any] = {}
        for field in PREF_FIELDS:
            if PREF_KIND.get(field) == "color":
                seed = parse_color(color_seeds.get(field), seed=DEFAULT_COLOR)
                out[field] = parse_color(data.get(field), seed=seed)
            else:
                val = str(data.get(field, "")).strip()
                if val in PREF_OPTIONS[field]:
                    out[field] = val
                else:
                    out[field] = corrected.get(field, PREF_OPTIONS[field][0])

        out = _apply_hard_rules(out, meal=str(context.get("meal", "")), corrected=corrected)

        # Corrected always overrides (canonicalize color corrections).
        for k, v in corrected.items():
            if k in _COLOR_FIELD_SET:
                out[k] = parse_color(v, seed=parse_color(color_seeds.get(k), seed=DEFAULT_COLOR))
            else:
                out[k] = v

        # Final validation (categorical only; color values are already canonical).
        for field in PREF_FIELDS:
            if PREF_KIND.get(field) == "color":
                continue
            if out[field] not in PREF_OPTIONS[field]:
                out[field] = PREF_OPTIONS[field][0]
        return out