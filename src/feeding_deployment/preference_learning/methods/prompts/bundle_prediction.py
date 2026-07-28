from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict, List, Optional

from feeding_deployment.preference_learning import config as root_config  # type: ignore
from feeding_deployment.preference_learning.config.physical_capabilities import (
    PHYSICAL_CAPABILITY_PROFILES,
)
from feeding_deployment.preference_learning.data_generation.prompts.system_description import (
    get_system_description_prompt,
)
from feeding_deployment.preference_learning.methods.utils import (
    _extract_truth_bundle,
)

BUNDLE_PREDICTION_PROMPT_PATH = Path(__file__).parent / "bundle_prediction.txt"
_PREF_FIELDS: List[str] = [name for (name, _, _) in root_config.PREFERENCE_BUNDLE]

_PHYSICAL_CAPABILITY_BY_LABEL = {p.label: p for p in PHYSICAL_CAPABILITY_PROFILES}


def _render_memory_block(
    memory_mode: str,
    ltm_summary: str,
    retrieved_block: str,
    full_history_block: str,
) -> str:
    """The cross-day memory section of the prompt, rendered per memory mode.
    Working memory is not part of this block -- current-meal corrections are
    live state and keep their own template section in every mode."""
    if memory_mode == "three_layer":
        return (
            "=== SEMANTIC MEMORY: Prior on the user's preference encoding inferred from all prior meals ===\n"
            f"{ltm_summary}\n"
            "======\n"
            "\n"
            "=== EPISODIC MEMORY: Similar prior meals retrieved from memory with ground truth user preference bundles ===\n"
            f"{retrieved_block}\n"
            "======"
        )
    if memory_mode == "single_full_history":
        return (
            "=== FULL HISTORY MEMORY: Every prior finalized meal in chronological order, with ground truth user preference bundles ===\n"
            f"{full_history_block}\n"
            "======"
        )
    if memory_mode == "no_memory":
        return "(no prior memory)"
    raise ValueError(f"Unknown memory_mode={memory_mode!r}")


def _render_memory_priority_lines(memory_mode: str) -> str:
    """Items 2+ of the prompt's priority list (item 1, the physical profile,
    is fixed in the template)."""
    if memory_mode == "three_layer":
        return "2. WORKING MEMORY\n3. EPISODIC MEMORY\n4. SEMANTIC MEMORY"
    if memory_mode == "single_full_history":
        return "2. WORKING MEMORY\n3. FULL HISTORY MEMORY"
    if memory_mode == "no_memory":
        return "2. WORKING MEMORY"
    raise ValueError(f"Unknown memory_mode={memory_mode!r}")


def get_bundle_prediction_prompt(
    physical_profile_label: str,
    ltm_summary: str,
    retrieved_block: str,
    context: dict,
    corrected_block: str,
    options_block: str,
    meal_contents: str = "(not provided)",
    confirmed_block: str = "(none)",
    *,
    memory_mode: str = "three_layer",
    full_history_block: str = "",
    prior_predictions_block: str = "(none — this is your first prediction this meal)",
    physical_profile_description: str | None = None,
) -> str:
    template = BUNDLE_PREDICTION_PROMPT_PATH.read_text(encoding="utf-8")

    system_description = get_system_description_prompt()

    if physical_profile_description is not None:
        desc = physical_profile_description.strip()
        if not desc:
            raise ValueError("physical_profile_description is empty")
    else:
        if physical_profile_label not in _PHYSICAL_CAPABILITY_BY_LABEL:
            valid = ", ".join(sorted(_PHYSICAL_CAPABILITY_BY_LABEL.keys()))
            raise ValueError(f"Unknown physical_profile_label={physical_profile_label!r}. Valid: {valid}")
        desc = _PHYSICAL_CAPABILITY_BY_LABEL[physical_profile_label].description

    return template.format(
        system_description=system_description,
        physical_profile=desc,
        memory_block=_render_memory_block(
            memory_mode, ltm_summary, retrieved_block, full_history_block
        ),
        memory_priority_lines=_render_memory_priority_lines(memory_mode),
        meal=context.get("meal"),
        setting=context.get("setting"),
        time_of_day=context.get("time_of_day"),
        meal_contents=meal_contents,
        confirmed_block=confirmed_block,
        corrected_block=corrected_block,
        prior_predictions_block=prior_predictions_block,
        options_block=options_block,
        pref_fields_csv=", ".join(_PREF_FIELDS),
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Print the bundle prediction prompt.")
    p.add_argument("--data-file", required=True, help="Path to one JSON dataset file.")
    p.add_argument(
        "--day",
        type=int,
        required=True,
        help="Day number to render (e.g., 1, 10, 30). This matches the 'day' field in the JSON.",
    )
    p.add_argument(
        "--ltm-summary-file",
        default="",
        help=(
            "Path to a text file containing the LTM summary to use in the prompt. "
            "If not provided, will use 'N/A'."
        ),
    )
    p.add_argument(
        "--retrieved-block-file",
        default="",
        help=(
            "Optional path to a text file containing retrieved memory/context to include in the prompt. "
            "If not provided, retrieved_block will be empty."
        ),
    )
    return p.parse_args()


def _build_options_block() -> str:
    # root_config.PREFERENCE_BUNDLE is a list of (name, _, options)
    pref_options = {name: opts for (name, _, opts) in root_config.PREFERENCE_BUNDLE}

    lines: List[str] = []
    for field in _PREF_FIELDS:
        opts = pref_options[field]
        lines.append(f"- {field}: [{', '.join(opts)}]")
    return "\n".join(lines)





def main() -> int:
    args = parse_args()

    data_path = Path(args.data_file)
    if not data_path.exists():
        raise SystemExit(f"Dataset file not found: {args.data_file}")

    data = json.loads(data_path.read_text(encoding="utf-8"))

    user = str(data.get("user", "unknown"))
    physical_profile_label = str(data.get("physical_profile_label", "")).strip()
    if not physical_profile_label:
        raise SystemExit("Dataset missing required field: 'physical_profile_label'")

    days: List[Dict[str, Any]] = list(data.get("days", []))

    # Find record whose "day" field matches args.day
    day_rec: Optional[Dict[str, Any]] = next((r for r in days if int(r.get("day", -1)) == args.day), None)
    if day_rec is None:
        available = sorted({int(r.get("day", 0)) for r in days})
        raise SystemExit(f"Day {args.day} not found in dataset. Available days: {available}")

    context: Dict[str, Any] = day_rec.get("context", {}) or {}

    # Ground truth (corrected) preference bundle for this day
    truth = _extract_truth_bundle(day_rec)
    corrected_block = "\n".join([f"{k}={v}" for k, v in truth.items()])

    ltm_summary = "N/A"
    if args.ltm_summary_file:
        if not Path(args.ltm_summary_file).exists():
            raise SystemExit(f"LTM summary file not found: {args.ltm_summary_file}")
        ltm_summary_json = json.loads(Path(args.ltm_summary_file).read_text(encoding="utf-8"))
        # use key "ltm_summary"
        ltm_summary = ltm_summary_json.get("ltm_summary", "N/A")
    
    retrieved_block = "N/A" # to be implemented in future work

    # Options block (all dimensions + allowed options)
    options_block = _build_options_block()

    # Meal contents (solids/dips) so text dims can be grounded in concrete foods.
    meal_info = root_config.MEAL_STRUCTURE.get(str(context.get("meal", "")), {})
    if meal_info:
        solids = ", ".join(meal_info.get("dippable_items", []) or []) or "(none)"
        dips = ", ".join(meal_info.get("sauces", []) or []) or "(none)"
        meal_contents = f"solid items: {solids}\ndips/sauces: {dips}"
    else:
        meal_contents = f"meal: {context.get('meal')} (contents unknown)"

    prompt = get_bundle_prediction_prompt(
        physical_profile_label=physical_profile_label,
        ltm_summary=ltm_summary,
        retrieved_block=retrieved_block,
        context=context,
        corrected_block=corrected_block,
        options_block=options_block,
        meal_contents=meal_contents,
    )

    print(f"User={user} | physical_profile_label={physical_profile_label} | day={args.day}", flush=True)
    print(prompt)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())