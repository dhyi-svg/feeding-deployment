"""Manual (human-in-the-loop) dataset generator for deployment simulations.

Same day loop and output schema as generate_deployment_dataset_llm.py, but a
human plays the preference oracle: each simulated day the script shows the
sampled context (meal, setting, time of day, transient affective state) and
collects the 20 human-decidable dims (categorical + text) from the terminal.

The 7 continuous dims (plate colors, nav offsets) are NOT collected -- they are
deterministic given a user's continuous_tables and the day's context, so they
can be injected into the finished dataset later. Until then, do not run the
evaluation on the output: the truth extractor fills missing continuous fields
with factory defaults instead of erroring.

Run with the same --seed as an LLM-generated dataset to get the identical
30-day context schedule (human oracle vs LLM oracle on the same meals).

Usage:
    PYTHONPATH=src python3 -m feeding_deployment.preference_learning.data_generation.generate_deployment_dataset_manual \
        --user manual_1 --seed 42 --days 30 --output-dir out_manual

Per-dim keys: number = pick option, Enter = keep default (previous day's
choice), b = back one question, ? = show the full dim description.
"""

import argparse
import json
import os
import random
import sys
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

from feeding_deployment.preference_learning.config.affective_state import AFFECTIVE_STATES
from feeding_deployment.preference_learning.config.mealtime_context import (
    MEALS,
    SETTINGS,
    TIMES_OF_DAY,
)
from feeding_deployment.preference_learning.config.preference_bundle import (
    DEFAULT_BITE_ORDERING,
    PREFERENCE_BUNDLE,
)
from feeding_deployment.preference_learning.data_generation.generate_deployment_dataset_llm import (
    apply_hard_rules,
    get_meal_info,
)

# The dims a human decides each day, in the order the real webapp asks them
# during a meal (initial ask -> microwave ask -> table ask; see
# emulate_preference_pipeline.py). Continuous dims (color/nav_offset) are
# excluded on purpose -- see module docstring.
from feeding_deployment.integration.preference_session import (
    INITIAL_PREF_DIMS,
    TABLE_PREF_DIMS,
)

_ASK_PHASES = [
    ("AT MEAL START (before fetching the plate)", list(INITIAL_PREF_DIMS)),
    ("MICROWAVE ASK", ["microwave_time"]),
    ("AT THE TABLE (before feeding begins)", list(TABLE_PREF_DIMS)),
]
_DIM_BY_FIELD = {dim.field: dim for dim in PREFERENCE_BUNDLE}
MANUAL_DIMS = [_DIM_BY_FIELD[f] for _, fields in _ASK_PHASES for f in fields]
# Phase title shown when the session reaches each dim (parallel to MANUAL_DIMS).
MANUAL_PHASE_BY_FIELD = {f: title for title, fields in _ASK_PHASES for f in fields}
assert {d.field for d in MANUAL_DIMS} == {
    d.field for d in PREFERENCE_BUNDLE if d.kind in ("categorical", "text")
}, "webapp ask order and PREFERENCE_BUNDLE manual dims diverged"

# The first days of every manual run use this fixed context sequence instead
# of the seeded random draw ("night" is not in TIMES_OF_DAY; evening is the
# closest catalog value). The transient affective state is still drawn from
# the seeded rng. Later days go back to random sampling.
FIXED_FIRST_CONTEXTS: List[Dict[str, str]] = [
    {"meal": "chicken nuggets", "setting": "Personal", "time_of_day": "evening"},
    {"meal": "general tso's chicken and broccoli", "setting": "Watching TV with TV in Front", "time_of_day": "noon"},
    {"meal": "chicken breast strips and hash brown", "setting": "Watching TV with TV in Front", "time_of_day": "noon"},
]
for _ctx in FIXED_FIRST_CONTEXTS:
    assert _ctx["meal"] in MEALS, f"fixed meal not in catalog: {_ctx['meal']!r}"
    assert _ctx["setting"] in SETTINGS, f"fixed setting not in catalog: {_ctx['setting']!r}"
    assert _ctx["time_of_day"] in TIMES_OF_DAY, f"fixed time_of_day not in catalog: {_ctx['time_of_day']!r}"

_BACK = object()


def _print_context_banner(day: int, days: int, context: Dict[str, str], meal_info: Dict[str, Any]) -> None:
    print("\n" + "=" * 72)
    print(f"DAY {day}/{days}")
    print("=" * 72)
    print(f"  Meal:            {context['meal']}")
    print(f"  Dippable items:  {', '.join(meal_info['dippable_items']) or 'None'}")
    print(f"  Sauces:          {', '.join(meal_info['sauces']) or 'None'}")
    print(f"  Storage:         {meal_info['storage_condition']} | serve {meal_info['intended_serving_temp']}")
    print(f"  Setting:         {context['setting']}")
    print(f"  Time of day:     {context['time_of_day']}")
    print(f"  Affective state: {context['transient_affective_state']}")
    print("=" * 72)


def _ask_dim(dim, index: int, total: int, default: Optional[str], options: Optional[List[str]] = None):
    """Ask one dim. Returns the chosen value, or _BACK. ``options`` overrides
    dim.options (used to hide choices a hard rule would reject)."""
    if options is None:
        options = dim.options
    if default is not None and dim.kind != "text" and default not in options:
        default = None
    print(f"\n[{index}/{total}] {dim.label}")
    if dim.short_description:
        print(f"    {dim.short_description}")

    if dim.kind == "text":
        shown_default = default if default is not None else DEFAULT_BITE_ORDERING
        print(f"    (free text; Enter = {shown_default!r}, b = back, ? = help)")
        while True:
            try:
                line = input("  > ").strip()
            except EOFError:
                raise KeyboardInterrupt
            if line == "?":
                print(f"    {dim.description}")
                continue
            if line == "b":
                return _BACK
            return line if line else shown_default

    for i, opt in enumerate(options, start=1):
        marker = " (default)" if opt == default else ""
        print(f"    {i}) {opt}{marker}")
    hint = "Enter = default, " if default is not None else ""
    print(f"    ({hint}b = back, ? = help)")
    while True:
        try:
            line = input("  > ").strip()
        except EOFError:
            raise KeyboardInterrupt
        if line == "?":
            print(f"    {dim.description}")
            continue
        if line == "b":
            return _BACK
        if line == "" and default is not None:
            return default
        if line.isdigit() and 1 <= int(line) <= len(options):
            return options[int(line) - 1]
        print(f"    Invalid input. Enter 1-{len(options)}" + (", or press Enter for the default." if default is not None else "."))


def collect_day_preferences(
    meal_info: Dict[str, Any], prev_choices: Optional[Dict[str, str]]
) -> Tuple[Dict[str, str], Dict[str, str]]:
    """Walk the human through the manual dims for one day."""
    choices: Dict[str, str] = {}
    rationales: Dict[str, str] = {}
    total = len(MANUAL_DIMS)
    asked: List[int] = []  # indices of dims that were actually asked, in order

    i = 0
    shown_phase: Optional[str] = None
    while i < len(MANUAL_DIMS):
        dim = MANUAL_DIMS[i]

        phase = MANUAL_PHASE_BY_FIELD[dim.field]
        if phase != shown_phase:
            print(f"\n----- {phase} -----")
            shown_phase = phase

        # Auto-fills the hard rules would force anyway -- skip the question.
        if dim.field == "outside_mouth_distance" and choices.get("transfer_mode") == "inside mouth transfer":
            choices[dim.field] = "not applicable"
            rationales[dim.field] = "manual: auto (inside mouth transfer)"
            i += 1
            continue
        if dim.field == "bite_dipping_preference" and not (meal_info["has_dippable"] and meal_info["has_sauce"]):
            choices[dim.field] = "do not dip"
            rationales[dim.field] = "manual: auto (meal has no dippable items or no sauces)"
            i += 1
            continue

        options = None
        if dim.field == "outside_mouth_distance" and choices.get("transfer_mode") == "outside mouth transfer":
            options = [o for o in dim.options if o != "not applicable"]

        default = prev_choices.get(dim.field) if prev_choices else None
        ans = _ask_dim(dim, i + 1, total, default, options)
        if ans is _BACK:
            if asked:
                i = asked.pop()
            else:
                print("    (already at the first question)")
            continue
        choices[dim.field] = ans
        rationales[dim.field] = "manual"
        asked.append(i)
        i += 1

    # Same guardrails as the LLM generator, in case a rule became violated
    # after the auto-fill point (e.g. user went back and changed transfer_mode).
    after = apply_hard_rules(choices, meal_info)
    for k, v_after in after.items():
        v_before = choices.get(k)
        if v_before != v_after:
            rationales[k] = (rationales.get(k, "") + f" [HARD RULE OVERRIDE: {v_before!r} -> {v_after!r}]").strip()
    return after, rationales


def _confirm_day(choices: Dict[str, str], prev_choices: Optional[Dict[str, str]]) -> bool:
    print("\n--- Day summary (* = changed from previous day) ---")
    for dim in MANUAL_DIMS:
        val = choices[dim.field]
        changed = "*" if prev_choices is not None and prev_choices.get(dim.field) != val else " "
        print(f"  {changed} {dim.field}: {val}")
    try:
        line = input("Accept day? (Enter = accept, r = redo) > ").strip().lower()
    except EOFError:
        raise KeyboardInterrupt
    return line != "r"


def run_deployment(
    user_name: str,
    deployment_id: str,
    physical_profile_label: str,
    seed: Optional[int],
    days: int,
    output_dir: str,
    output_filename: Optional[str] = None,
) -> str:
    """Interactive counterpart of the LLM run_deployment: same context draws,
    same resume-in-place behavior, same output schema (model="manual")."""
    rng = random.Random(seed)

    os.makedirs(output_dir, exist_ok=True)
    out_path = os.path.join(output_dir, output_filename or f"{user_name}__{deployment_id}__{days}d.json")

    existing_by_day: Dict[int, Dict[str, Any]] = {}
    if os.path.exists(out_path):
        try:
            with open(out_path, "r", encoding="utf-8") as f:
                prev = json.load(f)
        except Exception as e:
            raise ValueError(
                f"Existing output file is unreadable ({e}): {out_path}. "
                "Delete or move it to regenerate from scratch."
            )
        prev_cfg = prev.get("config", {}) or {}
        for key, ours, theirs in (
            ("user", user_name, str(prev.get("user"))),
            ("seed", seed, prev_cfg.get("seed")),
            ("model", "manual", prev_cfg.get("model")),
        ):
            if theirs != ours:
                raise ValueError(
                    f"Cannot resume {out_path}: recorded {key}={theirs!r} does not match "
                    f"this run's {key}={ours!r}. Use a fresh output dir or matching flags."
                )
        existing_by_day = {int(r["day"]): r for r in prev.get("days", [])}
        if existing_by_day:
            print(f"Resuming {os.path.basename(out_path)}: {len(existing_by_day)} day(s) already recorded.")

    day_records: List[Dict[str, Any]] = []

    def _flush() -> None:
        payload: Dict[str, Any] = {
            "user": user_name,
            "deployment_id": deployment_id,
            "physical_profile_label": physical_profile_label,
            "config": {"days": days, "seed": seed, "model": "manual"},
            "user_preference_encoding": {},
            "days": day_records,
        }
        tmp_path = out_path + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
        os.replace(tmp_path, out_path)

    prev_choices: Optional[Dict[str, str]] = None

    for day in range(1, days + 1):
        # Context draws happen unconditionally so the rng stream stays aligned
        # with the original run when resuming past recorded days.
        meal = rng.choice(MEALS)
        setting = rng.choice(SETTINGS)
        time_of_day = rng.choice(TIMES_OF_DAY)
        affective_state = rng.choice(AFFECTIVE_STATES)
        if day <= len(FIXED_FIRST_CONTEXTS):
            fixed = FIXED_FIRST_CONTEXTS[day - 1]
            meal, setting, time_of_day = fixed["meal"], fixed["setting"], fixed["time_of_day"]
        context = {
            "meal": meal,
            "setting": setting,
            "time_of_day": time_of_day,
            "transient_affective_state": affective_state,
        }

        if day in existing_by_day:
            rec = existing_by_day[day]
            if seed is not None and (rec.get("context", {}) or {}) != context:
                raise ValueError(
                    f"Resume mismatch on day {day}: recorded context {rec.get('context')} != "
                    f"re-derived context {context}. The seed or catalogs changed since "
                    "the original run; use a fresh output dir."
                )
            day_records.append(rec)
            prev_choices = {
                f: (v.get("choice") if isinstance(v, dict) else v)
                for f, v in (rec.get("preferences", {}) or {}).items()
                if f in {d.field for d in MANUAL_DIMS}
            }
            print(f"=== Day {day}/{days} (already recorded, skipping) ===")
            continue

        meal_info = get_meal_info(meal)
        while True:
            _print_context_banner(day, days, context, meal_info)
            choices, rationales = collect_day_preferences(meal_info, prev_choices)
            if _confirm_day(choices, prev_choices):
                break
            print("Redoing day ...")

        day_records.append(
            {
                "day": day,
                "context": context,
                "preferences": {
                    field: {"choice": choices[field], "rationale": rationales.get(field, "")}
                    for field in sorted(choices.keys())
                },
            }
        )
        prev_choices = choices
        _flush()
        print(f"=== Day {day} recorded (written to disk) ===")

    _flush()
    return out_path


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual (human-in-the-loop) dataset generator (deployment).")
    parser.add_argument("--user", required=True, help="User name used in the output payload and filename.")
    parser.add_argument("--physical-profile", default="manual", help="Physical profile label to record (default: manual).")
    parser.add_argument("--deployment-id", default="dep1", help="Deployment identifier")
    parser.add_argument("--seed", type=int, default=None, help="Random seed for the context schedule (match an LLM run's seed to get identical contexts).")
    parser.add_argument("--days", type=int, default=30, help="Number of days (default: 30)")
    parser.add_argument(
        "--output-dir",
        default="out_manual",
        help="Base output directory (default: out_manual). A run timestamp is appended "
        "(<output-dir>_run_<ts>) so existing generated data is never overwritten.",
    )
    parser.add_argument(
        "--no-timestamp",
        action="store_true",
        help="Write into --output-dir exactly as given (required to resume an interrupted run).",
    )
    parser.add_argument("--output-filename", default=None, help="Output filename (default: <user>__<deployment-id>__<days>d.json)")
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)

    out_dir = args.output_dir
    if not args.no_timestamp:
        out_dir = f"{out_dir.rstrip('/')}_run_{datetime.now().strftime('%Y_%m_%d__%H_%M_%S')}"
    out_dir = os.path.abspath(out_dir)

    try:
        out_path = run_deployment(
            user_name=args.user,
            deployment_id=args.deployment_id,
            physical_profile_label=args.physical_profile,
            seed=args.seed,
            days=args.days,
            output_dir=out_dir,
            output_filename=args.output_filename,
        )
    except KeyboardInterrupt:
        print(
            "\nInterrupted. All fully recorded days are on disk; resume with the same "
            f"flags plus: --output-dir {out_dir!r} --no-timestamp",
            file=sys.stderr,
        )
        return 130
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    print(f"\n✓ Done. Wrote: {out_path}")
    print(
        "Note: the 7 continuous dims (plate colors, nav offsets) are not in this "
        "dataset; inject them from a user's continuous_tables before running the evaluation."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
