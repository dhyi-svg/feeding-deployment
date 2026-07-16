"""Terminal emulation of the full preference-learning pipeline in run.py.

Walks the REAL per-meal pipeline — real ``PredictionModel`` (Claude + OpenAI
embeddings + cross-day memory on disk), real ``PreferenceSession``, real
behavior-tree YAML reads/writes, real ``DataLogger`` day record — through the
exact staged sequence of ``run.py``'s personalized meal, but with every robot
touchpoint replaced by a terminal prompt:

- meal context      -> terminal pickers (``terminal_collect_context``)
- ask pages         -> ``TerminalCorrectionInterface`` (stdin, incl. free-text)
- plate pickups     -> print the detection color from the pickup BT YAML and
                       accept an optional ``h,s,v[,range]`` correction, written
                       back to the YAML exactly like the on-robot color picker,
                       then ``session.record_color(location)``
- navigations       -> print the applied PositionOffset and accept an optional
                       ``dx dy dyaw`` teleop adjustment, composed onto the
                       total in SE(2) and written back to the YAML exactly like
                       the on-robot post-arrival adjustment, then
                       ``session.record_nav_offset(location)``
- microwave routing -> ``session.apply_microwave`` on a plain atom set; the
                       microwave leg (and its color correction) is skipped for
                       "no microwave", mirroring the planner detour

No robot, no ROS, no web interface, no PDDL planner, no PyBullet. The CLI and
the ``log/<user>/`` layout match run.py: one meal per invocation with a
mandatory ``--day N``; days must be sequential; a fresh run of an existing day
OVERWRITES that day's memory. Requires ``ANTHROPIC_API_KEY`` and
``OPENAI_API_KEY``.

Out of scope (web-interface features, not preference-pipeline logic): the
settings overlay, checkpoint/resume, and the real bite-selection/transfer
pages. ``flair=None`` means bite_ordering is still predicted/asked/learned but
the free-text grammar cleanup and FLAIR side effects are skipped.

Example:
    python -m feeding_deployment.integration.emulate_preference_pipeline \
        --user stresstest --day 1
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
from pathlib import Path
from types import SimpleNamespace

from feeding_deployment.integration.apply_preferences import (
    _load_yaml,
    _save_yaml,
    _set_param_value,
)
from feeding_deployment.integration.data_logger import DataLogger
from feeding_deployment.integration.preference_context import build_preference_context
from feeding_deployment.integration.preference_session import (
    DEFAULT_PHYSICAL_PROFILE,
    INITIAL_PREF_DIMS,
    TABLE_PREF_DIMS,
    PreferenceSession,
    _NAV_OFFSET_PARAM,
    _nav_yaml_name,
    _pickup_yaml_name,
)
from feeding_deployment.integration.terminal_preferences import (
    TerminalCorrectionInterface,
    terminal_collect_context,
)
from feeding_deployment.preference_learning.config.preference_bundle import (
    NAV_OFFSET_BOUNDS,
    color_from_bt,
    color_to_bt,
    format_color,
    format_nav_offset,
    nav_offset_from_bt,
    parse_color,
)
from feeding_deployment.preference_learning.methods.prediction_model import (
    DEFAULT_MEMORY_MODE,
    MEMORY_MODES,
    PredictionModel,
)


# ---------------------------------------------------------------------------
# YAML helpers (the emulated "skill write-back" side of the pipeline)
# ---------------------------------------------------------------------------

def _read_param(bt_path: Path, name: str):
    data = _load_yaml(bt_path)
    for param in data.get("parameters", []):
        if param.get("name") == name:
            return param.get("value")
    return None


def _write_params(bt_path: Path, values: dict) -> None:
    """Set parameter values in a BT YAML, upserting PositionOffset if absent
    (mirrors PreferenceSession._write_nav_offset_to_bt for pre-offset trees)."""
    data = _load_yaml(bt_path)
    for name, value in values.items():
        if not _set_param_value(data, name, value):
            if name == "PositionOffset":
                data.setdefault("parameters", []).append(
                    {**_NAV_OFFSET_PARAM, "value": value}
                )
            else:
                raise KeyError(f"Parameter {name!r} not found in {bt_path.name}")
    _save_yaml(bt_path, data)


# ---------------------------------------------------------------------------
# SE(2) math (mirrors NavigateHLA._se2_compose/_wrap without importing the
# ROS-facing action stack)
# ---------------------------------------------------------------------------

def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _se2_compose(x: float, y: float, yaw: float, dx: float, dy: float, dyaw: float):
    """(x,y,yaw) o (dx,dy,dyaw): apply a local-frame motion to an SE(2) pose."""
    c, s = math.cos(yaw), math.sin(yaw)
    return x + dx * c - dy * s, y + dx * s + dy * c, _wrap(yaw + dyaw)


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


# ---------------------------------------------------------------------------
# Terminal touchpoints
# ---------------------------------------------------------------------------

def _emulate_pickup_color(session: PreferenceSession, bt_dir: Path, location: str) -> None:
    """Emulate the plate pickup at ``location``: show the detection color the
    session wrote into the pickup BT YAML, optionally take a user correction
    (as the on-robot color picker would), write it back, and record the dim."""
    # Pre-skill join, mirroring run.py: pick_plate_* consumes predictions, so
    # settle any in-flight background reprediction before this "BT" reads its
    # YAML (and before the user's correction can race the worker's writes).
    session.wait_for_reprediction()
    bt_path = bt_dir / _pickup_yaml_name(location)
    current = color_from_bt(
        _read_param(bt_path, "HandleColor"), _read_param(bt_path, "ColorRange")
    )
    print(f"\n=== [skill] pick_plate_from_{location} (emulated) ===")
    print(f"  Detecting handle with color {format_color(current)}")
    # confirm_manipulation preference: "no" skips the detection page entirely
    # (the color picker is unreachable, exactly as on-robot); auto-continue is
    # only annotated -- a terminal prompt has no real countdown.
    confirm_mode = str(session.bundle.get("confirm_manipulation", "yes (without any auto-continue)"))
    if confirm_mode == "no":
        print("  (detection page skipped: manipulation confirmation is 'no')")
        session.record_color(location)
        return
    if confirm_mode == "yes (with auto-continue countdown)":
        print(f"  (page would auto-confirm after {session.wait_seconds:.0f}s on-robot)")
    while True:
        raw = input(
            "  [Enter] = detection confirmed  |  h,s,v[,range] = corrected color: "
        ).strip()
        if not raw:
            break
        parts = [p.strip() for p in raw.replace(";", ",").split(",") if p.strip()]
        if len(parts) not in (3, 4):
            print("  Invalid input. Enter 3 or 4 comma-separated numbers (h,s,v[,range]).")
            continue
        try:
            corrected = {"h": float(parts[0]), "s": float(parts[1]), "v": float(parts[2])}
            if len(parts) == 4:
                corrected["range"] = float(parts[3])
        except ValueError:
            print("  Invalid number. Try again.")
            continue
        corrected = parse_color(corrected, seed=current)  # clip into valid ranges
        handle_color, color_range = color_to_bt(corrected)
        _write_params(bt_path, {"HandleColor": handle_color, "ColorRange": color_range})
        print(f"  Corrected color written to {bt_path.name}: {format_color(corrected)}")
        break
    session.record_color(location)


def _emulate_navigation(session: PreferenceSession, bt_dir: Path, location: str) -> None:
    """Emulate a navigation to ``location``: show the PositionOffset applied to
    the goal, optionally take a teleop adjustment (composed onto the total in
    the arrived pose's local frame, as on-robot), write it back, and record the
    dim."""
    # Pre-skill join, mirroring run.py (navigate_to_* consumes predictions).
    session.wait_for_reprediction()
    bt_path = bt_dir / _nav_yaml_name(location)
    prev = nav_offset_from_bt(_read_param(bt_path, "PositionOffset"))
    print(f"\n=== [skill] navigate_to_{location} (emulated) ===")
    print(f"  Applied learned offset to goal: {format_nav_offset(prev)}")
    # confirm_navigation_arrival preference: "no" skips the arrival page (the
    # offset stays frozen, exactly as on-robot); auto-continue is annotated only.
    confirm_mode = str(session.bundle.get("confirm_navigation_arrival", "yes (with auto-continue countdown)"))
    if confirm_mode == "no":
        print("  (arrival page skipped: navigation confirmation is 'no')")
        session.record_nav_offset(location)
        return
    if confirm_mode == "yes (with auto-continue countdown)":
        print(f"  (page would auto-accept after {session.wait_seconds:.0f}s on-robot)")
    while True:
        raw = input(
            "  [Enter] = position OK  |  dx dy dyaw = teleop adjustment (m, m, rad): "
        ).strip()
        if not raw:
            break
        parts = raw.replace(",", " ").split()
        if len(parts) != 3:
            print("  Invalid input. Enter 3 numbers (dx dy dyaw).")
            continue
        try:
            dx, dy, dyaw = (float(p) for p in parts)
        except ValueError:
            print("  Invalid number. Try again.")
            continue
        # New TOTAL = previous total composed with the user's local-frame
        # motion (the emulated robot parks exactly at nominal o prev, so this
        # matches the on-robot measure-against-nominal accumulation).
        total = _se2_compose(prev["dx"], prev["dy"], prev["dyaw"], dx, dy, dyaw)
        clamped = [
            _clamp(total[0], NAV_OFFSET_BOUNDS["dx"]),
            _clamp(total[1], NAV_OFFSET_BOUNDS["dy"]),
            _clamp(total[2], NAV_OFFSET_BOUNDS["dyaw"]),
        ]
        if clamped != list(total):
            print("  NOTE: total offset saturated at the +/-0.5 m / +/-45 deg bounds.")
        _write_params(bt_path, {"PositionOffset": clamped})
        print(
            f"  New total offset written to {bt_path.name}: "
            f"dx={clamped[0]:+.3f},dy={clamped[1]:+.3f},dyaw={clamped[2]:+.3f}"
        )
        break
    session.record_nav_offset(location)


def _print_explanations(session: PreferenceSession) -> None:
    """Compact dump of the model's per-open-dim reasons from the most recent
    (re)prediction, so the stress tester can judge the reasoning live."""
    # Corrections repredict in the background; settle the worker so the
    # printed reasoning reflects the corrections just made.
    session.wait_for_reprediction()
    latent = getattr(session, "last_latent_inference", "") or ""
    expl = getattr(session, "last_explanations", None) or {}
    if latent:
        print(f"  [why] latent-factor inference: {latent}")
    if not expl:
        return
    print("  [why] model reasoning for open dims:")
    for field, why in expl.items():
        print(f"    - {field}: {why}")


def _emulate_feeding_menu() -> None:
    """Minimal stand-in for the task-selection loop: bite/sip/wipe are no-ops
    (their pages are web-interface features, not preference-pipeline logic);
    'finish' proceeds to the plate-away sequence + memory update."""
    print("\n=== Feeding (emulated task selection) ===")
    while True:
        choice = input("  Task [bite / sip / wipe / finish]: ").strip().lower()
        if choice in ("bite", "sip", "wipe"):
            print(f"  (robot would perform a {choice} transfer using the finalized preferences)")
        elif choice == "finish":
            return
        else:
            print("  Enter one of: bite, sip, wipe, finish.")


# ---------------------------------------------------------------------------
# Setup (mirrors _Runner.__init__ / _build_prediction_model in run.py)
# ---------------------------------------------------------------------------

def _seed_user_dir(log_dir: Path) -> None:
    """New user: seed persistent state from the factory defaults, exactly like
    run.py, so a later on-robot run of the same user finds the expected tree."""
    if log_dir.exists():
        return
    os.makedirs(log_dir, exist_ok=True)
    package_dir = Path(__file__).parents[1]

    bt_dir = log_dir / "behavior_trees"
    bt_dir.mkdir(exist_ok=True)
    original_bt_dir = package_dir / "actions" / "behavior_trees"
    assert original_bt_dir.exists()
    for original in original_bt_dir.glob("*.yaml"):
        shutil.copy(original, bt_dir)

    gestures_dir = log_dir / "gesture_detectors"
    gestures_dir.mkdir(exist_ok=True)
    original_gestures = (
        package_dir / "perception" / "gestures_perception" / "synthesized_gesture_detectors.py"
    )
    assert original_gestures.exists()
    shutil.copy(original_gestures, gestures_dir)


def _build_prediction_model(
    user: str, profile: str, log_dir: Path, day: int,
    memory_mode: str = DEFAULT_MEMORY_MODE,
) -> PredictionModel:
    model = PredictionModel(
        user=user,
        physical_profile_label="deployment_physical_profile",
        logs_dir=log_dir / "preference_learning",
        physical_profile_description=profile,
        memory_mode=memory_mode,
    )
    model.validate_sequential_day(day)
    model.load_prior_memory(day)
    return model


# ---------------------------------------------------------------------------
# One emulated meal
# ---------------------------------------------------------------------------

def run_emulated_meal(session: PreferenceSession, bt_dir: Path, day: int) -> dict:
    session.start()
    print(
        "Predicted preference bundle (initial):",
        json.dumps(session._loggable_bundle(), indent=2),
    )
    _print_explanations(session)
    session.ask(INITIAL_PREF_DIMS)
    _print_explanations(session)

    # Prep: microwave-front start -> fridge -> pick plate (fridge color) ->
    # holder -> close fridge (mirrors run.py's PlacePlateOnHolder + CloseDoor).
    _emulate_navigation(session, bt_dir, "fridge")
    print("\n=== [skill] open_fridge (emulated) ===")
    _emulate_pickup_color(session, bt_dir, "fridge")
    print("=== [skill] place_plate_on_holder / close_fridge (emulated) ===")

    # Microwave preference -> planner routing (plain set + string atom: the
    # helper only add()/discard()s, so any hashable stands in for the
    # FoodHeated GroundAtom).
    session.ask(["microwave_time"])
    _print_explanations(session)
    atoms: set = set()
    duration = session.apply_microwave(atoms, "FoodHeated")
    if duration is None:
        print("Microwave preference: no microwave (planner would skip the microwave leg).")
    else:
        print(f"Microwave preference: {duration:.0f}s (planner would route through the microwave).")
        _emulate_navigation(session, bt_dir, "microwave")
        print("\n=== [skill] open_microwave / place_plate / press_button (emulated) ===")
        _emulate_pickup_color(session, bt_dir, "microwave")
        print("=== [skill] close_microwave (emulated) ===")

    # To the table; table dims just before feeding.
    _emulate_navigation(session, bt_dir, "table")
    print("=== [skill] place_plate_on_table (emulated) ===")
    session.ask(TABLE_PREF_DIMS)
    _print_explanations(session)

    _emulate_feeding_menu()

    # Finish: pick plate from table (table color) -> sink (mirrors
    # PlacePlateInSink planning PickPlateFromTable first).
    _emulate_pickup_color(session, bt_dir, "table")
    _emulate_navigation(session, bt_dir, "sink")
    print("=== [skill] place_plate_in_sink (emulated) ===")

    print(f"\n[learn] Updating memory models (day {day}) ...")
    ground_truth = session.finalize_meal(day)
    print(f"[learn] Memory update complete (day {day}).")
    return ground_truth


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Terminal emulation of run.py's preference-learning pipeline "
            "(no robot, no ROS, no web interface). One meal per invocation, "
            "day-by-day, with run.py's log/<user>/ layout."
        )
    )
    parser.add_argument("--user", type=str, default="", help="Name of the user (as in run.py).")
    parser.add_argument(
        "--day", type=int, required=True,
        help="Deployment day number (mandatory; sequential, no gaps; a fresh "
             "run of an existing day OVERWRITES that day's memory).",
    )
    parser.add_argument(
        "--physical_profile_file", type=str, default="",
        help="UTF-8 text file describing the user's physical capabilities "
             "(defaults to the deployment default profile).",
    )
    parser.add_argument("--pref_meal", type=str, default="",
                        help="Optional preset meal label (skips the terminal context pickers).")
    parser.add_argument("--pref_setting", type=str, default="Personal")
    parser.add_argument("--pref_time_of_day", type=str, default="morning")
    parser.add_argument(
        "--pref_memory_mode", type=str, choices=list(MEMORY_MODES), default=DEFAULT_MEMORY_MODE,
        help="Cross-day preference memory backend (mirrors run.py's "
             "--pref_memory_mode). Default: %(default)s.",
    )
    args = parser.parse_args()

    if args.user == "":
        raise ValueError("Please provide a user name.")

    # OpenAI is only used for episodic-retrieval embeddings, i.e. three_layer mode.
    required_keys = ["ANTHROPIC_API_KEY"]
    if args.pref_memory_mode == "three_layer":
        required_keys.append("OPENAI_API_KEY")
    for key in required_keys:
        if not os.environ.get(key, "").strip():
            print(f"ERROR: {key} is not set. The emulator makes real LLM/embedding calls.")
            return 1

    if args.physical_profile_file.strip():
        profile_path = Path(args.physical_profile_file.strip())
        if not profile_path.is_file():
            raise ValueError(f"physical profile file not found: {profile_path}")
        profile = profile_path.read_text(encoding="utf-8").strip()
        if not profile:
            raise ValueError(f"physical profile file is empty: {profile_path}")
    else:
        profile = DEFAULT_PHYSICAL_PROFILE

    log_dir = Path(__file__).parent / "log" / args.user
    _seed_user_dir(log_dir)
    bt_dir = log_dir / "behavior_trees"
    data_logger = DataLogger(state_dir=log_dir, day=args.day)

    try:
        if args.pref_meal.strip():
            context = build_preference_context(
                meal=args.pref_meal.strip(),
                setting=args.pref_setting,
                time_of_day=args.pref_time_of_day,
            )
        else:
            ctx = terminal_collect_context()
            context = build_preference_context(
                meal=ctx["meal"], setting=ctx["setting"], time_of_day=ctx["time_of_day"]
            )
        print("Preference context (meal / setting / time_of_day):", context)

        model = _build_prediction_model(
            args.user, profile, log_dir, args.day, memory_mode=args.pref_memory_mode
        )
        existing = model.working_memory_dir / f"day_{args.day:04d}.json"
        if existing.exists():
            print(
                f"[learn] NOTE: day {args.day} memory already exists; finalizing will "
                f"OVERWRITE day_{args.day:04d}.json (working memory plus this mode's "
                f"cross-day memory)."
            )

        session = PreferenceSession(
            model,
            bt_dir,
            dict(context),
            web_interface=TerminalCorrectionInterface(),
            data_logger=data_logger,
            # Minimal stand-in so apply_transfer_mode's scene update runs; with
            # hla_map={} there is no transfer object to reconstruct.
            scene_description=SimpleNamespace(transfer_type="outside"),
            hla_map={},
            flair=None,
        )

        ground_truth = run_emulated_meal(session, bt_dir, args.day)

        loggable = {
            k: (format_color(v) if isinstance(v, dict) and "h" in v
                else format_nav_offset(v) if isinstance(v, dict) and "dx" in v
                else v)
            for k, v in ground_truth.items()
        }
        print("\nFinalized ground-truth bundle:", json.dumps(loggable, indent=2))
        print(f"\nState written under: {log_dir}")
        print(f"  behavior trees:     {bt_dir}")
        print(f"  preference memory:  {log_dir / 'preference_learning' / args.user}")
        print(f"  day release record: {log_dir / f'day_{args.day:02d}'}")
        return 0
    finally:
        data_logger.close()


if __name__ == "__main__":
    raise SystemExit(main())
