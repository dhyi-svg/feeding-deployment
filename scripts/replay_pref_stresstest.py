"""Scripted, non-interactive replay of a recorded preference-learning deployment.

Reads a recorded user's per-day memory records (context + the corrections they
made) and replays the same meals through the REAL pipeline — real
PredictionModel (LLM + embeddings + cross-day memory), real PreferenceSession,
real BT-YAML writes — under a fresh replay user. Used to A/B prompt changes
against the exact input of a previous stress test.

For every day it walks run.py's staged order (initial dims -> fridge ->
microwave? -> table -> table dims -> finish) and:
- answers each ask step with the recorded desired value if one exists (a
  correction only when it differs from the live prediction), else accepts;
- at each plate pickup, writes the recorded desired color into the pickup YAML
  (simulating the color picker) and lets ``record_color`` finalize it;
- after each color correction, reports whether the still-open plate-color dims
  FOLLOWED the correction in the triggered reprediction (the within-meal
  correlation behavior under test), and whether explanations covered the open
  dims.

Requires ANTHROPIC_API_KEY and OPENAI_API_KEY. Roughly (1 + #corrections) LLM
calls per day plus one LTM call per day.

Example:
    PYTHONPATH=src python3 scripts/replay_pref_stresstest.py --wipe
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
from pathlib import Path
from types import SimpleNamespace

REPO_SRC = Path(__file__).resolve().parents[1] / "src"
if str(REPO_SRC) not in sys.path:
    sys.path.insert(0, str(REPO_SRC))

from feeding_deployment.integration.emulate_preference_pipeline import (  # noqa: E402
    _read_param,
    _seed_user_dir,
    _write_params,
)
from feeding_deployment.integration.preference_session import (  # noqa: E402
    DEFAULT_PHYSICAL_PROFILE,
    INITIAL_PREF_DIMS,
    PreferenceSession,
    TABLE_PREF_DIMS,
    _pickup_yaml_name,
)
from feeding_deployment.preference_learning.config.preference_bundle import (  # noqa: E402
    COLOR_FIELD_BY_LOCATION,
    color_to_bt,
    format_color,
    parse_color,
)
from feeding_deployment.preference_learning.methods.prediction_model import (  # noqa: E402
    PredictionModel,
)
from feeding_deployment.preference_learning.methods.utils import PREF_FIELDS  # noqa: E402

INTEGRATION_DIR = REPO_SRC / "feeding_deployment" / "integration"

_LOCATION_BY_COLOR_FIELD = {v: k for k, v in COLOR_FIELD_BY_LOCATION.items()}


class ScriptedCorrectionInterface:
    """Stepwise-correction stand-in: answers each ask with the recorded desired
    value when one exists (a correction only if it differs from the live
    prediction), else confirms the prediction."""

    def __init__(self, desired: dict) -> None:
        self.desired = dict(desired)

    def start_preference_correction(self, total: int, autocontinue_seconds: float) -> None:
        pass

    def send_preference_step(self, field, predicted, options, step, total,
                             autocontinue_seconds, kind="categorical"):
        return self.desired.get(field, predicted)

    def finish_preference_correction(self) -> None:
        pass


def load_scenario(source: Path) -> list[dict]:
    """Per-day {day, context, desired} from a recorded user's memory records.

    ``desired`` = the day's ``corrected`` dict (what the user ended up wanting),
    plus the recorded microwave_time (parsed from the episode text) so the
    replay routes through the microwave exactly like the recorded meal did.
    """
    wm_dirs = sorted((source / "preference_learning").glob("*/working_memory"))
    if not wm_dirs:
        raise SystemExit(f"No working_memory records under {source}/preference_learning/*/")
    wm_dir = wm_dirs[0]
    days = []
    for path in sorted(wm_dir.glob("day_*.json")):
        rec = json.loads(path.read_text(encoding="utf-8"))
        desired = dict(rec.get("corrected") or {})
        if "microwave_time" not in desired:
            ep_path = wm_dir.parent / "episodic_memory" / path.name
            if ep_path.exists():
                ep = json.loads(ep_path.read_text(encoding="utf-8"))
                m = re.search(r"microwave_time=([^;]+);", ep.get("episode_text", ""))
                if m:
                    desired["microwave_time"] = m.group(1).strip()
        days.append({"day": int(rec["day"]), "context": dict(rec["context"]), "desired": desired})
    return days


def _open_color_fields(session: PreferenceSession) -> list[str]:
    return [f for f in COLOR_FIELD_BY_LOCATION.values() if f not in session.finalized]


def _hsv(v) -> tuple:
    c = parse_color(v)
    return (c["h"], c["s"], c["v"])


def _replay_pickup(session: PreferenceSession, bt_dir: Path, location: str,
                   desired: dict, day: int, prop_rows: list, expl_log: list) -> None:
    """Emulate the pickup at ``location``: write the desired color (if any) into
    the YAML like the color picker would, record the dim, and measure whether
    the OTHER open color dims followed in the triggered reprediction."""
    field = COLOR_FIELD_BY_LOCATION[location]
    desired_color = desired.get(field)
    bt_path = bt_dir / _pickup_yaml_name(location)

    predicted = parse_color(session.bundle.get(field))
    others_before = {f: parse_color(session.bundle.get(f)) for f in _open_color_fields(session) if f != field}

    if desired_color is not None:
        canonical = parse_color(desired_color)
        handle_color, color_range = color_to_bt(canonical)
        _write_params(bt_path, {"HandleColor": handle_color, "ColorRange": color_range})
    session.record_color(location)
    # record_color schedules the propagation reprediction in the BACKGROUND;
    # settle it before reading the other open dims, or the rows below measure
    # the pre-repredict bundle (looks like zero propagation at every effort).
    session.wait_for_reprediction()

    is_correction = desired_color is not None and _hsv(desired_color) != _hsv(predicted)
    if is_correction:
        for f, before in others_before.items():
            if f in session.finalized:
                continue  # was finalized meanwhile; not an open dim
            after = parse_color(session.bundle.get(f))
            prop_rows.append({
                "day": day,
                "corrected_field": field,
                "corrected_to": format_color(desired_color),
                "open_field": f,
                "before": format_color(before),
                "after": format_color(after),
                "followed": _hsv(after) == _hsv(desired_color),
                "moved": _hsv(after) != _hsv(before),
            })
        _record_explanations(session, day, f"after {field} correction", expl_log)


def _record_explanations(session: PreferenceSession, day: int, stage: str, expl_log: list) -> None:
    expl = dict(session.last_explanations or {})
    open_dims = [f for f in PREF_FIELDS if f not in session.finalized]
    expl_log.append({
        "day": day,
        "stage": stage,
        "latent_inference": getattr(session, "last_latent_inference", ""),
        "n_open": len(open_dims),
        "n_explained": len([f for f in open_dims if f in expl]),
        "missing": [f for f in open_dims if f not in expl][:8],
        "explanations": expl,
    })


def replay_day(user: str, log_dir: Path, day_rec: dict, profile: str,
               prop_rows: list, expl_log: list) -> None:
    day, context, desired = day_rec["day"], day_rec["context"], day_rec["desired"]
    bt_dir = log_dir / "behavior_trees"

    model = PredictionModel(
        user=user,
        physical_profile_label="deployment_physical_profile",
        logs_dir=log_dir / "preference_learning",
        physical_profile_description=profile,
    )
    model.validate_sequential_day(day)
    model.load_prior_memory(day)

    session = PreferenceSession(
        model, bt_dir, dict(context),
        web_interface=ScriptedCorrectionInterface(desired),
        data_logger=None,
        scene_description=SimpleNamespace(transfer_type="outside"),
        hla_map={}, flair=None,
    )

    print(f"\n===== Replay day {day}: {context['meal']!r} / {context['setting']} / "
          f"{context['time_of_day']} | scripted dims: {sorted(desired)} =====")
    session.start()
    _record_explanations(session, day, "start", expl_log)

    session.ask(INITIAL_PREF_DIMS)
    session.record_nav_offset("fridge")
    _replay_pickup(session, bt_dir, "fridge", desired, day, prop_rows, expl_log)

    session.ask(["microwave_time"])
    duration = session.apply_microwave(set(), "FoodHeated")
    if duration is not None:
        session.record_nav_offset("microwave")
        _replay_pickup(session, bt_dir, "microwave", desired, day, prop_rows, expl_log)

    session.record_nav_offset("table")
    session.ask(TABLE_PREF_DIMS)
    _replay_pickup(session, bt_dir, "table", desired, day, prop_rows, expl_log)
    session.record_nav_offset("sink")

    session.finalize_meal(day)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--scenario-from", type=Path,
                        default=INTEGRATION_DIR / "log" / "stresstest",
                        help="Recorded user log dir to replay (default: log/stresstest).")
    parser.add_argument("--user", type=str, default="stresstest_replay")
    parser.add_argument("--wipe", action="store_true",
                        help="Delete the replay user's log dir first (days must restart at 1).")
    parser.add_argument("--physical_profile_file", type=str, default="")
    args = parser.parse_args()

    for key in ("ANTHROPIC_API_KEY", "OPENAI_API_KEY"):
        if not os.environ.get(key, "").strip():
            print(f"ERROR: {key} is not set.")
            return 1

    profile = DEFAULT_PHYSICAL_PROFILE
    if args.physical_profile_file.strip():
        profile = Path(args.physical_profile_file).read_text(encoding="utf-8").strip()

    log_dir = INTEGRATION_DIR / "log" / args.user
    if log_dir.exists():
        if not args.wipe:
            raise SystemExit(f"{log_dir} exists; pass --wipe to start the replay from day 1.")
        shutil.rmtree(log_dir)
    _seed_user_dir(log_dir)

    scenario = load_scenario(args.scenario_from)
    print(f"Loaded {len(scenario)} recorded day(s) from {args.scenario_from}")

    prop_rows: list = []
    expl_log: list = []
    for day_rec in scenario:
        replay_day(args.user, log_dir, day_rec, profile, prop_rows, expl_log)

    # ------------------------------------------------------------------ #
    # Report
    # ------------------------------------------------------------------ #
    print("\n================ PROPAGATION REPORT (open color dims after a color correction) ================")
    followed = 0
    for r in prop_rows:
        print(f"  day {r['day']}: {r['corrected_field']} -> {r['corrected_to']} | "
              f"{r['open_field']}: {r['before']} -> {r['after']} | "
              f"followed={r['followed']} moved={r['moved']}")
        followed += bool(r["followed"])
    if prop_rows:
        print(f"  SUMMARY: followed {followed}/{len(prop_rows)} "
              f"(moved {sum(bool(r['moved']) for r in prop_rows)}/{len(prop_rows)})")
    else:
        print("  (no color-correction repredictions occurred)")

    print("\n================ EXPLANATIONS COVERAGE ================")
    for e in expl_log:
        print(f"  day {e['day']} [{e['stage']}]: {e['n_explained']}/{e['n_open']} open dims explained"
              + (f" | missing: {e['missing']}" if e["missing"] else ""))

    report_path = log_dir / "replay_report.json"
    report_path.write_text(json.dumps(
        {"propagation": prop_rows, "explanations": expl_log}, indent=2), encoding="utf-8")
    print(f"\nFull report (incl. all explanation texts): {report_path}")

    # Any unparseable LLM responses were logged with this marker.
    calls_dir = log_dir / "preference_learning" / args.user / "prediction_model_llm_calls"
    bad = [p.name for p in calls_dir.glob("*.txt")
           if "Failed to parse" in p.read_text(encoding="utf-8")]
    if bad:
        print(f"ERROR: {len(bad)} LLM response(s) failed to parse: {bad}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
