#!/usr/bin/env python3
"""Compute total feeding time for deployment days.

    total feeding time = (meal window) - (union of researcher-marked intervals)

The researcher-marked intervals (interventions / explanations) come from
``researcher_events.jsonl`` in each day directory, written by the researcher
timer web tool (``researcher_timer.py``, port 8081). They are events *outside*
the system -- a human stepping in or explaining -- so they are stamped manually
and subtracted from the wall-clock meal window.

The meal window comes from the researcher's own Start Meal / Finish Feeding
marks when they exist (they know when the meal really began and ended). Either
boundary the researcher did not mark falls back to the timestamps in the day
directory rather than trusting ``metadata.json`` alone: on days with several
run.py sessions each restart overwrites ``started``, so metadata can even end
up with ``ended`` earlier than ``started``. The fallback takes the earliest /
latest of metadata start/end, ``events.jsonl`` and ``user_inputs.jsonl``.

Intentionally stdlib-only so it runs anywhere the logs are copied to (analysis
laptops without the robot environment). ``researcher_timer.py`` imports the
parsing helpers from here -- this module is the single source of truth for the
researcher-interval format.

Usage:
    python compute_feeding_time.py log/<user>/day_01 [more day dirs ...]
    python compute_feeding_time.py --user <user>            # all days
    python compute_feeding_time.py --user <user> --day 3
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

LOG_ROOT = Path(__file__).resolve().parent / "log"
RESEARCHER_EVENTS_FILENAME = "researcher_events.jsonl"

# researcher_events.jsonl record categories (written by researcher_timer.py)
CATEGORY_INTERVAL = "researcher_interval"
CATEGORY_DELETED = "researcher_interval_deleted"
CATEGORY_MEAL = "researcher_meal"  # phase: "start" | "end"
INTERVAL_KINDS = ("intervention", "explanation")


def _read_jsonl(path: Path) -> list[dict]:
    """Best-effort JSONL reader: skips unparseable / non-dict lines."""
    records = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rec = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if isinstance(rec, dict):
                    records.append(rec)
    except OSError:
        pass
    return records


def load_researcher_intervals(day_dir: Path) -> tuple[list[dict], list[str]]:
    """Parse researcher_events.jsonl into intervals.

    Returns ``(intervals, warnings)`` where each interval is
    ``{"id", "kind", "start", "end", "note"}`` (``end`` is None while the
    interval is still open), sorted by start time. Tombstoned (deleted)
    intervals are excluded. ``warnings`` lists structural anomalies worth a
    human look (end without start, duplicate ids, ...).
    """
    records = _read_jsonl(Path(day_dir) / RESEARCHER_EVENTS_FILENAME)
    deleted = {r.get("id") for r in records if r.get("category") == CATEGORY_DELETED}
    by_id: dict[Any, dict] = {}
    warnings: list[str] = []

    for rec in records:
        if rec.get("category") != CATEGORY_INTERVAL:
            continue
        rid, phase, epoch = rec.get("id"), rec.get("phase"), rec.get("epoch")
        if rid is None or not isinstance(epoch, (int, float)):
            warnings.append(f"malformed interval record skipped: {rec}")
            continue
        if phase == "start":
            if rid in by_id:
                warnings.append(f"duplicate start for interval id {rid}; keeping first")
                continue
            by_id[rid] = {
                "id": rid,
                "kind": rec.get("kind", "intervention"),
                "start": float(epoch),
                "end": None,
                "note": rec.get("note") or "",
            }
        elif phase == "end":
            interval = by_id.get(rid)
            if interval is None:
                warnings.append(f"end without start for interval id {rid}; skipped")
                continue
            if interval["end"] is not None:
                warnings.append(f"duplicate end for interval id {rid}; keeping first")
                continue
            interval["end"] = float(epoch)
            if rec.get("note"):
                interval["note"] = rec["note"]

    intervals = sorted(
        (iv for rid, iv in by_id.items() if rid not in deleted),
        key=lambda iv: iv["start"],
    )
    for iv in intervals:
        if iv["end"] is not None and iv["end"] < iv["start"]:
            warnings.append(f"interval id {iv['id']} ends before it starts; ignored in totals")
    return intervals, warnings


def load_researcher_meal_marks(day_dir: Path) -> list[tuple[str, float]]:
    """``(phase, epoch)`` for every researcher_meal record, sorted by epoch.

    The researcher marks when the meal actually starts and ends from the timer
    web tool; ``phase`` is ``"start"`` or ``"end"``. Malformed records are
    skipped. This is the source of truth for the meal boundaries.
    """
    marks = []
    for rec in _read_jsonl(Path(day_dir) / RESEARCHER_EVENTS_FILENAME):
        if rec.get("category") != CATEGORY_MEAL:
            continue
        phase, epoch = rec.get("phase"), rec.get("epoch")
        if phase in ("start", "end") and isinstance(epoch, (int, float)):
            marks.append((phase, float(epoch)))
    marks.sort(key=lambda m: m[1])
    return marks


def researcher_meal_window(day_dir: Path) -> tuple[float | None, float | None]:
    """Meal window from researcher marks: (earliest start, latest end).

    Either side is None when the researcher never marked it. Spanning all
    starts/ends keeps the whole-meal window even if a day has several
    start/finish pairs.
    """
    marks = load_researcher_meal_marks(day_dir)
    starts = [epoch for phase, epoch in marks if phase == "start"]
    ends = [epoch for phase, epoch in marks if phase == "end"]
    return (min(starts) if starts else None, max(ends) if ends else None)


def _spans(intervals: list[dict], now: float | None = None,
           clamp: tuple[float, float] | None = None) -> list[tuple[float, float]]:
    """Intervals -> concrete (start, end) spans; open ones close at ``now``.

    Open intervals are dropped when ``now`` is None. With ``clamp=(t0, t1)``
    spans are cut to the window and out-of-window spans are dropped.
    """
    spans = []
    for iv in intervals:
        start, end = iv["start"], iv["end"]
        if end is None:
            if now is None:
                continue
            end = max(now, start)
        if end < start:
            continue
        if clamp is not None:
            start, end = max(start, clamp[0]), min(end, clamp[1])
            if end <= start:
                continue
        spans.append((start, end))
    return spans


def sum_seconds(intervals: list[dict], now: float | None = None,
                clamp: tuple[float, float] | None = None) -> float:
    return sum(end - start for start, end in _spans(intervals, now, clamp))


def union_seconds(intervals: list[dict], now: float | None = None,
                  clamp: tuple[float, float] | None = None) -> float:
    """Total seconds covered by the union of the intervals.

    The union (not the sum) is what gets subtracted from the meal window, so
    an explanation given during an intervention is not deducted twice.
    """
    spans = sorted(_spans(intervals, now, clamp))
    total = 0.0
    merged_end = None
    for start, end in spans:
        if merged_end is None or start > merged_end:
            total += end - start
            merged_end = end
        elif end > merged_end:
            total += end - merged_end
            merged_end = end
    return total


def meal_window(day_dir: Path) -> tuple[float | None, float | None, dict]:
    """Meal window (start, end) for a day directory.

    The researcher's own marks (Start Meal / Finish Feeding, via the timer web
    tool) are authoritative: when present they define each end of the window.
    Whichever side the researcher did not mark falls back to the run.py-derived
    boundary -- the earliest / latest timestamp across metadata.json
    (started/ended) and the first/last lines of events.jsonl and
    user_inputs.jsonl. Returns ``(start, end, sources)`` where ``sources``
    records each candidate for the summary printout; start/end are None when
    no source is available at all.
    """
    day_dir = Path(day_dir)
    sources: dict[str, Any] = {}

    try:
        meta = json.loads((day_dir / "metadata.json").read_text(encoding="utf-8"))
        for key in ("started", "ended"):
            epoch = (meta.get(key) or {}).get("epoch")
            if isinstance(epoch, (int, float)):
                sources[f"metadata.{key}"] = float(epoch)
    except (OSError, json.JSONDecodeError):
        pass

    for name in ("events.jsonl", "user_inputs.jsonl"):
        records = _read_jsonl(day_dir / name)
        epochs = [r["epoch"] for r in records if isinstance(r.get("epoch"), (int, float))]
        if epochs:
            sources[f"{name}.first"] = float(epochs[0])
            sources[f"{name}.last"] = float(epochs[-1])

    derived_start = min(sources.values()) if sources else None
    derived_end = max(sources.values()) if sources else None

    r_start, r_end = researcher_meal_window(day_dir)
    if r_start is not None:
        sources["researcher.start"] = r_start
    if r_end is not None:
        sources["researcher.end"] = r_end

    window_start = r_start if r_start is not None else derived_start
    window_end = r_end if r_end is not None else derived_end
    return window_start, window_end, sources


def summarize_day(day_dir: Path, now: float | None = None) -> dict:
    """One day's feeding-time accounting as a plain dict (used by CLI and web)."""
    day_dir = Path(day_dir)
    intervals, warnings = load_researcher_intervals(day_dir)
    window_start, window_end, window_sources = meal_window(day_dir)

    open_ids = [iv["id"] for iv in intervals if iv["end"] is None]
    if open_ids and now is None:
        warnings.append(
            f"interval id(s) {open_ids} never ended; excluded from totals -- "
            "check researcher_events.jsonl")

    per_kind = {kind: sum_seconds([iv for iv in intervals if iv["kind"] == kind], now)
                for kind in INTERVAL_KINDS}
    union_raw = union_seconds(intervals, now)

    total_s = feeding_s = union_clamped = None
    if window_start is not None and window_end is not None and window_end > window_start:
        total_s = window_end - window_start
        union_clamped = union_seconds(intervals, now, clamp=(window_start, window_end))
        feeding_s = total_s - union_clamped
        if union_clamped < union_raw - 1.0:
            warnings.append(
                f"{union_raw - union_clamped:.0f}s of marked intervals fall outside "
                "the derived meal window (only the in-window part is deducted)")

    return {
        "day_dir": str(day_dir),
        "intervals": intervals,
        "open_ids": open_ids,
        "per_kind_s": per_kind,
        "union_s": union_raw,
        "union_in_window_s": union_clamped,
        "window_start": window_start,
        "window_end": window_end,
        "window_sources": window_sources,
        "total_s": total_s,
        "feeding_s": feeding_s,
        "warnings": warnings,
    }


def format_hms(seconds: float | None) -> str:
    if seconds is None:
        return "--:--:--"
    seconds = int(round(seconds))
    return f"{seconds // 3600}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"


def print_summary(summary: dict) -> None:
    from datetime import datetime

    def _clock(epoch):
        return (datetime.fromtimestamp(epoch).strftime("%H:%M:%S")
                if epoch is not None else "?")

    print(f"\n=== {summary['day_dir']} ===")
    print(f"  meal window : {_clock(summary['window_start'])} -> "
          f"{_clock(summary['window_end'])}  (total {format_hms(summary['total_s'])})")
    for kind in INTERVAL_KINDS:
        count = sum(1 for iv in summary["intervals"] if iv["kind"] == kind)
        print(f"  {kind:<12}: {format_hms(summary['per_kind_s'][kind])}  ({count} interval(s))")
    print(f"  union       : {format_hms(summary['union_s'])} marked overall, "
          f"{format_hms(summary['union_in_window_s'])} inside the window (deducted)")
    print(f"  FEEDING TIME: {format_hms(summary['feeding_s'])}")
    for iv in summary["intervals"]:
        duration = format_hms(iv["end"] - iv["start"]) if iv["end"] is not None else "OPEN"
        note = f"  -- {iv['note']}" if iv["note"] else ""
        print(f"    [{iv['id']}] {iv['kind']:<12} {_clock(iv['start'])} -> "
              f"{_clock(iv['end'])}  {duration}{note}")
    for warning in summary["warnings"]:
        print(f"  WARNING: {warning}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("day_dirs", nargs="*", type=Path,
                        help="day directories (log/<user>/day_NN)")
    parser.add_argument("--user", help="summarize this user's days under the log root")
    parser.add_argument("--day", type=int, help="restrict --user to one day number")
    args = parser.parse_args()

    day_dirs = list(args.day_dirs)
    if args.user:
        user_dir = LOG_ROOT / args.user
        if args.day is not None:
            day_dirs.append(user_dir / f"day_{args.day:02d}")
        else:
            day_dirs.extend(sorted(user_dir.glob("day_*")))
    if not day_dirs:
        parser.error("give day directories, or --user (optionally with --day)")

    for day_dir in day_dirs:
        if not day_dir.is_dir():
            print(f"\n=== {day_dir} ===\n  (missing directory, skipped)")
            continue
        print_summary(summarize_day(day_dir))
    print()


if __name__ == "__main__":
    main()
