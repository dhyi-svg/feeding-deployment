#!/usr/bin/env python3
"""
Deterministic analyzer for one day's ``prediction_model_llm_calls`` directory.

Each *.txt file in the directory is ONE re-run of the bundle-prediction model,
named ``YYYYMMDD_HHMMSS.txt`` (so lexicographic sort == chronological). Every
file has ===MODEL===, ===PROMPT===, ===RESPONSE=== sections; the PROMPT carries
a WORKING MEMORY block (meal context + the CONFIRMED / CORRECTED lists) and the
RESPONSE is the predicted bundle JSON (latent_inference + one value per dim +
explanations).

This script does everything that can be computed exactly -- parsing, accuracy,
the correlated-correction ledger, self-inflicted / non-bearing drift, volatility,
pipeline-stage mapping -- and writes:

    <out>/metrics.json       full structured facts (the LLM report consumes this)
    <out>/per_correction.csv per-step table (spreadsheet-friendly)
    <out>/g1_accuracy.png    accuracy + projected-remaining-corrections vs step
    <out>/g2_dim_heatmap.png per-dimension correct/wrong/pinned heatmap
    <out>/g3_ledger.png      positive/negative/lateral correlated changes per event
    <out>/g4_color_nav.png   color/nav distance-to-GT over steps

No LLM is involved. Interpretation is a separate step (see report_prompt.md).

Usage:
    python analyze_day.py <prediction_model_llm_calls_dir> [--out <dir>]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# --------------------------------------------------------------------------- #
# Dimension taxonomy. Kept explicit (not inferred) so the report is stable even
# if a future response omits a field. 20 categorical + 1 text + 3 color + 4 nav.
# --------------------------------------------------------------------------- #
CATEGORICAL = [
    "robot_speed", "microwave_time", "skewering_axis", "confirm_feeding_pickup",
    "confirm_navigation_arrival", "confirm_manipulation", "transfer_mode",
    "outside_mouth_distance", "convey_robot_ready_for_initiating_transfer",
    "detect_user_ready_for_initiating_transfer_feeding",
    "detect_user_ready_for_initiating_transfer_drinking",
    "detect_user_ready_for_initiating_transfer_wiping",
    "convey_robot_ready_for_completing_transfer",
    "detect_user_completed_transfer_feeding",
    "detect_user_completed_transfer_drinking",
    "detect_user_completed_transfer_wiping", "retract_between_bites",
    "bite_dipping_preference", "wait_before_autocontinue_bite_selection",
    "wait_before_autocontinue_task_selection",
]
TEXT = ["bite_ordering"]
COLOR = ["plate_color_fridge", "plate_color_microwave", "plate_color_table"]
NAV = ["nav_offset_table", "nav_offset_microwave", "nav_offset_sink", "nav_offset_fridge"]

KIND = {**{f: "categorical" for f in CATEGORICAL}, **{f: "text" for f in TEXT},
        **{f: "color" for f in COLOR}, **{f: "nav" for f in NAV}}

# Pipeline stage each dim is surfaced in (see preference_session.py phases).
STAGE = {
    "robot_speed": "Initial", "confirm_navigation_arrival": "Initial",
    "confirm_manipulation": "Initial", "microwave_time": "Microwave ask",
    **{f: "Table prefs / feeding loop" for f in [
        "skewering_axis", "confirm_feeding_pickup", "bite_dipping_preference",
        "bite_ordering", "transfer_mode", "outside_mouth_distance",
        "convey_robot_ready_for_initiating_transfer",
        "convey_robot_ready_for_completing_transfer",
        "detect_user_ready_for_initiating_transfer_feeding",
        "detect_user_ready_for_initiating_transfer_drinking",
        "detect_user_ready_for_initiating_transfer_wiping",
        "detect_user_completed_transfer_feeding",
        "detect_user_completed_transfer_drinking",
        "detect_user_completed_transfer_wiping", "retract_between_bites",
        "wait_before_autocontinue_bite_selection",
        "wait_before_autocontinue_task_selection"]},
    **{f: "Plate pickup (color)" for f in COLOR},
    **{f: "Navigation (nav offset)" for f in NAV},
}

HEDGE_PATTERNS = [
    "not directly tied", "not tied", "seed", "default", "no correction",
    "retained", "carried over", "carried-over", "unrelated", "neutral",
]

# --------------------------------------------------------------------------- #
# Parsing
# --------------------------------------------------------------------------- #
def _parse_kv_value(raw: str) -> Any:
    """A CONFIRMED/CORRECTED value: color (h=..,s=..,v=..,range=..), nav
    (dx=..,dy=..,dyaw=..), or a plain categorical/text string."""
    raw = raw.strip()
    if re.match(r"^[a-z]+=", raw) and "," in raw and "=" in raw:
        parts = dict(p.split("=", 1) for p in raw.split(",") if "=" in p)
        try:
            if {"h", "s", "v"} <= set(parts):
                return {k: float(parts[k]) for k in parts}
            if {"dx", "dy", "dyaw"} <= set(parts):
                return {k: float(parts[k]) for k in parts}
        except ValueError:
            return raw
    return raw


def _parse_block(prompt: str, header: str) -> Dict[str, Any]:
    """Fields listed under a CONFIRMED/CORRECTED header until the blank line."""
    out: Dict[str, Any] = {}
    lines = prompt.splitlines()
    capture = False
    for line in lines:
        if header in line:
            capture = True
            continue
        if capture:
            s = line.strip()
            if s == "":
                break
            if s == "(none)":
                continue
            if "=" in s:
                k, v = s.split("=", 1)
                out[k.strip()] = _parse_kv_value(v)
    return out


def _extract_context(prompt: str) -> Dict[str, str]:
    ctx = {}
    for key in ("meal", "setting", "time_of_day"):
        m = re.search(rf"^- {key}:\s*(.+)$", prompt, re.MULTILINE)
        if m:
            ctx[key] = m.group(1).strip()
    return ctx


def _prior_memory_present(prompt: str) -> bool:
    """True if any cross-day memory block has non-empty content (i.e. not the
    user's first day). Handles both single_full_history and three_layer."""
    for hdr in ("FULL HISTORY MEMORY", "EPISODIC MEMORY", "SEMANTIC MEMORY"):
        m = re.search(re.escape(hdr) + r".*?===\n(.*?)\n======", prompt, re.DOTALL)
        if m and m.group(1).strip() and m.group(1).strip() != "(no prior memory)":
            return True
    return False


def parse_file(path: Path) -> Dict[str, Any]:
    text = path.read_text(encoding="utf-8")
    model_line = ""
    mm = re.search(r"===MODEL===\n(.*?)\n", text)
    if mm:
        model_line = mm.group(1).strip()
    prompt = text.split("===PROMPT===", 1)[1].split("===RESPONSE===", 1)[0] \
        if "===PROMPT===" in text else ""
    resp_raw = text.split("===RESPONSE===", 1)[1].strip() if "===RESPONSE===" in text else ""
    try:
        response = json.loads(resp_raw)
        parse_ok = True
    except Exception:
        response = {}
        parse_ok = False

    confirmed = _parse_block(prompt, "CONFIRMED this meal")
    corrected = _parse_block(prompt, "CORRECTED this meal")
    predicted = {f: response.get(f) for f in KIND if f in response}

    return {
        "file": path.name,
        "ts": path.stem.split("_")[-1],
        "model_line": model_line,
        "context": _extract_context(prompt),
        "prior_memory_present": _prior_memory_present(prompt),
        "confirmed": confirmed,
        "corrected": corrected,
        "predicted": predicted,
        "latent_inference": response.get("latent_inference", ""),
        "explanations": response.get("explanations", {}) if isinstance(response.get("explanations"), dict) else {},
        "parse_ok": parse_ok,
    }


# --------------------------------------------------------------------------- #
# Distances for continuous dims
# --------------------------------------------------------------------------- #
def color_dist(a: Any, b: Any) -> Optional[float]:
    if not (isinstance(a, dict) and isinstance(b, dict)):
        return None
    try:
        return math.sqrt(sum((float(a[k]) - float(b[k])) ** 2 for k in ("h", "s", "v")))
    except (KeyError, ValueError, TypeError):
        return None


def nav_dist(a: Any, b: Any) -> Optional[float]:
    if not (isinstance(a, dict) and isinstance(b, dict)):
        return None
    try:
        d_xy = math.sqrt((float(a["dx"]) - float(b["dx"])) ** 2 + (float(a["dy"]) - float(b["dy"])) ** 2)
        return d_xy + abs(float(a["dyaw"]) - float(b["dyaw"]))
    except (KeyError, ValueError, TypeError):
        return None


COLOR_TOL = 3.0    # HSV Euclidean units below which a color is "unchanged"/"correct"
NAV_TOL = 0.02     # metres+rad below which an offset is "unchanged"/"correct"


# --------------------------------------------------------------------------- #
# Analysis
# --------------------------------------------------------------------------- #
def analyze(files: List[Dict[str, Any]]) -> Dict[str, Any]:
    last = files[-1]
    # Ground truth = final pinned state (corrected wins over confirmed).
    gt: Dict[str, Any] = {**last["confirmed"], **last["corrected"]}
    unresolved = [f for f in KIND if f not in gt]

    def cat_accuracy(pred: Dict[str, Any]) -> Tuple[int, int]:
        known = [f for f in CATEGORICAL if f in gt]
        correct = sum(1 for f in known if pred.get(f) == gt[f])
        return correct, len(known)

    # ---- Trajectory ---------------------------------------------------------
    trajectory = []
    for i, fr in enumerate(files):
        pinned = {**fr["confirmed"], **fr["corrected"]}
        acc, denom = cat_accuracy(fr["predicted"])
        remaining = sum(
            1 for f in CATEGORICAL
            if f in gt and f not in pinned and fr["predicted"].get(f) != gt[f]
        )
        event = "INIT" if (i == 0 and not fr["corrected"] and not fr["confirmed"]) else "correction"
        trajectory.append({
            "idx": i, "file": fr["file"], "ts": fr["ts"], "event": event,
            "cat_accuracy": acc, "cat_denom": denom,
            "remaining_corrections": remaining,
            "confirmed_fields": sorted(fr["confirmed"].keys()),
            "corrected_fields": sorted(fr["corrected"].keys()),
            "latent_inference": fr["latent_inference"],
            "parse_ok": fr["parse_ok"],
        })

    # ---- Transitions + correlated ledger -----------------------------------
    def classify(old: Any, new: Any, truth: Any) -> str:
        if new == truth and old != truth:
            return "POSITIVE"
        if new != truth and old == truth:
            return "NEGATIVE"
        return "LATERAL"

    transitions = []
    ledger = {"POSITIVE": 0, "NEGATIVE": 0, "LATERAL": 0}
    hedge_candidates = []
    for i in range(1, len(files)):
        prev, cur = files[i - 1], files[i]
        pinned_prev = set(prev["confirmed"]) | set(prev["corrected"])
        newly_corrected = [f for f in cur["corrected"] if f not in prev["corrected"]]
        newly_confirmed = [f for f in cur["confirmed"] if f not in prev["confirmed"]]

        direct = [{
            "field": f, "kind": KIND.get(f, "?"),
            "old": prev["predicted"].get(f), "new": cur["predicted"].get(f),
        } for f in newly_corrected]
        trigger_kinds = sorted({KIND.get(f, "?") for f in newly_corrected})

        correlated = []
        for f in CATEGORICAL:
            if f in newly_corrected or f in pinned_prev or f not in gt:
                continue
            o, n = prev["predicted"].get(f), cur["predicted"].get(f)
            if o != n:
                kl = classify(o, n, gt[f])
                correlated.append({"field": f, "old": o, "new": n, "klass": kl, "gt": gt[f]})
                ledger[kl] += 1
                expl = cur["explanations"].get(f, "")
                if any(p in expl.lower() for p in HEDGE_PATTERNS):
                    hedge_candidates.append({
                        "field": f, "at": cur["file"], "change": f"{o!r}->{n!r}",
                        "klass": kl, "explanation": expl,
                    })

        acc_before = trajectory[i - 1]["cat_accuracy"]
        acc_after = trajectory[i]["cat_accuracy"]
        delta_direct = sum(1 for d in direct if d["kind"] == "categorical")
        delta_correlated = sum({"POSITIVE": 1, "NEGATIVE": -1, "LATERAL": 0}[c["klass"]] for c in correlated)

        transitions.append({
            "from": prev["file"], "to": cur["file"],
            "trigger_fields": newly_corrected, "trigger_kinds": trigger_kinds,
            "direct": direct, "correlated": correlated,
            "newly_confirmed": newly_confirmed,
            "acc_before": acc_before, "acc_after": acc_after,
            "acc_delta_direct": delta_direct, "acc_delta_correlated": delta_correlated,
            "self_check_ok": (acc_after - acc_before) == (delta_direct + delta_correlated),
        })

    # ---- Findings -----------------------------------------------------------
    # Self-inflicted: predicted right at init, later drifted wrong, then corrected.
    self_inflicted = []
    for f in CATEGORICAL:
        if f not in gt:
            continue
        if files[0]["predicted"].get(f) != gt[f]:
            continue  # not right at init
        corr_idx = next((i for i, fr in enumerate(files) if f in fr["corrected"]), None)
        if corr_idx is None:
            continue
        drift_idx = next((i for i in range(1, corr_idx)
                          if files[i]["predicted"].get(f) != gt[f]), None)
        if drift_idx is not None:
            self_inflicted.append({
                "field": f, "init_value": files[0]["predicted"].get(f),
                "drifted_at": files[drift_idx]["file"],
                "corrected_at": files[corr_idx]["file"],
            })

    # Non-bearing drift: NEGATIVE correlated change whose trigger was color/nav.
    non_bearing = []
    for t in transitions:
        if set(t["trigger_kinds"]) <= {"color", "nav"}:
            for c in t["correlated"]:
                if c["klass"] == "NEGATIVE":
                    non_bearing.append({
                        "trigger_fields": t["trigger_fields"], "trigger_kinds": t["trigger_kinds"],
                        "drifted_field": c["field"], "change": f'{c["old"]!r}->{c["new"]!r}',
                        "at": t["to"],
                    })

    # Re-corrections: confirmed at some file, corrected later.
    re_corrections = []
    for f in KIND:
        conf_idx = next((i for i, fr in enumerate(files) if f in fr["confirmed"]), None)
        corr_idx = next((i for i, fr in enumerate(files) if f in fr["corrected"]), None)
        if conf_idx is not None and corr_idx is not None and corr_idx > conf_idx:
            re_corrections.append({
                "field": f, "kind": KIND[f],
                "confirmed_value": files[conf_idx]["confirmed"].get(f),
                "confirmed_at": files[conf_idx]["file"],
                "corrected_value": files[corr_idx]["corrected"].get(f),
                "corrected_at": files[corr_idx]["file"],
            })

    # Volatility: number of prediction changes + flip-flop detection.
    volatility = []
    for f in CATEGORICAL:
        seq = [fr["predicted"].get(f) for fr in files]
        changes = sum(1 for a, b in zip(seq, seq[1:]) if a != b)
        seen, flip = set(), False
        prev_v = object()
        for v in seq:
            if v != prev_v:
                if v in seen:
                    flip = True
                seen.add(v)
                prev_v = v
        if changes:
            volatility.append({"field": f, "n_changes": changes, "flip_flopped": flip,
                               "sequence": seq})
    volatility.sort(key=lambda d: (-d["n_changes"], not d["flip_flopped"]))

    # Color / nav drift vs GT over steps (for G4 + findings).
    cont_drift = {}
    for f in COLOR + NAV:
        if f not in gt:
            continue
        distfn = color_dist if KIND[f] == "color" else nav_dist
        cont_drift[f] = [distfn(fr["predicted"].get(f), gt[f]) for fr in files]

    return {
        "ground_truth": gt, "unresolved_dims": unresolved,
        "trajectory": trajectory, "transitions": transitions,
        "ledger": ledger, "hedge_candidates": hedge_candidates,
        "findings": {
            "self_inflicted": self_inflicted, "non_bearing_drift": non_bearing,
            "re_corrections": re_corrections, "volatility": volatility,
        },
        "continuous_drift": cont_drift,
        "stages": {f: STAGE.get(f, "?") for f in KIND},
    }


# --------------------------------------------------------------------------- #
# Plots
# --------------------------------------------------------------------------- #
def make_plots(files, result, out: Path) -> List[str]:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    import numpy as np

    made = []
    traj = result["trajectory"]
    xs = list(range(len(traj)))
    xlabels = [f"{t['ts'][:2]}:{t['ts'][2:4]}:{t['ts'][4:]}" for t in traj]

    # G1: accuracy + remaining corrections
    fig, ax1 = plt.subplots(figsize=(11, 5))
    acc = [t["cat_accuracy"] for t in traj]
    rem = [t["remaining_corrections"] for t in traj]
    ax1.plot(xs, acc, "o-", color="#2a7", label="Categorical accuracy")
    ax1.set_ylabel("Categorical dims correct (of %d)" % (traj[0]["cat_denom"] or 20), color="#2a7")
    ax1.set_ylim(0, (traj[0]["cat_denom"] or 20) + 1)
    ax2 = ax1.twinx()
    ax2.plot(xs, rem, "s--", color="#c33", label="Projected remaining corrections")
    ax2.set_ylabel("Open dims still != GT", color="#c33")
    ax1.set_xticks(xs)
    ax1.set_xticklabels(xlabels, rotation=45, ha="right", fontsize=8)
    for t in result["transitions"]:
        i = next(k for k, tr in enumerate(traj) if tr["file"] == t["to"])
        if t["trigger_fields"]:
            ax1.annotate("+".join(t["trigger_fields"]), (i, acc[i]),
                         fontsize=6, rotation=30, ha="left", va="bottom")
    ax1.set_title("Accuracy & remaining corrections over the meal")
    ax1.set_xlabel("Re-prediction step (LLM call)")
    fig.tight_layout()
    p = out / "g1_accuracy.png"; fig.savefig(p, dpi=130); plt.close(fig); made.append(p.name)

    # G2: per-dimension heatmap (0=wrong,1=correct,2=pinned-correct)
    gt = result["ground_truth"]
    dims = [f for f in sorted(CATEGORICAL, key=lambda x: (STAGE.get(x, ""), x)) if f in gt]
    grid = np.zeros((len(dims), len(files)))
    for j, fr in enumerate(files):
        pinned = {**fr["confirmed"], **fr["corrected"]}
        for r, f in enumerate(dims):
            ok = fr["predicted"].get(f) == gt[f]
            grid[r, j] = (2 if f in pinned else 1) if ok else 0
    cmap = ListedColormap(["#e05555", "#bfe3bf", "#2a7d2a"])
    fig, ax = plt.subplots(figsize=(11, 8))
    ax.imshow(grid, aspect="auto", cmap=cmap, vmin=0, vmax=2)
    ax.set_xticks(xs); ax.set_xticklabels(xlabels, rotation=45, ha="right", fontsize=7)
    ax.set_yticks(range(len(dims))); ax.set_yticklabels(dims, fontsize=7)
    ax.set_title("Per-dimension prediction: red=wrong, light=open&correct, dark=pinned")
    fig.tight_layout()
    p = out / "g2_dim_heatmap.png"; fig.savefig(p, dpi=130); plt.close(fig); made.append(p.name)

    # G3: correlated ledger per event
    ev = [t for t in result["transitions"] if t["correlated"]]
    if ev:
        labels = ["+".join(t["trigger_fields"]) or t["to"] for t in ev]
        pos = [sum(1 for c in t["correlated"] if c["klass"] == "POSITIVE") for t in ev]
        neg = [sum(1 for c in t["correlated"] if c["klass"] == "NEGATIVE") for t in ev]
        lat = [sum(1 for c in t["correlated"] if c["klass"] == "LATERAL") for t in ev]
        x = np.arange(len(ev))
        fig, ax = plt.subplots(figsize=(11, 5))
        ax.bar(x, pos, color="#2a7d2a", label="positive")
        ax.bar(x, lat, bottom=pos, color="#bbb", label="lateral")
        ax.bar(x, [-n for n in neg], color="#e05555", label="negative")
        ax.axhline(0, color="k", lw=0.6)
        ax.set_xticks(x); ax.set_xticklabels(labels, rotation=40, ha="right", fontsize=7)
        ax.set_ylabel("# correlated open-dim changes")
        ax.set_title("Correlated corrections per event (up=helped, down=hurt)")
        ax.legend(fontsize=8)
        fig.tight_layout()
        p = out / "g3_ledger.png"; fig.savefig(p, dpi=130); plt.close(fig); made.append(p.name)

    # G4: color/nav distance to GT
    cd = result["continuous_drift"]
    if cd:
        fig, ax = plt.subplots(figsize=(11, 5))
        for f, series in cd.items():
            ys = [v if v is not None else np.nan for v in series]
            ax.plot(xs, ys, "o-", label=f, markersize=3)
        ax.set_xticks(xs); ax.set_xticklabels(xlabels, rotation=45, ha="right", fontsize=7)
        ax.set_ylabel("distance to final GT")
        ax.set_title("Color / nav-offset drift vs final ground truth")
        ax.legend(fontsize=7)
        fig.tight_layout()
        p = out / "g4_color_nav.png"; fig.savefig(p, dpi=130); plt.close(fig); made.append(p.name)

    return made


# --------------------------------------------------------------------------- #
# Markdown tables (Table 1: init prediction vs truth; Table 2: correction walk)
# --------------------------------------------------------------------------- #
_ALIAS = {
    "wait_before_autocontinue_bite_selection": "wait_bite",
    "wait_before_autocontinue_task_selection": "wait_task",
    "convey_robot_ready_for_initiating_transfer": "convey_init",
    "convey_robot_ready_for_completing_transfer": "convey_complete",
    "detect_user_completed_transfer_feeding": "detect_completed_feeding",
}


def _alias(f: str) -> str:
    return _ALIAS.get(f, f)


def _short(v: Any) -> str:
    """Compact rendering for Table 2 (countdowns -> the number, etc.)."""
    if not isinstance(v, str):
        return str(v)
    m = re.match(r"countdown \((\d+) sec\)", v)
    if m:
        return m.group(1)
    return {
        "30 secs": "30s",
        "proceed automatically after a pause": "proceed auto",
    }.get(v, v.replace("speech + LED", "speech+LED"))


def write_tables(files: List[Dict[str, Any]], result: Dict[str, Any], out: Path) -> Path:
    gt = result["ground_truth"]
    init = files[0]
    init_expl = init["explanations"]

    # ---- Table 1: initial categorical prediction vs user's actual preference
    t1 = ["### Categorical (20)", "",
          "| Dimension | Model predicted | User's actual preference |  | Reason |",
          "| --- | --- | --- | --- | --- |"]
    for f in CATEGORICAL:
        pred = init["predicted"].get(f)
        truth = gt.get(f, "(unresolved)")
        ok = f in gt and pred == gt[f]
        actual = str(truth) if ok else f"**{truth}**"
        reason = "" if ok else str(init_expl.get(f, "")).strip()
        t1.append(f"| {f} | {pred} | {actual} | {'✓' if ok else '✗'} | {reason} |")

    # ---- Table 2: correction-by-correction walkthrough
    recorr = {d["field"] for d in result["findings"]["re_corrections"]}
    selfinf = {d["field"]: d["corrected_at"] for d in result["findings"]["self_inflicted"]}

    def fmt_direct(t) -> str:
        parts = []
        for d in t["direct"]:
            f, k = d["field"], d["kind"]
            if k != "categorical":
                parts.append(f"`{f}` *({k.upper()})*")
                continue
            o, n = _short(d["old"]), _short(d["new"])
            if f in recorr:
                s = f"`{_alias(f)}` [conf. {o}]→**{n}** ✓ ⟲"
            else:
                s = f"`{_alias(f)}` {o}→**{n}** ✓"
            if selfinf.get(f) == t["to"]:
                s += " *(undoing drift)*"
            parts.append(s)
        return " · ".join(parts) if parts else "—"

    def fmt_correlated(t) -> str:
        chunks = []
        for c in t["correlated"]:
            a, o, n, g = _alias(c["field"]), _short(c["old"]), _short(c["new"]), c["gt"]
            if c["klass"] == "POSITIVE":
                chunks.append(f"**+** `{a}` {o}→{n} ✓")
            elif c["klass"] == "NEGATIVE":
                chunks.append(f"**−** `{a}` {o}→**{n}** ✗")
            else:
                oc = "✓" if c["old"] == g else "✗"
                nc = "✓" if c["new"] == g else "✗"
                chunks.append(f"~ `{a}` {o}→{n} ({oc}→{nc})")
        return " · ".join(chunks) if chunks else "none"

    traj = result["trajectory"]
    t2 = ["", "### Correction walkthrough", "",
          "| Step (file) | Direct correction (event) | Correlated prediction changes | Acc |",
          "| --- | --- | --- | --- |",
          f"| `{traj[0]['ts']}` init | — initial prediction — | — | **{traj[0]['cat_accuracy']}** |"]
    for t in result["transitions"]:
        step = t["to"].replace(".txt", "").split("_")[-1]
        t2.append(f"| `{step}` | {fmt_direct(t)} | {fmt_correlated(t)} | "
                  f"{t['acc_before']}→**{t['acc_after']}** |")

    path = out / "tables.md"
    path.write_text("\n".join(t1 + t2) + "\n", encoding="utf-8")
    return path


# --------------------------------------------------------------------------- #
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", help="path to a prediction_model_llm_calls directory")
    ap.add_argument("--out", default=None, help="output dir (default: <dir>/../analysis)")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    src = Path(args.dir)
    paths = sorted(src.glob("*.txt"))
    if not paths:
        raise SystemExit(f"No .txt files in {src}")
    files = [parse_file(p) for p in paths]

    out = Path(args.out) if args.out else src.parent / "analysis"
    out.mkdir(parents=True, exist_ok=True)

    result = analyze(files)
    ctx = files[0]["context"]
    result["meta"] = {
        "dir": str(src), "n_files": len(files),
        "files": [f["file"] for f in files],
        "model_line": files[0]["model_line"],
        "meal_context": ctx,
        "prior_memory_present": files[0]["prior_memory_present"],
        "parse_failures": [f["file"] for f in files if not f["parse_ok"]],
        "color_tol": COLOR_TOL, "nav_tol": NAV_TOL,
    }
    # carry per-file explanations for the LLM step (small, useful for quoting)
    result["explanations_by_file"] = {f["file"]: f["explanations"] for f in files}

    (out / "metrics.json").write_text(json.dumps(result, indent=2, default=str), encoding="utf-8")
    write_tables(files, result, out)

    # per_correction.csv
    with open(out / "per_correction.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["step", "file", "event", "trigger", "trigger_kind", "stage",
                    "acc_before", "acc_after", "pos", "neg", "lat", "correlated_detail"])
        w.writerow([0, files[0]["file"], "INIT", "", "", "",
                    "", result["trajectory"][0]["cat_accuracy"], "", "", "", ""])
        for i, t in enumerate(result["transitions"], start=1):
            trig = "+".join(t["trigger_fields"])
            stage = "; ".join(sorted({STAGE.get(f, "?") for f in t["trigger_fields"]}))
            pos = sum(1 for c in t["correlated"] if c["klass"] == "POSITIVE")
            neg = sum(1 for c in t["correlated"] if c["klass"] == "NEGATIVE")
            lat = sum(1 for c in t["correlated"] if c["klass"] == "LATERAL")
            detail = "; ".join(f'{c["field"]}:{c["old"]}->{c["new"]}({c["klass"][:3]})'
                               for c in t["correlated"])
            w.writerow([i, t["to"], "correction", trig, "+".join(t["trigger_kinds"]),
                        stage, t["acc_before"], t["acc_after"], pos, neg, lat, detail])

    plots = []
    if not args.no_plots:
        try:
            plots = make_plots(files, result, out)
        except Exception as e:  # never let plotting kill the metrics output
            print(f"[warn] plotting failed: {e}")

    # ---- console summary ----------------------------------------------------
    tr = result["trajectory"]
    print(f"dir            : {src}")
    print(f"files          : {len(files)}  parse_failures={result['meta']['parse_failures'] or 'none'}")
    print(f"meal           : {ctx.get('meal')} | {ctx.get('setting')} | {ctx.get('time_of_day')}")
    print(f"prior memory   : {'present' if result['meta']['prior_memory_present'] else 'EMPTY (first day)'}")
    print(f"accuracy traj  : {' -> '.join(str(t['cat_accuracy']) for t in tr)} / {tr[0]['cat_denom']}")
    print(f"remaining traj : {' -> '.join(str(t['remaining_corrections']) for t in tr)}")
    print(f"ledger         : +{result['ledger']['POSITIVE']} / -{result['ledger']['NEGATIVE']} / ={result['ledger']['LATERAL']} (pos/neg/lateral correlated)")
    print(f"self-inflicted : {[d['field'] for d in result['findings']['self_inflicted']] or 'none'}")
    print(f"non-bearing    : {[(d['trigger_fields'], d['drifted_field']) for d in result['findings']['non_bearing_drift']] or 'none'}")
    print(f"re-corrections : {[d['field'] for d in result['findings']['re_corrections']] or 'none'}")
    print(f"most volatile  : {[(d['field'], d['n_changes']) for d in result['findings']['volatility'][:5]]}")
    bad = [t for t in result['transitions'] if not t['self_check_ok']]
    if bad:
        print(f"[warn] {len(bad)} transition(s) failed the acc self-check")
    print(f"unresolved dims: {result['unresolved_dims'] or 'none'}")
    print(f"wrote          : {out}/metrics.json, tables.md, per_correction.csv" + (f", {', '.join(plots)}" if plots else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
