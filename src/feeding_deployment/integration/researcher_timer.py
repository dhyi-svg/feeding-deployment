#!/usr/bin/env python3
"""Researcher intervention timer -- standalone web tool on port 8081.

During a study meal the participant's iPad occupies the main webapp (port
8080). This serves a separate single page where the *researcher* (phone or
laptop, http://192.168.1.2:8081) timestamps interventions and explanations --
events outside the system that the robot cannot capture itself. The offline
``compute_feeding_time.py`` then reports
``feeding time = meal window - union(marked intervals)``.

Design constraints (why this is not part of run.py or the Vue app):
- Interventions happen exactly when the system is down, so this depends on
  nothing: no roscore, no rosbridge, no run.py. Plain Flask + JSONL files.
- Timestamps are stamped SERVER-side so they share the robot's clock with
  data_logger's events.jsonl/metadata.json regardless of the phone's clock.
- Append-only log (``log/<user>/day_NN/researcher_events.jsonl``): interval
  starts are written at press time (a crash mid-interval keeps the start), and
  mistakes are tombstoned, never rewritten.
- Best-effort mirror of every record onto /deployment/annotations (via a
  ``rostopic pub`` subprocess, so ROS being down can never hurt the server)
  keeps the per-meal rosbags self-annotating, matching data_logger.

Launched together with the participant webapp by ``webapp/launch_app.sh``.
State (which user/day is selected) lives server-side and is persisted, so any
device can reconnect -- or the server restart -- without losing anything.
"""

from __future__ import annotations

import json
import logging
import re
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

from flask import Flask, Response, jsonify, request

try:
    from feeding_deployment.integration.compute_feeding_time import (
        CATEGORY_DELETED, CATEGORY_INTERVAL, INTERVAL_KINDS, LOG_ROOT,
        RESEARCHER_EVENTS_FILENAME, _read_jsonl, load_researcher_intervals,
        meal_window)
except ImportError:  # run directly as `python researcher_timer.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from compute_feeding_time import (
        CATEGORY_DELETED, CATEGORY_INTERVAL, INTERVAL_KINDS, LOG_ROOT,
        RESEARCHER_EVENTS_FILENAME, _read_jsonl, load_researcher_intervals,
        meal_window)

PORT = 8081
SELECTION_FILE = LOG_ROOT / ".researcher_timer_state.json"
ANNOTATIONS_TOPIC = "/deployment/annotations"
_USER_RE = re.compile(r"^[A-Za-z0-9._-]+$")

app = Flask(__name__)
_lock = threading.Lock()  # guards selection + read-modify-append of the day file
_selected: Path | None = None


# -- session (user/day) handling ---------------------------------------------

def _load_selection() -> Path | None:
    try:
        raw = json.loads(SELECTION_FILE.read_text(encoding="utf-8")).get("dir", "")
        path = Path(raw)
        if _valid_day_dir(path) and path.is_dir():
            return path
    except (OSError, json.JSONDecodeError):
        pass
    return None


def _persist_selection(day_dir: Path) -> None:
    try:
        SELECTION_FILE.write_text(json.dumps({"dir": str(day_dir)}), encoding="utf-8")
    except OSError:
        pass


def _valid_day_dir(path: Path) -> bool:
    """True iff path is LOG_ROOT/<user>/day_NN (no traversal outside the root)."""
    try:
        path = path.resolve()
        return (path.parent.parent == LOG_ROOT.resolve()
                and re.fullmatch(r"day_\d+", path.name) is not None
                and _USER_RE.fullmatch(path.parent.name) is not None)
    except OSError:
        return False


def _list_sessions(limit: int = 25) -> list[dict]:
    """Existing day dirs, most recently active first."""
    sessions = []
    for day_dir in LOG_ROOT.glob("*/day_*"):
        if not day_dir.is_dir() or not _valid_day_dir(day_dir):
            continue
        mtime = day_dir.stat().st_mtime
        for name in ("metadata.json", RESEARCHER_EVENTS_FILENAME, "events.jsonl"):
            try:
                mtime = max(mtime, (day_dir / name).stat().st_mtime)
            except OSError:
                pass
        sessions.append({
            "dir": str(day_dir),
            "user": day_dir.parent.name,
            "day": int(day_dir.name.split("_")[1]),
            "label": f"{day_dir.parent.name} / {day_dir.name}",
            "mtime": mtime,
        })
    sessions.sort(key=lambda s: s["mtime"], reverse=True)
    return sessions[:limit]


# -- record writing -----------------------------------------------------------

def _stamp() -> dict:
    now = time.time()
    return {"epoch": now, "iso": datetime.fromtimestamp(now).isoformat()}


def _append_record(day_dir: Path, record: dict) -> None:
    # Caller must hold _lock.
    with open(day_dir / RESEARCHER_EVENTS_FILENAME, "a", encoding="utf-8") as f:
        f.write(json.dumps(record, default=str) + "\n")
    _mirror_to_ros(record)


def _mirror_to_ros(record: dict) -> None:
    """Best-effort publish onto /deployment/annotations for the meal rosbags.

    Uses a one-shot ``rostopic pub`` in a daemon thread instead of an in-process
    rospy node: a subprocess cannot take the Flask server down with it, needs no
    node-lifecycle management across roscore restarts, and simply fails quietly
    when ROS is down (the JSONL file is the source of truth regardless).
    """
    def _publish() -> None:
        try:
            subprocess.run(
                ["rostopic", "pub", "-1", ANNOTATIONS_TOPIC, "std_msgs/String",
                 json.dumps({"data": json.dumps(record, default=str)})],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=20, check=False)
        except Exception:  # noqa: BLE001 - mirroring must never disturb the tool
            pass

    threading.Thread(target=_publish, daemon=True).start()


def _next_interval_id(day_dir: Path) -> int:
    ids = [r["id"] for r in _read_jsonl(day_dir / RESEARCHER_EVENTS_FILENAME)
           if isinstance(r.get("id"), int)]
    return max(ids, default=0) + 1


# -- API ----------------------------------------------------------------------

@app.get("/api/state")
def api_state():
    with _lock:
        day_dir = _selected
    state = {
        "server_epoch": time.time(),
        "sessions": _list_sessions(),
        "session": None,
        "intervals": [],
        "warnings": [],
        "window_start": None,
    }
    if day_dir is not None:
        intervals, warnings = load_researcher_intervals(day_dir)
        window_start, _, _ = meal_window(day_dir)
        state.update({
            "session": {"dir": str(day_dir), "user": day_dir.parent.name,
                        "day": int(day_dir.name.split("_")[1])},
            "intervals": intervals,
            "warnings": warnings,
            "window_start": window_start,
        })
    return jsonify(state)


@app.post("/api/session")
def api_session():
    """Select an existing day dir ({dir}) or create/select one ({user, day})."""
    global _selected
    body = request.get_json(silent=True) or {}
    if body.get("dir"):
        day_dir = Path(body["dir"])
        if not (_valid_day_dir(day_dir) and day_dir.is_dir()):
            return jsonify({"error": "unknown session"}), 400
    else:
        user, day = str(body.get("user", "")).strip(), body.get("day")
        try:
            day = int(day)
        except (TypeError, ValueError):
            return jsonify({"error": "day must be a number"}), 400
        if not _USER_RE.fullmatch(user) or not 0 <= day <= 999:
            return jsonify({"error": "bad user name or day"}), 400
        day_dir = LOG_ROOT / user / f"day_{day:02d}"
        day_dir.mkdir(parents=True, exist_ok=True)
    with _lock:
        _selected = day_dir.resolve()
        _persist_selection(_selected)
    return jsonify({"ok": True})


@app.post("/api/press")
def api_press():
    """Toggle an interval: first press opens it (start stamped immediately),
    second press closes it. The optional note rides on whichever record the
    press produces; a note given at close time wins."""
    body = request.get_json(silent=True) or {}
    kind, note = body.get("kind"), str(body.get("note", "")).strip()
    if kind not in INTERVAL_KINDS:
        return jsonify({"error": f"kind must be one of {INTERVAL_KINDS}"}), 400
    with _lock:
        if _selected is None:
            return jsonify({"error": "no session selected"}), 400
        intervals, _ = load_researcher_intervals(_selected)
        open_interval = next(
            (iv for iv in intervals if iv["kind"] == kind and iv["end"] is None), None)
        record = {**_stamp(), "category": CATEGORY_INTERVAL, "kind": kind}
        if open_interval is None:
            record.update(phase="start", id=_next_interval_id(_selected))
        else:
            record.update(phase="end", id=open_interval["id"])
        if note:
            record["note"] = note
        _append_record(_selected, record)
    return jsonify({"ok": True, "phase": record["phase"], "id": record["id"]})


@app.post("/api/delete")
def api_delete():
    body = request.get_json(silent=True) or {}
    interval_id = body.get("id")
    if not isinstance(interval_id, int):
        return jsonify({"error": "id must be an integer"}), 400
    with _lock:
        if _selected is None:
            return jsonify({"error": "no session selected"}), 400
        _append_record(_selected, {**_stamp(), "category": CATEGORY_DELETED,
                                   "id": interval_id})
    return jsonify({"ok": True})


@app.get("/")
def index():
    return Response(PAGE, mimetype="text/html")


# -- page ----------------------------------------------------------------------

PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1">
<title>Researcher Timer</title>
<style>
  * { box-sizing: border-box; margin: 0; -webkit-tap-highlight-color: transparent; }
  body { font-family: -apple-system, 'Segoe UI', Roboto, sans-serif;
         background: #111318; color: #e7e9ee; padding: 14px; max-width: 640px;
         margin: 0 auto; }
  header { display: flex; gap: 8px; align-items: center; margin-bottom: 12px; }
  header h1 { font-size: 15px; font-weight: 600; color: #9aa1af; flex: 1;
              white-space: nowrap; }
  select, .ghost-btn { background: #1b1e26; color: #e7e9ee; border: 1px solid #2c3140;
              border-radius: 8px; padding: 8px 10px; font-size: 14px; max-width: 46%; }
  #banner { display: none; background: #7f1d1d; color: #fee; border-radius: 8px;
            padding: 8px 12px; margin-bottom: 10px; font-size: 13px; }
  #meal { color: #9aa1af; font-size: 13px; margin-bottom: 12px; }
  .buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  .card { display: flex; flex-direction: column; gap: 8px; }
  .big { border: 2px solid; border-radius: 14px; background: #1b1e26; color: inherit;
         min-height: 120px; font-size: 17px; font-weight: 700; cursor: pointer;
         display: flex; flex-direction: column; align-items: center;
         justify-content: center; gap: 6px; width: 100%; }
  .big .elapsed { font-size: 26px; font-variant-numeric: tabular-nums; }
  .big .hint { font-size: 12px; font-weight: 400; opacity: .75; }
  .big.intervention { border-color: #e5484d; }
  .big.explanation  { border-color: #4f83f1; }
  .big.intervention.active { background: #e5484d; color: #fff; }
  .big.explanation.active  { background: #4f83f1; color: #fff; }
  .card textarea { background: #1b1e26; border: 1px solid #2c3140; border-radius: 8px;
                color: #e7e9ee; padding: 9px 10px; font-size: 14px; width: 100%;
                font-family: inherit; line-height: 1.4; min-height: 84px;
                resize: vertical; }
  .totals { display: flex; flex-wrap: wrap; gap: 8px; margin: 14px 0; }
  .chip { background: #1b1e26; border: 1px solid #2c3140; border-radius: 10px;
          padding: 7px 11px; font-size: 12px; color: #9aa1af; }
  .chip b { display: block; color: #e7e9ee; font-size: 16px; font-weight: 600;
            font-variant-numeric: tabular-nums; }
  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th { text-align: left; color: #9aa1af; font-weight: 500; padding: 6px 6px;
       border-bottom: 1px solid #2c3140; }
  td { padding: 8px 6px; border-bottom: 1px solid #20242e;
       font-variant-numeric: tabular-nums; }
  td.note { color: #9aa1af; word-break: break-word; white-space: pre-wrap; }
  .dot { display: inline-block; width: 9px; height: 9px; border-radius: 50%;
         margin-right: 6px; }
  .dot.intervention { background: #e5484d; } .dot.explanation { background: #4f83f1; }
  .open-tag { color: #f5b04c; font-weight: 600; }
  .del { background: none; border: none; color: #9aa1af; font-size: 16px;
         cursor: pointer; padding: 2px 8px; }
  #warnings { color: #f5b04c; font-size: 12px; margin-top: 10px; }
  #nosession { text-align: center; color: #9aa1af; padding: 40px 10px; display: none; }
</style>
</head>
<body>
<header>
  <h1>⏱ Researcher Timer</h1>
  <select id="session-select"></select>
  <button class="ghost-btn" id="new-session">＋ new</button>
</header>
<div id="banner">Server unreachable — presses are NOT being recorded.</div>
<div id="meal"></div>
<div id="nosession">No session selected — pick one above or create a new one.</div>
<div id="app" style="display:none">
  <div class="buttons">
    <div class="card">
      <button class="big intervention" id="btn-intervention"></button>
      <textarea id="note-intervention" rows="4" placeholder="note (optional, saved on stop)"></textarea>
    </div>
    <div class="card">
      <button class="big explanation" id="btn-explanation"></button>
      <textarea id="note-explanation" rows="4" placeholder="note (optional, saved on stop)"></textarea>
    </div>
  </div>
  <div class="totals" id="totals"></div>
  <table>
    <thead><tr><th>#</th><th>kind</th><th>start</th><th>dur</th><th>note</th><th></th></tr></thead>
    <tbody id="rows"></tbody>
  </table>
  <div id="warnings"></div>
</div>
<script>
'use strict';
const KINDS = ['intervention', 'explanation'];
let st = null;          // last /api/state payload
let skew = 0;           // server_epoch - local epoch, so timers use the robot clock
let busy = false;
let sessionsJson = '';

const $ = id => document.getElementById(id);
const esc = s => s.replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const now = () => Date.now() / 1000 + skew;
const fmt = s => { s = Math.max(0, Math.round(s));
  return Math.floor(s/3600) + ':' + String(Math.floor(s/60)%60).padStart(2,'0')
         + ':' + String(s%60).padStart(2,'0'); };
const clock = e => new Date(e * 1000).toLocaleTimeString('en-GB');

async function api(path, body) {
  const res = await fetch(path, body === undefined ? {} :
    {method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)});
  return res.json();
}

async function poll() {
  try {
    const data = await api('/api/state');
    skew = data.server_epoch - Date.now() / 1000;
    st = data;
    $('banner').style.display = 'none';
    renderSessions();
    render();
  } catch (e) {
    $('banner').style.display = 'block';
  }
}

function renderSessions() {
  const json = JSON.stringify([st.sessions, st.session && st.session.dir]);
  if (json === sessionsJson) return;   // don't rebuild the dropdown under the user
  sessionsJson = json;
  const sel = $('session-select');
  sel.innerHTML = st.sessions.map(s =>
    `<option value="${esc(s.dir)}">${esc(s.label)}</option>`).join('');
  if (st.session) sel.value = st.session.dir;
}

function render() {
  if (!st) return;
  $('nosession').style.display = st.session ? 'none' : 'block';
  $('app').style.display = st.session ? 'block' : 'none';
  if (!st.session) { $('meal').textContent = ''; return; }
  const t = now();

  for (const kind of KINDS) {
    const open = st.intervals.find(iv => iv.kind === kind && iv.end === null);
    const btn = $('btn-' + kind);
    btn.classList.toggle('active', !!open);
    btn.innerHTML = open
      ? `<span>STOP ${kind.toUpperCase()}</span><span class="elapsed">${fmt(t - open.start)}</span>`
      : `<span>${kind.toUpperCase()}</span><span class="hint">tap to start</span>`;
  }

  const sum = f => st.intervals.reduce((a, iv) =>
    a + (f(iv) ? (iv.end === null ? t : iv.end) - iv.start : 0), 0);
  const spans = st.intervals.map(iv => [iv.start, iv.end === null ? t : iv.end])
    .sort((a, b) => a[0] - b[0]);
  let union = 0, hi = -1;
  for (const [s, e] of spans) {
    if (s > hi) { union += e - s; hi = e; }
    else if (e > hi) { union += e - hi; hi = e; }
  }
  const chips = [
    ['intervention Σ', fmt(sum(iv => iv.kind === 'intervention'))],
    ['explanation Σ', fmt(sum(iv => iv.kind === 'explanation'))],
    ['deducted (union)', fmt(union)],
  ];
  if (st.window_start !== null) {
    chips.push(['meal elapsed', fmt(t - st.window_start)]);
    chips.push(['est. feeding time', fmt(t - st.window_start - union)]);
    $('meal').textContent = `Writing to ${st.session.user} / day ${st.session.day}` +
      ` · meal started ${clock(st.window_start)}`;
  } else {
    $('meal').textContent = `Writing to ${st.session.user} / day ${st.session.day}` +
      ` · no run.py data yet (meal clock starts with the system)`;
  }
  $('totals').innerHTML = chips.map(([label, value]) =>
    `<div class="chip">${label}<b>${value}</b></div>`).join('');

  $('rows').innerHTML = st.intervals.slice().reverse().map(iv => `
    <tr><td>${iv.id}</td>
    <td><span class="dot ${iv.kind}"></span>${iv.kind.slice(0, 6)}</td>
    <td>${clock(iv.start)}</td>
    <td>${iv.end === null ? '<span class="open-tag">' + fmt(t - iv.start) + ' …</span>'
                          : fmt(iv.end - iv.start)}</td>
    <td class="note">${esc(iv.note || '')}</td>
    <td><button class="del" onclick="removeInterval(${iv.id})">✕</button></td></tr>`).join('');

  $('warnings').innerHTML = st.warnings.map(w => '⚠ ' + esc(w)).join('<br>');
}

async function press(kind) {
  if (busy || !st || !st.session) return;
  busy = true;
  try {
    const noteInput = $('note-' + kind);
    const result = await api('/api/press', {kind, note: noteInput.value});
    if (result.phase === 'end') noteInput.value = '';
    await poll();
  } catch (e) { $('banner').style.display = 'block'; }
  finally { setTimeout(() => { busy = false; }, 600); }   // swallow double-taps
}

async function removeInterval(id) {
  if (!confirm('Delete interval #' + id + '? (It is tombstoned in the log, not erased.)')) return;
  try { await api('/api/delete', {id}); await poll(); }
  catch (e) { $('banner').style.display = 'block'; }
}

$('btn-intervention').onclick = () => press('intervention');
$('btn-explanation').onclick = () => press('explanation');
$('session-select').onchange = async e => {
  try { await api('/api/session', {dir: e.target.value}); await poll(); }
  catch (err) { $('banner').style.display = 'block'; }
};
$('new-session').onclick = async () => {
  const user = prompt('User name (log/<user>/):'); if (!user) return;
  const day = prompt('Day number:'); if (day === null || day === '') return;
  const result = await api('/api/session', {user: user.trim(), day: Number(day)});
  if (result.error) alert(result.error);
  sessionsJson = '';   // force dropdown rebuild
  await poll();
};

poll();
setInterval(poll, 2500);
setInterval(render, 1000);
</script>
</body>
</html>
"""


def main() -> None:
    global _selected
    # The page polls /api/state every 2.5s; without this, werkzeug writes an
    # access-log line per poll and researcher_timer.log is pure spam. Keep only
    # warnings/errors -- the day's data lives in researcher_events.jsonl anyway.
    logging.getLogger("werkzeug").setLevel(logging.WARNING)
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    _selected = _load_selection()
    if _selected is None:
        sessions = _list_sessions(limit=1)
        _selected = Path(sessions[0]["dir"]) if sessions else None
    print(f"[researcher_timer] {datetime.now().isoformat(timespec='seconds')} "
          f"serving on http://0.0.0.0:{PORT} "
          f"(session: {_selected or 'none selected'})")
    app.run(host="0.0.0.0", port=PORT, threaded=True)


if __name__ == "__main__":
    main()
