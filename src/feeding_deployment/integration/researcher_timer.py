#!/usr/bin/env python3
"""Researcher timer -- standalone web tool on port 8081.

During a study meal the participant's iPad occupies the main webapp (port
8080). This serves a separate single page where the *researcher* (phone or
laptop, http://192.168.1.2:8081) marks the meal boundaries and timestamps
interventions and explanations -- events outside the system that the robot
cannot capture itself. The flow is deliberately linear:

    Start Meal  ->  { Intervention | Explanation | Finish Feeding }
                        Intervention / Explanation -> write a note -> Finish

The offline ``compute_feeding_time.py`` then reports
``feeding time = meal window - union(marked intervals)``, taking the meal
window straight from the Start Meal / Finish Feeding marks recorded here.

Design constraints (why this is not part of run.py or the Vue app):
- Interventions happen exactly when the system is down, so this depends on
  nothing: no roscore, no rosbridge, no run.py. Plain Flask + JSONL files.
- Timestamps are stamped SERVER-side so they share the robot's clock with
  data_logger's events.jsonl/metadata.json regardless of the phone's clock.
- Append-only log (``log/<user>/day_NN/researcher_events.jsonl``): meal and
  interval starts are written the moment they are pressed, so a crash mid-meal
  or mid-interval keeps everything recorded up to that point.
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
        CATEGORY_INTERVAL, CATEGORY_MEAL, INTERVAL_KINDS, LOG_ROOT,
        RESEARCHER_EVENTS_FILENAME, _read_jsonl, load_researcher_intervals,
        load_researcher_meal_marks)
except ImportError:  # run directly as `python researcher_timer.py`
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from compute_feeding_time import (
        CATEGORY_INTERVAL, CATEGORY_MEAL, INTERVAL_KINDS, LOG_ROOT,
        RESEARCHER_EVENTS_FILENAME, _read_jsonl, load_researcher_intervals,
        load_researcher_meal_marks)

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


def _meal_active(day_dir: Path) -> tuple[bool, float | None]:
    """Replay the meal marks -> (is a meal open now, epoch it started).

    A meal is open when the latest researcher_meal mark is a ``start``. Returns
    ``(False, None)`` before the first Start Meal or after a Finish Feeding.
    """
    active, start = False, None
    for phase, epoch in load_researcher_meal_marks(day_dir):
        if phase == "start":
            active, start = True, epoch
        else:  # "end"
            active, start = False, None
    return active, start


# -- API ----------------------------------------------------------------------

@app.get("/api/state")
def api_state():
    with _lock:
        day_dir = _selected
    state = {
        "server_epoch": time.time(),
        "sessions": _list_sessions(),
        "session": None,
        "meal_active": False,
        "meal_start": None,
        "open_interval": None,
    }
    if day_dir is not None:
        active, meal_start = _meal_active(day_dir)
        intervals, _ = load_researcher_intervals(day_dir)
        open_iv = next((iv for iv in intervals if iv["end"] is None), None)
        state.update({
            "session": {"dir": str(day_dir), "user": day_dir.parent.name,
                        "day": int(day_dir.name.split("_")[1])},
            "meal_active": active,
            "meal_start": meal_start,
            "open_interval": ({"id": open_iv["id"], "kind": open_iv["kind"],
                               "start": open_iv["start"]} if open_iv else None),
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


@app.post("/api/meal")
def api_meal():
    """Mark the meal boundary: ``{phase: "start" | "end"}``.

    Idempotent -- starting an already-running meal or ending a stopped one is a
    no-op, so a stale client or a double-tap can never corrupt the window."""
    body = request.get_json(silent=True) or {}
    phase = body.get("phase")
    if phase not in ("start", "end"):
        return jsonify({"error": 'phase must be "start" or "end"'}), 400
    with _lock:
        if _selected is None:
            return jsonify({"error": "no session selected"}), 400
        active, _ = _meal_active(_selected)
        if (phase == "start") == active:  # start while running / end while stopped
            return jsonify({"ok": True, "noop": True})
        _append_record(_selected, {**_stamp(), "category": CATEGORY_MEAL, "phase": phase})
    return jsonify({"ok": True, "phase": phase})


@app.post("/api/press")
def api_press():
    """Toggle an interval: first press opens it (start stamped immediately),
    second press closes it. Opening one requires a running meal; the optional
    note rides on the closing record."""
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
            active, _ = _meal_active(_selected)
            if not active:
                return jsonify({"error": "start the meal first"}), 400
            record.update(phase="start", id=_next_interval_id(_selected))
        else:
            record.update(phase="end", id=open_interval["id"])
        if note:
            record["note"] = note
        _append_record(_selected, record)
    return jsonify({"ok": True, "phase": record["phase"], "id": record["id"]})


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
  :root {
    --bg:#f4f5f7; --surface:#ffffff; --text:#1b2130; --muted:#667085;
    --border:#e3e6ec; --shadow:0 1px 2px rgba(16,24,40,.05), 0 2px 6px rgba(16,24,40,.06);
    --intervention:#d64545; --explanation:#2f6fed; --note:#f2a01f; --note-strong:#c2620c; --start:#12805c; --danger:#b42318;
  }
  @media (prefers-color-scheme: dark) {
    :root {
      --bg:#0f1216; --surface:#171b22; --text:#e7e9ee; --muted:#98a1b0;
      --border:#272c36; --shadow:none;
      --intervention:#e5605f; --explanation:#5b8bf5; --note:#f4b13f; --note-strong:#f4b13f; --start:#1f9d72; --danger:#e5605f;
    }
  }
  * { box-sizing:border-box; margin:0; -webkit-tap-highlight-color:transparent; }
  html, body { height:100%; }
  body { font-family:-apple-system, 'Segoe UI', Roboto, sans-serif; background:var(--bg);
         color:var(--text); -webkit-font-smoothing:antialiased; }
  #app { max-width:460px; margin:0 auto; min-height:100%; padding:20px 18px 28px;
         display:flex; flex-direction:column; }

  #topbar { display:none; align-items:center; justify-content:space-between;
            padding:11px 15px; margin-bottom:20px; background:var(--surface);
            border:1px solid var(--border); border-radius:12px; box-shadow:var(--shadow); }
  #topbar.show { display:flex; }
  #topbar .session { font-size:14px; font-weight:600; }
  #topbar .meal-clock { font-size:15px; font-weight:600; color:var(--muted);
            font-variant-numeric:tabular-nums; letter-spacing:.02em; }

  .screen { display:none; flex-direction:column; gap:14px; }
  .screen.show { display:flex; }
  #screen-setup, #screen-main, #screen-event { flex:1; }
  #screen-setup, #screen-main { justify-content:center; }

  .brand { text-align:center; margin-bottom:6px; }
  .brand h1 { font-size:15px; font-weight:600; color:var(--muted); letter-spacing:.02em; }
  .field label { display:block; font-size:12px; font-weight:600; color:var(--muted);
            text-transform:uppercase; letter-spacing:.06em; margin-bottom:8px; }
  .session-row { display:flex; gap:8px; }
  #session-select { flex:1; background:var(--surface); color:var(--text);
            border:1px solid var(--border); border-radius:12px; padding:14px 12px;
            font-size:16px; box-shadow:var(--shadow); appearance:none; }
  .setup-note { text-align:center; font-size:13px; color:var(--muted); }

  .btn { border:none; border-radius:14px; font-family:inherit; font-size:17px;
         font-weight:600; cursor:pointer; width:100%; padding:18px; color:#fff;
         transition:transform .04s ease, opacity .15s ease; }
  .btn:active { transform:scale(.985); }
  .btn:disabled { opacity:.45; cursor:default; }
  .btn.ghost { flex:0 0 auto; width:auto; background:var(--surface); color:var(--text);
         border:1px solid var(--border); box-shadow:var(--shadow); padding:14px 18px; }
  .btn.start { background:var(--start); box-shadow:var(--shadow); }

  #screen-main { gap:16px; }
  .action { display:flex; align-items:center; justify-content:center; text-align:center;
         color:#fff; box-shadow:var(--shadow); padding:22px 20px; font-size:19px; }
  .action.intervention { background:var(--intervention); }
  .action.explanation { background:var(--explanation); }
  .action.note { background:var(--note); color:#1b2130; }
  .action.finish { margin-top:56px; background:var(--surface); color:var(--muted);
         border:1px solid var(--border); font-size:16px; padding:16px; }

  .event-head { display:flex; align-items:baseline; justify-content:space-between; }
  .event-kind { font-size:21px; font-weight:700; }
  .event-kind.intervention { color:var(--intervention); }
  .event-kind.explanation { color:var(--explanation); }
  .event-kind.note { color:var(--note-strong); }
  .event-clock { font-size:18px; font-weight:600; color:var(--muted);
         font-variant-numeric:tabular-nums; }
  #event-note { width:100%; flex:1; min-height:160px; resize:none; font-family:inherit;
         font-size:16px; line-height:1.5; color:var(--text); background:var(--surface);
         border:1px solid var(--border); border-radius:14px; padding:16px;
         box-shadow:var(--shadow); }
  #event-note:focus { outline:2px solid var(--border); outline-offset:0; }
  .btn.primary.intervention { background:var(--intervention); }
  .btn.primary.explanation { background:var(--explanation); }
  .btn.primary.note { background:var(--note); color:#1b2130; }

  #banner { display:none; background:var(--danger); color:#fff; border-radius:12px;
         padding:12px 14px; margin-top:16px; font-size:14px; text-align:center; }
  #banner.show { display:block; }
</style>
</head>
<body>
<main id="app">
  <div id="topbar">
    <span class="session" id="session-label"></span>
    <span class="meal-clock" id="meal-clock"></span>
  </div>

  <section class="screen show" id="screen-setup">
    <div class="brand"><h1>Researcher Timer</h1></div>
    <div class="field">
      <label for="session-select">Session</label>
      <div class="session-row">
        <select id="session-select"></select>
        <button class="btn ghost" id="new-session">New</button>
      </div>
    </div>
    <button class="btn start" id="btn-start">Start Meal</button>
    <p class="setup-note" id="setup-note"></p>
  </section>

  <section class="screen" id="screen-main">
    <button class="btn action intervention" id="btn-intervention">Intervention</button>
    <button class="btn action explanation" id="btn-explanation">Explanation</button>
    <button class="btn action note" id="btn-note">Note</button>
    <button class="btn action finish" id="btn-finish">Finish Feeding</button>
  </section>

  <section class="screen" id="screen-event">
    <div class="event-head">
      <span class="event-kind" id="event-kind"></span>
      <span class="event-clock" id="event-clock"></span>
    </div>
    <textarea id="event-note"></textarea>
    <button class="btn primary" id="btn-finish-event">Finish</button>
  </section>

  <div id="banner">Server unreachable — actions are NOT being recorded.</div>
</main>
<script>
'use strict';
let st = null;          // last /api/state payload
let skew = 0;           // server_epoch - local epoch, so timers use the robot clock
let busy = false;
let sessionsJson = '';
let eventId = null;     // open interval currently shown -- guards the note textarea

const $ = id => document.getElementById(id);
const esc = s => String(s).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const cap = s => s.charAt(0).toUpperCase() + s.slice(1);
const now = () => Date.now() / 1000 + skew;
const fmt = s => { s = Math.max(0, Math.round(s));
  return Math.floor(s/3600) + ':' + String(Math.floor(s/60)%60).padStart(2,'0')
         + ':' + String(s%60).padStart(2,'0'); };

async function api(path, body) {
  const res = await fetch(path, body === undefined ? {} :
    {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});
  return res.json();
}

async function poll() {
  try {
    const data = await api('/api/state');
    skew = data.server_epoch - Date.now() / 1000;
    st = data;
    $('banner').classList.remove('show');
    renderSessions();
    render();
  } catch (e) {
    $('banner').classList.add('show');
  }
}

function renderSessions() {
  const json = JSON.stringify([st.sessions, st.session && st.session.dir]);
  if (json === sessionsJson) return;   // don't rebuild the dropdown under the user
  sessionsJson = json;
  const sel = $('session-select');
  sel.innerHTML = st.sessions.length
    ? st.sessions.map(s => `<option value="${esc(s.dir)}">${esc(s.label)}</option>`).join('')
    : '<option value="" disabled selected>no sessions yet</option>';
  if (st.session) sel.value = st.session.dir;
}

function screenFor() {
  if (!st || !st.session) return 'setup';
  if (st.open_interval) return 'event';
  if (st.meal_active) return 'main';
  return 'setup';
}

function render() {
  if (!st) return;
  const screen = screenFor();
  for (const name of ['setup', 'main', 'event'])
    $('screen-' + name).classList.toggle('show', name === screen);
  $('topbar').classList.toggle('show', screen !== 'setup');

  $('btn-start').disabled = !st.session;
  $('setup-note').textContent = st.session
    ? `Will record to ${st.session.user} / day ${st.session.day}`
    : 'Pick or create a session to begin.';
  if (st.session)
    $('session-label').textContent = `${st.session.user} / day ${st.session.day}`;

  if (screen === 'event') {
    const iv = st.open_interval;
    if (iv.id !== eventId) {   // a fresh interval -- set up its screen once
      eventId = iv.id;
      const note = $('event-note');
      note.value = '';
      note.placeholder = iv.kind === 'note' ? 'Write your note…' : `Describe the ${iv.kind}…`;
      $('event-kind').textContent = cap(iv.kind);
      $('event-kind').className = 'event-kind ' + iv.kind;
      $('btn-finish-event').textContent = 'Finish ' + cap(iv.kind);
      $('btn-finish-event').className = 'btn primary ' + iv.kind;
      setTimeout(() => note.focus(), 60);
    }
  } else {
    eventId = null;
  }
  tick();
}

function tick() {
  if (!st) return;
  if (st.meal_start != null) $('meal-clock').textContent = fmt(now() - st.meal_start);
  if (st.open_interval) $('event-clock').textContent = fmt(now() - st.open_interval.start);
}

async function act(fn) {
  if (busy) return;
  busy = true;
  try { const r = await fn(); if (r && r.error) alert(r.error); await poll(); }
  catch (e) { $('banner').classList.add('show'); }
  finally { setTimeout(() => { busy = false; }, 500); }   // swallow double-taps
}

$('btn-start').onclick = () => act(() => api('/api/meal', {phase: 'start'}));
$('btn-intervention').onclick = () => act(() => api('/api/press', {kind: 'intervention'}));
$('btn-explanation').onclick = () => act(() => api('/api/press', {kind: 'explanation'}));
$('btn-note').onclick = () => act(() => api('/api/press', {kind: 'note'}));
$('btn-finish-event').onclick = () => {
  if (!st || !st.open_interval) return;
  const kind = st.open_interval.kind, note = $('event-note').value;
  act(() => api('/api/press', {kind, note}));
};
$('btn-finish').onclick = () => {
  if (!confirm('End the meal now? This records the meal end time.')) return;
  act(() => api('/api/meal', {phase: 'end'}));
};
$('session-select').onchange = e => act(() => api('/api/session', {dir: e.target.value}));
$('new-session').onclick = async () => {
  const user = prompt('User name (log/<user>/):'); if (!user) return;
  const day = prompt('Day number:'); if (day === null || day === '') return;
  const result = await api('/api/session', {user: user.trim(), day: Number(day)});
  if (result && result.error) { alert(result.error); return; }
  sessionsJson = '';   // force dropdown rebuild
  await poll();
};

poll();
setInterval(poll, 2000);
setInterval(tick, 1000);
</script>
</body>
</html>
"""


def main() -> None:
    global _selected
    # The page polls /api/state every 2s; without this, werkzeug writes an
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
